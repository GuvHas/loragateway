#include "hal_esp32.h"

#include <Arduino.h>
#include <WiFi.h>

namespace gateway {

// ---------------------------------------------------------------------
// Esp32WifiRadio
// ---------------------------------------------------------------------

bool Esp32WifiRadio::connected() {
    return WiFi.status() == WL_CONNECTED;
}

void Esp32WifiRadio::reconnect() {
    WiFi.reconnect();
}

// ---------------------------------------------------------------------
// Esp32LoRaReceiver
// ---------------------------------------------------------------------

Esp32LoRaReceiver* Esp32LoRaReceiver::instance_ = nullptr;

Esp32LoRaReceiver::Esp32LoRaReceiver() {
    instance_ = this;
}

void Esp32LoRaReceiver::begin() {
    LoRa.onReceive(&Esp32LoRaReceiver::handleDio0ReceiveTrampoline);
    LoRa.receive(); // continuous-receive mode: DIO0 now fires on every RX done
}

void Esp32LoRaReceiver::handleDio0ReceiveTrampoline(int packetSize) {
    if (instance_) instance_->onDio0Receive(packetSize);
}

void Esp32LoRaReceiver::onDio0Receive(int packetSize) {
    if (packetSize <= 0) return;

    size_t head = head_.load(std::memory_order_relaxed);
    size_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= kRingCapacity) {
        // Consumer (receive(), drained every orchestrator tick()) isn't
        // keeping up; drop rather than block in an ISR or overrun the ring.
        droppedByIsr_.fetch_add(1, std::memory_order_relaxed);
        return;
    }

    IsrPacket& slot = ring_[head % kRingCapacity];
    slot.length = 0;
    while (LoRa.available() && slot.length < kMaxPacketBytes) {
        slot.data[slot.length++] = static_cast<uint8_t>(LoRa.read());
    }
    slot.rssi = LoRa.packetRssi();

    // Publish the slot to the consumer only after it's fully written.
    head_.store(head + 1, std::memory_order_release);
}

bool Esp32LoRaReceiver::receive(RawPacket& out) {
    size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {
        return false; // ring buffer empty
    }

    const IsrPacket& slot = ring_[tail % kRingCapacity];
    out.data.assign(reinterpret_cast<const char*>(slot.data), slot.length);
    out.rssi = slot.rssi;

    tail_.store(tail + 1, std::memory_order_release);
    return true;
}

// ---------------------------------------------------------------------
// Esp32MqttClient
// ---------------------------------------------------------------------

Esp32MqttClient::Esp32MqttClient(PubSubClient& client) : client_(client) {
    // PubSubClient.h picks MQTT_CALLBACK_SIGNATURE as a std::function on
    // ESP32 (vs. a bare function pointer on platforms without <functional>),
    // so this capturing lambda can bind directly -- no static instance
    // pointer/trampoline needed here, unlike LoRa.onReceive() in
    // Esp32LoRaReceiver, which only accepts a plain function pointer with no
    // user-data slot.
    client_.setCallback([this](char* topic, uint8_t* payload, unsigned int length) {
        dispatchIncomingMessage(topic, payload, length);
    });
}

void Esp32MqttClient::configure(const std::string& deviceNamePrefix, const std::string& user,
                                 const std::string& pass, const std::string& lwtTopic) {
    deviceNamePrefix_ = deviceNamePrefix;
    user_ = user;
    pass_ = pass;
    lwtTopic_ = lwtTopic;
}

bool Esp32MqttClient::connected() {
    return client_.connected();
}

bool Esp32MqttClient::connect() {
    String clientId = String(deviceNamePrefix_.c_str()) + "-" + String(random(0xffff), HEX);
    bool ok = client_.connect(clientId.c_str(), user_.c_str(), pass_.c_str(), lwtTopic_.c_str(), 1,
                               true, "offline");
    if (ok) {
        client_.publish(lwtTopic_.c_str(), "online", true);
    }
    return ok;
}

void Esp32MqttClient::disconnect() {
    client_.disconnect();
}

bool Esp32MqttClient::publish(const std::string& topic, const std::string& payload, bool retain) {
    return client_.publish(topic.c_str(), payload.c_str(), retain);
}

bool Esp32MqttClient::subscribe(const std::string& topic, MessageCallback callback) {
    subscriptions_[topic] = std::move(callback);
    return client_.subscribe(topic.c_str());
}

void Esp32MqttClient::dispatchIncomingMessage(char* topic, uint8_t* payload, unsigned int length) {
    auto it = subscriptions_.find(topic);
    if (it == subscriptions_.end()) return;
    it->second(topic, std::string(reinterpret_cast<const char*>(payload), length));
}

void Esp32MqttClient::loop() {
    client_.loop();
}

// ---------------------------------------------------------------------
// Esp32NodeStore
// ---------------------------------------------------------------------

Esp32NodeStore::Esp32NodeStore(Preferences& prefs) : prefs_(prefs) {}

std::string Esp32NodeStore::loadAllowListCsv() {
    return std::string(prefs_.getString("allow", "").c_str());
}

void Esp32NodeStore::saveAllowListCsv(const std::string& csv) {
    prefs_.putString("allow", csv.c_str());
}

std::string Esp32NodeStore::loadNodeVersionsCsv() {
    return std::string(prefs_.getString("swver", "").c_str());
}

void Esp32NodeStore::saveNodeVersionsCsv(const std::string& csv) {
    prefs_.putString("swver", csv.c_str());
}

// ---------------------------------------------------------------------
// Esp32Display
// ---------------------------------------------------------------------

namespace {

constexpr int kLineHeightPx = 13;    // ArialMT_Plain_10's own line height
constexpr int kFooterTopPx = 52;     // where the footer starts; content must stay above this
constexpr int kScreenWidthPx = 128;

// Truncates `text` (appending "...") until it fits on one physical row, so a
// showLines() entry can never wrap into more rows than the caller accounted
// for and can never collide with the footer drawn below it.
String fitToOneLine(SSD1306& display, const std::string& text) {
    String full(text.c_str());
    if (display.getStringWidth(full) <= kScreenWidthPx) return full;

    std::string truncated = text;
    while (!truncated.empty() &&
           display.getStringWidth(String((truncated + "...").c_str())) > kScreenWidthPx) {
        truncated.pop_back();
    }
    return String((truncated + "...").c_str());
}

} // namespace

Esp32Display::Esp32Display(SSD1306& display, ActivityCallback onActivity)
    : display_(display), onActivity_(onActivity) {}

void Esp32Display::showLines(const std::vector<std::string>& lines) {
    if (onActivity_) onActivity_();

    display_.clear();
    display_.setFont(ArialMT_Plain_10);

    // Each entry is drawn as exactly one physical row (truncated if it
    // doesn't fit), so `y` always advances by a known amount and the loop
    // stops before it would ever draw into the footer's territory below.
    int y = 0;
    for (const auto& line : lines) {
        if (y >= kFooterTopPx) break;
        display_.drawString(0, y, fitToOneLine(display_, line));
        y += kLineHeightPx;
    }

    // Footer: IP address + a coarse signal-strength indicator, drawn on
    // every update (mirrors the gateway's previous drawFooter() helper).
    display_.drawLine(0, 52, 128, 52);
    display_.setFont(ArialMT_Plain_10);
    display_.drawString(0, 54, WiFi.localIP().toString());
    long rssi = WiFi.RSSI();
    int bars = (rssi > -55) ? 4 : (rssi > -65) ? 3 : (rssi > -75) ? 2 : 1;
    if (rssi == 0) bars = 0;
    String signalStr = String(bars) + "/4";
    int strWidth = display_.getStringWidth(signalStr);
    display_.drawString(128 - strWidth, 54, signalStr);

    display_.display();
}

// ---------------------------------------------------------------------
// Esp32Clock
// ---------------------------------------------------------------------

unsigned long Esp32Clock::millis() {
    return ::millis();
}

} // namespace gateway
