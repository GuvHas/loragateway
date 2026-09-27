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

bool Esp32LoRaReceiver::receive(RawPacket& out) {
    int packetSize = LoRa.parsePacket();
    if (!packetSize) return false;

    std::string data;
    data.reserve(packetSize);
    while (LoRa.available()) {
        data += static_cast<char>(LoRa.read());
    }

    out.data = std::move(data);
    out.rssi = LoRa.packetRssi();
    return true;
}

// ---------------------------------------------------------------------
// Esp32MqttClient
// ---------------------------------------------------------------------

Esp32MqttClient::Esp32MqttClient(PubSubClient& client) : client_(client) {}

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
