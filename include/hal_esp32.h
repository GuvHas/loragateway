#pragma once

// Concrete ESP32 implementations of the HAL interfaces (hal.h). These are
// deliberately thin: each method is a near-direct call into the underlying
// Arduino library, with no gateway business logic (that all lives in
// GatewayOrchestrator / payload_parser, which are tested natively). This
// header is excluded from the native test build (see [env:native] in
// platformio.ini) because it pulls in Arduino/LoRa/PubSubClient/Preferences.

#include <LoRa.h>
#include <PubSubClient.h>
#include <Preferences.h>
#include "SSD1306.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>

#include "hal.h"

namespace gateway {

// Wraps the ESP32 Arduino core's WiFi station state/reconnect.
class Esp32WifiRadio : public IWifiRadio {
public:
    bool connected() override;
    void reconnect() override;
};

// Wraps the global `LoRa` singleton (sandeepmistry/LoRa), receiving packets
// via the SX1276's DIO0 interrupt instead of polling LoRa.parsePacket() from
// loop(). Polling only checks for a packet once per loop() iteration, so a
// packet arriving while loop() is stuck in blocking network/socket I/O (e.g.
// PubSubClient::connect() against an unreachable broker) would simply be
// missed -- the SX1276's own FIFO doesn't buffer a second packet on top of
// an undrained one. The interrupt handler drains the FIFO into a small
// fixed-size ring buffer the moment DIO0 fires, independent of what loop()
// is doing; receive() (called from GatewayOrchestrator::tick(), see
// orchestrator.cpp) then just drains that ring buffer.
class Esp32LoRaReceiver : public ILoRaReceiver {
public:
    Esp32LoRaReceiver();

    // Attaches the DIO0 interrupt and puts the radio into continuous-receive
    // mode. Call once from setup(), after LoRa.begin()/setSpreadingFactor()/
    // enableCrc() have already configured the radio -- this only changes the
    // receive strategy, not the radio parameters.
    void begin();

    bool receive(RawPacket& out) override;

    // Packets the ISR had to drop because the ring buffer was still full
    // (receive() wasn't being drained fast enough to keep up with arrivals).
    // Exposed for diagnostics; stays 0 in normal operation since the buffer
    // is drained every orchestrator tick().
    uint32_t droppedByIsr() const { return droppedByIsr_.load(std::memory_order_relaxed); }

private:
    // Sized well above what a single orchestrator tick() could plausibly
    // need to absorb (a LoRa packet's airtime alone is tens of
    // milliseconds, so packets can't physically arrive faster than that) --
    // this is headroom for a slow tick() (e.g. a blocked MQTT call), not a
    // steady-state queue depth.
    static constexpr size_t kRingCapacity = 8;
    static constexpr size_t kMaxPacketBytes = 255; // LoRa's max payload size

    struct IsrPacket {
        uint8_t data[kMaxPacketBytes];
        size_t length = 0;
        int rssi = 0;
    };

    // Invoked directly from the LoRa library's DIO0 interrupt handler (see
    // LoRa.onReceive() in begin()). Must stay fast and allocation-free: it
    // only copies bytes already latched in the SX1276's FIFO into a
    // fixed-size ring buffer slot via LoRa.available()/read()/packetRssi()
    // -- no heap allocation, no logging, no blocking.
    void IRAM_ATTR onDio0Receive(int packetSize);

    // LoRa.onReceive() takes a plain function pointer with no user-data
    // parameter, so it can't bind directly to a non-static member function.
    // There is exactly one Esp32LoRaReceiver instance in this firmware (see
    // main.cpp's global objects), so a single static pointer is sufficient.
    static Esp32LoRaReceiver* instance_;
    static void IRAM_ATTR handleDio0ReceiveTrampoline(int packetSize);

    IsrPacket ring_[kRingCapacity];
    // Single-producer (the ISR)/single-consumer (receive(), called from
    // loop() via GatewayOrchestrator::tick()) index pair: only the ISR ever
    // writes head_, only receive() ever writes tail_, and each side only
    // *reads* the other's index -- so no lock is needed, just the
    // acquire/release ordering used in the .cpp to make sure a slot's data
    // is fully written before its index becomes visible to the other side.
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
    std::atomic<uint32_t> droppedByIsr_{0};
};

// Wraps a PubSubClient. `configure()` must be called (and re-called after
// any config-portal save) before connect() is used, since the client id,
// credentials and LWT topic can all change at runtime.
class Esp32MqttClient : public IMqttClient {
public:
    explicit Esp32MqttClient(PubSubClient& client);

    void configure(const std::string& deviceNamePrefix, const std::string& user,
                   const std::string& pass, const std::string& lwtTopic);

    bool connected() override;
    bool connect() override;
    void disconnect() override;
    bool publish(const std::string& topic, const std::string& payload, bool retain) override;
    bool subscribe(const std::string& topic, MessageCallback callback) override;
    void loop() override;

private:
    // PubSubClient's incoming-message callback only fires from inside
    // loop() (called synchronously from GatewayOrchestrator::tick(), i.e.
    // normal application context, never an interrupt) -- unlike LoRa's
    // onReceive() (see Esp32LoRaReceiver), PubSubClient.h's
    // MQTT_CALLBACK_SIGNATURE is a std::function on ESP32/ESP8266, so it can
    // bind straight to a capturing lambda in the constructor. No static
    // instance pointer/trampoline indirection is needed here.
    void dispatchIncomingMessage(char* topic, uint8_t* payload, unsigned int length);

    PubSubClient& client_;
    std::string deviceNamePrefix_;
    std::string user_;
    std::string pass_;
    std::string lwtTopic_;
    std::map<std::string, MessageCallback> subscriptions_;
};

// Wraps Preferences for the "allow" (allowlist CSV) key.
class Esp32NodeStore : public INodeStore {
public:
    explicit Esp32NodeStore(Preferences& prefs);

    std::string loadAllowListCsv() override;
    void saveAllowListCsv(const std::string& csv) override;

private:
    Preferences& prefs_;
};

// Wraps an SSD1306 display. `onActivity` is invoked on every showLines()
// call so the caller can drive its own screen-timeout/wake bookkeeping
// (kept in main.cpp) without this class needing to know about it.
class Esp32Display : public IDisplay {
public:
    using ActivityCallback = void (*)();

    Esp32Display(SSD1306& display, ActivityCallback onActivity);

    void showLines(const std::vector<std::string>& lines) override;

private:
    SSD1306& display_;
    ActivityCallback onActivity_;
};

// Wraps Arduino's millis().
class Esp32Clock : public IClock {
public:
    unsigned long millis() override;
};

} // namespace gateway
