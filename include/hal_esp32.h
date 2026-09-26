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

#include "hal.h"

namespace gateway {

// Wraps the global `LoRa` singleton (sandeepmistry/LoRa).
class Esp32LoRaReceiver : public ILoRaReceiver {
public:
    bool receive(RawPacket& out) override;
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
    void loop() override;

private:
    PubSubClient& client_;
    std::string deviceNamePrefix_;
    std::string user_;
    std::string pass_;
    std::string lwtTopic_;
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
