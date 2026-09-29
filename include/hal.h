#pragma once

// Hardware Abstraction Layer for the LoRa gateway. Each interface here
// covers exactly one hardware/network concern the orchestrator depends on,
// so GatewayOrchestrator (include/orchestrator.h) can be driven entirely by
// fakes in native unit tests. Concrete ESP32 implementations live in
// hal_esp32.h/.cpp; nothing in this header includes Arduino.h.

#include <functional>
#include <string>
#include <vector>

namespace gateway {

struct RawPacket {
    std::string data;
    int rssi = 0;
};

class IWifiRadio {
public:
    virtual ~IWifiRadio() = default;

    virtual bool connected() = 0;

    // Requests a reconnect attempt. Does not block waiting for the result;
    // connected() reflects success (or continued failure) on a later call.
    virtual void reconnect() = 0;
};

class ILoRaReceiver {
public:
    virtual ~ILoRaReceiver() = default;

    // Non-blocking: returns true and fills `out` if a packet was waiting,
    // false otherwise (mirrors LoRa.parsePacket() == 0).
    virtual bool receive(RawPacket& out) = 0;
};

class IMqttClient {
public:
    virtual ~IMqttClient() = default;

    virtual bool connected() = 0;

    // Attempts a single (re)connect using whatever server/credentials/LWT
    // the concrete adapter was configured with. May take as long as the
    // underlying client's own connect timeout; the orchestrator is
    // responsible for not calling this more often than its backoff allows.
    virtual bool connect() = 0;

    virtual void disconnect() = 0;

    virtual bool publish(const std::string& topic, const std::string& payload, bool retain) = 0;

    using MessageCallback = std::function<void(const std::string& topic, const std::string& payload)>;

    // Subscribes to `topic`; `callback` is invoked (synchronously, from
    // loop() below -- never from interrupt context) whenever a message
    // arrives on it. Must be re-called after every successful connect():
    // MQTT subscriptions are broker-side state and don't survive a
    // disconnect/reconnect, unlike this registration itself, which the
    // concrete adapter keeps around to re-subscribe with on the next
    // connect(). Returns false if the subscribe request itself failed.
    virtual bool subscribe(const std::string& topic, MessageCallback callback) = 0;

    // Services the underlying client (keep-alive ping, incoming messages --
    // this is what actually invokes a subscribe() callback when a message
    // has arrived).
    virtual void loop() = 0;
};

class INodeStore {
public:
    virtual ~INodeStore() = default;

    virtual std::string loadAllowListCsv() = 0;
    virtual void saveAllowListCsv(const std::string& csv) = 0;
};

class IDisplay {
public:
    virtual ~IDisplay() = default;

    // Renders the given lines, replacing whatever was shown before.
    virtual void showLines(const std::vector<std::string>& lines) = 0;
};

class IClock {
public:
    virtual ~IClock() = default;

    virtual unsigned long millis() = 0;
};

} // namespace gateway
