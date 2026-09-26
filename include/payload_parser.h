#pragma once

// Pure, hardware-free logic for the LoRa gateway: sensor payload parsing,
// the node allowlist, routing decisions, and the MQTT/HTML string
// sanitizers that close the topic-injection and stored-XSS gaps in the
// original inline implementation. No Arduino.h, no Arduino String, no
// direct calls into LoRa/PubSubClient/WiFi/Preferences/display — everything
// here is safe to compile and unit test off-target (see [env:native] in
// platformio.ini).

#include <cstdint>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace gateway {

// ---------------------------------------------------------------------
// Sensor error code
// ---------------------------------------------------------------------

enum class SensorError {
    None,
    Dht,
    Unknown, // any non-empty value the parser doesn't recognize yet
};

SensorError parseSensorError(const std::string& raw);
std::string sensorErrorToString(SensorError err);

// ---------------------------------------------------------------------
// Parsed sensor reading
// ---------------------------------------------------------------------

struct SensorReading {
    std::string id; // sanitized: safe to use as an MQTT topic segment
    std::optional<float> temperatureC;
    std::optional<float> humidityPct;
    std::optional<float> batteryVoltage;
    uint32_t bootCount = 0;
    uint32_t seq = 0;
    bool lowBattery = false;
    SensorError err = SensorError::None;
    std::string rawErr = "none"; // preserved verbatim for forward-compat forwarding
};

enum class ParseError {
    None,
    InvalidJson,   // deserializeJson failed (malformed / truncated / empty)
    NotAnObject,   // valid JSON but not a top-level object
    MissingId,     // "id" key absent
    EmptyId,       // "id" present but blank, or blank after sanitization
    WrongType,     // a known field had a JSON type that doesn't fit its contract
};

struct ParseResult {
    ParseError error = ParseError::None;
    SensorReading reading;

    bool ok() const { return error == ParseError::None; }
};

// Strictly validates types: t/h/v must be numeric or JSON null (never a
// string/bool/object); boot/seq must be non-negative integers; lb must be
// 0/1 or a JSON bool; err must be a string. Any mismatch is reported as
// ParseError::WrongType rather than silently coerced.
ParseResult parseSensorPayload(const char* json, size_t length);
ParseResult parseSensorPayload(const std::string& json);

// ---------------------------------------------------------------------
// String sanitizers (close the topic-injection / stored-XSS gaps)
// ---------------------------------------------------------------------

// Restricts a raw, attacker-controlled node id (from an unauthenticated LoRa
// packet) to a safe MQTT topic segment: strips '/', '+', '#', NUL and other
// control characters, trims whitespace, and caps the length. The result is
// safe to splice into an MQTT topic string.
std::string sanitizeMqttTopicSegment(const std::string& raw, size_t maxLen = 40);

// Escapes '&', '<', '>', '"' and '\'' for safe embedding in HTML text/attribute
// context. Used as defense-in-depth wherever a node id is rendered on the
// device-management web page, even though ids are already topic-sanitized.
std::string htmlEscape(const std::string& raw);

// ---------------------------------------------------------------------
// Allowlist (replaces the inline CSV string scanning in main.cpp)
// ---------------------------------------------------------------------

class AllowList {
public:
    AllowList() = default;
    explicit AllowList(const std::string& csv);

    bool isAllowed(const std::string& id) const;

    // Returns true if the allowlist actually changed.
    bool approve(const std::string& id);
    bool remove(const std::string& id);

    std::string toCsv() const;
    const std::vector<std::string>& entries() const { return entries_; }

private:
    std::vector<std::string> entries_;
};

// ---------------------------------------------------------------------
// Routing decision
// ---------------------------------------------------------------------

enum class RouteDecision {
    Pending,  // unknown node: not yet approved, should not be published
    Approved, // known node: safe to publish to `topic`
};

struct RoutingResult {
    RouteDecision decision = RouteDecision::Pending;
    std::string topic; // "<baseTopic>/<sanitized-lowercase-id>", set when Approved
};

// `nodeId` must already be sanitized (SensorReading::id is).
RoutingResult decideRoute(const std::string& nodeId,
                           const std::string& baseTopic,
                           const AllowList& allowList);

// ---------------------------------------------------------------------
// Outbound MQTT message builders
// ---------------------------------------------------------------------
// These build (topic, payload) pairs only; nothing here touches a network
// client. The caller (main.cpp / the Phase 2 orchestrator) is responsible
// for actually publishing.

struct MqttMessage {
    std::string topic;
    std::string payload;
};

struct GatewayIdentity {
    std::string deviceName; // human-readable, e.g. "LoRaGateway"
    std::string baseTopic;  // e.g. "lora/incoming"
};

struct GatewayStats {
    unsigned long uptimeSeconds = 0;
    uint32_t freeHeapBytes = 0;
    int32_t wifiRssi = 0;
    unsigned long packetsReceived = 0;
    std::string ipAddress;
    bool onlyKnownNodes = false;
};

std::string availabilityTopic(const std::string& baseTopic);

// State message for a single sensor reading, published to
// "<baseTopic>/<sanitized-id>" (the RoutingResult::topic from decideRoute).
MqttMessage buildSensorStateMessage(const SensorReading& reading,
                                     int rssi,
                                     const std::string& topic);

// Home Assistant MQTT-discovery configs for one sensor node (temperature,
// humidity, battery voltage, signal, boot count, sequence, low-battery,
// error), mirroring the entities the original sendAutoDiscovery() published.
std::vector<MqttMessage> buildAutoDiscoveryMessages(const std::string& nodeId,
                                                     const GatewayIdentity& gateway);

// Home Assistant MQTT-discovery configs for the gateway's own diagnostic
// sensors (WiFi signal, free heap, packet count).
std::vector<MqttMessage> buildGatewayDiscoveryMessages(const GatewayIdentity& gateway);

// Gateway self-status state message published periodically.
MqttMessage buildGatewayStatusMessage(const GatewayIdentity& gateway, const GatewayStats& stats);

} // namespace gateway
