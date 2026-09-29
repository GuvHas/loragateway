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
    // False when the payload omitted "seq" entirely (bootCount/seq then just
    // default to 0, indistinguishable from a real seq-0 reading). Consumers
    // that key behavior off seq -- e.g. GatewayOrchestrator's retransmission
    // dedup -- must check this first, or a node that never sends "seq" would
    // look identical on every packet and get treated as an endless duplicate
    // after its first reading.
    bool hasSeq = false;
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

// Percent-encodes a string for safe use as a URL query-parameter value
// (RFC 3986 unreserved characters pass through unescaped; everything else
// becomes %XX). Needed in addition to htmlEscape() wherever a node id is
// placed inside an href's query string: HTML-entity-encoding alone (e.g.
// "&" -> "&amp;") is decoded back to "&" by the browser before the URL is
// parsed, so a raw "&" in a node id would still split the query string and
// let /approve or /remove act on the wrong id.
std::string urlEncodeComponent(const std::string& raw);

// Restricts a string to the character set Home Assistant's MQTT discovery
// topic requires for its node_id/object_id segments: [a-z0-9_-] (lowercased
// first). This is *stricter* than sanitizeMqttTopicSegment(), which only
// guards against characters MQTT itself disallows ('/', '+', '#') — a
// perfectly valid MQTT topic character like '~' or '.' is still illegal in
// an HA discovery topic and gets silently rejected by HA (logged, not
// errored back to the gateway) if it isn't also stripped here. Disallowed
// characters are replaced with '_'; if any substitution happened, an 8-hex-
// character hash of the original bytes is appended so that two different
// ids which would otherwise collapse to the same slug (e.g. "a.b" and
// "a~b", or a malformed id colliding with an already-legal "a_b") stay
// distinguishable instead of one silently overwriting the other's discovery
// config/uniq_id/device-id in Home Assistant. An already-clean id is
// returned unchanged (no hash suffix), so existing HA entities for
// well-formed node names are unaffected. Used only for the discovery topic/
// uniq_id/device id — NOT for the sensor's actual state topic, which has no
// such restriction and must keep matching decideRoute()'s topic exactly.
std::string haSafeSlug(const std::string& raw);

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
    // Store-and-forward queue health (GatewayOrchestrator, Phase 2): how many
    // messages are currently held because MQTT was unreachable, and how many
    // have been permanently dropped because the queue filled up while still
    // offline (see GatewayOrchestrator::kMaxQueuedMessages).
    uint32_t queueDepth = 0;
    unsigned long packetsDropped = 0;
};

// Gateway firmware version, embedded as "sw" in every discovery message's
// device block -- both the gateway's own HA device page and every per-node
// device page (registered by an instance of this firmware), so a firmware
// upgrade that changes the discovery schema is traceable from HA. A sensor
// node's own firmware version isn't part of the payload contract
// (SensorReading carries none), so a node's "sw" reflects the gateway that
// discovered it, not the sensor node's own firmware -- documented at the
// call site in buildEntityDiscovery().
constexpr const char* kFirmwareVersion = "1.1.0";

// Payload the gateway's restart button (buildGatewayCommandDiscoveryMessages)
// publishes on press, and the only payload GatewayOrchestrator's restart
// command handler acts on -- kept as one shared constant so the discovery
// message and the handler can't silently drift apart.
constexpr const char* kRestartCommandPayload = "PRESS";

std::string availabilityTopic(const std::string& baseTopic);

// Command topic for a named gateway command, e.g.
// gatewayCommandTopic("lora/incoming", "restart") ->
// "lora/incoming/gateway/command/restart". Mirrors the existing
// "<baseTopic>/gateway/state" and availabilityTopic()'s "<baseTopic>/gateway/
// status" conventions. Intended for more than just restart: any future
// gateway-directed command (and eventually per-node LoRa downlinks) can use
// the same "<baseTopic>/gateway/command/<name>" shape.
std::string gatewayCommandTopic(const std::string& baseTopic, const std::string& commandName);

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
// sensors (WiFi signal, free heap, packet count, store-and-forward queue
// depth, dropped-message count, uptime).
std::vector<MqttMessage> buildGatewayDiscoveryMessages(const GatewayIdentity& gateway);

// Home Assistant MQTT-discovery configs for the gateway's own command
// entities (currently just a "Restart" button, device_class: restart).
// Separate from buildGatewayDiscoveryMessages() because these are `button`
// (command-only, no state) entities rather than `sensor` ones -- a different
// HA discovery component, hence a different discovery topic shape.
std::vector<MqttMessage> buildGatewayCommandDiscoveryMessages(const GatewayIdentity& gateway);

// Gateway self-status state message published periodically.
MqttMessage buildGatewayStatusMessage(const GatewayIdentity& gateway, const GatewayStats& stats);

} // namespace gateway
