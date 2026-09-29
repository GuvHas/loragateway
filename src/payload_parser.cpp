#include "payload_parser.h"

#include <ArduinoJson.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

namespace gateway {

namespace {

constexpr size_t kSensorJsonCapacity = 512;
// 700, not 600: the "sw" (firmware version) field on the gateway's own
// discovery entities' nested "dev" object pushed the previous 600-byte
// capacity just over the edge, which ArduinoJson handles by silently
// dropping the assignment rather than erroring -- caught by test_parser's
// discovery tests deserializing and asserting on "sw" specifically. Still
// needed even though per-node discovery no longer carries "sw" (see
// buildEntityDiscovery()): this capacity is shared by both.
constexpr size_t kDiscoveryJsonCapacity = 700;

// Falls back to "dev" only if scripts/inject_git_version.py's build flag
// somehow didn't run (e.g. building outside PlatformIO, or the native test
// env, which deliberately skips it -- see platformio.ini's comment). On the
// real firmware build this is always the actual git short hash.
#ifndef GATEWAY_FW_VERSION
#define GATEWAY_FW_VERSION "dev"
#endif
constexpr size_t kStatusJsonCapacity = 384;

std::string trim(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start]))) start++;
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1]))) end--;
    return s.substr(start, end - start);
}

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

bool equalsIgnoreCase(const std::string& a, const std::string& b) {
    return toLower(a) == toLower(b);
}

// In JSON, every number (integer or floating point literal) is a "float"
// as far as ArduinoJson's type coercion is concerned, so this alone is
// enough to reject strings/bools/objects/arrays passed for a numeric field.
bool isNumericVariant(JsonVariantConst v) {
    return v.is<float>();
}

bool tryReadOptionalFloat(JsonVariantConst v, std::optional<float>& out) {
    if (v.isNull()) {
        out.reset();
        return true;
    }
    if (!isNumericVariant(v)) return false;
    out = v.as<float>();
    return true;
}

bool tryReadNonNegativeInt(JsonVariantConst v, uint32_t& out) {
    if (!isNumericVariant(v)) return false;
    double d = v.as<double>();
    // Reject non-finite, fractional (e.g. 1.5), and out-of-range values
    // instead of silently truncating/UB-casting them: boot/seq are
    // contractually non-negative integers, not "whatever fits after a cast".
    if (!std::isfinite(d)) return false;
    if (d != std::floor(d)) return false;
    if (d < 0 || d > static_cast<double>(std::numeric_limits<uint32_t>::max())) return false;
    out = static_cast<uint32_t>(d);
    return true;
}

} // namespace

// ---------------------------------------------------------------------
// Sensor error code
// ---------------------------------------------------------------------

SensorError parseSensorError(const std::string& raw) {
    std::string lower = toLower(trim(raw));
    if (lower.empty() || lower == "none") return SensorError::None;
    if (lower == "dht") return SensorError::Dht;
    return SensorError::Unknown;
}

std::string sensorErrorToString(SensorError err) {
    switch (err) {
        case SensorError::None: return "none";
        case SensorError::Dht: return "dht";
        case SensorError::Unknown:
        default: return "unknown";
    }
}

// ---------------------------------------------------------------------
// Sanitizers
// ---------------------------------------------------------------------

std::string sanitizeMqttTopicSegment(const std::string& raw, size_t maxLen) {
    std::string trimmed = trim(raw);
    std::string out;
    out.reserve(trimmed.size());
    for (unsigned char c : trimmed) {
        if (c < 0x20 || c == 0x7f) continue;             // control characters
        if (c == '/' || c == '+' || c == '#') continue;  // MQTT topic-structural characters
        if (c == ' ') {
            out += '_';
        } else {
            out += static_cast<char>(c);
        }
        if (out.size() >= maxLen) break;
    }
    return out;
}

std::string htmlEscape(const std::string& raw) {
    std::string out;
    out.reserve(raw.size());
    for (char c : raw) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&#39;"; break;
            default: out += c; break;
        }
    }
    return out;
}

std::string urlEncodeComponent(const std::string& raw) {
    static const char* hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(raw.size());
    for (unsigned char c : raw) {
        bool isUnreserved = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                             (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~';
        if (isUnreserved) {
            out += static_cast<char>(c);
        } else {
            out += '%';
            out += hex[(c >> 4) & 0xF];
            out += hex[c & 0xF];
        }
    }
    return out;
}

uint32_t fnv1a32(const std::string& data) {
    uint32_t hash = 2166136261u;
    for (unsigned char c : data) {
        hash ^= c;
        hash *= 16777619u;
    }
    return hash;
}

std::string toHex8(uint32_t value) {
    static const char* hex = "0123456789abcdef";
    std::string out(8, '0');
    for (int i = 7; i >= 0; --i) {
        out[static_cast<size_t>(i)] = hex[value & 0xF];
        value >>= 4;
    }
    return out;
}

std::string haSafeSlug(const std::string& raw) {
    std::string out = toLower(raw);
    bool changed = false;
    for (char& c : out) {
        bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) {
            c = '_';
            changed = true;
        }
    }
    if (changed) {
        // Substituting every disallowed character with '_' is lossy: "a.b",
        // "a~b" and the already-legal "a_b" would otherwise all collapse to
        // the same slug, so one node's discovery config could silently
        // overwrite another's uniq_id/device-id/topic in Home Assistant.
        // Appending a hash of the *original* bytes makes any id that needed
        // substitution collision-resistant against every other id, while an
        // already-clean id (the overwhelmingly common case) keeps its
        // existing, human-readable, unchanged slug.
        out += '_';
        out += toHex8(fnv1a32(raw));
    }
    return out;
}

// ---------------------------------------------------------------------
// Payload parsing
// ---------------------------------------------------------------------

ParseResult parseSensorPayload(const char* json, size_t length) {
    ParseResult result;

    StaticJsonDocument<kSensorJsonCapacity> doc;
    DeserializationError err = deserializeJson(doc, json, length);
    if (err) {
        result.error = ParseError::InvalidJson;
        return result;
    }

    if (!doc.is<JsonObject>()) {
        result.error = ParseError::NotAnObject;
        return result;
    }
    JsonObject obj = doc.as<JsonObject>();

    if (!obj.containsKey("id")) {
        result.error = ParseError::MissingId;
        return result;
    }
    JsonVariantConst idVar = obj["id"];
    if (!idVar.is<const char*>()) {
        result.error = ParseError::WrongType;
        return result;
    }
    const char* rawIdPtr = idVar.as<const char*>();
    std::string sanitizedId = sanitizeMqttTopicSegment(rawIdPtr ? rawIdPtr : "");
    if (sanitizedId.empty()) {
        result.error = ParseError::EmptyId;
        return result;
    }

    SensorReading reading;
    reading.id = sanitizedId;

    if (obj.containsKey("t") && !tryReadOptionalFloat(obj["t"], reading.temperatureC)) {
        result.error = ParseError::WrongType;
        return result;
    }
    if (obj.containsKey("h") && !tryReadOptionalFloat(obj["h"], reading.humidityPct)) {
        result.error = ParseError::WrongType;
        return result;
    }
    if (obj.containsKey("v") && !tryReadOptionalFloat(obj["v"], reading.batteryVoltage)) {
        result.error = ParseError::WrongType;
        return result;
    }

    if (obj.containsKey("boot") && !tryReadNonNegativeInt(obj["boot"], reading.bootCount)) {
        result.error = ParseError::WrongType;
        return result;
    }
    reading.hasSeq = obj.containsKey("seq");
    if (reading.hasSeq && !tryReadNonNegativeInt(obj["seq"], reading.seq)) {
        result.error = ParseError::WrongType;
        return result;
    }

    if (obj.containsKey("lb")) {
        JsonVariantConst lbVar = obj["lb"];
        if (lbVar.is<bool>()) {
            reading.lowBattery = lbVar.as<bool>();
        } else if (isNumericVariant(lbVar)) {
            // Compare the exact value, not as<int>()'s truncation: 0.5 or 1.9
            // must be rejected, not silently coerced into a valid 0/1.
            double v = lbVar.as<double>();
            if (v != 0.0 && v != 1.0) {
                result.error = ParseError::WrongType;
                return result;
            }
            reading.lowBattery = (v == 1.0);
        } else {
            result.error = ParseError::WrongType;
            return result;
        }
    }

    if (obj.containsKey("err")) {
        JsonVariantConst errVar = obj["err"];
        if (!errVar.is<const char*>()) {
            result.error = ParseError::WrongType;
            return result;
        }
        const char* rawErrPtr = errVar.as<const char*>();
        reading.rawErr = rawErrPtr ? rawErrPtr : "none";
        reading.err = parseSensorError(reading.rawErr);
    }

    result.reading = reading;
    return result;
}

ParseResult parseSensorPayload(const std::string& json) {
    return parseSensorPayload(json.c_str(), json.size());
}

// ---------------------------------------------------------------------
// AllowList
// ---------------------------------------------------------------------

AllowList::AllowList(const std::string& csv) {
    std::string trimmed = trim(csv);
    size_t start = 0;
    while (start <= trimmed.size()) {
        size_t comma = trimmed.find(',', start);
        if (comma == std::string::npos) comma = trimmed.size();
        std::string entry = trim(trimmed.substr(start, comma - start));
        if (!entry.empty()) {
            bool exists = std::any_of(entries_.begin(), entries_.end(),
                                       [&](const std::string& e) { return equalsIgnoreCase(e, entry); });
            if (!exists) entries_.push_back(entry);
        }
        start = comma + 1;
    }
}

bool AllowList::isAllowed(const std::string& id) const {
    return std::any_of(entries_.begin(), entries_.end(),
                        [&](const std::string& e) { return equalsIgnoreCase(e, id); });
}

bool AllowList::approve(const std::string& id) {
    std::string sanitized = sanitizeMqttTopicSegment(id);
    if (sanitized.empty() || isAllowed(sanitized)) return false;
    entries_.push_back(sanitized);
    return true;
}

bool AllowList::remove(const std::string& id) {
    auto it = std::find_if(entries_.begin(), entries_.end(),
                            [&](const std::string& e) { return equalsIgnoreCase(e, id); });
    if (it == entries_.end()) return false;
    entries_.erase(it);
    return true;
}

std::string AllowList::toCsv() const {
    std::string out;
    for (const auto& e : entries_) {
        if (!out.empty()) out += ",";
        out += e;
    }
    return out;
}

// ---------------------------------------------------------------------
// Routing
// ---------------------------------------------------------------------

RoutingResult decideRoute(const std::string& nodeId,
                           const std::string& baseTopic,
                           const AllowList& allowList) {
    RoutingResult result;
    if (!allowList.isAllowed(nodeId)) {
        result.decision = RouteDecision::Pending;
        return result;
    }
    result.decision = RouteDecision::Approved;
    result.topic = baseTopic + "/" + toLower(nodeId);
    return result;
}

// ---------------------------------------------------------------------
// Outbound MQTT message builders
// ---------------------------------------------------------------------

std::string availabilityTopic(const std::string& baseTopic) {
    return baseTopic + "/gateway/status";
}

std::string gatewayCommandTopic(const std::string& baseTopic, const std::string& commandName) {
    return baseTopic + "/gateway/command/" + commandName;
}

MqttMessage buildSensorStateMessage(const SensorReading& reading, int rssi, const std::string& topic) {
    StaticJsonDocument<kSensorJsonCapacity> doc;
    doc["id"] = reading.id;
    if (reading.temperatureC.has_value()) doc["t"] = *reading.temperatureC; else doc["t"] = nullptr;
    if (reading.humidityPct.has_value()) doc["h"] = *reading.humidityPct; else doc["h"] = nullptr;
    if (reading.batteryVoltage.has_value()) doc["v"] = *reading.batteryVoltage; else doc["v"] = nullptr;
    doc["boot"] = reading.bootCount;
    doc["seq"] = reading.seq;
    doc["lb"] = reading.lowBattery ? 1 : 0;
    doc["err"] = reading.rawErr;
    doc["rssi"] = rssi;

    MqttMessage msg;
    msg.topic = topic;
    serializeJson(doc, msg.payload);
    return msg;
}

namespace {

MqttMessage buildEntityDiscovery(const std::string& component,
                                  const std::string& nodeId,
                                  const std::string& suffix,
                                  const std::string& nameSuffix,
                                  const std::string& valTpl,
                                  const std::string& unit,
                                  const std::string& devClass,
                                  const GatewayIdentity& gateway,
                                  const std::string& entCat = "",
                                  int precision = -1) {
    // stateTopic must exactly match the topic decideRoute() actually
    // publishes readings to (which only guarantees MQTT-safety, not HA's
    // stricter discovery-topic charset) -- so it stays on the plain
    // lowercase id. haId is a *separate*, HA-discovery-safe slug used only
    // for this message's own topic/uniq_id/device-id, since those must
    // satisfy HA's [a-z0-9_-] requirement regardless of what the state
    // topic looks like.
    std::string safeId = toLower(nodeId);
    std::string haId = haSafeSlug(nodeId);
    std::string stateTopic = gateway.baseTopic + "/" + safeId;
    std::string availTopic = availabilityTopic(gateway.baseTopic);

    StaticJsonDocument<kDiscoveryJsonCapacity> doc;
    doc["name"] = nodeId + " " + nameSuffix;
    doc["stat_t"] = stateTopic;
    doc["val_tpl"] = valTpl;
    if (!unit.empty()) doc["unit_of_meas"] = unit;
    if (!devClass.empty()) doc["dev_cla"] = devClass;
    doc["uniq_id"] = "lora_" + haId + "_" + suffix;
    doc["avty_t"] = availTopic;
    if (!entCat.empty()) doc["ent_cat"] = entCat;
    if (precision >= 0) doc["sugg_dsp_prec"] = precision;

    JsonObject dev = doc.createNestedObject("dev");
    dev["ids"] = "lora_" + haId;
    dev["name"] = nodeId;
    dev["mdl"] = "LoRa Sensor Node";
    dev["mf"] = "DIY";
    dev["via_device"] = gateway.deviceName;
    // Deliberately no "sw" here: a sensor node's own firmware version isn't
    // part of the payload contract (nodes have no OTA update path -- they're
    // battery-powered and physically remote -- so it can't be added without
    // a field the node would need to actually send), and stamping the
    // gateway's own version here instead was actively misleading: two nodes
    // running genuinely different firmware would both show the gateway's
    // version, looking like they were running the same thing.

    MqttMessage msg;
    msg.topic = "homeassistant/" + component + "/lora_" + haId + "_" + suffix + "/config";
    serializeJson(doc, msg.payload);
    return msg;
}

} // namespace

std::vector<MqttMessage> buildAutoDiscoveryMessages(const std::string& nodeId,
                                                     const GatewayIdentity& gateway) {
    std::vector<MqttMessage> messages;
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "t", "Temperature",
                                             "{{ value_json.t }}", "°C", "temperature", gateway));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "h", "Humidity",
                                             "{{ value_json.h }}", "%", "humidity", gateway));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "v", "Battery",
                                             "{{ value_json.v }}", "V", "voltage", gateway, "", 2));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "r", "Signal",
                                             "{{ value_json.rssi }}", "dBm", "signal_strength", gateway));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "boot", "Boot Count",
                                             "{{ value_json.boot | default(0) }}", "restarts", "",
                                             gateway, "diagnostic"));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "seq", "Sequence",
                                             "{{ value_json.seq | default(0) }}", "", "",
                                             gateway, "diagnostic"));
    messages.push_back(buildEntityDiscovery(
        "binary_sensor", nodeId, "lb", "Low Battery",
        "{{ 'ON' if value_json.lb is defined and value_json.lb == 1 else 'OFF' }}", "", "battery", gateway));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "err", "Error",
                                             "{{ value_json.err | default('none', true) }}", "", "",
                                             gateway, "diagnostic"));
    return messages;
}

std::vector<MqttMessage> buildGatewayDiscoveryMessages(const GatewayIdentity& gateway) {
    // gwId is only ever used for this discovery topic/uniq_id/device-id, not
    // for a real published topic, so it can go straight to the HA-safe slug
    // (deviceName is user-editable via the config portal and could contain
    // characters HA's discovery topic doesn't allow).
    std::string gwId = haSafeSlug(gateway.deviceName);
    std::string stateTopic = gateway.baseTopic + "/gateway/state";
    std::string availTopic = availabilityTopic(gateway.baseTopic);

    auto buildGwSensor = [&](const std::string& suffix, const std::string& nameSuffix,
                              const std::string& valTpl, const std::string& unit,
                              const std::string& devClass) {
        StaticJsonDocument<kDiscoveryJsonCapacity> doc;
        doc["name"] = gateway.deviceName + " " + nameSuffix;
        doc["stat_t"] = stateTopic;
        doc["val_tpl"] = valTpl;
        doc["unit_of_meas"] = unit;
        if (!devClass.empty()) doc["dev_cla"] = devClass;
        doc["uniq_id"] = gwId + "_" + suffix;
        doc["avty_t"] = availTopic;
        doc["ent_cat"] = "diagnostic";

        JsonObject dev = doc.createNestedObject("dev");
        dev["ids"] = gwId;
        dev["name"] = gateway.deviceName;
        dev["mdl"] = "ESP32 LoRa Gateway";
        dev["mf"] = "DIY";
        dev["sw"] = GATEWAY_FW_VERSION;

        MqttMessage msg;
        msg.topic = "homeassistant/sensor/" + gwId + "_" + suffix + "/config";
        serializeJson(doc, msg.payload);
        return msg;
    };

    std::vector<MqttMessage> messages;
    messages.push_back(buildGwSensor("wifi", "WiFi Signal", "{{ value_json.wifi_rssi }}", "dBm", "signal_strength"));
    messages.push_back(buildGwSensor("heap", "Free Memory", "{{ value_json.free_heap }}", "B", ""));
    messages.push_back(buildGwSensor("pkts", "Packets Received", "{{ value_json.packets_rx }}", "pkts", ""));
    messages.push_back(buildGwSensor("queue", "Queue Depth", "{{ value_json.queue_depth }}", "msgs", ""));
    messages.push_back(
        buildGwSensor("dropped", "Packets Dropped", "{{ value_json.packets_dropped }}", "msgs", ""));
    // uptime_s is already published in every gateway status message (see
    // buildGatewayStatusMessage() below) but had no discovery entity, so it
    // was never visible in Home Assistant despite being on the wire.
    messages.push_back(buildGwSensor("uptime", "Uptime", "{{ value_json.uptime_s }}", "s", "duration"));
    return messages;
}

std::vector<MqttMessage> buildGatewayCommandDiscoveryMessages(const GatewayIdentity& gateway) {
    std::string gwId = haSafeSlug(gateway.deviceName);
    std::string availTopic = availabilityTopic(gateway.baseTopic);

    auto buildGwButton = [&](const std::string& suffix, const std::string& nameSuffix,
                              const std::string& cmdTopic, const std::string& devClass) {
        StaticJsonDocument<kDiscoveryJsonCapacity> doc;
        doc["name"] = gateway.deviceName + " " + nameSuffix;
        doc["cmd_t"] = cmdTopic;
        doc["payload_press"] = kRestartCommandPayload;
        if (!devClass.empty()) doc["dev_cla"] = devClass;
        doc["uniq_id"] = gwId + "_" + suffix;
        doc["avty_t"] = availTopic;
        // Restart (like any command that changes device behavior rather than
        // reporting it) is a "config" entity in HA's convention, distinct
        // from the "diagnostic" category used for the read-only sensors in
        // buildGatewayDiscoveryMessages().
        doc["ent_cat"] = "config";

        JsonObject dev = doc.createNestedObject("dev");
        dev["ids"] = gwId;
        dev["name"] = gateway.deviceName;
        dev["mdl"] = "ESP32 LoRa Gateway";
        dev["mf"] = "DIY";
        dev["sw"] = GATEWAY_FW_VERSION;

        MqttMessage msg;
        msg.topic = "homeassistant/button/" + gwId + "_" + suffix + "/config";
        serializeJson(doc, msg.payload);
        return msg;
    };

    std::vector<MqttMessage> messages;
    messages.push_back(buildGwButton("restart", "Restart", gatewayCommandTopic(gateway.baseTopic, "restart"),
                                      "restart"));
    return messages;
}

MqttMessage buildGatewayStatusMessage(const GatewayIdentity& gateway, const GatewayStats& stats) {
    StaticJsonDocument<kStatusJsonCapacity> doc;
    doc["uptime_s"] = stats.uptimeSeconds;
    doc["free_heap"] = stats.freeHeapBytes;
    doc["wifi_rssi"] = stats.wifiRssi;
    doc["packets_rx"] = stats.packetsReceived;
    doc["ip"] = stats.ipAddress;
    doc["enablecrc"] = true;
    doc["invertiq"] = false;
    doc["onlyknown"] = stats.onlyKnownNodes;
    doc["queue_depth"] = stats.queueDepth;
    doc["packets_dropped"] = stats.packetsDropped;

    MqttMessage msg;
    msg.topic = gateway.baseTopic + "/gateway/state";
    serializeJson(doc, msg.payload);
    return msg;
}

} // namespace gateway
