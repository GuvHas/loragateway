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
// 832, not 700: a friendly node name (up to kMaxNodeNameLength bytes, in
// both the entity name and the device name) pushed 700 over the edge, and
// node discovery carries "sw" again. test_parser's
// test_build_auto_discovery_messages_fit_longest_display_name guards it.
constexpr size_t kDiscoveryJsonCapacity = 832;

// Falls back to "dev" only if scripts/inject_git_version.py's build flag
// somehow didn't run (e.g. building outside PlatformIO, or the native test
// env, which deliberately skips it -- see platformio.ini's comment). On the
// real firmware build this is always the actual git short hash.
#ifndef GATEWAY_FW_VERSION
#define GATEWAY_FW_VERSION "dev"
#endif
// 13 members. Sized for the native (64-bit) test build, where each slot is
// twice the ESP32's size; ArduinoJson silently drops members that don't fit,
// which test_build_gateway_status_message catches.
constexpr size_t kStatusJsonCapacity = 640;

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

std::string sanitizeNodeName(const std::string& raw) {
    std::string kept;
    for (char c : raw) {
        unsigned char u = static_cast<unsigned char>(c);
        if (u < 0x20 || u == 0x7f || c == ',' || c == '=') continue;
        kept += c;
    }
    size_t first = kept.find_first_not_of(' ');
    if (first == std::string::npos) return "";
    size_t last = kept.find_last_not_of(' ');
    kept = kept.substr(first, last - first + 1);
    if (kept.size() > kMaxNodeNameLength) {
        size_t cut = kMaxNodeNameLength;
        // Back up to the start of the UTF-8 character the cut lands in.
        while (cut > 0 && (static_cast<unsigned char>(kept[cut]) & 0xC0) == 0x80) cut--;
        kept = kept.substr(0, cut);
        size_t end = kept.find_last_not_of(' ');
        kept = end == std::string::npos ? "" : kept.substr(0, end + 1);
    }
    return kept;
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

    // Optional: only a node's cold-boot packet is expected to carry this
    // (see SensorReading::swVersion), but nothing here enforces that -- the
    // gateway trusts whatever bootCount/seq accompany it, same as every
    // other field on this unauthenticated link.
    if (obj.containsKey("sw")) {
        JsonVariantConst swVar = obj["sw"];
        if (!swVar.is<const char*>()) {
            result.error = ParseError::WrongType;
            return result;
        }
        const char* rawSwPtr = swVar.as<const char*>();
        reading.swVersion = rawSwPtr ? std::string(rawSwPtr) : std::string();
    }

    result.reading = reading;
    return result;
}

ParseResult parseSensorPayload(const std::string& json) {
    return parseSensorPayload(json.c_str(), json.size());
}

// ---------------------------------------------------------------------
// Binary frame v1
// ---------------------------------------------------------------------

namespace {

constexpr uint8_t kFlagLowBattery = 0x01;
constexpr uint8_t kFlagSensorError = 0x02;
constexpr uint8_t kFlagSwPresent = 0x04;
constexpr int16_t kNullTemperature = INT16_MIN;   // 0x8000
constexpr uint16_t kNullHumidity = 0xFFFF;
// Bytes after the id: boot (3), seq (3), t (2), h (2), v (1).
constexpr size_t kFixedFieldBytes = 11;

uint32_t readU24(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16);
}

uint16_t readU16(const uint8_t* p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

}  // namespace

ParseResult parseBinarySensorFrame(const uint8_t* data, size_t length) {
    ParseResult result;
    if (length < 3 || data[0] != kBinaryFrameV1Magic) {
        result.error = ParseError::BadFrame;
        return result;
    }
    const uint8_t flags = data[1];
    const size_t idLen = data[2];
    const size_t fixedEnd = 3 + idLen + kFixedFieldBytes;
    const bool hasSw = (flags & kFlagSwPresent) != 0;

    // Exact length check before reading anything past the header.
    size_t swLen = 0;
    if (hasSw) {
        if (length < fixedEnd + 1) {
            result.error = ParseError::BadFrame;
            return result;
        }
        swLen = data[fixedEnd];
        if (swLen == 0 || length != fixedEnd + 1 + swLen) {
            result.error = ParseError::BadFrame;
            return result;
        }
    } else if (length != fixedEnd) {
        result.error = ParseError::BadFrame;
        return result;
    }

    std::string sanitizedId = sanitizeMqttTopicSegment(
        std::string(reinterpret_cast<const char*>(data + 3), idLen));
    if (sanitizedId.empty()) {
        result.error = ParseError::EmptyId;
        return result;
    }

    const uint8_t* p = data + 3 + idLen;
    SensorReading reading;
    reading.id = sanitizedId;
    reading.bootCount = readU24(p);
    reading.seq = readU24(p + 3);
    reading.hasSeq = true;

    int16_t t = static_cast<int16_t>(readU16(p + 6));
    if (t != kNullTemperature) reading.temperatureC = static_cast<float>(t) / 10.0f;
    uint16_t h = readU16(p + 8);
    if (h != kNullHumidity) reading.humidityPct = static_cast<float>(h) / 10.0f;
    // 10 mV steps from 2.00 V. Divided as millivolts so the float is the
    // nearest one to the decimal value, matching what the JSON parser gets
    // from "3.90" and keeping the MQTT state message byte-identical.
    reading.batteryVoltage = static_cast<float>(2000 + 10 * p[10]) / 1000.0f;

    reading.lowBattery = (flags & kFlagLowBattery) != 0;
    reading.rawErr = (flags & kFlagSensorError) ? "dht" : "none";
    reading.err = parseSensorError(reading.rawErr);

    if (hasSw) {
        reading.swVersion = std::string(reinterpret_cast<const char*>(data + fixedEnd + 1), swLen);
    }

    result.reading = reading;
    return result;
}

ParseResult parseSensorPacket(const std::string& raw) {
    if (!raw.empty() && static_cast<uint8_t>(raw[0]) == kBinaryFrameV1Magic) {
        return parseBinarySensorFrame(reinterpret_cast<const uint8_t*>(raw.data()), raw.size());
    }
    return parseSensorPayload(raw);
}

std::string unparsedPacketPayload(const std::string& raw) {
    if (raw.empty() || static_cast<uint8_t>(raw[0]) != kBinaryFrameV1Magic) return raw;
    static const char* kHex = "0123456789abcdef";
    std::string out;
    out.reserve(raw.size() * 2);
    for (unsigned char c : raw) {
        out.push_back(kHex[c >> 4]);
        out.push_back(kHex[c & 0x0F]);
    }
    return out;
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

void PacketLossWindow::reset() {
    slots_.clear();
    hasLast_ = false;
    lastSeq_ = 0;
}

void PacketLossWindow::received(uint32_t seq) {
    if (hasLast_ && seq > lastSeq_) {
        // Cap the gap: anything past the window would be trimmed anyway, and
        // a huge jump must not build a huge queue.
        uint32_t gap = seq - lastSeq_ - 1;
        if (gap > kWindow) gap = kWindow;
        for (uint32_t i = 0; i < gap; ++i) slots_.push_back(false);
    }
    slots_.push_back(true);
    while (slots_.size() > kWindow) slots_.pop_front();
    hasLast_ = true;
    lastSeq_ = seq;
}

std::optional<float> PacketLossWindow::lossPercent() const {
    if (slots_.empty()) return std::nullopt;
    size_t lost = 0;
    for (bool got : slots_) {
        if (!got) ++lost;
    }
    float pct = 100.0f * static_cast<float>(lost) / static_cast<float>(slots_.size());
    return std::round(pct * 10.0f) / 10.0f;
}

MqttMessage buildSensorStateMessage(const SensorReading& reading, const LinkQuality& link,
                                     const std::string& topic) {
    StaticJsonDocument<kSensorJsonCapacity> doc;
    doc["id"] = reading.id;
    if (reading.temperatureC.has_value()) doc["t"] = *reading.temperatureC; else doc["t"] = nullptr;
    if (reading.humidityPct.has_value()) doc["h"] = *reading.humidityPct; else doc["h"] = nullptr;
    if (reading.batteryVoltage.has_value()) doc["v"] = *reading.batteryVoltage; else doc["v"] = nullptr;
    doc["boot"] = reading.bootCount;
    doc["seq"] = reading.seq;
    doc["lb"] = reading.lowBattery ? 1 : 0;
    doc["err"] = reading.rawErr;
    doc["rssi"] = link.rssi;
    if (link.snr.has_value()) doc["snr"] = *link.snr; else doc["snr"] = nullptr;
    if (link.lossPct.has_value()) doc["loss"] = *link.lossPct; else doc["loss"] = nullptr;

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
                                  int precision = -1,
                                  const std::string& swVersion = "",
                                  const std::string& displayName = "") {
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

    const std::string& shownName = displayName.empty() ? nodeId : displayName;

    StaticJsonDocument<kDiscoveryJsonCapacity> doc;
    doc["name"] = shownName + " " + nameSuffix;
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
    dev["name"] = shownName;
    dev["mdl"] = "LoRa Sensor Node";
    dev["mf"] = "DIY";
    dev["via_device"] = gateway.deviceName;
    // Explicit JSON null when the node's actual version isn't known yet, not
    // simply omitted (Codex review on PR #22): Home Assistant merges
    // device-info fields across the several discovery configs that share one
    // device id (its own discovery docs describe this), which only works if
    // an *omitted* field means "leave whatever's already there" -- so a node
    // already discovered without a known version would keep showing a stale
    // one forever if this key just vanished from a later update. An explicit
    // null instead asks HA to clear it, the same "no value" idiom this file
    // already uses for missing sensor readings (see doc["t"]/doc["h"]/
    // doc["v"] above). Once a real version is known (a node's cold-boot
    // packet carried "sw" -- see SensorReading::swVersion and
    // GatewayOrchestrator's swVersionByNode_), it's threaded through here
    // instead: a sensor node's own firmware version was never part of the
    // steady-state payload contract (nodes have no OTA path -- they're
    // battery-powered and physically remote -- and re-sending a static
    // string on every packet would waste airtime/battery for nothing), and
    // stamping the *gateway's* version here instead, as this file used to,
    // was actively misleading: two nodes on genuinely different firmware
    // both showed the same "sw", looking like they were running identical
    // firmware.
    if (!swVersion.empty()) {
        dev["sw"] = swVersion;
    } else {
        dev["sw"] = nullptr;
    }

    MqttMessage msg;
    msg.topic = "homeassistant/" + component + "/lora_" + haId + "_" + suffix + "/config";
    serializeJson(doc, msg.payload);
    return msg;
}

} // namespace

std::vector<MqttMessage> buildAutoDiscoveryMessages(const std::string& nodeId,
                                                     const GatewayIdentity& gateway,
                                                     const std::string& swVersion,
                                                     const std::string& displayName) {
    std::vector<MqttMessage> messages;
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "t", "Temperature",
                                             "{{ value_json.t }}", "°C", "temperature", gateway, "", -1,
                                             swVersion, displayName));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "h", "Humidity",
                                             "{{ value_json.h }}", "%", "humidity", gateway, "", -1,
                                             swVersion, displayName));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "v", "Battery",
                                             "{{ value_json.v }}", "V", "voltage", gateway, "", 2, swVersion, displayName));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "r", "Signal",
                                             "{{ value_json.rssi }}", "dBm", "signal_strength", gateway, "",
                                             -1, swVersion, displayName));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "boot", "Boot Count",
                                             "{{ value_json.boot | default(0) }}", "restarts", "",
                                             gateway, "diagnostic", -1, swVersion, displayName));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "seq", "Sequence",
                                             "{{ value_json.seq | default(0) }}", "", "",
                                             gateway, "diagnostic", -1, swVersion, displayName));
    messages.push_back(buildEntityDiscovery(
        "binary_sensor", nodeId, "lb", "Low Battery",
        "{{ 'ON' if value_json.lb is defined and value_json.lb == 1 else 'OFF' }}", "", "battery", gateway,
        "", -1, swVersion, displayName));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "err", "Error",
                                             "{{ value_json.err | default('none', true) }}", "", "",
                                             gateway, "diagnostic", -1, swVersion, displayName));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "snr", "SNR", "{{ value_json.snr }}",
                                             "dB", "signal_strength", gateway, "diagnostic", -1,
                                             swVersion, displayName));
    messages.push_back(buildEntityDiscovery("sensor", nodeId, "loss", "Packet Loss",
                                             "{{ value_json.loss }}", "%", "", gateway, "diagnostic", 1,
                                             swVersion, displayName));
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
    doc["wifi_reconnects"] = stats.wifiReconnects;
    doc["wifi_disc_reason"] = stats.wifiDisconnectReason;

    MqttMessage msg;
    msg.topic = gateway.baseTopic + "/gateway/state";
    serializeJson(doc, msg.payload);
    return msg;
}

} // namespace gateway
