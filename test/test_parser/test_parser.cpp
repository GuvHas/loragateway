#include <unity.h>

#include <ArduinoJson.h>

#include <cstring>

#include "payload_parser.h"

using namespace gateway;

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------
// parseSensorPayload: the sensor payload contract from the loratemp node
// ---------------------------------------------------------------------

static void test_parse_success_payload(void) {
    const char* json =
        R"({"id":"node_name","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":10,"lb":0,"err":"none"})";
    ParseResult result = parseSensorPayload(json);

    TEST_ASSERT_TRUE(result.ok());
    TEST_ASSERT_EQUAL_STRING("node_name", result.reading.id.c_str());
    TEST_ASSERT_TRUE(result.reading.temperatureC.has_value());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 22.5f, *result.reading.temperatureC);
    TEST_ASSERT_TRUE(result.reading.humidityPct.has_value());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 45.0f, *result.reading.humidityPct);
    TEST_ASSERT_TRUE(result.reading.batteryVoltage.has_value());
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.1f, *result.reading.batteryVoltage);
    TEST_ASSERT_EQUAL_UINT32(12, result.reading.bootCount);
    TEST_ASSERT_EQUAL_UINT32(10, result.reading.seq);
    TEST_ASSERT_FALSE(result.reading.lowBattery);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(SensorError::None), static_cast<int>(result.reading.err));
    TEST_ASSERT_EQUAL_STRING("none", result.reading.rawErr.c_str());
}

static void test_parse_dht_failure_payload_keeps_t_and_h_null(void) {
    const char* json =
        R"({"id":"node_name","t":null,"h":null,"v":4.1,"boot":13,"seq":11,"lb":0,"err":"dht"})";
    ParseResult result = parseSensorPayload(json);

    TEST_ASSERT_TRUE(result.ok());
    TEST_ASSERT_FALSE(result.reading.temperatureC.has_value());
    TEST_ASSERT_FALSE(result.reading.humidityPct.has_value());
    TEST_ASSERT_TRUE(result.reading.batteryVoltage.has_value());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(SensorError::Dht), static_cast<int>(result.reading.err));
    TEST_ASSERT_EQUAL_STRING("dht", result.reading.rawErr.c_str());
}

static void test_parse_low_battery_payload(void) {
    const char* json =
        R"({"id":"node_name","t":22.5,"h":45.0,"v":3.2,"boot":14,"seq":12,"lb":1,"err":"none"})";
    ParseResult result = parseSensorPayload(json);

    TEST_ASSERT_TRUE(result.ok());
    TEST_ASSERT_TRUE(result.reading.lowBattery);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.2f, *result.reading.batteryVoltage);
}

static void test_parse_defaults_when_optional_fields_missing(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"minimal"})"));

    TEST_ASSERT_TRUE(result.ok());
    TEST_ASSERT_FALSE(result.reading.temperatureC.has_value());
    TEST_ASSERT_FALSE(result.reading.humidityPct.has_value());
    TEST_ASSERT_FALSE(result.reading.batteryVoltage.has_value());
    TEST_ASSERT_EQUAL_UINT32(0, result.reading.bootCount);
    TEST_ASSERT_EQUAL_UINT32(0, result.reading.seq);
    TEST_ASSERT_FALSE(result.reading.lowBattery);
    TEST_ASSERT_EQUAL_STRING("none", result.reading.rawErr.c_str());
}

static void test_parse_empty_string_is_invalid_json(void) {
    ParseResult result = parseSensorPayload("", 0);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::InvalidJson), static_cast<int>(result.error));
}

static void test_parse_truncated_json_is_invalid_json(void) {
    const char* json = R"({"id":"node_name","t":22.5,"h":)"; // cut mid-value
    ParseResult result = parseSensorPayload(json, strlen(json));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::InvalidJson), static_cast<int>(result.error));
}

static void test_parse_garbage_bytes_is_invalid_json(void) {
    const char raw[] = {0x01, 0x02, (char)0xff, 0x00};
    ParseResult result = parseSensorPayload(raw, sizeof(raw) - 1);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::InvalidJson), static_cast<int>(result.error));
}

static void test_parse_non_object_json_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string("[1,2,3]"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::NotAnObject), static_cast<int>(result.error));
}

static void test_parse_missing_id_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"t":22.5,"h":45.0})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::MissingId), static_cast<int>(result.error));
}

static void test_parse_id_wrong_type_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":12345})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_blank_id_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"   "})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::EmptyId), static_cast<int>(result.error));
}

static void test_parse_temperature_wrong_type_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","t":"warm"})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_humidity_wrong_type_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","h":true})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_boot_wrong_type_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","boot":"twelve"})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_negative_boot_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","boot":-1})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_fractional_boot_is_rejected(void) {
    // boot/seq are contractually non-negative integers; a fractional value
    // must not be silently truncated (e.g. 1.5 -> 1).
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","boot":1.5})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_boot_overflow_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","boot":1e20})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_lb_out_of_range_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","lb":5})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_lb_fractional_is_rejected(void) {
    // as<int>() would truncate 0.5 -> 0 and 1.9 -> 1, silently accepting an
    // out-of-contract value as if it were a valid boolean flag.
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","lb":0.5})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));

    ParseResult result2 = parseSensorPayload(std::string(R"({"id":"n1","lb":1.9})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result2.error));
}

static void test_parse_lb_accepts_json_bool(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","lb":true})"));
    TEST_ASSERT_TRUE(result.ok());
    TEST_ASSERT_TRUE(result.reading.lowBattery);
}

static void test_parse_err_wrong_type_is_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","err":123})"));
    TEST_ASSERT_EQUAL_INT(static_cast<int>(ParseError::WrongType), static_cast<int>(result.error));
}

static void test_parse_unknown_err_value_is_forwarded_not_rejected(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"n1","err":"radio_timeout"})"));
    TEST_ASSERT_TRUE(result.ok());
    TEST_ASSERT_EQUAL_INT(static_cast<int>(SensorError::Unknown), static_cast<int>(result.reading.err));
    TEST_ASSERT_EQUAL_STRING("radio_timeout", result.reading.rawErr.c_str());
}

// ---------------------------------------------------------------------
// Adversarial / untrusted node ids (the LoRa layer has no authentication)
// ---------------------------------------------------------------------

static void test_parse_sanitizes_mqtt_wildcards_out_of_id(void) {
    ParseResult result = parseSensorPayload(std::string(R"({"id":"evil/topic#+id"})"));
    TEST_ASSERT_TRUE(result.ok());
    TEST_ASSERT_EQUAL_STRING("eviltopicid", result.reading.id.c_str());
}

static void test_sanitize_mqtt_topic_segment_truncates_long_ids(void) {
    std::string longId(100, 'a');
    std::string sanitized = sanitizeMqttTopicSegment(longId, 40);
    TEST_ASSERT_EQUAL_UINT32(40, sanitized.size());
}

static void test_sanitize_mqtt_topic_segment_strips_control_characters(void) {
    std::string raw("node\x01\x02name", 10);
    std::string sanitized = sanitizeMqttTopicSegment(raw);
    TEST_ASSERT_EQUAL_STRING("nodename", sanitized.c_str());
}

static void test_html_escape_neutralizes_script_tag(void) {
    std::string escaped = htmlEscape("<script>alert(1)</script>");
    TEST_ASSERT_EQUAL_STRING("&lt;script&gt;alert(1)&lt;/script&gt;", escaped.c_str());
}

static void test_adversarial_id_is_topic_safe_but_still_needs_html_escaping(void) {
    // sanitizeMqttTopicSegment only guarantees MQTT-topic safety; it does not
    // strip HTML metacharacters. htmlEscape() must be applied separately
    // whenever a node id is rendered into the device-management page.
    ParseResult result = parseSensorPayload(std::string(R"({"id":"<script>alert(1)</script>"})"));
    TEST_ASSERT_TRUE(result.ok());
    TEST_ASSERT_TRUE(result.reading.id.find('/') == std::string::npos);
    TEST_ASSERT_TRUE(result.reading.id.find('#') == std::string::npos);

    std::string renderedInHtml = htmlEscape(result.reading.id);
    TEST_ASSERT_TRUE(renderedInHtml.find("<script>") == std::string::npos);
}

static void test_url_encode_component_escapes_reserved_characters(void) {
    // sanitizeMqttTopicSegment doesn't strip '&', '=', ' ', etc., since none
    // of those are MQTT-unsafe -- but they ARE query-string-unsafe, so a
    // node id containing them must still be percent-encoded before being
    // placed in an href's query string.
    TEST_ASSERT_EQUAL_STRING("a%26b", urlEncodeComponent("a&b").c_str());
    TEST_ASSERT_EQUAL_STRING("a%3Db", urlEncodeComponent("a=b").c_str());
    TEST_ASSERT_EQUAL_STRING("a%20b", urlEncodeComponent("a b").c_str());
    TEST_ASSERT_EQUAL_STRING("abc-_.~123", urlEncodeComponent("abc-_.~123").c_str());
}

static void test_ha_safe_slug_replaces_illegal_characters(void) {
    // Home Assistant's discovery topic restricts node_id/object_id segments
    // to [a-z0-9_-] -- stricter than plain MQTT topic rules, which allow
    // '~' (e.g. a corrupted-in-transit reading like "ga~agetemp"). Disallowed
    // characters become '_', not stripped, so corruption stays visible and
    // two different bad ids can't collide into the same slug.
    TEST_ASSERT_EQUAL_STRING("ga_agetemp", haSafeSlug("ga~agetemp").c_str());
    TEST_ASSERT_EQUAL_STRING("kitchen", haSafeSlug("Kitchen").c_str());
    TEST_ASSERT_EQUAL_STRING("a_b_c", haSafeSlug("a.b/c").c_str());
}

static void test_url_encoding_survives_html_escaping_round_trip(void) {
    // The bug this guards against: htmlEscape("a&b") -> "a&amp;b", which a
    // browser decodes straight back to "a&b" before parsing the query
    // string, so /approve?id=a&amp;b is requested as /approve?id=a&b and
    // only "a" reaches the server. URL-encoding first closes that gap.
    std::string raw = "a&b";
    std::string hrefValue = htmlEscape(urlEncodeComponent(raw));
    TEST_ASSERT_TRUE(hrefValue.find('&') == std::string::npos);
    TEST_ASSERT_EQUAL_STRING("a%26b", hrefValue.c_str());
}

// ---------------------------------------------------------------------
// AllowList
// ---------------------------------------------------------------------

static void test_allowlist_parses_csv_and_checks_case_insensitively(void) {
    AllowList list("Kitchen, Garage ,Attic");
    TEST_ASSERT_TRUE(list.isAllowed("kitchen"));
    TEST_ASSERT_TRUE(list.isAllowed("GARAGE"));
    TEST_ASSERT_TRUE(list.isAllowed("Attic"));
    TEST_ASSERT_FALSE(list.isAllowed("basement"));
}

static void test_allowlist_empty_csv_allows_nothing(void) {
    AllowList list("");
    TEST_ASSERT_FALSE(list.isAllowed("anything"));
}

static void test_allowlist_approve_adds_and_dedupes(void) {
    AllowList list("kitchen");
    TEST_ASSERT_TRUE(list.approve("garage"));
    TEST_ASSERT_FALSE(list.approve("Kitchen")); // already present, case-insensitive
    TEST_ASSERT_TRUE(list.isAllowed("garage"));
    TEST_ASSERT_EQUAL_UINT32(2, list.entries().size());
}

static void test_allowlist_approve_rejects_blank_id(void) {
    AllowList list;
    TEST_ASSERT_FALSE(list.approve("   "));
    TEST_ASSERT_EQUAL_UINT32(0, list.entries().size());
}

static void test_allowlist_remove_deletes_entry(void) {
    AllowList list("kitchen,garage");
    TEST_ASSERT_TRUE(list.remove("Garage"));
    TEST_ASSERT_FALSE(list.isAllowed("garage"));
    TEST_ASSERT_FALSE(list.remove("garage")); // already gone
}

static void test_allowlist_to_csv_round_trips(void) {
    AllowList list("kitchen,garage");
    AllowList reloaded(list.toCsv());
    TEST_ASSERT_TRUE(reloaded.isAllowed("kitchen"));
    TEST_ASSERT_TRUE(reloaded.isAllowed("garage"));
}

// ---------------------------------------------------------------------
// Routing
// ---------------------------------------------------------------------

static void test_decide_route_pending_for_unknown_node(void) {
    AllowList list("kitchen");
    RoutingResult result = decideRoute("garage", "lora/incoming", list);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RouteDecision::Pending), static_cast<int>(result.decision));
    TEST_ASSERT_TRUE(result.topic.empty());
}

static void test_decide_route_approved_builds_lowercase_topic(void) {
    AllowList list("Kitchen");
    RoutingResult result = decideRoute("Kitchen", "lora/incoming", list);
    TEST_ASSERT_EQUAL_INT(static_cast<int>(RouteDecision::Approved), static_cast<int>(result.decision));
    TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", result.topic.c_str());
}

// ---------------------------------------------------------------------
// Outbound MQTT message builders
// ---------------------------------------------------------------------

static void test_availability_topic_suffix(void) {
    TEST_ASSERT_EQUAL_STRING("lora/incoming/gateway/status", availabilityTopic("lora/incoming").c_str());
}

static void test_build_sensor_state_message_preserves_null_readings(void) {
    ParseResult parsed = parseSensorPayload(std::string(
        R"({"id":"node_name","t":null,"h":null,"v":4.1,"boot":13,"seq":11,"lb":0,"err":"dht"})"));
    TEST_ASSERT_TRUE(parsed.ok());

    MqttMessage msg = buildSensorStateMessage(parsed.reading, -72, "lora/incoming/node_name");
    TEST_ASSERT_EQUAL_STRING("lora/incoming/node_name", msg.topic.c_str());

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, msg.payload);
    TEST_ASSERT_FALSE(err);
    TEST_ASSERT_TRUE(doc["t"].isNull());
    TEST_ASSERT_TRUE(doc["h"].isNull());
    TEST_ASSERT_EQUAL_STRING("dht", doc["err"].as<const char*>());
    TEST_ASSERT_EQUAL_INT(-72, doc["rssi"].as<int>());
    TEST_ASSERT_EQUAL_INT(0, doc["lb"].as<int>());
}

static void test_build_auto_discovery_messages_covers_all_entities(void) {
    GatewayIdentity gateway{"LoRaGateway", "lora/incoming"};
    std::vector<MqttMessage> messages = buildAutoDiscoveryMessages("Kitchen", gateway);

    TEST_ASSERT_EQUAL_UINT32(8, messages.size());
    TEST_ASSERT_EQUAL_STRING("homeassistant/sensor/lora_kitchen_t/config", messages[0].topic.c_str());

    // Deserializing (rather than just measuring) a document needs more pool
    // capacity than serializing it did, so this buffer is deliberately
    // larger than the StaticJsonDocument<600> used to build the payload.
    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, messages[0].payload);
    TEST_ASSERT_FALSE(err);
    TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", doc["stat_t"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("LoRaGateway", doc["dev"]["via_device"].as<const char*>());
}

static void test_build_gateway_discovery_messages_covers_diagnostics(void) {
    GatewayIdentity gateway{"LoRaGateway", "lora/incoming"};
    std::vector<MqttMessage> messages = buildGatewayDiscoveryMessages(gateway);

    // wifi, heap, packets_rx, queue_depth, packets_dropped.
    TEST_ASSERT_EQUAL_UINT32(5, messages.size());
    TEST_ASSERT_EQUAL_STRING("homeassistant/sensor/loragateway_wifi/config", messages[0].topic.c_str());
    TEST_ASSERT_EQUAL_STRING("homeassistant/sensor/loragateway_queue/config", messages[3].topic.c_str());
    TEST_ASSERT_EQUAL_STRING("homeassistant/sensor/loragateway_dropped/config", messages[4].topic.c_str());
}

static void test_build_auto_discovery_messages_sanitizes_ha_illegal_characters(void) {
    // Regression test for a real incident: a node id like "ga~agetemp"
    // (e.g. an over-the-air bit error that still passed LoRa's CRC) must not
    // produce an illegal Home Assistant discovery topic. HA logs and drops
    // such a message rather than erroring back to the gateway, so this only
    // ever surfaces as "missing entities" unless it's tested here.
    GatewayIdentity gateway{"LoRaGateway", "lora/incoming"};
    std::vector<MqttMessage> messages = buildAutoDiscoveryMessages("ga~agetemp", gateway);

    TEST_ASSERT_EQUAL_STRING("homeassistant/sensor/lora_ga_agetemp_t/config", messages[0].topic.c_str());

    StaticJsonDocument<1024> doc;
    DeserializationError err = deserializeJson(doc, messages[0].payload);
    TEST_ASSERT_FALSE(err);
    TEST_ASSERT_EQUAL_STRING("lora_ga_agetemp_t", doc["uniq_id"].as<const char*>());
    TEST_ASSERT_EQUAL_STRING("lora_ga_agetemp", doc["dev"]["ids"].as<const char*>());
    // The state topic is untouched by this fix: it must keep matching
    // decideRoute()'s actual publish topic, which only guarantees
    // MQTT-safety, not HA's stricter discovery-topic charset.
    TEST_ASSERT_EQUAL_STRING("lora/incoming/ga~agetemp", doc["stat_t"].as<const char*>());
}

static void test_build_gateway_discovery_messages_sanitizes_ha_illegal_characters(void) {
    // deviceName is user-editable via the config portal and isn't restricted
    // to HA-safe characters, so it needs the same treatment as a node id.
    GatewayIdentity gateway{"Kitchen's Gateway!", "lora/incoming"};
    std::vector<MqttMessage> messages = buildGatewayDiscoveryMessages(gateway);

    TEST_ASSERT_EQUAL_STRING("homeassistant/sensor/kitchen_s_gateway__wifi/config",
                              messages[0].topic.c_str());
}

static void test_build_gateway_status_message(void) {
    GatewayIdentity gateway{"LoRaGateway", "lora/incoming"};
    GatewayStats stats;
    stats.uptimeSeconds = 42;
    stats.freeHeapBytes = 123456;
    stats.wifiRssi = -55;
    stats.packetsReceived = 7;
    stats.ipAddress = "192.168.1.50";
    stats.onlyKnownNodes = true;
    stats.queueDepth = 3;
    stats.packetsDropped = 2;

    MqttMessage msg = buildGatewayStatusMessage(gateway, stats);
    TEST_ASSERT_EQUAL_STRING("lora/incoming/gateway/state", msg.topic.c_str());

    StaticJsonDocument<512> doc;
    DeserializationError err = deserializeJson(doc, msg.payload);
    TEST_ASSERT_FALSE(err);
    TEST_ASSERT_EQUAL_UINT32(42, doc["uptime_s"].as<uint32_t>());
    TEST_ASSERT_EQUAL_STRING("192.168.1.50", doc["ip"].as<const char*>());
    TEST_ASSERT_TRUE(doc["onlyknown"].as<bool>());
    TEST_ASSERT_EQUAL_UINT32(3, doc["queue_depth"].as<uint32_t>());
    TEST_ASSERT_EQUAL_UINT32(2, doc["packets_dropped"].as<uint32_t>());
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_parse_success_payload);
    RUN_TEST(test_parse_dht_failure_payload_keeps_t_and_h_null);
    RUN_TEST(test_parse_low_battery_payload);
    RUN_TEST(test_parse_defaults_when_optional_fields_missing);

    RUN_TEST(test_parse_empty_string_is_invalid_json);
    RUN_TEST(test_parse_truncated_json_is_invalid_json);
    RUN_TEST(test_parse_garbage_bytes_is_invalid_json);
    RUN_TEST(test_parse_non_object_json_is_rejected);
    RUN_TEST(test_parse_missing_id_is_rejected);
    RUN_TEST(test_parse_id_wrong_type_is_rejected);
    RUN_TEST(test_parse_blank_id_is_rejected);
    RUN_TEST(test_parse_temperature_wrong_type_is_rejected);
    RUN_TEST(test_parse_humidity_wrong_type_is_rejected);
    RUN_TEST(test_parse_boot_wrong_type_is_rejected);
    RUN_TEST(test_parse_negative_boot_is_rejected);
    RUN_TEST(test_parse_fractional_boot_is_rejected);
    RUN_TEST(test_parse_boot_overflow_is_rejected);
    RUN_TEST(test_parse_lb_out_of_range_is_rejected);
    RUN_TEST(test_parse_lb_fractional_is_rejected);
    RUN_TEST(test_parse_lb_accepts_json_bool);
    RUN_TEST(test_parse_err_wrong_type_is_rejected);
    RUN_TEST(test_parse_unknown_err_value_is_forwarded_not_rejected);

    RUN_TEST(test_parse_sanitizes_mqtt_wildcards_out_of_id);
    RUN_TEST(test_sanitize_mqtt_topic_segment_truncates_long_ids);
    RUN_TEST(test_sanitize_mqtt_topic_segment_strips_control_characters);
    RUN_TEST(test_html_escape_neutralizes_script_tag);
    RUN_TEST(test_adversarial_id_is_topic_safe_but_still_needs_html_escaping);
    RUN_TEST(test_url_encode_component_escapes_reserved_characters);
    RUN_TEST(test_ha_safe_slug_replaces_illegal_characters);
    RUN_TEST(test_url_encoding_survives_html_escaping_round_trip);

    RUN_TEST(test_allowlist_parses_csv_and_checks_case_insensitively);
    RUN_TEST(test_allowlist_empty_csv_allows_nothing);
    RUN_TEST(test_allowlist_approve_adds_and_dedupes);
    RUN_TEST(test_allowlist_approve_rejects_blank_id);
    RUN_TEST(test_allowlist_remove_deletes_entry);
    RUN_TEST(test_allowlist_to_csv_round_trips);

    RUN_TEST(test_decide_route_pending_for_unknown_node);
    RUN_TEST(test_decide_route_approved_builds_lowercase_topic);

    RUN_TEST(test_availability_topic_suffix);
    RUN_TEST(test_build_sensor_state_message_preserves_null_readings);
    RUN_TEST(test_build_auto_discovery_messages_covers_all_entities);
    RUN_TEST(test_build_gateway_discovery_messages_covers_diagnostics);
    RUN_TEST(test_build_auto_discovery_messages_sanitizes_ha_illegal_characters);
    RUN_TEST(test_build_gateway_discovery_messages_sanitizes_ha_illegal_characters);
    RUN_TEST(test_build_gateway_status_message);

    return UNITY_END();
}
