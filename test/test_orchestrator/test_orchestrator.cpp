#include <unity.h>

#include <string>

#include "fakes.h"
#include "orchestrator.h"

void setUp(void) {}
void tearDown(void) {}

namespace {

const char* kKitchenSuccessPayload =
    R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":10,"lb":0,"err":"none"})";

} // namespace

// ---------------------------------------------------------------------
// Happy path: LoRa receive -> route -> MQTT publish
// ---------------------------------------------------------------------

static void test_happy_path_publishes_discovery_and_state(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true; // already connected
    FakeStore store("kitchen"); // pre-approved
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload, -72);
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(1, orchestrator.packetsReceived());
    TEST_ASSERT_EQUAL_UINT32(0, orchestrator.queuedMessageCount());

    // 8 discovery messages (first time this node is seen) + 1 state message.
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());

    const auto& stateMsg = mqtt.published.back();
    TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", stateMsg.topic.c_str());
    TEST_ASSERT_FALSE(stateMsg.retain);
    TEST_ASSERT_TRUE(stateMsg.payload.find("\"rssi\":-72") != std::string::npos);

    // A second packet from the same node should not re-send discovery.
    loRa.push(kKitchenSuccessPayload, -70);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(10, mqtt.published.size());
}

static void test_unknown_node_is_pending_and_not_published(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store(""); // nothing approved
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(R"({"id":"newnode","t":20.0})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(0, mqtt.published.size());
    TEST_ASSERT_EQUAL_UINT32(0, orchestrator.queuedMessageCount());

    auto pending = orchestrator.pendingNodeIds();
    TEST_ASSERT_EQUAL_UINT32(1, pending.size());
    TEST_ASSERT_EQUAL_STRING("newnode", pending[0].c_str());
}

static void test_approve_persists_and_clears_pending(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store("");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(R"({"id":"newnode"})");
    orchestrator.tick();
    TEST_ASSERT_FALSE(orchestrator.pendingNodeIds().empty());

    TEST_ASSERT_TRUE(orchestrator.approveNode("newnode"));
    TEST_ASSERT_TRUE(orchestrator.pendingNodeIds().empty());
    TEST_ASSERT_EQUAL_STRING("newnode", store.csv_.c_str());
    TEST_ASSERT_EQUAL_INT(1, store.saveCount);
}

// ---------------------------------------------------------------------
// MQTT disconnect handling: queue while offline, respect backoff, flush
// on reconnect.
// ---------------------------------------------------------------------

static void test_offline_packets_are_queued_not_lost(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true; // WiFi is fine; only the broker is unreachable
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connectShouldSucceed = false;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();

    TEST_ASSERT_EQUAL_INT(1, mqtt.connectAttempts);
    TEST_ASSERT_EQUAL_UINT32(0, mqtt.published.size());
    // 8 discovery + 1 state message, all queued instead of lost.
    TEST_ASSERT_EQUAL_UINT32(9, orchestrator.queuedMessageCount());
}

static void test_reconnect_backoff_is_respected_then_flushes_queue(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connectShouldSucceed = false;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick(); // t=0: first connect attempt, fails, packet queued
    TEST_ASSERT_EQUAL_INT(1, mqtt.connectAttempts);
    TEST_ASSERT_EQUAL_UINT32(9, orchestrator.queuedMessageCount());

    // Still within the 5s backoff window: no new attempt.
    orchestrator.tick();
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, mqtt.connectAttempts);

    clock.advance(4999);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, mqtt.connectAttempts); // still not elapsed

    clock.advance(1); // now exactly 5000ms since the first attempt
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(2, mqtt.connectAttempts); // backoff elapsed, retried (still fails)
    TEST_ASSERT_EQUAL_UINT32(9, orchestrator.queuedMessageCount()); // unchanged, still offline

    // Now let the broker accept the connection.
    mqtt.connectShouldSucceed = true;
    clock.advance(5000);
    orchestrator.tick();

    TEST_ASSERT_EQUAL_INT(3, mqtt.connectAttempts);
    TEST_ASSERT_TRUE(mqtt.connected());
    TEST_ASSERT_EQUAL_UINT32(0, orchestrator.queuedMessageCount());
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());
}

static void test_malformed_packet_is_queued_to_base_topic_when_offline(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connectShouldSucceed = false;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push("not json");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(1, orchestrator.queuedMessageCount());
    TEST_ASSERT_EQUAL_STRING("lora/incoming", orchestrator.pendingQueue().front().topic.c_str());
    TEST_ASSERT_EQUAL_STRING("not json", orchestrator.pendingQueue().front().payload.c_str());
}

// ---------------------------------------------------------------------
// Queue overrun: drop oldest once the bound is exceeded, and count it.
// ---------------------------------------------------------------------

static void test_queue_overrun_drops_oldest_and_counts_dropped(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connectShouldSucceed = false; // stays offline for the whole test
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // First packet from "kitchen": 8 discovery messages + 1 state message queued.
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, orchestrator.queuedMessageCount());
    TEST_ASSERT_EQUAL_UINT32(0, orchestrator.droppedMessageCount());

    // 25 more packets from the same (already-discovered) node: each adds
    // exactly one more state message to the queue, well past the 20 cap.
    for (int i = 0; i < 25; ++i) {
        loRa.push(kKitchenSuccessPayload);
        orchestrator.tick();
    }

    TEST_ASSERT_EQUAL_UINT32(gateway::GatewayOrchestrator::kMaxQueuedMessages,
                              orchestrator.queuedMessageCount());
    // 34 messages were ever enqueued (9 + 25); only 20 fit, so 14 were dropped.
    TEST_ASSERT_EQUAL_UINT32(14, orchestrator.droppedMessageCount());

    // The oldest entries (the discovery configs) must have been evicted;
    // only the most recent sensor-state messages should remain.
    for (const auto& msg : orchestrator.pendingQueue()) {
        TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", msg.topic.c_str());
    }

    TEST_ASSERT_EQUAL_UINT32(0, mqtt.published.size()); // never came online
}

// ---------------------------------------------------------------------
// WiFi disconnect handling: MQTT reconnect is deferred while WiFi is down,
// LoRa packets are still captured/queued, and WiFi reconnect itself
// respects its own backoff.
// ---------------------------------------------------------------------

static void test_wifi_down_defers_mqtt_but_still_queues_packets(void) {
    FakeWifiRadio wifi; // starts disconnected
    wifi.reconnectShouldSucceed = false; // router/AP still down
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();

    TEST_ASSERT_EQUAL_INT(1, wifi.reconnectAttempts);
    TEST_ASSERT_EQUAL_INT(0, mqtt.connectAttempts); // never even tried MQTT without WiFi
    TEST_ASSERT_EQUAL_UINT32(9, orchestrator.queuedMessageCount()); // packet still captured

    // Still within the WiFi backoff window: no retry yet, but packets keep queuing.
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, wifi.reconnectAttempts);
    TEST_ASSERT_EQUAL_INT(0, mqtt.connectAttempts);
    TEST_ASSERT_EQUAL_UINT32(10, orchestrator.queuedMessageCount());
}

static void test_wifi_reconnect_backoff_then_mqtt_follows(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick(); // t=0: first WiFi reconnect attempt, fails
    TEST_ASSERT_EQUAL_INT(1, wifi.reconnectAttempts);

    clock.advance(9999);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, wifi.reconnectAttempts); // still within the 10s backoff

    clock.advance(1); // exactly 10000ms elapsed
    wifi.reconnectShouldSucceed = true; // router/AP comes back
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(2, wifi.reconnectAttempts);
    TEST_ASSERT_TRUE(wifi.connected());
    TEST_ASSERT_EQUAL_INT(0, mqtt.connectAttempts); // this tick only reconnected WiFi

    // Now that WiFi is back, the next tick attempts MQTT.
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, mqtt.connectAttempts);
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_happy_path_publishes_discovery_and_state);
    RUN_TEST(test_unknown_node_is_pending_and_not_published);
    RUN_TEST(test_approve_persists_and_clears_pending);

    RUN_TEST(test_offline_packets_are_queued_not_lost);
    RUN_TEST(test_reconnect_backoff_is_respected_then_flushes_queue);
    RUN_TEST(test_malformed_packet_is_queued_to_base_topic_when_offline);

    RUN_TEST(test_queue_overrun_drops_oldest_and_counts_dropped);

    RUN_TEST(test_wifi_down_defers_mqtt_but_still_queues_packets);
    RUN_TEST(test_wifi_reconnect_backoff_then_mqtt_follows);

    return UNITY_END();
}
