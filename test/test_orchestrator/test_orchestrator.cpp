#include <unity.h>

#include <string>

#include "fakes.h"
#include "orchestrator.h"

void setUp(void) {}
void tearDown(void) {}

namespace {

const char* kKitchenSuccessPayload =
    R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":10,"lb":0,"err":"none"})";

// Same node/boot as kKitchenSuccessPayload but a higher seq, so it isn't
// treated as a duplicate retransmission by the seq-dedup logic below --
// used wherever a test needs a second, genuinely new packet from "kitchen".
const char* kKitchenSuccessPayloadSeq11 =
    R"({"id":"kitchen","t":22.6,"h":45.2,"v":4.1,"boot":12,"seq":11,"lb":0,"err":"none"})";

// Spy for GatewayOrchestrator::PacketLogCallback, which is a plain function
// pointer (like Esp32Display's ActivityCallback), so it can't capture --
// state has to live in globals reset at the top of each test that uses it.
std::string g_loggedTopic;
std::string g_loggedPayload;
int g_logCallCount = 0;

void resetPacketLogSpy() {
    g_loggedTopic.clear();
    g_loggedPayload.clear();
    g_logCallCount = 0;
}

void packetLogSpy(const std::string& topic, const std::string& payload) {
    g_loggedTopic = topic;
    g_loggedPayload = payload;
    g_logCallCount++;
}

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

    // A second, genuinely new packet (higher seq) from the same node should
    // not re-send discovery.
    loRa.push(kKitchenSuccessPayloadSeq11, -70);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(10, mqtt.published.size());
}

// A single tick() must drain every packet the HAL currently has buffered,
// not just one -- this is what actually delivers on interrupt-driven LoRa
// receive's promise (see Esp32LoRaReceiver's ring buffer in hal_esp32.h): a
// burst that arrived while tick() was busy elsewhere must be fully caught up
// on the very next tick(), not trickled out one packet per loop() iteration.
static void test_tick_drains_all_queued_lora_packets_in_one_call(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen,garage,bedroom");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // Three packets from three different nodes, all queued on the HAL side
    // before a single tick() call.
    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":10,"lb":0,"err":"none"})");
    loRa.push(R"({"id":"garage","t":18.0,"h":50.0,"v":4.0,"boot":1,"seq":1,"lb":0,"err":"none"})");
    loRa.push(R"({"id":"bedroom","t":21.0,"h":40.0,"v":4.2,"boot":3,"seq":5,"lb":0,"err":"none"})");

    orchestrator.tick(); // must drain all three, not just the first

    TEST_ASSERT_EQUAL_UINT32(3, orchestrator.packetsReceived());
    // 8 discovery + 1 state message per node, for all 3 nodes.
    TEST_ASSERT_EQUAL_UINT32(27, mqtt.published.size());
}

// Codex review (P2): draining "until the buffer reports empty" has no
// guaranteed termination if the producer (an ISR on real hardware) keeps
// refilling as fast as tick() drains -- that would starve WiFi/OTA
// housekeeping and the watchdog reset, eventually rebooting the gateway.
// tick() must therefore process at most kMaxPacketsPerTick packets per call,
// even when more are already available, catching up over subsequent ticks.
static void test_tick_caps_packets_drained_per_call(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // Prime discovery so every later packet only contributes one message,
    // isolating this test to the per-tick packet-count cap alone.
    loRa.push(kKitchenSuccessPayload); // seq 10
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size()); // 8 discovery + 1 state

    // Queue more packets than the per-tick budget allows.
    const size_t kExtra = gateway::GatewayOrchestrator::kMaxPacketsPerTick + 5;
    for (size_t i = 0; i < kExtra; ++i) {
        std::string payload = std::string(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":)") +
                               std::to_string(11 + i) + R"(,"lb":0,"err":"none"})";
        loRa.push(payload);
    }

    orchestrator.tick(); // must NOT drain all of kExtra in one call
    TEST_ASSERT_EQUAL_UINT32(9 + gateway::GatewayOrchestrator::kMaxPacketsPerTick, mqtt.published.size());

    orchestrator.tick(); // catches up on the remaining 5 next tick
    TEST_ASSERT_EQUAL_UINT32(9 + kExtra, mqtt.published.size());
}

// ---------------------------------------------------------------------
// OLED gets a curated summary, not the raw JSON (the JSON either wraps
// illegibly across a 128x64 screen or gets truncated to something
// meaningless -- see Esp32Display::showLines()); the full JSON is instead
// handed to an optional PacketLogCallback so it stays visible over Serial.
// ---------------------------------------------------------------------

static void test_forwarded_packet_shows_curated_summary_not_raw_json(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload, -72);
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(3, display.lastLines.size());
    TEST_ASSERT_EQUAL_STRING("Fwd: lora/incoming/kitchen", display.lastLines[0].c_str());
    TEST_ASSERT_EQUAL_STRING("T: 22.5C H: 45.0%", display.lastLines[1].c_str());
    TEST_ASSERT_EQUAL_STRING("V: 4.1V", display.lastLines[2].c_str());
    for (const auto& line : display.lastLines) {
        TEST_ASSERT_TRUE(line.find('{') == std::string::npos);
    }
}

static void test_forwarded_summary_flags_low_battery_and_dht_error(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("garage");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // DHT failure (t/h null) plus a low-battery reading.
    loRa.push(R"({"id":"garage","t":null,"h":null,"v":3.2,"boot":1,"seq":1,"lb":1,"err":"dht"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(3, display.lastLines.size());
    TEST_ASSERT_EQUAL_STRING("T: -- H: --", display.lastLines[1].c_str());
    TEST_ASSERT_EQUAL_STRING("V: 3.2V LOW ERR:dht", display.lastLines[2].c_str());
}

static void test_packet_forwarded_callback_receives_full_json_payload(void) {
    resetPacketLogSpy();

    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(
        wifi, loRa, mqtt, store, display, clock, "LoRaGateway", "lora/incoming",
        gateway::GatewayOrchestrator::kDefaultMqttReconnectBackoffMs,
        gateway::GatewayOrchestrator::kDefaultWifiReconnectBackoffMs, packetLogSpy);
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload, -72);
    orchestrator.tick();

    TEST_ASSERT_EQUAL_INT(1, g_logCallCount);
    TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", g_loggedTopic.c_str());
    // Unlike the OLED summary, the log callback gets the complete JSON.
    TEST_ASSERT_TRUE(g_loggedPayload.find("\"id\":\"kitchen\"") != std::string::npos);
    TEST_ASSERT_TRUE(g_loggedPayload.find("\"rssi\":-72") != std::string::npos);
}

static void test_packet_forwarded_callback_does_not_fire_for_pending_node(void) {
    resetPacketLogSpy();

    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store(""); // nothing approved
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(
        wifi, loRa, mqtt, store, display, clock, "LoRaGateway", "lora/incoming",
        gateway::GatewayOrchestrator::kDefaultMqttReconnectBackoffMs,
        gateway::GatewayOrchestrator::kDefaultWifiReconnectBackoffMs, packetLogSpy);
    orchestrator.begin();

    loRa.push(R"({"id":"newnode"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_INT(0, g_logCallCount);
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
// Seq-based deduplication: a node with nothing new to say may resend its
// last reading (there's no ack protocol, so a node can't tell whether its
// last transmission actually got through); a retransmission should still
// count as received but must not be re-published/re-logged/re-displayed.
// ---------------------------------------------------------------------

static void test_duplicate_seq_is_counted_but_not_published(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload); // seq 10
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size()); // 8 discovery + 1 state

    loRa.push(kKitchenSuccessPayload); // exact retransmission: same seq 10
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(2, orchestrator.packetsReceived()); // still counted as received
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());          // but NOT re-published
}

static void test_lower_seq_than_last_seen_is_treated_as_duplicate(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload); // seq 10
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());

    // An out-of-order/stale retransmission carrying a lower seq than the
    // highest already seen for this node's current boot.
    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":5,"lb":0,"err":"none"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(2, orchestrator.packetsReceived());
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());
}

static void test_higher_seq_is_published_normally(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload); // seq 10
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());

    loRa.push(kKitchenSuccessPayloadSeq11); // seq 11: genuinely new
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(10, mqtt.published.size());
}

static void test_reboot_resets_dedup_even_with_lower_seq(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":50,"lb":0,"err":"none"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());

    // Node rebooted (boot count increased) and its own seq counter restarted
    // at 1 -- lower than the last-seen 50, but must NOT be treated as a
    // duplicate, or a rebooted node would go silent forever.
    loRa.push(R"({"id":"kitchen","t":22.0,"h":44.0,"v":4.1,"boot":13,"seq":1,"lb":0,"err":"none"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(10, mqtt.published.size());
}

static void test_dedup_is_bypassed_when_payload_has_no_seq_field(void) {
    // Regression test: a payload that never sends "seq" at all has
    // bootCount/seq default to (0, 0) on every single packet (see
    // SensorReading::hasSeq). Without gating dedup on hasSeq, this node's
    // very first packet would set lastSeen to (0, 0), and every later
    // packet -- despite being a genuinely new, distinct reading -- would
    // look like a duplicate of (0, 0) forever and never get published again.
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("legacy");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(R"({"id":"legacy","t":20.0,"h":40.0,"v":4.0})"); // no boot/seq at all
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size()); // 8 discovery + 1 state

    loRa.push(R"({"id":"legacy","t":20.5,"h":41.0,"v":4.0})"); // still no boot/seq
    orchestrator.tick();
    loRa.push(R"({"id":"legacy","t":21.0,"h":42.0,"v":4.0})"); // still no boot/seq
    orchestrator.tick();

    // Each of these must publish its own state message -- none should be
    // suppressed as a "duplicate".
    TEST_ASSERT_EQUAL_UINT32(11, mqtt.published.size());
}

static void test_duplicate_is_not_shown_on_display_or_logged(void) {
    resetPacketLogSpy();

    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(
        wifi, loRa, mqtt, store, display, clock, "LoRaGateway", "lora/incoming",
        gateway::GatewayOrchestrator::kDefaultMqttReconnectBackoffMs,
        gateway::GatewayOrchestrator::kDefaultWifiReconnectBackoffMs, packetLogSpy);
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, g_logCallCount);
    TEST_ASSERT_EQUAL_INT(1, display.showCount);

    loRa.push(kKitchenSuccessPayload); // duplicate
    orchestrator.tick();

    TEST_ASSERT_EQUAL_INT(1, g_logCallCount);  // unchanged: not re-logged
    TEST_ASSERT_EQUAL_INT(1, display.showCount); // unchanged: OLED not re-flashed
}

// ---------------------------------------------------------------------
// Cold-boot node firmware versioning: a node only sends "sw" on its
// cold-boot packet (see SensorReading::swVersion), and the gateway must
// remember it and re-publish that node's discovery so Home Assistant shows
// the real version -- but only when it actually changes, never erasing it
// just because a later packet has no "sw" at all.
// ---------------------------------------------------------------------

static void test_cold_boot_sw_version_is_included_in_initial_discovery(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // This node's very first packet is itself a cold boot (boot:0) carrying "sw".
    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":0,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size()); // 8 discovery + 1 state
    for (size_t i = 0; i < 8; ++i) {
        TEST_ASSERT_TRUE(mqtt.published[i].payload.find("\"a1b2c3d\"") != std::string::npos);
    }
}

static void test_new_sw_version_triggers_discovery_republish(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // First packet ever, no "sw" (an already-running node that predates this
    // feature, or simply lost its very first cold-boot transmission).
    loRa.push(kKitchenSuccessPayload); // boot:12, seq:10, no "sw"
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size()); // 8 discovery + 1 state

    // A later reboot (battery swap / reflash): higher bootCount resets seq,
    // and this cold-boot packet now carries "sw" for the first time.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":13,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();

    // 8 fresh discovery messages + 1 state message on top of the first batch.
    TEST_ASSERT_EQUAL_UINT32(18, mqtt.published.size());
    for (size_t i = 9; i < 17; ++i) {
        TEST_ASSERT_TRUE(mqtt.published[i].payload.find("\"a1b2c3d\"") != std::string::npos);
    }
}

static void test_subsequent_packets_without_sw_do_not_erase_stored_version(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // Cold boot carrying "sw": initial discovery already includes it.
    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":0,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());

    // Every subsequent packet (bootCount unchanged) omits "sw" entirely, as
    // real nodes do to save airtime/battery -- must not trigger another
    // discovery republish, and must not clear the version already known.
    loRa.push(R"({"id":"kitchen","t":22.6,"h":45.1,"v":4.1,"boot":0,"seq":1})");
    orchestrator.tick();
    loRa.push(R"({"id":"kitchen","t":22.7,"h":45.2,"v":4.1,"boot":0,"seq":2})");
    orchestrator.tick();

    // Only the two state messages were added -- no extra discovery batch.
    TEST_ASSERT_EQUAL_UINT32(11, mqtt.published.size());

    // A later reboot reporting the *same* "sw" it already told the gateway.
    // If the sw-less packets above had wrongly cleared the stored version,
    // this would look like a brand-new value (empty -> "a1b2c3d") and
    // wrongly fire a full discovery republish; it must not, proving the
    // stored version survived the sw-less packets untouched.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":1,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(12, mqtt.published.size()); // one more state message only
}

static void test_same_sw_version_again_does_not_retrigger_discovery(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":0,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());

    // Another cold-boot packet reporting the *same* version it already told
    // the gateway (e.g. a brown-out reboot on unchanged firmware) must not
    // re-send discovery -- nothing about the node's HA representation needs
    // to change.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":1,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(10, mqtt.published.size()); // one more state message only
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
    mqtt.connected_ = true; // start online so this node's discovery completes
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // First packet while online: discovery completes immediately, nothing queued.
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(0, orchestrator.queuedMessageCount());
    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size());

    // Now the broker goes away. This node is already fully discovered, so
    // its further packets each queue only their one state message -- this
    // test is purely about FIFO/drop-oldest queue mechanics, not discovery.
    // Each iteration uses a strictly increasing seq so seq-dedup doesn't
    // suppress them (they'd otherwise all look like retransmissions of the
    // very first packet's seq 10).
    mqtt.connected_ = false;
    mqtt.connectShouldSucceed = false;

    for (int i = 0; i < 25; ++i) {
        std::string payload = std::string(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":)") +
                               std::to_string(11 + i) + R"(,"lb":0,"err":"none"})";
        loRa.push(payload);
        orchestrator.tick();
    }

    TEST_ASSERT_EQUAL_UINT32(gateway::GatewayOrchestrator::kMaxQueuedMessages,
                              orchestrator.queuedMessageCount());
    // 25 messages were enqueued while offline; only 20 fit, so 5 were dropped.
    TEST_ASSERT_EQUAL_UINT32(25 - gateway::GatewayOrchestrator::kMaxQueuedMessages,
                              orchestrator.droppedMessageCount());

    for (const auto& msg : orchestrator.pendingQueue()) {
        TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", msg.topic.c_str());
    }

    TEST_ASSERT_EQUAL_UINT32(9, mqtt.published.size()); // unchanged since going offline
}

// A discovery message evicted from the queue before ever reaching MQTT must
// not leave its node permanently un-discovered: the node's next packet
// should re-enqueue a fresh set of discovery messages instead of Home
// Assistant silently missing that node's entities forever.
static void test_evicted_discovery_is_regenerated_on_next_packet(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connectShouldSucceed = false; // stays offline for the whole test
    FakeStore store("kitchen,garage");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // Kitchen's 8 discovery messages + 1 state message get queued but never
    // delivered (still offline).
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(9, orchestrator.queuedMessageCount());

    // Flood with unrelated "garage" traffic -- never another kitchen packet
    // -- until every one of kitchen's original queued messages has been
    // evicted by the 20-message cap. Each iteration uses a strictly
    // increasing seq so seq-dedup doesn't suppress all but the first of
    // these (which would leave the queue too small to ever evict kitchen).
    for (int i = 0; i < 40; ++i) {
        std::string garagePayload = std::string(R"({"id":"garage","t":18.0,"h":50.0,"v":4.0,"boot":1,"seq":)") +
                                     std::to_string(1 + i) + R"(,"lb":0,"err":"none"})";
        loRa.push(garagePayload);
        orchestrator.tick();
    }

    for (const auto& msg : orchestrator.pendingQueue()) {
        TEST_ASSERT_TRUE(msg.topic.find("kitchen") == std::string::npos);
    }

    // Kitchen's next packet must re-trigger discovery from scratch.
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();

    bool sawFreshKitchenDiscovery = false;
    for (const auto& msg : orchestrator.pendingQueue()) {
        if (msg.topic.find("homeassistant/") != std::string::npos &&
            msg.topic.find("lora_kitchen") != std::string::npos) {
            sawFreshKitchenDiscovery = true;
            break;
        }
    }
    TEST_ASSERT_TRUE(sawFreshKitchenDiscovery);
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

    // Still within the WiFi backoff window: no retry yet, but packets keep
    // queuing (a genuinely new packet -- higher seq -- so it isn't deduped).
    loRa.push(kKitchenSuccessPayloadSeq11);
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

// ---------------------------------------------------------------------
// Gateway command handling: subscribing to the restart command topic on
// every MQTT (re)connect, and reacting to an inbound message on it. This is
// the first real consumer of IMqttClient::subscribe() -- the foundation for
// HA command buttons and, eventually, per-node LoRa downlinks.
// ---------------------------------------------------------------------

static void test_connect_subscribes_to_restart_command_topic(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt; // starts disconnected
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick(); // connects, then must (re-)subscribe

    TEST_ASSERT_TRUE(mqtt.connected());
    TEST_ASSERT_EQUAL_UINT32(1, mqtt.subscribeCalls.size());
    TEST_ASSERT_EQUAL_STRING("lora/incoming/gateway/command/restart", mqtt.subscribeCalls[0].c_str());
}

static void test_restart_command_sets_restart_requested_flag(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    TEST_ASSERT_FALSE(orchestrator.restartRequested());

    orchestrator.tick(); // connects and subscribes to the restart command topic
    mqtt.simulateIncomingMessage("lora/incoming/gateway/command/restart", "PRESS");

    TEST_ASSERT_TRUE(orchestrator.restartRequested());
}

static void test_restart_command_ignores_unexpected_payload(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick();
    // Something other than the button's exact "PRESS" payload -- e.g. a
    // stray retained message, or a topic collision -- must not trigger a
    // restart.
    mqtt.simulateIncomingMessage("lora/incoming/gateway/command/restart", "garbage");

    TEST_ASSERT_FALSE(orchestrator.restartRequested());
}

static void test_resubscribes_to_restart_topic_after_reconnect(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connectShouldSucceed = false; // broker unreachable at first
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick(); // connect fails: no subscribe yet
    TEST_ASSERT_EQUAL_UINT32(0, mqtt.subscribeCalls.size());

    // Broker comes back; backoff has elapsed by the default 5000ms.
    mqtt.connectShouldSucceed = true;
    clock.advance(5000);
    orchestrator.tick();

    // MQTT subscriptions don't survive a disconnect/reconnect on a real
    // broker, so this must be redone on every successful connect().
    TEST_ASSERT_EQUAL_UINT32(1, mqtt.subscribeCalls.size());
    mqtt.simulateIncomingMessage("lora/incoming/gateway/command/restart", "PRESS");
    TEST_ASSERT_TRUE(orchestrator.restartRequested());
}

// Codex review (P1): a *retained* "PRESS" left on the command topic (a
// manual `mosquitto_pub -r`, a misconfigured automation, or a button whose
// retain setting changes) would otherwise be redelivered by the broker on
// every future subscribe -- i.e. every MQTT reconnect -- causing an
// infinite reboot loop. Consuming a valid command must clear any retention
// on that topic.
static void test_restart_command_clears_retained_message_to_prevent_reboot_loop(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick();
    mqtt.simulateIncomingMessage("lora/incoming/gateway/command/restart", "PRESS");

    TEST_ASSERT_TRUE(orchestrator.restartRequested());
    TEST_ASSERT_FALSE(mqtt.published.empty());
    const auto& clearMsg = mqtt.published.back();
    TEST_ASSERT_EQUAL_STRING("lora/incoming/gateway/command/restart", clearMsg.topic.c_str());
    TEST_ASSERT_EQUAL_STRING("", clearMsg.payload.c_str());
    TEST_ASSERT_TRUE(clearMsg.retain);
}

// Codex review (P2): if connect() succeeds but the SUBSCRIBE packet write
// itself fails (e.g. a socket drop mid-write) without taking the connection
// down, connected() stays true but restart commands would otherwise go
// unheard forever, since subscribing was previously only attempted once,
// right after connect(). It must keep retrying every tick until it succeeds.
static void test_failed_subscribe_is_retried_on_a_later_tick(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.subscribeShouldSucceed = false;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick(); // connects; the subscribe attempt fails
    TEST_ASSERT_TRUE(mqtt.connected());
    TEST_ASSERT_EQUAL_UINT32(1, mqtt.subscribeCalls.size());

    orchestrator.tick(); // still connected: must retry the failed subscribe
    TEST_ASSERT_EQUAL_UINT32(2, mqtt.subscribeCalls.size());

    mqtt.subscribeShouldSucceed = true;
    orchestrator.tick(); // this attempt succeeds
    TEST_ASSERT_EQUAL_UINT32(3, mqtt.subscribeCalls.size());

    // Once subscribed, further ticks must not keep re-subscribing.
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(3, mqtt.subscribeCalls.size());

    mqtt.simulateIncomingMessage("lora/incoming/gateway/command/restart", "PRESS");
    TEST_ASSERT_TRUE(orchestrator.restartRequested());
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_happy_path_publishes_discovery_and_state);
    RUN_TEST(test_tick_drains_all_queued_lora_packets_in_one_call);
    RUN_TEST(test_tick_caps_packets_drained_per_call);
    RUN_TEST(test_forwarded_packet_shows_curated_summary_not_raw_json);
    RUN_TEST(test_forwarded_summary_flags_low_battery_and_dht_error);
    RUN_TEST(test_packet_forwarded_callback_receives_full_json_payload);
    RUN_TEST(test_packet_forwarded_callback_does_not_fire_for_pending_node);
    RUN_TEST(test_unknown_node_is_pending_and_not_published);
    RUN_TEST(test_approve_persists_and_clears_pending);

    RUN_TEST(test_duplicate_seq_is_counted_but_not_published);
    RUN_TEST(test_lower_seq_than_last_seen_is_treated_as_duplicate);
    RUN_TEST(test_higher_seq_is_published_normally);
    RUN_TEST(test_reboot_resets_dedup_even_with_lower_seq);
    RUN_TEST(test_dedup_is_bypassed_when_payload_has_no_seq_field);
    RUN_TEST(test_duplicate_is_not_shown_on_display_or_logged);

    RUN_TEST(test_cold_boot_sw_version_is_included_in_initial_discovery);
    RUN_TEST(test_new_sw_version_triggers_discovery_republish);
    RUN_TEST(test_subsequent_packets_without_sw_do_not_erase_stored_version);
    RUN_TEST(test_same_sw_version_again_does_not_retrigger_discovery);

    RUN_TEST(test_offline_packets_are_queued_not_lost);
    RUN_TEST(test_reconnect_backoff_is_respected_then_flushes_queue);
    RUN_TEST(test_malformed_packet_is_queued_to_base_topic_when_offline);

    RUN_TEST(test_queue_overrun_drops_oldest_and_counts_dropped);
    RUN_TEST(test_evicted_discovery_is_regenerated_on_next_packet);

    RUN_TEST(test_wifi_down_defers_mqtt_but_still_queues_packets);
    RUN_TEST(test_wifi_reconnect_backoff_then_mqtt_follows);

    RUN_TEST(test_connect_subscribes_to_restart_command_topic);
    RUN_TEST(test_restart_command_sets_restart_requested_flag);
    RUN_TEST(test_restart_command_ignores_unexpected_payload);
    RUN_TEST(test_resubscribes_to_restart_topic_after_reconnect);
    RUN_TEST(test_restart_command_clears_retained_message_to_prevent_reboot_loop);
    RUN_TEST(test_failed_subscribe_is_retried_on_a_later_tick);

    return UNITY_END();
}
