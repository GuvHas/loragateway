#include <unity.h>

#include <string>

#include "fakes.h"
#include "orchestrator.h"

void setUp(void) {}
void tearDown(void) {}

namespace {

// Discovery configs published per node on first sighting.
constexpr uint32_t kD = gateway::kNodeDiscoveryEntityCount;

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

    // kD discovery messages (first time this node is seen) + 1 state message.
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    const auto& stateMsg = mqtt.published.back();
    TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", stateMsg.topic.c_str());
    // Retained, same as the gateway's own status message (see
    // test_state_message_is_retained_to_avoid_ha_discovery_race below for
    // why this matters): a non-retained state published in the same burst
    // as this node's discovery configs can race Home Assistant's discovery
    // processing and simply be missed.
    TEST_ASSERT_TRUE(stateMsg.retain);
    TEST_ASSERT_TRUE(stateMsg.payload.find("\"rssi\":-72") != std::string::npos);

    // A second, genuinely new packet (higher seq) from the same node should
    // not re-send discovery.
    loRa.push(kKitchenSuccessPayloadSeq11, -70);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 2, mqtt.published.size());
}

// Field report: Home Assistant only updated a node's entities starting with
// its *second* transmission, never its first. On first sighting, this
// node's kD discovery configs and its state message are all published
// synchronously in the same burst; Home Assistant must finish processing a
// discovery config (create the entity, subscribe to its state topic) before
// it can receive anything on that topic. A non-retained state message that
// reaches the broker before that subscription exists is gone forever --
// unlike discovery, which this codebase already retains. Every state
// message must be retained for exactly the same reason the gateway's own
// status message already is (see main.cpp's publishGatewayStatus()).
static void test_state_message_is_retained_to_avoid_ha_discovery_race(void) {
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

    // Every one of this node's kD discovery messages must be retained...
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());
    for (size_t i = 0; i < kD; ++i) {
        TEST_ASSERT_TRUE(mqtt.published[i].retain);
    }
    // ...and so must the state message published in the very same burst,
    // immediately after them.
    TEST_ASSERT_TRUE(mqtt.published[kD].retain);
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
    // kD discovery + 1 state message per node, for all 3 nodes.
    TEST_ASSERT_EQUAL_UINT32(3 * (kD + 1), mqtt.published.size());
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // kD discovery + 1 state

    // Queue more packets than the per-tick budget allows.
    const size_t kExtra = gateway::GatewayOrchestrator::kMaxPacketsPerTick + 5;
    for (size_t i = 0; i < kExtra; ++i) {
        std::string payload = std::string(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":)") +
                               std::to_string(11 + i) + R"(,"lb":0,"err":"none"})";
        loRa.push(payload);
    }

    orchestrator.tick(); // must NOT drain all of kExtra in one call
    TEST_ASSERT_EQUAL_UINT32(kD + 1 + gateway::GatewayOrchestrator::kMaxPacketsPerTick, mqtt.published.size());

    orchestrator.tick(); // catches up on the remaining 5 next tick
    TEST_ASSERT_EQUAL_UINT32(kD + 1 + kExtra, mqtt.published.size());
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

// Codex review on PR #25: state messages are now retained (to close a Home
// Assistant discovery race -- see test_state_message_is_retained_to_avoid_ha_discovery_race),
// which means a removed node's last reading would otherwise stay
// permanently served to any future subscriber of that topic -- including a
// re-approved node that reuses the exact same id -- looking exactly like
// live current data. removeNode() must publish an empty retained payload to
// clear it.
static void test_remove_node_clears_retained_state_message(void) {
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

    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // kD discovery + 1 (retained) state

    TEST_ASSERT_TRUE(orchestrator.removeNode("kitchen"));

    const auto& tombstone = mqtt.published.back();
    TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", tombstone.topic.c_str());
    TEST_ASSERT_EQUAL_STRING("", tombstone.payload.c_str());
    TEST_ASSERT_TRUE(tombstone.retain);
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // kD discovery + 1 state

    loRa.push(kKitchenSuccessPayload); // exact retransmission: same seq 10
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(2, orchestrator.packetsReceived()); // still counted as received
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());          // but NOT re-published
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    // An out-of-order/stale retransmission carrying a lower seq than the
    // highest already seen for this node's current boot.
    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":5,"lb":0,"err":"none"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(2, orchestrator.packetsReceived());
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    loRa.push(kKitchenSuccessPayloadSeq11); // seq 11: genuinely new
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 2, mqtt.published.size());
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    // Node rebooted (boot count increased) and its own seq counter restarted
    // at 1 -- lower than the last-seen 50, but must NOT be treated as a
    // duplicate, or a rebooted node would go silent forever.
    loRa.push(R"({"id":"kitchen","t":22.0,"h":44.0,"v":4.1,"boot":13,"seq":1,"lb":0,"err":"none"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(kD + 2, mqtt.published.size());
}

// Field report: an already-approved node was reflashed (resetting its RTC
// memory, so bootCount restarted from 1), but the gateway -- a long-running
// process that had never itself restarted -- still remembered a much higher
// bootCount from before the reflash. Every post-reflash packet looked like a
// bootCount that went backwards and was silently dropped as a duplicate
// forever, even though the node was fully approved and transmitting fine. A
// bootCount of exactly 1 (this codebase's cold-boot signal) must reset
// tracking even when it's lower than what's already stored.
static void test_reflash_with_lower_boot_count_is_not_treated_as_duplicate(void) {
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

    // Node has been running a long time before the gateway ever restarted.
    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":50,"seq":100,"lb":0,"err":"none"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    // Node gets reflashed/battery-swapped: its RTC memory resets, so this
    // cold boot's bootCount/seq are far *lower* than what the gateway still
    // remembers -- must still be accepted, not suppressed forever.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":1,"seq":1,"lb":0,"err":"none"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 2, mqtt.published.size());

    // And dedup must still work normally from this new, lower watermark:
    // a retransmission of that same reflash packet must be dropped.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":1,"seq":1,"lb":0,"err":"none"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 2, mqtt.published.size());
}

// Codex review (P2) on PR #24: the first fix attempt reset tracking on *any*
// bootCount change, not just 1 or an increase -- which let two previously-
// accepted, non-cold-boot bootCounts be replayed alternately to bypass
// dedup indefinitely (each looked like a fresh "different" session relative
// to the other). A lower bootCount that *isn't* 1 must still be rejected as
// a duplicate, exactly as before the reflash fix.
static void test_lower_boot_count_other_than_one_is_still_treated_as_duplicate(void) {
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

    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":50,"seq":100,"lb":0,"err":"none"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    // A captured older packet (or an attacker's forged one) with a lower
    // bootCount that is NOT 1 -- not a plausible cold boot, just a backwards
    // value -- must still be rejected, same as pre-reflash-fix behavior.
    loRa.push(R"({"id":"kitchen","t":99.0,"h":99.0,"v":4.1,"boot":5,"seq":1,"lb":0,"err":"none"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(2, orchestrator.packetsReceived()); // still counted as received
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());          // but NOT published

    // Replaying the original high-watermark packet again afterward must
    // also still be rejected (same boot, same-or-lower seq) -- confirming
    // the rejected lower-bootCount attempt didn't corrupt the watermark.
    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":50,"seq":100,"lb":0,"err":"none"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // kD discovery + 1 state

    loRa.push(R"({"id":"legacy","t":20.5,"h":41.0,"v":4.0})"); // still no boot/seq
    orchestrator.tick();
    loRa.push(R"({"id":"legacy","t":21.0,"h":42.0,"v":4.0})"); // still no boot/seq
    orchestrator.tick();

    // Each of these must publish its own state message -- none should be
    // suppressed as a "duplicate".
    TEST_ASSERT_EQUAL_UINT32(kD + 3, mqtt.published.size());
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

    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // kD discovery + 1 state
    for (size_t i = 0; i < kD; ++i) {
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // kD discovery + 1 state

    // A later reboot (battery swap / reflash): higher bootCount resets seq,
    // and this cold-boot packet now carries "sw" for the first time.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":13,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();

    // kD fresh discovery messages + 1 state message on top of the first batch.
    TEST_ASSERT_EQUAL_UINT32(2 * kD + 2, mqtt.published.size());
    for (size_t i = kD + 1; i < 2 * kD + 1; ++i) {
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    // Every subsequent packet (bootCount unchanged) omits "sw" entirely, as
    // real nodes do to save airtime/battery -- must not trigger another
    // discovery republish, and must not clear the version already known.
    loRa.push(R"({"id":"kitchen","t":22.6,"h":45.1,"v":4.1,"boot":0,"seq":1})");
    orchestrator.tick();
    loRa.push(R"({"id":"kitchen","t":22.7,"h":45.2,"v":4.1,"boot":0,"seq":2})");
    orchestrator.tick();

    // Only the two state messages were added -- no extra discovery batch.
    TEST_ASSERT_EQUAL_UINT32(kD + 3, mqtt.published.size());

    // A later reboot reporting the *same* "sw" it already told the gateway.
    // If the sw-less packets above had wrongly cleared the stored version,
    // this would look like a brand-new value (empty -> "a1b2c3d") and
    // wrongly fire a full discovery republish; it must not, proving the
    // stored version survived the sw-less packets untouched.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":1,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(kD + 4, mqtt.published.size()); // one more state message only
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    // Another cold-boot packet reporting the *same* version it already told
    // the gateway (e.g. a brown-out reboot on unchanged firmware) must not
    // re-send discovery -- nothing about the node's HA representation needs
    // to change.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":1,"seq":0,"sw":"a1b2c3d"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(kD + 2, mqtt.published.size()); // one more state message only
}

// Codex review (P1): swVersionByNode_ was RAM-only, so a gateway restart
// forgot every version it had learned and republished null for any node not
// due for another cold boot anytime soon. It must be persisted through
// INodeStore and reloaded on the next begin().
static void test_learned_sw_version_is_persisted_and_survives_restart(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen"); // shared across the "restart" below
    FakeDisplay display;
    FakeClock clock;

    {
        gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                                   "LoRaGateway", "lora/incoming");
        orchestrator.begin();

        loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":0,"seq":0,"sw":"a1b2c3d"})");
        orchestrator.tick();
    }

    TEST_ASSERT_TRUE(store.nodeVersionsSaveCount > 0);
    TEST_ASSERT_TRUE(store.nodeVersionsCsv_.find("kitchen=a1b2c3d") != std::string::npos);

    // A brand-new orchestrator instance over the same (persisted) store --
    // standing in for a gateway restart, which wipes every in-memory map
    // (discoveredNodes_, pendingDiscoveryCount_, swVersionByNode_ before this
    // fix) but not whatever was actually saved to the store.
    FakeLoRa loRa2;
    FakeMqtt mqtt2;
    mqtt2.connected_ = true;
    gateway::GatewayOrchestrator orchestrator2(wifi, loRa2, mqtt2, store, display, clock, "LoRaGateway",
                                                "lora/incoming");
    orchestrator2.begin(); // must reload the persisted version

    // This node's next packet after the "restart" carries no "sw" (an
    // ordinary, non-cold-boot transmission) -- it's still this orchestrator
    // instance's first sight of "kitchen", so it rebuilds discovery from
    // scratch. Without the fix this publishes null; with it, the version
    // learned before the "restart" must still be there.
    loRa2.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":0,"seq":1})");
    orchestrator2.tick();

    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt2.published.size()); // kD discovery + 1 state
    for (size_t i = 0; i < kD; ++i) {
        TEST_ASSERT_TRUE(mqtt2.published[i].payload.find("\"a1b2c3d\"") != std::string::npos);
    }
}

// Codex review (P1): a brand new, not-yet-approved node's cold-boot packet
// used to return through the Pending branch before "sw" was ever looked at.
// Since a node only sends "sw" on that one cold-boot packet, approving it
// afterwards meant its real version was gone for good until a second,
// possibly very distant, cold boot. Capturing "sw" must happen before the
// Pending-node early return.
static void test_sw_reported_before_approval_is_not_lost(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store(""); // nothing approved yet
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // The new node's only cold-boot packet arrives *before* it's approved.
    loRa.push(R"({"id":"newnode","t":20.0,"h":40.0,"v":4.0,"boot":0,"seq":0,"sw":"f00dcaf"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(0, mqtt.published.size()); // still pending, nothing published

    TEST_ASSERT_TRUE(orchestrator.approveNode("newnode"));

    // Every packet from here on (same boot session) omits "sw", as real
    // nodes do -- this must still show the version captured while it was
    // still pending, not null.
    loRa.push(R"({"id":"newnode","t":20.5,"h":41.0,"v":4.0,"boot":0,"seq":1})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // kD discovery + 1 state
    for (size_t i = 0; i < kD; ++i) {
        TEST_ASSERT_TRUE(mqtt.published[i].payload.find("\"f00dcaf\"") != std::string::npos);
    }
}

// Codex review (P2): if a node's "sw" changes again while its *previous*
// discovery batch is still draining out of the store-and-forward queue
// (broker unreachable), the naive fix would just drop the new value on the
// floor -- once the stale batch finally flushes and completes, a later
// packet reporting that same already-stored new value never looks "changed"
// again, so it would never get published. The change must be deferred and
// applied the moment the in-flight batch actually finishes.
static void test_sw_change_during_in_flight_discovery_is_applied_once_batch_completes(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connectShouldSucceed = false; // broker unreachable: discovery queues, never completes
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // Cold boot #1: "v1". Discovery batch gets queued but the broker is down,
    // so it never actually completes -- pendingDiscoveryCount_ for "kitchen"
    // stays non-zero indefinitely.
    loRa.push(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":0,"seq":0,"sw":"v1"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(0, mqtt.published.size());
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount()); // kD discovery(v1) + 1 state

    // Cold boot #2 (a reflash while still offline): "v2". The v1 batch is
    // still in flight, so this must be deferred, not dropped.
    loRa.push(R"({"id":"kitchen","t":21.0,"h":44.0,"v":4.1,"boot":1,"seq":0,"sw":"v2"})");
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(0, mqtt.published.size());
    TEST_ASSERT_EQUAL_UINT32(kD + 2, orchestrator.queuedMessageCount()); // + 1 more state message only

    // Broker comes back: the stale v1 batch flushes and completes, which
    // must immediately trigger the deferred v2 republish.
    mqtt.connectShouldSucceed = true;
    clock.advance(gateway::GatewayOrchestrator::kDefaultMqttReconnectBackoffMs);
    orchestrator.tick();

    // kD (v1) + kD (v2, republished the instant the v1 batch completed) + 2
    // state messages.
    TEST_ASSERT_EQUAL_UINT32(2 * kD + 2, mqtt.published.size());
    TEST_ASSERT_EQUAL_UINT32(0, orchestrator.queuedMessageCount());
    for (size_t i = 0; i < kD; ++i) {
        TEST_ASSERT_TRUE(mqtt.published[i].payload.find("\"v1\"") != std::string::npos);
    }
    for (size_t i = kD; i < 2 * kD; ++i) {
        TEST_ASSERT_TRUE(mqtt.published[i].payload.find("\"v2\"") != std::string::npos);
    }
}

// ---------------------------------------------------------------------
// Friendly node names set on the /devices page
// ---------------------------------------------------------------------

static void test_node_name_republishes_discovery_with_name(void) {
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
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    TEST_ASSERT_TRUE(orchestrator.setNodeName("kitchen", "Kitchen Window"));
    TEST_ASSERT_EQUAL_STRING("Kitchen Window", orchestrator.nodeName("kitchen").c_str());
    TEST_ASSERT_EQUAL_STRING("kitchen=Kitchen Window", store.nodeNamesCsv_.c_str());

    // A fresh discovery batch with the new name; unique ids stay on the id.
    TEST_ASSERT_EQUAL_UINT32(2 * kD + 1, mqtt.published.size());
    for (size_t i = kD + 1; i < 2 * kD + 1; ++i) {
        const std::string& payload = mqtt.published[i].payload;
        TEST_ASSERT_TRUE(payload.find("\"name\":\"Kitchen Window") != std::string::npos);
        TEST_ASSERT_TRUE(payload.find("\"uniq_id\":\"lora_kitchen_") != std::string::npos);
    }

    // Setting the same name again changes nothing.
    TEST_ASSERT_FALSE(orchestrator.setNodeName("kitchen", "  Kitchen Window "));
    TEST_ASSERT_EQUAL_UINT32(2 * kD + 1, mqtt.published.size());
}

static void test_node_name_cleared_goes_back_to_id(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store("kitchen");
    store.nodeNamesCsv_ = "kitchen=Kitchen Window";
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();
    // Loaded from the store at begin(), and used for the first discovery.
    TEST_ASSERT_EQUAL_STRING("Kitchen Window", orchestrator.nodeName("kitchen").c_str());
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_TRUE(mqtt.published[0].payload.find("\"Kitchen Window\"") != std::string::npos);

    TEST_ASSERT_TRUE(orchestrator.setNodeName("kitchen", ""));
    TEST_ASSERT_EQUAL_STRING("", orchestrator.nodeName("kitchen").c_str());
    TEST_ASSERT_EQUAL_STRING("", store.nodeNamesCsv_.c_str());
    TEST_ASSERT_EQUAL_UINT32(2 * kD + 1, mqtt.published.size());
    TEST_ASSERT_TRUE(mqtt.published[kD + 1].payload.find("\"name\":\"kitchen Temperature\"") !=
                     std::string::npos);
}

static void test_node_name_rejected_for_unapproved_node(void) {
    FakeWifiRadio wifi;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();
    TEST_ASSERT_FALSE(orchestrator.setNodeName("garage", "Garage"));
    TEST_ASSERT_EQUAL_INT(0, store.nodeNamesSaveCount);
}

static void test_node_name_before_first_packet_is_used_by_first_discovery(void) {
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
    TEST_ASSERT_TRUE(orchestrator.setNodeName("kitchen", "Kitchen"));
    TEST_ASSERT_EQUAL_UINT32(0, mqtt.published.size()); // not discovered yet: nothing to republish

    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());
    TEST_ASSERT_TRUE(mqtt.published[0].payload.find("\"name\":\"Kitchen Temperature\"") !=
                     std::string::npos);
}

static void test_node_name_during_in_flight_discovery_is_applied_once_batch_completes(void) {
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
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount());

    TEST_ASSERT_TRUE(orchestrator.setNodeName("kitchen", "Kitchen"));
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount()); // deferred, not queued twice

    mqtt.connectShouldSucceed = true;
    clock.advance(gateway::GatewayOrchestrator::kDefaultMqttReconnectBackoffMs);
    orchestrator.tick();
    // The old batch completes before its queued state message goes out, so
    // the renamed batch comes straight after it.
    TEST_ASSERT_EQUAL_UINT32(2 * kD + 1, mqtt.published.size());
    for (size_t i = kD; i < 2 * kD; ++i) {
        TEST_ASSERT_TRUE(mqtt.published[i].payload.find("\"Kitchen\"") != std::string::npos);
    }
}

static void test_remove_node_forgets_its_name(void) {
    FakeWifiRadio wifi;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();
    TEST_ASSERT_TRUE(orchestrator.setNodeName("kitchen", "Kitchen"));
    TEST_ASSERT_TRUE(orchestrator.removeNode("kitchen"));
    TEST_ASSERT_EQUAL_STRING("", orchestrator.nodeName("kitchen").c_str());
    TEST_ASSERT_EQUAL_STRING("", store.nodeNamesCsv_.c_str());
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
    // kD discovery + 1 state message, all queued instead of lost.
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount());
}

// Two nodes seen for the first time during a broker outage must both fit in
// the offline queue, discovery and state, without dropping anything.
static void test_two_new_nodes_fit_in_offline_queue(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connectShouldSucceed = false;
    FakeStore store("kitchen,garage");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload);
    loRa.push(R"({"id":"garage","t":18.0,"h":50.0,"v":4.0,"boot":1,"seq":1,"lb":0,"err":"none"})");
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(2 * (kD + 1), orchestrator.queuedMessageCount());
    TEST_ASSERT_EQUAL_UINT32(0, orchestrator.droppedMessageCount());
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount());

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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount()); // unchanged, still offline

    // Now let the broker accept the connection.
    mqtt.connectShouldSucceed = true;
    clock.advance(5000);
    orchestrator.tick();

    TEST_ASSERT_EQUAL_INT(3, mqtt.connectAttempts);
    TEST_ASSERT_TRUE(mqtt.connected());
    TEST_ASSERT_EQUAL_UINT32(0, orchestrator.queuedMessageCount());
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());
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
// Binary frame v1: decoded into the same reading, published the same way.
// ---------------------------------------------------------------------

// kKitchenSuccessPayload as a binary frame: id "kitchen", boot 12, seq 10,
// t 22.5 (225), h 45.0 (450), v 4.10 ((4100 - 2000) / 10 = 210).
const std::string kKitchenBinaryFrame(
    "\xC1\x00\x07kitchen\x0C\x00\x00\x0A\x00\x00\xE1\x00\xC2\x01\xD2", 21);

static std::string publishedStateFor(const std::string& packet) {
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
    loRa.push(packet, -72);
    orchestrator.tick();
    if (mqtt.published.empty()) return "";
    const auto& msg = mqtt.published.back();
    return msg.topic + " " + msg.payload;
}

static void test_binary_frame_publishes_same_state_as_json(void) {
    std::string fromJson = publishedStateFor(kKitchenSuccessPayload);
    std::string fromBinary = publishedStateFor(kKitchenBinaryFrame);
    TEST_ASSERT_TRUE(fromJson.rfind("lora/incoming/kitchen ", 0) == 0);
    TEST_ASSERT_EQUAL_STRING(fromJson.c_str(), fromBinary.c_str());
}

// A broken binary frame still goes to the base topic, as text: raw bytes
// with NULs would be cut short or unreadable in an MQTT client.
static void test_malformed_binary_frame_forwarded_as_hex(void) {
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

    loRa.push(std::string("\xC1\x00\x07", 3));
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(1, orchestrator.queuedMessageCount());
    TEST_ASSERT_EQUAL_STRING("lora/incoming", orchestrator.pendingQueue().front().topic.c_str());
    TEST_ASSERT_EQUAL_STRING("c10007", orchestrator.pendingQueue().front().payload.c_str());
}

// ---------------------------------------------------------------------
// Link quality: SNR from the radio, loss % from gaps in seq
// ---------------------------------------------------------------------

static std::string lastStateFor(FakeMqtt& mqtt, const char* topic) {
    for (auto it = mqtt.published.rbegin(); it != mqtt.published.rend(); ++it) {
        if (it->topic == topic) return it->payload;
    }
    return "";
}

static void test_state_message_carries_packet_snr(void) {
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

    loRa.push(kKitchenSuccessPayload, -95, -3.25f);
    orchestrator.tick();

    std::string state = lastStateFor(mqtt, "lora/incoming/kitchen");
    TEST_ASSERT_TRUE(state.find("\"snr\":-3.25") != std::string::npos);
    TEST_ASSERT_TRUE(state.find("\"loss\":0") != std::string::npos);
}

static void test_state_message_loss_reflects_seq_gap(void) {
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

    loRa.push(kKitchenSuccessPayload);  // boot 12, seq 10
    orchestrator.tick();
    loRa.push(R"({"id":"kitchen","t":22.6,"h":45.2,"v":4.1,"boot":12,"seq":13,"lb":0,"err":"none"})");
    orchestrator.tick();  // 11 and 12 lost: 2 of 4 expected

    std::string state = lastStateFor(mqtt, "lora/incoming/kitchen");
    TEST_ASSERT_TRUE(state.find("\"loss\":50") != std::string::npos);
}

static void test_duplicate_does_not_change_loss(void) {
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

    loRa.push(kKitchenSuccessPayload);
    loRa.push(kKitchenSuccessPayload);  // retransmission, dropped
    loRa.push(kKitchenSuccessPayloadSeq11);
    orchestrator.tick();

    std::string state = lastStateFor(mqtt, "lora/incoming/kitchen");
    TEST_ASSERT_TRUE(state.find("\"loss\":0") != std::string::npos);
}

static void test_cold_boot_resets_loss(void) {
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

    loRa.push(kKitchenSuccessPayload);  // seq 10
    loRa.push(R"({"id":"kitchen","t":22.6,"boot":12,"seq":20})");  // 9 lost
    loRa.push(R"({"id":"kitchen","t":22.7,"boot":1,"seq":1})");    // reflashed
    orchestrator.tick();

    std::string state = lastStateFor(mqtt, "lora/incoming/kitchen");
    TEST_ASSERT_TRUE(state.find("\"loss\":0") != std::string::npos);
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size());

    // Now the broker goes away. This node is already fully discovered, so
    // its further packets each queue only their one state message -- this
    // test is purely about FIFO/drop-oldest queue mechanics, not discovery.
    // Each iteration uses a strictly increasing seq so seq-dedup doesn't
    // suppress them (they'd otherwise all look like retransmissions of the
    // very first packet's seq 10).
    mqtt.connected_ = false;
    mqtt.connectShouldSucceed = false;

    const int kOfflinePackets = static_cast<int>(gateway::GatewayOrchestrator::kMaxQueuedMessages) + 5;
    for (int i = 0; i < kOfflinePackets; ++i) {
        std::string payload = std::string(R"({"id":"kitchen","t":22.5,"h":45.0,"v":4.1,"boot":12,"seq":)") +
                               std::to_string(11 + i) + R"(,"lb":0,"err":"none"})";
        loRa.push(payload);
        orchestrator.tick();
    }

    TEST_ASSERT_EQUAL_UINT32(gateway::GatewayOrchestrator::kMaxQueuedMessages,
                              orchestrator.queuedMessageCount());
    // Five more messages were enqueued while offline than fit, so 5 were dropped.
    TEST_ASSERT_EQUAL_UINT32(5,
                              orchestrator.droppedMessageCount());

    for (const auto& msg : orchestrator.pendingQueue()) {
        TEST_ASSERT_EQUAL_STRING("lora/incoming/kitchen", msg.topic.c_str());
    }

    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // unchanged since going offline
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

    // Kitchen's kD discovery messages + 1 state message get queued but never
    // delivered (still offline).
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount());

    // Flood with unrelated "garage" traffic -- never another kitchen packet
    // -- until every one of kitchen's original queued messages has been
    // evicted by the queue cap. Each iteration uses a strictly
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
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount()); // packet still captured

    // Still within the WiFi backoff window: no retry yet, but packets keep
    // queuing (a genuinely new packet -- higher seq -- so it isn't deduped).
    loRa.push(kKitchenSuccessPayloadSeq11);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, wifi.reconnectAttempts);
    TEST_ASSERT_EQUAL_INT(0, mqtt.connectAttempts);
    TEST_ASSERT_EQUAL_UINT32(kD + 2, orchestrator.queuedMessageCount());
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
    TEST_ASSERT_EQUAL_UINT32(1, mqtt.subscribeCallsFor("lora/incoming/gateway/command/restart"));
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
    TEST_ASSERT_EQUAL_UINT32(0, mqtt.subscribeCallsFor("lora/incoming/gateway/command/restart"));

    // Broker comes back; backoff has elapsed by the default 5000ms.
    mqtt.connectShouldSucceed = true;
    clock.advance(5000);
    orchestrator.tick();

    // MQTT subscriptions don't survive a disconnect/reconnect on a real
    // broker, so this must be redone on every successful connect().
    TEST_ASSERT_EQUAL_UINT32(1, mqtt.subscribeCallsFor("lora/incoming/gateway/command/restart"));
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
    TEST_ASSERT_EQUAL_UINT32(1, mqtt.subscribeCallsFor("lora/incoming/gateway/command/restart"));

    orchestrator.tick(); // still connected: must retry the failed subscribe
    TEST_ASSERT_EQUAL_UINT32(2, mqtt.subscribeCallsFor("lora/incoming/gateway/command/restart"));

    mqtt.subscribeShouldSucceed = true;
    orchestrator.tick(); // this attempt succeeds
    TEST_ASSERT_EQUAL_UINT32(3, mqtt.subscribeCallsFor("lora/incoming/gateway/command/restart"));

    // Once subscribed, further ticks must not keep re-subscribing.
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(3, mqtt.subscribeCallsFor("lora/incoming/gateway/command/restart"));

    mqtt.simulateIncomingMessage("lora/incoming/gateway/command/restart", "PRESS");
    TEST_ASSERT_TRUE(orchestrator.restartRequested());
}

// ---------------------------------------------------------------------
// Recovering from network restarts (AP reboots, router updates). A field
// report: after the network came back, the gateway sometimes never did.
// ---------------------------------------------------------------------

namespace {
const unsigned long kTenMinutesMs = 10UL * 60UL * 1000UL;
}

// PubSubClient keeps reporting connected() after WiFi drops, because the
// old TCP socket is never closed. Publishing into it blocks in
// NetworkClient::write() (up to 10 s per call) and loses the message, so
// while WiFi is down every message must go to the offline queue instead.
static void test_messages_are_queued_while_wifi_down_even_if_mqtt_reports_connected(void) {
    FakeWifiRadio wifi; // down
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true; // stale session from before the outage
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();

    TEST_ASSERT_EQUAL_UINT32(0, mqtt.published.size());
    TEST_ASSERT_EQUAL_UINT32(kD + 1, orchestrator.queuedMessageCount());
    TEST_ASSERT_FALSE(orchestrator.online());
}

static void test_stale_mqtt_session_is_closed_once_when_wifi_drops(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    mqtt.connected_ = true;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, mqtt.disconnectCalls);
    TEST_ASSERT_FALSE(mqtt.connected());

    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, mqtt.disconnectCalls);
}

static void test_online_requires_both_wifi_and_mqtt(void) {
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

    TEST_ASSERT_FALSE(orchestrator.online()); // WiFi only
    mqtt.connected_ = true;
    TEST_ASSERT_TRUE(orchestrator.online());
    wifi.connected_ = false;
    TEST_ASSERT_FALSE(orchestrator.online()); // stale MQTT only
}

// A fixed 10 s retry tears down a connection attempt that is still waiting
// for DHCP from a router that is itself rebooting. Back off instead.
static void test_wifi_reconnect_backoff_doubles_up_to_one_minute(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    // Expected gaps between attempts: 10 s, 20 s, 40 s, then capped at 60 s.
    const unsigned long gaps[] = {10000, 20000, 40000, 60000, 60000};
    orchestrator.tick(); // attempt 1 at t=0
    int expected = 1;
    for (unsigned long gap : gaps) {
        clock.advance(gap - 1);
        orchestrator.tick();
        TEST_ASSERT_EQUAL_INT(expected, wifi.reconnectAttempts + wifi.hardResetAttempts);
        clock.advance(1);
        orchestrator.tick();
        expected++;
        TEST_ASSERT_EQUAL_INT(expected, wifi.reconnectAttempts + wifi.hardResetAttempts);
    }
}

// WiFi.reconnect() only calls esp_wifi_connect(). It cannot bring back a
// station that WiFiManager switched off, or unstick a driver that is
// "already connecting", so every third attempt is a full reset instead.
static void test_every_third_wifi_attempt_is_a_hard_reset(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    for (int i = 0; i < 6; ++i) {
        orchestrator.tick();
        clock.advance(60000); // always past the backoff
    }

    TEST_ASSERT_EQUAL_INT(4, wifi.reconnectAttempts);
    TEST_ASSERT_EQUAL_INT(2, wifi.hardResetAttempts);
}

static void test_wifi_backoff_and_attempt_count_reset_after_reconnect(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick();       // attempt 1
    clock.advance(10000);
    orchestrator.tick();       // attempt 2, next gap would be 20 s
    clock.advance(20000);
    wifi.reconnectShouldSucceed = true;
    orchestrator.tick();       // attempt 3 (a hard reset) succeeds
    TEST_ASSERT_TRUE(wifi.connected());
    orchestrator.tick();       // online for a tick

    // A new outage starts from scratch: a plain reconnect right away, then 10 s.
    wifi.connected_ = false;
    wifi.reconnectShouldSucceed = false;
    int reconnectsBefore = wifi.reconnectAttempts;
    int hardResetsBefore = wifi.hardResetAttempts;
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(reconnectsBefore + 1, wifi.reconnectAttempts);
    clock.advance(10000);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(reconnectsBefore + 2, wifi.reconnectAttempts);
    TEST_ASSERT_EQUAL_INT(hardResetsBefore, wifi.hardResetAttempts);
}

// Last resort: a reboot clears any WiFi driver state the steps above can't.
static void test_restart_requested_after_ten_minutes_without_wifi(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;
    clock.set(5000);

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick(); // outage noticed at t=5 s
    clock.advance(kTenMinutesMs - 1);
    orchestrator.tick();
    TEST_ASSERT_FALSE(orchestrator.restartRequested());

    clock.advance(1);
    orchestrator.tick();
    TEST_ASSERT_TRUE(orchestrator.restartRequested());
    TEST_ASSERT_EQUAL_STRING("WiFi offline 10 min", orchestrator.restartReason().c_str());
}

static void test_offline_timer_resets_when_wifi_returns(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick();
    clock.advance(kTenMinutesMs - 1000);
    wifi.connected_ = true; // back just in time
    orchestrator.tick();

    wifi.connected_ = false; // drops again
    orchestrator.tick();
    clock.advance(kTenMinutesMs - 1);
    orchestrator.tick();
    TEST_ASSERT_FALSE(orchestrator.restartRequested());
}

// While WiFiManager's setup portal is open the radio is in AP mode for the
// person configuring it. Reconnecting, resetting or rebooting would kick
// them off, so recovery waits until the portal closes.
static void test_wifi_recovery_paused_while_setup_portal_is_open(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();
    orchestrator.setWifiRecoveryEnabled(false);

    for (int i = 0; i < 30; ++i) {
        orchestrator.tick();
        clock.advance(60000);
    }
    TEST_ASSERT_EQUAL_INT(0, wifi.reconnectAttempts);
    TEST_ASSERT_EQUAL_INT(0, wifi.hardResetAttempts);
    TEST_ASSERT_FALSE(orchestrator.restartRequested());

    // Portal closes: recovery resumes with a fresh offline timer.
    orchestrator.setWifiRecoveryEnabled(true);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_INT(1, wifi.reconnectAttempts);
    clock.advance(kTenMinutesMs - 1);
    orchestrator.tick();
    TEST_ASSERT_FALSE(orchestrator.restartRequested());
}

static void test_wifi_reconnect_attempts_are_counted_for_diagnostics(void) {
    FakeWifiRadio wifi;
    wifi.reconnectShouldSucceed = false;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store;
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    for (int i = 0; i < 4; ++i) {
        orchestrator.tick();
        clock.advance(60000);
    }
    TEST_ASSERT_EQUAL_UINT32(4, orchestrator.wifiReconnectAttempts());
}

static void test_mqtt_restart_command_reason(void) {
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
    TEST_ASSERT_EQUAL_STRING("MQTT command", orchestrator.restartReason().c_str());
}

// ---------------------------------------------------------------------
// Home Assistant restarts: HA publishes "online" on homeassistant/status
// when it starts, and expects integrations to resend discovery.
// ---------------------------------------------------------------------

static void test_subscribes_to_home_assistant_status(void) {
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
    TEST_ASSERT_EQUAL_UINT32(1, mqtt.subscribeCallsFor("homeassistant/status"));
}

static void test_home_assistant_online_triggers_rediscovery(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick(); // connect + subscribe
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // kD discovery + state
    TEST_ASSERT_FALSE(orchestrator.takeHomeAssistantRestart());

    mqtt.simulateIncomingMessage("homeassistant/status", "online");
    TEST_ASSERT_TRUE(orchestrator.takeHomeAssistantRestart());
    TEST_ASSERT_FALSE(orchestrator.takeHomeAssistantRestart()); // consumed

    mqtt.published.clear();
    loRa.push(kKitchenSuccessPayloadSeq11);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(kD + 1, mqtt.published.size()); // discovery sent again
}

static void test_home_assistant_offline_is_ignored(void) {
    FakeWifiRadio wifi;
    wifi.connected_ = true;
    FakeLoRa loRa;
    FakeMqtt mqtt;
    FakeStore store("kitchen");
    FakeDisplay display;
    FakeClock clock;

    gateway::GatewayOrchestrator orchestrator(wifi, loRa, mqtt, store, display, clock,
                                               "LoRaGateway", "lora/incoming");
    orchestrator.begin();

    orchestrator.tick();
    loRa.push(kKitchenSuccessPayload);
    orchestrator.tick();

    mqtt.simulateIncomingMessage("homeassistant/status", "offline");
    TEST_ASSERT_FALSE(orchestrator.takeHomeAssistantRestart());

    mqtt.published.clear();
    loRa.push(kKitchenSuccessPayloadSeq11);
    orchestrator.tick();
    TEST_ASSERT_EQUAL_UINT32(1, mqtt.published.size()); // state only
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_happy_path_publishes_discovery_and_state);
    RUN_TEST(test_state_message_is_retained_to_avoid_ha_discovery_race);
    RUN_TEST(test_tick_drains_all_queued_lora_packets_in_one_call);
    RUN_TEST(test_tick_caps_packets_drained_per_call);
    RUN_TEST(test_forwarded_packet_shows_curated_summary_not_raw_json);
    RUN_TEST(test_forwarded_summary_flags_low_battery_and_dht_error);
    RUN_TEST(test_packet_forwarded_callback_receives_full_json_payload);
    RUN_TEST(test_packet_forwarded_callback_does_not_fire_for_pending_node);
    RUN_TEST(test_unknown_node_is_pending_and_not_published);
    RUN_TEST(test_approve_persists_and_clears_pending);
    RUN_TEST(test_remove_node_clears_retained_state_message);

    RUN_TEST(test_duplicate_seq_is_counted_but_not_published);
    RUN_TEST(test_lower_seq_than_last_seen_is_treated_as_duplicate);
    RUN_TEST(test_higher_seq_is_published_normally);
    RUN_TEST(test_reboot_resets_dedup_even_with_lower_seq);
    RUN_TEST(test_reflash_with_lower_boot_count_is_not_treated_as_duplicate);
    RUN_TEST(test_lower_boot_count_other_than_one_is_still_treated_as_duplicate);
    RUN_TEST(test_dedup_is_bypassed_when_payload_has_no_seq_field);
    RUN_TEST(test_duplicate_is_not_shown_on_display_or_logged);

    RUN_TEST(test_cold_boot_sw_version_is_included_in_initial_discovery);
    RUN_TEST(test_new_sw_version_triggers_discovery_republish);
    RUN_TEST(test_subsequent_packets_without_sw_do_not_erase_stored_version);
    RUN_TEST(test_same_sw_version_again_does_not_retrigger_discovery);
    RUN_TEST(test_learned_sw_version_is_persisted_and_survives_restart);
    RUN_TEST(test_sw_reported_before_approval_is_not_lost);
    RUN_TEST(test_sw_change_during_in_flight_discovery_is_applied_once_batch_completes);
    RUN_TEST(test_node_name_republishes_discovery_with_name);
    RUN_TEST(test_node_name_cleared_goes_back_to_id);
    RUN_TEST(test_node_name_rejected_for_unapproved_node);
    RUN_TEST(test_node_name_before_first_packet_is_used_by_first_discovery);
    RUN_TEST(test_node_name_during_in_flight_discovery_is_applied_once_batch_completes);
    RUN_TEST(test_remove_node_forgets_its_name);

    RUN_TEST(test_offline_packets_are_queued_not_lost);
    RUN_TEST(test_two_new_nodes_fit_in_offline_queue);
    RUN_TEST(test_reconnect_backoff_is_respected_then_flushes_queue);
    RUN_TEST(test_malformed_packet_is_queued_to_base_topic_when_offline);
    RUN_TEST(test_state_message_carries_packet_snr);
    RUN_TEST(test_state_message_loss_reflects_seq_gap);
    RUN_TEST(test_duplicate_does_not_change_loss);
    RUN_TEST(test_cold_boot_resets_loss);
    RUN_TEST(test_binary_frame_publishes_same_state_as_json);
    RUN_TEST(test_malformed_binary_frame_forwarded_as_hex);

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

    RUN_TEST(test_messages_are_queued_while_wifi_down_even_if_mqtt_reports_connected);
    RUN_TEST(test_stale_mqtt_session_is_closed_once_when_wifi_drops);
    RUN_TEST(test_online_requires_both_wifi_and_mqtt);
    RUN_TEST(test_wifi_reconnect_backoff_doubles_up_to_one_minute);
    RUN_TEST(test_every_third_wifi_attempt_is_a_hard_reset);
    RUN_TEST(test_wifi_backoff_and_attempt_count_reset_after_reconnect);
    RUN_TEST(test_restart_requested_after_ten_minutes_without_wifi);
    RUN_TEST(test_offline_timer_resets_when_wifi_returns);
    RUN_TEST(test_wifi_recovery_paused_while_setup_portal_is_open);
    RUN_TEST(test_wifi_reconnect_attempts_are_counted_for_diagnostics);
    RUN_TEST(test_mqtt_restart_command_reason);

    RUN_TEST(test_subscribes_to_home_assistant_status);
    RUN_TEST(test_home_assistant_online_triggers_rediscovery);
    RUN_TEST(test_home_assistant_offline_is_ignored);

    return UNITY_END();
}
