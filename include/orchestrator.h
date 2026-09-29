#pragma once

// GatewayOrchestrator ties the Phase 1 pure functions (payload_parser.h)
// to the Phase 2 HAL interfaces (hal.h): it is the one place that knows
// "a LoRa packet came in, what do we do about it", and is driven entirely
// through interfaces so it can be unit tested natively with fakes (see
// test/test_orchestrator). It never touches Arduino, LoRa, PubSubClient,
// Preferences, WiFi, or a display directly.

#include <cstddef>
#include <cstdint>
#include <deque>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "hal.h"
#include "payload_parser.h"

namespace gateway {

struct QueuedMessage {
    std::string topic;
    std::string payload;
    bool retain = false;
    // Non-empty when this message is one of a node's auto-discovery configs;
    // used to track discovery completion (see pendingDiscoveryCount_) so a
    // node is only marked discovered once every one of its discovery
    // messages has actually reached MQTT, not merely been enqueued.
    std::string discoveryNodeId;
};

class GatewayOrchestrator {
public:
    // Bound on the store-and-forward queue used while MQTT is unreachable.
    // Beyond this, the oldest queued message is dropped to make room for
    // the newest one (see enqueueOrPublish()), and droppedMessageCount()
    // increments.
    static constexpr size_t kMaxQueuedMessages = 20;

    // Bound on how many LoRa packets a single tick() will drain from the HAL
    // (see ingestLoRaPacket()). The LoRa link is unauthenticated (a node id
    // is treated as attacker-controlled elsewhere in this codebase, e.g.
    // sanitizeMqttTopicSegment()), and the HAL's own producer (an ISR on the
    // ESP32 build) can keep refilling its buffer for as long as packets keep
    // arriving -- so draining "until the buffer reports empty" has no
    // guaranteed termination if arrivals keep pace with draining. This fixed
    // budget guarantees tick() always returns in bounded time regardless of
    // concurrent arrivals, so WiFi/OTA housekeeping and the watchdog reset
    // (both once per outer loop() iteration) are never starved.
    static constexpr size_t kMaxPacketsPerTick = 20;

    static constexpr unsigned long kDefaultMqttReconnectBackoffMs = 5000;
    static constexpr unsigned long kDefaultWifiReconnectBackoffMs = 10000;

    // Invoked with (topic, payload) whenever a LoRa packet is successfully
    // parsed and routed to an approved node, regardless of whether MQTT is
    // currently reachable. A plain function pointer, like IDisplay's
    // ActivityCallback in hal_esp32.h, so the orchestrator stays free of any
    // Serial/logging dependency; main.cpp wires this to Serial.println() so
    // the full JSON payload is still visible over USB even though the OLED
    // (see buildForwardedSummary() in orchestrator.cpp) only shows a short,
    // human-readable summary.
    using PacketLogCallback = void (*)(const std::string& topic, const std::string& payload);

    GatewayOrchestrator(IWifiRadio& wifi, ILoRaReceiver& loRa, IMqttClient& mqtt, INodeStore& store,
                        IDisplay& display, IClock& clock, std::string deviceName,
                        std::string baseTopic,
                        unsigned long mqttReconnectBackoffMs = kDefaultMqttReconnectBackoffMs,
                        unsigned long wifiReconnectBackoffMs = kDefaultWifiReconnectBackoffMs,
                        PacketLogCallback onPacketForwarded = nullptr);

    // Loads the persisted allowlist from `store`. Call once during setup,
    // after the store itself is ready to be read from.
    void begin();

    // Drives one iteration: WiFi connection housekeeping (reconnect with
    // backoff when down), MQTT connection housekeeping (loop()/reconnect,
    // only attempted while WiFi is up), and LoRa packet ingestion (always,
    // so packets are captured — and queued — even while fully offline).
    void tick();

    void setIdentity(const std::string& deviceName, const std::string& baseTopic);
    void clearDiscoveredNodes();

    // Returns true if the allowlist actually changed.
    bool approveNode(const std::string& id);
    bool removeNode(const std::string& id);

    const AllowList& allowList() const { return allowList_; }
    std::vector<std::string> pendingNodeIds() const;

    unsigned long packetsReceived() const { return packetsReceived_; }
    size_t queuedMessageCount() const { return outboundQueue_.size(); }
    const std::deque<QueuedMessage>& pendingQueue() const { return outboundQueue_; }

    // Messages permanently discarded because the store-and-forward queue
    // was already at kMaxQueuedMessages when a new one needed to be queued.
    unsigned long droppedMessageCount() const { return droppedMessages_; }

    // True once a valid restart command has arrived on this gateway's
    // restart command topic (see gatewayCommandTopic() and
    // buildGatewayCommandDiscoveryMessages()'s restart button). The
    // orchestrator only sets the flag -- actually restarting the hardware is
    // main.cpp's job (ESP.restart()), since that's not something a
    // hardware-free, natively-tested class should do itself.
    bool restartRequested() const { return restartRequested_; }

private:
    void attemptWifiReconnect();
    void attemptMqttReconnect();
    bool subscribeToCommands();
    void flushQueue();
    void ingestLoRaPacket();
    void ingestOnePacket(const RawPacket& packet);
    void enqueueOrPublish(const MqttMessage& msg, bool retain, const std::string& discoveryNodeId = "");
    void onMessagePublished(const std::string& discoveryNodeId);
    void onMessageDropped(const std::string& discoveryNodeId);
    void publishNodeDiscovery(const std::string& nodeId, const std::string& swVersion);
    GatewayIdentity identity() const;

    IWifiRadio& wifi_;
    ILoRaReceiver& loRa_;
    IMqttClient& mqtt_;
    INodeStore& store_;
    IDisplay& display_;
    IClock& clock_;

    std::string deviceName_;
    std::string baseTopic_;
    unsigned long mqttReconnectBackoffMs_;
    unsigned long wifiReconnectBackoffMs_;
    PacketLogCallback onPacketForwarded_;

    AllowList allowList_;
    std::set<std::string> pendingNodes_;
    std::set<std::string> discoveredNodes_;
    // Node id -> number of its discovery messages not yet confirmed
    // published. Absent from both this map and discoveredNodes_ means
    // discovery has never been attempted (or was abandoned after an
    // eviction) and should be retried on the node's next packet.
    std::map<std::string, size_t> pendingDiscoveryCount_;

    // (bootCount, seq) of the highest reading actually accepted per node, so
    // a retransmission (there's no ack protocol, so a node can't tell
    // whether its last packet got through) can be dropped instead of
    // re-published/re-logged/re-displayed. See ingestLoRaPacket().
    struct LastSeen {
        uint32_t bootCount = 0;
        uint32_t seq = 0;
    };
    std::map<std::string, LastSeen> lastSeenByNode_;

    // Node id -> last "sw" value that node actually reported (see
    // SensorReading::swVersion). A node only sends "sw" on its cold-boot
    // packet to avoid wasting airtime/battery, so most packets carry no "sw"
    // at all -- that must leave whatever's here untouched, not erase it.
    // Absent from this map means "never reported", which discovery renders
    // as an explicit JSON null (see buildAutoDiscoveryMessages()). Persisted
    // via store_.save/loadNodeVersionsCsv() (Codex review on PR #23): purely
    // in-memory, a gateway restart would forget every version it had learned
    // and republish null for any node not due for another cold boot anytime
    // soon.
    std::map<std::string, std::string> swVersionByNode_;

    // Node ids whose known "sw" changed while their previous discovery batch
    // was still draining out of pendingDiscoveryCount_/outboundQueue_ (see
    // ingestOnePacket()). Republishing immediately would desync that count
    // from messages already in flight from the old batch, so the fresh
    // republish is deferred until onMessagePublished() sees that batch
    // actually finish -- without this, a version that changes again before
    // the first batch completes would be silently and permanently lost,
    // since a later packet reporting the same already-stored value never
    // looks "changed" again (Codex review on PR #23).
    std::set<std::string> pendingRediscovery_;

    std::deque<QueuedMessage> outboundQueue_;

    unsigned long packetsReceived_ = 0;
    unsigned long droppedMessages_ = 0;

    bool hasAttemptedMqttReconnect_ = false;
    unsigned long lastMqttReconnectAttemptMs_ = 0;

    bool hasAttemptedWifiReconnect_ = false;
    unsigned long lastWifiReconnectAttemptMs_ = 0;

    bool restartRequested_ = false;
    // Whether subscribeToCommands() has succeeded since the last connect();
    // reset to false on disconnect and retried every tick() while connected
    // until it succeeds (see tick()).
    bool commandsSubscribed_ = false;
};

} // namespace gateway
