#pragma once

// GatewayOrchestrator ties the Phase 1 pure functions (payload_parser.h)
// to the Phase 2 HAL interfaces (hal.h): it is the one place that knows
// "a LoRa packet came in, what do we do about it", and is driven entirely
// through interfaces so it can be unit tested natively with fakes (see
// test/test_orchestrator). It never touches Arduino, LoRa, PubSubClient,
// Preferences, WiFi, or a display directly.

#include <cstddef>
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
    static constexpr unsigned long kDefaultMqttReconnectBackoffMs = 5000;
    static constexpr unsigned long kDefaultWifiReconnectBackoffMs = 10000;

    GatewayOrchestrator(IWifiRadio& wifi, ILoRaReceiver& loRa, IMqttClient& mqtt, INodeStore& store,
                        IDisplay& display, IClock& clock, std::string deviceName,
                        std::string baseTopic,
                        unsigned long mqttReconnectBackoffMs = kDefaultMqttReconnectBackoffMs,
                        unsigned long wifiReconnectBackoffMs = kDefaultWifiReconnectBackoffMs);

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

private:
    void attemptWifiReconnect();
    void attemptMqttReconnect();
    void flushQueue();
    void ingestLoRaPacket();
    void enqueueOrPublish(const MqttMessage& msg, bool retain, const std::string& discoveryNodeId = "");
    void onMessagePublished(const std::string& discoveryNodeId);
    void onMessageDropped(const std::string& discoveryNodeId);
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

    AllowList allowList_;
    std::set<std::string> pendingNodes_;
    std::set<std::string> discoveredNodes_;
    // Node id -> number of its discovery messages not yet confirmed
    // published. Absent from both this map and discoveredNodes_ means
    // discovery has never been attempted (or was abandoned after an
    // eviction) and should be retried on the node's next packet.
    std::map<std::string, size_t> pendingDiscoveryCount_;

    std::deque<QueuedMessage> outboundQueue_;

    unsigned long packetsReceived_ = 0;
    unsigned long droppedMessages_ = 0;

    bool hasAttemptedMqttReconnect_ = false;
    unsigned long lastMqttReconnectAttemptMs_ = 0;

    bool hasAttemptedWifiReconnect_ = false;
    unsigned long lastWifiReconnectAttemptMs_ = 0;
};

} // namespace gateway
