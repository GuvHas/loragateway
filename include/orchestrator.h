#pragma once

// GatewayOrchestrator ties the Phase 1 pure functions (payload_parser.h)
// to the Phase 2 HAL interfaces (hal.h): it is the one place that knows
// "a LoRa packet came in, what do we do about it", and is driven entirely
// through interfaces so it can be unit tested natively with fakes (see
// test/test_orchestrator). It never touches Arduino, LoRa, PubSubClient,
// Preferences, or a display directly.

#include <cstddef>
#include <deque>
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
};

class GatewayOrchestrator {
public:
    // Bound on the store-and-forward queue used while MQTT is unreachable.
    // Beyond this, the oldest queued message is dropped to make room for
    // the newest one (see enqueueOrPublish()).
    static constexpr size_t kMaxQueuedMessages = 20;
    static constexpr unsigned long kDefaultReconnectBackoffMs = 5000;

    GatewayOrchestrator(ILoRaReceiver& loRa, IMqttClient& mqtt, INodeStore& store,
                        IDisplay& display, IClock& clock, std::string deviceName,
                        std::string baseTopic,
                        unsigned long reconnectBackoffMs = kDefaultReconnectBackoffMs);

    // Loads the persisted allowlist from `store`. Call once during setup,
    // after the store itself is ready to be read from.
    void begin();

    // Drives one iteration. MQTT connection housekeeping (loop()/reconnect)
    // only runs when `wifiConnected` is true, since attempting an MQTT
    // connect without a network is pointless; LoRa packet ingestion always
    // runs, so packets are captured (and queued) even while offline.
    void tick(bool wifiConnected);

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

private:
    void attemptReconnect();
    void flushQueue();
    void ingestLoRaPacket();
    void enqueueOrPublish(const MqttMessage& msg, bool retain);
    GatewayIdentity identity() const;

    ILoRaReceiver& loRa_;
    IMqttClient& mqtt_;
    INodeStore& store_;
    IDisplay& display_;
    IClock& clock_;

    std::string deviceName_;
    std::string baseTopic_;
    unsigned long reconnectBackoffMs_;

    AllowList allowList_;
    std::set<std::string> pendingNodes_;
    std::set<std::string> discoveredNodes_;

    std::deque<QueuedMessage> outboundQueue_;

    unsigned long packetsReceived_ = 0;
    bool hasAttemptedReconnect_ = false;
    unsigned long lastReconnectAttemptMs_ = 0;
};

} // namespace gateway
