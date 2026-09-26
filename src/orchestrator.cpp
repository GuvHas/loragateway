#include "orchestrator.h"

namespace gateway {

GatewayOrchestrator::GatewayOrchestrator(IWifiRadio& wifi, ILoRaReceiver& loRa, IMqttClient& mqtt,
                                          INodeStore& store, IDisplay& display, IClock& clock,
                                          std::string deviceName, std::string baseTopic,
                                          unsigned long mqttReconnectBackoffMs,
                                          unsigned long wifiReconnectBackoffMs)
    : wifi_(wifi),
      loRa_(loRa),
      mqtt_(mqtt),
      store_(store),
      display_(display),
      clock_(clock),
      deviceName_(std::move(deviceName)),
      baseTopic_(std::move(baseTopic)),
      mqttReconnectBackoffMs_(mqttReconnectBackoffMs),
      wifiReconnectBackoffMs_(wifiReconnectBackoffMs) {}

void GatewayOrchestrator::begin() {
    allowList_ = AllowList(store_.loadAllowListCsv());
}

void GatewayOrchestrator::setIdentity(const std::string& deviceName, const std::string& baseTopic) {
    deviceName_ = deviceName;
    baseTopic_ = baseTopic;
}

void GatewayOrchestrator::clearDiscoveredNodes() {
    discoveredNodes_.clear();
    pendingDiscoveryCount_.clear();
}

bool GatewayOrchestrator::approveNode(const std::string& id) {
    if (!allowList_.approve(id)) return false;
    store_.saveAllowListCsv(allowList_.toCsv());
    pendingNodes_.erase(id);
    return true;
}

bool GatewayOrchestrator::removeNode(const std::string& id) {
    if (!allowList_.remove(id)) return false;
    store_.saveAllowListCsv(allowList_.toCsv());
    discoveredNodes_.erase(id);
    pendingDiscoveryCount_.erase(id);
    return true;
}

std::vector<std::string> GatewayOrchestrator::pendingNodeIds() const {
    return std::vector<std::string>(pendingNodes_.begin(), pendingNodes_.end());
}

GatewayIdentity GatewayOrchestrator::identity() const {
    return GatewayIdentity{deviceName_, baseTopic_};
}

void GatewayOrchestrator::tick() {
    if (wifi_.connected()) {
        if (mqtt_.connected()) {
            mqtt_.loop();
            flushQueue();
        } else {
            attemptMqttReconnect();
        }
    } else {
        attemptWifiReconnect();
    }
    ingestLoRaPacket();
}

void GatewayOrchestrator::attemptWifiReconnect() {
    unsigned long now = clock_.millis();
    if (hasAttemptedWifiReconnect_ && (now - lastWifiReconnectAttemptMs_) < wifiReconnectBackoffMs_) {
        return;
    }
    hasAttemptedWifiReconnect_ = true;
    lastWifiReconnectAttemptMs_ = now;

    display_.showLines({"WiFi Reconnecting..."});
    wifi_.reconnect(); // non-blocking; connected() reflects the outcome on a later tick()
}

void GatewayOrchestrator::attemptMqttReconnect() {
    unsigned long now = clock_.millis();
    if (hasAttemptedMqttReconnect_ && (now - lastMqttReconnectAttemptMs_) < mqttReconnectBackoffMs_) {
        return;
    }
    hasAttemptedMqttReconnect_ = true;
    lastMqttReconnectAttemptMs_ = now;

    display_.showLines({"MQTT Reconnecting..."});
    if (mqtt_.connect()) {
        display_.showLines({"MQTT Connected!"});
        flushQueue();
    }
}

void GatewayOrchestrator::flushQueue() {
    while (!outboundQueue_.empty()) {
        const QueuedMessage front = outboundQueue_.front();
        if (!mqtt_.publish(front.topic, front.payload, front.retain)) {
            break; // leave the rest queued, retry next tick
        }
        outboundQueue_.pop_front();
        onMessagePublished(front.discoveryNodeId);
    }
}

void GatewayOrchestrator::enqueueOrPublish(const MqttMessage& msg, bool retain,
                                            const std::string& discoveryNodeId) {
    if (mqtt_.connected() && mqtt_.publish(msg.topic, msg.payload, retain)) {
        onMessagePublished(discoveryNodeId);
        return;
    }
    outboundQueue_.push_back(QueuedMessage{msg.topic, msg.payload, retain, discoveryNodeId});
    while (outboundQueue_.size() > kMaxQueuedMessages) {
        onMessageDropped(outboundQueue_.front().discoveryNodeId);
        outboundQueue_.pop_front(); // drop oldest to make room for the newest
        droppedMessages_++;
    }
}

void GatewayOrchestrator::onMessagePublished(const std::string& discoveryNodeId) {
    if (discoveryNodeId.empty()) return;
    auto it = pendingDiscoveryCount_.find(discoveryNodeId);
    if (it == pendingDiscoveryCount_.end()) return;
    if (--(it->second) == 0) {
        discoveredNodes_.insert(discoveryNodeId);
        pendingDiscoveryCount_.erase(it);
    }
}

void GatewayOrchestrator::onMessageDropped(const std::string& discoveryNodeId) {
    if (discoveryNodeId.empty()) return;
    // Any one of a node's discovery messages being evicted means Home
    // Assistant will never see the complete set; abandon the attempt so the
    // node's next packet retries discovery from scratch instead of being
    // marked discovered without ever actually delivering all the configs.
    pendingDiscoveryCount_.erase(discoveryNodeId);
}

void GatewayOrchestrator::ingestLoRaPacket() {
    RawPacket packet;
    if (!loRa_.receive(packet)) return;

    packetsReceived_++;

    ParseResult parsed = parseSensorPayload(packet.data);
    if (!parsed.ok()) {
        // Malformed/truncated JSON, or a payload that failed strict field
        // validation: forward the raw bytes to the base topic rather than
        // dropping the packet or crashing (still subject to store-and-forward).
        enqueueOrPublish(MqttMessage{baseTopic_, packet.data}, false);
        return;
    }

    const SensorReading& reading = parsed.reading;
    RoutingResult route = decideRoute(reading.id, baseTopic_, allowList_);

    if (route.decision == RouteDecision::Pending) {
        pendingNodes_.insert(reading.id);
        display_.showLines({"New device: " + reading.id, "Approve via /devices"});
        return;
    }

    bool discoveryInFlight = pendingDiscoveryCount_.find(reading.id) != pendingDiscoveryCount_.end();
    if (discoveredNodes_.find(reading.id) == discoveredNodes_.end() && !discoveryInFlight) {
        auto discoveryMsgs = buildAutoDiscoveryMessages(reading.id, identity());
        pendingDiscoveryCount_[reading.id] = discoveryMsgs.size();
        for (const auto& msg : discoveryMsgs) {
            enqueueOrPublish(msg, true, reading.id);
        }
    }

    MqttMessage stateMsg = buildSensorStateMessage(reading, packet.rssi, route.topic);
    enqueueOrPublish(stateMsg, false);
    display_.showLines({"Fwd: " + stateMsg.topic, stateMsg.payload});
}

} // namespace gateway
