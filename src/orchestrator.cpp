#include "orchestrator.h"

namespace gateway {

GatewayOrchestrator::GatewayOrchestrator(ILoRaReceiver& loRa, IMqttClient& mqtt, INodeStore& store,
                                          IDisplay& display, IClock& clock, std::string deviceName,
                                          std::string baseTopic, unsigned long reconnectBackoffMs)
    : loRa_(loRa),
      mqtt_(mqtt),
      store_(store),
      display_(display),
      clock_(clock),
      deviceName_(std::move(deviceName)),
      baseTopic_(std::move(baseTopic)),
      reconnectBackoffMs_(reconnectBackoffMs) {}

void GatewayOrchestrator::begin() {
    allowList_ = AllowList(store_.loadAllowListCsv());
}

void GatewayOrchestrator::setIdentity(const std::string& deviceName, const std::string& baseTopic) {
    deviceName_ = deviceName;
    baseTopic_ = baseTopic;
}

void GatewayOrchestrator::clearDiscoveredNodes() {
    discoveredNodes_.clear();
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
    return true;
}

std::vector<std::string> GatewayOrchestrator::pendingNodeIds() const {
    return std::vector<std::string>(pendingNodes_.begin(), pendingNodes_.end());
}

GatewayIdentity GatewayOrchestrator::identity() const {
    return GatewayIdentity{deviceName_, baseTopic_};
}

void GatewayOrchestrator::tick(bool wifiConnected) {
    if (wifiConnected) {
        if (mqtt_.connected()) {
            mqtt_.loop();
            flushQueue();
        } else {
            attemptReconnect();
        }
    }
    ingestLoRaPacket();
}

void GatewayOrchestrator::attemptReconnect() {
    unsigned long now = clock_.millis();
    if (hasAttemptedReconnect_ && (now - lastReconnectAttemptMs_) < reconnectBackoffMs_) {
        return;
    }
    hasAttemptedReconnect_ = true;
    lastReconnectAttemptMs_ = now;

    display_.showLines({"MQTT Reconnecting..."});
    if (mqtt_.connect()) {
        display_.showLines({"MQTT Connected!"});
        flushQueue();
    }
}

void GatewayOrchestrator::flushQueue() {
    while (!outboundQueue_.empty()) {
        const QueuedMessage& front = outboundQueue_.front();
        if (!mqtt_.publish(front.topic, front.payload, front.retain)) {
            break; // leave the rest queued, retry next tick
        }
        outboundQueue_.pop_front();
    }
}

void GatewayOrchestrator::enqueueOrPublish(const MqttMessage& msg, bool retain) {
    if (mqtt_.connected() && mqtt_.publish(msg.topic, msg.payload, retain)) {
        return;
    }
    outboundQueue_.push_back(QueuedMessage{msg.topic, msg.payload, retain});
    while (outboundQueue_.size() > kMaxQueuedMessages) {
        outboundQueue_.pop_front(); // drop oldest to make room for the newest
    }
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

    if (discoveredNodes_.find(reading.id) == discoveredNodes_.end()) {
        for (const auto& msg : buildAutoDiscoveryMessages(reading.id, identity())) {
            enqueueOrPublish(msg, true);
        }
        discoveredNodes_.insert(reading.id);
    }

    MqttMessage stateMsg = buildSensorStateMessage(reading, packet.rssi, route.topic);
    enqueueOrPublish(stateMsg, false);
    display_.showLines({"Fwd: " + stateMsg.topic, stateMsg.payload});
}

} // namespace gateway
