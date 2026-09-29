#include "orchestrator.h"

#include <cstdio>

namespace gateway {

namespace {

std::string formatOneDecimal(float value) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.1f", static_cast<double>(value));
    return std::string(buf);
}

// Builds a short, human-readable summary of a forwarded reading for the
// OLED (3 short lines instead of the raw JSON payload, which either wraps
// illegibly across the screen or gets truncated to something meaningless --
// see Esp32Display::showLines()). The full JSON is still available via
// PacketLogCallback / MQTT for anyone who needs every field.
std::vector<std::string> buildForwardedSummary(const SensorReading& reading, const std::string& topic) {
    std::string tempStr = reading.temperatureC.has_value() ? formatOneDecimal(*reading.temperatureC) + "C" : "--";
    std::string humidityStr =
        reading.humidityPct.has_value() ? formatOneDecimal(*reading.humidityPct) + "%" : "--";
    std::string voltageStr =
        reading.batteryVoltage.has_value() ? formatOneDecimal(*reading.batteryVoltage) + "V" : "--";

    std::string line3 = "V: " + voltageStr;
    if (reading.lowBattery) line3 += " LOW";
    if (reading.err != SensorError::None) line3 += " ERR:" + reading.rawErr;

    return {
        "Fwd: " + topic,
        "T: " + tempStr + " H: " + humidityStr,
        line3,
    };
}

} // namespace

GatewayOrchestrator::GatewayOrchestrator(IWifiRadio& wifi, ILoRaReceiver& loRa, IMqttClient& mqtt,
                                          INodeStore& store, IDisplay& display, IClock& clock,
                                          std::string deviceName, std::string baseTopic,
                                          unsigned long mqttReconnectBackoffMs,
                                          unsigned long wifiReconnectBackoffMs,
                                          PacketLogCallback onPacketForwarded)
    : wifi_(wifi),
      loRa_(loRa),
      mqtt_(mqtt),
      store_(store),
      display_(display),
      clock_(clock),
      deviceName_(std::move(deviceName)),
      baseTopic_(std::move(baseTopic)),
      mqttReconnectBackoffMs_(mqttReconnectBackoffMs),
      wifiReconnectBackoffMs_(wifiReconnectBackoffMs),
      onPacketForwarded_(onPacketForwarded) {}

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
    // A removed id can later be re-approved for a physically different node
    // (a relabeled or replacement device); it should get a clean discovery
    // rather than inheriting whatever firmware version the old device last
    // reported under this id.
    swVersionByNode_.erase(id);
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
            // Retried every tick until it succeeds, not just once right
            // after connect(): the SUBSCRIBE packet write itself can fail
            // (e.g. a socket drop mid-write) without necessarily taking the
            // connection down, in which case connected() stays true but
            // commands would otherwise go unheard until some later,
            // unrelated disconnect/reconnect forced a retry.
            if (!commandsSubscribed_) {
                commandsSubscribed_ = subscribeToCommands();
            }
            flushQueue();
        } else {
            commandsSubscribed_ = false;
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
        // Also retried every tick while connected (see tick()) in case this
        // particular attempt fails without the connection itself dropping.
        commandsSubscribed_ = subscribeToCommands();
        flushQueue();
    }
}

bool GatewayOrchestrator::subscribeToCommands() {
    return mqtt_.subscribe(
        gatewayCommandTopic(baseTopic_, "restart"), [this](const std::string& topic, const std::string& payload) {
            if (payload != kRestartCommandPayload) return;
            // Clear any retained message on this topic before acting on it:
            // without this, a stale retained "PRESS" -- left by a manual
            // `mosquitto_pub -r`, a misconfigured automation, or even a
            // button whose own retain setting gets changed -- would be
            // redelivered by the broker on every future subscribe (i.e.
            // every MQTT reconnect), causing an infinite reboot loop that
            // only clearing the broker's retained message by hand could
            // break.
            mqtt_.publish(topic, "", true);
            restartRequested_ = true;
        });
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
    // Drains up to kMaxPacketsPerTick packets currently buffered by the HAL
    // per call, not just one, so a burst that arrived while tick() was busy
    // elsewhere (e.g. blocked in MQTT I/O) doesn't trickle out one packet
    // per subsequent loop() iteration. Bounded by a fixed budget rather than
    // "keep going until receive() reports empty": the HAL's producer (an
    // ISR on the ESP32 build) can keep refilling its own buffer for as long
    // as packets keep arriving, so an unbounded loop has no guaranteed
    // termination if arrivals keep pace with draining -- see
    // kMaxPacketsPerTick's comment.
    RawPacket packet;
    for (size_t i = 0; i < kMaxPacketsPerTick && loRa_.receive(packet); ++i) {
        ingestOnePacket(packet);
    }
}

void GatewayOrchestrator::ingestOnePacket(const RawPacket& packet) {
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
    bool isFirstDiscovery = discoveredNodes_.find(reading.id) == discoveredNodes_.end() && !discoveryInFlight;

    // "sw" only arrives on a node's cold-boot packet (see
    // SensorReading::swVersion); most packets carry no "sw" at all, and that
    // must leave whatever's already known untouched -- an absent optional
    // here means "nothing new to report", never "clear it". Only a value
    // that's actually different from what's stored (a real flash/battery-swap
    // cold boot, not a retransmitted cold-boot packet) is worth republishing
    // discovery over.
    auto swIt = swVersionByNode_.find(reading.id);
    std::string knownSw = swIt != swVersionByNode_.end() ? swIt->second : "";
    bool swChanged = reading.swVersion.has_value() && *reading.swVersion != knownSw;
    if (swChanged) {
        swVersionByNode_[reading.id] = *reading.swVersion;
        knownSw = *reading.swVersion;
    }

    // Rebuild and (re-)publish this node's full discovery set on first sight,
    // or when its known "sw" just changed -- but not both at once, and not
    // while a previous discovery attempt for this node is still draining out
    // of the queue: overwriting pendingDiscoveryCount_[reading.id] mid-flight
    // would desync it from the enqueued-but-not-yet-published messages still
    // counting down against the old value (see onMessagePublished()).
    if (isFirstDiscovery || (swChanged && !discoveryInFlight)) {
        auto discoveryMsgs = buildAutoDiscoveryMessages(reading.id, identity(), knownSw);
        pendingDiscoveryCount_[reading.id] = discoveryMsgs.size();
        for (const auto& msg : discoveryMsgs) {
            enqueueOrPublish(msg, true, reading.id);
        }
    }

    // Deduplicate retransmissions: track the highest (bootCount, seq)
    // accepted per node and drop anything that isn't strictly newer, except
    // that a higher bootCount (a reboot) always resets the check, since a
    // rebooted node's own seq counter restarts at 0 and would otherwise look
    // like an endless stream of "already seen" values.
    //
    // Skipped entirely when the payload has no "seq" (reading.hasSeq is
    // false): bootCount/seq then just default to 0 on every single packet
    // (see SensorReading::hasSeq), which would otherwise make every packet
    // after this node's first look like a duplicate of (0, 0) forever.
    bool isDuplicate = false;
    if (reading.hasSeq) {
        auto seenIt = lastSeenByNode_.find(reading.id);
        if (seenIt == lastSeenByNode_.end()) {
            lastSeenByNode_[reading.id] = LastSeen{reading.bootCount, reading.seq};
        } else if (reading.bootCount > seenIt->second.bootCount) {
            seenIt->second = LastSeen{reading.bootCount, reading.seq};
        } else if (reading.bootCount == seenIt->second.bootCount && reading.seq > seenIt->second.seq) {
            seenIt->second.seq = reading.seq;
        } else {
            // Same-or-earlier seq within the same boot (a retransmission), or
            // a bootCount that went backwards (untrusted input over an
            // unauthenticated LoRa link) -- either way, already seen.
            isDuplicate = true;
        }
    }
    if (isDuplicate) return;

    MqttMessage stateMsg = buildSensorStateMessage(reading, packet.rssi, route.topic);
    if (onPacketForwarded_) onPacketForwarded_(stateMsg.topic, stateMsg.payload);
    enqueueOrPublish(stateMsg, false);
    display_.showLines(buildForwardedSummary(reading, route.topic));
}

} // namespace gateway
