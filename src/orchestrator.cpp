#include "orchestrator.h"

#include <cctype>
#include <cstdio>

namespace gateway {

namespace {

// Mirrors decideRoute()'s own lowercasing of a sanitized node id into its
// state topic segment (payload_parser.cpp keeps its own toLower() private to
// that translation unit), so removeNode() can reconstruct a node's exact
// state topic without a RawPacket/decideRoute() call.
std::string toLowerAscii(const std::string& s) {
    std::string out = s;
    for (char& c : out) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return out;
}

const char* const kHomeAssistantStatusTopic = "homeassistant/status";

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

// Serializes/parses swVersionByNode_ for INodeStore::save/loadNodeVersionsCsv(),
// and nameByNode_ for save/loadNodeNamesCsv() (sanitizeNodeName() strips ','
// and '=' from names for this reason; see parseNodeNames()).
// "id=version" pairs joined by ',', mirroring AllowList's own CSV format and
// its same pragmatic assumption that neither a (sanitized) node id nor a git
// short hash contains '=' or ',' -- this is gateway-internal persistence, not
// something an attacker controls independently of what's already trusted
// elsewhere (a node id reaching this point has already passed
// sanitizeMqttTopicSegment()).
std::string serializeNodeVersions(const std::map<std::string, std::string>& versions) {
    std::string out;
    for (const auto& entry : versions) {
        if (!out.empty()) out += ",";
        out += entry.first + "=" + entry.second;
    }
    return out;
}

std::map<std::string, std::string> parseNodeVersions(const std::string& csv) {
    std::map<std::string, std::string> out;
    size_t start = 0;
    while (start <= csv.size()) {
        size_t comma = csv.find(',', start);
        if (comma == std::string::npos) comma = csv.size();
        std::string entry = csv.substr(start, comma - start);
        size_t eq = entry.find('=');
        if (eq != std::string::npos) {
            std::string id = entry.substr(0, eq);
            std::string version = entry.substr(eq + 1);
            if (!id.empty() && !version.empty()) out[id] = version;
        }
        start = comma + 1;
    }
    return out;
}

// Parses nameByNode_'s "id=name" CSV. Splits each entry at its *last* '=':
// sanitizeNodeName() never lets a name contain '=', but a node id may
// (sanitizeMqttTopicSegment() allows it), so splitting at the first '=' would
// file "sensor=1=Garage" under "sensor" after a restart.
std::map<std::string, std::string> parseNodeNames(const std::string& csv) {
    std::map<std::string, std::string> out;
    size_t start = 0;
    while (start <= csv.size()) {
        size_t comma = csv.find(',', start);
        if (comma == std::string::npos) comma = csv.size();
        std::string entry = csv.substr(start, comma - start);
        size_t eq = entry.rfind('=');
        if (eq != std::string::npos) {
            std::string id = entry.substr(0, eq);
            std::string name = entry.substr(eq + 1);
            if (!id.empty() && !name.empty()) out[id] = name;
        }
        start = comma + 1;
    }
    return out;
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
    swVersionByNode_ = parseNodeVersions(store_.loadNodeVersionsCsv());
    nameByNode_ = parseNodeNames(store_.loadNodeNamesCsv());
}

void GatewayOrchestrator::setIdentity(const std::string& deviceName, const std::string& baseTopic) {
    deviceName_ = deviceName;
    baseTopic_ = baseTopic;
}

void GatewayOrchestrator::clearDiscoveredNodes() {
    discoveredNodes_.clear();
    pendingDiscoveryCount_.clear();
    // Any batch pendingRediscovery_ was waiting on just got wiped above, so
    // the next packet from these nodes will already take the isFirstDiscovery
    // path in ingestOnePacket() with whatever's current in swVersionByNode_ --
    // leaving a stale entry here would only cause a harmless but wasteful
    // duplicate republish once that fresh batch completes.
    pendingRediscovery_.clear();
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
    pendingRediscovery_.erase(id);
    // A removed id can later be re-approved for a physically different node
    // (a relabeled or replacement device); it should get a clean discovery
    // rather than inheriting whatever firmware version the old device last
    // reported under this id.
    swVersionByNode_.erase(id);
    lossByNode_.erase(id);
    store_.saveNodeVersionsCsv(serializeNodeVersions(swVersionByNode_));
    if (nameByNode_.erase(id) > 0) store_.saveNodeNamesCsv(serializeNodeVersions(nameByNode_));

    // Clear this node's retained last-known reading (Codex review on PR
    // #25): now that state messages are retained -- to close a Home
    // Assistant discovery race, see ingestOnePacket() -- a removed node's
    // last reading would otherwise stay permanently served to any future
    // subscriber (including a re-approved node that reuses this exact id,
    // or Home Assistant simply reconnecting) looking exactly like live
    // current data. An empty retained payload on the same topic is the
    // standard MQTT idiom for asking the broker to delete a retained
    // message.
    enqueueOrPublish(MqttMessage{baseTopic_ + "/" + toLowerAscii(id), ""}, true);
    return true;
}

bool GatewayOrchestrator::setNodeName(const std::string& id, const std::string& name) {
    if (!allowList_.isAllowed(id)) return false;
    std::string clean = sanitizeNodeName(name);
    // A name equal to the id is the same as no name.
    if (clean == id) clean.clear();
    if (clean == nodeName(id)) return false;
    if (clean.empty()) {
        nameByNode_.erase(id);
    } else {
        nameByNode_[id] = clean;
    }
    store_.saveNodeNamesCsv(serializeNodeVersions(nameByNode_));
    requestRediscovery(id);
    return true;
}

std::string GatewayOrchestrator::nodeName(const std::string& id) const {
    auto it = nameByNode_.find(id);
    return it != nameByNode_.end() ? it->second : "";
}

std::vector<std::string> GatewayOrchestrator::pendingNodeIds() const {
    return std::vector<std::string>(pendingNodes_.begin(), pendingNodes_.end());
}

GatewayIdentity GatewayOrchestrator::identity() const {
    return GatewayIdentity{deviceName_, baseTopic_};
}

void GatewayOrchestrator::setWifiRecoveryEnabled(bool enabled) {
    if (enabled == wifiRecoveryEnabled_) return;
    wifiRecoveryEnabled_ = enabled;
    // Either way, the next outage tick starts a fresh outage: no restart
    // for time spent with the portal open, and an immediate first attempt
    // once it closes.
    wifiOutageActive_ = false;
}

bool GatewayOrchestrator::takeHomeAssistantRestart() {
    bool restarted = homeAssistantRestarted_;
    homeAssistantRestarted_ = false;
    return restarted;
}

void GatewayOrchestrator::requestRestart(const std::string& reason) {
    restartRequested_ = true;
    restartReason_ = reason;
}

void GatewayOrchestrator::tick() {
    if (wifi_.connected()) {
        wifiOutageActive_ = false;
        if (mqtt_.connected()) {
            mqtt_.loop();
            // Retried every tick until it succeeds, not just once right
            // after connect(): the SUBSCRIBE packet write itself can fail
            // (e.g. a socket drop mid-write) without necessarily taking the
            // connection down, in which case connected() stays true but
            // commands would otherwise go unheard until some later,
            // unrelated disconnect/reconnect forced a retry.
            if (!commandsSubscribed_ || !haStatusSubscribed_) {
                subscribeToCommands();
            }
            flushQueue();
        } else {
            commandsSubscribed_ = false;
            haStatusSubscribed_ = false;
            attemptMqttReconnect();
        }
    } else {
        handleWifiDown();
    }
    ingestLoRaPacket();
}

void GatewayOrchestrator::handleWifiDown() {
    // PubSubClient doesn't notice that WiFi went away: its socket stays
    // "connected" and every write into it can block for ~10 s once the TCP
    // send buffer fills, enough to trip the task watchdog during an AP
    // reboot. Close the stale session now; it reconnects normally once WiFi
    // is back instead of waiting for a keepalive timeout.
    if (mqtt_.connected()) {
        mqtt_.disconnect();
    }
    commandsSubscribed_ = false;
    haStatusSubscribed_ = false;

    if (!wifiRecoveryEnabled_) return;

    unsigned long now = clock_.millis();
    if (!wifiOutageActive_) {
        wifiOutageActive_ = true;
        wifiOutageStartMs_ = now;
        wifiAttemptsThisOutage_ = 0;
        hasAttemptedWifiReconnect_ = false;
        currentWifiBackoffMs_ = wifiReconnectBackoffMs_;
    }

    if (!restartRequested_ && (now - wifiOutageStartMs_) >= kWifiOfflineRestartMs) {
        requestRestart("WiFi offline 10 min");
        return;
    }

    attemptWifiReconnect();
}

void GatewayOrchestrator::attemptWifiReconnect() {
    unsigned long now = clock_.millis();
    if (hasAttemptedWifiReconnect_ && (now - lastWifiReconnectAttemptMs_) < currentWifiBackoffMs_) {
        return;
    }
    if (hasAttemptedWifiReconnect_) {
        // Double the gap after each failed attempt in this outage, capped.
        currentWifiBackoffMs_ = currentWifiBackoffMs_ >= kMaxWifiReconnectBackoffMs / 2
                                    ? kMaxWifiReconnectBackoffMs
                                    : currentWifiBackoffMs_ * 2;
    }
    hasAttemptedWifiReconnect_ = true;
    lastWifiReconnectAttemptMs_ = now;
    wifiAttemptsThisOutage_++;
    wifiReconnectAttemptsTotal_++;

    // non-blocking either way; connected() reflects the outcome on a later tick()
    if (wifiAttemptsThisOutage_ % kWifiHardResetEveryNAttempts == 0) {
        display_.showLines({"WiFi Reconnecting...", "(radio reset)"});
        wifi_.hardReset();
    } else {
        display_.showLines({"WiFi Reconnecting..."});
        wifi_.reconnect();
    }
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
        subscribeToCommands();
        flushQueue();
    }
}

bool GatewayOrchestrator::subscribeToCommands() {
    if (!commandsSubscribed_) {
        commandsSubscribed_ = mqtt_.subscribe(
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
                requestRestart("MQTT command");
            });
    }
    if (!haStatusSubscribed_) {
        // Home Assistant's birth message: "online" when it (re)starts. If the
        // broker lost its retained discovery configs meanwhile, HA would
        // otherwise never see this gateway's entities again until the
        // gateway itself rebooted. Node discovery is rebuilt on each node's
        // next packet; main.cpp resends the gateway's own discovery.
        haStatusSubscribed_ = mqtt_.subscribe(
            kHomeAssistantStatusTopic, [this](const std::string&, const std::string& payload) {
                if (payload != "online") return;
                clearDiscoveredNodes();
                homeAssistantRestarted_ = true;
            });
    }
    return commandsSubscribed_ && haStatusSubscribed_;
}

void GatewayOrchestrator::flushQueue() {
    while (!outboundQueue_.empty() && online()) {
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
    // online(), not mqtt_.connected(): see online()'s comment.
    if (online() && mqtt_.publish(msg.topic, msg.payload, retain)) {
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

        // A newer "sw" arrived while this batch was still draining out (see
        // ingestOnePacket()) and was deferred rather than dropped; it's safe
        // to reuse pendingDiscoveryCount_[discoveryNodeId] again now that
        // this batch is fully accounted for, so republish immediately
        // instead of waiting for a cold boot that might not happen again.
        auto dirtyIt = pendingRediscovery_.find(discoveryNodeId);
        if (dirtyIt != pendingRediscovery_.end()) {
            pendingRediscovery_.erase(dirtyIt);
            publishNodeDiscovery(discoveryNodeId);
        }
    }
}

void GatewayOrchestrator::onMessageDropped(const std::string& discoveryNodeId) {
    if (discoveryNodeId.empty()) return;
    // Any one of a node's discovery messages being evicted means Home
    // Assistant will never see the complete set; abandon the attempt so the
    // node's next packet retries discovery from scratch instead of being
    // marked discovered without ever actually delivering all the configs.
    pendingDiscoveryCount_.erase(discoveryNodeId);
    // That next packet's isFirstDiscovery path (see ingestOnePacket()) will
    // already rebuild with whatever's current in swVersionByNode_, so a
    // leftover dirty flag here would only cause a redundant duplicate
    // republish once that fresh batch completes.
    pendingRediscovery_.erase(discoveryNodeId);
}

void GatewayOrchestrator::requestRediscovery(const std::string& nodeId) {
    if (pendingDiscoveryCount_.find(nodeId) != pendingDiscoveryCount_.end()) {
        pendingRediscovery_.insert(nodeId);
    } else if (discoveredNodes_.find(nodeId) != discoveredNodes_.end()) {
        publishNodeDiscovery(nodeId);
    }
}

void GatewayOrchestrator::publishNodeDiscovery(const std::string& nodeId) {
    auto swIt = swVersionByNode_.find(nodeId);
    std::string swVersion = swIt != swVersionByNode_.end() ? swIt->second : "";
    auto discoveryMsgs = buildAutoDiscoveryMessages(nodeId, identity(), swVersion, nodeName(nodeId));
    pendingDiscoveryCount_[nodeId] = discoveryMsgs.size();
    for (const auto& msg : discoveryMsgs) {
        enqueueOrPublish(msg, true, nodeId);
    }
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

    ParseResult parsed = parseSensorPacket(packet.data);
    if (!parsed.ok()) {
        // Malformed/truncated JSON or binary frame, or a payload that failed
        // strict field validation: forward it to the base topic rather than
        // dropping the packet or crashing (still subject to store-and-forward).
        // A binary frame goes out as hex text; see unparsedPacketPayload().
        enqueueOrPublish(MqttMessage{baseTopic_, unparsedPacketPayload(packet.data)}, false);
        return;
    }

    const SensorReading& reading = parsed.reading;

    // Captured and persisted regardless of approval state, before the
    // Pending-node early return below: a brand new node's very first packet
    // is itself a cold boot and the only opportunity to learn its real
    // version before approval, since every packet after that (once it's
    // already running) omits "sw" -- capturing this only after the Pending
    // check would mean a node approved after its cold-boot packet never gets
    // credited with the version it already announced (Codex review on PR
    // #23). Most packets carry no "sw" at all, and that must leave whatever's
    // already known untouched -- an absent optional here means "nothing new
    // to report", never "clear it". Only a value that's actually different
    // from what's stored (a real flash/battery-swap cold boot, not a
    // retransmitted cold-boot packet) is worth persisting and republishing
    // discovery over.
    auto swIt = swVersionByNode_.find(reading.id);
    std::string knownSw = swIt != swVersionByNode_.end() ? swIt->second : "";
    bool swChanged = reading.swVersion.has_value() && *reading.swVersion != knownSw;
    if (swChanged) {
        swVersionByNode_[reading.id] = *reading.swVersion;
        store_.saveNodeVersionsCsv(serializeNodeVersions(swVersionByNode_));
    }

    RoutingResult route = decideRoute(reading.id, baseTopic_, allowList_);

    if (route.decision == RouteDecision::Pending) {
        pendingNodes_.insert(reading.id);
        display_.showLines({"New device: " + reading.id, "Approve via /devices"});
        return;
    }

    bool discoveryInFlight = pendingDiscoveryCount_.find(reading.id) != pendingDiscoveryCount_.end();
    bool isFirstDiscovery = discoveredNodes_.find(reading.id) == discoveredNodes_.end() && !discoveryInFlight;

    // Rebuild and (re-)publish this node's full discovery set on first
    // sight, or when its known "sw" just changed. A change while a previous
    // batch is still draining out of the queue can't republish immediately
    // -- overwriting pendingDiscoveryCount_[reading.id] mid-flight would
    // desync it from messages already counting down against the old value
    // -- so it's deferred instead (see pendingRediscovery_ and
    // onMessagePublished()).
    if (isFirstDiscovery) {
        publishNodeDiscovery(reading.id);
    } else if (swChanged) {
        requestRediscovery(reading.id);
    }

    // Deduplicate retransmissions: track the highest (bootCount, seq)
    // accepted per node and drop anything that isn't strictly newer, except
    // that a higher bootCount (a normal reboot) always resets the check --
    // a rebooted node's own seq counter restarts at 0 and would otherwise
    // look like an endless stream of "already seen" values -- and so does a
    // bootCount of exactly 1, this codebase's established cold-boot signal
    // (RTC memory, and so bootCount, resets to 0 only on power loss -- a
    // flash or battery swap -- and the node increments it to 1 before ever
    // transmitting; see loratemp's isScheduledDisplayBoot()/runNode() and
    // SensorReading::swVersion's comment). Without that second case, a
    // reflash/battery-swap -- whose bootCount restarts from 1, possibly far
    // below whatever high-water mark this long-running gateway process
    // still remembers from before the reflash -- looked exactly like an
    // attacker replaying an old packet with a backwards bootCount, and was
    // silently dropped forever with no recovery short of also restarting
    // the gateway (a field report: a reflashed, already-approved node never
    // appeared in MQTT until the gateway was manually restarted).
    //
    // Deliberately narrower than "any bootCount change resets the check"
    // (Codex review on PR #24): that would let an attacker alternate
    // between two previously-accepted bootCounts to bypass dedup
    // indefinitely, each one looking like a fresh "different" session
    // relative to the other. Gating the lower-bootCount exception on
    // exactly 1 limits it to the one value a legitimate cold boot can
    // actually produce; a captured bootCount-1 packet can still be
    // replayed, but only as that one specific stale reading, not as an
    // arbitrary pivot between any two sessions an attacker has observed --
    // consistent with this link's existing threat model (no authentication,
    // but no amplification of what a single captured packet can do either).
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
            PacketLossWindow& window = lossByNode_[reading.id];
            window.reset();
            window.received(reading.seq);
        } else if (reading.bootCount > seenIt->second.bootCount ||
                   (reading.bootCount == 1 && reading.bootCount != seenIt->second.bootCount)) {
            seenIt->second = LastSeen{reading.bootCount, reading.seq};
            PacketLossWindow& window = lossByNode_[reading.id];
            window.reset();
            window.received(reading.seq);
        } else if (reading.bootCount == seenIt->second.bootCount && reading.seq > seenIt->second.seq) {
            seenIt->second.seq = reading.seq;
            lossByNode_[reading.id].received(reading.seq);
        } else {
            // Same-or-earlier seq within the same boot (a retransmission),
            // or a bootCount that went backwards without looking like a
            // cold boot -- either way, already seen.
            isDuplicate = true;
        }
    }
    if (isDuplicate) return;

    LinkQuality link;
    link.rssi = packet.rssi;
    link.snr = packet.snr;
    if (reading.hasSeq) {
        auto lossIt = lossByNode_.find(reading.id);
        if (lossIt != lossByNode_.end()) link.lossPct = lossIt->second.lossPercent();
    }
    MqttMessage stateMsg = buildSensorStateMessage(reading, link, route.topic);
    if (onPacketForwarded_) onPacketForwarded_(stateMsg.topic, stateMsg.payload);
    // Retained, same convention as the gateway's own status message (see
    // main.cpp's publishGatewayStatus()): on a node's first sighting (or any
    // re-discovery after a gateway restart/reflash), this state message is
    // published immediately after that node's 10 discovery configs, in the
    // same synchronous burst. Home Assistant needs to finish processing a
    // discovery config -- creating the entity and subscribing to its state
    // topic -- before it can receive anything published to that topic; a
    // *non*-retained state message that lands on the broker before that
    // subscription exists is gone forever, and the entity sits
    // unavailable/stale until the node's *next* transmission, minutes later
    // (a field report: Home Assistant only updated on a node's second
    // packet, never its first). A retained message doesn't have this race:
    // the broker hands it to a client immediately upon SUBSCRIBE regardless
    // of what was published in between.
    enqueueOrPublish(stateMsg, true);
    display_.showLines(buildForwardedSummary(reading, route.topic));
}

} // namespace gateway
