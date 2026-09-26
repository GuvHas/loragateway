#pragma once

// In-memory fakes for the HAL interfaces (include/hal.h), used to drive
// GatewayOrchestrator in native unit tests without any real hardware.

#include <deque>
#include <string>
#include <vector>

#include "hal.h"

class FakeLoRa : public gateway::ILoRaReceiver {
public:
    void push(const std::string& data, int rssi = -60) {
        queue_.push_back(gateway::RawPacket{data, rssi});
    }

    bool receive(gateway::RawPacket& out) override {
        if (queue_.empty()) return false;
        out = queue_.front();
        queue_.pop_front();
        return true;
    }

private:
    std::deque<gateway::RawPacket> queue_;
};

class FakeMqtt : public gateway::IMqttClient {
public:
    struct PublishedMessage {
        std::string topic;
        std::string payload;
        bool retain;
    };

    bool connected() override { return connected_; }

    bool connect() override {
        connectAttempts++;
        connected_ = connectShouldSucceed;
        return connected_;
    }

    void disconnect() override { connected_ = false; }

    bool publish(const std::string& topic, const std::string& payload, bool retain) override {
        if (!connected_ || !publishShouldSucceed) return false;
        published.push_back(PublishedMessage{topic, payload, retain});
        return true;
    }

    void loop() override { loopCalls++; }

    // Test controls
    bool connectShouldSucceed = true;
    bool publishShouldSucceed = true;
    bool connected_ = false;

    // Test observations
    int connectAttempts = 0;
    int loopCalls = 0;
    std::vector<PublishedMessage> published;
};

class FakeStore : public gateway::INodeStore {
public:
    explicit FakeStore(std::string initialCsv = "") : csv_(std::move(initialCsv)) {}

    std::string loadAllowListCsv() override { return csv_; }

    void saveAllowListCsv(const std::string& csv) override {
        csv_ = csv;
        saveCount++;
    }

    std::string csv_;
    int saveCount = 0;
};

class FakeDisplay : public gateway::IDisplay {
public:
    void showLines(const std::vector<std::string>& lines) override {
        lastLines = lines;
        showCount++;
    }

    std::vector<std::string> lastLines;
    int showCount = 0;
};

class FakeClock : public gateway::IClock {
public:
    unsigned long millis() override { return now_; }

    void advance(unsigned long ms) { now_ += ms; }
    void set(unsigned long ms) { now_ = ms; }

private:
    unsigned long now_ = 0;
};
