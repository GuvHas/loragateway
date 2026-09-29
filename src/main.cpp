#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SSD1306.h"
#include <WiFiManager.h>
#include <Preferences.h>
#include <string>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>

#include "hal_esp32.h"
#include "orchestrator.h"
#include "payload_parser.h"

// ==========================================
//        HARDWARE PINS (TTGO LoRa32 V2.1)
// ==========================================
#define SCK_PIN  5
#define MISO_PIN 19
#define MOSI_PIN 27
#define SS_PIN   18
#define RST_PIN  23
#define DI0_PIN  26
#define BAND 868E6
#define LORA_SF  9   // Must match sender spreading factor (7-12)

// ==========================================
//              TIMING CONSTANTS
// ==========================================
#define LOGO_DISPLAY_MS        5000
#define SCREEN_TIMEOUT_MS      30000
#define MQTT_RECONNECT_MS      5000
#define WIFI_RECONNECT_MS      10000
#define WAKE_ON_SAVE_MS        10000
#define WAKE_ON_PACKET_MS      5000
#define STATUS_PUBLISH_MS      60000
#define WDT_TIMEOUT_S          30

// ==========================================
//              BUFFER SIZES
// ==========================================
#define MQTT_BUFFER_SIZE 1024
#define FIELD_LEN        40
#define PORT_LEN         6
#define LORA_MAX_PACKET  255

// ==========================================
//              LOGO BITMAP
// ==========================================
const unsigned char my_bitmap_logo1_modified [] PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0xa0,0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x54,0xb5,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x95,
  0x4a,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x80,0x52,0x52,
  0x0a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0xaa,0xaa,0x02,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xa0,0x4a,0x49,0x01,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x29,0x55,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x28,0x55,0x15,0x20,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x48,0x49,0x12,0x50,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x54,0xaa,0x0a,0x28,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0xa4,0xaa,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xaa,0x92,0x00,0x8a,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x94,0x54,0x00,0x84,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x52,0x15,0x00,0x45,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x2a,0x05,0x80,0x82,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x4a,0x01,0x40,0xa1,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x52,
  0x01,0x20,0xa0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x55,0x00,
  0xa0,0x90,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x2a,0x00,0x48,
  0x50,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x10,0xa8,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x14,0x94,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0a,0xa4,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0xaa,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x09,0x55,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x05,0x49,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x02,0x15,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0xa0,0x80,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x90,0x80,0x14,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x28,0x40,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x24,0xa0,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x2a,0x20,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x08,0x50,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xfe,0x3f,0xfc,0x7f,0xfc,0xff,0x01,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0xab,0x2a,0x56,0xd5,0x54,0xd7,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x80,0x01,0x60,0x02,0x80,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x01,0x00,0x06,0x00,0x00,0x03,0xf8,0x3f,0xfe,0xff,0xc7,0xff,0x01,0x00,
  0x01,0x00,0x02,0x00,0x00,0x02,0x58,0x75,0xae,0xae,0xc6,0xaa,0x03,0x80,0x01,
  0x00,0x02,0x00,0x00,0x02,0x0c,0x20,0x02,0x04,0x4c,0x00,0x02,0x00,0x01,0x00,
  0x06,0x00,0x00,0x03,0x08,0x60,0x06,0x06,0xc4,0x00,0x02,0x00,0x01,0x3e,0x02,
  0xf8,0x00,0x02,0x0c,0x40,0x04,0x04,0x4c,0x00,0x03,0x80,0x01,0x34,0x06,0xa8,
  0x00,0x03,0xf8,0x77,0x06,0x0c,0xc8,0x00,0x02,0x00,0x01,0x60,0x02,0x80,0x00,
  0x02,0xac,0x5a,0x04,0x06,0x4c,0x00,0x02,0x00,0x01,0x20,0x06,0xc0,0x00,0x02,
  0x08,0x00,0x02,0x04,0x44,0x00,0x03,0x80,0x01,0x20,0x02,0x80,0x00,0x03,0x0c,
  0x00,0x06,0x04,0xcc,0x00,0x02,0x00,0x01,0x60,0x06,0x80,0x00,0x02,0x08,0x00,
  0x04,0x0c,0x44,0x00,0x03,0x00,0x6f,0x35,0x54,0xd5,0x00,0x03,0x58,0x55,0x06,
  0x06,0xcc,0x55,0x03,0x00,0xfb,0x1f,0xfc,0x7f,0x00,0x02,0xf8,0x3f,0x02,0x04,
  0xc4,0xff,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x60,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x40,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

// ==========================================
//        DEFAULT / STORAGE VARIABLES
// ==========================================
char mqtt_server[FIELD_LEN] = "";
char mqtt_port[PORT_LEN] = "1883";
char mqtt_user[FIELD_LEN] = "";
char mqtt_pass[FIELD_LEN] = "";
char mqtt_topic[FIELD_LEN] = "lora/incoming";
char device_name[FIELD_LEN] = "LoRaGateway";

bool shouldSaveConfig = false;
unsigned long lastScreenUpdate = 0;
unsigned long screenTimeout = SCREEN_TIMEOUT_MS;
bool isScreenOn = true;
unsigned long lastStatusPublish = 0;

// Forward declaration: Esp32Display (below) invokes this on every display
// update so the existing screen-timeout bookkeeping keeps working without
// that display adapter needing to know about it.
void wakeDisplay(unsigned long duration_ms);

// Forward declaration: passed to the orchestrator (below) as its
// PacketLogCallback, so the full JSON payload is still visible over Serial
// even though the OLED (see Esp32Display::showLines()) only shows a short,
// curated summary of each forwarded reading.
void logForwardedPacket(const std::string& topic, const std::string& payload);

// ==========================================
//             GLOBAL OBJECTS
// ==========================================
SSD1306 display(0x3C, 21, 22);
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
WiFiManager wm;

// Hardware Abstraction Layer adapters (include/hal_esp32.h) and the
// orchestrator that owns all WiFi/LoRa/MQTT/allowlist business logic
// (include/orchestrator.h, unit-tested natively in test/test_orchestrator).
gateway::Esp32WifiRadio wifiRadio;
gateway::Esp32LoRaReceiver loRaReceiver;
gateway::Esp32MqttClient mqttAdapter(client);
gateway::Esp32NodeStore nodeStore(preferences);
gateway::Esp32Display esp32Display(display, []() { wakeDisplay(WAKE_ON_PACKET_MS); });
gateway::Esp32Clock esp32Clock;

gateway::GatewayOrchestrator orchestrator(wifiRadio, loRaReceiver, mqttAdapter, nodeStore,
                                           esp32Display, esp32Clock, std::string(device_name),
                                           std::string(mqtt_topic), MQTT_RECONNECT_MS,
                                           WIFI_RECONNECT_MS, logForwardedPacket);

// WiFiManager Parameters
WiFiManagerParameter custom_device_name("devname", "Device Name", "LoRaGateway", FIELD_LEN);
WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP", "", FIELD_LEN);
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", "1883", PORT_LEN);
WiFiManagerParameter custom_mqtt_user("user", "MQTT User", "", FIELD_LEN);
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", "", FIELD_LEN);
WiFiManagerParameter custom_mqtt_topic("topic", "MQTT Base Topic", "lora/incoming", FIELD_LEN);

void saveConfigCallback () {
  Serial.println("Settings changed via Web Portal!");
  shouldSaveConfig = true;
}

// ==========================================
//          SAFE STRING HELPERS
// ==========================================

void safeCopy(char* dest, const char* src, size_t destSize) {
  strncpy(dest, src, destSize - 1);
  dest[destSize - 1] = '\0';
}

gateway::GatewayIdentity currentGatewayIdentity() {
  return gateway::GatewayIdentity{std::string(device_name), std::string(mqtt_topic)};
}

// "1d 2h 3m 4s"-style formatting for the /devices live-metrics panel;
// omits leading zero units (e.g. "45s" alone once uptime is under a minute).
String formatUptime(unsigned long totalSeconds) {
  unsigned long days = totalSeconds / 86400;
  unsigned long hours = (totalSeconds % 86400) / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;

  String out;
  if (days > 0) out += String(days) + "d ";
  if (days > 0 || hours > 0) out += String(hours) + "h ";
  if (days > 0 || hours > 0 || minutes > 0) out += String(minutes) + "m ";
  out += String(seconds) + "s";
  return out;
}

// ==========================================
//         DEVICE MANAGEMENT WEB PAGE
// ==========================================
void handleDevicesPage() {
  String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>Device Management</title>";
  html += "<style>";
  html += "body{font-family:sans-serif;margin:20px;background:#1a1a2e;color:#e0e0e0;}";
  html += "h1{color:#0fbcf9;}h2{color:#aaa;border-bottom:1px solid #333;padding-bottom:5px;}";
  html += ".dev{display:flex;justify-content:space-between;align-items:center;padding:10px;margin:5px 0;background:#16213e;border-radius:6px;}";
  html += ".dev .name{font-size:1.1em;font-weight:bold;}";
  html += ".btn{padding:8px 16px;border:none;border-radius:4px;cursor:pointer;font-size:0.9em;text-decoration:none;color:#fff;}";
  html += ".approve{background:#27ae60;}.remove{background:#c0392b;}.clear{background:#2980b9;}";
  html += ".none{color:#666;font-style:italic;padding:10px;}";
  html += "a.back{color:#0fbcf9;display:inline-block;margin-top:15px;}";
  html += "</style></head><body>";
  html += "<h1>Device Management</h1>";

  // --- Live gateway metrics ---
  html += "<h2>Gateway Status</h2>";
  html += "<div class='dev'><span class='name'>Uptime</span><span>" + formatUptime(millis() / 1000) +
          "</span></div>";
  html += "<div class='dev'><span class='name'>WiFi RSSI</span><span>" + String(WiFi.RSSI()) +
          " dBm</span></div>";
  html += "<div class='dev'><span class='name'>Queue Depth</span><span>" +
          String(orchestrator.queuedMessageCount()) + "</span></div>";
  html += "<div class='dev'><span class='name'>Packets Dropped</span><span>" +
          String(orchestrator.droppedMessageCount()) + "</span></div>";
  html += "<div class='dev'><span class='name'>Discovered Nodes</span>";
  html += "<a class='btn clear' href='/clear-discovered'>Clear Discovered Nodes</a></div>";

  // --- Pending (unapproved) nodes ---
  // Node ids come from unauthenticated LoRa packets. Display text is
  // HTML-escaped (gateway::htmlEscape); href query values additionally need
  // URL-encoding first (gateway::urlEncodeComponent) since a raw '&' would
  // survive HTML-escaping (-> "&amp;") only for the browser to decode it
  // straight back to '&' and split the query string, misrouting the
  // approve/remove action to the wrong id. See test_parser for both.
  html += "<h2>Pending Devices</h2>";
  auto pendingIds = orchestrator.pendingNodeIds();
  if (pendingIds.empty()) {
    html += "<div class='none'>No new devices detected yet.</div>";
  } else {
    for (const auto& id : pendingIds) {
      String name = String(gateway::htmlEscape(id).c_str());
      String href = String(gateway::htmlEscape(gateway::urlEncodeComponent(id)).c_str());
      html += "<div class='dev'><span class='name'>" + name + "</span>";
      html += "<a class='btn approve' href='/approve?id=" + href + "'>Approve</a></div>";
    }
  }

  // --- Approved nodes ---
  html += "<h2>Approved Devices</h2>";
  if (orchestrator.allowList().entries().empty()) {
    html += "<div class='none'>No approved devices.</div>";
  } else {
    for (const auto& entry : orchestrator.allowList().entries()) {
      String name = String(gateway::htmlEscape(entry).c_str());
      String href = String(gateway::htmlEscape(gateway::urlEncodeComponent(entry)).c_str());
      html += "<div class='dev'><span class='name'>" + name + "</span>";
      html += "<a class='btn remove' href='/remove?id=" + href + "'>Remove</a></div>";
    }
  }

  html += "<a class='back' href='/'>Back to settings</a>";
  html += "</body></html>";
  wm.server->send(200, "text/html", html);
}

void handleApprove() {
  if (wm.server->hasArg("id")) {
    orchestrator.approveNode(wm.server->arg("id").c_str());
  }
  wm.server->sendHeader("Location", "/devices", true);
  wm.server->send(302, "text/plain", "Redirecting...");
}

void handleRemove() {
  if (wm.server->hasArg("id")) {
    orchestrator.removeNode(wm.server->arg("id").c_str());
  }
  wm.server->sendHeader("Location", "/devices", true);
  wm.server->send(302, "text/plain", "Redirecting...");
}

// Forces every node to re-run Home Assistant auto-discovery on its next
// packet (see GatewayOrchestrator::clearDiscoveredNodes()). Does not touch
// the allowlist -- approved nodes stay approved, this only clears the
// "already discovered" bookkeeping.
void handleClearDiscovered() {
  orchestrator.clearDiscoveredNodes();
  wm.server->sendHeader("Location", "/devices", true);
  wm.server->send(302, "text/plain", "Redirecting...");
}

// ==========================================
//           HELPER FUNCTIONS
// ==========================================

void present_logo() {
  display.clear();
  display.displayOn();
  display.drawXbm(0,0,112,64,my_bitmap_logo1_modified);
  display.display();
  delay(LOGO_DISPLAY_MS);
}

void setup_display() {
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
}

void wakeDisplay(unsigned long duration_ms) {
  display.displayOn();
  isScreenOn = true;
  lastScreenUpdate = millis();
  screenTimeout = duration_ms;
}

// The OLED only shows a short summary of each forwarded reading (see
// Esp32Display::showLines()), so the full JSON is logged here instead —
// this is the orchestrator's PacketLogCallback, fired for every packet
// that's successfully parsed and routed to an approved node.
void logForwardedPacket(const std::string& topic, const std::string& payload) {
  Serial.print("RX ");
  Serial.print(topic.c_str());
  Serial.print(": ");
  Serial.println(payload.c_str());
}

// ==========================================
//         GATEWAY STATUS PUBLISHING
// ==========================================
// Per-node auto-discovery, LoRa ingestion/routing, sensor-state publishing
// and MQTT reconnect are all owned by `orchestrator` (see
// include/orchestrator.h). What's left here is the gateway's own periodic
// diagnostic status, which isn't part of that per-packet flow.

void publishGatewayStatus() {
  if (!mqttAdapter.connected()) return;

  gateway::GatewayStats stats;
  stats.uptimeSeconds = millis() / 1000;
  stats.freeHeapBytes = ESP.getFreeHeap();
  stats.wifiRssi = WiFi.RSSI();
  stats.packetsReceived = orchestrator.packetsReceived();
  stats.ipAddress = WiFi.localIP().toString().c_str();
  stats.onlyKnownNodes = !orchestrator.allowList().entries().empty();
  stats.queueDepth = orchestrator.queuedMessageCount();
  stats.packetsDropped = orchestrator.droppedMessageCount();

  gateway::MqttMessage msg = gateway::buildGatewayStatusMessage(currentGatewayIdentity(), stats);
  mqttAdapter.publish(msg.topic, msg.payload, true);
}

void sendGatewayDiscovery() {
  for (const auto& msg : gateway::buildGatewayDiscoveryMessages(currentGatewayIdentity())) {
    mqttAdapter.publish(msg.topic, msg.payload, true);
  }
}

// ==========================================
//                 SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  esp_task_wdt_init(WDT_TIMEOUT_S, true);
  esp_task_wdt_add(NULL);

  setup_display();
  present_logo();

  display.clear();
  display.drawString(0, 0, "Booting System...");
  display.display();
  wakeDisplay(SCREEN_TIMEOUT_MS);

  preferences.begin("loraconf", false);
  if(preferences.getString("server", "").length() > 0){
     preferences.getString("server").toCharArray(mqtt_server, FIELD_LEN);
     preferences.getString("port").toCharArray(mqtt_port, PORT_LEN);
     preferences.getString("user").toCharArray(mqtt_user, FIELD_LEN);
     preferences.getString("pass").toCharArray(mqtt_pass, FIELD_LEN);
     preferences.getString("topic").toCharArray(mqtt_topic, FIELD_LEN);
  }
  if(preferences.getString("devname", "").length() > 0){
     preferences.getString("devname").toCharArray(device_name, FIELD_LEN);
  }

  orchestrator.setIdentity(device_name, mqtt_topic);
  mqttAdapter.configure(device_name, mqtt_user, mqtt_pass,
                         gateway::availabilityTopic(std::string(mqtt_topic)));
  orchestrator.begin(); // loads the persisted allowlist via nodeStore

  WiFi.setHostname(device_name);

  custom_mqtt_server.setValue(mqtt_server, FIELD_LEN);
  custom_mqtt_port.setValue(mqtt_port, PORT_LEN);
  custom_mqtt_user.setValue(mqtt_user, FIELD_LEN);
  custom_mqtt_pass.setValue(mqtt_pass, FIELD_LEN);
  custom_mqtt_topic.setValue(mqtt_topic, FIELD_LEN);
  custom_device_name.setValue(device_name, FIELD_LEN);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  if (!LoRa.begin(BAND)) {
    Serial.println("LoRa Failed!");
    display.drawString(0, 15, "LoRa Hardware Fail!");
    display.display();
    while (1) delay(1000);
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.enableCrc();

  // Switches from polling LoRa.parsePacket() (once per loop() iteration) to
  // an interrupt-driven receive via the SX1276's DIO0 pin, so a packet
  // arriving while loop() is stuck in blocking MQTT/network I/O still gets
  // captured instead of missed. See Esp32LoRaReceiver::begin() and its
  // class comment in hal_esp32.h.
  loRaReceiver.begin();

  wm.setConfigPortalBlocking(false);
  wm.setSaveConfigCallback(saveConfigCallback);

  // Add "Manage Devices" link to the main portal menu
  const char* menu[] = {"wifi", "param", "info", "custom", "exit"};
  wm.setMenu(menu, 5);
  wm.setCustomMenuHTML("<form action='/devices' method='get'><button type='submit'>Manage Devices</button></form>");

  wm.addParameter(&custom_device_name);
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.addParameter(&custom_mqtt_topic);

  display.clear();
  display.drawString(0, 0, "Connecting WiFi...");
  display.drawString(0, 15, "AP: LoRaGateway-Setup");
  display.display();

  if(wm.autoConnect("LoRaGateway-Setup")) {
      Serial.println("WiFi Connected!");
  } else {
      Serial.println("WiFi Portal Active");
  }

  wm.startWebPortal();

  // Register custom device management routes on WiFiManager's web server
  wm.server->on("/devices", handleDevicesPage);
  wm.server->on("/approve", handleApprove);
  wm.server->on("/remove",  handleRemove);
  wm.server->on("/clear-discovered", handleClearDiscovered);

  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setBufferSize(MQTT_BUFFER_SIZE);

  // Setup OTA updates
  ArduinoOTA.setHostname(device_name);
  ArduinoOTA.onStart([]() {
    display.displayOn();
    display.clear();
    display.drawString(0, 0, "OTA Update...");
    display.display();
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int pct = progress / (total / 100);
    display.clear();
    display.drawString(0, 0, "OTA Update...");
    display.drawProgressBar(0, 20, 120, 10, pct);
    display.drawString(0, 35, String(pct) + "%");
    display.display();
  });
  ArduinoOTA.onEnd([]() {
    display.clear();
    display.drawString(0, 0, "OTA Complete!");
    display.drawString(0, 15, "Rebooting...");
    display.display();
  });
  ArduinoOTA.onError([](ota_error_t error) {
    display.clear();
    display.drawString(0, 0, "OTA Failed!");
    display.display();
    Serial.printf("OTA Error[%u]\n", error);
  });
  ArduinoOTA.begin();

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Gateway Ready");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, "IP: " + WiFi.localIP().toString());
  display.drawString(0, 35, "Screen off in 30s");
  display.display();
}

// ==========================================
//                 LOOP
// ==========================================
void loop() {
  esp_task_wdt_reset();

  wm.process();
  ArduinoOTA.handle();

  if (isScreenOn && (millis() - lastScreenUpdate > screenTimeout)) {
    display.displayOff();
    isScreenOn = false;
  }

  if (shouldSaveConfig) {
    shouldSaveConfig = false;
    wakeDisplay(WAKE_ON_SAVE_MS);
    display.clear();
    display.drawString(0,0, "Saving Settings...");
    display.display();

    String new_name = String(custom_device_name.getValue());
    bool nameChanged = (new_name != String(device_name));

    safeCopy(mqtt_server, custom_mqtt_server.getValue(), sizeof(mqtt_server));
    safeCopy(mqtt_port,   custom_mqtt_port.getValue(),   sizeof(mqtt_port));
    safeCopy(mqtt_user,   custom_mqtt_user.getValue(),   sizeof(mqtt_user));
    safeCopy(mqtt_pass,   custom_mqtt_pass.getValue(),   sizeof(mqtt_pass));
    safeCopy(mqtt_topic,  custom_mqtt_topic.getValue(),  sizeof(mqtt_topic));
    safeCopy(device_name, custom_device_name.getValue(), sizeof(device_name));

    preferences.putString("server", mqtt_server);
    preferences.putString("port", mqtt_port);
    preferences.putString("user", mqtt_user);
    preferences.putString("pass", mqtt_pass);
    preferences.putString("topic", mqtt_topic);
    preferences.putString("devname", device_name);

    orchestrator.setIdentity(device_name, mqtt_topic);
    orchestrator.clearDiscoveredNodes();
    mqttAdapter.configure(device_name, mqtt_user, mqtt_pass,
                           gateway::availabilityTopic(std::string(mqtt_topic)));
    client.setServer(mqtt_server, atoi(mqtt_port));
    mqttAdapter.disconnect();

    if (nameChanged) {
      WiFi.setHostname(device_name);
      ArduinoOTA.setHostname(device_name);
      WiFi.disconnect();
      WiFi.reconnect();
    }
  }

  // WiFi reconnect (with backoff), LoRa ingestion, allowlist/routing
  // decisions, sensor-state and discovery publishing, the store-and-forward
  // queue, and MQTT connect/backoff all live in the orchestrator
  // (include/orchestrator.h), unit-tested natively in test/test_orchestrator.
  orchestrator.tick();

  // mqttAdapter.connected() alone is a sufficient gate here: MQTT can't be
  // connected without WiFi also being up.
  if (mqttAdapter.connected() && (millis() - lastStatusPublish > STATUS_PUBLISH_MS)) {
    lastStatusPublish = millis();
    static bool gatewayDiscoverySent = false;
    if (!gatewayDiscoverySent) {
      sendGatewayDiscovery();
      gatewayDiscoverySent = true;
    }
    publishGatewayStatus();
  }
}
