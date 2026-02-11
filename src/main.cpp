#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SSD1306.h"
#include <WiFiManager.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <set>
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>

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
#define WAKE_ON_MQTT_RECONNECT 5000
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
#define ALLOWLIST_LEN    200
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
char allowed_nodes[ALLOWLIST_LEN] = "";

bool shouldSaveConfig = false;
unsigned long lastScreenUpdate = 0;
unsigned long screenTimeout = SCREEN_TIMEOUT_MS;
bool isScreenOn = true;
unsigned long lastStatusPublish = 0;
unsigned long packetCount = 0;

// Nodes seen on-air but not yet approved (in-memory only, re-populated on receive)
std::set<String> pending_nodes;

// Set for O(1) lookup of already-discovered nodes
std::set<String> discovered_nodes;

// ==========================================
//             GLOBAL OBJECTS
// ==========================================
SSD1306 display(0x3C, 21, 22);
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;
WiFiManager wm;

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

bool isNodeAllowed(const String& id) {
  String list = String(allowed_nodes);
  list.trim();
  if (list.length() == 0) return false;

  int start = 0;
  while (start <= (int)list.length()) {
    int comma = list.indexOf(',', start);
    if (comma == -1) comma = list.length();
    String entry = list.substring(start, comma);
    entry.trim();
    if (entry.equalsIgnoreCase(id)) return true;
    start = comma + 1;
  }
  return false;
}

void approveNode(const String& id) {
  if (isNodeAllowed(id)) return;

  String list = String(allowed_nodes);
  list.trim();
  if (list.length() > 0) list += ",";
  list += id;
  safeCopy(allowed_nodes, list.c_str(), sizeof(allowed_nodes));

  preferences.putString("allow", allowed_nodes);
  pending_nodes.erase(id);
  Serial.println("APPROVED node: " + id);
}

void removeNode(const String& id) {
  String list = String(allowed_nodes);
  String newList = "";
  int start = 0;
  while (start <= (int)list.length()) {
    int comma = list.indexOf(',', start);
    if (comma == -1) comma = list.length();
    String entry = list.substring(start, comma);
    entry.trim();
    if (!entry.equalsIgnoreCase(id) && entry.length() > 0) {
      if (newList.length() > 0) newList += ",";
      newList += entry;
    }
    start = comma + 1;
  }
  safeCopy(allowed_nodes, newList.c_str(), sizeof(allowed_nodes));
  preferences.putString("allow", allowed_nodes);
  discovered_nodes.erase(id);
  Serial.println("REMOVED node: " + id);
}

String availabilityTopic() {
  return String(mqtt_topic) + "/gateway/status";
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
  html += ".approve{background:#27ae60;}.remove{background:#c0392b;}";
  html += ".none{color:#666;font-style:italic;padding:10px;}";
  html += "a.back{color:#0fbcf9;display:inline-block;margin-top:15px;}";
  html += "</style></head><body>";
  html += "<h1>Device Management</h1>";

  // --- Pending (unapproved) nodes ---
  html += "<h2>Pending Devices</h2>";
  if (pending_nodes.empty()) {
    html += "<div class='none'>No new devices detected yet.</div>";
  } else {
    for (const String& id : pending_nodes) {
      html += "<div class='dev'><span class='name'>" + id + "</span>";
      html += "<a class='btn approve' href='/approve?id=" + id + "'>Approve</a></div>";
    }
  }

  // --- Approved nodes ---
  html += "<h2>Approved Devices</h2>";
  String list = String(allowed_nodes);
  list.trim();
  if (list.length() == 0) {
    html += "<div class='none'>No approved devices.</div>";
  } else {
    int start = 0;
    while (start <= (int)list.length()) {
      int comma = list.indexOf(',', start);
      if (comma == -1) comma = list.length();
      String entry = list.substring(start, comma);
      entry.trim();
      if (entry.length() > 0) {
        html += "<div class='dev'><span class='name'>" + entry + "</span>";
        html += "<a class='btn remove' href='/remove?id=" + entry + "'>Remove</a></div>";
      }
      start = comma + 1;
    }
  }

  html += "<a class='back' href='/'>Back to settings</a>";
  html += "</body></html>";
  wm.server->send(200, "text/html", html);
}

void handleApprove() {
  if (wm.server->hasArg("id")) {
    String id = wm.server->arg("id");
    approveNode(id);
  }
  wm.server->sendHeader("Location", "/devices", true);
  wm.server->send(302, "text/plain", "Redirecting...");
}

void handleRemove() {
  if (wm.server->hasArg("id")) {
    String id = wm.server->arg("id");
    removeNode(id);
  }
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

void drawFooter() {
  display.drawLine(0, 52, 128, 52);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 54, WiFi.localIP().toString());
  long rssi = WiFi.RSSI();
  int bars = (rssi > -55) ? 4 : (rssi > -65) ? 3 : (rssi > -75) ? 2 : 1;
  if (rssi == 0) bars = 0;
  String signalStr = String(bars) + "/4";
  int strWidth = display.getStringWidth(signalStr);
  display.drawString(128 - strWidth, 54, signalStr);
}

void reconnect() {
  static unsigned long lastReconnectAttempt = 0;
  unsigned long now = millis();

  if (now - lastReconnectAttempt > MQTT_RECONNECT_MS) {
    lastReconnectAttempt = now;
    Serial.print("Connecting to MQTT...");
    if (!isScreenOn) wakeDisplay(WAKE_ON_MQTT_RECONNECT);
    display.clear();
    display.drawString(0, 0, "MQTT Reconnecting...");
    display.display();

    int port = atoi(mqtt_port);
    client.setServer(mqtt_server, port);
    String clientId = String(device_name) + "-" + String(random(0xffff), HEX);

    String lwt_topic = availabilityTopic();
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass,
                       lwt_topic.c_str(), 1, true, "offline")) {
      Serial.println("connected");
      client.publish(lwt_topic.c_str(), "online", true);
      display.clear();
      display.drawString(0, 0, "MQTT Connected!");
      display.display();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again later");
    }
  }
}

// ==========================================
//         GATEWAY STATUS PUBLISHING
// ==========================================
void publishGatewayStatus() {
  if (!client.connected()) return;

  StaticJsonDocument<256> doc;
  doc["uptime_s"] = millis() / 1000;
  doc["free_heap"] = ESP.getFreeHeap();
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["packets_rx"] = packetCount;
  doc["ip"] = WiFi.localIP().toString();

  String payload;
  serializeJson(doc, payload);

  String topic = String(mqtt_topic) + "/gateway/state";
  client.publish(topic.c_str(), payload.c_str(), true);
}

// ==========================================
//        AUTO DISCOVERY FUNCTION
// ==========================================
void sendAutoDiscovery(const String& node_id) {
  Serial.println("Sending Auto Discovery for: " + node_id);

  String safe_id = node_id;
  safe_id.toLowerCase();

  String state_topic = String(mqtt_topic) + "/" + safe_id;
  String avail_topic = availabilityTopic();

  StaticJsonDocument<600> doc;

  JsonObject dev = doc.createNestedObject("dev");
  dev["ids"] = "lora_" + safe_id;
  dev["name"] = node_id;
  dev["mdl"] = "LoRa Sensor Node";
  dev["mf"] = "DIY";
  dev["via_device"] = String(device_name);

  String dev_buf;
  serializeJson(dev, dev_buf);

  auto publishEntity = [&](const char* component, const char* suffix,
                           const char* name_suffix, const char* val_tpl,
                           const char* unit, const char* dev_class,
                           const char* ent_cat = "", int precision = -1) {
    doc.clear();
    doc["name"] = node_id + " " + name_suffix;
    doc["stat_t"] = state_topic;
    doc["val_tpl"] = val_tpl;
    if (strlen(unit) > 0) doc["unit_of_meas"] = unit;
    if (strlen(dev_class) > 0) doc["dev_cla"] = dev_class;
    doc["uniq_id"] = "lora_" + safe_id + "_" + suffix;
    doc["avty_t"] = avail_topic;
    if (strlen(ent_cat) > 0) doc["ent_cat"] = ent_cat;
    if (precision >= 0) {
      doc["sugg_dsp_prec"] = precision;
    }
    StaticJsonDocument<200> dev_doc;
    deserializeJson(dev_doc, dev_buf);
    doc["dev"] = dev_doc.as<JsonObject>();

    String topic = "homeassistant/" + String(component) + "/lora_" + safe_id + "_" + suffix + "/config";
    String buffer;
    serializeJson(doc, buffer);
    client.publish(topic.c_str(), buffer.c_str(), true);
  };

  publishEntity("sensor", "t", "Temperature", "{{ value_json.t }}", "\u00b0C", "temperature");
  publishEntity("sensor", "h", "Humidity",    "{{ value_json.h }}", "%",    "humidity");
  publishEntity("sensor", "v", "Battery",     "{{ value_json.v }}", "V",    "voltage", "", 2);
  publishEntity("sensor", "r", "Signal",      "{{ value_json.rssi }}", "dBm", "signal_strength");

  publishEntity("sensor", "boot", "Boot Count",
                "{{ value_json.boot | default(0) }}", "restarts", "",
                "diagnostic");

  publishEntity("binary_sensor", "lb", "Low Battery",
                "{{ 'ON' if value_json.lb is defined and value_json.lb == 1 else 'OFF' }}",
                "", "battery");

  publishEntity("sensor", "err", "Error",
                "{{ value_json.err | default('none') }}", "", "",
                "diagnostic");
}

void sendGatewayDiscovery() {
  String avail_topic = availabilityTopic();
  String state_topic = String(mqtt_topic) + "/gateway/state";
  String gw_id = String(device_name);
  gw_id.toLowerCase();
  gw_id.replace(" ", "_");

  StaticJsonDocument<600> doc;
  JsonObject dev = doc.createNestedObject("dev");
  dev["ids"] = gw_id;
  dev["name"] = String(device_name);
  dev["mdl"] = "ESP32 LoRa Gateway";
  dev["mf"] = "DIY";

  String dev_buf;
  serializeJson(dev, dev_buf);

  auto publishGwSensor = [&](const char* suffix, const char* name_suffix,
                              const char* val_tpl, const char* unit,
                              const char* dev_class) {
    doc.clear();
    doc["name"] = String(device_name) + " " + name_suffix;
    doc["stat_t"] = state_topic;
    doc["val_tpl"] = val_tpl;
    doc["unit_of_meas"] = unit;
    if (strlen(dev_class) > 0) doc["dev_cla"] = dev_class;
    doc["uniq_id"] = gw_id + "_" + suffix;
    doc["avty_t"] = avail_topic;
    doc["ent_cat"] = "diagnostic";

    StaticJsonDocument<200> dev_doc;
    deserializeJson(dev_doc, dev_buf);
    doc["dev"] = dev_doc.as<JsonObject>();

    String topic = "homeassistant/sensor/" + gw_id + "_" + suffix + "/config";
    String buffer;
    serializeJson(doc, buffer);
    client.publish(topic.c_str(), buffer.c_str(), true);
  };

  publishGwSensor("wifi", "WiFi Signal", "{{ value_json.wifi_rssi }}", "dBm", "signal_strength");
  publishGwSensor("heap", "Free Memory", "{{ value_json.free_heap }}", "B", "");
  publishGwSensor("pkts", "Packets Received", "{{ value_json.packets_rx }}", "pkts", "");
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
  preferences.getString("allow", "").toCharArray(allowed_nodes, ALLOWLIST_LEN);

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

  wm.setConfigPortalBlocking(false);
  wm.setSaveConfigCallback(saveConfigCallback);
  wm.addParameter(&custom_device_name);
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.addParameter(&custom_mqtt_topic);

  // Add a link to the device management page in the WiFiManager portal
  WiFiManagerParameter devices_link("<br><a href='/devices' style='color:#0fbcf9;font-size:1.1em;'>Manage Devices</a>");
  wm.addParameter(&devices_link);

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

    discovered_nodes.clear();
    client.disconnect();

    if (nameChanged) {
      WiFi.setHostname(device_name);
      ArduinoOTA.setHostname(device_name);
      WiFi.disconnect();
      WiFi.reconnect();
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnect();
      }
      client.loop();

      if (client.connected() && (millis() - lastStatusPublish > STATUS_PUBLISH_MS)) {
        lastStatusPublish = millis();
        static bool gatewayDiscoverySent = false;
        if (!gatewayDiscoverySent) {
          sendGatewayDiscovery();
          gatewayDiscoverySent = true;
        }
        publishGatewayStatus();
      }
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String raw_data;
    raw_data.reserve(packetSize);
    while (LoRa.available()) {
      raw_data += (char)LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    packetCount++;

    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, raw_data);

    String finalTopic = mqtt_topic;
    String incoming;

    if (!error) {
        if (doc.containsKey("id")) {
            String id = doc["id"].as<String>();

            if (!isNodeAllowed(id)) {
              // Track as pending — will appear on the /devices web page
              pending_nodes.insert(id);
              Serial.println("RX PENDING: " + id + " — approve via http://" + WiFi.localIP().toString() + "/devices");
              wakeDisplay(WAKE_ON_PACKET_MS);
              display.clear();
              display.setFont(ArialMT_Plain_10);
              display.drawString(0, 0, "New device: " + id);
              display.drawString(0, 15, "Approve at:");
              display.drawString(0, 30, "http://" + WiFi.localIP().toString() + "/devices");
              drawFooter();
              display.display();
              return;
            }

            if (discovered_nodes.find(id) == discovered_nodes.end()) {
              sendAutoDiscovery(id);
              discovered_nodes.insert(id);
            }

            String safe_id = id;
            safe_id.toLowerCase();
            finalTopic = String(mqtt_topic) + "/" + safe_id;
        }

        doc["rssi"] = rssi;
        serializeJson(doc, incoming);

        Serial.print("RX: ");
        Serial.println(incoming);

        client.publish(finalTopic.c_str(), incoming.c_str());

        wakeDisplay(WAKE_ON_PACKET_MS);
        display.clear();
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 0, "Fwd: " + finalTopic);
        display.drawStringMaxWidth(0, 15, 128, incoming);
        drawFooter();
        display.display();

    } else {
        Serial.print("RX (Raw): ");
        Serial.println(raw_data);
        client.publish(finalTopic.c_str(), raw_data.c_str());
    }
  }
}
