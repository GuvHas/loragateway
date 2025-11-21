#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SSD1306.h"
#include <WiFiManager.h> 
#include <Preferences.h> 
#include <ArduinoJson.h> // <--- Include JSON Library

// ==========================================
//           HARDWARE PINS (TTGO V1.6)
// ==========================================
#define SCK_PIN  5
#define MISO_PIN 19
#define MOSI_PIN 27
#define SS_PIN   18
#define RST_PIN  23 
#define DI0_PIN  26
#define BAND 868E6 

// ==========================================
//        DEFAULT / STORAGE VARIABLES
// ==========================================
char mqtt_server[40] = "";
char mqtt_port[6] = "1883";
char mqtt_user[40] = "";
char mqtt_pass[40] = "";
char mqtt_topic[40] = "lora/incoming"; // This is now the BASE topic
char device_name[40] = "LoRaGateway";

bool shouldSaveConfig = false;

// ==========================================
//           SCREEN MANAGEMENT
// ==========================================
unsigned long lastScreenUpdate = 0;
unsigned long screenTimeout = 30000; 
bool isScreenOn = true;

// ==========================================
//             GLOBAL OBJECTS
// ==========================================
SSD1306 display(0x3C, 21, 22);
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences; 

WiFiManager wm;
WiFiManagerParameter custom_device_name("devname", "Device Name", "LoRaGateway", 40);
WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP", "", 40);
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", "1883", 6);
WiFiManagerParameter custom_mqtt_user("user", "MQTT User", "", 40);
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", "", 40);
WiFiManagerParameter custom_mqtt_topic("topic", "MQTT Base Topic", "lora/incoming", 40);

void saveConfigCallback () {
  Serial.println("Settings changed via Web Portal!");
  shouldSaveConfig = true;
}

// ==========================================
//           HELPER FUNCTIONS
// ==========================================

void setup_display() {
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
}

void wakeDisplay(int duration_ms) {
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
  
  if (now - lastReconnectAttempt > 5000) {
    lastReconnectAttempt = now;
    Serial.print("Connecting to MQTT...");
    
    if (!isScreenOn) wakeDisplay(5000);
    display.clear();
    display.drawString(0, 0, "MQTT Reconnecting...");
    display.display();

    int port = atoi(mqtt_port);
    client.setServer(mqtt_server, port);
    String clientId = String(device_name) + "-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
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
//                 SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  setup_display();
  display.drawString(0, 0, "Booting System...");
  display.display();
  wakeDisplay(30000); 

  preferences.begin("loraconf", false);
  if(preferences.getString("server", "").length() > 0){
     preferences.getString("server").toCharArray(mqtt_server, 40);
     preferences.getString("port").toCharArray(mqtt_port, 6);
     preferences.getString("user").toCharArray(mqtt_user, 40);
     preferences.getString("pass").toCharArray(mqtt_pass, 40);
     preferences.getString("topic").toCharArray(mqtt_topic, 40);
  }
  if(preferences.getString("devname", "").length() > 0){
     preferences.getString("devname").toCharArray(device_name, 40);
  }
  
  WiFi.setHostname(device_name);

  custom_mqtt_server.setValue(mqtt_server, 40);
  custom_mqtt_port.setValue(mqtt_port, 6);
  custom_mqtt_user.setValue(mqtt_user, 40);
  custom_mqtt_pass.setValue(mqtt_pass, 40);
  custom_mqtt_topic.setValue(mqtt_topic, 40);
  custom_device_name.setValue(device_name, 40);

  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  if (!LoRa.begin(BAND)) {
    Serial.println("LoRa Failed!");
    display.drawString(0, 15, "LoRa Hardware Fail!");
    display.display();
    while (1) delay(1000);
  }

  wm.setConfigPortalBlocking(false);
  wm.setSaveConfigCallback(saveConfigCallback);
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
  client.setServer(mqtt_server, atoi(mqtt_port));
  
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
  wm.process();

  if (isScreenOn && (millis() - lastScreenUpdate > screenTimeout)) {
    display.displayOff();
    isScreenOn = false;
  }

  if (shouldSaveConfig) {
    shouldSaveConfig = false;
    wakeDisplay(10000);
    display.clear();
    display.drawString(0,0, "Saving Settings...");
    display.display();
    
    String new_name = String(custom_device_name.getValue());
    bool nameChanged = (new_name != String(device_name));

    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_user, custom_mqtt_user.getValue());
    strcpy(mqtt_pass, custom_mqtt_pass.getValue());
    strcpy(mqtt_topic, custom_mqtt_topic.getValue());
    strcpy(device_name, custom_device_name.getValue());

    preferences.putString("server", mqtt_server);
    preferences.putString("port", mqtt_port);
    preferences.putString("user", mqtt_user);
    preferences.putString("pass", mqtt_pass);
    preferences.putString("topic", mqtt_topic);
    preferences.putString("devname", device_name);
    
    client.disconnect(); 

    if (nameChanged) {
      WiFi.setHostname(device_name);
      WiFi.disconnect(); 
      WiFi.reconnect();
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnect();
      }
      client.loop();
  }

  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    
    Serial.print("RX: ");
    Serial.println(incoming);

    // === NEW LOGIC START ===
    // 1. Parse JSON to find the ID
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, incoming);
    
    String finalTopic = mqtt_topic; // Start with default "lora/incoming"
    
    if (!error && doc.containsKey("id")) {
        // 2. Extract ID and make it lowercase (e.g., "GarageTemp" -> "garagetemp")
        String id = doc["id"].as<String>();
        id.toLowerCase();
        
        // 3. Append to base topic -> "lora/incoming/garagetemp"
        finalTopic = String(mqtt_topic) + "/" + id;
    } else {
        Serial.println("JSON Parse Error or ID missing, using base topic");
    }

    // 4. Publish to the dynamic subtopic
    client.publish(finalTopic.c_str(), incoming.c_str());
    // === NEW LOGIC END ===

    wakeDisplay(5000);
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Fwd: " + finalTopic); // Show the topic we used
    display.drawStringMaxWidth(0, 15, 128, incoming);
    display.drawString(0, 35, "RSSI: " + String(LoRa.packetRssi()) + "dBm");
    drawFooter();
    display.display();
  }
}