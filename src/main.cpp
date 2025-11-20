#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SSD1306.h"
#include <WiFiManager.h> 
#include <Preferences.h> 

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
// Default topic is now GENERIC to allow multiple sensors
char mqtt_topic[40] = "lora/incoming"; 

bool shouldSaveConfig = false;

// ==========================================
//             GLOBAL OBJECTS
// ==========================================
SSD1306 display(0x3C, 21, 22);
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences; 

// Callback notifying us the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
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

// Helper to draw the footer with IP and Signal strength
void drawFooter() {
  display.drawLine(0, 52, 128, 52); // Separator line
  display.setFont(ArialMT_Plain_10);
  
  // Show IP
  display.drawString(0, 54, WiFi.localIP().toString());
  
  // Show Signal Bars (Simple visual)
  long rssi = WiFi.RSSI();
  int bars = 0;
  if (rssi > -55) bars = 4;
  else if (rssi > -65) bars = 3;
  else if (rssi > -75) bars = 2;
  else bars = 1;
  
  String signalStr = "WiFi: " + String(bars) + "/4";
  // Right align the signal strength
  int strWidth = display.getStringWidth(signalStr);
  display.drawString(128 - strWidth, 54, signalStr);
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    
    String clientId = "LoRaGW-" + String(random(0xffff), HEX);
    int port = atoi(mqtt_port);
    client.setServer(mqtt_server, port);

    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      // Once connected, we stay silent on screen until data arrives
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5s");
      
      // Show error on screen
      display.clear();
      display.drawString(0, 0, "MQTT Error!");
      display.drawString(0, 15, "State: " + String(client.state()));
      drawFooter();
      display.display();
      delay(5000);
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

  // 1. LOAD SETTINGS
  preferences.begin("loraconf", false);
  String s_server = preferences.getString("server", "");
  String s_port = preferences.getString("port", "1883");
  String s_user = preferences.getString("user", "");
  String s_pass = preferences.getString("pass", "");
  String s_topic = preferences.getString("topic", "lora/incoming"); // Generic Default

  s_server.toCharArray(mqtt_server, 40);
  s_port.toCharArray(mqtt_port, 6);
  s_user.toCharArray(mqtt_user, 40);
  s_pass.toCharArray(mqtt_pass, 40);
  s_topic.toCharArray(mqtt_topic, 40);

  // 2. START LORA
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  if (!LoRa.begin(BAND)) {
    Serial.println("LoRa Failed!");
    display.drawString(0, 15, "LoRa Hardware Fail!");
    display.display();
    while (1) delay(1000);
  }

  // 3. WIFI MANAGER
  WiFiManager wm;
  wm.setSaveConfigCallback(saveConfigCallback);

  // Custom Parameters
  WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "MQTT User", mqtt_user, 40);
  WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", mqtt_pass, 40);
  WiFiManagerParameter custom_mqtt_topic("topic", "MQTT Topic", mqtt_topic, 40);

  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.addParameter(&custom_mqtt_topic);

  // Info Screen
  display.clear();
  display.drawString(0, 0, "Setup Mode");
  display.drawString(0, 15, "Connect to WiFi:");
  display.drawString(0, 25, "AP: LoRaGateway-Setup");
  display.display();

  wm.setConfigPortalTimeout(180); 
  
  if (!wm.autoConnect("LoRaGateway-Setup")) {
    Serial.println("timeout, restarting");
    delay(3000);
    ESP.restart(); 
  }

  // 4. SAVE CONFIG
  if (shouldSaveConfig) {
    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_user, custom_mqtt_user.getValue());
    strcpy(mqtt_pass, custom_mqtt_pass.getValue());
    strcpy(mqtt_topic, custom_mqtt_topic.getValue());

    preferences.putString("server", mqtt_server);
    preferences.putString("port", mqtt_port);
    preferences.putString("user", mqtt_user);
    preferences.putString("pass", mqtt_pass);
    preferences.putString("topic", mqtt_topic);
  }
  preferences.end();

  // 5. READY STATE
  client.setServer(mqtt_server, atoi(mqtt_port));

  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Gateway Ready");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, "Listening on 868MHz");
  display.drawString(0, 30, "Dest: " + String(mqtt_topic));
  drawFooter();
  display.display();
}

// ==========================================
//                 LOOP
// ==========================================
void loop() {
  // 1. Network Maintenance
  if (!client.connected()) {
    reconnect();
    // Redraw "Ready" screen after reconnecting
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.drawString(0, 0, "Gateway Ready");
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 20, "Listening...");
    drawFooter();
    display.display();
  }
  client.loop();

  // 2. LoRa Handling
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    
    Serial.print("RX: ");
    Serial.println(incoming);

    // Send to MQTT
    client.publish(mqtt_topic, incoming.c_str());

    // Update Display with Data
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Packet Forwarded!");
    
    // Display the raw JSON so you can see which sensor sent it
    // Wrap text is automatic in some libs, but we truncate to fit
    display.drawStringMaxWidth(0, 15, 128, incoming);
    
    display.drawString(0, 35, "RSSI: " + String(LoRa.packetRssi()) + "dBm");
    
    drawFooter(); // Always draw the footer with IP
    display.display();
  }
}