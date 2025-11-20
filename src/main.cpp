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
char mqtt_topic[40] = "lora/incoming";
char device_name[40] = "LoRaGateway"; // Default Friendly Name

bool shouldSaveConfig = false;

// ==========================================
//             GLOBAL OBJECTS
// ==========================================
SSD1306 display(0x3C, 21, 22);
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences; 

// ** Global WiFiManager & Parameters **
WiFiManager wm;
// Custom fields for the portal
WiFiManagerParameter custom_device_name("devname", "Device Name", "LoRaGateway", 40);
WiFiManagerParameter custom_mqtt_server("server", "MQTT Server IP", "", 40);
WiFiManagerParameter custom_mqtt_port("port", "MQTT Port", "1883", 6);
WiFiManagerParameter custom_mqtt_user("user", "MQTT User", "", 40);
WiFiManagerParameter custom_mqtt_pass("pass", "MQTT Password", "", 40);
WiFiManagerParameter custom_mqtt_topic("topic", "MQTT Topic", "lora/incoming", 40);

// Callback: Detects when user hits "Save"
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

void drawFooter() {
  display.drawLine(0, 52, 128, 52); 
  display.setFont(ArialMT_Plain_10);
  
  // Show IP
  display.drawString(0, 54, WiFi.localIP().toString());
  
  // Show Signal
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
    
    int port = atoi(mqtt_port);
    client.setServer(mqtt_server, port);
    
    String clientId = String(device_name) + "-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
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

  // 1. LOAD SETTINGS FROM MEMORY
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
  
  // 2. APPLY HOSTNAME (Must be done before WiFi connects)
  WiFi.setHostname(device_name);

  // 3. INJECT VALUES INTO WEB PORTAL
  custom_mqtt_server.setValue(mqtt_server, 40);
  custom_mqtt_port.setValue(mqtt_port, 6);
  custom_mqtt_user.setValue(mqtt_user, 40);
  custom_mqtt_pass.setValue(mqtt_pass, 40);
  custom_mqtt_topic.setValue(mqtt_topic, 40);
  custom_device_name.setValue(device_name, 40);

  // 4. START LORA
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  if (!LoRa.begin(BAND)) {
    Serial.println("LoRa Failed!");
    display.drawString(0, 15, "LoRa Hardware Fail!");
    display.display();
    while (1) delay(1000);
  }

  // 5. CONFIGURE WIFI MANAGER
  wm.setConfigPortalBlocking(false);
  wm.setSaveConfigCallback(saveConfigCallback);

  // Add parameters (Order matters for display)
  wm.addParameter(&custom_device_name); // <--- Added here
  wm.addParameter(&custom_mqtt_server);
  wm.addParameter(&custom_mqtt_port);
  wm.addParameter(&custom_mqtt_user);
  wm.addParameter(&custom_mqtt_pass);
  wm.addParameter(&custom_mqtt_topic);

  // Info on screen
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

  // 6. READY STATE
  client.setServer(mqtt_server, atoi(mqtt_port));
  
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, "Gateway Ready");
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 20, "Name: " + String(device_name));
  display.display();
}

// ==========================================
//                 LOOP
// ==========================================
void loop() {
  wm.process();

  // HANDLE SETTINGS CHANGE
  if (shouldSaveConfig) {
    shouldSaveConfig = false;
    
    // Check if name changed to trigger a WiFi refresh
    String new_name = String(custom_device_name.getValue());
    bool nameChanged = (new_name != String(device_name));

    strcpy(mqtt_server, custom_mqtt_server.getValue());
    strcpy(mqtt_port, custom_mqtt_port.getValue());
    strcpy(mqtt_user, custom_mqtt_user.getValue());
    strcpy(mqtt_pass, custom_mqtt_pass.getValue());
    strcpy(mqtt_topic, custom_mqtt_topic.getValue());
    strcpy(device_name, custom_device_name.getValue());

    Serial.println("Saving config...");
    preferences.putString("server", mqtt_server);
    preferences.putString("port", mqtt_port);
    preferences.putString("user", mqtt_user);
    preferences.putString("pass", mqtt_pass);
    preferences.putString("topic", mqtt_topic);
    preferences.putString("devname", device_name);
    
    // Disconnect MQTT to force reconnect with new settings
    client.disconnect(); 

    // If name changed, restart WiFi to apply new Hostname
    if (nameChanged) {
      Serial.println("Name changed, restarting WiFi...");
      WiFi.setHostname(device_name);
      // A brief disconnect/reconnect cycle
      WiFi.disconnect(); 
      WiFi.reconnect();
    }
  }

  // MANAGE WIFI & MQTT
  if (WiFi.status() == WL_CONNECTED) {
      if (!client.connected()) {
        reconnect();
        
        // Update Screen
        display.clear();
        display.setFont(ArialMT_Plain_16);
        display.drawString(0, 0, "Gateway Ready");
        display.setFont(ArialMT_Plain_10);
        display.drawString(0, 20, "Web IP:");
        display.drawString(0, 30, WiFi.localIP().toString());
        drawFooter();
        display.display();
      }
      client.loop();
  }

  // LISTEN FOR LORA
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    
    Serial.print("RX: ");
    Serial.println(incoming);

    client.publish(mqtt_topic, incoming.c_str());

    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "Packet Forwarded!");
    display.drawStringMaxWidth(0, 15, 128, incoming);
    display.drawString(0, 35, "RSSI: " + String(LoRa.packetRssi()) + "dBm");
    
    drawFooter();
    display.display();
  }
}