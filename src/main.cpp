#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SSD1306.h"

// ==========================================
//              USER CONFIGURATION
// ==========================================
const char* ssid = "YOUR_SSID";          // <--- EDIT THIS
const char* password = "YOUR_WIFI_PASSWORD";  // <--- EDIT THIS
const char* mqtt_server = "192.168.1.xxx";    // <--- EDIT THIS
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt_user";          // <--- EDIT THIS (or leave blank)
const char* mqtt_pass = "mqtt_password";      // <--- EDIT THIS (or leave blank)
const char* mqtt_topic = "lora/sensors";

// ==========================================
//           HARDWARE PINS
// ==========================================
#define SCK_PIN  5
#define MISO_PIN 19
#define MOSI_PIN 27
#define SS_PIN   18
#define RST_PIN  23 
#define DI0_PIN  26
#define BAND 868E6 

// Objects
SSD1306 display(0x3C, 21, 22);
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting WiFi");
  WiFi.begin(ssid, password);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    retries++;
    if (retries > 30) { 
      Serial.println("\nWiFi Failed! Continuing in Offline Mode...");
      return; 
    }
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "LoRaGateway-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5s");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== BOOTING GATEWAY v3.0 ===");

  // 1. Init LoRa (Moved FIRST to ensure radio works)
  Serial.print("Step 1: Init LoRa...");
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  
  if (!LoRa.begin(BAND)) {
    Serial.println(" FAILED!");
    // If LoRa fails, we loop here safely
    while (1) {
      Serial.println("LoRa Init Failed - Check Pins!");
      delay(2000);
    }
  }
  Serial.println(" OK!");

  // 2. Init Network
  Serial.print("Step 2: Init WiFi...");
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  // 3. Init OLED (Moved LAST and simplified)
  // We removed the Pin 16 toggle causing the crash
  Serial.print("Step 3: Init OLED...");
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.drawString(0, 0, "Gateway Running");
  display.drawString(0, 15, "WiFi: " + WiFi.localIP().toString());
  display.drawString(0, 30, "Waiting for Data...");
  display.display();
  Serial.println(" OK!");
  
  Serial.println("=== SYSTEM READY ===");
}

void loop() {
  // 1. Keep WiFi/MQTT Alive
  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
  }

  // 2. Listen for LoRa
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    
    // Debug to Serial
    Serial.print("RX Packet: ");
    Serial.println(incoming);
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());

    // Send to Home Assistant
    if (client.connected()) {
      client.publish(mqtt_topic, incoming.c_str());
    }

    // Update Screen
    display.clear();
    display.drawString(0, 0, "RX Data:");
    display.drawString(0, 15, incoming);
    display.drawString(0, 35, "RSSI: " + String(LoRa.packetRssi()));
    display.display();
  }
}