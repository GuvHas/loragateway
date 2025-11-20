#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SSD1306.h"

// --- USER CONFIGURATION ---
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
const char* mqtt_server = "192.168.1.XXX"; // IP of your Home Assistant / MQTT Broker
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt_user";       // MQTT User (if set in HA)
const char* mqtt_pass = "mqtt_password";   // MQTT Password (if set in HA)

// Topic where we will publish the raw JSON
const char* mqtt_topic = "lora/garage/sensors";

// --- HARDWARE CONFIG (TTGO T3 v1.6.1) ---
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
  Serial.print("Connecting to ");
  Serial.println(ssid);
  display.drawString(0, 10, "WiFi: Connecting...");
  display.display();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  display.drawString(0, 20, "WiFi: OK");
  display.display();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "LoRaGateway-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      display.drawString(0, 30, "MQTT: OK");
      display.display();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Init Display
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "LoRa Gateway Init");
  display.display();

  // Init LoRa
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    display.drawString(0, 40, "LoRa Failed!");
    display.display();
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
  
  // Init Network
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

void loop() {
  // Ensure network connectivity
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Check for LoRa packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String incoming = "";
    
    // Read packet
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    Serial.print("Received packet: ");
    Serial.println(incoming);

    // Publish to Home Assistant
    // Note: We send the exact JSON string we received from the sender
    // Example payload: {"id":"GarageTemp","t":25.4,"h":50.2}
    client.publish(mqtt_topic, incoming.c_str());

    // Update Display for debug
    display.clear();
    display.drawString(0, 0, "Packet Received:");
    display.drawString(0, 15, incoming);
    display.drawString(0, 30, "RSSI: " + String(LoRa.packetRssi()));
    display.display();
  }
}
