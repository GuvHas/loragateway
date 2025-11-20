#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "SSD1306.h"

// ==========================================
//              USER CONFIGURATION
// ==========================================
const char* ssid = "YOUR_WIFI_SSID";          // <--- EDIT THIS
const char* password = "YOUR_WIFI_PASSWORD";  // <--- EDIT THIS

// MQTT Broker Settings (Home Assistant)
const char* mqtt_server = "192.168.1.XXX";    // <--- EDIT THIS (IP of Home Assistant)
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt_user";          // <--- EDIT THIS (if required)
const char* mqtt_pass = "mqtt_password";      // <--- EDIT THIS (if required)

// The topic where data will be published
const char* mqtt_topic = "lora/garage/sensors";

// ==========================================
//           HARDWARE PINS (TTGO V1.6)
// ==========================================
#define SCK_PIN  5
#define MISO_PIN 19
#define MOSI_PIN 27
#define SS_PIN   18
#define RST_PIN  23 
#define DI0_PIN  26
#define BAND 868E6 // 868E6 for Europe, 915E6 for North America

// ==========================================
//             GLOBAL OBJECTS
// ==========================================
SSD1306 display(0x3C, 21, 22);
WiFiClient espClient;
PubSubClient client(espClient);

// ==========================================
//             HELPER FUNCTIONS
// ==========================================

// Connect to WiFi
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  display.clear();
  display.drawString(0, 0, "Connecting to WiFi...");
  display.drawString(0, 15, ssid);
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
  
  display.clear();
  display.drawString(0, 0, "WiFi Connected!");
  display.drawString(0, 15, WiFi.localIP().toString());
  display.display();
}

// Connect to MQTT
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    
    // Create a random client ID so the broker doesn't reject us
    String clientId = "LoRaGateway-";
    clientId += String(random(0xffff), HEX);
    
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      display.drawString(0, 30, "MQTT Connected");
      display.display();
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      display.drawString(0, 30, "MQTT Fail! Retrying...");
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

  // 1. Init OLED
  pinMode(16, OUTPUT); // OLED Reset if needed
  digitalWrite(16, LOW);
  delay(50);
  digitalWrite(16, HIGH);
  
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);

  // 2. Init LoRa
  Serial.println("LoRa Receiver Init");
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  LoRa.setPins(SS_PIN, RST_PIN, DI0_PIN);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    display.drawString(0, 50, "LoRa Init Failed!");
    display.display();
    while (1);
  }
  Serial.println("LoRa Init OK");

  // 3. Init Network
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
}

// ==========================================
//                 LOOP
// ==========================================
void loop() {
  // 1. Ensure WiFi/MQTT is alive
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // 2. Check for LoRa packets
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // Received a packet
    String incoming = "";

    // Read packet
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    Serial.print("Received packet: ");
    Serial.println(incoming);
    Serial.print("RSSI: ");
    Serial.println(LoRa.packetRssi());

    // 3. Forward to Home Assistant via MQTT
    // We send the exact JSON string received from the sensor
    client.publish(mqtt_topic, incoming.c_str());

    // 4. Update Display
    display.clear();
    display.drawString(0, 0, "Packet Received!");
    display.drawString(0, 15, incoming);
    display.drawString(0, 30, "RSSI: " + String(LoRa.packetRssi()));
    display.drawString(0, 45, "IP: " + WiFi.localIP().toString());
    display.display();
  }
}
