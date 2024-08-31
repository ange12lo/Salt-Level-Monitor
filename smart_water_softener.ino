#include <HardwareSerial.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h" 

#define SENSOR_RX_PIN 9
#define SENSOR_TX_PIN 8


HardwareSerial SensorSerial(1);
WiFiClient espClient;
PubSubClient client(espClient);

const int BUFFER_SIZE = 10;
float distances[BUFFER_SIZE];
int distanceIndex = 0;

unsigned long lastMqttSendTime = 0;
const unsigned long mqttSendInterval = 30000; // 30 sek in Millisekunden

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32SensorClient", mqtt_username, mqtt_password)) {
      // Verbunden
    } else {
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  // Kurze Verzögerung für Stabilität
  delay(1000);
  
  SensorSerial.begin(115200, SERIAL_8N1, SENSOR_RX_PIN, SENSOR_TX_PIN);
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    distances[i] = 0;
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  static uint8_t buffer[4];
  static uint8_t bufferIndex = 0;
  
  while (SensorSerial.available()) {
    uint8_t data = SensorSerial.read();
    buffer[bufferIndex++] = data;
    
    if (bufferIndex == 4) {
      processPacket(buffer);
      bufferIndex = 0;
    }
  }

  unsigned long currentTime = millis();
  if (currentTime - lastMqttSendTime >= mqttSendInterval) {
    sendMqttMessage();
    lastMqttSendTime = currentTime;
  }
}

void processPacket(uint8_t* packet) {
  uint16_t value = (packet[1] << 8) | packet[2];
  float distance = value / 10.0;  // Umrechnung in cm

  distances[distanceIndex] = distance;
  distanceIndex = (distanceIndex + 1) % BUFFER_SIZE;

  if (Serial) {
    float avgDistance = calculateAverage();
    Serial.print("Distanz: ");
    Serial.print(distance, 2);
    Serial.print(" cm | Durchschnitt: ");
    Serial.print(avgDistance, 2);
    Serial.println(" cm");
  }
}

float calculateAverage() {
  float sum = 0;
  for (int i = 0; i < BUFFER_SIZE; i++) {
    sum += distances[i];
  }
  return sum / BUFFER_SIZE;
}

void sendMqttMessage() {
  float avgDistance = calculateAverage();

  char msg[50];
  snprintf(msg, 50, "%.2f", avgDistance);
  
  if (client.publish(mqtt_topic, msg)) {
    if (Serial) {
      Serial.print("MQTT Nachricht gesendet: ");
      Serial.println(msg);
    }
  } else {
    if (Serial) {
      Serial.println("Fehler beim Senden der MQTT Nachricht");
    }
  }
}