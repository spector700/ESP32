#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

const char *ap_ssid = "ESP-Apollo-LUT";
const char *mqtt_server = "192.168.1.100";
const char *mqtt_user = "mqtt";
const char *mqtt_password = "pass";

const int onboardLed = 2;
String onboardLedState;

// Mqtt client
WiFiClient espClient;
PubSubClient client(espClient);

const char *floor60_topic = "LUT/lights/floor60";

bool floor60_state = false;
const int floor60_pin = 26;

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("APOLLO-MQTT-ESP", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      client.subscribe(floor60_topic);

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print(message);
  Serial.println();

  if (strcmp(topic, floor60_topic) == 0) {
    floor60_state = (message == "ON");
    digitalWrite(26, floor60_state ? HIGH : LOW);
  }
}

void setup() {
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP

  Serial.begin(921600);

  // Make sure the led is off
  pinMode(onboardLed, OUTPUT);
  digitalWrite(onboardLed, LOW);

  // WiFiManager, Local intialization. Once its business is done, there is no
  // need to keep it around
  WiFiManager wm;

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  // wm.resetSettings();

  if (!wm.autoConnect(ap_ssid, NULL)) {
    Serial.println("Failed to connect");
    ESP.restart();
  } else {
    // if you get here you have connected to the WiFi
    digitalWrite(onboardLed, HIGH);
    Serial.println("connected to: " + WiFi.SSID());
    Serial.println();
    Serial.println("Connecting to MQTT server...");
    Serial.println();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  }

  pinMode(floor60_pin, OUTPUT);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
