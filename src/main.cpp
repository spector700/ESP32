#include "lights.h"
#include <Adafruit_PWMServoDriver.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <WiFiManager.h>

#include <ESPAsyncWebServer.h>

#define DEBUG_SERIAL

#if defined(DEBUG_SERIAL)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

bool MQTT_HA_AUTO_DISCOVERY = false;

#define MQTT_CONNECTION_TIMEOUT 5000

#define AP_SSID "ESP-Apollo-LUT"
#define MQTT_SERVER "192.168.1.10"
#define MQTT_USER "mqtt"
#define MQTT_PASS "pass"

LightManager lightManager;

const int onboardLed = 2;

// Mqtt client
WiFiClient espClient;
PubSubClient client(espClient);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
// you can also call it with a different address you want
// Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

Light *floor60 = lightManager.addLight("Floor 60", 1, &pwm);

AsyncWebServer server(80);

///////////////////////////////////////////////////////////////////////////
//   MQTT
///////////////////////////////////////////////////////////////////////////

/*
  Function called to publish to a MQTT topic with the given payload
*/
void publishToMQTT(const char *topic, const char *payload, bool retained) {
  if (client.publish(topic, payload, retained)) {
    DEBUG_PRINT(F("INFO: MQTT message published successfully, topic: "));
    DEBUG_PRINTLN(topic);
    DEBUG_PRINT(F(", payload: "));
    DEBUG_PRINTLN(payload);
  } else {
    DEBUG_PRINTLN(F("ERROR: MQTT message not published, either connection "
                    "lost, or message too large. Topic: "));
    DEBUG_PRINTLN(topic);
    DEBUG_PRINTLN("payload size: " + String(strlen(payload)));
    DEBUG_PRINT(F(" , payload: "));
    DEBUG_PRINTLN(payload);
  }
}

/*
  Function called to subscribe to a MQTT topic
*/
void subscribeToMQTT(const char *topic) {
  if (client.subscribe(topic)) {
    DEBUG_PRINT(F("INFO: Sending the MQTT subscribe succeeded for topic: "));
    DEBUG_PRINTLN(topic);
  } else {
    DEBUG_PRINT(F("ERROR: Sending the MQTT subscribe failed for topic: "));
    DEBUG_PRINTLN(topic);
  }
}

/*
  Function called to setup the MQTT auto discovery
*/
void setupHaDiscovery() {
  char topic[50];
  snprintf(topic, sizeof(topic), "homeassistant/device/%s/config",
           lightManager.getDeviceId().c_str());

  if (MQTT_HA_AUTO_DISCOVERY) {
    char buffer[2048];
    JsonDocument doc;
    doc.clear();

    // Json payload for the device
    JsonObject device = doc["device"].to<JsonObject>();
    device["ids"] = lightManager.getDeviceId().c_str();
    device["name"] = "Apollo Launch Tower";
    device["mf"] = "Apollo";
    device["mdl"] = "ESP32";
    device["cu"] = String("http://") + WiFi.localIP().toString() +
                   "/"; // web interface for device,
    JsonObject origin = doc["o"].to<JsonObject>();
    origin["name"] = "Apollo Launch Tower";
    origin["url"] = String("http://") + WiFi.localIP().toString() +
                    "/"; // web interface for device,

    // Json payload for the entity
    JsonObject components = doc["cmps"].to<JsonObject>();
    JsonObject all =
        components[lightManager.getAllLightsUid()].to<JsonObject>();
    all["p"] = "light";
    all["name"] = lightManager.getAllLightsName().c_str();
    all["uniq_id"] = lightManager.getAllLightsUid().c_str();
    all["icon"] = "mdi:lightbulb-group";
    all["stat_t"] = lightManager.getAllLightsStatTopic().c_str();
    all["cmd_t"] = lightManager.getAllLightsCmndTopic().c_str();
    all["brightness"] = "true";
    all["bri_scl"] = "100";
    all["bri_stat_t"] = strcpy(
        (char *)lightManager.getAllLightsStatTopic().c_str(), "/brightness");
    all["bri_cmd_t"] = strcpy(
        (char *)lightManager.getAllLightsCmndTopic().c_str(), "/brightness");

    for (const auto &light : lightManager.getAllLights()) {
      JsonObject docEnt = components[light->getUid()].to<JsonObject>();
      docEnt["p"] = "light";
      docEnt["dev-cla"] = "switch";
      docEnt["name"] = light->getName().c_str();
      docEnt["uniq_id"] = light->getUid().c_str();
      docEnt["icon"] = "mdi:light-flood-down";
      docEnt["stat_t"] = light->getStatTopic().c_str();
      docEnt["cmd_t"] = light->getCmndTopic().c_str();
    }

    if (serializeJson(doc, buffer) == 0) {
      DEBUG_PRINTLN(F("ERROR: Failed to serialize JSON"));
      return;
    }
    publishToMQTT(topic, buffer, true);
  } else {
    // remove all entities by publishing empty payloads (with retained flag)
    publishToMQTT(topic, "", true);
  }
}

/**
 * @brief Attempts to connect to the MQTT broker.
 *
 * This function checks if the MQTT client is connected. If not, it attempts to
 * establish a connection using the provided credentials. If the connection is
 * successful, it subscribes to the specified MQTT topic. If the connection
 * fails, it prints error messages and retries after a delay.
 *
 * @note The function uses a timeout mechanism to avoid frequent connection
 * attempts.
 */
static unsigned long lastMQTTConnection = 0;
void connectMqtt() {
  // Only attempt to connect if:
  // 1. Client is not connected
  // 2. Enough time has passed since last attempt
  if (!client.connected() &&
      ((millis() < lastMQTTConnection) || // Handle overflow
       (millis() - lastMQTTConnection >= MQTT_CONNECTION_TIMEOUT))) {
    DEBUG_PRINTLN("Attempting MQTT connection...");
    if (client.connect(AP_SSID, MQTT_USER, MQTT_PASS)) {
      DEBUG_PRINTLN(F("INFO: Successfully connected to MQTT broker"));

      subscribeToMQTT(lightManager.getAllLightsCmndTopic().c_str());
      subscribeToMQTT(floor60->getCmndTopic().c_str());
    } else {
      DEBUG_PRINTLN(F("ERROR: MQTT connection failed"));
      DEBUG_PRINT(F("INFO: Broker: "));
      DEBUG_PRINTLN(MQTT_SERVER);
      DEBUG_PRINT(F("INFO: Error code: "));
      DEBUG_PRINTLN(client.state());
      DEBUG_PRINTLN(F("Will retry in 5 seconds"));
      delay(5000);
    }
    // Update last connection attempt timestamp
    lastMQTTConnection = millis();
  }
}

/*
   Function called when a MQTT message has arrived
   @param topic   The topic of the MQTT message
   @param payload The payload of the MQTT message
   @param length  The length of the payload
*/
void handleMqttMessage(char *topic, byte *payload, unsigned int length) {
  DEBUG_PRINT("INFO: Message arrived [");
  DEBUG_PRINT(topic);
  DEBUG_PRINT("]: ");

  String message;
  for (int i = 0; i < length; i++) {
    message.concat((char)payload[i]);
  }
  DEBUG_PRINTLN(message);

  for (const auto &light : lightManager.getAllLights()) {
    DEBUG_PRINTLN("INFO: Checking topic: " +
                  String(light->getCmndTopic().c_str()));
    if (strcmp(topic, light->getCmndTopic().c_str()) == 0) {
      light->setState((message == "ON"));
      break;
    } else {
      DEBUG_PRINTLN(strcat("WARNING: No light found for topic: ", topic));
    }
  }
}

///////////////////////////////////////////////////////////////////////////
//   WiFi
///////////////////////////////////////////////////////////////////////////

/*
   Function called to setup the connection to the WiFi AP
*/
void setupWiFi() {
  DEBUG_PRINT(F("INFO: WiFi connecting to: "));
  DEBUG_PRINTLN(AP_SSID);

  delay(10);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // Make sure the led is off
  pinMode(onboardLed, OUTPUT);
  digitalWrite(onboardLed, LOW);

  // WiFiManager, Local initialization.
  WiFiManager wm;

  // reset settings - wipe stored credentials for testing
  // these are stored by the esp library
  // wm.resetSettings();

  if (!wm.autoConnect(AP_SSID, NULL)) {
    Serial.println("Failed to connect");

    // reset and try again, or maybe put it to deep sleep
    delay(3000);
    ESP.restart();
  } else {
    // if you get here you have connected to the WiFi
    digitalWrite(onboardLed, HIGH);
    WiFi.hostname(AP_SSID);

    client.setServer(MQTT_SERVER, 1883);
    // Increase the buffer size to handle larger messages
    client.setBufferSize(2048);
    client.setCallback(handleMqttMessage);

    // Handle web callbacks for enabling or disabling discovery
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hello, world");
    });
    server.on("/discovery_on", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/html",
                    "<h1>Discovery ON...<h1><h3>Home Assistant MQTT Discovery "
                    "enabled</h3>");
      delay(200);
      MQTT_HA_AUTO_DISCOVERY = true;
      setupHaDiscovery();
    });
    server.on("/discovery_off", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/html",
                    "<h1>Discovery OFF...<h1><h3>Home Assistant MQTT Discovery "
                    "disabled. Previous entities removed.</h3>");
      delay(200);
      MQTT_HA_AUTO_DISCOVERY = false;
      setupHaDiscovery();
    });
    server.begin();
  }
}

void setup() {
#if defined(DEBUG_SERIAL)
  Serial.begin(921600);
#endif

  setupWiFi();

  pwm.begin();
  pwm.setPWMFreq(1600); // This is the maximum PWM frequency for LED's
}

void loop() {
  if (!client.connected()) {
    connectMqtt();
  }
  client.loop();
}
