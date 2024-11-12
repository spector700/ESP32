#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>
#include <cstdio>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#define DEBUG_SERIAL

#if defined(DEBUG_SERIAL)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

#define CMND_TOPIC "cmnd/apollo_lut/"
#define STAT_TOPIC "stat/apollo_lut/"

enum EFFECTS { DEFUALT, FADE, BLINK, FLASH };

// Helper function to convert to snake case
std::string convertToSnakeCase(std::string str) {
  // Convert to lowercase
  std::transform(str.begin(), str.end(), str.begin(), ::tolower);
  // Replace spaces with underscores
  std::replace(str.begin(), str.end(), ' ', '_');
  return str;
}

class Light {
private:
  std::string m_name;
  std::string m_uid;
  std::string m_cmnd_topic;
  std::string m_stat_topic;
  Adafruit_PWMServoDriver *m_pwm;
  uint8_t m_pin;
  bool m_state;

public:
  Light(const std::string &lightName, uint8_t &lightPin,
        const std::string &deviceId, Adafruit_PWMServoDriver &pwm)
      : m_name(lightName), m_pin(lightPin), m_state(false), m_pwm(&pwm) {

    const std::string id_name = convertToSnakeCase(m_name);
    m_uid = id_name + "_" + deviceId;

    // Create MQTT topics
    m_cmnd_topic = CMND_TOPIC + id_name + "/light";
    m_stat_topic = STAT_TOPIC + id_name + "/light";
  }

  // Getters
  const std::string &getName() const { return m_name; }
  const std::string &getUid() const { return m_uid; }
  const std::string &getCmndTopic() const { return m_cmnd_topic; }
  const std::string &getStatTopic() const { return m_stat_topic; }
  uint8_t getPin() const { return m_pin; }
  bool getState() const { return m_state; }

  // State control
  void setState(bool newState) {
    m_state = newState;
    DEBUG_PRINTLN("Name: " + String(m_name.c_str()));
    DEBUG_PRINTLN("Setting state to: " + String((newState) ? "ON" : "OFF"));
    DEBUG_PRINTLN("Pin: " + String(m_pin));

    const uint16_t MAX_PWM = 4095;
    const uint16_t STEP = 16;

    /*
       Function called to fade the lights on or off
       @param pin     The pin to fade, 16 for all lights
       @param message The message to determine if the lights should be faded on
       or off
    */
    if (newState) {
      DEBUG_PRINTLN("Turning on");
      for (uint16_t brightness = 0; brightness <= MAX_PWM; brightness += STEP) {
        m_pwm->setPWM(m_pin, 0, brightness);
        delay(1);
      }

      // Ensure fully on
      m_pwm->setPWM(m_pin, 0, 4095);
    } else {
      DEBUG_PRINTLN("Turning off");
      for (uint16_t brightness = MAX_PWM; brightness >= STEP;
           brightness -= STEP) {
        m_pwm->setPWM(m_pin, 0, brightness);
        delay(1);
      }

      // Ensure fully off
      m_pwm->setPWM(m_pin, 0, 0);
    }
  }

  void toggle() { setState(!m_state); }
};

class LightManager {
private:
  std::string device_id;
  std::vector<std::unique_ptr<Light>> lights;
  std::string m_name_all_lights;
  std::string m_uid_all_lights;
  std::string m_cmnd_topic_all_lights;
  std::string m_stat_topic_all_lights;
  bool m_state_all_lights;

  std::string generateDeviceId() {
    uint64_t chipId = ESP.getEfuseMac(); // Get chip ID
    std::stringstream ss;
    ss << "lut_" << std::hex << std::setfill('0') << std::setw(8)
       << (chipId & 0xFFFFFF); // Take last 8 digits
    return ss.str();
  }

public:
  LightManager()
      : m_name_all_lights("All Lights"), m_state_all_lights(false),
        device_id(generateDeviceId()) {

    const std::string id_name = convertToSnakeCase(m_name_all_lights);
    m_uid_all_lights = id_name + "_" + device_id;

    // Create MQTT topics
    m_cmnd_topic_all_lights = CMND_TOPIC + id_name + "/light";
    m_stat_topic_all_lights = STAT_TOPIC + id_name + "/light";
  }

  /**
   * @brief Adds a new light to the collection
   * @param name The name of the light
   * @param pin The pwm board pin for the light
   * @param pwm The PWM driver for the light
   * @return A pointer to the newly created Light object
   */
  Light *addLight(const std::string &name, uint8_t pin,
                  Adafruit_PWMServoDriver *pwm) {
    auto light = std::unique_ptr<Light>(new Light(name, pin, device_id, *pwm));
    Light *lightPtr = light.get();
    lights.push_back(std::move(light));
    return lightPtr;
  }

  /**
   * @brief Finds a light by its MQTT command topic
   * @param cmndTopic The MQTT command topic of the light
   * @return A pointer to the found Light object, or nullptr if not found
   */
  Light *findLight(const std::string &cmndTopic) {
    for (const auto &light : lights) {
      if (light->getCmndTopic() == cmndTopic) {
        return light.get();
      }
    }
    return nullptr;
  }
  // Getters
  const std::string &getAllLightsName() const { return m_name_all_lights; }
  const std::string &getAllLightsUid() const { return m_uid_all_lights; }
  const std::string &getAllLightsCmndTopic() const {
    return m_cmnd_topic_all_lights;
  }
  const std::string &getAllLightsStatTopic() const {
    return m_stat_topic_all_lights;
  }
  bool getAllLightsState() const { return m_state_all_lights; }
  const std::string &getDeviceId() const { return device_id; }

  /**
   * @brief Returns a reference to the collection of lights
   * @return A reference to the vector of Light objects
   */
  const std::vector<std::unique_ptr<Light>> &getAllLights() const {
    return lights;
  }
};