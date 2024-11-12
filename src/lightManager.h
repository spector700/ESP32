#pragma once

#include "light.h"
#include <Arduino.h>
#include <memory>
#include <vector>

class LightManager {
private:
  String device_id;
  std::vector<std::unique_ptr<Light>> lights;
  String m_name_all_lights;
  String m_uid_all_lights;
  String m_cmnd_topic_all_lights;
  String m_stat_topic_all_lights;
  bool m_state_all_lights;

public:
  LightManager();

  /**
   * @brief Adds a new light to the collection
   * @param name The name of the light
   * @param pin The pwm board pin for the light
   * @param pwm The PWM driver for the light
   * @return A pointer to the newly created Light object
   */
  Light *addLight(const String &name, uint8_t pin,
                  Adafruit_PWMServoDriver *pwm);

  /**
   * @brief Finds a light by its MQTT command topic
   * @param cmndTopic The MQTT command topic of the light
   * @return A pointer to the found Light object, or nullptr if not found
   */
  Light *findLight(const String &cmndTopic);

  // Getters
  const String &getAllLightsName() const;
  const String &getAllLightsUid() const;
  const String &getAllLightsCmndTopic() const;
  const String &getAllLightsStatTopic() const;
  bool getAllLightsState() const;
  const String &getDeviceId() const;

  /**
   * @brief Returns a reference to the collection of lights
   * @return A reference to the vector of Light objects
   */
  const std::vector<std::unique_ptr<Light>> &getAllLights() const;
};