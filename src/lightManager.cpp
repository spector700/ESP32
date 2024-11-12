#include "lightManager.h"
#include "utils.h"

LightManager::LightManager()
    : m_name_all_lights("All Lights"), m_state_all_lights(false),
      device_id(generateDeviceId()) {

  const std::string id_name = convertToSnakeCase(m_name_all_lights);
  m_uid_all_lights = id_name + "_" + device_id;

  // Create MQTT topics
  m_cmnd_topic_all_lights = CMND_TOPIC + id_name + "/light";
  m_stat_topic_all_lights = STAT_TOPIC + id_name + "/light";
}

Light *LightManager::addLight(const std::string &name, uint8_t pin,
                              Adafruit_PWMServoDriver *pwm) {
  auto light = std::unique_ptr<Light>(new Light(name, pin, device_id, *pwm));
  Light *lightPtr = light.get();
  lights.push_back(std::move(light));
  return lightPtr;
}

Light *LightManager::findLight(const std::string &cmndTopic) {
  for (const auto &light : lights) {
    if (light->getCmndTopic() == cmndTopic) {
      return light.get();
    }
  }
  return nullptr;
}

const std::string &LightManager::getAllLightsName() const {
  return m_name_all_lights;
}
const std::string &LightManager::getAllLightsUid() const {
  return m_uid_all_lights;
}
const std::string &LightManager::getAllLightsCmndTopic() const {
  return m_cmnd_topic_all_lights;
}
const std::string &LightManager::getAllLightsStatTopic() const {
  return m_stat_topic_all_lights;
}
bool LightManager::getAllLightsState() const { return m_state_all_lights; }
const std::string &LightManager::getDeviceId() const { return device_id; }

const std::vector<std::unique_ptr<Light>> &LightManager::getAllLights() const {
  return lights;
}