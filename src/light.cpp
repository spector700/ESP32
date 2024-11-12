#include "light.h"
#include "utils.h"
#include <Adafruit_PWMServoDriver.h>

Light::Light(const String &lightName, uint8_t lightPin, const String &deviceId,
             Adafruit_PWMServoDriver &pwm)
    : m_name(lightName), m_pin(lightPin), m_state(false), m_pwm(&pwm) {

  const String id_name = convertToSnakeCase(m_name);
  m_uid = id_name + "_" + deviceId;

  // Create MQTT topics
  m_cmnd_topic = CMND_TOPIC + id_name + "/light";
  m_stat_topic = STAT_TOPIC + id_name + "/light";
}

const String &Light::getName() const { return m_name; }
const String &Light::getUid() const { return m_uid; }
const String &Light::getCmndTopic() const { return m_cmnd_topic; }
const String &Light::getStatTopic() const { return m_stat_topic; }
uint8_t Light::getPin() const { return m_pin; }
bool Light::getState() const { return m_state; }

void Light::setState(bool newState) {
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

void Light::toggle() { setState(!m_state); }