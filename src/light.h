#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <string>

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
  Light(const std::string &lightName, uint8_t lightPin,
        const std::string &deviceId, Adafruit_PWMServoDriver &pwm);

  // Getters
  const std::string &getName() const;
  const std::string &getUid() const;
  const std::string &getCmndTopic() const;
  const std::string &getStatTopic() const;
  uint8_t getPin() const;
  bool getState() const;

  // State control
  void setState(bool newState);

  void toggle();
};
