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
  String m_name;
  String m_uid;
  String m_cmnd_topic;
  String m_stat_topic;
  Adafruit_PWMServoDriver *m_pwm;
  uint8_t m_pin;
  bool m_state;

public:
  Light(const String &lightName, uint8_t lightPin, const String &deviceId,
        Adafruit_PWMServoDriver &pwm);

  // Getters
  const String &getName() const;
  const String &getUid() const;
  const String &getCmndTopic() const;
  const String &getStatTopic() const;
  uint8_t getPin() const;
  bool getState() const;

  // State control
  void setState(bool newState);

  void toggle();
};
