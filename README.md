# ESP32 Apollo LUT Controller

This project provides firmware for controlling LEDs and servos for the [3D printed Apollo Launch Umbilical Tower (LUT)](https://www.printables.com/model/621901-apollo-umbilical-tower-new).It is controlled with MQTT and home assistant.

If not connected to wifi, It will create a access point for you to connect to and enter your wifi details.

Each floor, arm and crane lights are wired in series. I am using PCA9685 PWM driver boards to connect each of those for brightness control.

## Hardware Requirements

- ESP32 development board
- PCA9685 PWM driver boards
- 3mm LED's
- 5v Power supply
- Servos (I dont know yet)

## Setup

1. Clone this repository
2. Configure `config.h` with your MQTT settings
3. Flash the firmware to your ESP32
4. Connect to the access point and enter Wifi
5. Add the device to Home Assistant
