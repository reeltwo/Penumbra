# Minimal drive system for the ESP32 #

This sketch provides a basic drive system running on the ESP32. It depends on the [Reeltwo](https://reeltwo.github.io/Reeltwo) library. It is configured to use a single PS3 Navigation controller, but you can also use a regular PS3 or PS4 controller with minimal changes.

# Default Pinout #

Serial pins used to communicate (if using serial)
   #define SERIAL1_RX_PIN 4
   #define SERIAL1_TX_PIN 2

   #define LEFT_MOTOR_PWM      23
   #define RIGHT_MOTOR_PWM     22

Optional for Roboteq:
If running a Microbasic script to adjust the max RPM you can define:

   #define THROTTLE_MOTOR_PWM  21
