#ifndef __PENUMBRA_VARIABLE_SETTINGS_H__
#define __PENUMBRA_VARIABLE_SETTINGS_H__

// Define additional settings based on the drive system of choice.

#if DRIVE_SYSTEM == DRIVE_SYSTEM_SABER
  #if DOME_DRIVE == DOME_DRIVE_SABER
    #define DOME_DRIVE_SERIAL    Serial1
  #endif
  #define CHANNEL_MIXING       true   // set to true premix channels before sending commands
  #define DRIVE_BAUD_RATE      9600
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_PWM
  #define NEED_DRIVE_PWM_PINS
  #define CHANNEL_MIXING       false  // set to false if motor controller will mix the channels
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM
  #define NEED_DRIVE_PWM_PINS
  #define CHANNEL_MIXING       false  // set to false if motor controller will mix the channels
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_SERIAL
  #define DRIVE_BAUD_RATE      115200
  #define CHANNEL_MIXING       false  // set to false if motor controller will mix the channels
#elif DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL
  #define NEED_DRIVE_PWM_PINS
  #define DRIVE_BAUD_RATE      115200
  #define CHANNEL_MIXING       false  // set to false if motor controller will mix the channels
#else
  #error Unsupported DRIVE_SYSTEM
#endif

// Baud rates

#ifndef DRIVE_BAUD_RATE
  #define DRIVE_BAUD_RATE      9600
#endif

#ifndef MARCDUINO_BAUD_RATE
  #define MARCDUINO_BAUD_RATE  9600
#endif

// Gesture settings

#ifndef MAX_GESTURE_LENGTH
  #define MAX_GESTURE_LENGTH   20
#endif

#ifndef GESTURE_TIMEOUT_MS
  #define GESTURE_TIMEOUT_MS   2000
#endif

// Set the dome serial address when using Syren10 for
// the dome but not using Sabertooth for the drive.

#if DOME_DRIVE == DOME_DRIVE_SABER && !defined(DOME_DRIVE_SERIAL)
  #define DOME_DRIVE_SERIAL    Serial2
#endif

// Create some helper definitions.

#if DRIVE_SYSTEM == DRIVE_SYSTEM_PWM || \
    DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM || \
    DRIVE_SYSTEM == DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL
  #define NEED_DRIVE_PWM_PINS
#endif

#if DOME_DRIVE == DOME_DRIVE_PWM
  #define NEED_DOME_PWM_PINS
#endif

// Allow MY_BT_ADDR and DRIVE_STICK_BT_ADDR to be undefined above.

#ifndef MY_BT_ADDR
  #define MY_BT_ADDR nullptr
#endif
#ifndef DRIVE_STICK_BT_ADDR
  #define DRIVE_STICK_BT_ADDR nullptr
#endif
#ifndef DOME_STICK_BT_ADDR
  #define DOME_STICK_BT_ADDR nullptr
#endif

// Set PWM index and settings.

#ifdef NEED_DRIVE_PWM_PINS
  #define LEFT_MOTOR_PWM_INDEX  0
  #define RIGHT_MOTOR_PWM_INDEX 1
  #ifdef THROTTLE_MOTOR_PWM
    #define THROTTLE_PWM_INDEX 2
    #define DRIVE_PWM_SETTINGS LEFT_MOTOR_PWM_INDEX, RIGHT_MOTOR_PWM_INDEX, THROTTLE_PWM_INDEX
  #else
    #define DRIVE_PWM_SETTINGS LEFT_MOTOR_PWM_INDEX, RIGHT_MOTOR_PWM_INDEX
  #endif
#endif

#ifdef NEED_DOME_PWM_PINS
  #ifdef NEED_DRIVE_PWM_PINS
    #ifdef THROTTLE_MOTOR_PWM
      #define DOME_PWM_INDEX 3
    #else
      #define DOME_PWM_INDEX 2
    #endif
  #else
    #define DOME_PWM_INDEX 0
  #endif
  #define DOME_PWM_SETTINGS DOME_PWM_INDEX
#endif

#endif
