#ifndef __PENUMBRA_USER_SETTINGS_H__
#define __PENUMBRA_USER_SETTINGS_H__

#include "pin-map.h"
#include "src/Fixed_Settings.h"    // This MUST be the first line in this file.

// =============================================================

// ------------------------------
// Debug modes

// Uncomment a line to enable the debugging mode.

#define USE_DEBUG
//#define USE_MOTOR_DEBUG
//#define USE_VERBOSE_DOME_DEBUG
//#define USE_SERVO_DEBUG
//#define USE_VERBOSE_SERVO_DEBUG

// ------------------------------
// USB Host support
//
// Uncomment the following line if you are using Penumbra board and want to use the USB
// port for pairing PS3 controllers.
//#define USE_USB

// ------------------------------
// Bluetooth

// Bluetooth address of this ESP32 device. If you already have a Shadow system configured
// the easiest thing is reuse the address of your USB Bluetooth dongle here. Alternatively,
// you can use sixaxispair to pair your controllers with the ESP32. Uncomment if you wish
// to use the ESP32 bluetooth address

#ifndef USE_USB
#define MY_BT_ADDR             "24:6f:28:44:a5:ae"
#endif
//#define MY_BT_ADDR           "03:03:03:03:03:03"
//#define MY_BT_ADDR           "84:C5:A6:61:AC:37"
//#define MY_BT_ADDR           "b6:0c:76:94:05:b0" (PS4)

// Assign a Bluetooth address here if you want a specific controller as the
// drive stick or dome stick otherwise it will be first come first serve.

#define DRIVE_STICK_BT_ADDR    nullptr
//#define DRIVE_STICK_BT_ADDR  "00:07:04:09:b4:05"

#define DOME_STICK_BT_ADDR     nullptr
//#define DOME_STICK_BT_ADDR   "e0:ae:5e:9b:e1:04"

// ------------------------------
// Controller type

// Uncomment one of the following that is the type of PS controller you are using (default PS3)

#define DRIVE_CONTROLLER_TYPE   kPS3Nav
//#define DRIVE_CONTROLLER_TYPE   kPS3
//#define DRIVE_CONTROLLER_TYPE   kPS4

// ------------------------------
// Drive system

// Select an option from the following.
// Choose from:
//  DRIVE_SYSTEM_PWM
//  DRIVE_SYSTEM_SABER
//  DRIVE_SYSTEM_ROBOTEQ_PWM
//  DRIVE_SYSTEM_ROBOTEQ_SERIAL
//  DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL

#define DRIVE_SYSTEM    DRIVE_SYSTEM_ROBOTEQ_PWM

// ------------------------------
// Dome system

// Select an option from the following.
// Choose from:
//  DOME_DRIVE_NONE
//  DOME_DRIVE_PWM
//  DOME_DRIVE_SABER

#define DOME_DRIVE    DOME_DRIVE_SABER

// Uncomment to enable dome controller gestures.
// Press joystick button L3. Will collect a series of button presses and joystick movements.
// Press joystick button L3 to recognize gesture.

#define DOME_CONTROLLER_GESTURES

// Uncomment to disable throotle boost mode where l2 will increase speed above MAX_SPEED
#define ENABLE_TANK_DRIVE_THROOTLE_BOOST_MODE
#define ENABLE_DOME_DRIVE_THROOTLE_BOOST_MODE

// Drive using left stick or right stick
#define TANK_DRIVE_LEFT_STICK
//#define TANK_DRIVE_RIGHT_STICK

// Enable so L1 (left stick) or R1 (right stick) will be used as a hard break 
#define TANK_DRIVE_USE_HARD_STOP

// Spin dome using right stick or left stick
#if defined(TANK_DRIVE_LEFT_STICK)
#define DOME_DRIVE_RIGHT_STICK
#else
#define DOME_DRIVE_LEFT_STICK
#endif

// Enable so L1 (left stick) or R1 (right stick) will be used as a hard break 
//#define DOME_DRIVE_USE_HARD_STOP

#if (defined(TANK_DRIVE_LEFT_STICK) && defined(DOME_DRIVE_LEFT_STICK)) || \
    (defined(TANK_DRIVE_RIGHT_STICK) && defined(DOME_DRIVE_RIGHT_STICK))
 #error Use opposite sticks for dome and feet motors
#endif

// Default drive system configuration. Can be changed through web interface and stored in flash

#define MAXIMUM_SPEED        0.5f   // Top speed limiter - percentage 0.0 - 1.0. default 50%
#define MAXIMUM_GUEST_SPEED  0.3f   // Top speed for a guest joystick (if used) - percentage 0.0 - 1.0. default 30%
#define ACCELERATION_SCALE   100    // Scale value of 1 means instant. Scale value of 100 means that the throttle will increase 1/100 every 25ms
#define DECELERATION_SCALE   20     // Scale value of 1 means instant. Scale value of 20 means that the throttle will decrease 1/20 every 25ms
#define SCALING              true   // set to true if acceleration/decelleration should be applied
#define THROTTLE_INVERTED    false  // set to true if throttle should be inverted
#define TURN_INVERTED        false  // set to true if turn should be inverted

#define DOME_INVERTED        false  // set to true if dome drive should be inverted

// ------------------------------
// SYK radio controller

// Uncomment to enable the optional SYK base radio controller

//#define USE_RADIO

// ------------------------------
// Enable WiFi

// Comment this line out if you do not want to use the WiFi to
// change preferences via a web application on your phone.

#define USE_WIFI

// Configure this section only if the WiFi is enabled.

#ifdef USE_WIFI

// Uncomment to enable mDNS
#define USE_MDNS

// Uncomment to enable OTA update
//#define USE_OTA

// Uncomment to enable Web interface
#define USE_WIFI_WEB

#define WIFI_ENABLED            true
// Set these to your desired credentials.
#define WIFI_AP_NAME            "R2D2"
#define WIFI_AP_PASSPHRASE      "Astromech"
#define WIFI_ACCESS_POINT       true  /* true if access point: false if joining existing wifi */

// Alternatively join an existing AP
//#define WIFI_AP_NAME          "MyWifi"
//#define WIFI_AP_PASSPHRASE    "MyPassword"
//#define WIFI_ACCESS_POINT     false  /* true if access point: false if joining existing wifi */
#endif

// =============================================================

#include "src/Variable_Settings.h"    // This MUST be the last line in this file.

#endif
