#ifndef __PENUMBRA_USER_SETTINGS_H__
#define __PENUMBRA_USER_SETTINGS_H__

#include "src/Fixed_Settings.h"    // This MUST be the first line in this file.

// =============================================================

// ------------------------------
// Debug modes

// Uncomment a line to enable the debugging mode.

#define USE_DEBUG
//#define USE_MOTOR_DEBUG
//#define USE_SERVO_DEBUG
//#define USE_VERBOSE_SERVO_DEBUG

// ------------------------------
// Bluetooth

// Bluetooth address of this ESP32 device. If you already have a Shadow system configured
// the easiest thing is reuse the address of your USB Bluetooth dongle here. Alternatively,
// you can use sixaxispair to pair your controllers with the ESP32.

#define MY_BT_ADDR        "ac:67:b2:6c:70:8c"  // The address to which my controller is currently paired
//#define MY_BT_ADDR      "7c:9e:bd:d8:eB:00"  // The address of this ESP32 device
//#define MY_BT_ADDR      "00:15:83:F3:61:A6"  // My SHADOW controller's USB Dongle MAC address

// Assign a Bluetooth address here if you want a specific controller as the
// drive stick or dome stick otherwise it will be first come first serve.

#define DRIVE_STICK_BT_ADDR    nullptr
//#define DRIVE_STICK_BT_ADDR  "00:07:04:09:b4:05"

#define DOME_STICK_BT_ADDR     nullptr
//#define DOME_STICK_BT_ADDR   "e0:ae:5e:9b:e1:04"

// ------------------------------
// Controller type

// Uncomment one of the following that is the type of PS controller you are using (default PS3)

#define DRIVE_CONTROLLER_TYPE     kPS4
//#define DRIVE_CONTROLLER_TYPE   kPS3
//#define DRIVE_CONTROLLER_TYPE   kPS3Nav

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

//#define DOME_CONTROLLER_GESTURES

// ------------------------------
// SYK radio controller

// Uncomment to enable the optional SYK base radio controller

//#define USE_RADIO

// ------------------------------
// Assign pins

#define SERIAL1_RX_PIN 4    // Set to your board's TX pin for Serial1.
#define SERIAL1_TX_PIN 2    // Set to your board's RX pin for Serial1.

#define SERIAL2_RX_PIN 18   // Set to your board's TX pin for Serial2. 
#define SERIAL2_TX_PIN 19   // Set to your board's RX pin for Serial2.

// Uncomment the following to enable use of Software Serial for Marcduino

//#include "SoftwareSerial.h"
//#define SERIAL_MARCDUINO_TX_PIN 5 // Set to the pin you use for Software Serial

// Set the following only if you use PWM for your drive system.
// You may ignore this if you do not use PWM for your drive system.

#define LEFT_MOTOR_PWM      23  // Set to a PWM pin for left foot motor.
#define RIGHT_MOTOR_PWM     22  // Set to a PWM pin for right foot motor.
#define THROTTLE_MOTOR_PWM  21  // Optional Roboteq pin used for MicroBasic scripts running on the Roboteq controller to change the throttle.
                                // If the Microbasic script is not runnig this PWM signal will have no effect.

// Set the following only if you use PWM for your dome system.
// You may ignore this if you do not use PWM for your dome system.

#define DOME_MOTOR_PWM      27  // Set to a PWM pin used to control the dome motor

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
