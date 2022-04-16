#ifndef __PENUMBRA_FIXED_SETTINGS_H__
#define __PENUMBRA_FIXED_SETTINGS_H__

// Drive System options

#define DRIVE_SYSTEM_PWM                1
#define DRIVE_SYSTEM_SABER              2
#define DRIVE_SYSTEM_ROBOTEQ_PWM        3
#define DRIVE_SYSTEM_ROBOTEQ_SERIAL     4
#define DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL 5

// Dome System options

#define DOME_DRIVE_NONE      0
#define DOME_DRIVE_PWM       1
#define DOME_DRIVE_SABER     2

// Default drive system configuration. Can be changed through web interface and stored in flash

#define MAXIMUM_SPEED        0.5f   // Top speed limiter - percentage 0.0 - 1.0. default 50%
#define MAXIMUM_GUEST_SPEED  0.3f   // Top speed for a guest joystick (if used) - percentage 0.0 - 1.0. default 30%
#define ACCELERATION_SCALE   100    // Scale value of 1 means instant. Scale value of 100 means that the throttle will increase 1/100 every 25ms
#define DECELRATION_SCALE    20     // Scale value of 1 means instant. Scale value of 20 means that the throttle will decrease 1/20 every 25ms
#define SCALING              true   // set to true if acceleration/decelleration should be applied
#define THROTTLE_INVERTED    false  // set to true if throttle should be inverted
#define TURN_INVERTED        false  // set to true if turn should be inverted

// Penumbra preferences

#define PREFERENCE_DRIVE_SPEED              "maxspeed"
#define PREFERENCE_DRIVE_THROTTLE_ACC_SCALE "throttleaccscale"
#define PREFERENCE_DRIVE_THROTTLE_DEC_SCALE "throttledecscale"
#define PREFERENCE_DRIVE_TURN_ACC_SCALE     "turnaccscale"
#define PREFERENCE_DRIVE_TURN_DEC_SCALE     "turndecscale"
#define PREFERENCE_DRIVE_GUEST_SPEED        "guestmaxspeed"
#define PREFERENCE_DRIVE_SCALING            "scaling"
#define PREFERENCE_DRIVE_MIXING             "mixing"
#define PREFERENCE_DRIVE_THROTTLE_INVERT    "throttleinvert"
#define PREFERENCE_DRIVE_TURN_INVERT        "turninvert"
#define PREFERENCE_WIFI_ENABLED             "wifi"
#define PREFERENCE_WIFI_SSID                "ssid"
#define PREFERENCE_WIFI_PASS                "pass"
#define PREFERENCE_WIFI_AP                  "ap"

#endif
