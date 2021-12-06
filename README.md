# Minimal drive system for the ESP32 #

This sketch provides a basic drive system running on the ESP32. It depends on the [Reeltwo](https://reeltwo.github.io/Reeltwo) library. It has been updated to control a dome motor and support a second PS3Nav controller or a single PS3/PS4 controller. The dome motor needs to be either PWM based or Sabertooth Serial.

Default dome and motor drive system is Sabertooth. Change this macro to change the foot motor drive:

```C++
#define DRIVE_SYSTEM         DRIVE_SYSTEM_ROBOTEQ_PWM_SERIAL
```

Change this macro to change the dome motor system:

```C++
#define DOME_DRIVE           DOME_DRIVE_SABER
```

To disable the dome motor support change it to:

```C++
#define DOME_DRIVE           DOME_DRIVE_NONE
```


If using two PS3Nav controllers it also supports gestures. Press L3 to start gesture recognition. Execute a series of button presses and/or joystick movements and follow with a second L3 press to end gesture recognition. You should see the sketch print:

```C++
GESTURE: <gesture code>
```

Gesture codes for buttons are as follows:

	X: Cross
	O: Circle
	U: Up
	R: Down
	L: Left
	P: PS

Gesture codes for the analog stick are as follows:

	1 2 3
	4 5 6
	7 8 9

For example:
```C++
GESTURE: UDU	(KeyPad: Up-Down-Up)
GESTURE: 252	(Analog stick Up-Center-Up)
```

You can disable gesture support by commenting or deleting the following:

```C++
#define DOME_CONTROLLER_GESTURES
```


# Default Pinout #

Serial pins used to communicate (if using serial)

```C++
#define SERIAL1_RX_PIN 	    4
#define SERIAL1_TX_PIN 	    2

// Pins if using PWM
#define LEFT_MOTOR_PWM      23
#define RIGHT_MOTOR_PWM     22
#define DOME_MOTOR_PWM      27
```

Optional for Roboteq:
If running a Microbasic script to adjust the max RPM you can define:

```C++
#define THROTTLE_MOTOR_PWM  21
```
