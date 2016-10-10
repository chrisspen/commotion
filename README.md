ComMotion - firmware and drivers
================================

The ComMotion motor controller is an Arduino shield for controlling up to 4 brushed DC motors
via I2C.

The controller is essentially two Arduinos working in tandem, with each controlling and monitoring
two motors.

It supports monitoring current, voltage and encoders. However, many of these features were not
fully implemented by the manufacturer and exposed via the I2C interface. 

This code has two parts, the firmware, which is flashed onto the ComMotion, and the examples, which
are flashed onto the host Arduino and demonstrate how to control the ComMotion shield.

Installation
------------

The ComMotion mimics an Arduino, so its firmware can be flashed by using the standard Arduino IDE.

Normally, to flash an Arduino, you'd plug it in to a host computer via USB or ICSP.

Since the ComMotion is in the form of a shield, with no direct USB or ICSP interfaces, you'll use
the host Arduino as the programmer.

To do this, you'll first need to flash the host Arduino with sketch to turn it into an ICSP.
Follow the instructions in "ComMotion Programming Tutorial.pdf".

In general, the procedure is:

1. Attach the ComMotion to an Arduino Uno, and flash the ArduinoISP sketch onto the Arduino Uno.
2. Short the RST jumper on the ComMotion.
3. Set the ComMotion's MCU selection switch to the left.
4. Flash the ComMotion sketch to the ComMotion using File->Upload Using Programmer (not not use the normal Upload button!)
5. Set the ComMotion's MCU selection switch to the left.
6. Flash the ComMotion sketch to the ComMotion using File->Upload Using Programmer (not not use the normal Upload button!)
7. Un-short the RST jumper on the ComMotion and set the MCU selection switch to center.
8. Flash your custom sketch to the Arduino Uno.

Usage
-----

See the examples for how to control the ComMotion from an Arduino.

Related Projects
----------------

https://github.com/Blueglide/ComMotion
