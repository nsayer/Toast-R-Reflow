Toast-R-Reflow
==============

Yet another attempt to convert a Toaster-oven into a SMD Reflow oven.

This code is going to be paired with an ATTiny 85 based controller board to run the process. An AD854x thermocouple
amplifier will allow a thermocouple to drive an analog input pin on the controller. Two of the pins will be used to
independently control the two heating elements of the oven. The last two available pins will be used with TinyWireM
as an i2c bus to drive an Adafruit 2 line display shield or backpack.

This project contains two versions of the code:

Toast-R-Reflow.ino is the orignal version, intended for the ATTiny85 + i2c display. This is intended for the original standalone controller as well as backpack controllers with versions less than 1.0.

Toast-R-Reflow (84).ino is the version for ATTiny84 backpacks, which is backpacks starting with version 1.0.

ORIGINAL CODE
=============

This project requires the following libraries:

* TinyWireM: https://github.com/adafruit/TinyWireM
* PID: https://github.com/br3ttb/Arduino-PID-Library
* LiquidTWI2: https://github.com/lincomatic/LiquidTWI2

It also will require some sort of Arduino IDE support for ATTiny controllers. I used the MIT patch at
http://hlt.media.mit.edu/?p=1695 originally, but with newer Arduino packages, you should be able to
use the board manager and find support for raw ATTiny chips.

Note that as of version 0.5, this code tickles a bug in Arduino that you need to work around.

You need to find the file hardware/arduino/cores/arduino/wiring_analog.c and apply this patch:

```
--- wiring_analog.c.orig	2013-12-12 12:44:11.000000000 -0800
+++ wiring_analog.c	2013-12-12 12:16:16.000000000 -0800
@@ -66,7 +66,7 @@
 	// channel (low 4 bits).  this also sets ADLAR (left-adjust result)
 	// to 0 (the default).
 #if defined(ADMUX)
-	ADMUX = (analog_reference << 6) | (pin & 0x07);
+	ADMUX = (analog_reference << 6) | (pin & 0x07) | ((analog_reference & 0x4)?0x10:0);
 #endif
 
 	// without a delay, we seem to read from the wrong channel
```

That will allow analogReference() to set the correct ADMUX bits when the value is greater than 4, which is
what turns on the 2.56 volt reference.


ATTiny84 VERSION
================

This version requires:

* PID: https://github.com/br3ttb/Arduino-PID-Library
* LiquidCrystal (built-in)

This version also requires ATTiny support, but does not require the patch to wiring_analog.c.

You should configure the IDE for a raw ATTiny84 running at 8 MHz from its internal oscillator. The fuse settings
for that are: low = 0xE2, high = 0xDF, extended = 0xFF

Model II VERSION
================

This version has the same library requirements as the ATTiny84 version, but does not require ATTiny support. Instead,
it requires a patch to boards.txt to add support for "raw" ATMega328P chips without the Arduino bootloader and
programming using an AVR ISP programmer.

Here's a suitable addition:

```
##############################################################

usbtiny328.name=[usbtinyisp]ATmega328

usbtiny328.upload.using=usbtinyisp
usbtiny328.upload.maximum_size=32768

usbtiny328.build.mcu=atmega328p
usbtiny328.build.f_cpu=16000000L
usbtiny328.build.core=arduino
usbtiny328.build.variant=standard
```

As with the model I controller, with newer Arduino IDEs, the board manager has options for "raw" ATMega chips.

Alternatively, you can load an Arduino bootloader in. If you do this, you can pretend that the board is an Arduino UNO
for the purposes of bootloading and compiling the firmware. To load the firmware this way, use an FTDI cable and connect
to the 6 pin serial header on the board and upload the firmware in the usual Arduino manner.
