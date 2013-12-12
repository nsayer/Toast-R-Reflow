Toast-R-Reflow
==============

Yet another attempt to convert a Toaster-oven into a SMD Reflow oven.

This code is going to be paired with an ATTiny 85 based controller board to run the process. An AD854x thermocouple
amplifier will allow a thermocouple to drive an analog input pin on the controller. Two of the pins will be used to
independently control the two heating elements of the oven. The last two available pins will be used with TinyWireM
as an i2c bus to drive an Adafruit 2 line display shield or backpack.

This project requires the following libraries:

* TinyWireM: https://github.com/adafruit/TinyWireM
* PID: https://github.com/br3ttb/Arduino-PID-Library
* LiquidTWI2: https://github.com/lincomatic/LiquidTWI2

It also will require some sort of Arduino IDE support for ATTiny controllers. I used the MIT patch at
http://hlt.media.mit.edu/?p=1695

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
