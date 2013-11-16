Toast-R-Reflow
==============

Yet another attempt to convert a Toaster-oven into a SMD Reflow oven.

This code is going to be paired with an ATTiny 85 based controller board to run the process. An AD854x thermocouple
amplifier will allow a thermocouple to drive an analog input pin on the controller. Two of the pins will be used to
independently control the two heating elements of the oven. The last two available pins will be used with TinyWireM
as an i2c bus to drive an Adafruit 2 line display shield or backpack.

This project requires the following libraries:

TinyWireM   https://github.com/adafruit/TinyWireM
PID         https://github.com/br3ttb/Arduino-PID-Library
LiquidTWI2  https://github.com/lincomatic/LiquidTWI2

It also will require some sort of Arduino IDE support for ATTiny controllers. I used the MIT patch at
http://hlt.media.mit.edu/?p=1695

Version 0.1 has not actually been converted to ATTiny yet. I'm developing the code at present with an Uno. As soon
as the oven can actually reflow a board, then I'll make the final hardware, and then convert the sketch
for that.
