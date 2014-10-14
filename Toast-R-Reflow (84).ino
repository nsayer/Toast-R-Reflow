/*

 Toast-R-Reflow (84)
 Copyright 2013 Nicholas W. Sayer
 
 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 
*/

/*

This version of the code is designed for the ATTiny84 variant of the controller
board, which is hardware versions > 1.0

*/

// Versions 1.2 and beyond include an external reference voltage. For versions 1.0 and 1.1
// comment out this line
#define EXTERNAL_REF

#include <avr/pgmspace.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

// The pins connected up to the LCD.
#define LCD_D4 7
#define LCD_D5 6
#define LCD_D6 5
#define LCD_D7 4
#define LCD_RS 3
#ifdef EXTERNAL_REF
#define LCD_E 2
#define LCD_RW 255 // This tells LiquidCrystal that there is no RW connection
#else
#define LCD_RW 2
#define LCD_E 1
#endif

// This is the temperature reading analog pin.
#ifdef EXTERNAL_REF
#define TEMP_SENSOR_PIN A1
#else
#define TEMP_SENSOR_PIN A0
#endif

// These pins turn on the two heating elements.
#define ELEMENT_ONE_PIN 9
#define ELEMENT_TWO_PIN 8

// This is the pin with the button.
#define BUTTON_SELECT 10

// This value is the full-scale temperature. It's the reference voltage
// divided by the 5 mV/degC scaling of the AD849x.
//
#ifdef EXTERNAL_REF
// 1.8 volts is 360 degrees C
#define FULL_SCALE_TEMP 360.0
#else
// 5 volts is 1000 degrees C
#define FULL_SCALE_TEMP 1000.0
#endif

// How often do we update the displayed temp?
#define DISPLAY_UPDATE_INTERVAL 500

// A reasonable approximation of room temperature.
#define AMBIENT_TEMP 25.0

// fiddle these knobs
#define K_P 500
#define K_I 0.1
#define K_D 5

// The number of milliseconds for each cycle of the control output.
// The duty cycle is adjusted by the PID.
#define PWM_PULSE_WIDTH 1000

// To get a temperature reading, read it a bunch of times and take the average.
// This will cut down on the noise.
#define SAMPLE_COUNT 3

// If we see any state change on the button, we ignore all changes for this long
#define BUTTON_DEBOUNCE_INTERVAL 50
// How long does the button have to stay down before we call it a LONG push?
#define BUTTON_LONG_START 250

// This is the enumeration of the output values for checkEvent()
#define EVENT_NONE 0
#define EVENT_SHORT_PUSH 1
#define EVENT_LONG_PUSH 2

// This is the magic value for the degree mark for the display. If your display
// happens to have some sort of whacky Kanji character instead, then you'll need
// to look up in your display's datasheet. You might try 0xD4 as a second choice.
#define DEGREE_CHAR (0xDF)

#define VERSION "(84) 0.3"

char p_buffer[17]; // enough for one line on the LCD.
#define _P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)

// missing from the Arduino IDE
#ifndef pgm_read_ptr
#define pgm_read_ptr(p) ((prog_void*)pgm_read_word(p))
#endif

struct curve_point {
  // Display this string on the display during this phase. Maximum 8 characters long.
  prog_char *phase_name;
  // The duration of this phase, in milliseconds
  unsigned long duration_millis;
  // The setpoint will drift smoothly across the phase from the last
  // set point to this temperature, arriving there at the very end.
  double target_temp;
};

// This table is the complete operational profile of the oven.
// This example is intended for tin-lead based paste. For RoHS
// solder, you'll need to adjust it.

// In order to put the operating profile into PROGMEM, we have to "unroll" the entire thing
// so that we can insure that each separate piece makes it into PROGMEM. First, all of the
// curve point name strings.

char PH_txt[] PROGMEM = "Preheat";
char SK_txt[] PROGMEM = "Soak";
char N_txt[] PROGMEM = "";
char RF_txt[] PROGMEM = "Reflow";
char CL_txt[] PROGMEM = "Cool";

// Next, each curve point, which represents a section of time where the oven will
// transition from the previous point to the next. It's defined as how much time we
// will spend, and what the target will be at the end of that time.

// Drift from the ambient temperature to 150 deg C over 90 seconds.
struct curve_point PT_1 PROGMEM = { PH_txt, 90000, 150.0 };
// Drift more slowly up to 180 deg C over 60 seconds.
struct curve_point PT_2 PROGMEM = { SK_txt, 60000, 180.0 };
// This entry will cause the setpoint to "snap" to the next temperature rather
// than drift over the course of an interval. The name won't be displayed because the duration is 0,
// but a NULL name will end the table, so use an empty string instead.
// This will force the oven to move to the reflow temperature as quickly as possible.
struct curve_point PT_3 PROGMEM = { N_txt, 0, 230.0 };
// It's going to take around 80 seconds to get to peak. Hang out there a bit.
struct curve_point PT_4 PROGMEM = { RF_txt, 90000, 230.0 };
// There is a maximum cooling rate to avoid thermal shock. The oven will likely cool slower than
// this on its own anyway. It might be a good idea to open the door a bit, but if you get over-agressive
// with cooling, then this entry will compensate for that.
struct curve_point PT_5 PROGMEM = { CL_txt, 90000, 100.0 };
// This entry ends the table. Don't leave it out!
struct curve_point PT_6 PROGMEM = { NULL, 0, 0.0 };

// Now the actual table itself.
prog_void* profile[] PROGMEM = { &PT_1, &PT_2, &PT_3, &PT_4, &PT_5, &PT_6 };

LiquidCrystal display(LCD_RS, LCD_RW, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

unsigned long start_time, pwm_time, lastDisplayUpdate, button_debounce_time, button_press_time;
unsigned int display_mode;

double setPoint, currentTemp, outputDuty;

PID pid(&currentTemp, &outputDuty, &setPoint, K_P, K_I, K_D, DIRECT);

// Look for button events. We support "short" pushes and "long" pushes.
// This method is responsible for debouncing and timing the pushes.
unsigned int checkEvent() {
  unsigned long now = millis();
  if (button_debounce_time != 0) {
    if (now - button_debounce_time < BUTTON_DEBOUNCE_INTERVAL) {
      // debounce is in progress
      return EVENT_NONE;
    } else {
      // debounce is over
      button_debounce_time = 0;
    }
  }
  boolean button = digitalRead(BUTTON_SELECT);
  if (button) {
    // Button is down
    if (button_press_time == 0) { // this is the start of a press.
      button_debounce_time = button_press_time = now;
    }
    return EVENT_NONE; // We don't know what this button-push is going to be yet
  } else {
    // Button released
    if (button_press_time == 0) return EVENT_NONE; // It wasn't down anyway.
    // We are now ending a button-push. First, start debuncing.
    button_debounce_time = now;
    unsigned long push_duration = now - button_press_time;
    button_press_time = 0;
    if (push_duration > BUTTON_LONG_START) {
      return EVENT_LONG_PUSH;
    } else {
      return EVENT_SHORT_PUSH;
    }
  }
}

// Format and display a temperature value.
void displayTemp(double temp) {
  if (temp < 10) display.print(' ');
  if (temp < 100) display.print(' ');
  display.print((int)temp);
  display.print('.');
  display.print(((int)(temp * 10.0)) % 10);
  display.print((char)DEGREE_CHAR);
  display.print('C');
}

// Sample the temperature pin a few times and take an average.
// Figure out the voltage, and then the temperature from that.
void updateTemp() {
  unsigned long sum = 0, mv;
  analogRead(TEMP_SENSOR_PIN); // throw this one away
  for(int i = 0; i < SAMPLE_COUNT; i++) {
    delay(10);
    mv = analogRead(TEMP_SENSOR_PIN);
    sum += mv;
  }
  mv = sum / SAMPLE_COUNT;
  currentTemp = ((double)mv) * (FULL_SCALE_TEMP / 1024.0);
}

// Call this when the cycle is finished. Also, call it at
// startup to initialize everything.
void finish() {
  start_time = 0;
  pwm_time = 0;
  pid.SetMode(MANUAL);
  digitalWrite(ELEMENT_ONE_PIN, LOW);
  digitalWrite(ELEMENT_TWO_PIN, LOW);
  display.clear();
  display.print(_P("Waiting"));
}

// Which phase are we in now? (or -1 for finished)
static int getCurrentPhase(unsigned long time) {
  unsigned long so_far = 0;
  for(int i = 0; true; i++) {
    struct curve_point this_point;
    memcpy_P(&this_point, pgm_read_ptr(profile + i), sizeof(struct curve_point));
    if (this_point.phase_name == NULL) break;
    if (so_far + this_point.duration_millis > time) { // we're in THIS portion of the profile
      return i;
    }
    so_far += this_point.duration_millis;
  }
  return -1;
}

// How many milliseconds into a cycle does the given phase number start?
static unsigned long phaseStartTime(int phase) {
  unsigned long so_far = 0;
  for(int i = 0; i < phase; i++) {
    struct curve_point this_point;
    memcpy_P(&this_point, pgm_read_ptr(profile + i), sizeof(struct curve_point));
    so_far += this_point.duration_millis;
  }
  return so_far;
}

void setup() {
  display.begin(16, 2);
#ifdef EXTERNAL_REF
  analogReference(EXTERNAL);
#else
  analogReference(DEFAULT); // Vcc
#endif
  pinMode(ELEMENT_ONE_PIN, OUTPUT);
  pinMode(ELEMENT_TWO_PIN, OUTPUT);
  digitalWrite(ELEMENT_ONE_PIN, LOW);
  digitalWrite(ELEMENT_TWO_PIN, LOW);
  
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  
  pid.SetOutputLimits(0.0, PWM_PULSE_WIDTH);
  pid.SetMode(MANUAL);
  
  lastDisplayUpdate = 0;
  start_time = 0;
  button_debounce_time = 0;
  button_press_time = 0;
  display_mode = 0;

  display.clear();
  
  display.setCursor(0,0);
  display.print(_P("Toast-R-Reflow"));
  display.setCursor(0, 1);
  display.print(_P(VERSION));
  
  delay(2000);
  finish();
}

void loop() {
  updateTemp();
  boolean doDisplayUpdate = false;
  {
    unsigned long now = millis();
    if (lastDisplayUpdate == 0 || now - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL) {
      doDisplayUpdate = true;
      lastDisplayUpdate = now;
      display.setCursor(0, 1);
      displayTemp(currentTemp);
    }
  }
  if (start_time == 0) {
    // We're not running. Wait for the button.
    unsigned int event = checkEvent();
    switch(event) {
      case EVENT_SHORT_PUSH:
      case EVENT_LONG_PUSH:
        // We really want to just re-initialize the PID.
        // The only way to do that with the existing API
        // is to transition from MANUAL to AUTOMATIC.
        // For this reason, setup() and finish() will
        // set the mode to MANUAL (which is otherwise
        // pointless because we don't call Compute() when
        // the oven isn't running).
        pid.SetMode(AUTOMATIC);
        start_time = millis();
        return;
    }
    
  } else {
    // We're running.
    unsigned long now = millis();
    unsigned long profile_time = now - start_time;
    int currentPhase = getCurrentPhase(profile_time);
    if (currentPhase < 0) {
      // All done!
      finish();
      return;
    }
    struct curve_point this_point;
    memcpy_P(&this_point, pgm_read_ptr(profile + currentPhase), sizeof(struct curve_point));
    
    unsigned int event = checkEvent();
    switch(event) {
      case EVENT_SHORT_PUSH:
      case EVENT_LONG_PUSH:
        display_mode ^= 1; // pick the other mode
        break;
    }

    if (doDisplayUpdate) {
      // more display updates to do.
        
      // The time
      unsigned long profile_time = now - start_time;
      unsigned int profile_sec = profile_time / 1000;
      unsigned int profile_min = profile_sec / 60;
      profile_sec %= 60;
      display.setCursor(10, 0);
      if (profile_min < 10) display.print('0');
      display.print(profile_min);
      display.print(':');
      if (profile_sec < 10) display.print('0');
      display.print(profile_sec);
    
      // The phase name    
      display.setCursor(0, 0);
      // This is just like the macro, but the string isn't a local constant
      strcpy_P(p_buffer, this_point.phase_name);
      display.print(p_buffer);
      for(unsigned int j = 0; j < 8 - strlen(p_buffer); j++) display.print(' ');
        
      display.setCursor(8, 1);
      switch(display_mode) {
        case 0:
          // the setpoint
          displayTemp(setPoint);
          break;
        case 1:
          // the oven power
          int mils = (outputDuty * 1000) / PWM_PULSE_WIDTH;
          if (mils < 1000) display.print(' ');
          if (mils < 100) display.print(' ');
          display.print(mils / 10);
          display.print('.');
          display.print(mils % 10);
          display.print('%');
          display.print(' ');
          display.print(' ');
          break;
      }
    }
    // The concept here is that we have two heating elements
    // that we can independently control.
    //
    // We could just turn them on and off at the same time, but
    // then why did we go to the trouble of buying two triacs?
    //
    // Instead, we can try and arrange them to pulse at different times.
    // This will help encourage convection (hopefully), as well as
    // reducing the instantaneous power demand (at least when the duty cycle
    // is less than 50%).
    //
    // So start one of them (#2) at the beginning of the interval, and end the other (#1)
    // at the end of the interval.
    if (pwm_time == 0 || now - pwm_time > PWM_PULSE_WIDTH) {
      // Time to start a new PWM interval.
      pwm_time = now;
      // Turn element one off. We may turn it on later.
      digitalWrite(ELEMENT_ONE_PIN, LOW);
      // Only start element two if we're actually going to do *anything*
      // We will turn it off later.
      digitalWrite(ELEMENT_TWO_PIN, (outputDuty > 0.0)?HIGH:LOW);
    } else {
      // We're somewhere in the middle of the current interval.
      unsigned long place_in_pulse = now - pwm_time;
      if (place_in_pulse >= outputDuty)
        digitalWrite(ELEMENT_TWO_PIN, LOW); // their pulse is over - turn the juice off
      if (place_in_pulse >= (PWM_PULSE_WIDTH - outputDuty))
        digitalWrite(ELEMENT_ONE_PIN, HIGH); // their pulse is ready to begin - turn the juice on
    }
    
    // Now update the set point.
    // What was the last target temp? That's where we're coming *from* in this phase
    double last_temp;
    {
      struct curve_point last_point;
      if (currentPhase != 0)
        memcpy_P(&last_point, pgm_read_ptr(profile + currentPhase - 1), sizeof(struct curve_point));
      last_temp = (currentPhase == 0)?AMBIENT_TEMP:last_point.target_temp;
    }
    // Where are we in this phase?
    unsigned long position_in_phase = profile_time - phaseStartTime(currentPhase);
    // What fraction of the current phase is that?
    double fraction_of_phase = ((double)position_in_phase) / ((double) this_point.duration_millis);
    // How much is the temperature going to change during this phase?
    double temp_delta = this_point.target_temp - last_temp;
    // The set point is the fraction of the delta that's the same as the fraction of the complete phase.
    setPoint = temp_delta * fraction_of_phase + last_temp;

    pid.Compute();   
  }
}
