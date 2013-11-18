/*

 Toast-R-Reflow
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

#include <TinyWireM.h>
#include <LiquidTWI2.h>
#include <PID_v1.h>

#define LCD_I2C_ADDR 0x20 // for adafruit shield or backpack

// This is the AD595 reading analog pin.
#define TEMP_SENSOR_PIN 3

// These pins turn on the two heating elements.
#define ELEMENT_ONE_PIN 1
#define ELEMENT_TWO_PIN 4

// This value is degrees C per A/D unit.
// The spec sheet for the AD595 says the output is 10 mV per degree C.
// The A/D range is 5 volts over 1024 units. That's 4.8 mV per unit.
// mV per unit divided by mv per degC = degC per unit.
#define TEMP_SCALING_FACTOR 0.48828125
// If you're using an AD8495, then that chip has a spec of 5 mV per degree C.
// #define TEMP_SCALING_FACTOR 0.9765625

// How often do we update the displayed temp?
#define DISPLAY_UPDATE_INTERVAL 250

// A reasonable approximation of room temperature.
#define AMBIENT_TEMP 25.0

// fiddle these knobs
#define K_P 500
#define K_I 0
#define K_D 0

// The number of milliseconds for each cycle of the control output.
// The duty cycle is adjusted by the PID.
#define PWM_PULSE_WIDTH 1000

// Which button do we use?
#define BUTTON BUTTON_SELECT
// If we see any state change on the button, we ignore all changes for this long
#define BUTTON_DEBOUNCE_INTERVAL 50
// How long does the button have to stay down before we call it a LONG push?
#define BUTTON_LONG_START 250

#define EVENT_NONE 0
#define EVENT_SHORT_PUSH 1
#define EVENT_LONG_PUSH 2

#define VERSION "0.2"

struct curve_point {
  // Display this string on the display during this phase
  const char *phase_name;
  // The duration of this phase, in milliseconds
  unsigned long duration_millis;
  // The setpoint will drift smoothly across the phase from the last
  // set point to this temperature, arriving there at the very end.
  double target_temp;
};

// This table is the complete operational profile of the oven.
// This example is intended for tin-lead based paste. For RoHS
// solder, you'll need to adjust it.
struct curve_point profile[] = {
  // Drift from the ambient temperature to 150 deg C over 90 seconds.
  { "Preheat", 90000, 150.0 },
  // Stay at 150 deg C for 90 seconds.
  { "Soak", 90000, 150.0 },
  // This entry will cause the setpoint to "snap" to the next temperature rather
  // than drift over the course of an interval. The name won't be displayed because the duration is 0,
  // but a NULL name will end the table, so use an empty string instead.
  // This will force the oven to move to the reflow temperature as quickly as possible.
  { "", 0, 225.0},
  // get to reflow then stay there for 30 seconds.
  { "Reflow", 30000, 225.0 },
  // There is a maximum cooling rate to avoid thermal shock. The oven will likely cool slower than
  // this on its own anyway.
  { "Cool", 60000, AMBIENT_TEMP },
  // This entry ends the table. Don't leave it out!
  { NULL, 0, 0.0 }
};

LiquidTWI2 display(LCD_I2C_ADDR,0,1);

unsigned long start_time, pwm_time, lastDisplayUpdate, button_debounce_time, button_press_time;

//unsigned int error;

double setPoint, currentTemp, outputDuty;

PID pid(&currentTemp, &outputDuty, &setPoint, K_P, K_I, K_D, DIRECT);

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
  unsigned int buttons = display.readButtons();
  if ((buttons & BUTTON) != 0) {
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

void displayTemp(double temp) {
  if (temp < 10) display.print(" ");
  if (temp < 100) display.print(" ");
  display.print((int)temp);
  display.print(".");
  display.print(((int)(temp * 10.0)) % 10);
  display.print((char)0xDF);
  display.print("C");
}

void updateTemp() {
  unsigned int val = analogRead(TEMP_SENSOR_PIN);
  currentTemp = TEMP_SCALING_FACTOR * val;

}

/*
void err(const char *msg) {
  digitalWrite(ELEMENT_ONE_PIN, LOW);
  digitalWrite(ELEMENT_TWO_PIN, LOW);
  display.setBacklight(RED);
  display.clear();
  display.print(msg);
  error = 1;
}
*/

void finish() {
  start_time = 0;
  pwm_time = 0;
  pid.SetMode(MANUAL);
  digitalWrite(ELEMENT_ONE_PIN, LOW);
  digitalWrite(ELEMENT_TWO_PIN, LOW);
  display.setBacklight(YELLOW);
  display.clear();
  display.print("Waiting");
}

void computeSetPoint(unsigned long time) {
  unsigned long so_far = 0;
  double last_temp = AMBIENT_TEMP;
  for(int i = 0; profile[i].phase_name != NULL; i++) {
    if (so_far + profile[i].duration_millis > time) { // we're in THIS portion of the profile
      display.setCursor(0, 0);
      display.print(profile[i].phase_name);
      for(int j = 0; j < 8 - strlen(profile[i].phase_name); j++) display.print(" ");
      unsigned long position_in_phase = time - so_far;
      double fraction_of_phase = ((double)position_in_phase) / ((double) profile[i].duration_millis);
      double delta = profile[i].target_temp - last_temp;
      setPoint = delta * fraction_of_phase + last_temp;
      return;
    }
    so_far += profile[i].duration_millis;
    last_temp = profile[i].target_temp;
  }
  // We fell off the end. We must be done.
  finish();
}
  
void setup() {
  display.setMCPType(LTI_TYPE_MCP23017);
  display.begin(16, 2);
  
  pinMode(ELEMENT_ONE_PIN, OUTPUT);
  pinMode(ELEMENT_TWO_PIN, OUTPUT);
  digitalWrite(ELEMENT_ONE_PIN, LOW);
  digitalWrite(ELEMENT_TWO_PIN, LOW);
  
  pid.SetOutputLimits(0.0, PWM_PULSE_WIDTH);
  pid.SetMode(MANUAL);
  
  lastDisplayUpdate = 0;
  start_time = 0;
  //error = 0;
  button_debounce_time = 0;
  button_press_time = 0;

  display.clear();
  display.setBacklight(WHITE);
  
  display.setCursor(0,0);
  display.print("Toast-R-Reflow");
  display.setCursor(0, 1);
  display.print(VERSION);
  
  delay(2000);
  finish();
}

void loop() {
  updateTemp();
  {
    unsigned long now = millis();
    if (lastDisplayUpdate == 0 || now - lastDisplayUpdate > DISPLAY_UPDATE_INTERVAL) {
      lastDisplayUpdate = now;
      display.setCursor(0, 1);
      displayTemp(currentTemp);
    }
  }
  /*
  if (error) {
    // XXX What to do?
    return;
  }
  */
  if (start_time == 0) {
    // wait for the button
    unsigned int event = checkEvent();
    switch(event) {
      case EVENT_SHORT_PUSH:
      case EVENT_LONG_PUSH:
        display.setBacklight(GREEN);
        // We really want to just re-initialize the PID.
        // The only way to do that with the existing API
        // is to transition from MANUAL to AUTOMATIC.
        // For this reason, setup() and finish() will
        // set the mode to MANUAL (which is otherwise
        // pointless because we don't call Compute() when
        // the oven isn't running).
        pid.SetMode(AUTOMATIC);
        start_time = millis();
        break;
    }
    
  } else {
    unsigned long now = millis();
    unsigned long profile_time = now - start_time;
    unsigned int profile_sec = profile_time / 1000;
    unsigned int profile_min = profile_sec / 60;
    profile_sec %= 60;
    display.setCursor(10, 0);
    if (profile_min < 10) display.print("0");
    display.print(profile_min);
    display.print(":");
    if (profile_sec < 10) display.print("0");
    display.print(profile_sec);
    
    display.setCursor(8, 1);
    displayTemp(setPoint);
    
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
      pwm_time = now;
      digitalWrite(ELEMENT_ONE_PIN, LOW);
      // Only start element two if we're actually going to do *anything*
      digitalWrite(ELEMENT_TWO_PIN, (outputDuty > 0.0)?HIGH:LOW);
    } else {
      unsigned long place_in_pulse = now - pwm_time;
      if (place_in_pulse > outputDuty)
        digitalWrite(ELEMENT_TWO_PIN, LOW); // their pulse is over - turn the juice off
      if (place_in_pulse > (PWM_PULSE_WIDTH - outputDuty))
        digitalWrite(ELEMENT_ONE_PIN, HIGH); // their pulse is ready to begin - turn the juice on
    }
    computeSetPoint(profile_time);
    pid.Compute();   
  }
}
