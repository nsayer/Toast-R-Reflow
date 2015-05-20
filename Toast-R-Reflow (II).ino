/*

 Toast-R-Reflow II
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

This version of the code is designed for the ATMega328 / MAX31855 variant of the controller
board, which is model II.

*/

#include <avr/progmem.h>
#include <avr/wdt.h>
#include <LiquidCrystal.h>
#include <PID_v1.h>

// The pins connected to the MAX31855 thermocouple chip.
#define TEMP_CS 7
#define TEMP_DO 12
#define TEMP_CLK 13

// How many microseconds is the clock width of each half-cycle?
// It's actually 100 nanoseconds, but 1 microsecond will do.
#define MAX31855_BIT_DELAY 1

// The pins connected up to the LCD.
#define LCD_D4 10
#define LCD_D5 11
#define LCD_D6 12
#define LCD_D7 13
#define LCD_RS 9
#define LCD_E 8

// These pins turn on the two heating elements.
#define ELEMENT_ONE_PIN 2
#define ELEMENT_TWO_PIN 3

// This is the pin with the button.
#define BUTTON_SELECT 4
#define BUTTON_UP 5
#define BUTTON_DOWN 6

// How often do we update the displayed temp?
#define DISPLAY_UPDATE_INTERVAL 500

// fiddle these knobs
#define K_P 150
#define K_I 0.1
#define K_D 10

// The number of milliseconds for each cycle of the control output.
// The duty cycle is adjusted by the PID.
#define PWM_PULSE_WIDTH 1000

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

// baud rate for the serial port
#define SERIAL_BAUD 9600

// milliseconds between serial log intervals
#define SERIAL_LOG_INTERVAL 500

#define SIZE_OF_PROG_POINTER (sizeof(void*))

// Thanks to Gareth Evans at http://todbot.com/blog/2008/06/19/how-to-do-big-strings-in-arduino/
// Note that you must be careful not to use this macro more than once per "statement", lest you
// risk overwriting the buffer before it is used. So no using it inside methods that return
// strings that are then used in snprintf statements that themselves use this macro.
char p_buffer[17];
#define P(str) (strncpy_P(p_buffer, PSTR(str), sizeof(p_buffer)), p_buffer)

#define VERSION "(II) 1.2"

struct curve_point {
  // Display this string on the display during this phase. Maximum 8 characters long.
  char *phase_name;
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

char name_a_txt[] PROGMEM = "SnPb";
// Next, each curve point, which represents a section of time where the oven will
// transition from the previous point to the next. It's defined as how much time we
// will spend, and what the target will be at the end of that time.

// This special entry ends a profile. Always add it to the end!
struct curve_point PT_END PROGMEM = { NULL, 0, 0.0 };

// Drift from the ambient temperature to 150 deg C over 90 seconds.
struct curve_point PT_A_1 PROGMEM = { PH_txt, 90000, 150.0 };
// Drift more slowly up to 180 deg C over 60 seconds.
struct curve_point PT_A_2 PROGMEM = { SK_txt, 60000, 180.0 };
// This entry will cause the setpoint to "snap" to the next temperature rather
// than drift over the course of an interval. The name won't be displayed because the duration is 0,
// but a NULL name will end the table, so use an empty string instead.
// This will force the oven to move to the reflow temperature as quickly as possible.
struct curve_point PT_A_3 PROGMEM = { N_txt, 0, 230.0 };
// It's going to take around 80 seconds to get to peak. Hang out there a bit.
struct curve_point PT_A_4 PROGMEM = { RF_txt, 90000, 230.0 };
// There is a maximum cooling rate to avoid thermal shock. The oven will likely cool slower than
// this on its own anyway. It might be a good idea to open the door a bit, but if you get over-agressive
// with cooling, then this entry will compensate for that.
struct curve_point PT_A_5 PROGMEM = { CL_txt, 90000, 100.0 };

// Now the actual table itself.
void* profile_a[] PROGMEM = { &PT_A_1, &PT_A_2, &PT_A_3, &PT_A_4, &PT_A_5, &PT_END };

char name_b_txt[] PROGMEM = "RoHS";

// Drift from the ambient temperature to 150 deg C over 90 seconds.
struct curve_point PT_B_1 PROGMEM = { PH_txt, 90000, 150.0 };
// Drift more slowly up to 180 deg C over 60 seconds.
struct curve_point PT_B_2 PROGMEM = { SK_txt, 60000, 180.0 };
// This entry will cause the setpoint to "snap" to the next temperature rather
// than drift over the course of an interval. The name won't be displayed because the duration is 0,
// but a NULL name will end the table, so use an empty string instead.
// This will force the oven to move to the reflow temperature as quickly as possible.
struct curve_point PT_B_3 PROGMEM = { N_txt, 0, 250.0 };
// It's going to take around 80 seconds to get to peak. Hang out there a bit.
struct curve_point PT_B_4 PROGMEM = { RF_txt, 90000, 250.0 };
// There is a maximum cooling rate to avoid thermal shock. The oven will likely cool slower than
// this on its own anyway. It might be a good idea to open the door a bit, but if you get over-agressive
// with cooling, then this entry will compensate for that.
struct curve_point PT_B_5 PROGMEM = { CL_txt, 90000, 100.0 };

void* profile_b[] PROGMEM = { &PT_B_1, &PT_B_2, &PT_B_3, &PT_B_4, &PT_B_5, &PT_END };

char name_c_txt[] PROGMEM = "Bake";

// Drift from ambient to 125 deg C over an hour
struct curve_point PT_C_1 PROGMEM = { PH_txt, 3600000, 125.0 };
// Stay there for 10 more hours
struct curve_point PT_C_2 PROGMEM = { name_c_txt, 36000000, 125.0 };

void *profile_c[] PROGMEM = { &PT_C_1, &PT_C_2, &PT_END };

#define PROFILE_COUNT 3
void* profiles[] PROGMEM = { profile_a, profile_b, profile_c };
void* profile_names[] PROGMEM = { name_a_txt, name_b_txt, name_c_txt };

// missing from the Arduino IDE
#ifndef pgm_read_ptr
#define pgm_read_ptr(p) ((void*)pgm_read_word(p))
#endif

LiquidCrystal display(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

unsigned long start_time, pwm_time, lastDisplayUpdate, button_debounce_time, button_press_time, lastSerialLog;
unsigned char active_profile;
unsigned int display_mode;
boolean faulted; // This is whether or not we've *noticed* the fault.

double setPoint, currentTemp, outputDuty, referenceTemp;
boolean fault; // This is set by updateTemp()
unsigned char fault_bits; // This too.

PID pid(&currentTemp, &outputDuty, &setPoint, K_P, K_I, K_D, DIRECT);

// Look for button events. We support "short" pushes and "long" pushes.
// This method is responsible for debouncing and timing the pushes.
static unsigned int checkEvent() {
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
  boolean button = digitalRead(BUTTON_SELECT) == LOW;
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

static inline void formatTemp(double temp) {
  int deg = (int)(temp * 10);
  sprintf(p_buffer, "%3d.%1d%cC", deg / 10, deg % 10, DEGREE_CHAR);
}

// Format and display a temperature value.
static inline void displayTemp(double temp) {
  formatTemp(temp);
  display.print(p_buffer);
}

static inline void updateTemp() {
  // The DO and CLK pins are shared with the LCD, so we have to... rewire them.
  pinMode(TEMP_DO, INPUT);
  pinMode(TEMP_CLK, OUTPUT);
  digitalWrite(TEMP_CLK, LOW);
  delayMicroseconds(MAX31855_BIT_DELAY);
  digitalWrite(TEMP_CS, LOW);
  delayMicroseconds(MAX31855_BIT_DELAY);

  uint32_t temp_bits = 0;
  for(int i = 0; i < 32; i++) {
    digitalWrite(TEMP_CLK, HIGH);
    delayMicroseconds(MAX31855_BIT_DELAY);
    temp_bits <<= 1;
    temp_bits |= (digitalRead(TEMP_DO) == HIGH)?1:0;
    digitalWrite(TEMP_CLK, LOW);
    delayMicroseconds(MAX31855_BIT_DELAY);
  }
  digitalWrite(TEMP_CS, HIGH);
  pinMode(TEMP_DO, OUTPUT); // give the pin back to LiquidCrystal.
  
  // The format of the read data:
  // F E D C B A 9 8 7 6 5 4 3 2 1 0 F E D C B A 9 8 7 6 5 4 3 2 1 0
  // x x x x x x x x x x x x x x 0 f y y y y y y y y y y y y 0 a b c
  // x = thermocouple temperature
  // y = reference temperature
  // f = 1 if any of a, b or c are 1 (fault)
  // a = 1 if the thermocouple is shorted to Vcc
  // b = 1 if the thermocouple is shorted to ground
  // c = 1 if the thermocouple is open
  // Both temps are 2s compliment signed numbers.
  // x is in 1/4 degree units. y is in 1/16 degree units.

  fault = (temp_bits & 0x10000) != 0;
  fault_bits = temp_bits & 0x7;
  
  int16_t x = (int16_t)(temp_bits >> 16); // Take the top word and make it signed.
  x >>= 2; // This shift will be sign-extended because we copied it to an int type
  currentTemp = x / 4.0; // Now divide and float
  int16_t y = (int16_t)(temp_bits);
  y >>= 4;
  referenceTemp = y / 16.0;
  
}


// Call this when the cycle is finished. Also, call it at
// startup to initialize everything.
void finish(boolean silent = false) {
  start_time = 0;
  pwm_time = 0;
  pid.SetMode(MANUAL);
  digitalWrite(ELEMENT_ONE_PIN, LOW);
  digitalWrite(ELEMENT_TWO_PIN, LOW);
  if (silent) return;
  display.clear();
  display.print(P("Waiting"));
  display.setCursor(10, 0);
  strncpy_P(p_buffer, (char*)pgm_read_ptr(((uint16_t)profile_names) + SIZE_OF_PROG_POINTER * active_profile), sizeof(p_buffer));
  display.print(p_buffer);
}

static inline void* currentProfile() {
  return pgm_read_ptr(((uint16_t)profiles) + SIZE_OF_PROG_POINTER * active_profile);
}

// Which phase are we in now? (or -1 for finished)
static inline int getCurrentPhase(unsigned long time) {
  unsigned long so_far = 0;
  for(int i = 0; true; i++) {
    struct curve_point this_point;
    memcpy_P(&this_point, pgm_read_ptr(((uint16_t)currentProfile()) + i * SIZE_OF_PROG_POINTER), sizeof(struct curve_point));
    if (this_point.phase_name == NULL) break;
    if (so_far + this_point.duration_millis > time) { // we're in THIS portion of the profile
      return i;
    }
    so_far += this_point.duration_millis;
  }
  return -1;
}

// How many milliseconds into a cycle does the given phase number start?
static inline unsigned long phaseStartTime(int phase) {
  unsigned long so_far = 0;
  for(int i = 0; i < phase; i++) {
    struct curve_point this_point;
    memcpy_P(&this_point, pgm_read_ptr(((uint16_t)currentProfile()) + i * SIZE_OF_PROG_POINTER), sizeof(struct curve_point));
    so_far += this_point.duration_millis;
  }
  return so_far;
}

void setup() {
  // This must be done as early as possible to prevent the watchdog from biting during reset.
  MCUSR = 0;
  wdt_disable();
  
  Serial.begin(SERIAL_BAUD);
  display.begin(16, 2);
  pinMode(ELEMENT_ONE_PIN, OUTPUT);
  pinMode(ELEMENT_TWO_PIN, OUTPUT);
  pinMode(TEMP_CS, OUTPUT);
  digitalWrite(ELEMENT_ONE_PIN, LOW);
  digitalWrite(ELEMENT_TWO_PIN, LOW);
  digitalWrite(TEMP_CS, HIGH);
  
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
  
  pid.SetOutputLimits(0.0, PWM_PULSE_WIDTH);
  pid.SetMode(MANUAL);
  
  lastDisplayUpdate = 0;
  lastSerialLog = 0;
  start_time = 0;
  button_debounce_time = 0;
  button_press_time = 0;
  display_mode = 0;
  active_profile = 0;
  faulted = false;
  
  display.clear();
  
  display.setCursor(0,0);
  display.print(P("Toast-R-Reflow"));
  display.setCursor(0, 1);
  display.print(P(VERSION));
  
  delay(2000);
  wdt_enable(WDTO_500MS);
  finish();
}

void loop() {
  wdt_reset();
  updateTemp();
  if (fault) {
    if (!faulted) {
      faulted = true;
      finish(true);
      // complain bitterly
      display.clear();
      display.setCursor(2, 0);
      display.print(P("THERM. FAULT"));
      display.setCursor(0, 1);
      display.print(P("Fault bits: "));
      display.print(fault_bits);
    }
    return;
  } else {
    if (faulted) {
      // If the fault just went away, clean up the display
      finish();
    }
    faulted = false;
    // and carry on...
  }
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
      case EVENT_LONG_PUSH:
        if (++active_profile >= PROFILE_COUNT) active_profile = 0;
        display.setCursor(10, 0);
        strncpy_P(p_buffer, (char*)pgm_read_ptr(((uint16_t)profile_names) + SIZE_OF_PROG_POINTER * active_profile), sizeof(p_buffer));
        display.print(p_buffer);
        break;
      case EVENT_SHORT_PUSH:
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
    
    void* profile = currentProfile();
    struct curve_point this_point;
    memcpy_P(&this_point, pgm_read_ptr(((uint16_t)profile) + currentPhase * SIZE_OF_PROG_POINTER), sizeof(struct curve_point));
       
    unsigned int event = checkEvent();
    switch(event) {
      case EVENT_LONG_PUSH:
        finish();
        return;
      case EVENT_SHORT_PUSH:
        display_mode ^= 1; // pick the other mode
        break;
    }
    if (lastSerialLog == 0 || now - lastSerialLog > SERIAL_LOG_INTERVAL) {
      lastSerialLog = now;
      if (start_time == 0)
        Serial.print("Wait ");
      else {
        int sec = (int)((now - start_time) / 1000);
        sprintf(p_buffer, "%02d:%02d:%02d ", sec / 3600, (sec/60) % 60, sec % 60);
        Serial.print(p_buffer);
      }
      formatTemp(currentTemp);
      Serial.print(p_buffer);
      if (start_time == 0)
        Serial.print("\r\n");
      else {
        formatTemp(setPoint);
        Serial.print(p_buffer);
        Serial.print("\r\n");
      }
    }
    if (doDisplayUpdate) {
      // more display updates to do.
        
      // The time
      display.setCursor(8, 0);
      unsigned long profile_time = now - start_time;
      unsigned int sec = profile_time / 1000;
      sprintf(p_buffer, "%02d:%02d:%02d ", sec / 3600, (sec/60) % 60, sec % 60);
      display.print(p_buffer);
    
      // The phase name    
      display.setCursor(0, 0);
      strncpy_P(p_buffer, this_point.phase_name, sizeof(p_buffer));
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
          sprintf(p_buffer, "%3d.%1d%%  ", mils / 10, mils % 10);
          display.print(p_buffer);
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
      if (currentPhase != 0) {
        memcpy_P(&last_point, pgm_read_ptr(((uint16_t)profile) + (currentPhase - 1)*SIZE_OF_PROG_POINTER), sizeof(struct curve_point));
        last_temp = last_point.target_temp;
      } else {
        last_temp = referenceTemp; // Assume this is the ambient temp.
      }
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
