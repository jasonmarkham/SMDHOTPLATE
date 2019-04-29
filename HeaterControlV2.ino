//*********************
// * HW CONFIGURATION *
// Version jk2019
// added Buzzer
// added temp sensor 2
// added Voltage In - 
//*********************
//#define DEBUG	// output the results to Serial

// Enable only one of the temperature sensors
//#define MAX31855
//#define MAX6675
#define NTC // using Temperature table

#define OLED	// Use OLED display
//#define VIRTUALTEMPERATURE	//For debug purposes, when temp sensor is disconnected
//#define SOFTRESET	// If your Arduino has broken bootloader (watchdog is not working) then enable this dirty trick or better - flash optiboot to your arduino.
//#define TIMER_MINUTES // Count in minutes/hours. If not defined then seconds/minutes
//*********************

#include "encoder.h"
#include "adc.h"
#include <PID_v1.h>
#include <EEPROM.h>
#include "eeprom.h"
#include <avr/wdt.h>

#ifdef OLED
#include <U8g2lib.h>
#include "oled.h"
#endif


// pin definitions
#define FAN_PIN            2 // FAN for cooling stage
#define encoderPinA        3 // Interrupt pin (coupled with capacitor to GND)
#define encoderPinB        4 // Interrupt pin (coupled with capacitor to GND)
#define BUZZER_PIN         5 // buzzer connected to pin 
#define MOSFET_PIN 		     6 // Heater pin 
#define BUTTON_PIN        12 // encoder switch Pin for knob button
#define LED_PIN 		      13 // Led pin  
#define TEMPSENSOR_PIN_0 	A0 // Temperature sensor connected to pin A0

#define TEMPSENSOR_PIN_1  A1 // Temperature sensor connected to pin A1
#define VOLTAGE_PIN       A2 // Potential divider with Input 100k / 10k - Measure input Voltage


// RAMP can always be 3 deg per second

#ifdef TIMER_MINUTES
#define TIMER_DIVIDER 1000 * 60
#else
#define TIMER_DIVIDER 1000
#endif

// ADC=1023 - temp sensor is not connected

// some macros:
// mosfet (HIGH - OFF, LOW - ON)
#define H_ON 	digitalWrite(MOSFET_PIN,HIGH);digitalWrite(LED_PIN, HIGH);
#define H_OFF 	digitalWrite(MOSFET_PIN,LOW);digitalWrite(LED_PIN, LOW);

uint8_t ControlType;		// 0 - Manual or 1 - Auto
//uint8_t ProcessStage;		// 0 - just hold manual temperature; 1 - Ramp to Preheat; 2 - Preheat; 3 - Ramp to Reflow; 4 - Reflow; 5 - Cool down; 255 - finished

// PID global variables
#define PID_WINDOWSIZE 500	// upper limit of PID output
#define PID_ABSTEMPDIFFERENCE 10	// Difference in set and current temperature when PID should not work.
#define PID_P_FACTOR 1.0	//PID values are stored in EEPROM in int format. So, scale them (div) before use.
#define PID_I_FACTOR 10.0	//PID values are stored in EEPROM in int format. So, scale them (div) before use.
#define PID_D_FACTOR 1.0	//PID values are stored in EEPROM in int format. So, scale them (div) before use.

double currentTemp = 0;
double setPoint = 0;
double outputVal;

int pid_P, pid_I, pid_D; // PID values
int auto_preheatTemp, auto_preheatTime, auto_reflowTemp, auto_reflowTime, manual_temp;

unsigned long soft_pwm_millis;

//input/output variables passed by reference, so they are updated automatically
PID myPID(&currentTemp, &outputVal, &setPoint, 0, 0, 0, DIRECT); // PID values will be set later

unsigned long timer_millis;
int timer_counter;
boolean timer_active = false;
uint8_t value_editable = 0;	// Edit Preset temperature or Timer on the fly (0 - nothing editable, 1 - Preset Temp, 2 - Timer)
unsigned long value_editable_millis;


#ifdef DEBUG
unsigned long serial_ms = 0;
#endif

#ifdef VIRTUALTEMPERATURE
unsigned long virtualTemperature_ms = 0;
boolean virtualTemperature_Direction = true;
#endif

void setup() {

  wdt_disable();
  pinMode(MOSFET_PIN, OUTPUT);
  H_OFF // TURN OFF HEATER

  pinMode(LED_PIN, OUTPUT);

  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  // Sound the buzzer with a short beep
  BuzzerShortbeep();


#ifdef DEBUG
  Serial.begin(115200);
#endif

#ifdef OLED
  u8g2.begin();
  u8g2.setDrawColor(2);	// Xor is default mode across all sketch.
  u8g2.setFontMode(1);	// Or is default mode
#endif

  Temperature_init();

#ifdef DEBUG
  Serial.print(F("Temperature Sensors A0: "));
  Serial.println(getTemperature_0());
  Serial.print(F("Temperature Sensors A1: "));
  Serial.println(getTemperature_1());
  Serial.print(F("Input Voltage A2: "));
  Serial.println(getVoltage());

#endif

  // read PID values from EEPROM
  restore_settingsEEPROM();
  // we use FACTOR for PID values to get rid of comas in interface.
  myPID.SetTunings((float)pid_P / PID_P_FACTOR, (float)pid_I / PID_I_FACTOR, (float)pid_D / PID_D_FACTOR);
  myPID.SetOutputLimits(0, PID_WINDOWSIZE);	//set PID output range

  //show the input voltage 
  u8g2.setFont(u8g2_font_timR18_tn);  //numeric font for current temperature
  u8g2.setCursor(0, 31);
  uint16_t curval=round(getVoltage());
  u8g2.print(curval);
  u8g2.sendBuffer();

  delay(3000);

  initEncoder();

  waitUntilButtonReleased(); // in case it was

  // main screen (choose Manual/Auto)
#ifdef OLED
  drawMenu_AutoManual(ControlType);
#endif
#ifdef DEBUG
  Serial.print(F("ControlType is: ")); Serial.println(ControlType);
#endif
  char encVal = 0;  // signed value - nothing is pressed
  while (encVal != 127) {
#ifdef OLED
    drawMenu_AutoManual(ControlType);
#endif
    encVal = rotaryEncRead();
    if (encVal != 127 && encVal != 0) {
      if (encVal > 0) {
        ControlType = 0;
      } else {
        ControlType = 1;
      }
#ifdef DEBUG
      Serial.print(F("ControlType is: ")); Serial.println(ControlType);
#endif
    }
    currentTemp = getTemperature_0();
  }
  EEPROM.update(EEPROM_CONTROLTYPE, ControlType);	// store selected Control Type for the next time
#ifdef OLED
  u8g2.clearDisplay();
#endif
  // wait until button released
  encoderLongPressmillis = millis();
  while (rotaryEncRead() == 127) {
    if (is_rotaryEncLongPress()) {
      // go to config menu
      configureParams();
      encoderLongPressmillis = millis();
    }
  }

  if (ControlType == 0) {
    setPoint = manual_temp;	// last saved temperature.
#ifdef DEBUG
    Serial.println(F("Start Manual mode"));
#endif
  } else {
    setPoint = auto_preheatTemp;
#ifdef DEBUG
    Serial.println(F("Start Automatic mode"));
#endif
  }

  timer_counter = 0;
  timer_active = false;

  myPID.SetMode(AUTOMATIC); // turn on PID


#ifdef VIRTUALTEMPERATURE
  currentTemp = setPoint - PID_ABSTEMPDIFFERENCE * 2;
#endif

  soft_pwm_millis = millis() - 50000;	// Before turning on the heater wait for some seconds

}

void loop() {

  // Exit by holding button for 2 seconds...
  WDT_Init();	// keep system alive

#ifdef VIRTUALTEMPERATURE
  if (virtualTemperature_ms + TIMER_DIVIDER < millis()) {
    virtualTemperature_ms = millis();
    currentTemp += (virtualTemperature_Direction ? 0.4 : -0.4);
    if (virtualTemperature_Direction && currentTemp > setPoint + PID_ABSTEMPDIFFERENCE) {
      virtualTemperature_Direction = false;
    }
    if (!virtualTemperature_Direction && currentTemp < setPoint - PID_ABSTEMPDIFFERENCE - 1) {
      virtualTemperature_Direction = true;
    }
  }
#else
  currentTemp = (getTemperature_0() + getTemperature_1() / 2);
#endif


  // use PID only when difference is small (to prevent windup of I)
  if (abs(setPoint - currentTemp) < PID_ABSTEMPDIFFERENCE) {
    myPID.Compute();
  } else {
    outputVal = (setPoint > currentTemp ? PID_WINDOWSIZE : 0); // ON/OFF control
  }
  doSoftwarePWM((uint16_t)outputVal);	// make slow PWM to prevent unnecessary mosfet heating

  if (ControlType == 0) {
    adjustValues(1);	//adjust values on the fly (only preset temp to adjust)
    doManualReflow();
  } else {
    adjustValues(2);	//adjust values on the fly (adjust both, preset temp and timer)
    doAutoReflow();
  }

  // debug
#ifdef DEBUG
  if (serial_ms + 1000 < millis()) {
    serial_ms = millis();
    Serial.print(F("Set: ")); Serial.print(setPoint);
    Serial.print(F(", Actual: ")); Serial.print(currentTemp);
    Serial.print(F(", PWM: ")); Serial.print(outputVal);
    Serial.print(F(", Timer: ")); Serial.println(timer_counter);
  }
#endif

  // increment or decrement timer
  if (timer_millis + TIMER_DIVIDER < millis()) {
    uint16_t seconds_passed = (millis() - timer_millis) / TIMER_DIVIDER;
    timer_millis = millis();
    if (timer_active) {
      timer_counter += (ControlType == 0 ? seconds_passed : -seconds_passed); // increment only in manual mode.
    }
  }

  // DO LOW PRIORITY TASKS
  BuzzerHandler();
}


// PWM output
void doSoftwarePWM(uint16_t pwm_val) {
  if (millis() - soft_pwm_millis > PID_WINDOWSIZE)
  { //time to shift the Relay Window
    soft_pwm_millis += PID_WINDOWSIZE;
  }
  if (pwm_val < millis() - soft_pwm_millis) {
    H_OFF
  } else {
    H_ON
  }

}

void doManualReflow() {
  // draw screen
#if defined (OLED) && !defined (LOGGER)
  u8g2.clearBuffer();
  printManual();

  if (value_editable == 1) {
    u8g2.drawBox(46, 0, 33, 10);
  }

  printPresetTemperature();
  printTime(timer_counter);
  printCurrentTemperature();

  inlineLogger_fill();
  inlineLogger_draw();

  printHeaterState(); //print icon of the heater ON/OFF state

  u8g2.sendBuffer();
#endif

#ifdef LOGGER
  //  logger128secOLED();
#endif

  if (currentTemp >= (setPoint - 2)) {
    timer_active = true; // Timer running if temperature near or reached preset temp.
  } else {
    timer_active = false;
  }
}

void doAutoReflow() {
  // process current step (variable ControlType)
  switch (ControlType) {
    case 1:	// ramp to preheat temperature
      // setpoint is already set...
      // wait until temperature is reached the auto_preheatTemp. Then go to the next step
      if (currentTemp >= (setPoint - 2)) {
        timer_counter = auto_preheatTime;
        timer_active = true;
        ControlType = 2;	// go to the next step
        BuzzerShortbeep();
      }
      break;
    case 2:	// wait for timer
      if (timer_counter <= 0) {
        timer_counter = 0;
        timer_active = false;
        setPoint = auto_reflowTemp;	// set temperature for next step
        ControlType = 3;	// go to the next step
        BuzzerShortbeep();
      }
      break;
    case 3:	// ramp to reflow temperature
      // wait until temperature is reached the auto_preheatTemp. Then go to the next step
      if (currentTemp >= (setPoint - 2)) {
        timer_counter = auto_reflowTime;
        timer_active = true;
        ControlType = 4;	// go to the next step
        BuzzerShortbeep();
      }
      break;
    case 4:	// wait for timer
      if (timer_counter <= 0) {
        timer_counter = 0;
        timer_active = false;
        setPoint = 20;	// Cool down
        digitalWrite(FAN_PIN, HIGH);
        ControlType = 5;	// go to the next step
        BuzzerLongbeep(); //finished cycle
      }
      break;
    case 5:	// wait for cooling down
      if (currentTemp < 50) {
        ControlType = 6; // on the next step do nothing
        digitalWrite(FAN_PIN, LOW);
        BuzzerShortbeep();
      }
  }

#ifdef OLED
  u8g2.clearBuffer();
  printAuto();
  if (value_editable == 1) {
    u8g2.drawBox(46, 0, 33, 10);
  } else if (value_editable == 2) {
    u8g2.drawBox(85, 0, 33, 10);
  }
  printPresetTemperature();
  printTime(timer_counter);
  printCurrentTemperature();
  printHeaterState(); //print icon of the heater ON/OFF state
  printStepTextGraphic();

  u8g2.sendBuffer();
#endif
}

void adjustValues(uint8_t vals) {
  int val_adjust = 0;	// for adjusting temperature or time on the fly
  char encVal = 127;  // signed value - just enter to the loop
  while (encVal == 127) { //loop here while button is pressed (waiting longer than 2 seconds will reset the board (Exit to the init menu).
    encVal = rotaryEncRead();
    if (encVal != 127 && value_editable > 0) {
      val_adjust = encVal;
    } else if (encVal == 127) {
      H_OFF	// turn off heater, because we will freeze here for some time...
      waitUntilButtonReleased();
      value_editable++;	// change edit mode
      if (value_editable > vals) {
        value_editable = 0;
      }
      value_editable_millis = millis();		// start timer for edit time within 3 seconds
    }
  }
  // adjust current temperature or timer
  if (val_adjust != 0 && ControlType < 5) {
    val_adjust *= 10; // adjust in 10's steps
    if (value_editable == 2) {
      timer_counter = constrain(timer_counter + val_adjust, 0, 990);
    } else if (value_editable == 1) {
      setPoint = constrain((int)setPoint + val_adjust, 20, 300);
    }
    value_editable_millis = millis();
  }
  // reset timer_editable flag if edit time expired
  if (value_editable > 0 && value_editable_millis + 3000 < millis()) {
    value_editable = 0;
  }
}

float getVoltage() {
  float vout = 0.0;
  float vin = 0.0;
  float R1 = 100000.0; // resistance of R1 (100K) -see text!
  float R2 = 10000.0; // resistance of R2 (10K) - see text!
  int value = 0;

  // read the value at analog input
  value = analogRead(VOLTAGE_PIN);
  vout = (value * 5.0) / 1024.0; // see text
  vin = vout / (R2 / (R1 + R2));

  return vin;

}
