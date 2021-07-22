/**
 * @file main.cpp
 * @author Charlie Kindel
 */

// Libraries
#include <Wire.h>
#include <Arduino.h>
#include <ArduinoLog.h>
#include <assert.h>

#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>
#include <SparkFun_VL53L1X.h>
#include <SparkFun_Qwiic_Button.h>
#include <SparkFun_Qwiic_Relay.h>
#include <P1AM.h>

#define HALT_ON_ASSERT

//#include "avr8-stub.h"

#include "main.h"
#include "i2c.h"
#include "Api.h"
#include "Sensors.h"
#include "Motor.h"

// Wait for serial
#ifndef DRV_DEBUG_H_
void waitForSerial(unsigned long timeout_millis) {
  // Start Serial
  Serial.begin(115200);

  unsigned long start = millis();

  while (!Serial && !Serial.available()) {
    if (millis() - start > timeout_millis) break;
  }
}
#endif

Api& api = Api::getInstance();

// Devices
DistanceSensor fwdDistanceSensor = DistanceSensor(i2c::i2cDevices::ForwardDistanceSensorAddr);
DistanceSensor rwdDistanceSensor = DistanceSensor(i2c::i2cDevices::RearwardDistanceSensorAddr);

QwiicContactSensor openedSensor;
QwiicContactSensor closedSensor;

static const uint16_t ACTUATOR_RELAY1_ADDR = 0x18;  // First relay of LA H bridge
static const uint16_t ACTUATOR_RELAY2_ADDR = 0x19;  // Second relay of LA H bridge
Qwiic_Relay _relays[2] = {ACTUATOR_RELAY1_ADDR, ACTUATOR_RELAY2_ADDR};


Motor motor;

void setup(void) {
  waitForSerial(10000);  // wait no more than 10 seconds
  
  Log.begin(LOG_LEVEL_VERBOSE, &Serial, false);
  Log.noticeln(F("------------- TV Slider TEST RIG "));
  
  api.begin();
  Wire.begin();

  bool success = true;
  // setup the i2c bus
  if (!Bus.begin()) {
    success = false;
    Log.errorln(F("ERROR: Bus.begin failed"));
  }

  // scan the bus and report all devices found
  Log.noticeln(F("------------- I2C Bus Scan "));
  Bus.scanI2C(10);

  // scan the mux and report all devices found
  //Log.noticeln(F("------------- Mux test "));
  //Bus.testI2CMux(10);

  Log.noticeln(F("------------- Forward Distance Sensor "));
  fwdDistanceSensor.begin();
  Log.noticeln(F("------------- Rearward Distance Sensor "));
  rwdDistanceSensor.begin();

  // setup the motor
  Log.noticeln(F("------------- Motor Controller "));
  motor.begin();

  if (success == false) {
    Log.errorln(F("------------- TESTS DID NOT PASS "));
  }
}

void loop() {
  // listen for and process REST commands from Ethernet
  // and button presses from keypad
  api.handle();
}

// Debug & Logging helpers
// handle diagnostic informations given by assertion and abort program
// execution:
void __assert(const char* __func, const char* __file, int __lineno, const char* __sexp) {
#ifndef DRV_DEBUG_H_
  Serial.flush();
#endif
  // transmit diagnostic information
  char out[255];
  sprintf_P(out, PSTR("FATAL: %s:%d: assert: '%s' in %s()"), __file, __lineno, __sexp, __func);

  Log.begin(LOG_LEVEL_VERBOSE, &Serial, false);
  Log.fatalln(out);

#ifdef HALT_ON_ASSERT
  Log.fatalln(F("program execution halted"));
#endif

#ifndef DRV_DEBUG_H_
  Serial.flush();
#endif

#ifdef HALT_ON_ASSERT
  // abort program execution.
  abort();
#endif
}

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else   // __ARM__
extern char* __brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else   // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

// Used for logging with AdruinoLog
void printPrefix(Print* _logOutput, int logLevel) {
  printTimestamp(_logOutput);
  //printLogLevel (_logOutput, logLevel);
}

void printTimestamp(Print* _logOutput) {
  // Division constants
  const long MSECS_PER_SEC = 1000;
  const long SECS_PER_MIN = 60;
  const long SECS_PER_HOUR = 3600;
  const long SECS_PER_DAY = 86400;

  // Total time
  const long msecs = millis();
  const long secs = msecs / MSECS_PER_SEC;

  // Time in components
  const long MilliSeconds = msecs % MSECS_PER_SEC;
  const long Seconds = secs % SECS_PER_MIN;
  const long Minutes = (secs / SECS_PER_MIN) % SECS_PER_MIN;
  const long Hours = (secs % SECS_PER_DAY) / SECS_PER_HOUR;

  // Time as string
  char timestamp[20];
  sprintf_P(timestamp, PSTR("%02ld:%02ld:%02ld.%03ld "), Hours, Minutes, Seconds, MilliSeconds);
  _logOutput->print(timestamp);
}