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
#include <i2c.h>
#include <Sensors.h>
#include <Motor.h>
#include <Relay.h>
#include <Commands.h>

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

/**
 * @brief An enum for clean defn of addresses
 * 
 */
enum myi2cAddresses
{
  MuxAddr = 0x70,
  OpenedSensorAddr = 0x6F,
  ClosedSensorAddr = 0x6E,
  ActuatorRelay1Addr = 0x18,
  ActuatorRelay2Addr = 0x19,
  MotorControllerAddr = 0x58,
  ForwardDistanceSensorAddr = 0x04,
  RearwardDistanceSensorAddr = 0x05,
  ForwardEndRangeSensorAddr = 0x6D,
  RearwardEndRangeSensorAddr = 0x6C,
  ResurrectionRelayAddr = 0x1A
};

/**
 * @brief This array is passed to the i2c class at begin; it uses it to track all devices.
 * 
 */
i2cDevice my_devices[] = {
    {false, true, myi2cAddresses::MuxAddr, (__FlashStringHelper*)PSTR("Mux"), false},
    {false, false, myi2cAddresses::OpenedSensorAddr, (__FlashStringHelper*)PSTR("Opened Sensor"), false},
    {false, false, myi2cAddresses::ClosedSensorAddr, (__FlashStringHelper*)PSTR("Closed Sensor"), false},
    {false, false, myi2cAddresses::ActuatorRelay1Addr, (__FlashStringHelper*)PSTR("Actuator Relay1"), false},
    {false, false, myi2cAddresses::ActuatorRelay2Addr, (__FlashStringHelper*)PSTR("Actuator Relay2"), false},
    {false, false, myi2cAddresses::MotorControllerAddr, (__FlashStringHelper*)PSTR("Motor Controller"), false},
    {true, false, myi2cAddresses::ForwardDistanceSensorAddr, (__FlashStringHelper*)PSTR("Forward Distance Sensor"), false},
    {true, false, myi2cAddresses::RearwardDistanceSensorAddr, (__FlashStringHelper*)PSTR("Rearward Distance Sensor"), false},
    {false, false, myi2cAddresses::ForwardEndRangeSensorAddr, (__FlashStringHelper*)PSTR("Forward End-range Sensor"), false},
    {false, false, myi2cAddresses::RearwardEndRangeSensorAddr, (__FlashStringHelper*)PSTR("Rearward End-range Sensor"), false},
    {false, false, myi2cAddresses::ResurrectionRelayAddr, (__FlashStringHelper*)PSTR("Resurrection Relay"), false},
};

/**
 * @brief all of our i2c devices
 * 
 */
DistanceSensor _fwdDistanceSensor = DistanceSensor(myi2cAddresses::ForwardDistanceSensorAddr);
DistanceSensor _rwdDistanceSensor = DistanceSensor(myi2cAddresses::RearwardDistanceSensorAddr);
QwiicContactSensor _fwdEndRangeSensor = QwiicContactSensor(myi2cAddresses::ForwardEndRangeSensorAddr);
QwiicContactSensor _rwdEndRangeSensor = QwiicContactSensor(myi2cAddresses::RearwardEndRangeSensorAddr);
QwiicContactSensor _openedSensor = QwiicContactSensor(myi2cAddresses::OpenedSensorAddr);
QwiicContactSensor _closedSensor = QwiicContactSensor(myi2cAddresses::ClosedSensorAddr);
Relay _actuatorRelays[2] = {myi2cAddresses::ActuatorRelay1Addr, myi2cAddresses::ActuatorRelay2Addr};
Relay _ressurectRelay = myi2cAddresses::ResurrectionRelayAddr;
Motor _motorController = myi2cAddresses::MotorControllerAddr;

/**
 * @brief Commands for the shell/serial command processor.
 * 
 */
ShellCommandRegister* cmdLog = ShellCommandClass(log, "Sets logging - [serial*|shell] [verbose*|info|silent]", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  int level = LOG_LEVEL_VERBOSE;
  if (argc > 2 && !strcmp_P(argv[2], PSTR("verbose"))) {
    level = LOG_LEVEL_VERBOSE;
  } else if (argc > 2 && !strcmp_P(argv[2], PSTR("info"))) {
    level = LOG_LEVEL_INFO;
  } else if (argc > 2 && !strcmp_P(argv[2], PSTR("silent"))) {
    level = LOG_LEVEL_SILENT;
  }

  if (argc > 1 && !strcmp_P(argv[1], PSTR("serial"))) {
    Log.begin(level, &Serial, false);
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("shell"))) {
    Log.begin(level, &shell, false);
  }
});

ShellCommandRegister* cmdShell = ShellCommandClass(shell, "Configures shell - [telnet*|raw]", {
  Commands::getInstance().logCommand(command->name, argc, argv);
  if (argc > 1 && !strcmp_P(argv[1], PSTR("raw"))) {
    shell.setEcho(false);
    shell.setPrompt(nullptr);
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("telnet"))) {
    shell.setEcho(true);
    shell.setPrompt(Commands::getInstance().shellPrompt);
  }
});

ShellCommandRegister* cmdReboot = ShellCommandClass(reboot, "Reboots the microcontroller", {
  Commands::getInstance().logCommand(command->name, argc, argv);
  Log.errorln("REBOOTING IN 1 second.");
  P1.configWD(1000, TOGGLE);
  while (1) {
    delay(5000);
  }
});

ShellCommandRegister* cmdInit = ShellCommandClass(init, "Inits and tests - [all|i2c|mux|fwd|rwd|motor|opened|closed|fwdend|rwdend|relay1|relay2|resurrect]", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("all"))) {
    setup();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("i2c"))) {
    Bus.scanI2C(30);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("mux"))) {
    Bus.testI2CMux(30);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("fwd"))) {
    _fwdDistanceSensor.begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("rwd"))) {
    _rwdDistanceSensor.begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("motor"))) {
    _motorController.begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("fwdend"))) {
    _fwdEndRangeSensor.begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("rwdend"))) {
    _rwdEndRangeSensor.begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("opened"))) {
    _openedSensor.begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("closed"))) {
    _closedSensor.begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("relay1"))) {
    _actuatorRelays[0].begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("relay2"))) {
    _actuatorRelays[1].begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("resurrect"))) {
    _ressurectRelay.begin();
  }
});

ShellCommandRegister* cmdGet = ShellCommandClass(get, "Gets state - [fwd|rwd|opened|closed|fwdend|rwdend|relay1|relay2|resurrect]", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("fwd"))) {
    _fwdDistanceSensor.getDistance();
    Log.noticeln(_fwdDistanceSensor);
    shell.println(_fwdDistanceSensor._cachedDistance, DEC);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("rwd"))) {
    _rwdDistanceSensor.getDistance();
    Log.noticeln(_rwdDistanceSensor);
    shell.println(_rwdDistanceSensor._cachedDistance, DEC);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("fwdend"))) {
    Log.noticeln(_fwdEndRangeSensor);
    shell.println(_fwdEndRangeSensor);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("rwdend"))) {
    Log.noticeln(_rwdEndRangeSensor);
    shell.println(_rwdEndRangeSensor);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("opened"))) {
    Log.noticeln(_openedSensor);
    shell.println(_openedSensor);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("closed"))) {
    Log.noticeln(_closedSensor);
    shell.println(_closedSensor);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("relay1"))) {
    Log.noticeln(F("%d"), _actuatorRelays[0].getState());
    shell.println(_actuatorRelays[0].getState(), DEC);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("relay2"))) {
    Log.noticeln(F("%d"), _actuatorRelays[1].getState());
    shell.println(_actuatorRelays[1].getState(), DEC);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("resurrect"))) {
    Log.noticeln(F("%d"), _ressurectRelay.getState());
    shell.println(_ressurectRelay.getState(), DEC);
  }
});

ShellCommandRegister* cmdRelay = ShellCommandClass(relay, "Controls a relay - [1|2|res] ([on|off|toggle])", {
  Commands::getInstance().logCommand(command->name, argc, argv);
  Relay* relay = nullptr;

  if (argc > 1 && !strcmp_P(argv[1], PSTR("1"))) {
    relay = &_actuatorRelays[0];
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("2"))) {
    relay = &_actuatorRelays[1];
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("res"))) {
    relay = &_ressurectRelay;
  }

  if (argc > 2 && relay != nullptr) {
    if (!strcmp_P(argv[2], PSTR("on"))) {
      relay->turnRelayOn();
    }
    if (!strcmp_P(argv[2], PSTR("off"))) {
      relay->turnRelayOff();
    }
    if (!strcmp_P(argv[2], PSTR("toggle"))) {
      relay->toggleRelay();
    }
  }
  if (relay != nullptr) {
    relay->probe();
    shell.println(*relay);
  }
});

ShellCommandRegister* cmdMotor = ShellCommandClass(motor, "Controls the motor - [speed (0-255)|accel (0-255)|fwd|rev|stop|off]", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("fwd"))) {
    _motorController.forward();
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("rev"))) {
    _motorController.reverse();
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("stop"))) {
    _motorController.setSpeed(0);
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("off"))) {
    _motorController.emergencyStop();
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("speed"))) {
    if (argc > 2) {
      _motorController.setSpeed(strtol(argv[2], nullptr, 10));
    } else {
      _motorController.probe();
      shell.println(_motorController.getSpeed(), DEC);
      return;
    }
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("accel"))) {
    if (argc > 2) {
      _motorController.setAcceleration(strtol(argv[2], nullptr, 10));
    } else {
      _motorController.probe();
      shell.println(_motorController.getDirectionString());
      return;
    }
  }
  _motorController.probe();
  shell.println(_motorController);
});

ShellCommandRegister* cmdLA = ShellCommandClass(la, "Controls linear actuator via relays 1 & 2 - ([extend|retract|off])", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("extend"))) {
    _actuatorRelays[0].turnRelayOn();
    _actuatorRelays[1].turnRelayOff();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("retract"))) {
    _actuatorRelays[1].turnRelayOn();
    _actuatorRelays[0].turnRelayOff();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("off"))) {
    _actuatorRelays[0].turnRelayOff();
    _actuatorRelays[1].turnRelayOff();
  }
  shell.print(_actuatorRelays[0]);
  shell.print(F(", "));
  shell.println(_actuatorRelays[1]);
});

/**
 * @brief good 'ole Arudinio setup()
 * 
 */
void setup(void) {
  bool success = true;
  waitForSerial(10000);  // wait no more than 10 seconds
  Log.begin(LOG_LEVEL_VERBOSE, &Serial, false);

  Log.noticeln(F("------------- I2C / Qwiic Bus Tester "));

  Cmds.begin();
  Wire.begin();

  Log.noticeln(F("------------- I2C Bus Setup"));

  if (!Bus.begin(my_devices, sizeof(my_devices) / sizeof(i2cDevice))) {
    success = false;
    Log.errorln(F("ERROR: Bus.begin failed"));
  } else {
    Log.noticeln(F("The I2C bus initialzied"));
  }

  // scan the bus and report all devices found
  Log.noticeln(F("------------- I2C Bus Scan "));
  Bus.scanI2C(10);

  // scan the mux and report all devices found
  //Log.noticeln(F("------------- Mux test "));
  //Bus.testI2CMux(10);

  Log.noticeln(F("------------- Forward Distance Sensor "));
  _fwdDistanceSensor.begin();
  Log.noticeln(F("------------- Rearward Distance Sensor "));
  _rwdDistanceSensor.begin();

  Log.noticeln(F("------------- Forward End-Range Contact Sensor "));
  _fwdEndRangeSensor.begin();
  Log.noticeln(F("------------- Rearward End-Range Contact Sensor "));
  _rwdEndRangeSensor.begin();

  Log.noticeln(F("------------- Door Opened Contact Sensor "));
  _openedSensor.begin();
  Log.noticeln(F("------------- Door Closed Contact Sensor "));
  _closedSensor.begin();

  Log.noticeln(F("------------- Linear Actuator Relay 1 "));
  _actuatorRelays[0].begin();
  Log.noticeln(F("------------- Linear Actuator Relay 2 "));
  _actuatorRelays[1].begin();

  Log.noticeln(F("------------- Resurrection Relay"));
  _ressurectRelay.begin();

  // setup the motor
  Log.noticeln(F("------------- Motor Controller "));
  _motorController.begin();

  if (success == false) {
    Log.errorln(F("------------- TESTS DID NOT PASS "));
  }
  Log.noticeln(F("------------- Initialization finished. Ready for commands."));
}

void loop() {
  // listen for and process REST commands from Ethernet
  // and button presses from keypad
  Cmds.handle();
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