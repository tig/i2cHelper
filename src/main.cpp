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
#include <Adafruit_BusIO_register.h>
#include <SparkFun_VL53L1X.h>
#include <SparkFun_Qwiic_Button.h>
#include <SparkFun_Qwiic_relay.h>

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
  ForwardDistanceSensorAddr = 0x29,   // VL53L1X hard wired i2c address 0x52 bit shifted
  RearwardDistanceSensorAddr = 0x29,  // VL53L1X hard wired i2c address 0x52 bit shifted
  ForwardEndRangeSensorAddr = 0x6D,
  RearwardEndRangeSensorAddr = 0x6C,
  ResurrectionRelayAddr = 0x1A
};

/**
 * @brief the order of these must match the order of the devices in my_devices!
 * TODO: build in a check.
 * 
 */
enum myi2cDevices
{
  Mux,
  OpenedSensor,
  ClosedSensor,
  ActuatorRelay1,
  ActuatorRelay2,
  MotorController,
  ForwardDistanceSensor,
  RearwardDistanceSensor,
  ForwardEndRangeSensor,
  RearwardEndRangeSensor,
  ResurrectionRelay
};

i2cDevice* _mux = new i2cDevice(myi2cAddresses::MuxAddr, 0xFF, (__FlashStringHelper*)PSTR("Qwiic Mux"), new QWIICMUX(), true);
QwiicContactSensor* _openedSensor = new QwiicContactSensor(myi2cAddresses::OpenedSensorAddr, 0xFF, (__FlashStringHelper*)PSTR("Opened Sensor"), nullptr);
QwiicContactSensor* _closedSensor = new QwiicContactSensor(myi2cAddresses::ClosedSensorAddr, 0xFF, (__FlashStringHelper*)PSTR("Closed Sensor"), nullptr);
QwiicRelay* _actuatorRelay1 = new QwiicRelay(myi2cAddresses::ActuatorRelay1Addr, 0xFF, (__FlashStringHelper*)PSTR("Actuator Relay1"), nullptr);
QwiicRelay* _actuatorRelay2 = new QwiicRelay(myi2cAddresses::ActuatorRelay2Addr, 0xFF, (__FlashStringHelper*)PSTR("Actuator Relay2"), nullptr);
Motor* _motorController = new Motor(myi2cAddresses::MotorControllerAddr, 0xFF, (__FlashStringHelper*)PSTR("Motor Controller"), nullptr);
VL53L1XDistanceSensor* _forwardDistanceSensor = new VL53L1XDistanceSensor(myi2cAddresses::ForwardDistanceSensorAddr, 0x04, (__FlashStringHelper*)PSTR("Forward Distance Sensor"), new QWIICMUX());
VL53L1XDistanceSensor* _rearwardDistanceSensor = new VL53L1XDistanceSensor(myi2cAddresses::RearwardDistanceSensorAddr, 0x05, (__FlashStringHelper*)PSTR("Rearward Distance Sensor"), new QWIICMUX());
QwiicContactSensor* _forwardEndRangeSensor = new QwiicContactSensor(myi2cAddresses::ForwardEndRangeSensorAddr, 0x04, (__FlashStringHelper*)PSTR("Forward End-range Sensor"), new QWIICMUX());
QwiicContactSensor* _rearwardEndRangeSensor = new QwiicContactSensor(myi2cAddresses::RearwardEndRangeSensorAddr, 0x05, (__FlashStringHelper*)PSTR("Rearward End-range Sensor"), new QWIICMUX());
QwiicRelay* _resurrectionRelay = new QwiicRelay(myi2cAddresses::ResurrectionRelayAddr, 0xFF, (__FlashStringHelper*)PSTR("Resurrection Relay"), nullptr);

/**
 * @brief This array is passed to the i2c class at begin; it uses it to track all devices.
 * 
 */
i2cDevice* my_devices[] = {
    _mux,
    _openedSensor,
    _closedSensor,
    _actuatorRelay1,
    _actuatorRelay2,
    _motorController,
    _forwardDistanceSensor,
    _rearwardDistanceSensor,
    _forwardEndRangeSensor,
    _rearwardEndRangeSensor,
    _resurrectionRelay};

/**
 * @brief Commands for the shell/serial command processor.
 * 
 */
ShellCommandRegister* cmdLog = ShellCommandClass(log, "Sets logging - [serial*|shell]|[verbose*|info|silent]", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  Log.noticeln(F("changing shell"));
  int level = Log.getLevel();
  if (argc > 2 && !strcmp_P(argv[2], PSTR("verbose"))) {
    level = LOG_LEVEL_VERBOSE;
  } else if (argc > 2 && !strcmp_P(argv[2], PSTR("info"))) {
    level = LOG_LEVEL_INFO;
  } else if (argc > 2 && !strcmp_P(argv[2], PSTR("silent"))) {
    level = LOG_LEVEL_SILENT;
  }
  Log.setLevel(level);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("serial"))) {
    Log.begin(level, &Serial, false);
    Log.noticeln(F("serial"));
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("shell"))) {
    Log.begin(level, &shell, false);
    Log.noticeln(F("shell"));
  }
});

ShellCommandRegister* cmdShell = ShellCommandClass(shell, "Configures shell - [telnet*|raw]", {
  Commands::getInstance().logCommand(command->name, argc, argv);
  if (argc > 1 && !strcmp_P(argv[1], PSTR("raw"))) {
    Commands::getInstance().setRaw(true);
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("telnet"))) {
    Commands::getInstance().setRaw(false);
  }
});

ShellCommandRegister* cmdInit = ShellCommandClass(init, "Inits and tests - [all|i2c|mux|fwd|rwd|motor|opened|closed|fwdend|rwdend|relay1|relay2|resurrect]", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  bool success = true;
  if (argc > 1 && !strcmp_P(argv[1], PSTR("all"))) {
    success = begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("i2c"))) {
    success = Bus.scanI2C(30);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("mux"))) {
    success = Bus.testI2CMux(30);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("fwd"))) {
    success = _forwardDistanceSensor->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("rwd"))) {
    success = _rearwardDistanceSensor->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("motor"))) {
    success = _motorController->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("fwdend"))) {
    success = _forwardEndRangeSensor->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("rwdend"))) {
    success = _rearwardEndRangeSensor->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("opened"))) {
    success = _openedSensor->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("closed"))) {
    success = _closedSensor->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("relay1"))) {
    success = _actuatorRelay1->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("relay2"))) {
    success = _actuatorRelay2->begin();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("resurrect"))) {
    success = _resurrectionRelay->begin();
  }
  shell.println(success, DEC);
});

ShellCommandRegister* cmdGet = ShellCommandClass(get, "Gets state - [all|motor|fwd|rwd|opened|closed|fwdend|rwdend|relay1|relay2|resurrect]", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  bool all = (argc > 1 && !strcmp_P(argv[1], PSTR("all")));

  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("motor")))) {
    _motorController->probe();
    Log.noticeln(F("%p"), *_motorController);
    Commands::getInstance()._telnetShell.println(*_motorController);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("fwd")))) {
    _forwardDistanceSensor->distance();
    Log.noticeln(F("%p"), *_forwardDistanceSensor);
    Commands::getInstance()._telnetShell.println(*_forwardDistanceSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("rwd")))) {
    _rearwardDistanceSensor->distance();
    Log.noticeln(F("%p"), *_rearwardDistanceSensor);
    Commands::getInstance()._telnetShell.println(*_rearwardDistanceSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("fwdend")))) {
    _forwardEndRangeSensor->isContacted();
    Log.noticeln(F("%p"), *_forwardEndRangeSensor);
    Commands::getInstance()._telnetShell.println(*_forwardEndRangeSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("rwdend")))) {
    _rearwardEndRangeSensor->isContacted();
    Log.noticeln(F("%p"), *_rearwardEndRangeSensor);
    Commands::getInstance()._telnetShell.println(*_rearwardEndRangeSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("opened")))) {
    Log.noticeln(F("%p"), *_openedSensor);
    Commands::getInstance()._telnetShell.println(*_openedSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("closed")))) {
    Log.noticeln(F("%p"), *_closedSensor);
    Commands::getInstance()._telnetShell.println(*_closedSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("relay1")))) {
    Log.noticeln(F("%p"), *_actuatorRelay1);
    Commands::getInstance()._telnetShell.println(*_actuatorRelay1);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("relay2")))) {
    Log.noticeln(F("%p"), *_actuatorRelay2);
    Commands::getInstance()._telnetShell.println(*_actuatorRelay2);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("resurrect")))) {
    Log.noticeln(F("%p"), *_resurrectionRelay);
    Commands::getInstance()._telnetShell.println(*_resurrectionRelay);
  }
});

ShellCommandRegister* cmdRelay = ShellCommandClass(relay, "Controls a relay - [1|2|res] ([on|off|toggle])", {
  Commands::getInstance().logCommand(command->name, argc, argv);
  Relay* relay = nullptr;

  if (argc > 1 && !strcmp_P(argv[1], PSTR("1"))) {
    relay = _actuatorRelay1;
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("2"))) {
    relay = _actuatorRelay2;
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("res"))) {
    relay = _resurrectionRelay;
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
    Commands::getInstance()._telnetShell.println(*relay);
  }
});

ShellCommandRegister* cmdMotor = ShellCommandClass(motor, "Controls the motor - [speed (0-255)|accel (0-255)|fwd|rev|stop|off]", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("fwd"))) {
    _motorController->forward();
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("rev"))) {
    _motorController->reverse();
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("stop"))) {
    _motorController->setSpeed(0);
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("off"))) {
    _motorController->emergencyStop();
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("speed"))) {
    if (argc > 2) {
      _motorController->setSpeed(strtol(argv[2], nullptr, 10));
    } else {
      _motorController->probe();
      Commands::getInstance()._telnetShell.println(_motorController->speed(), DEC);
      return;
    }
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("accel"))) {
    if (argc > 2) {
      _motorController->setAcceleration(strtol(argv[2], nullptr, 10));
    } else {
      _motorController->probe();
      Commands::getInstance()._telnetShell.println(_motorController->directionString());
      return;
    }
  }
  _motorController->probe();
  Commands::getInstance()._telnetShell.println(*_motorController);
});

ShellCommandRegister* cmdLA = ShellCommandClass(la, "Controls linear actuator via relays 1 & 2 - ([extend|retract|off])", {
  Commands::getInstance().logCommand(command->name, argc, argv);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("extend"))) {
    _actuatorRelay1->turnRelayOn();
    _actuatorRelay2->turnRelayOff();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("retract"))) {
    _actuatorRelay2->turnRelayOn();
    _actuatorRelay1->turnRelayOff();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("off"))) {
    _actuatorRelay1->turnRelayOff();
    _actuatorRelay2->turnRelayOff();
  }
  Commands::getInstance()._telnetShell.print(*_actuatorRelay1);
  Commands::getInstance()._telnetShell.print(F(", "));
  shell.println(*_actuatorRelay2);
});

/**
 * @brief Inits all bus related stuff; called from `setup()` and the `init` command.
 * 
 */
bool begin() {
  bool success = true;

  Log.noticeln(F("------------- I2C Bus Setup"));

  Log.noticeln(F("------------- Qwiic Mux"));
  if (!_mux->begin()) {
    success = false;
  }

  Log.noticeln(F("------------- I2C Bus "));
  if (!Bus.begin(my_devices, sizeof(my_devices) / sizeof(i2cDevice*))) {
    success = false;
    Log.errorln(F("ERROR: Bus.begin failed"));
  } else {
    Log.noticeln(F("The I2C bus initialzied"));
  }

  // scan the bus and report all devices found
  // Log.noticeln(F("------------- I2C Bus Scan "));
  // if (!Bus.scanI2C(10)) {
  //   success = false;
  // }

  // scan the mux and report all devices found
  //Log.noticeln(F("------------- Mux test "));
  //Bus.testI2CMux(10);

  Log.noticeln(F("------------- Forward Distance Sensor "));

  Log.traceln(F("Hard coded i2cDevice::begin - %S - (%X:%X) isMux = %T, mux &%X - deviceAddress = %X"),
      _forwardDistanceSensor->name(), _forwardDistanceSensor->address(), _forwardDistanceSensor->muxPort(), _forwardDistanceSensor->isMux(), _forwardDistanceSensor->mux(), _forwardDistanceSensor->mux()->getAddress());
  assert(_forwardDistanceSensor->mux());

  if (_mux->mux()->setPort(_forwardDistanceSensor->muxPort())) {
    Log.traceln(F("setPort worked"));
  } else {
    Log.traceln(F("setPort failed"));
  }

  Log.traceln("\nProbe: ");
  if (!Bus.probeDevice(_forwardDistanceSensor->address(), _forwardDistanceSensor->muxPort(), (const char*)_forwardDistanceSensor->name(), 100)) {
    Log.traceln(F("ERROR probe failed"));
  }
  Log.traceln("  - done probing\n");

  QWIICMUX* _testmux = new QWIICMUX();
  Log.traceln(F("Manual QWIICMUX setup - %S - (%X:%X) isMux = %T, mux &%X - deviceAddress = %X"),
      _forwardDistanceSensor->name(), _forwardDistanceSensor->address(), _forwardDistanceSensor->muxPort(),
      _forwardDistanceSensor->isMux(), _testmux, _testmux->getAddress());
  bool b = _testmux->begin(0x70);
  if (b) {
    Log.traceln(F(" begin OK - %S - (%X:%X) isMux = %T, mux &%X - deviceAddress = %X"),
        _forwardDistanceSensor->name(), _forwardDistanceSensor->address(), _forwardDistanceSensor->muxPort(),
        _forwardDistanceSensor->isMux(), _testmux, _testmux->getAddress());

    if (!_testmux->setPort(_forwardDistanceSensor->muxPort())) {
      Log.errorln(F("    ERROR: QWIICMUX couldn't set the port."));
      return false;
    } else {
      Log.errorln(F("    YAY: QWIICMUX set the port."));
      Log.traceln("\nProbe: ");
      if (!Bus.probeDevice(_forwardDistanceSensor->address(), _forwardDistanceSensor->muxPort(), (const char*)_forwardDistanceSensor->name(), 100)) {
        Log.traceln(F("ERROR probe failed"));
      }
      Log.traceln("  - done probing\n");
    }

  } else {
    Log.traceln(F(" begin FAILED - %S - (%X:%X) isMux = %T, mux &%X - deviceAddress = %X"),
        _forwardDistanceSensor->name(), _forwardDistanceSensor->address(), _forwardDistanceSensor->muxPort(),
        _forwardDistanceSensor->isMux(), _testmux, _testmux->getAddress());
  }

  // //delay(1000);
  // if (_mux->mux()->setPort(_forwardDistanceSensor->muxPort())) {
  //   Log.errorln(F("    ERROR: QWIICMUX couldn't set the port."));
  //   return false;
  // }

  Log.traceln("\nProbe: ");
  if (!Bus.probeDevice(_forwardDistanceSensor->address(), _forwardDistanceSensor->muxPort(), (const char*)_forwardDistanceSensor->name(), 100)) {
    Log.traceln(F("ERROR probe failed"));
  }
  Log.traceln("  - done probing\n");

  if (!_forwardDistanceSensor->begin()) {
    success = false;
  }
  Log.noticeln(F("------------- Rearward Distance Sensor "));
  if (!_rearwardDistanceSensor->begin()) {
    success = false;
  }

  Log.noticeln(F("------------- Forward End-Range Contact Sensor "));
  if (!_forwardEndRangeSensor->begin()) {
    success = false;
  }
  Log.noticeln(F("------------- Rearward End-Range Contact Sensor "));
  if (!_rearwardEndRangeSensor->begin()) {
    success = false;
  }

  Log.noticeln(F("------------- Door Opened Contact Sensor "));
  if (!_openedSensor->begin()) {
    success = false;
  }
  Log.noticeln(F("------------- Door Closed Contact Sensor "));
  if (!_closedSensor->begin()) {
    success = false;
  }

  Log.noticeln(F("------------- Linear Actuator Relay 1 "));
  if (!_actuatorRelay1->begin()) {
    success = false;
  }
  Log.noticeln(F("------------- Linear Actuator Relay 2 "));
  if (!_actuatorRelay2->begin()) {
    success = false;
  }

  Log.noticeln(F("------------- Resurrection Relay"));
  if (!_resurrectionRelay->begin()) {
    success = false;
  }

  // setup the motor
  Log.noticeln(F("------------- Motor Controller "));
  if (!_motorController->begin()) {
    success = false;
  }

  Log.noticeln(F("------------- Initialization finished. Ready for commands."));
  return success;
}

/**
 * @brief good 'ole Arudinio setup()
 * 
 */
void setup(void) {
  waitForSerial(10000);  // wait no more than 10 seconds
  Log.begin(LOG_LEVEL_VERBOSE, &Serial, false);
  Log.noticeln(F("------------- I2C / Qwiic Bus Tester "));

  if (!Cmds.begin()) {
    Log.errorln(F("ERROR: Commands failed to initalize"));
  }

#if defined(ARDUINO_ARCH_ESP32)
  if (!Wire.begin()) {
    Log.errorln(F("ERROR: Wire.begin failed"));
  } else {
    Log.noticeln(F("Wire initialzied"));
  }
#else
  Wire.begin();
  Log.noticeln(F("Wire initialzied"));
#endif

  //begin();

  //Cmds._serialShell.help();
}

void loop() {
  // listen for and process REST commands from Ethernet
  // and button presses from keypad
  Cmds.handle();
}

// Debug & Logging helpers
// handle diagnostic informations given by assertion and abort program
// execution:
#ifdef NDEBUG
void __assert(const char* __func, const char* __file, int __lineno, const char* __sexp) {
#else
void __assert_func(const char* __file, int __lineno, const char* __func, const char* __sexp) {
#endif
#ifndef DRV_DEBUG_H_
  Serial.flush();
#endif
  // transmit diagnostic information
  char out[255];
  sprintf_P(out, PSTR("\nFATAL: %s:%d: assert: '%s' in %s"), __file, __lineno, __sexp, __func);

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