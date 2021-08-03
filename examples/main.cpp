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
  ForwardEndRangeSensorAddr = 0x6C,   // reprogrammed
  RearwardEndRangeSensorAddr = 0x6C,  // jumpers soldered
  ResurrectionRelayAddr = 0x1A
};

i2cDevice* _mux = new i2cDevice(myi2cAddresses::MuxAddr, 0xFF, myi2cAddresses::MuxAddr, (__FlashStringHelper*)PSTR("Qwiic Mux"), true);
QwiicContactSensor* _openedSensor = new QwiicContactSensor(myi2cAddresses::OpenedSensorAddr, 0xFF, 0xFF, (__FlashStringHelper*)PSTR("Opened Sensor"));
QwiicContactSensor* _closedSensor = new QwiicContactSensor(myi2cAddresses::ClosedSensorAddr, 0xFF, 0xFF, (__FlashStringHelper*)PSTR("Closed Sensor"));
QwiicRelay* _actuatorRelay1 = new QwiicRelay(myi2cAddresses::ActuatorRelay1Addr, 0xFF, 0xFF, (__FlashStringHelper*)PSTR("Actuator Relay1"));
QwiicRelay* _actuatorRelay2 = new QwiicRelay(myi2cAddresses::ActuatorRelay2Addr, 0xFF, 0xFF, (__FlashStringHelper*)PSTR("Actuator Relay2"));
AdafruitMotorController* _motorController = new AdafruitMotorController(myi2cAddresses::MotorControllerAddr, 0xFF, 0xFF, (__FlashStringHelper*)PSTR("Motor Controller"));
VL53L1XDistanceSensor* _forwardDistanceSensor = new VL53L1XDistanceSensor(myi2cAddresses::ForwardDistanceSensorAddr, 0x04, myi2cAddresses::MuxAddr, (__FlashStringHelper*)PSTR("Forward Distance Sensor"));
VL53L1XDistanceSensor* _rearwardDistanceSensor = new VL53L1XDistanceSensor(myi2cAddresses::RearwardDistanceSensorAddr, 0x05, myi2cAddresses::MuxAddr, (__FlashStringHelper*)PSTR("Rearward Distance Sensor"));
QwiicContactSensor* _forwardEndRangeSensor = new QwiicContactSensor(myi2cAddresses::ForwardEndRangeSensorAddr, 0x04, myi2cAddresses::MuxAddr, (__FlashStringHelper*)PSTR("Forward End-range Sensor"));
QwiicContactSensor* _rearwardEndRangeSensor = new QwiicContactSensor(myi2cAddresses::RearwardEndRangeSensorAddr, 0x05, myi2cAddresses::MuxAddr, (__FlashStringHelper*)PSTR("Rearward End-range Sensor"));
QwiicRelay* _resurrectionRelay = new QwiicRelay(myi2cAddresses::ResurrectionRelayAddr, 0xFF, 0xFF, (__FlashStringHelper*)PSTR("Resurrection Relay"));

ContactSensor* _esp32Button = new ContactSensor(0xFF, 0xFF, 0xFF, (__FlashStringHelper*)PSTR("ESP32 Button"));

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
    _resurrectionRelay,
    _esp32Button};

unsigned long DELAY_TIME = 250;
unsigned long _timer = 0;
/**
 * @brief Number of seconds (DELAY_TIME) to log state in the loop
 */
int _logStateInLoopFor = 0;

/**
 * @brief Commands for the shell/serial command processor.
 * 
 */
ShellCommandRegister* cmdLog = ShellCommandClass(log, "Sets logging - [serial*|shell]|[verbose*|info|silent]", {
  Cmds.logCommand(command->name, argc, argv);

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
    Log.noticeln(F("Changing logging to log to %S"), argv[1]);
    Log.begin(level, &Serial, false);
    Log.noticeln(F("serial"));
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("shell"))) {
    Log.noticeln(F("Changing logging to log to %S"), argv[1]);
    Log.begin(level, &shell, false);
    Log.noticeln(F("shell"));
  }
});

ShellCommandRegister* cmdShell = ShellCommandClass(shell, "Configures shell - [telnet*|raw]", {
  Cmds.logCommand(command->name, argc, argv);
  if (argc > 1 && !strcmp_P(argv[1], PSTR("raw"))) {
    shell.setEcho(false);
    shell.setPrompt(nullptr);
    shell.println(F("shell = raw"));
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("telnet"))) {
    shell.setEcho(true);
    shell.setPrompt(Cmds.shellPrompt);
    shell.println(F("shell = telnet"));
  }
});

ShellCommandRegister* cmdInit = ShellCommandClass(init, "Inits - [all|wifi <ssid> <pwd>|mux|fwd|rwd|motor|opened|closed|fwdend|rwdend|relay1|relay2|resurrect] [<n>]", {
  Cmds.logCommand(command->name, argc, argv);

  if (argc == 4 && !strcmp_P(argv[1], PSTR("wifi"))) {
    Cmds.beginServer(argv[2], argv[3]);
    return;
  }

  bool success = true;
  int iterations = 1;
  if (argc > 2) {
    iterations = strtol(argv[2], nullptr, 10);
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("all"))) {
    for (int i = 0; i < iterations; i++) {
      Log.noticeln(F("------------- Initializing all devices"));
      if (!Bus.begin(my_devices, sizeof(my_devices) / sizeof(i2cDevice*))) {
        success = false;
        Log.errorln(F("ERROR: Bus.begin() failed"));
      } else {
        Log.noticeln(F("\n------------- All devices initialzied. The system is functional.\n"));
        success = true;
        _logStateInLoopFor = 1;
      }
    }
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

  if (argc > 1 && !strcmp_P(argv[1], PSTR("mux"))) {
    success = _mux->begin();
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
  shell.println();
  shell.print(command->name);
  shell.print(F(" = "));
  shell.println(success, DEC);
});

ShellCommandRegister* cmdGet = ShellCommandClass(get, "Gets state - [all|i2c|ip|motor|fwd|rwd|opened|closed|fwdend|rwdend|relay1|relay2|resurrect]", {
  Cmds.logCommand(command->name, argc, argv);

  bool all = (argc > 1 && !strcmp_P(argv[1], PSTR("all")));

  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("ip")))) {
    Log.noticeln(F("%p"), &Cmds);
    shell.println(Cmds);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("i2c")))) {
    Log.noticeln(F("------------- Verifying All Devices are Working "));
    Bus.probeAll(10);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("motor")))) {
    _motorController->probe();
    Log.noticeln(F("%p"), *_motorController);
    shell.println(*_motorController);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("fwd")))) {
    _forwardDistanceSensor->distance();
    Log.noticeln(F("%p"), *_forwardDistanceSensor);
    shell.println(*_forwardDistanceSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("rwd")))) {
    _rearwardDistanceSensor->distance();
    Log.noticeln(F("%p"), *_rearwardDistanceSensor);
    shell.println(*_rearwardDistanceSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("fwdend")))) {
    _forwardEndRangeSensor->probe();
    Log.noticeln(F("%p"), *_forwardEndRangeSensor);
    shell.println(*_forwardEndRangeSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("rwdend")))) {
    _rearwardEndRangeSensor->probe();
    Log.noticeln(F("%p"), *_rearwardEndRangeSensor);
    shell.println(*_rearwardEndRangeSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("opened")))) {
    _openedSensor->probe();
    Log.noticeln(F("%p"), *_openedSensor);
    shell.println(*_openedSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("closed")))) {
    _closedSensor->probe();
    Log.noticeln(F("%p"), *_closedSensor);
    shell.println(*_closedSensor);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("relay1")))) {
    _actuatorRelay1->probe();
    Log.noticeln(F("%p"), *_actuatorRelay1);
    shell.println(*_actuatorRelay1);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("relay2")))) {
    _actuatorRelay2->probe();
    Log.noticeln(F("%p"), *_actuatorRelay2);
    shell.println(*_actuatorRelay2);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("resurrect")))) {
    _resurrectionRelay->probe();
    Log.noticeln(F("%p"), *_resurrectionRelay);
    shell.println(*_resurrectionRelay);
  }
  if (all || (argc > 1 && !strcmp_P(argv[1], PSTR("btn")))) {
    _esp32Button->probe();
    Log.noticeln(F("%p"), *_esp32Button);
    shell.println(*_esp32Button);
  }
});

ShellCommandRegister* cmdRelay = ShellCommandClass(relay, "Controls a relay - [1|2|res] ([on|off|toggle])", {
  Cmds.logCommand(command->name, argc, argv);
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
    shell.println(*relay);
  }
});

ShellCommandRegister* cmdMotor = ShellCommandClass(motor, "Controls the motor - [init|speed (0-255)|accel (0-255)|fwd|rev|stop|off]", {
  Cmds.logCommand(command->name, argc, argv);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("init"))) {
    _motorController->begin();
    return;
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("fwd"))) {
    _motorController->forward();
    _logStateInLoopFor = 20;
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("rev"))) {
    _motorController->reverse();
    _logStateInLoopFor = 20;
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("stop"))) {
    _motorController->setSpeed(0);
    _logStateInLoopFor = 20;
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("off"))) {
    _motorController->emergencyStop();
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("speed"))) {
    if (argc > 2) {
      _motorController->setSpeed(strtol(argv[2], nullptr, 10));
      _logStateInLoopFor = 20;
    } else {
      _motorController->probe();
      shell.println(_motorController->speed(), DEC);
      return;
    }
  } else if (argc > 1 && !strcmp_P(argv[1], PSTR("accel"))) {
    if (argc > 2) {
      _motorController->setAcceleration(strtol(argv[2], nullptr, 10));
    } else {
      _motorController->probe();
      shell.println(_motorController->directionString());
      return;
    }
  } else {
    Cmds._serialShell.help();
    shell.help();
  }
  _motorController->probe();
  shell.println(*_motorController);
});

#define LA_DELAY 0

ShellCommandRegister* cmdLA = ShellCommandClass(la, "Controls linear actuator via relays 1 & 2 - ([extend|retract|off])", {
  Cmds.logCommand(command->name, argc, argv);

  if (argc > 1 && !strcmp_P(argv[1], PSTR("extend"))) {
    delay(LA_DELAY);
    _actuatorRelay1->turnRelayOff();
    delay(LA_DELAY);
    _actuatorRelay2->turnRelayOn();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("retract"))) {
    delay(LA_DELAY);
    _actuatorRelay2->turnRelayOff();
    delay(LA_DELAY);
    _actuatorRelay1->turnRelayOn();
  }
  if (argc > 1 && !strcmp_P(argv[1], PSTR("off"))) {
    delay(LA_DELAY);
    _actuatorRelay1->turnRelayOff();
    delay(LA_DELAY);
    _actuatorRelay2->turnRelayOff();
  }
  _actuatorRelay1->probe();
  shell.print(*_actuatorRelay1);
  shell.print(F(", "));
  _actuatorRelay2->probe();
  shell.println(*_actuatorRelay2);
  //shell.println(*_actuatorRelay2);
});

void btns() {
  Wire.begin();
  Log.noticeln(F("Wire initialzied"));

  QWIICMUX mux;

  mux.begin();

  QwiicButton button1, button2;

  button1.begin(0x6C);  // port 0x04
  button2.begin(0x6C);  // port 0x05

  mux.setPort(0x04);
  Log.traceln(F("button1 (%X:%X) = %d"), button1.getI2Caddress(), mux.getPort(), button1.isPressed());

  mux.setPort(0x05);
  Log.traceln(F("button2 (%X:%X) = %d"), button2.getI2Caddress(), mux.getPort(), button2.isPressed());
}

uint8_t fromHex(const char* s) {
  char* p;
  long n = strtoul(s, &p, 16);
  if (p == nullptr || *p != 0) {
    // not a number
    return 0;
  } else {
    return n;
  }
}

void relays() {
  Log.noticeln(F("------------- Running Relay Test "));
  Wire.begin();
  Log.noticeln(F("Wire initialzied"));

  Qwiic_Relay r1(ActuatorRelay1Addr), r2(ActuatorRelay2Addr);

  r1.begin();
  r2.begin();

  Log.traceln(F("relay 1 = %X"), r1.getState());
  Log.traceln(F("relay 2 = %X"), r2.getState());
  Log.traceln(F("  Delay is %d ms"), LA_DELAY);

  for (int i = 0; i < 5; i++) {
    // extend
    Log.traceln(F("Retract: relay 1 = %d, relay 2 = %d"), r1.getState(), r2.getState());
    r2.turnRelayOff();
    delay(LA_DELAY);
    Log.traceln(F("  relay 2 should be OFF = %d"), r2.getState());

    r1.turnRelayOn();
    delay(LA_DELAY);
    Log.traceln(F("  relay 1 should be ON = %d"), r1.getState());
    Log.traceln(F("  waiting 1000ms."));
    delay(1000);

    // stop
    Log.traceln(F("Stop: relay 1 = %d, relay 2 = %d"), r1.getState(), r2.getState());
    r1.turnRelayOff();
    delay(LA_DELAY);
    Log.traceln(F("  relay 1 should be OFF = %d"), r1.getState());

    r2.turnRelayOff();
    delay(LA_DELAY);
    Log.traceln(F("  relay 1 should be OFF = %d"), r2.getState());

    // retract
    Log.traceln(F("Extend: relay 1 = %d, relay 2 = %d"), r1.getState(), r2.getState());
    r1.turnRelayOff();
    delay(LA_DELAY);
    Log.traceln(F("  relay 1 should be OFF = %d"), r1.getState());

    r2.turnRelayOn();
    delay(LA_DELAY);
    Log.traceln(F("  relay 2 should be ON = %d"), r2.getState());
    Log.traceln(F("  waiting 2000ms."));
    delay(2000);

    // stop
    Log.traceln(F("Stop: relay 1 = %d, relay 2 = %d"), r1.getState(), r2.getState());
    r1.turnRelayOff();
    delay(LA_DELAY);
    Log.traceln(F("  relay 1 should be OFF = %d"), r1.getState());

    r2.turnRelayOff();
    delay(LA_DELAY);
    Log.traceln(F("  relay 1 should be OFF = %d"), r2.getState());
  }
}

ShellCommandRegister* cmdi2c = ShellCommandClass(test, "runs i2c tests and tools [scan|btns|progbtn <old> <new>]", {
  Cmds.logCommand(command->name, argc, argv);
  bool success = true;
  if (argc > 1 && !strcmp_P(argv[1], PSTR("relays"))) {
    relays();
  }

  if (argc > 1 && !strcmp_P(argv[1], PSTR("btns"))) {
    btns();
  }

  if (argc > 1 && !strcmp_P(argv[1], PSTR("scan"))) {
    Log.noticeln(F("------------- Scanning I2C bus for ALL devices "));
    success = Bus.scan(10);
  }

  if (argc > 1 && !strcmp_P(argv[1], PSTR("progbtn"))) {
    uint8_t addr = 0;
    uint8_t newAddr = 0;
    if (argc > 2) {
      addr = fromHex(argv[2]);
    }
    if (argc > 3) {
      newAddr = fromHex(argv[3]);
    }

    Log.noticeln(F("progbtn: addr = %X, newAddr = %X"), addr, newAddr);

    if (addr != 0 && newAddr != 0 && addr != newAddr && ((newAddr > 0x08 && newAddr < 0x77))) {
      Log.noticeln(F("Changing device %X's address to %X"), addr, newAddr);

      Wire.begin();
      Log.noticeln(F("Wire initialzied"));

      QwiicButton btn;
      if (btn.begin(addr)) {
        if (btn.getI2Caddress() != addr) {
          Log.errorln(F("ERROR: address retreived from %X does not match (%X)"), addr, btn.getI2Caddress());
          return;
        }
        if (btn.isConnected()) {
          Log.noticeln(F("  Found %X. Setting address to %X"), btn.getI2Caddress(), newAddr);

          if (!btn.setI2Caddress(newAddr)) {
            Log.errorln(F("ERROR: setI2Caddress(%X) failed"), newAddr);
            return;
          }

          delay(100);  //give the hardware time to do whatever configuration it needs to do

          if (btn.isConnected()) {
            Log.noticeln(F("  %X isconnected()"), btn.getI2Caddress());
          } else {
            Log.errorln(F("ERROR: after isConnected() failed"));
            return;
          }

          if (btn.begin(newAddr)) {
            Log.noticeln(F("  Begin worked for %X. New address is %X"), btn.getI2Caddress(), newAddr);
          }
        } else {
          Log.errorln(F("ERROR: isConnected failed"));
        }
      } else {
        Log.errorln(F("ERROR: begin(%X) failed"), addr);
      }
    } else {
      Log.errorln(F("ERROR: invalid addreses"));
    }
  }
  Log.noticeln(F("test complete - success = %T"), success);
  shell.println(success, DEC);
});

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

#if 0
  Cmds._serialShell.help();
#endif

  if (Bus.initialized() == false) {
    Log.errorln("NOTE: Bus and devices are not initialzied. Use `init all` command.");
  }

  // _actuatorRelay1->registerStateChange([]() {
  //   Log.noticeln(F("State Changed: %p"), _actuatorRelay1);
  // });

  // _esp32Button->registerStateChange([]() {
  //   Log.noticeln(F("%p"), _esp32Button);
  //   for (uint16_t i = 0; i < sizeof(Cmds._telnetShell) / sizeof(Shell); i++) {
  //     if (Cmds._shellClient[i] != nullptr) {
  //       Cmds._telnetShell[i].println(*_esp32Button);
  //     }
  //   }
  // });
  //my_devices, sizeof(my_devices) / sizeof(i2cDevice*)
  for (uint8_t i = 0; i < sizeof(my_devices) / sizeof(i2cDevice*); i++) {
    my_devices[i]->registerStateChange([](i2cDevice* device) {
      //Log.noticeln(F("%p"), device);
      for (uint16_t i = 0; i < sizeof(Cmds._telnetShell) / sizeof(Shell); i++) {
        if (Cmds._shellClient[i] != nullptr) {
          Cmds._telnetShell[i].println(*device);
        }
      }
    });
  }

  _timer = millis();
}

void loop() {
  // listen for and process REST commands from Ethernet
  // and button presses from keypad
  Cmds.handle();

  if (/*_logStateInLoopFor > 0 && Bus.initialized() &&*/ (millis() - _timer) >= DELAY_TIME) {
    _timer = millis();
    //_logStateInLoopFor--;

    for (uint8_t i = 0; i < sizeof(my_devices) / sizeof(i2cDevice*); i++) {
      if (my_devices[i]->initialized()) {
        my_devices[i]->probe();
      }
    }
  }
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