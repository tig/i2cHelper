#pragma once

#include <Arduino.h>
#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>

struct i2cDevice {
  bool isOnMux;
  uint8_t address;
  const char* name;
  bool found;
};

class i2c {
 public:
  enum i2cDevices
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

  i2c(i2cDevice* devs, uint8_t num) : _devices(devs), _numDevices(num) {};

  i2cDevice* _devices;
  int8_t _numDevices;

  // I2C Mux from SparkFun
  QWIICMUX _i2cMux;

  bool begin();

  i2cDevice* find(uint8_t address);
  i2cDevice* findDeviceOnMux(uint8_t port);

  // testing the i2c mux to see if we can figure out why Forward port is failing
  // to init
  bool scanI2C(int delayTime);
  bool testI2CMux(int delayTime);
};

extern i2c Bus;