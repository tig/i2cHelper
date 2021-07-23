#pragma once

#include <Arduino.h>
#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>

/**
 * @brief defines each i2c device. Create an array of these and
 * pass to `begin()`.
 * 
 */
struct i2cDevice {
  bool isOnMux;
  bool isMux;
  uint8_t address;
  const __FlashStringHelper* name;
  bool found;
};

class i2c {
 public:
  i2c() : _devices(nullptr), _numDevices(0) {};

  i2cDevice* _devices;
  int8_t _numDevices;

  // I2C Mux from SparkFun
  // TODO: The way this is written, only one mux device is supported
  QWIICMUX _mux;
  uint8_t _muxAddr;

  /**
   * @brief sets up the bus 
   * 
   * @param devices array of device specs
   * @param num number of elements in devices
   * @return true 
   * @return false 
   */
  bool begin(i2cDevice* devices, size_t num);

  i2cDevice* findDevice(uint8_t address);
  i2cDevice* findDeviceOnMux(uint8_t port);

  // testing the i2c mux to see if we can figure out why Forward port is failing
  // to init
  bool scanI2C(int delayTime);
  bool testI2CMux(int delayTime);
};

extern i2c Bus;