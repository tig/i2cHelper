#pragma once

#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>
#include <SparkFun_VL53L1X.h>
#include <SparkFun_Qwiic_Button.h>
#include <SparkFun_Qwiic_Relay.h>

#include "i2c.h"

/**
 * @brief base-class for Contact Sensor. Used when simulating real hardware.
 * 
 */
class ContactSensor : public Printable {
 public:
  ContactSensor(uint8_t address) : _address(address) {}
  ContactSensor() : ContactSensor(0) {}

  virtual bool begin() { return true; };

  virtual bool isContacted() {
    return _contact;
  };

  // for testing only
  virtual void setContact(bool contact) {
    _contact = contact;
  };

  void address(uint8_t address) { _address = address; }
  uint8_t getAddress() { return _address; };

  /**
   * @brief `Printable::printTo` - prints the current motor state (direction & speed)
   *
   * @param p
   * @return size_t
   */
  size_t printTo(Print& p) const {
    return p.print(_contact ? F("closed") : F("opened"));
  };

 protected:
  bool _contact;

 private:
  uint8_t _address;
};

/**
 * @brief wraps a SparkFun Qwiic Button
 * 
 */
class QwiicContactSensor : public ContactSensor {
 public:
  using ContactSensor::ContactSensor;
  bool begin() override {
    Log.noticeln(F("Enabling %S on I2C address %X"), Bus.findDevice(getAddress())->name, getAddress());    
    bool result = _button.begin(getAddress());
    _contact = _button.isPressed();
    Log.traceln(F("  isContacted = %T, FirmwareVersion = %d, I2Caddress = %X"), _contact, _button.getFirmwareVersion(), _button.getI2Caddress());
    if (!result) {
      Log.errorln(F("  ERROR: Sensor setup failed. isConnected = %T, DeviceID() = %X"), _button.isConnected(), _button.deviceID());
    }
    return result;
  };

  virtual bool isContacted() override {
    return _contact = _button.isPressed();
  };

  void setContact(bool contact) override {
    ContactSensor::setContact(contact);
    digitalWrite(getAddress(), isContacted());
  };

 private:
  QwiicButton _button;
};

/**
 * @brief Wraps a parkFun Qwiic VL53L1X distance sensor, providing diagnostics and 
 * other helpful stuff.
 * 
 * See https://learn.sparkfun.com/tutorials/qwiic-distance-sensor-vl53l1x-hookup-guide
 * SparkFun Library repo: https://github.com/sparkfun/SparkFun_VL53L1X_Arduino_Library
 *  * 
 * NOTE: We currently use a forked version of the VL53L1X library 
 * to get One-Shot Ranging trigger support:
 * https://github.com/josephduchesne/SparkFun_VL53L1X_Arduino_Library
 * 
 * 
 */
class DistanceSensor : public Printable {
 public:
  const uint8_t SENSOR_PERIOD = 180;         // How fast to sample - min is 100, but that can result in bad readings some times
  const uint8_t SENSOR_TIMING_BUDGET = 100;  // Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
  const uint16_t SENSOR_WAIT_PERIOD = 5000;  // Num msec to wait until giving up on checkforDataReady

  DistanceSensor(uint8_t muxPort) : _muxPort(muxPort){};

  SFEVL53L1X* _sensor = nullptr;
  uint8_t _muxPort;

  /**
   * @brief initalize the sensor
   * 
   * @param quiet disables logging
   * @return true if init worked
   * @return false if init didn't work
   */
  bool begin(bool quiet = false) {
    bool success = true;

    if (_sensor != nullptr) {
      delete _sensor;
    }
    _sensor = new SFEVL53L1X(Wire);

    if (!quiet) {
      Log.noticeln(F("Enabling %S on mux (%X) port %X)"), Bus.findDeviceOnMux(_muxPort)->name, Bus._muxAddr, _muxPort);
      Log.traceln(F("  Previous mux port: %X"), Bus._mux.getPort());
    }

    if (!Bus._mux.setPort(_muxPort)) {
      if (!quiet) {
        Log.errorln(F("    ERROR: i2cMux couldn't set the port."));
      }
      success = false;
    }

    uint16_t result;
    if ((result = (uint16_t)_sensor->begin()) != 0)  // Begin returns 0 on a good init
    {
      if (quiet) {
        return false;
      }

      for (uint16_t msecs = 0; !_sensor->checkForDataReady() && msecs <= 500; msecs++) {
        delay(1);
        if (msecs == SENSOR_WAIT_PERIOD) {
          Log.errorln(F("    ERROR: Sensor failed to make data ready."));
          success = false;
          break;
        }
      }
      Log.errorln(F("    ERROR: Sensor failed to begin. Result: %d. Please check wiring."), result);
      Log.traceln(F("          checkID: %d"), (uint16_t)_sensor->checkID());
      // VL53L1X_Version_t ver;
      // _sensor->_device->VL53L1X_GetSWVersion(&ver);
      VL53L1X_Version_t ver = _sensor->getSoftwareVersion();
      Log.traceln(F("  softwareVersion: %d.%d.%d.%d"), ver.major, ver.minor, ver.revision, ver.build);
      Log.traceln(F("       I2CAddress: %X"), (uint16_t)_sensor->getI2CAddress());
      Log.traceln(F("      getSensorID: %X"), (uint16_t)_sensor->getSensorID());

      success = false;
    } else {
      _sensor->setIntermeasurementPeriod(SENSOR_PERIOD);
      _sensor->setDistanceModeLong();
      _sensor->setTimingBudgetInMs(SENSOR_TIMING_BUDGET);

      // Set the initial distance
      _sensor->startOneshotRanging();
      for (uint16_t msecs = 0; !_sensor->checkForDataReady() && msecs <= SENSOR_WAIT_PERIOD; msecs++) {
        delay(1);
        if (msecs == SENSOR_WAIT_PERIOD) {
          if (!quiet) {
            Log.errorln(F("    ERROR: Sensor failed to make data ready."));
          }
          success = false;
          break;
        }
      }
      _cachedDistance = _sensor->getDistance();
      if (!quiet) {
        Log.noticeln(F("%p"), this);
      }
    }
    return success;
  }

  int _cachedDistance = 0;

  int getDistance() {
    if (!Bus._mux.setPort(_muxPort)) {
      Log.errorln(F("ERROR: getDistance() - i2cMux couldn't set the port."));
      return 0;
    }
    _sensor->startOneshotRanging();
    for (uint16_t msecs = 0; !_sensor->checkForDataReady() && msecs <= SENSOR_WAIT_PERIOD; msecs++) {
      delay(1);
      if (msecs == SENSOR_WAIT_PERIOD) {
        Log.errorln(F("ERROR: getDistance() - Sensor failed to make data ready."));
        return 0;
        break;
      }
    }
    return _cachedDistance = _sensor->getDistance();
  }

  /**
   * @brief `Printable::printTo` - prints the current State & Trigger
   * 'State(Trigger)'
   *
   * @param p
   * @return size_t
   */
  size_t printTo(Print& p) const {
    int n = p.print(Bus.findDeviceOnMux(_muxPort)->name);
    n += p.print(F(" = "));
    n += p.print(_cachedDistance, DEC);
    return n + p.print(F(" mm"));
  };
};