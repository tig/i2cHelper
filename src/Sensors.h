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
class ContactSensor : public i2cDevice {
 public:
  using i2cDevice::i2cDevice;

  virtual bool isContacted() {
    setPort();
    return _contact;
  };

  // for testing only
  virtual void setContact(bool contact) {
    setPort();
    _contact = contact;
  };

  virtual size_t printTo(Print& p) const override {
    int n = i2cDevice::printTo(p);
    return n += p.print(_contact ? F("closed") : F("opened"));
  };

 protected:
  bool _contact;
};

/**
 * @brief Wraps a SparkFun Qwiic Button
 * 
 */
class QwiicContactSensor : public ContactSensor {
 public:
  using ContactSensor::ContactSensor;

  virtual bool begin() override {
    // Call base which sets up mux if needed
    bool success = i2cDevice::begin();
    if (success) {
      success = _button.begin(address());

      _contact = _button.isPressed();
      Log.traceln(F("  isContacted = %T, FirmwareVersion = %d, I2Caddress = %X"), _contact, _button.getFirmwareVersion(), _button.getI2Caddress());
      if (!success) {
        Log.errorln(F("  ERROR: %S setup failed. isConnected = %T, DeviceID() = %X"), name(), _button.isConnected(), _button.deviceID());
      }
      Log.noticeln(F("%p"), this);
    }
    return success;
  };

  virtual bool isContacted() override {
    ContactSensor::isContacted();
    return _contact = _button.isPressed();
  };

  void setContact(bool contact) override {
    ContactSensor::setContact(contact);
    digitalWrite(address(), contact);
  };

 private:
  QwiicButton _button;
};

class DistanceSensor : public i2cDevice {
 public:
  using i2cDevice::i2cDevice;

  virtual uint16_t distance() {
    return _cachedDistance;
  }

  // for simulation
  virtual void setDistance(uint16_t distance) {
    _cachedDistance = distance;
  }

  virtual size_t printTo(Print& p) const override {
    int n = i2cDevice::printTo(p);
    return n += p.print(_cachedDistance, DEC);
  };

 private:
  uint16_t _cachedDistance = 0;
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
class VL53L1XDistanceSensor : public DistanceSensor {
 public:
  const uint8_t SENSOR_PERIOD = 180;         // How fast to sample - min is 100, but that can result in bad readings some times
  const uint8_t SENSOR_TIMING_BUDGET = 100;  // Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
  const uint16_t SENSOR_WAIT_PERIOD = 1000;  // Num msec to wait until giving up on checkforDataReady

  using DistanceSensor::DistanceSensor;

  virtual bool begin() override {
    // Call base which sets up mux if needed
    bool success = i2cDevice::begin();
    if (success) {
      if (_sensor != nullptr) {
        delete _sensor;
      }
      _sensor = new SFEVL53L1X(Wire);

      uint16_t result;
      if ((result = (uint16_t)_sensor->begin()) != 0)  // Begin returns 0 on a good init
      {
        for (uint16_t msecs = 0; !_sensor->checkForDataReady() && msecs <= SENSOR_WAIT_PERIOD; msecs++) {
          delay(1);
          if (msecs == SENSOR_WAIT_PERIOD) {
            Log.errorln(F("    ERROR: Sensor failed to make data ready."));
            success = false;
            break;
          }
        }
        Log.errorln(F("    ERROR: Sensor failed to begin. Result: %d. Please check wiring."), result);
        Log.traceln(F("          checkID: %d"), (uint16_t)_sensor->checkID());
        VL53L1X_Version_t ver = _sensor->getSoftwareVersion();
        Log.traceln(F("  softwareVersion: %d.%d.%d.%d"), ver.major, ver.minor, ver.revision, ver.build);
        Log.traceln(F("       I2CAddress: %X"), (uint16_t)_sensor->getI2CAddress());
        Log.traceln(F("      getSensorID: %X"), (uint16_t)_sensor->getSensorID());

        success = false;
      } else {
        Log.traceln(F("       I2CAddress: %X"), (uint16_t)_sensor->getI2CAddress());
        _sensor->setIntermeasurementPeriod(SENSOR_PERIOD);
        _sensor->setDistanceModeLong();
        _sensor->setTimingBudgetInMs(SENSOR_TIMING_BUDGET);

        // Set the initial distance
        _sensor->startOneshotRanging();
        for (uint16_t msecs = 0; !_sensor->checkForDataReady() && msecs <= SENSOR_WAIT_PERIOD; msecs++) {
          delay(1);
          if (msecs == SENSOR_WAIT_PERIOD) {
            Log.errorln(F("    ERROR: Sensor failed to make data ready."));
            success = false;
            break;
          }
        }
        setDistance(_sensor->getDistance());
        Log.noticeln(F("%p"), this);
      }
    } else {
      Log.errorln(F("    ERROR: i2cDevice::begin() failed"));
    }
    return success;
  };

  virtual uint16_t distance() override {
    if (setPort()) {
      _sensor->startOneshotRanging();
      for (uint16_t msecs = 0; !_sensor->checkForDataReady() && msecs <= SENSOR_WAIT_PERIOD; msecs++) {
        delay(1);
        if (msecs == SENSOR_WAIT_PERIOD) {
          Log.errorln(F("ERROR: getDistance() - Sensor failed to make data ready."));
          return 0;
          break;
        }
      }
      setDistance(_sensor->getDistance());
      return distance();
    }
    return 0;
  }

  SFEVL53L1X* sensor() const { return _sensor; }

 private:
  SFEVL53L1X* _sensor = nullptr;
};