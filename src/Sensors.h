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

  const int BUTTON_PIN = 0;

  virtual bool begin() override {
    if (address() == 0xFF) {
      pinMode(BUTTON_PIN, INPUT_PULLUP);
      bool b = digitalRead(BUTTON_PIN) == LOW;
      setContact(b);
    }
    return i2cDevice::begin();
  }

  virtual bool isContacted() {
    setPort();
    //Log.traceln(F("ContactSensor::isContacted = %T"), _contact);

    if (address() == 0xFF) {
      bool b = digitalRead(BUTTON_PIN) == LOW;
      setContact(b);
    }
    return _contact;
  }

  virtual void probe() override {
    if (address() == 0xFF) {
      isContacted();
    }
  }

  virtual void setContact(bool contact) {
    setPort();
    if (_contact != contact) {
      _contact = contact;
      notify();
    }
  }

  virtual size_t printTo(Print& p) const override {
    int n = i2cDevice::printTo(p);
    return n += p.print(_contact ? F("closed") : F("opened"));
  }

 private:
  bool _contact = false;
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

      setContact(_button.isPressed());
      //Log.trace(F("  isContacted = %T, FirmwareVersion = %d, I2Caddress = %X"), _contact, _button.getFirmwareVersion(), _button.getI2Caddress());
      if (!success) {
        Log.errorln(F("  ERROR: %S setup failed. isConnected = %T, DeviceID() = %X"), name(), _button.isConnected(), _button.deviceID());
      }
      Log.trace(F(" [%p]"), this);
    }
    return success;
  }

  virtual void probe() override {
    isContacted();
  }

  virtual bool isContacted() override {
    // Call base to setPort
    bool contact = ContactSensor::isContacted();
    contact = _button.isPressed();
    ContactSensor::setContact(contact);
    return contact;
  }

  void setContact(bool contact) override {
    ContactSensor::setContact(contact);
    digitalWrite(address(), contact);
  }

 private:
  QwiicButton _button;
};

/**
 * @brief Base class for distance sensors
 * 
 */
class DistanceSensor : public i2cDevice {
 public:
  using i2cDevice::i2cDevice;

  virtual uint16_t distance() {
    return _cachedDistance;
  }

  virtual void setDistance(uint16_t distance) {
    if (_cachedDistance != distance) {
      _cachedDistance = distance;
      notify();
    }
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
  const uint8_t SENSOR_MIN_MOVEMENT = 5;     // 5 mm

  using DistanceSensor::DistanceSensor;

  virtual bool begin() override {
    // Call base which sets up mux if needed
    i2cDevice::begin();
    if (initialized()) {
      if (_sensor != nullptr) {
        delete _sensor;
      }
      _sensor = new SFEVL53L1X(Wire);

      uint16_t result;
      if ((result = (uint16_t)_sensor->begin()) != 0)  // Begin returns 0 on a good init
      {
        Log.errorln(F("\n    ERROR: Distance Sensor failed to begin. Result: %X. Please check wiring."), result);
        Log.traceln(F("          checkID: %d"), (uint16_t)_sensor->checkID());
        VL53L1X_Version_t ver = _sensor->getSoftwareVersion();
        Log.traceln(F("  softwareVersion: %d.%d.%d.%d"), ver.major, ver.minor, ver.revision, ver.build);
        Log.traceln(F("       I2CAddress: %X"), (uint16_t)_sensor->getI2CAddress());
        Log.traceln(F("      getSensorID: %X"), (uint16_t)_sensor->getSensorID());
        setInitialized(false);
      }
    } else {
      Log.errorln(F("    ERROR: i2cDevice::begin() failed"));
    }

    if (initialized() == true) {
      // Everything should be working. Set initial parameters and get
      // current distance.
      _sensor->setIntermeasurementPeriod(SENSOR_PERIOD);
      _sensor->setDistanceModeLong();
      _sensor->setTimingBudgetInMs(SENSOR_TIMING_BUDGET);

      // Set the initial distance
      _sensor->startOneshotRanging();
      for (uint16_t msecs = 0; !_sensor->checkForDataReady() && msecs <= SENSOR_WAIT_PERIOD; msecs++) {
        delay(1);
        if (msecs == SENSOR_WAIT_PERIOD) {
          Log.errorln(F("    ERROR: Distance Sensor failed to make data ready."));
          setInitialized(false);
          break;
        }
      }
      setDistance(_sensor->getDistance());
      Log.trace(F(" [%p]"), this);
    }

    return initialized();
  };

  virtual void setDistance(uint16_t distance) {
    uint16_t delta = 0;
    if (DistanceSensor::distance() > distance) delta = DistanceSensor::distance() - distance;
    if (DistanceSensor::distance() < distance) delta = distance - DistanceSensor::distance();
    if (delta > SENSOR_MIN_MOVEMENT) {
      DistanceSensor::setDistance(distance);
    }
  }

  virtual void probe() override {
    if (setPort()) {
      _sensor->startOneshotRanging();
      for (uint16_t msecs = 0; !_sensor->checkForDataReady() && msecs <= SENSOR_WAIT_PERIOD; msecs++) {
        delay(1);
        if (msecs == SENSOR_WAIT_PERIOD) {
          Log.errorln(F("ERROR: VL53L1XDistanceSensor::probe() - %S failed to make data ready."), name());
          return;
        }
      }
      setDistance(_sensor->getDistance());
    }
  }

  SFEVL53L1X* sensor() const { return _sensor; }

 private:
  SFEVL53L1X* _sensor = nullptr;
};