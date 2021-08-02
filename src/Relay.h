#pragma once
#include <Arduino.h>
#include <ArduinoLog.h>

#ifndef SIMULATION
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <SparkFun_Qwiic_Relay.h>
#endif

#include "i2c.h"

class Relay : public i2cDevice {
 public:
  using i2cDevice::i2cDevice;

  virtual void turnRelayOn() {
    _state = 1;
#ifndef SIMULATION
    digitalWrite(address(), _state);
#endif
    notify();
  };

  virtual void turnRelayOff() {
    _state = 0;
#ifndef SIMULATION
    digitalWrite(address(), _state);
#endif
    notify();
  };

  virtual void toggleRelay() {
    _state = _state ? 0 : 1;
#ifndef SIMULATION
    digitalWrite(address(), _state);
#endif
    notify();
  };

  virtual void setState(uint8_t state) {
    //Log.traceln(F("Relay::state() setting to %d"), state);
    if (_state != state) {
      _state = state;
      notify();
    }
  };

  virtual uint8_t state() {
    return _state;
  };

  virtual void probe() {
    state();
  }

  virtual size_t printTo(Print& p) const override {
    int n = i2cDevice::printTo(p);
    if (_state == 0xFF) {
      return n += p.print(F("err"));
    }
    return n += p.print(_state ? F("on") : F("off"));
  };

 private:
  uint8_t _state;
};

class QwiicRelay : public Relay {
 public:
  using Relay::Relay;

  virtual bool begin() override {
    // Call base which sets up mux if needed
    bool success = i2cDevice::begin();
    if (success) {
      if (_relay != nullptr) {
        delete _relay;
      }
      _relay = new Qwiic_Relay(address());
      success = _relay->begin();
      if (!success) {
        Log.errorln(F("  ERROR: %S setup failed"), name());
      }
      turnRelayOff();
      setState(_relay->getState());

      Log.trace(F(" [%p]"), this);
    }
    return success;
  };

  virtual void turnRelayOn() override {
    //Relay::turnRelayOn();
    _relay->turnRelayOn();
    //state();
  };

  virtual void turnRelayOff() override {
    //Relay::turnRelayOff();
    _relay->turnRelayOff();
    //state();
  };

  virtual void toggleRelay() override {
    //Relay::toggleRelay();
    _relay->toggleRelay();
    state();
  };

  virtual uint8_t state() override {
    uint8_t result = _relay->getState();
    //Log.traceln(F("QwiicRelay::state() relay says %d"), result);
    Relay::setState(result);
    return result;
  };

  virtual void probe() override {
    state();
  }

 private:
  Qwiic_Relay* _relay = nullptr;
};