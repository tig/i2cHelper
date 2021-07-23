#pragma once
#include <Arduino.h>
#include <ArduinoLog.h>

#ifdef ARDUINO_SAMD_MKRZERO
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <SparkFun_Qwiic_Relay.h>
#endif

#include "i2c.h"

class Relay : public Qwiic_Relay, public Printable {
 public:
  Relay(uint8_t address) : Qwiic_Relay(address), _address(address), _cachedOn(0){};

  bool begin() {
    bool success = true;
    Log.noticeln(F("Enabling %S on I2C address %X"), Bus.findDevice(_address)->name, _address);
#ifdef ARDUINO_SAMD_MKRZERO
    if (Qwiic_Relay::begin()) {
      turnRelayOff();
      getState();
      Log.noticeln(F("%S = %p"), Bus.findDevice(_address)->name, this);
    } else {
        Log.errorln(F("  ERRROR: Relay setup failed"));
        success = false;
    }
#else
    pinMode(_address, OUTPUT);
#endif
    return success;
  };

  void turnRelayOn() {
    _cachedOn = 1;
#ifdef ARDUINO_SAMD_MKRZERO
    Qwiic_Relay::turnRelayOn();
#else
    digitalWrite(_address, _cachedOn);
#endif
  };

  void turnRelayOff() {
    _cachedOn = 0;
#ifdef ARDUINO_SAMD_MKRZERO
    Qwiic_Relay::turnRelayOff();
#else
    digitalWrite(_address, _cachedOn);
#endif
  };

  void toggleRelay() {
    _cachedOn = _cachedOn ? 0 : 1;
#ifdef ARDUINO_SAMD_MKRZERO
    Qwiic_Relay::toggleRelay();
#else
    digitalWrite(_address, _cachedOn);
#endif
  };

  uint8_t getState() {
#ifdef ARDUINO_SAMD_MKRZERO
    _cachedOn = Qwiic_Relay::getState();
#else
#endif
    return _cachedOn;
  };

  void probe() {
      getState();
  }

  /**
   * @brief `Printable::printTo` - prints the current motor state (direction & speed)
   *
   * @param p
   * @return size_t
   */
  size_t printTo(Print& p) const {
    return p.print(_cachedOn ? F("on") : F("off"));
  };

 private:
  uint8_t _address;
  uint8_t _cachedOn;
};