#pragma once

#include <Arduino.h>
#include <assert.h>
#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>


/**
 * @brief defines each i2c device. Create an array of these and
 * pass to `begin()`.
 * 
 */
class i2cDevice : public Printable {
 public:
  i2cDevice(uint8_t address, uint8_t muxPort, const __FlashStringHelper* name, QWIICMUX* mux, bool isMux = false)
      : _address(address), _muxPort(muxPort), _name(name), _mux(mux), _isMux(isMux) {}

  /**
   * @brief sets up the device
   * 
   * @return true 
   * @return false 
   */
  virtual bool begin() { 
    Log.traceln(F("i2cDevice::begin - %S - (%X:%X) isMux = %T, mux &%X"), name(), address(), muxPort(), isMux(), mux());
    if (isMux() || muxPort() != 0xFF) assert(_mux); 
    if (mux() != nullptr) {
      if (!mux()->begin()) {
        Log.errorln(F("  ERROR: %S QWIICMUX begin failed for  (%X:%X) mux addr: %X"), name(), address(), muxPort(), mux()->getAddress());
        return false;
      }
    }
    return setPort(); 
  }

  virtual bool setPort() {
    if (isMux() || muxPort() != 0xFF) assert(_mux); 
    if (muxPort() == 0xFF) {
      //Log.traceln(F("Enabling %S on I2C address %X"), name(), address());
    } else {
      Log.traceln(F("Enabling %S on (%X:%X) - Mux address is %X"), name(), address(), muxPort(), mux()->getAddress());
      if (!mux()->setPort(muxPort())) {
        Log.errorln(F("  ERROR: %S QWIICMUX setPort failed for (%X:%X)"), name(), address(), muxPort());
        return false;
      }
    }
    return true;
  }

  uint8_t address() const { return _address; }
  void setAddress(uint8_t address) { _address = address; }
  uint8_t muxPort() const { return _muxPort; }
  void setMuxPort(uint8_t muxPort) { _muxPort = muxPort; }
  const __FlashStringHelper* name() const {
    if (_name == nullptr) {
      return F("n/a");
    } else {
      return _name;
    }
  }

  QWIICMUX* mux() { 
    if (isMux() || muxPort() != 0xFF) assert(_mux); 
    return _mux; 
  }
  bool isMux() const { return _isMux; }
  bool found() { return _found; }
  void setFound(bool found) { _found = found; }
  
  /**
   * @brief `Printable::printTo` - prints the current motor state (direction & speed)
   *
   * @param p
   * @return size_t
   */
  virtual size_t printTo(Print& p) const {
    int n = p.print(name());
    return n += p.print(F(" = "));
  }

 private:
  uint8_t _address;
  uint8_t _muxPort;  // 0xFF if not on mux
  const __FlashStringHelper* _name;
  QWIICMUX* _mux;  
  bool _isMux;
  bool _found;
};

// class i2cMux : public i2cDevice {
//   using i2cDevice::i2cDevice;

//  private:
//   QWIICMUX* _mux;
// };

class i2c {
 public:
  i2cDevice** _devices;
  int8_t _numDevices;

  /**
   * @brief sets up the bus 
   * 
   * @param devices array of device specs
   * @param num number of elements in devices
   * @return true 
   * @return false 
   */
  bool begin(i2cDevice** devices, size_t num);

  /**
   * @brief return the device specified in the i2cDevice table that has an
   * address of `address` and mux port of `port`.
   * 
   * @param address 
   * @param port 
   * @return i2cDevice* 
   */
  i2cDevice* findDevice(uint8_t address, uint8_t port);
  i2cDevice* findDeviceOnMux(uint8_t port);

  bool probeDevice(uint8_t address, uint8_t port, const char* name, int delayTime);
  
  // testing the i2c mux to see if we can figure out why Forward port is failing
  // to init
  bool scanI2C(int delayTime);
  bool testI2CMux(int delayTime);

  // =======================================================
  // Singleton support
  // https://stackoverflow.com/a/1008289/297526
 public:
  /**
   * @brief Returns the single instance object
   *
   */
  static i2c& getInstance() {
    static i2c instance;  // Guaranteed to be destroyed.
                          // Instantiated on first use.
    return instance;
  }

 private:
  // Prohibiting External Constructions
  i2c() : _devices(nullptr), _numDevices(0){};

  // C++ 11
  // =======
  // We can use the better technique of deleting the methods
  // we don't want.
 public:
  // This breaks printTo
  //i2c(i2c const&) = delete;
  void operator=(i2c const&) = delete;
  // =======================================================
};

extern i2c Bus;