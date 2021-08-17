#pragma once

#include <Arduino.h>
#include <assert.h>
#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>

#include <functional>
#include <vector>
using namespace std;

class i2cDevice;

/**
 * @brief defines each i2c device. Create an array of these and
 * pass to `begin()`.
 * 
 */
class i2cDevice : public Printable {
 public:
  i2cDevice(uint8_t address, uint8_t muxPort, uint8_t muxAddress, const __FlashStringHelper* name, bool isMux = false)
      : _address(address),
        _muxPort(muxPort),
        _muxAddress(muxAddress),
        _name(name),
        _isMux(isMux),
        _mux(nullptr),
        _initialized(false),
        _stateChanged(true) {
          if (isMux || muxAddress != 0xFF) {
            _mux = new QWIICMUX();
          }
        }

  /**
   * @brief sets up the device
   * 
   * @return true 
   * @return false 
   */
  virtual bool begin();

  /**
   * @brief updates cached members by reading from physical device.
   * 
   * @param forceNotify if true stateChange notification callbacks will be called. 
   * if false, callbacks will only be called if the state has changed since last notify
   * 
   */
  virtual void probe(bool forceNotify = false);

  virtual bool setPort();

  uint8_t address() const;
  void setAddress(uint8_t address);
  uint8_t muxPort() const;
  void setMuxPort(uint8_t muxPort);
  const __FlashStringHelper* name() const;
  QWIICMUX* mux();
  bool isMux() const;
  bool found();
  void setFound(bool found);
  bool initialized();
  void setInitialized(bool init);
  void setStateChanged() { _stateChanged = true; }
  bool stateChanged() { return _stateChanged; }
  virtual size_t printTo(Print& p) const;

  // template<class T> void registerStateChange(T* const object, void(T::* const func)(i2cDevice* device))
  // {
  //   using namespace std::placeholders; 
  //   _callbacks.emplace_back(std::bind(func, object, _1));
  // }

  // void registerStateChange(void(* const func)(i2cDevice* device)) 
  // {
  //   _callbacks.emplace_back(func);
  // }

  // void notify() 
  // {
  //   _stateChanged = false;
  //   for (const auto& cb : _callbacks)
  //     cb(this);
  // }


  template<class T> void registerStateChange(T* const object, std::function<void(i2cDevice* device)> func)
  {
    using namespace std::placeholders; 
    _callbacks.emplace_back(std::bind(func, object, _1));
  }

  void registerStateChange(std::function<void(i2cDevice* device)> func) 
  {
    _callbacks.emplace_back(func);
  }

  void notify() 
  {
    _stateChanged = false;
    for (const auto& cb : _callbacks) {
      //Log.traceln(F("-%S- callback"), name());
      cb(this);
    }      
  }

 private:
  std::vector<std::function<void(i2cDevice* device)>> _callbacks;

 private:
  uint8_t _address;
  uint8_t _muxPort;  // 0xFF if not on mux
  uint8_t _muxAddress; // 0xFF if not on mux
  const __FlashStringHelper* _name;
  bool _isMux;
  QWIICMUX* _mux;
  bool _found;
  bool _initialized;
  bool _stateChanged = true;
};

class i2c : public Printable {
 public:
  std::vector<i2cDevice*> _devices;

  /**
   * @brief sets up the bus 
   * 
   * @param devices array of device specs
   * @param num number of elements in devices
   * @param probeInterval number of milliseconds between each device probe; 0 to probe on every call to proveDevices
   * @return true 
   * @return false 
   */
  bool begin(i2cDevice** devices, size_t num, uint16_t probeInterval = 0);

  void add(i2cDevice* device);

  bool enableMuxPort(byte mux, byte portNumber);

  //Disables a specific port number
  bool disableMuxPort(byte mux, byte portNumber);

  /**
   * @brief return the device specified in the i2cDevice table that has an
   * address of `address` and mux port of `port`.
   * 
   * @param address 
   * @param port 
   * @return i2cDevice* 
   */
  i2cDevice* findDevice(uint8_t address, uint8_t port);

  bool testDevice(uint8_t address, uint8_t port, int delayTime);
  bool testAll(int delayTime);
  bool scan(int delayTime);

  bool initialized() { return _initialized; }
  void probeDevices(bool forceNotify = false);
  virtual size_t printTo(Print& p) const;

  unsigned long probeInterval() { return _probeInterval; }
  void setProbeInterval(unsigned long interval) { _probeInterval = interval; }

 private:


  bool _initialized;
  unsigned long _probeTimer;
  unsigned long _probeInterval;

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
  i2c() : _devices(), _initialized(false), _probeTimer(0), _probeInterval(0) {};

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