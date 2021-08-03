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

class EventHandler {
    public:
        void addHandler(std::function<void(int)> callback) {
        }
};

/**
 * @brief defines each i2c device. Create an array of these and
 * pass to `begin()`.
 * 
 */
class i2cDevice : public Printable {
 public:
  i2cDevice(uint8_t address, uint8_t muxPort, const __FlashStringHelper* name, QWIICMUX* mux, bool isMux = false)
      : _address(address),
        _muxPort(muxPort),
        _name(name),
        _mux(mux),
        _isMux(isMux),
        //_callbacks(nullptr),
        _initialized(false) {}

  /**
   * @brief sets up the device
   * 
   * @return true 
   * @return false 
   */
  virtual bool begin();
  virtual void probe();
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
  virtual size_t printTo(Print& p) const;

  template<class T> void registerStateChange(T* const object, void(T::* const func)(i2cDevice* device))
  {
    using namespace std::placeholders; 
    _callbacks.emplace_back(std::bind(func, object, _1));
  }

  void registerStateChange(void(* const func)(i2cDevice* device)) 
  {
    _callbacks.emplace_back(func);
  }

  void notify() 
  {
    for (const auto& cb : _callbacks)
      cb(this);
  }

 private:
  uint8_t _address;
  uint8_t _muxPort;  // 0xFF if not on mux
  const __FlashStringHelper* _name;
  QWIICMUX* _mux;
  bool _isMux;
  bool _found;
  std::vector<std::function<void(i2cDevice* device)>> _callbacks;
  bool _initialized;
};

class i2c {
 public:
  //i2cDevice** _devices;
  //int8_t _numDevices;

  std::vector<i2cDevice*> _devices;

  /**
   * @brief sets up the bus 
   * 
   * @param devices array of device specs
   * @param num number of elements in devices
   * @return true 
   * @return false 
   */
  bool begin(i2cDevice** devices, size_t num);

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

  bool probeDevice(uint8_t address, uint8_t port, int delayTime);

  bool probeAll(int delayTime);
  bool scan(int delayTime);

  bool initialized() { return _initialized; }

 private:
  bool _initialized;

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
  i2c() : _devices(), _initialized(false){};

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