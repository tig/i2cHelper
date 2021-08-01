#pragma once

#include <Arduino.h>
#include <assert.h>
#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>

class i2cDevice;

// /**
//  * \brief Function that gets called when the state of a i2cDevice changes
//  *
//  * \param device The i2cDevice
//  * \param propertyName The name of the property that changed
//  * \param newValue The new value
//  */
// typedef void (*StateChanged)(i2cDevice &device, const __FlashStringHelper* propertyName, void* newValue);


// template< typename Functor, typename ParamType >
// void DoSmth(Functor isGood, const ParamType param){
//    //...
//    if(isGood(param, some_int_calculated_here)) doSmthElse();
// }

/**
 * @brief defines each i2c device. Create an array of these and
 * pass to `begin()`.
 * 
 */
class i2cDevice : public Printable {
 public:
  i2cDevice(uint8_t address, uint8_t muxPort, const __FlashStringHelper* name, QWIICMUX* mux, bool isMux = false)
      : _address(address), _muxPort(muxPort), _name(name), _mux(mux), _isMux(isMux), _initialized(false) {}

  /**
   * @brief sets up the device
   * 
   * @return true 
   * @return false 
   */
  virtual bool begin();
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

 private:
  uint8_t _address;
  uint8_t _muxPort;  // 0xFF if not on mux
  const __FlashStringHelper* _name;
  QWIICMUX* _mux;
  bool _isMux;
  bool _found;
  bool _initialized;
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
  i2c() : _devices(nullptr), _numDevices(0), _initialized(false){};

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