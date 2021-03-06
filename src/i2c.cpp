#include <Arduino.h>
#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>

#include "i2c.h"
#include "Sensors.h"

static const char errorStr[] PROGMEM = "n/a";

// ---- i2cDevice class

/**
   * @brief sets up the device
   * 
   * @return true 
   * @return false 
   */
bool i2cDevice::begin() {
  //Log.traceln(F("i2cDevice::begin - %S - (%X:%X) isMux = %T, mux &%X"), name(), address(), muxPort(), isMux(), mux());

  if (isMux() || muxPort() != 0xFF) assert(mux());

  if (mux() != nullptr) {
    // BUGBUG: should really pass address() to begin()
    if (!mux()->begin()) {
      Log.errorln(F("  ERROR: %S QWIICMUX begin failed for (%X:%X) mux addr: %X"), name(), address(), muxPort(), mux()->getAddress());
      return false;
    }
    //Log.trace(F("back from QWIICMUX::begin."));
  }
  _initialized = true;
  _initialized = setPort();
  //Log.trace(F("(begin returning %)"), _initialized);
  return _initialized;
}

/**
   * @brief updates cached members by reading from physical device.
   * 
   * @param forceNotify if true stateChange notification callbacks will be called. 
   * if false, callbacks will only be called if the state has changed since last notify
   * 
   */
void i2cDevice::probe(bool forceNotify) {
  if (forceNotify || stateChanged()) {
    notify();
  }
}

bool i2cDevice::setPort() {
  //Log.trace(F("(setPort() %S on (%X:%X))"), name(), address(), muxPort());
  if (!initialized()) {
    Log.traceln(F("  ERROR: %S on (%X:%X) i2cDevice::setPort() when not initialized."), name(), address(), muxPort());
    return false;
  }
  if (isMux() || muxPort() != 0xFF) assert(_mux);
  if (muxPort() == 0xFF) {
    //Log.trace(F("(SetPort for %S on I2C address %X - Not needed; no mux)"), name(), address());
  } else {
    //Log.trace(F("(SetPort for %S on (%X:%X) - Mux address is %X)"), name(), address(), muxPort(), mux()->getAddress());
    if (!mux()->setPort(muxPort())) {
      Log.errorln(F("  ERROR: %S QWIICMUX setPort failed for (%X:%X)"), name(), address(), muxPort());
      return false;
    } else {
      //uint8_t b = mux()->getPort();
      //Log.trace(F("(GetPort for %S says %X)"), name(), b);
    }
  }
  return true;
}

uint8_t i2cDevice::address() const {
  return _address;
}

void i2cDevice::setAddress(uint8_t address) { _address = address; }
uint8_t i2cDevice::muxPort() const { return _muxPort; }
void i2cDevice::setMuxPort(uint8_t muxPort) { _muxPort = muxPort; }
const __FlashStringHelper* i2cDevice::name() const {
  if (_name == nullptr) {
    return F("n/a");
  } else {
    return _name;
  }
}

QWIICMUX* i2cDevice::mux() {
  if (isMux() || muxPort() != 0xFF) assert(_mux);
  return _mux;
}
bool i2cDevice::isMux() const { return _isMux; }
bool i2cDevice::found() { return _found; }
void i2cDevice::setFound(bool found) { _found = found; }
bool i2cDevice::initialized() { return _initialized; }
void i2cDevice::setInitialized(bool init) { _initialized = init; }
/**
   * @brief `Printable::printTo` - prints the current motor state (direction & speed)
   *
   * @param p
   * @return size_t
   */
size_t i2cDevice::printTo(Print& p) const {
  int n = p.print(name());
  return n += p.print(F(" = "));
}

// ---- i2c class

/**
 * @brief Global instance of the I2C bus.
 * 
 */
i2c Bus = i2c::getInstance();

/**
 * @brief Call to initalize the I2C bus. Can (?) be called mulitple times to re-initialize.
 * 
 * @return true 
 * @return false 
 */
bool i2c::begin(i2cDevice** devices, size_t num, uint16_t probeInterval) {
  bool success = true;
  _initialized = false;
  _probeTimer = millis();
  _probeInterval = probeInterval;

  for (size_t i = 0; i < num; i++) {
    _devices.emplace_back(devices[i]);
  }

#if defined(ARDUINO_ARCH_ESP32)
  if (!Wire.begin()) {
    Log.errorln(F("ERROR: Wire.begin failed"));
  } else {
    Log.traceln(F("Wire initialzied"));
  }
#else
  Wire.begin();
  Log.traceln(F("Wire initialzied"));
#endif

  Log.noticeln(F("Looking for %d device..."), _devices.size());
  for (uint8_t i = 0; i < _devices.size(); i++) {
    Log.notice(F("Address %X - %S --"), _devices[i]->address(), _devices[i]->name());
    _devices[i]->setFound(false);
    if (_devices[i]->isMux()) {
      Log.notice(F(" is a mux --"));
    }
    if (_devices[i]->muxPort() != 0xFF) {
      Log.notice(F(" on mux %X:%X --"), _devices[i]->muxPort(), _devices[i]->mux()->getAddress());
    }

    if (_devices[i]->begin()) {
      Log.noticeln(F(" OK!"));
    } else {
      Log.errorln(F("\n  ERROR: %S at %X was not found. Check wiring."), _devices[i]->name(), _devices[i]->address());
      success = false;
    }
  }
  return _initialized = success;
}

void i2c::add(i2cDevice* device) {
  _devices.emplace_back(device);
}

/**
 * @brief Given an I2C address, return the i2cDevice that matches from the array of devices
 * 
 * @param address - I2C address
 * @return i2cDevice* 
 */
i2cDevice* i2c::findDevice(uint8_t address, uint8_t port) {
  if (!initialized()) return nullptr;
  for (uint8_t i = 0; i < _devices.size(); i++) {
    if (_devices[i]->address() == address && _devices[i]->muxPort() == port) {
      return _devices[i];
    }
  }
  return nullptr;
}

/**
 * @brief scan the i2c bus for all devices. Uses the declare_devices array of
 * i2cDevice structs to pretty print results. 
 * 
 * @param delayTime 
 * @return true 
 * @return false 
 */
bool i2c::testAll(int delayTime) {
  if (!initialized()) {
    Log.traceln(F("ERROR: i2c::scanI2C when not initialized."));
    return false;
  }

  bool success = true;

  // Only scan for muxes that we know are there to save time
  for (uint8_t i = 0; i < _devices.size(); i++) {
    _devices[i]->setFound(false);
  }

  // Log.notice(F("I2C devices found: "));
  for (uint8_t i = 0; i < _devices.size(); i++) {
    if (!_devices[i]->isMux() && _devices[i]->mux() != nullptr) {
      //Log.trace(F("!isMux - setting port %S"), _devices[i]->name());
      if (!_devices[i]->setPort()) {
        Log.errorln(F("  ERROR: Mux at %X failed setPort"), _devices[i]->address());
      }
    }

    bool found = testDevice(_devices[i]->address(), _devices[i]->muxPort(), delayTime);
    _devices[i]->setFound(found);
  }

  Log.noticeln(F(""));

  for (uint8_t i = 0; i < _devices.size(); i++) {
    if (_devices[i]->found() == false) {
      Log.errorln(F("  ERROR: %S (%X) was not found"), _devices[i]->name(), _devices[i]->address());
      success = false;
    }
  }
  //Log.trace(F("i2c::scanI2C returning %T"), success);
  return success;
}

/**
 * @brief Probes for a device at `address` and mux port `port. Assumes `mux.setPort` has 
 * been called if needed.
 * 
 * @param address i2c address
 * @param port 
 * @param delayTime 
 * @return true 
 * @return false 
 */
bool i2c::testDevice(uint8_t address, uint8_t port, int delayTime) {
  byte error;
  bool success = true;

  __FlashStringHelper* name = (__FlashStringHelper*)F("n/a");
  i2cDevice* d = findDevice(address, port);
  if (d != nullptr) {
    name = (__FlashStringHelper*)d->name();
  }

  // The i2c_scanner uses the return value of
  // the Write.endTransmission to see if
  // a device did acknowledge to the address.
  // Log.traceln(F("\nProbing %S %X on port %X"), name, address, port);
  //Log.trace(F("[Probling %S (%X:%X)...] "), name, address, port);
  Wire.beginTransmission(address);
  delay(delayTime);
  error = Wire.endTransmission();

  if (error == 0) {
    Log.notice(F("%S (%X:%X), "), name, address, port);
  } else if (error == 4) {
    // Errors:
    //  0 : Success
    //  1 : Data too long
    //  2 : NACK on transmit of address
    //  3 : NACK on transmit of data
    //  4 : Other error

    Log.notice(F("%S (%X:%X) - (ERROR (%d): Unknown error),"),
        (const char*)name, address, port, error);
    success = false;
  } else {
    //Log.trace(F("[not found]"));
    success = false;
  }
  return success;
}

// from https://learn.sparkfun.com/tutorials/qwiic-mux-hookup-guide#arduino-example
#define MUX_ADDR 0x70  //7-bit unshifted default I2C Address

//Enables a specific port number
bool i2c::enableMuxPort(byte mux, byte portNumber) {
  if (portNumber > 7) portNumber = 7;

  Wire.beginTransmission(mux);
  //Read the current mux settings
  Wire.requestFrom(mux, (byte)0x01);
  if (!Wire.available()) return false;  //Error
  byte settings = Wire.read();

  //Set the wanted bit to enable the port
  settings |= (1 << portNumber);

  Wire.write(settings);
  Wire.endTransmission();
  return true;
}

//Disables a specific port number
bool i2c::disableMuxPort(uint8_t mux, uint8_t portNumber) {
  if (portNumber > 7) portNumber = 7;

  Wire.beginTransmission(mux);
  //Read the current mux settings
  Wire.requestFrom((int)mux, (int)1);
  if (!Wire.available()) return false;  //Error
  byte settings = Wire.read();

  //Clear the wanted bit to disable the port
  settings &= ~(1 << portNumber);

  Wire.write(settings);
  Wire.endTransmission();
  return true;
}

bool wasFound(uint8_t* foundAddr, uint8_t addr) {
  for (uint8_t i = 0; i < 127; i++) {
    if (foundAddr[i] == addr) return true;
  }
  return false;
}

/**
 * @brief Scan the entire i2c address range (mux 0-7, addr 0-127)
 * 
 * @param delayTime 
 * @return true 
 * @return false 
 */
bool i2c::scan(int delayTime) {
  bool success = true;

  Log.noticeln(F("Scanning the entire I2C address range (addr 0-127, mux 0-7)..."));

  Wire.begin();
  Log.noticeln(F("  Wire initialzied"));

  uint8_t foundAddr[128];
  QWIICMUX mux[8];
  uint8_t muxes = 0;
  for (uint8_t address = QWIIC_MUX_DEFAULT_ADDRESS; address < QWIIC_MUX_DEFAULT_ADDRESS + 8; address++) {
    if (mux[muxes].begin(address)) {
      Log.noticeln(F("  Device at I2C address %X is a mux."), address);
      muxes++;
    }
  }

  Log.noticeln(F("Scanning for devices not on a mux"));
  for (uint8_t address = 0; address < 127; address++) {
    foundAddr[address] = 0xFF;
    if (testDevice(address, 0xFF, delayTime)) {
      //              Log.notice(F("found!"));
      foundAddr[address] = address;
    }
  }
  Log.noticeln(F(""));

  for (uint8_t m = 0; m < muxes; m++) {
    Log.noticeln(F("Scanning for devices connected to the mux at %X"), mux->getAddress());
    for (uint8_t port = 0; port < 8; port++) {
      Log.notice(F("  Port %X: "), port);
      for (uint8_t address = 0; address < 127; address++) {
        //if (mux[m].setPort(port)) {
        if (!wasFound(foundAddr, address) && enableMuxPort(mux[m].getAddress(), port)) {
          if (testDevice(address, port, delayTime)) {
            //              Log.notice(F("found!"));
          }
          disableMuxPort(mux[m].getAddress(), port);
        }
      }
      Log.noticeln(F(""));
    }
  }
  return success;
}

/**
 * @brief Called every _probeInterval MS. Updates the cached state for each
 * device from the real world; this, in turn, caused the devices to call 
 * stateChanged callbacks as-appropriate. 
  * 
  * @param forceNotify if true stateChange notification callbacks will be called. 
  * if false, callbacks will only be called if the state has changed since last notify
 */
void i2c::probeDevices(bool forceNotify) {
  if (initialized() &&  (_probeInterval > 0) && (forceNotify || (millis() - _probeTimer) >= _probeInterval)) {
    //Log.traceln(F("i2c::probeDevices"));
    _probeTimer = millis();

    for (uint8_t i = 0; i < _devices.size(); i++) {
      if (_devices[i]->initialized()) {
        _devices[i]->probe(forceNotify);
      }
    }
    // Log.traceln(F("i2c:probeDevices took %dms"), millis() - _probeTimer); // nominally ~200ms
  }
}

/**
   * @brief `Printable::printTo` - prints the current motor state (direction & speed)
   *
   * @param p
   * @return size_t
   */
size_t i2c::printTo(Print& p) const {
  int n = 0;
  for (uint8_t i = 0; i < _devices.size(); i++) { 
    n += p.println(*_devices[i]);
  }
  return n; 
}