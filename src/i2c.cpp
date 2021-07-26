#include <Arduino.h>
#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>

#include "i2c.h"
#include "Sensors.h"

static const char errorStr[] PROGMEM = "n/a";

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
bool i2c::begin(i2cDevice** devices, size_t num) {
  bool success = true;
  _devices = devices;
  _numDevices = num;

  Log.traceln(F("Looking for these %d devices"), _numDevices);
  for (uint8_t i = 0; i < _numDevices; i++) {
    Log.trace(F("Address %X - %S"), _devices[i]->address(), _devices[i]->name());
    _devices[i]->setFound(false);
    if (_devices[i]->isMux()) {
      if (_devices[i]->mux()->begin(_devices[i]->address())) {
        Log.traceln(F(" -- is a mux and has initialized"));
      } else {
        Log.errorln(F("\n  ERROR: Mux at %X failed initialized"), _devices[i]->address());
        success = false;
      }
    } else if (_devices[i]->muxPort() != 0xFF) {
      Log.traceln(F(" -- mux'd device on mux port %X"), _devices[i]->muxPort());
    } else {
      Log.traceln(F(""));
    }
  }
  return success;
}

/**
 * @brief Given an I2C address, return the device that matches. 
 * 
 * @param address - I2C address
 * @return i2cDevice* 
 */
i2cDevice* i2c::findDevice(uint8_t address, uint8_t port) {
  for (uint8_t i = 0; i < _numDevices; i++) {
    if (_devices[i]->address() == address && _devices[i]->address() == port) {
      return _devices[i];
    }
  }
  return nullptr;
}

// /**
//  * @brief Given a mux port (0-7), return the corresponding device. `isOnMux` will be `true`
//  *
//  * @param port - Port on the I2c mux.
//  * @return i2cDevice*
//  */
// i2cDevice* i2c::findDeviceOnMux(uint8_t port) {
//   for (uint8_t i = 0; i < _numDevices; i++) {
//     if (_devices[i]->isOnMux == true && _devices[i]->address == port) {
//       return &_devices[i];
//     }
//   }
//   return nullptr;
// }

/**
 * @brief scan the i2c bus for all devices. Uses the declare_devices array of
 * i2cDevice structs to pretty print results. Deep scans into the mux and uses
 * knowledge of DistanceSensors to detect they are there.
 * 
 * @param delayTime 
 * @return true 
 * @return false 
 */
bool i2c::scanI2C(int delayTime) {
  bool success = true;

  // Only scan for muxes that we know are there to save time
  for (uint8_t i = 0; i < _numDevices; i++) {
    _devices[i]->setFound(false);
  }

  Log.notice(F("I2C devices found: "));
  for (uint8_t i = 0; i < _numDevices; i++) {
    if (!_devices[i]->isMux() && _devices[i]->mux() != nullptr) {
      //Log.trace(F("!isMux - setting port %S"), _devices[i]->name());
      if (!_devices[i]->mux()->setPort(_devices[i]->muxPort())) {
        Log.errorln(F("  ERROR: Mux at %X failed setPort"), _devices[i]->address());
      }
    }

    bool found = probeDevice(_devices[i]->address(), _devices[i]->muxPort(), (const char*)_devices[i]->name(), delayTime);
    _devices[i]->setFound(found);
  }

  Log.noticeln(F(""));

  for (uint8_t i = 0; i < _numDevices; i++) {
    if (_devices[i]->found() == false) {
      Log.errorln(F("  ERROR: %S (%X) was not found"), _devices[i]->name(), _devices[i]->address());
      success = false;
    }
  }

  return success;
}

/**
 * @brief Probes for a device at `address` and mux port `port. Assumes `mux.setPort` has 
 * been called if needed.
 * 
 * @param address i2c address
 * @param port 
 * @param name 
 * @param delayTime 
 * @return true 
 * @return false 
 */
bool i2c::probeDevice(uint8_t address, uint8_t port, const char* name, int delayTime) {
  byte error;
  bool success = true;
  // The i2c_scanner uses the return value of
  // the Write.endTransmission to see if
  // a device did acknowledge to the address.
  // Log.traceln(F("\nProbing %S %X on port %X"), name, address, port);
  Wire.beginTransmission(address);
  delay(delayTime);
  error = Wire.endTransmission();

  if (error == 0) {
    if (port != 0xFF) {
      Log.notice(F("%S (%X:%X), "), name, address, port);
    } else {
      Log.notice(F("%S (%X), "), name, address);
    }
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
    // not found, but no error
    success = false;
  }
  return success;
}

/**
 * @brief Scan the entire i2c address range (mux 0-7, addr 0-127)
 * 
 * @param delayTime 
 * @return true 
 * @return false 
 */
bool i2c::testI2CMux(int delayTime) {
  bool success = true;

  Log.noticeln(F("Scaning the entire I2C address range (addr 0-127, mux 0-7)..."));

  for (uint8_t address = 0; address < 127; address++) {
    QWIICMUX mux;
    if (mux.begin(address)) {
      Log.traceln(F("Device at I2C address %X is a mux, scanning addr 0-127 across all 8 ports"), address);
      for (uint8_t port = 0; port < 8; port++) {
        if (mux.setPort(port)) {
          Log.traceln(F("  (port: %d, State: %B, isConnected: %T)"),
              mux.getPort(), mux.getPortState(), mux.isConnected());
          Log.notice(F("  I2C devices found on mux at %X: "), address);
          for (uint8_t addressOnMux = 0; addressOnMux < 127; addressOnMux++) {
            probeDevice(addressOnMux, port, PSTR("n/a"), delayTime);
          }
          Log.noticeln(F(""));
        } else {
          // no device there
        }
      }
    } else {
      // not a mux
      Log.notice(F("I2C device found: "));
      probeDevice(address, 0xFF, PSTR("n/a"), delayTime);
      Log.noticeln(F(""));
    }
  }
  return success;
}
