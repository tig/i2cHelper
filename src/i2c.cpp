#include <Arduino.h>
#include <ArduinoLog.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Adafruit_BusIO_Register.h>

#include "i2c.h"
#include "Sensors.h"

static const char errorStr[] PROGMEM = "n/a";

i2cDevice declare_devices[] = {
    {false, i2c::MuxAddr, PSTR("Mux"), false},
    {false, i2c::OpenedSensorAddr, PSTR("Opened Sensor"), false},
    {false, i2c::ClosedSensorAddr, PSTR("Closed Sensor"), false},
    {false, i2c::ActuatorRelay1Addr, PSTR("Actuator Relay1"), false},
    {false, i2c::ActuatorRelay2Addr, PSTR("Actuator Relay2"), false},
    {false, i2c::MotorControllerAddr, PSTR("Motor Controller"), false},
    {true, i2c::ForwardDistanceSensorAddr, PSTR("Forward Distance Sensor"), false},
    {true, i2c::RearwardDistanceSensorAddr, PSTR("Rearward Distance Sensor"), false},
    {false, i2c::ForwardEndRangeSensorAddr, PSTR("Forward End-range Sensor"), false},
    {false, i2c::RearwardEndRangeSensorAddr, PSTR("Rearward End-range Sensor"), false},
    {false, i2c::ResurrectionRelayAddr, PSTR("Resurrection Relay"), false},
};

/**
 * @brief Global instance of the I2C bus.
 * 
 */
i2c Bus = i2c(declare_devices, sizeof(declare_devices) / sizeof(i2cDevice));

/**
 * @brief Call to initalize the I2C bus. Can (?) be called mulitple times to re-initialize.
 * 
 * @return true 
 * @return false 
 */
bool i2c::begin() {
  for (uint8_t i = 0; i < _numDevices; i++) {
    _devices[i].found = false;
  }
  return _i2cMux.begin(MuxAddr);
}

/**
 * @brief Given an I2C address, return the device that matches. 
 * 
 * @param address - I2C address
 * @return i2cDevice* 
 */
i2cDevice* i2c::find(uint8_t address) {
  for (uint8_t i = 0; i < _numDevices; i++) {
    if (_devices[i].isOnMux == false && _devices[i].address == address) {
      return &_devices[i];
    }
  }
  return nullptr;
}

/**
 * @brief Given a mux port (0-7), return the corresponding device. `isOnMux` will be `true`
 * 
 * @param port - Port on the I2c mux.
 * @return i2cDevice* 
 */
i2cDevice* i2c::findDeviceOnMux(uint8_t port) {
  for (uint8_t i = 0; i < _numDevices; i++) {
    if (_devices[i].isOnMux == true && _devices[i].address == port) {
      return &_devices[i];
    }
  }
  return nullptr;
}

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
  byte error, address;  // variable for error and I2C address
  bool success = true;

  for (uint8_t i = 0; i < _numDevices; i++) {
    _devices[i].found = false;
  }

  // Disable all ports on the mux (port # out of range)
  _i2cMux.setPort(8);

  Log.notice(F("I2C devices found: "));
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmission to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    delay(delayTime);
    error = Wire.endTransmission();

    if (error == 0) {
      auto dev = find(address);
      if (dev != nullptr) {
        dev->found = true;
      }
      Log.notice(F("%S (%X), "), (dev != nullptr) ? dev->name : errorStr, address);

      // Is this the Mux?
      if (dev->found == true && dev->address == MuxAddr) {
        for (uint8_t i = 0; i < _numDevices; i++) {
          if (_devices[i].isOnMux) {
            DistanceSensor sensor = DistanceSensor(_devices[i].address);
            if (sensor.begin(true)) {
              _devices[i].found = true;
              Log.notice(F("%S (Mux Port %X), "), _devices[i].name, _devices[i].address);
            } 
          }
        }
      }

    } else if (error == 4) {
      // Errors:
      //  0 : Success
      //  1 : Data too long
      //  2 : NACK on transmit of address
      //  3 : NACK on transmit of data
      //  4 : Other error

      Log.notice(F("%X, (ERROR (%d): Unknown error),"), address, error);
    }
  }
  Log.noticeln(F(""));

  for (uint8_t i = 0; i < _numDevices; i++) {
    if (_devices[i].found == false) {
      Log.errorln(F("  ERROR: %S (%X) was not found"), _devices[i].name, _devices[i].address);
    }
  }

  return success;
}

// Cycle through just the ports we care about to see if they are alive
// Valid ports are 0 through 7
// My solution has lidar sensors on ports 4 and 5
#define START_MUX_LCV 4
#define STOP_MUX_LCV  5

/**
 * @brief This exercise the mux.
 * 
 * @param delayTime 
 * @return true 
 * @return false 
 */
bool i2c::testI2CMux(int delayTime) {
  byte error, address;  // variable for error and I2C address
  bool success = true;
  byte currentPort = _i2cMux.getPort();

  Log.noticeln(F("Testing I2C Mux..."));
  _i2cMux.setPort(0);  // Connect master to port labeled '1' on the mux
  Log.traceln(F("  Port: %d, State: %B, isConnected: %T"),
      _i2cMux.getPort(), _i2cMux.getPortState(), _i2cMux.isConnected());

  for (int lcv = START_MUX_LCV; lcv <= STOP_MUX_LCV; lcv++) {
    auto dev = findDeviceOnMux(lcv);
    Log.noticeln(F("  Checking mux port: %S (%X)"), (dev != nullptr) ? dev->name : errorStr, lcv);

    // first set the port
    if (_i2cMux.setPort(lcv)) {
      Log.traceln(F("    Port: %d, State: %B, isConnected: %T"),
          _i2cMux.getPort(), _i2cMux.getPortState(), _i2cMux.isConnected());
    } else {
      Log.errorln(F("    ERROR: setPort FAILED - Port: %d, State: %B, isConnected: %T"),
          _i2cMux.getPort(), _i2cMux.getPortState(), _i2cMux.isConnected());
      if (dev->isOnMux) {
        success = false;
      }
    }

    delay(delayTime);
    Log.notice(F("    Using mux port %X, I2C device found at addresses: "), lcv);
    for (address = 1; address < 127; address++) {
      // The i2c_scanner uses the return value of
      // the Write.beginTransmission to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      delay(delayTime);
      error = Wire.endTransmission();

      if (error == 0) {
        auto dev = find(address);
        Log.notice(F("%S (%X), "), (dev != nullptr) ? dev->name : errorStr, address);
      } else if (error == 4) {
        // Errors:
        //  0 : Success
        //  1 : Data too long
        //  2 : NACK on transmit of address
        //  3 : NACK on transmit of data
        //  4 : Other error

        Log.notice(F("%X, (ERROR (%d): Unknown error),"), address, error);
      }
    }
    Log.noticeln(F(""));
  }

  _i2cMux.setPort(currentPort);
  return success;
}
