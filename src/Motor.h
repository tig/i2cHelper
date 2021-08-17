#pragma once
#include <Arduino.h>
#include <ArduinoLog.h>

#ifndef SIMULATION
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <SparkFun_VL53L1X.h>
#include <Adafruit_BusIO_Register.h>
#endif

#include "i2c.h"

/**
 * @brief Wraps the slider motor
 * 
 */
class Motor : public i2cDevice {
 public:
  using i2cDevice::i2cDevice;

  /**
   * @brief  Setup motor controller. Note that it MUST have 12v to initialize
   * @return true 
   * @return false 
   */
  virtual bool begin() override {
    //Log.traceln(F("Motor::begin()"));
    bool result = i2cDevice::begin();

#ifdef SIMULATION
    pinMode(address(), OUTPUT);
    digitalWrite(address(), LOW);
#endif
    return result;
  }

  virtual uint32_t status() {
    //Log.errorln(F("Motor::status()"));
    _statusString[0] = '\0';
    return _cachedStatus;
  }

  virtual void setStatus(uint32_t status) {
    _statusString[0] = '\0';
    if (_cachedStatus != status) {
      _cachedStatus = status;
      setStateChanged();
    }
  }

  /**
   * @brief Get the Status as a REAL MEMORY string using _cachedStatus
   * (call `probe()` first)
   * 
   * @return const __FlashStringHelper* status string
   */
  const char *getStatusString() const {
    // Bit 0 is read as high when the drive is accelerating the motor.
    // Bit 1 set high indicates that the current through the motor has reached 20Amps
    // Bit 2 set high indicates that the over temperature limiter is active.
    // Bit 7 is the busy flag (never seen)
    const char *nominal = PSTR("ok");
#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
    // these platforms do not support PROGMEM
    sprintf((char *)_statusString, "a: %s, c: %s, t: %s",
        (_cachedStatus & 1) ? "yes" : "no",
        (_cachedStatus & 2) ? "max" : nominal,
        (_cachedStatus & 3) ? "over" : nominal);
#else
    sprintf_P((char *)_statusString, F("a: %S | c: %S | t: %S"),
        (_cachedStatus & 1) ? F("yes") : F("no"),
        (_cachedStatus & 2) ? F("max") : (__FlashStringHelper *)nominal,
        (_cachedStatus & 3) ? F("over") : (__FlashStringHelper *)nominal);
#endif

    return _statusString;
  }

  /**
   * @brief Set the speed of the motor. 
   * 
   * @param speed 
   */
  virtual void setSpeed(uint8_t speed) {
    if (_cachedSpeed != speed) {
      _cachedSpeed = speed;
      setStateChanged();
    }

    if (!initialized()) {
      Log.traceln(F("ERROR: Motor::setSpeed() when not initialized."));
      return;
    }
#ifdef SIMULATION
    digitalWrite(address(), _cachedSpeed > 0 ? HIGH : LOW);
#endif
    //Log.traceln(F("Motor::setSpeed() - %p"), this);
  }

  virtual uint32_t speed() {
    return _cachedSpeed;
  }

  /**
   * @brief Set the acceleration rate of the motor. 
   * @param acceleration 0-255 rate of acceleration 
   */
  virtual void setAcceleration(uint8_t acceleration) {
    if (_cachedAccel != acceleration) {
      _cachedAccel = acceleration;
      setStateChanged();
    }

    if (!initialized()) {
      Log.traceln(F("ERROR: Motor::setAcceleration() when not initialized."));
      return;
    }

#ifndef SIMULATION
    digitalWrite(address(), _cachedAccel > 0 ? HIGH : LOW);
#endif
  }

  virtual uint32_t acceleration() {
    return _cachedAccel;
  }

  virtual uint32_t temperature() {
    return _cachedTemperature;
  }

  // supports simulation
  void setTemperature(uint32_t temp) {
    if (_cachedTemperature != temp) {
      _cachedTemperature = temp;
      //setStateChanged();
    }
  }

  virtual uint32_t current() {
    return _cachedCurrent;
  }

  // supports simulation
  void setCurrent(uint32_t current) {
    if (_cachedCurrent != current) {
      _cachedCurrent = current;
      //setStateChanged();
    }
  }

  /**
   * @brief Causes the motor to stop immediately. Do not do this regularly; use setSpeed instead
   * 
   */
  virtual void emergencyStop() {
    Log.errorln(F("\nERROR: Emergency stop of motor requested."));
    setDirection(MOTOR_STOP);
    stop();
    // Log.errorln(F("       Motor stopped. Must be re-initialized."));
    // setInitialized(false);
#ifdef SIMULATION
    digitalWrite(address(), LOW);
#endif
    if (stateChanged()) {
      notify();
    }
  }

  /**
   * @brief start the motor moving forward.
   * 
   */
  void forward() {
    //Log.traceln(F("%S::forward() "), name());
    if (!initialized()) {
      Log.errorln(F("ERROR: Motor::forward() when not initialized."));
      return;
    }
    setSpeed(Motor::speed());
    setDirection(MOTOR_FORWARD);
    if (stateChanged()) {
      notify();
    }
  }

  /**
   * @brief start the motor moving reverse.
   * 
   */
  void reverse() {
    //Log.traceln(F("%S::reverse() "), name());
    if (!initialized()) {
      Log.errorln(F("ERROR: Motor::reverse() when not initialized."));
      return;
    }
    setSpeed(Motor::speed());
    setDirection(MOTOR_BACKWARD);
    if (stateChanged()) {
      notify();
    }
  }

  /**
   * @brief normal stop of the motor (will continue to decelearate at accelration() rate)
   * 
   */
  virtual void stop() {
    //Log.traceln(F("%S::stop() "), name());
    if (!initialized()) {
      Log.errorln(F("ERROR: Motor::stop() when not initialized."));
      return;
    }
    setSpeed(STOP_SPEED);
    if (stateChanged()) {
      notify();
    }
  }

  /**
   * @brief Get the direction the motor is set to
   * 
   * @return uint8_t 
   */
  virtual uint8_t direction() {
    return _cachedDirection;
  }

  /**
   * @brief Set the Direction of the motor (fwd, rev, stop)
   * 
   * @param dir 
   */
  virtual void setDirection(uint8_t dir) {
    if (_cachedDirection != dir) {
      _cachedDirection = dir;
      setStateChanged();
    }
  }

  /**
   * @brief Get the Direction as a PROGMEM string using _cachedDirection
   * (call `probe()` first)
   * 
   * @return const __FlashStringHelper* 
   */
  const __FlashStringHelper *directionString() const {
    switch (_cachedDirection) {
      case MOTOR_STOP:
        return (__FlashStringHelper *)PSTR("Stopped");
        break;
      case MOTOR_FORWARD:
        return (__FlashStringHelper *)PSTR("Forward");
        break;
      case MOTOR_BACKWARD:
        return (__FlashStringHelper *)PSTR("Reverse");
        break;
    }
    return (__FlashStringHelper *)PSTR("n/a");
  }

  /**
   * @brief `Printable::printTo` - prints the current motor state (direction & speed)
   *
   * @param p
   * @return size_t
   */
  size_t printTo(Print &p) const {
    int n = i2cDevice::printTo(p);
    n += p.print(F("dir: "));
    n += p.print(directionString());
    n += p.print(F(", spd: "));
    n += p.print(_cachedSpeed, DEC);
    n += p.print(F(", acc: "));
    n += p.print(_cachedAccel, DEC);
    n += p.print(F(", amps: "));
    n += p.print(_cachedCurrent, DEC);
    n += p.print(F(", temp: "));
    n += p.print(_cachedTemperature, DEC);
    n += p.print(F(", "));
    return n += p.print(getStatusString());
  };

  /**
   * @brief MD04 Command Register (CMDREG) commands
   * 
   */
  static const uint8_t MOTOR_STOP = 0x00;  // DO NOT USE THIS EXCEPT IN ERROR STATE (see comment above)
  static const uint8_t MOTOR_FORWARD = 0x01;
  static const uint8_t MOTOR_BACKWARD = 0x02;

  /**
   * @brief MD04 motor controller speed register (SPEEDREG) settings
   * 
   */
  static const uint8_t ACCEL_RATE = 25;   // 0 is 0.187s, 255 is just under 8 seconds
  static const uint8_t FULL_SPEED = 255;  // Maximum speed
  static const uint8_t STOP_SPEED = 0;    // speed when stopped

 protected:
  uint8_t _cachedDirection = MOTOR_STOP;

 private:
  uint8_t _cachedStatus = 0;
  char _statusString[255] = "\0";
  uint32_t _cachedSpeed = FULL_SPEED;
  uint8_t _cachedAccel = ACCEL_RATE;
  uint8_t _cachedTemperature = 0;
  uint8_t _cachedCurrent = 0;
};

// https://www.robotshop.com/en/md04-24v-20a-h-bridge-usb-dc-motor-driver.html
class AdafruitMotorController : public Motor {
 public:
  using Motor::Motor;
  // AdafruitMotorController(uint8_t address, uint8_t muxPort,
  //     const __FlashStringHelper *name, QWIICMUX *mux, bool isMux = false)
  //     : Motor(address, muxPort, name, mux, isMux),
  //       _motorController(nullptr),
  //       _cmdReg(nullptr),
  //       _statusReg(nullptr),
  //       _speedReg(nullptr),
  //       _accelReg(nullptr),
  //       _tempReg(nullptr),
  //       _currentReg(nullptr) {}

  virtual bool begin() override {
    //Log.traceln(F("AdafruitMotorController::begin()"));
    bool success = Motor::begin();

    if (_motorController != nullptr) {
      Log.trace(F("  Motor was previously initialized. Cleaning up. "));
      delete _motorController;
      delete _cmdReg;
      delete _statusReg;
      delete _speedReg;
      delete _accelReg;
      delete _currentReg;
    }
    //Log.trace(F("  Initializing motor ["));

    _motorController = new Adafruit_I2CDevice(address());
    _cmdReg = new Adafruit_BusIO_Register(_motorController, REG_CMD, 1, LSBFIRST);
    _statusReg = new Adafruit_BusIO_Register(_motorController, REG_STATUS, 1, LSBFIRST);
    _speedReg = new Adafruit_BusIO_Register(_motorController, REG_SPEED, 1, LSBFIRST);
    _accelReg = new Adafruit_BusIO_Register(_motorController, REG_ACCEL, 1, LSBFIRST);
    _tempReg = new Adafruit_BusIO_Register(_motorController, REG_TEMP, 1, LSBFIRST);
    _currentReg = new Adafruit_BusIO_Register(_motorController, REG_CURRENT, 1, LSBFIRST);

    if (!_motorController->begin()) {
      Log.errorln(F("\nERROR: AdafruitMotorController failed to init"));
      setInitialized(false);
      return false;
    }

    // Must set speed first when stopping
    setSpeed(Motor::speed());
    setAcceleration(Motor::acceleration());
    setDirection(MOTOR_STOP);

    //Log.trace(F("  Verifying motor ["));
    uint8_t status;
    if (initialized() && !_statusReg->read(&status)) {
      Log.errorln(F("\nERROR: AdafruitMotorController failed to read back motor status"));
      setInitialized(false);
      return false;
    }

    Motor::setStatus(status);
    Log.trace(F(" [%p]"), this);
    return success;
  }

  /**
   * @brief updates cached members by reading from physical device.
   * 
   * @param forceNotify if true stateChange notification callbacks will be called. 
   * if false, callbacks will only be called if the state has changed since last notify
   * 
   */
  virtual void probe(bool forceNotify) override {
    if (!initialized()) {
      Log.traceln(F("ERROR: Motor::probe() when not initialized."));
      return;
    }

    uint8_t byte = 0;

    // ----------- status
    if (!_statusReg->read(&byte)) {
      Log.errorln(F("ERROR: AdafruitMotorController::status() failed to read motor status"));
      //emergencyStop();
      return;
    } else {
      // if (byte & 3) {
      //   Log.errorln(F("ERROR: AdafruitMotorController::status() is over temperature."));
      //   emergencyStop();
      // }
      Motor::setStatus(byte);
    }

    // ----------- speed
    if (!_speedReg->read(&byte)) {
      Log.errorln(F("ERROR: AdafruitMotorController failed to read motor speed"));
      return;
    } else {
      // DO NOT un-comment this and do not set the base speed; because of stop() we don't care.
      // Motor::setSpeed(byte);
    }

    // ----------- acceleration
    if (!_accelReg->read(&byte)) {
      Log.errorln(F("ERROR: AdafruitMotorController Failed to read motor acceleration"));
      return;
    } else {
      Motor::setAcceleration(byte);
    }

    // ----------- direction
    if (!_cmdReg->read(&byte)) {
      Log.errorln(F("ERROR: AdafruitMotorController failed to read motor dirction"));
      return;
    } else {
      Motor::setDirection(byte);
    }

    // ----------- temperature
    if (!_tempReg->read(&byte)) {
      Log.errorln(F("ERROR: AdafruitMotorController Failed to read motor temperature"));
      return;
    } else {
      Motor::setTemperature(byte);
    }

    // ----------- current
    if (!_currentReg->read(&byte)) {
      Log.errorln(F("ERROR: AdafruitMotorController Failed to read motor current"));
      return;
    } else {
      Motor::setCurrent(byte);
      // if (Motor::current() > 100) {
      //   Log.errorln(F("ERROR: AdafruitMotorController::current() - Current is over 100A - EMERGENCY STOP"));
      //   emergencyStop();
      // }
    }
    Motor::probe(forceNotify);
  }

  /**
   * @brief Set the Direction of the motor (fwd, rev, stop)
   * 
   * @param dir 
   */
  virtual void setDirection(uint8_t dir) override {
    if (!initialized()) return;
    if (!_cmdReg->write(dir, 1)) {
      Log.errorln(F("ERROR: AdafruitMotorController::setDirection() - write() failed"));
    } else {
      Motor::setDirection(dir);
    }
  }

  /**
   * @brief Set the speed the motor will run at. 
   * 
   * @param speed 
   */
  virtual void setSpeed(uint8_t speed) override {
    if (!initialized()) return;

    // MUST set speed first, then direction
    if (!_speedReg->write(speed, 1)) {
      Log.errorln(F("ERROR: AdafruitMotorController::setSpeed() - write() failed"));
      speed = 0;
      if (!_speedReg->read(&speed)) {
        Log.errorln(F("ERROR: AdafruitMotorController::setSpeed() failed to read back speed"));
      } 
    } 
    Motor::setSpeed(speed);
  }

  /**
   * @brief Set the acceleration rate of the motor. 
   * Writing zero to the acceleration register will allow maximum acceleration 
   * of 0 to full in 0.187 seconds. This value actually controls a timer which steps the current motor speed 
   * towards the requested motor speed. It does this every (((acceleration register)*125)+768)uS. A value of
   * zero gives 768uS/step or 243*768=186624uS (0.187s) to accelerate from 0 to full. A value of 255 is 
   * ((255*125)+768)*243=7932249uS or just under 8 seconds.
   * 
   * @param acceleration 0-255 rate of acceleration 
   */
  virtual void setAcceleration(uint8_t acceleration) override {
    if (!initialized()) return;
    // MUST set acceleration first, then direction
    if (!_accelReg->write(acceleration, 1)) {
      Log.errorln(F("ERROR: AdafruitMotorController::setAcceleration() - write() failed"));
    } else {
      Motor::setAcceleration(acceleration);
    }
  }

 private:
  Adafruit_I2CDevice *_motorController = nullptr;

  // MD04 I2C Motor controller
  Adafruit_BusIO_Register *_cmdReg = nullptr;
  Adafruit_BusIO_Register *_statusReg = nullptr;
  Adafruit_BusIO_Register *_speedReg = nullptr;
  Adafruit_BusIO_Register *_accelReg = nullptr;
  Adafruit_BusIO_Register *_tempReg = nullptr;
  Adafruit_BusIO_Register *_currentReg = nullptr;

  /**
  * @brief Devantech MD04 info
  *  - https://www.robotshop.com/media/files/pdf/md04tech.pdf)
  * Register   Name          Read/write  Description
  * 0          Command       R/W         Write 1 for forwards, 2 for reverse, 0 for instant stop
  * 1          Status        R           Acceleration, temperature and current status
  * 2          Speed         R/W         Motor speed 0-255 (0x00 - 0xFF)
  * 3          Acceleration  R/W         Motor acceleration 0-255 (0x00 - 0xFF)
  * 4          Temperature   R           Module temperature in degrees centigrade
  * 5          Motor current R           Motor current
  * 6          Unused        R           Read as zero
  * 7          Version       R           Software revision
  */

  /* MD04 Registers */
  /**
   * @brief Write 1 for forwards, 2 for reverse, 0 for instant stop;
   * Be sure to have set the speed and acceleration before issuing these commands.
   * 
   * The way to stop the motor is the same as other speed changes, write the new speed to the speed
   * register and re-issue the direction command. 
   * 
   * This will cause the motor to decelerate to a stop at the rate
   * set by the acceleration register. 
   * 
   * If you wish the motor to stop instantly and bypass the acceleration
   * feature then write zero here, be aware that bypassing the acceleration routines does put high stresses on
   * gearboxes of motors.
   */
  static const uint8_t REG_CMD = 0x00;

  /**
   * @brief Acceleration, temperature and current status
   * Bit 0 is read as high when the drive is accelerating the motor. It will be cleared when the requested 
   *   speed is achieved or the over current or over temperature limiters are active.
   * Bit 1 set high indicates that the current through the motor has reached 20Amps and is being limited to 
   *   that value, the red LED will be illuminated when this happens.
   * Bit 2 set high indicates that the over temperature limiter is active. Above a preset threshold, the motor 
   *   current will be reduced in proportion to the MD04 temperature. The module can still be used but the 
   *   motor power will be limited, the red LED will be illuminated when this happens. It should be noted that
   *   a few minutes of running continuously at 20A will cause the heatsink to get hot - watch your fingers!
   * Bit 7 is the busy flag. It is set high when you issue a new command to the module. It is cleared very 
   *   quickly and you are unlikely ever to see it set.
   */
  static const uint8_t REG_STATUS = 0x01;

  /**
   * @brief Motor speed 0-255 (0x00 - 0xFF)
   * Sets the maximum speed that the motor will accelerate to. 
   * It is actually the 8-bit value sent to the modules PWM controller.
   * Write a value of 0 to 243 (numbers from 243 to 255 are clamped to 243). 
   * The larger the number, the more power is applied to the motor
   */
  static const uint8_t REG_SPEED = 0x02;

  /**
   * @brief Motor acceleration 0-255 (0x00 - 0xFF)
   * This sets the rate at which the motor accelerates or decelerates from where it is towards the new speed 
   * set by the speed register. Write a value of 0 to 255, the larger the number the longer the module will 
   * take to reach the new speed. Writing zero to the acceleration register will allow maximum acceleration 
   * of 0 to full in 0.187 seconds. This value actually controls a timer which steps the current motor speed 
   * towards the requested motor speed. It does this every (((acceleration register)*125)+768)uS. A value of
   * zero gives 768uS/step or 243*768=186624uS (0.187s) to accelerate from 0 to full. A value of 255 is 
   * ((255*125)+768)*243=7932249uS or just under 8 seconds.
   */
  static const uint8_t REG_ACCEL = 0x03;

  /**
   * @brief Module temperature in degrees centigrade
   * 
   */
  static const uint8_t REG_TEMP = 0x04;

  /**
   * @brief Motor current. This is the value used internally to limit the motor current to 20A. You do not need to read or do 
   * anything with it. The value is proportional to motor current, with a value of 186 representing the 20A 
   * limit
   * 
   */
  static const uint8_t REG_CURRENT = 0x05;
};