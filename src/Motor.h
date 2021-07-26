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
    bool result = i2cDevice::begin();
    _cachedSpeed = STOP_SPEED;
    _cachedDirection = MOTOR_STOP;
    _cachedAccel = ACCEL_RATE;

#ifdef SIMULATION
    pinMode(address(), OUTPUT);
    digitalWrite(address(), LOW);
#endif
    return result;
  }

  virtual uint32_t getStatus() {
    _statusString[0] = '\0';
    return _cachedStatus;
  }

  virtual void setStatus(uint32_t status) {
    _statusString[0] = '\0';
    _cachedStatus = status;
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
    const char *nominal = PSTR("nominal");
#if defined(ARDUINO_ARCH_SAMD) || defined(ARDUINO_ARCH_ESP32)
    // these platforms do not support PROGMEM
    sprintf((char *)_statusString, "acceleration: %s, current: %s, temperature: %s",
        (_cachedStatus & 1) ? "accelerating" : nominal,
        (_cachedStatus & 2) ? "max" : nominal,
        (_cachedStatus & 3) ? "over" : nominal);
#else
    sprintf_P((char *)_statusString, F("acceleration: %S | current: %S | temperature: %S"),
        (_cachedStatus & 1) ? F("accelerating") : (__FlashStringHelper *)nominal,
        (_cachedStatus & 2) ? F("max") : (__FlashStringHelper *)nominal,
        (_cachedStatus & 3) ? F("over") : (__FlashStringHelper *)nominal);
#endif

    return _statusString;
  }

  /**
   * @brief Set the speed (and direction) of the motor. 
   * 
   * @param speed 
   */
  virtual void setSpeed(uint8_t speed) {
    _cachedSpeed = speed;
    //Log.traceln(F("Motor::setSpeed(%d); was %d"), speed, _cachedSpeed);
#ifdef SIMULATION
    digitalWrite(address(), _cachedSpeed > 0 ? HIGH : LOW);
#endif
  }

  virtual uint32_t speed() {
    return _cachedSpeed;
  }

  /**
   * @brief Set the acceleration rate of the motor. 
   * @param acceleration 0-255 rate of acceleration 
   */
  virtual void setAcceleration(uint8_t acceleration) {
    _cachedAccel = acceleration;
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
    _cachedTemperature = temp;
  }

  virtual uint32_t current() {
    return _cachedCurrent;
  }

  // supports simulation
  void setCurrent(uint32_t current) {
    _cachedCurrent = current;
  }

  /**
   * @brief Causes the motor to stop immediately. Do not do this regularly; use setSpeed instead
   * 
   */
  virtual void emergencyStop() {
    Log.errorln(F("ERROR: emergencyStop of motor"));
    _cachedSpeed = STOP_SPEED;
    _cachedDirection = MOTOR_STOP;
    delay(100);
#ifdef SIMULATION
    digitalWrite(address(), LOW);
#endif
  }

  /**
   * @brief start the motor moving forward.
   * 
   */
  void forward() {
    _cachedDirection = MOTOR_FORWARD;
    setSpeed(STOP_SPEED);
  }

  /**
   * @brief start the motor moving reverse.
   * 
   */
  void reverse() {
    _cachedDirection = MOTOR_BACKWARD;
    setSpeed(STOP_SPEED);
  }

  virtual uint8_t direction() {
    return _cachedDirection;
  }

  // for simulation
  void setDirection(uint8_t dir) {
    _cachedDirection = dir;
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
   * @brief Asks device for speed & direction and updates
   * cached members.
   * 
   */
  void probe() {
    getStatus();
    speed();
    acceleration();
    direction();
    temperature();
    current();
    i2cDevice::address();
  }

  /**
   * @brief `Printable::printTo` - prints the current motor state (direction & speed)
   *
   * @param p
   * @return size_t
   */
  size_t printTo(Print &p) const {
    int n = i2cDevice::printTo(p);
    n += p.print(F("direction: "));
    n += p.print(directionString());
    n += p.print(F(", speed: "));
    n += p.print(_cachedSpeed, DEC);
    n += p.print(F(", accel: "));
    n += p.print(_cachedAccel, DEC);
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
  static const uint8_t ACCEL_RATE = 10;   // 0 is 0.187s, 255 is just under 8 seconds
  static const uint8_t FULL_SPEED = 255;  // Maximum speed
  static const uint8_t STOP_SPEED = 0;    // speed when stopped

 protected:
  uint8_t _cachedDirection;

 private:
  uint8_t _cachedStatus;
  char _statusString[255];
  uint32_t _cachedSpeed;
  uint8_t _cachedAccel;
  uint8_t _cachedTemperature;
  uint8_t _cachedCurrent;
};

class AdafruitMotorController : public Motor {
 public:
  virtual bool begin() override {
    bool success = Motor::begin();

    if (_motorController != nullptr) {
      delete _motorController;
      delete _cmdReg;
      delete _statusReg;
      delete _speedReg;
      delete _accelReg;
      delete _currentReg;
    }
    _motorController = new Adafruit_I2CDevice(address());
    _cmdReg = new Adafruit_BusIO_Register(_motorController, REG_CMD, 1, LSBFIRST);
    _statusReg = new Adafruit_BusIO_Register(_motorController, REG_STATUS, 1, LSBFIRST);
    _speedReg = new Adafruit_BusIO_Register(_motorController, REG_SPEED, 1, LSBFIRST);
    _accelReg = new Adafruit_BusIO_Register(_motorController, REG_ACCEL, 1, LSBFIRST);
    _tempReg = new Adafruit_BusIO_Register(_motorController, REG_TEMP, 1, LSBFIRST);
    _currentReg = new Adafruit_BusIO_Register(_motorController, REG_CURRENT, 1, LSBFIRST);

    if (!_motorController->begin(false)) {
      Log.errorln(F(" ERROR: Did not find motor controller at %X"),
          address());
      success = false;
    } else {
      Log.trace(F("  Initializing motor ["));

      // Must set speed first when stopping
      Log.trace(F("speed -> %d, "), STOP_SPEED);
      _speedReg->write(STOP_SPEED, 1);

      Log.trace(F("accel rate -> %d, "), ACCEL_RATE);
      _accelReg->write(ACCEL_RATE, 1);

      Log.trace(F("command -> %d]"), MOTOR_STOP);
      _cmdReg->write(MOTOR_STOP, 1);

      Log.trace(F("  Verifying motor ["));
      uint8_t byte;
      _speedReg->read(&byte);
      //assert(byte == _cachedSpeed);
      Log.trace(F("speed = %d, "), byte);

      _accelReg->read(&byte);
      Log.trace(F("accel rate = %d, "), byte);

      _cmdReg->read(&byte);
      //assert(byte == _cachedDirection);
      Log.trace(F("command = %d]"), byte);

      Log.noticeln(F("%p"), this);
    }
    return success;
  }

  virtual uint32_t getStatus() {
    Motor::getStatus();
    uint8_t byte;
    if (!_statusReg->read(&byte)) {
      Log.errorln(F("ERROR: Failed to read motor temperature"));
    }
    setStatus(byte);
    return byte;
  }

  /**
   * @brief Set the speed (and direction) of the motor. 
   * Note that if the speed is set to 0, the motor WILL NOT STOP immediately.
   * Instead it will start decellerating according to _accelReg
   * 
   * @param speed 
   */
  virtual void setSpeed(uint8_t speed) override {
    Motor::setSpeed(speed);
    // MUST set speed first, then direction
    _speedReg->write(speed, 1);
    // note we don't do MOTOR_STOP; doing so overrides acelleration!
    _cmdReg->write(Motor::acceleration(), 1);
  }

  virtual uint32_t speed() override {
    uint8_t byte;
    if (!_speedReg->read(&byte)) {
      Log.errorln(F("ERROR: Failed to read motor speed"));
    }
    Motor::setSpeed(byte);
    return Motor::speed();
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
    Motor::setAcceleration(acceleration);
    // MUST set acceleration first, then direction
    _accelReg->write(acceleration, 1);
  }

  virtual uint32_t acceleration() override {
    uint8_t byte;
    if (!_accelReg->read(&byte)) {
      Log.errorln(F("ERROR: Failed to read motor acceleration"));
    }

    Motor::setAcceleration(byte);
    return Motor::acceleration();
  }

  virtual uint32_t temperature() override {
    uint8_t byte;
    if (!_tempReg->read(&byte)) {
      Log.errorln(F("ERROR: Failed to read motor temperature"));
    }
    //Log.traceln(F("Read motor temperature: %d, %d"), byte, (uint32_t)byte);
    Motor::setTemperature(byte);
    return Motor::temperature();
  }

  virtual uint32_t current() override {
    uint8_t byte;
    if (!_currentReg->read(&byte)) {
      Log.errorln(F("ERROR: Failed to read motor temperature"));
    }

    Motor::setCurrent(byte);
    return Motor::current();
  }

  /**
   * @brief Causes the motor to stop immediately. Do not do this regularly; use setSpeed instead
   * 
   */
  virtual void emergencyStop() override {
    Motor::emergencyStop();
    _speedReg->write(Motor::speed(), 1);
    _cmdReg->write(MOTOR_STOP, 1);
    // TODO: why is this here?
    delay(100);
  }

  virtual uint8_t direction() override {
    uint8_t byte;
    if (!_cmdReg->read(&byte)) {
      Log.errorln(F("ERROR: Failed to read motor dirction"));
    }
    Motor::setDirection(byte);
    return Motor::direction();
  }

 private:
  Adafruit_I2CDevice *_motorController;

  // MD04 I2C Motor controller
  Adafruit_BusIO_Register *_cmdReg;
  Adafruit_BusIO_Register *_statusReg;
  Adafruit_BusIO_Register *_speedReg;
  Adafruit_BusIO_Register *_accelReg;
  Adafruit_BusIO_Register *_tempReg;
  Adafruit_BusIO_Register *_currentReg;
  uint8_t _motorBuffer[8];  // buffer for accessing the motor controller I2C regs

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