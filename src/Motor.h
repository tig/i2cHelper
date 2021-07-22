#pragma once
#include <Arduino.h>
#include <ArduinoLog.h>

#ifdef ARDUINO_SAMD_MKRZERO
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <SparkFun_VL53L1X.h>
#include <Adafruit_BusIO_Register.h>
#endif

// Devantech MD04 info
//  - https://www.robotshop.com/media/files/pdf/md04tech.pdf)
// Register   Name          Read/write  Description
// 0          Command       R/W         Write 1 for forwards, 2 for reverse, 0 for instant stop
// 1          Status        R           Acceleration, temperature and current status
// 2          Speed         R/W         Motor speed 0-255 (0x00 - 0xFF)
// 3          Acceleration  R/W         Motor acceleration 0-255 (0x00 - 0xFF)
// 4          Temperature   R           Module temperature in degrees centigrade
// 5          Motor current R           Motor current
// 6          Unused        R           Read as zero
// 7          Version       R           Software revision

/**
 * @brief Wraps the slider motor
 * 
 */
class Motor {
 public:
/**
 * @brief   Address of MD04 motor controller on the I2C bus
 * 
 */
#ifdef ARDUINO_SAMD_MKRZERO
  static const uint8_t MD04_MOTOR_CONTROLLER_ADDRESS = 0x58;
  static const uint8_t RESURECTION_RELAY_ADDRESS = 0x1A;
#else
  static const uint8_t MD04_MOTOR_CONTROLLER_ADDRESS = 26;  // LED on Digital IO #26
  static const uint8_t RESURECTION_RELAY_ADDRESS = 0x1A;
#endif

  // MD04 Registers
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

  // MD04 Command Register (CMDREG) commands
  static const uint8_t MOTOR_STOP = 0x00;  // DO NOT USE THIS EXCEPT IN ERROR STATE (see comment above)
  static const uint8_t MOTOR_FORWARD = 0x01;
  static const uint8_t MOTOR_BACKWARD = 0x02;

  // MD04 motor controller speed register (SPEEDREG) settings
  static const uint8_t ACCEL_RATE = 10;   // 0 is 0.187s, 255 is just under 8 seconds
  static const uint8_t FULL_SPEED = 255;  // Maximum speed
  static const uint8_t STOP_SPEED = 0;    // speed when stopped

  Motor() {
    _cachedSpeed = STOP_SPEED;
    _cachedDirection = MOTOR_STOP;
  }

  /**
   * @brief  Setup motor controller. Note that it MUST have 12v to initialize
   * @return true 
   * @return false 
   */
  bool begin() {
    bool result = true;
#ifdef ARDUINO_SAMD_MKRZERO
    Log.noticeln(F("  Setting up I2C motor controller at %X"), MD04_MOTOR_CONTROLLER_ADDRESS);

    if (!_motorController.begin()) {
      Log.errorln(F("   ERROR: Did not find motor controller at %X"),
          _motorController.address());
      result = false;
    } else {
      Log.notice(F("    Initializing motor ["));

      // Must set speed first when stopping
      Log.notice(F("speed -> %d, "), STOP_SPEED);
      _speedReg.write(STOP_SPEED, 1);

      Log.notice(F("accel rate -> %d, "), ACCEL_RATE);
      _accelReg.write(ACCEL_RATE, 1);

      Log.noticeln(F("command -> %d]"), MOTOR_STOP);
      _cmdReg.write(MOTOR_STOP, 1);

      Log.notice(F("  Verifying motor ["));
      uint8_t byte;
      _speedReg.read(&byte);
      //assert(byte == _cachedSpeed);
      Log.notice(F("speed = %d, "), byte);

      _accelReg.read(&byte);
      Log.notice(F("accel rate = %d, "), byte);

      _cmdReg.read(&byte);
      //assert(byte == _cachedDirection);
      Log.notice(F("command = %d]"), byte);

      Log.noticeln(F("   Initialized!"));
    }
#else
    pinMode(MD04_MOTOR_CONTROLLER_ADDRESS, OUTPUT);
    digitalWrite(MD04_MOTOR_CONTROLLER_ADDRESS, LOW);

#endif
    return result;
  }

  uint32_t getSpeed() {
#ifdef ARDUINO_SAMD_MKRZERO
    uint8_t byte;
    if (!_speedReg.read(&byte)) {
      Log.errorln(F("ERROR: Failed to read motor speed"));
    }
    //Log.traceln(F("Read motor speed: %d, %d"), byte, (uint32_t)byte);
    _cachedSpeed = byte;
#endif
    return _cachedSpeed;
  }

  /**
   * @brief Causes the motor to stop immediately. Do not do this regularly; use setSpeed instead
   * 
   */
  void emergencyStop() {
    Log.errorln(F("ERROR: emergencyStop of motor"));
    _cachedSpeed = STOP_SPEED;
    _cachedDirection = MOTOR_STOP;
#ifdef ARDUINO_SAMD_MKRZERO
    _speedReg.write(_cachedSpeed, 1);
    _cmdReg.write(MOTOR_STOP, 1);
    delay(100);
#else
    digitalWrite(MD04_MOTOR_CONTROLLER_ADDRESS, LOW);
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

  uint8_t getDirection() {
    uint8_t byte;
#ifdef ARDUINO_SAMD_MKRZERO
    if (!_cmdReg.read(&byte)) {
      Log.errorln(F("ERROR: Failed to read motor dirction"));
    }
    _cachedDirection = byte;
#endif
    return _cachedDirection;
  }

  /**
   * @brief Set the speed (and direction) of the motor. 
   * Note that if the speed is set to 0, the motor WILL NOT STOP immediately.
   * Instead it will start decellerating according to _accelReg
   * 
   * @param speed 
   */
  void setSpeed(uint8_t speed) {
    _cachedSpeed = speed;
    //Log.traceln(F("Motor::setSpeed(%d); was %d"), speed, _cachedSpeed);
#ifdef ARDUINO_SAMD_MKRZERO

    // MUST set speed first, then direction
    _speedReg.write(_cachedSpeed, 1);

    // note we don't do MOTOR_STOP; doing so overrides acelleration!
    _cmdReg.write(_cachedDirection, 1);
#else
    digitalWrite(MD04_MOTOR_CONTROLLER_ADDRESS, _cachedSpeed > 0 ? HIGH : LOW);
#endif
  }

 private:
#ifdef ARDUINO_SAMD_MKRZERO
  // MD04 I2C Motor controller
  Adafruit_I2CDevice _motorController = Adafruit_I2CDevice(MD04_MOTOR_CONTROLLER_ADDRESS);
  Adafruit_BusIO_Register _cmdReg = Adafruit_BusIO_Register(&_motorController, REG_CMD, 1, LSBFIRST);
  Adafruit_BusIO_Register _statusReg = Adafruit_BusIO_Register(&_motorController, REG_STATUS, 1, LSBFIRST);
  Adafruit_BusIO_Register _speedReg = Adafruit_BusIO_Register(&_motorController, REG_SPEED, 1, LSBFIRST);
  Adafruit_BusIO_Register _accelReg = Adafruit_BusIO_Register(&_motorController, REG_ACCEL, 1, LSBFIRST);
  Adafruit_BusIO_Register _tempReg = Adafruit_BusIO_Register(&_motorController, REG_TEMP, 1, LSBFIRST);
  Adafruit_BusIO_Register _currentReg = Adafruit_BusIO_Register(&_motorController, REG_CURRENT, 1, LSBFIRST);
  uint8_t _motorBuffer[8];  // buffer for accessing the motor controller I2C regs
  uint8_t _tempMotorReg;
#endif

  /**
   * @brief Cached motor speed
   */
  uint32_t _cachedSpeed = 0;
  uint8_t _cachedDirection;
};