# I2C and Qwiic bus scanner and tester

Tests (scans) the I2C (and Sparkfun Qwiic) bus and connected devices on Arduino. Particularly usedful when uisng an I2C mux such as the Sparkfun Qwiic Mux and i2c extenders such as the Sparkfun Differential I2C Bus Extender.

* Currently built for the P1AM, but should work with any Arduino microcontroller 
with an I2C bus.
* Currenly coded for my specific setup, but easy to modify. Just change the `declare_devices` and related `enum`.
* The `i2c` class makes it easy to configure I2C devices within your real solution.
* Includes wrapper classes for several Sparkfun Qwiic-based devices:
    * SparkFun Qwiic Relay 
    * SparkFun Qwiic Button - modeled as a contact sensor
    * SpqrkFun VL53L1X Lidar Distance Sensor 
* Logs results to `Serial`
* Supports Telnet connections with a set of commands. Connect with Putty or whatever and type `?` or `help` to see available commands.

