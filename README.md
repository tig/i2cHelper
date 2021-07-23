# I2C / Qwiic bus scanner and tester

Tests (scans) the I2C (and Sparkfun Qwiic) bus and connected devices on Arduino. Particularly usedful when uisng an I2C mux such as the Sparkfun Qwiic Mux and i2c extenders such as the Sparkfun Differential I2C Bus Extender.



* Currently built for the P1AM, but should work with any Arduino microcontroller 
with an I2C bus.
* Currenly coded for my specific setup, but easy to modify. Just change the `declare_devices` and related `enum` and add code required to init, set, and query your specific devices.
* The `i2c` class makes it easy to configure I2C devices within your real solution.
* Includes wrapper classes for several Sparkfun Qwiic-based devices:
    * SparkFun Qwiic Relay 
    * SparkFun Qwiic Button - modeled as a contact sensor
    * SparkFun VL53L1X Lidar Distance Sensor 
* Logs results to `Serial`
* Supports Telnet connections with a set of commands. Connect with Putty or whatever and type `?` or `help` to see available commands.

## Usage

1. Load the project in VS Code/ PlatformIO
2. Build & Upload
3. Monitor - in the Serial Monitor type `?` and press enter:

```
>?
get     Gets state info - [fwd|rwd|motor ([speed|direction])]
init    Inits and tests - [i2c|mux|fwd|rwd|motor]
log     Sets logging - [serial*|shell] [verbose*|info|silent]
reboot  Reboots the microcontroller
set     Sets a property - [relay1|relay2|motor|resurrect] ([on|off] | [fwd|rev|stop|off])
shell   Configures shell - [telnet*|raw]
status  Prints status - [system|door|actuator|slider [fwd|rwd]]
> init i2c
>>>>> Command: init i2c  <<<<<
I2C devices found: Actuator Relay1 (0x0018), Actuator Relay2 (0x0019), Resurrection Relay (0x001A), Motor Controller (0x0058), Forward End-range Sensor (0x006D), Closed Sensor (0x006E), Opened Sensor (0x006F), Mux (0x0070), Forward Distance Sensor (Mux Port 0x0004), 
  ERROR: Rearward Distance Sensor (0x0005) was not found 
  ERROR: Rearward End-range Sensor (0x006C) was not found
> get fwd
>>>>> Command: get fwd  <<<<<
Forward Distance Sensor = 42 mm
42
> get motor direction
>>>>> Command: get motor direction  <<<<<
Stopped, speed = 0

Stopped
>
```