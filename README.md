# Twisted Fields RP2040 motor controller firmware

:warning: work in progress, being tested. do not use.

# Features

- SimpleFOC - FOC control for 2 motors
- Serial Commander interface for both motors
- Support for MT6701 encoder in SSI mode
- Support for MT6701 encoder in ABZ mode
- Status LED driver (3x Neopixel)





# Compiling the firmware

:warning: Current version depends on unreleased changes to the SimpleFOC library. To compile this project before the release of SimpleFOC 2.2.3, please git clone the [SimpleFOC dev branch](https://github.com/simplefoc/Arduino-FOC/tree/dev) into the lib/ directory of this project.

This project is set up to use PlatformIO. You can try in other environments, your mileage may vary. In particular, the board definition for the Twisted Fields RP2040 motor controller for PlatformIO is included in this project, and its equivalent for other environments would have to be configured first.

To compile the firmware, install PlatformIO, and then PlatformIO's RP2040 support.

The project should compile without further changes.