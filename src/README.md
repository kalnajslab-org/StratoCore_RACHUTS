# StratoPIB

This repository contains the code to run the Profiler Interface Board (PIB) on the Reeldown Aerosol, Clouds, Humidity, and Temperature Sensor (RACHuTS) flown by [LASP](https://lasp.colorado.edu/home/) on the CNES [Stratéole 2](https://strat2.org/) super-pressure balloon campaign. StratoPIB inherits functionality from [StratoCore](https://github.com/dastcvi/StratoCore). To understand StratoPIB, first read the documentation for StratoCore.

## Software Development Environment

All of the instruments use [Teensy 3.6](https://www.sparkfun.com/products/14057) Arduino-compatible MCU boards as the primary computer. Thus, this and all other Strateole 2 code is implemented for Arduino, meaning that all of this C++ code uses the Arduino drivers for the Teensy 3.6 and is compiled using the Arduino IDE with the [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) plug-in.

The Arduino main file can be found in `examples/StratoPIB_Main.ino`. To compile and load StratoPIB, open this main file in Arduino and follow the Teensyduino instructions.

*StratoPIB is known to work with Arduino 1.8.4 and Teensyduino 1.39, as well as with Arduino 1.8.11 and Teensyduino 1.51*

## RACHuTS Overview

RACHuTS is a unique instrument designed and built in LASP's Kalnajs Lab to perform in-situ profiles of up to two kilometers below a balloon platform by reeling down a sensor suite and then reeling it back up. Below is a simplified electronics block diagram of the system. The Profiler Interface Board (PIB), runs the StratoPIB software. The Motor Control Board software is in the [MCB](https://github.com/dastcvi/MCB) repository. The Profiling Unit software is in the [PUCode](https://github.com/kalnajslab/PUCode) repository. The motion controllers are commercial-off-the-shelf components from [Technosoft](https://technosoftmotion.com/en/home/).

<img src="/Documentation/ElectronicsFBD.png" alt="/Documentation/ElectronicsFBD.png" width="900"/>

## Testing

The [OBC Simulator](https://github.com/dastcvi/OBC_Simulator) is a piece of software developed specifically for LASP Stratéole 2 instrument testing using only the Teensy 3.6 USB port. It provides the full OBC interface to allow extensive testing. StratoCore must be configured (via its constructor) to use the `&Serial` pointer for both `zephyr_serial` and `debug_serial`, and the OBC Simulator will separately display Zephyr and debug messages, color-coded by severity.

## Components

The diagram below shows how StratoPIB extends the [StratoCore Components](https://github.com/dastcvi/StratoCore#components) to suit the needs of RACHuTS. All of the requisite pure virtual functions are implemented (mode functions, telecommand handler, action handler, etc.), and StratoPIB adds a few major components: the MCB Router, PU Router, and Configuration Manager.

<img src="/Documentation/StratoPIBComponents.png" alt="/Documentation/StratoPIBComponents.png" width="900"/>

## MCB and PU Routers

The MCB and PU are both able to be communicated with over TTL UART (the PU only if it is docked). Each interface is defined using classes that derive from [SerialComm](https://github.com/dastcvi/SerialComm), which is a simple, robust protocol for inter-Arduino serial communication. These interfaces are [MCBComm](https://github.com/dastcvi/MCBComm) and [PUComm](https://github.com/kalnajslab/PUCode).

A router is implemented for each the MCBComm and the PUComm that checks for new messages and handles them accordingly. The routers are called each main loop in the Arduino file right after the Zephyr OBC router.

## PIB Buffer Guard

All of the serial routers (Zephyr OBC, MCB, and PU) depend on configurable buffering implemented in the Arduino Teensy core libraries (see the [explanation in SerialComm](https://github.com/dastcvi/SerialComm#aside-on-arduinos-internal-serial-buffering)). The `PIBBufferGuard.h` file contains macros that ensure that the buffers have been correctly set, otherwise the macros will throw a compile-time error. On any computer that uses a Teensy where buffers are updated or memory is limited, it is recommended that you use a buffer guard like this for every project.

## Configuration Manager

Important configurations are stored in EEPROM on the PIB. The EEPROM storage is maintained by the `PIBConfigs` class, which derives from [TeensyEEPROM](https://github.com/dastcvi/TeensyEEPROM). This library is a wrapper for the core EEPROM library that protects against EEPROM failure. A hard-coded default for each configuration is maintained in FLASH memory, and a mutable runtime variable exists for each in RAM. Thus, if the EEPROM fails, the configurations can still be changed in RAM and will update to a default value on a processor reset. The configurations can be changed via telecommands.

## Action Handler

StratoCore necessitates an action handler for actions scheduled in the [Scheduler](https://github.com/dastcvi/StratoCore#scheduler). The action handler is a function called each time a scheduled action becomes ready. StratoPIB implements an "action flag" concept, which is just an enumerated boolean flag that goes stale (gets reset back to `false`) if it hasn't been read after a configurable number of loops (currently 3). This way, a mode function can set a flag, but the software designer doesn't have to handle the case of the mode being switched by StratoCore and the flag being left unchecked. The diagram below shows the "action flag" concept (the flag monitor is called automatically in the `InstrumentLoop` function):

<img src="/Documentation/ActionHandler.png" alt="/Documentation/ActionHandler.png" width="900"/>

## Telecommand Handler

Telecommands are handled in the `TCHandler.cpp` file. Typical telecommands will either cause actions to be scheduled or configurations to be changed. See [StratoCore Telecommand Handling](https://github.com/dastcvi/StratoCore#telecommand-handling) for a detailed look at how telecommands work, and see [StrateoleXML](https://github.com/dastcvi/StrateoleXML).

## Flight Mode

The RACHuTS flight mode is necessarily complex. It is divided into a manual mode and an autonomous mode so that the instrument can be commissioned in manual mode and then set to run in autonomous mode.

### Flight State Machines

On RACHuTS, there are several complex event sequences that need to be performed with regularity, such as performing a profile or offloading data from the profiling unit. To avoid code redundancy and to make the code clearer, these event sequences are sequestered into their own self-contained state machines that can be called either via telecommand in manual mode, or autonomously in autonomous mode. Each state machine is contained in its own source file and called as a function once per loop. The generic function format is:

```C++
bool Flight_SequenceName(bool restart_state);
```

The functions should be called once per loop until they conclude (signaled by returning `true`). When the function is called for the first time, it should be passed `true` in the `restart_state` parameter. For all subsequent calls, it should be passed `false`. In the case of an error, the function will still return `true` to signify that it has completed, but the `inst_substate` variable will automatically be set to `MODE_ERROR`. Thus, no additional external error handling is required. The following are all of the implemented event sequences with self-contained state machines:

```C++
bool Flight_CheckPU(bool restart_state);
bool Flight_Profile(bool restart_state);
bool Flight_ReDock(bool restart_state);
bool Flight_PUOffload(bool restart_state);
bool Flight_TSEN(bool restart_state);
bool Flight_ManualMotion(bool restart_state);
bool Flight_DockedProfile(bool restart_state);
```

### Flight Manual Mode

Manual mode is the default state of the instrument, though this can be changed in `PIBConfigs` via telecommand. In this state, the software simply checks once per loop for any telecommands and enters event sequence state machines as necessary. Additionally, it checks to see if it is time to get TSEN data from the PU: more on that in a subsequent section.

### Flight Autonomous Mode

Autonomous mode is used to automatically run a number of preconfigured profiles each night, according to the configurations set in `PIBConfigs`. Below is a simplified flowchart for the mode.

<img src="/Documentation/AutonomousMode.png" alt="/Documentation/AutonomousMode.png" width="900"/>

### TSEN Scheduling

TSEN (temperature) measurements are automatically generated by the profile unit when not profiling and stored until offloaded over serial to the PIB. Every 10 minutes, the PIB offloads the data. In the `InstrumentLoop` function, the `CheckTSEN` function is called that sets the `COMMAND_SEND_TSEN` action every 10 minutes. When not profiling or performing another task, the autnomous and manual mode loops both check for this flag and pull TSEN data accordingly using the `Flight_TSEN` state machine. Unlike the other event sequence state machines, this one can be overridden by the `ACTION_OVERRIDE_TSEN` flag being set in manual mode or the `ACTION_BEGIN_PROFILE` flag being set in autonomous mode.

## Other Modes

The modes other than flight (Standby, Safety, Low Power, and End of Flight) are all much simpler than flight. Look through the state machines in their individual source files to understand the operations. The only exception is that in Safety mode, the PIB commands the MCB to perform a full retract of the profiling unit, verifies it completes, sends a message informing the Zephyr OBC that it is safe, and verifies the Zephyr OBC sends an ACK.