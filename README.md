# StratoCore_RATCHUTS

This repository contains the code to run the Profiler Interface Board (PIB) on the Reeldown Aerosol, Clouds, Humidity, and Temperature Sensor (RACHuTS) flown by [LASP](https://lasp.colorado.edu/home/) on the CNES [Stratéole 2](https://strat2.org/) super-pressure balloon campaign. StratoPIB inherits functionality from [StratoCore](https://github.com/kalnajslab-org/StratoCore). To understand StratoPIB, first read the documentation for StratoCore.

> The [kalnajslab-org](https://github.com/kalnajslab-org) repositories linked
> throughout this document (StratoCore, SerialComm, MCBComm, TeensyEEPROM,
> StrateoleXML, MCB_T4.1, OBC_Simulator, PUCode) are forks of the original
> [dastcvi](https://github.com/dastcvi) repositories.

## Software Development Environment

> This system was upgraded from Teensy 3.6 to [Teensy 4.1](https://www.pjrc.com/teensy/).

All of the instruments use [Teensy 4.1](https://www.pjrc.com/teensy/) Arduino-compatible MCU boards as the primary computer. This project has migrated from the Arduino IDE to [PlatformIO](https://platformio.org/): the code uses the Arduino framework/drivers for the Teensy 4.1, but is built via PlatformIO's `teensy41` board definition rather than the Arduino IDE + [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) plug-in (see `platformio.ini`).

The main sketch file is `StratoCore_RATCHUTS.ino` at the repository root. To build, use the PlatformIO `rachuts` environment, e.g. `pio run -e rachuts`.

## RACHuTS Overview

RACHuTS is a unique instrument designed and built in LASP's Kalnajs Lab to perform in-situ profiles of up to two kilometers below a balloon platform by reeling down a sensor suite and then reeling it back up. Below is a simplified electronics block diagram of the system. The Profiler Interface Board (PIB), runs the StratoPIB software. The Motor Control Board software is in the [MCB](https://github.com/kalnajslab-org/MCB_T4.1) repository. The Profiling Unit software is in the [PUCode](https://github.com/kalnajslab-org/PUCode) repository. The motion controllers are commercial-off-the-shelf components from [Technosoft](https://technosoftmotion.com/en/home/).

<img src="/Documentation/images/ElectronicsFBD.png" alt="/Documentation/images/ElectronicsFBD.png" width="900"/>

## Testing

Testing is now performed with the [ZephyrSim](https://github.com/kalnajslab-org/ZephyrSim) simulator.

> The previous shared-serial-port configuration is deprecated: StratoCore
> no longer expects a single `&Serial` pointer passed for both
> `zephyr_serial` and `debug_serial`. **Both a dedicated Zephyr serial
> connection and a Teensy USB `Serial` connection are required.**
> `StratoRatchuts` is constructed with `ZEPHYR_SERIAL` (`Serial1`) for the
> Zephyr link and `DEBUG_SERIAL` (`Serial`, the Teensy USB port) for debug
> output (see `PIBHardware.h`).

**Caveat:** the system must *also* be tested against the canonical CNES
[OBC Simulator](https://github.com/kalnajslab-org/OBC_Simulator) — testing
against ZephyrSim alone is not sufficient validation before flight.

## Components

The diagram below shows how StratoPIB extends the [StratoCore Components](https://github.com/kalnajslab-org/StratoCore#components) to suit the needs of RACHuTS. All of the requisite pure virtual functions are implemented (mode functions, telecommand handler, action handler, etc.), and StratoPIB adds a few major components: the MCB Router, PU Router, and Configuration Manager.

<img src="/Documentation/images/StratoPIBComponents.png" alt="/Documentation/images/StratoPIBComponents.png" width="900"/>

## MCB and PU Routers

The MCB and PU are both able to be communicated with over TTL UART (the PU only if it is docked). Each interface is defined using classes that derive from [SerialComm](https://github.com/kalnajslab-org/SerialComm), which is a simple, robust protocol for inter-Arduino serial communication. These interfaces are [MCBComm](https://github.com/kalnajslab-org/MCBComm) and [PUComm](https://github.com/kalnajslab-org/PUCode).

A router is implemented for each the MCBComm and the PUComm that checks for new messages and handles them accordingly. The routers are called each main loop in the Arduino file right after the Zephyr OBC router.

## PIB Buffer Guard

All of the serial routers (Zephyr OBC, MCB, and PU) depend on configurable buffering implemented in the Arduino Teensy core libraries (see the [explanation in SerialComm](https://github.com/kalnajslab-org/SerialComm#aside-on-arduinos-internal-serial-buffering)). The `PIBBufferGuard.h` file contains macros that ensure that the buffers have been correctly set, otherwise the macros will throw a compile-time error. On any computer that uses a Teensy where buffers are updated or memory is limited, it is recommended that you use a buffer guard like this for every project.

## Configuration Manager

Important configurations are stored in EEPROM on the PIB. The EEPROM storage is maintained by the `PIBConfigs` class, which derives from [TeensyEEPROM](https://github.com/kalnajslab-org/TeensyEEPROM). This library is a wrapper for the core EEPROM library that protects against EEPROM failure. A hard-coded default for each configuration is maintained in FLASH memory, and a mutable runtime variable exists for each in RAM. Thus, if the EEPROM fails, the configurations can still be changed in RAM and will update to a default value on a processor reset. The configurations can be changed via telecommands.

## Action Handler

StratoCore necessitates an action handler for actions scheduled in the [Scheduler](https://github.com/kalnajslab-org/StratoCore#scheduler). The action handler is a function called each time a scheduled action becomes ready. StratoPIB implements an "action flag" concept, which is just an enumerated boolean flag that goes stale (gets reset back to `false`) if it hasn't been read after a configurable number of loops (currently 3). This way, a mode function can set a flag, but the software designer doesn't have to handle the case of the mode being switched by StratoCore and the flag being left unchecked. The diagram below shows the "action flag" concept (the flag monitor is called automatically in the `InstrumentLoop` function):

<img src="/Documentation/images/ActionHandler.png" alt="/Documentation/images/ActionHandler.png" width="900"/>

## Telecommand Handler

Telecommands are handled in the `TCHandler.cpp` file. Typical telecommands will either cause actions to be scheduled or configurations to be changed. See [StratoCore Telecommand Handling](https://github.com/kalnajslab-org/StratoCore#telecommand-handling) for a detailed look at how telecommands work, and see [StrateoleXML](https://github.com/kalnajslab-org/StrateoleXML).

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

<img src="/Documentation/images/AutonomousMode.png" alt="/Documentation/images/AutonomousMode.png" width="900"/>

### TSEN Scheduling

TSEN (temperature) measurements are automatically generated by the profile unit when not profiling and stored until offloaded over serial to the PIB. Every 10 minutes, the PIB offloads the data. In the `InstrumentLoop` function, the `CheckTSEN` function is called that sets the `COMMAND_SEND_TSEN` action every 10 minutes. When not profiling or performing another task, the autnomous and manual mode loops both check for this flag and pull TSEN data accordingly using the `Flight_TSEN` state machine. Unlike the other event sequence state machines, this one can be overridden by the `ACTION_OVERRIDE_TSEN` flag being set in manual mode or the `ACTION_BEGIN_PROFILE` flag being set in autonomous mode.

## Other Modes

The modes other than flight (Standby, Safety, Low Power, and End of Flight) are all much simpler than flight. Look through the state machines in their individual source files to understand the operations. The only exception is that in Safety mode, the PIB commands the MCB to perform a full retract of the profiling unit, verifies it completes, sends a message informing the Zephyr OBC that it is safe, and verifies the Zephyr OBC sends an ACK.