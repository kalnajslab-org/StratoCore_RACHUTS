# StratoPIB

This repository contains the code to run the Profiler Interface Board (PIB) on the Reeldown Aerosol, Clouds, Humidity, and Temperature Sensor (RACHuTS) flown by [LASP](https://lasp.colorado.edu/home/) on the CNES [Stratéole 2](https://strat2.org/) super-pressure balloon campaign. StratoPIB inherits functionality from [StratoCore](https://github.com/dastcvi/StratoCore). To understand StratoPIB, first read the documentation for StratoCore.

## Software Development Environment

All of the instruments use [Teensy 3.6](https://www.sparkfun.com/products/14057) Arduino-compatible MCU boards as the primary computer. Thus, this and all other Strateole 2 code is implemented for Arduino, meaning that all of this C++ code uses the Arduino drivers for the Teensy 3.6 and is compiled using the Arduino IDE with the [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html) plug-in.

*StratoPIB is known to work with Arduino 1.8.4 and Teensyduino 1.39, as well as with Arduino 1.8.11 and Teensyduino 1.51*

## RACHuTS Overview

RACHuTS is a unique instrument designed and built in LASP's Kalnajs Lab to perform in-situ profiles of up to two kilometers below a balloon platform by reeling down a sensor suite and then reeling it back up. Below is a simplified electronics block diagram of the system. The Profiler Interface Board (PIB), runs the StratoPIB software. The Motor Control Board software is in the [MCB](https://github.com/dastcvi/MCB) repository. The Profiling Unit software is in the [PUCode](https://github.com/kalnajslab/PUCode) repository. The motion controllers are commercial-off-the-shelf components from [Technosoft](https://technosoftmotion.com/en/home/).

<img src="/Documentation/ElectronicsFBD.png" alt="/Documentation/ElectronicsFBD.png" width="900"/>

## Testing

The [OBC Simulator](https://github.com/dastcvi/OBC_Simulator) is a piece of software developed specifically for LASP Stratéole 2 instrument testing using only the Teensy 3.6 USB port. It provides the full OBC interface to allow extensive testing. StratoCore must be configured (via its constructor) to use the `&Serial` pointer for both `zephyr_serial` and `debug_serial`, and the OBC Simulator will separately display Zephyr and debug messages, color-coded by severity.

## Components

<img src="/Documentation/StratoPIBComponents.png" alt="/Documentation/StratoPIBComponents.png" width="900"/>

## Action Handler

<img src="/Documentation/ActionHandler.png" alt="/Documentation/ActionHandler.png" width="900"/>

## Autonomous Mode

<img src="/Documentation/AutonomousMode.png" alt="/Documentation/AutonomousMode.png" width="900"/>