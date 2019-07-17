/*
 *  PIBHardware.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  
 *  Pin and port definitions for the PIB
 */

#ifndef PIBHARDWARE_H
#define PIBHARDWARE_H

// Serial Ports
#define ZEPHYR_SERIAL   Serial1
#define MCB_SERIAL      Serial2
#define PU_SERIAL       Serial3 // need to verify

// Digital Pins
#define PU_PWR_ENABLE   26
#define FORCEON_232     29
#define FORCEOFF_232    30
#define SAFE_PIN        31
//#define MCB_IO_1        32
#define PULSE_LED       38

// Analog Pins
#define VMON_3V3        A7
#define VMON_12V        A20
#define VMON_15V        A2
#define IMON_PU         A3

#endif /* PIBHARDWARE_H */