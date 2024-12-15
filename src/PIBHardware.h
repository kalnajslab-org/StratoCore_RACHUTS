/*
 *  PIBHardware.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  Updated for MonDo Board: November 2020
 *  Updated for T4.1 Rev E Board: December 2024
 *  Pin and port definitions for the PIB
 */

#ifndef PIBHARDWARE_H
#define PIBHARDWARE_H

// Serial Ports
#define DEBUG_SERIAL    Serial
#define ZEPHYR_SERIAL   Serial1
#define MCB_SERIAL      Serial3
#define PU_SERIAL       Serial2
#define RS41_SERIAL   Serial6

// Digital Pins
#define PU_PWR_ENABLE   2
#define FORCEON_232     41 //Unused on MonDo and Rev E
#define FORCEOFF_232    42 //Unused on MonDo and Rev E
#define SAFE_PIN        31
//#define MCB_IO_1        32
#define PULSE_LED       3
#define RS41_PWR        36

// Analog Pins
#define VMON_3V3        A16
#define VMON_56V        A17
#define VMON_15V        A2
#define IMON_PU         A8 //Profiler Current Monitor using 0.2 Ohm R on negative 
#define IMON_PU_BTS     A9 //Profiler Current Monitor using BTS Current feedback
#define IMON_MONDO      A3//Mondo current from current monitor chip

// LoRa Module
#define SS_PIN          38
#define RESET_PIN       29
#define INTERUPT_PIN    37
#define LORA_SCK        27
#define LORA_MISO       39
#define LORA_MOSI       26

#endif /* PIBHARDWARE_H */