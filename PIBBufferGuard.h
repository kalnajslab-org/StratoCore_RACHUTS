/*
 *  PIBBufferGuard.h
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *  
 *  Provides compiler checks that the software serial buffers are correctly sized.
 * 
 *  In order to do some of the asychronous communication at the data amounts
 *  and rates these payloads require, yet with the lack of direct UART interrupt
 *  access that Arduino gives, we need to modify the software Serial buffers in
 *  the Teensy 3.6 core.
 */

// To change a buffer, locate teensy/avr/cores/teensy3/HardwareSerial.h
// Add the following lines (uncommented) near the top (but after "#define 
// HardwareSerial_h") and change any buffer sizes desired from the 64-byte 
// default.
// -------------- Strateole 2 Buffer Changes --------------
// #define SERIAL1_RX_BUFFER_SIZE     64
// #define SERIAL2_RX_BUFFER_SIZE     64
// #define SERIAL3_RX_BUFFER_SIZE     64
// #define SERIAL4_RX_BUFFER_SIZE     64
// #define SERIAL5_RX_BUFFER_SIZE     64
// #define SERIAL6_RX_BUFFER_SIZE     64
// --------------------------------------------------------

// Each of the following checks to make sure the buffer has been redefined, and
// that the size is correct. If either is not true, a compiler error will be thrown.

#include "HardwareSerial.h" // not necessary, but makes it easy to find the file with a smart IDE

#ifndef SERIAL1_RX_BUFFER_SIZE
#error "Need to redefine the Serial1 buffer size to 512"
#elif SERIAL1_RX_BUFFER_SIZE != 512 // keep a little headroom for Zephyr
#error "Serial1 buffer should be 64 bytes"
#endif

#ifndef SERIAL2_RX_BUFFER_SIZE
#error "Need to redefine the Serial2 buffer size to 128"
#elif SERIAL2_RX_BUFFER_SIZE != 128 // MCB comms should be small
#error "Serial2 buffer should be 1024 bytes"
#endif

#ifndef SERIAL3_RX_BUFFER_SIZE
#error "Need to redefine the Serial3 buffer size to 4096"
#elif SERIAL3_RX_BUFFER_SIZE != 4096 // todo: optimize this for PU
#error "Serial3 buffer should be 64 bytes"
#endif

#ifndef SERIAL4_RX_BUFFER_SIZE
#error "Need to redefine the Serial4 buffer size to 64"
#elif SERIAL4_RX_BUFFER_SIZE != 64
#error "Serial4 buffer should be 64 bytes"
#endif

#ifndef SERIAL5_RX_BUFFER_SIZE
#error "Need to redefine the Serial5 buffer size to 64"
#elif SERIAL5_RX_BUFFER_SIZE != 64
#error "Serial5 buffer should be 64 bytes"
#endif

#ifndef SERIAL6_RX_BUFFER_SIZE
#error "Need to redefine the Serial6 buffer size to 64"
#elif SERIAL6_RX_BUFFER_SIZE != 64
#error "Serial6 buffer should be 64 bytes"
#endif



