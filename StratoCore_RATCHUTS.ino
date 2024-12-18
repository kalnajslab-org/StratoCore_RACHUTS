/*  StratoPIB_Main.ino
 *  Author: Alex St. Clair
 *  Created: July 2019
 * 
 *  Main Arduino file for the RACHuTS Profiler Interface
 *  Board, or PIB, using the StratoCore framework for
 *  LASP Strateole Zephyr interface boards.
 */

#include "src/StratoRatchuts.h"
#include <TimerOne.h>

#define LOOP_TENTHS     10 // defines loop period in 0.1s

StratoRatchuts pib;

// timer control variables
volatile uint8_t timer_counter = 0;
volatile bool loop_flag = false;

// ISR for timer
void ControlLoopTimer(void) {
  if (++timer_counter == LOOP_TENTHS) {
    timer_counter = 0;
    loop_flag = true;
  }
}

// Loop timing function
void WaitForControlTimer(void) {
  while (!loop_flag) delay(1);

  loop_flag = false;
}

// Standard Arduino setup function
void setup()
{
  Serial.begin(115200);
  ZEPHYR_SERIAL.begin(115200);
  MCB_SERIAL.begin(115200);
  PU_SERIAL.begin(115200);

  // Timer interrupt setup for main loop timing
  Timer1.initialize(100000); // 0.1 s
  Timer1.attachInterrupt(ControlLoopTimer);

  // Wait two cycles to align timing
  WaitForControlTimer();
  WaitForControlTimer();

  pib.InitializeCore();
  pib.InstrumentSetup();
}

// Standard Arduino loop function
void loop()
{
  // StratoCore loop functions
  pib.RunScheduler();
  pib.RunRouter();
  pib.RunMCBRouter();
  pib.RunPURouter();
  pib.RunMode();
  pib.InstrumentLoop();
  pib.KickWatchdog();

  // Wait for loop timer
  WaitForControlTimer();
}

