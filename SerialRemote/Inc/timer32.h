/*****************************************************************************
 *   timer32.h:  Header file for NXP LPC13xx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2008.08.20  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#ifndef __TIMER32_H 
#define __TIMER32_H

#include <type.h>

// CodeRed - variable name changed in CMSIS 1.3
#define SystemFrequency SystemCoreClock

#define TIME_INTERVAL	((SystemFrequency/LPC_SYSCON->SYSAHBCLKDIV)/100 - 1)
/* depending on the SystemFrequency and System AHB clock divider setting, 
if SystemFrequency = 60Mhz, SYSAHBCLKDIV = 4, SystemAHBFrequency = 1/4 SystemFrequency, 
10mSec = 150.000-1 counts */

void delay32Ms(uint8_t timer_num, uint32_t delayInMs);

#undef TIMER32_0_DEFAULT_HANDLER
#undef TIMER32_1_DEFAULT_HANDLER

#ifdef TIMER32_0_DEFAULT_HANDLER
extern volatile uint32_t timer32_0_counter;
#endif

#ifdef TIMER32_1_DEFAULT_HANDLER
extern volatile uint32_t timer32_1_counter;
#endif

void enable_timer32(uint8_t timer_num);
void disable_timer32(uint8_t timer_num);
void reset_timer32(uint8_t timer_num);
void init_timer32(uint8_t timer_num, uint32_t timerInterval);

#endif /* end __TIMER32_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
