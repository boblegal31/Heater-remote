/*----------------------------------------------------------------------------
 *      Name:    vcomdemo.c
 *      Purpose: USB virtual COM port Demo
 *      Version: V1.02
 *----------------------------------------------------------------------------
 *      This software is supplied "AS IS" without any warranties, express,
 *      implied or statutory, including but not limited to the implied
 *      warranties of fitness for purpose, satisfactory quality and
 *      noninfringement. Keil extends you a royalty-free right to reproduce
 *      and distribute executable files created using this software for use
 *      on NXP Semiconductors LPC microcontroller devices only. Nothing else 
 *      gives you the right to use this software.
 *
 * Copyright (c) 2009 Keil - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "LPC13xx.h"    
#include "type.h"

#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "usbcore.h"
#include "cdc.h"
#include "cdcuser.h"
#include "serial.h"
#include "vcomdemo.h"
#include "string.h"

#define     EN_IOCON        (1<<16)
#define     EN_USBREG       (1<<14)

void SysTick_Enable(uint32_t period);
void ParseCommands(const char *buffer, int length);
void Serial_Receive (void);
void processOOK (void);

/*----------------------------------------------------------------------------
 Initializes the VCOM port.
 Call this function before using VCOM_putchar or VCOM_getchar
 *---------------------------------------------------------------------------*/
void VCOM_Init(void) {

  CDC_Init ();
}

#define BUFF_LENGTH	256
/*----------------------------------------------------------------------------
  Reads character from USB buffer and writes to serial port buffer
 *---------------------------------------------------------------------------*/
void VCOM_Usb2Serial(void) {
  static char serBuf [BUFF_LENGTH];
  static int bufUsed;
  int  numBytesToRead, numBytesRead, numAvailByte;

  CDC_OutBufAvailChar (&numAvailByte);
  if (numAvailByte > 0) {
      numBytesToRead = numAvailByte > (BUFF_LENGTH - bufUsed)? (BUFF_LENGTH - bufUsed) : numAvailByte;
      numBytesRead = CDC_RdOutBuf (&serBuf[bufUsed], &numBytesToRead);
      bufUsed += numBytesRead;
      if (strchr (serBuf, '\n') || strchr (serBuf, '\r'))
      {
    	  ParseCommands (serBuf, bufUsed);
    	  memset (serBuf, 0, sizeof (serBuf));
    	  bufUsed = 0;
      }
      if (bufUsed == BUFF_LENGTH)
      {
    	  memset (serBuf, 0, sizeof (serBuf));
    	  bufUsed = 0;
      }
  }

}

/*----------------------------------------------------------------------------
  Main Program
 *---------------------------------------------------------------------------*/
int main (void) {
	  /* Basic chip initialization is taken care of in SystemInit() called
	   * from the startup code. SystemInit() and chip settings are defined
	   * in the CMSIS system_<part family>.c file.
	   */

	/* Enable IOCON, and USB blocks */
	LPC_SYSCON->SYSAHBCLKCTRL |= (EN_IOCON | EN_USBREG);

  USBIOClkConfig();

  SysTick_Enable(5);
  /* Enable TIMER32B0 to count at 1MHz */
  init_timer32 (0, 72);

  VCOM_Init();                              // VCOM Initialization
  GPIOInit();

  USB_Init();                               // USB Initialization
  USB_Connect(TRUE);                        // USB Connect

  enable_timer32 (0);

  while (!USB_Configuration) ;              // wait until USB is configured

  while (1) {                               // Loop forever
    VCOM_Usb2Serial();
	Serial_Receive ();
	LPC_PMU->PCON |= 2;
	__WFI();
  } // end while											   
} // end main ()

