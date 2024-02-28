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

#include "chip.h"
#include "type.h"

#include "usb.h"
#include "usbcfg.h"
#include "usbhw.h"
#include "usbcore.h"
#include "cdc.h"
#include "cdcuser.h"
#include "serial.h"
//#include "vcomdemo.h"
#include "string.h"

#define     EN_IOCON        (1<<16)
#define     EN_USBREG       (1<<14)

extern void SysTick_Enable(uint32_t period);
extern void rflink_decode_commands(const char *buffer, int length);
extern void Serial_Receive (void);
extern void processOOK (void);
extern void UpdateSlotTelegrams (uint8_t intSlot);

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
    	  rflink_decode_commands (serBuf, bufUsed);
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
  int last_ring = 0, cur_ring;

  Chip_USB_Init ();
  Chip_GPIO_Init (NULL);

  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_0, MD_DIGMODE | MD_PUP);
  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_1, MD_DIGMODE | MD_PUP);
  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_2, MD_DIGMODE | MD_PUP);
  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_3, MD_DIGMODE | MD_PUP);
  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_4, MD_PUP);

  Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 0, 7);

  UpdateSlotTelegrams (1);

  USBIOClkConfig();
  SystemCoreClockUpdate ();

  SysTick_Enable(5);
  /* Enable PIO1_5 as capture input to timer0 */
  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_5, IOCON_FUNC2);

  /* Enable TIMER32B0 to count at 1MHz */
  Chip_TIMER_Init (LPC_TIMER32_0);
  Chip_TIMER_Reset (LPC_TIMER32_0);
  Chip_TIMER_PrescaleSet (LPC_TIMER32_0, 72);
  Chip_TIMER_CaptureRisingEdgeEnable (LPC_TIMER32_0, 0);
  Chip_TIMER_CaptureFallingEdgeEnable (LPC_TIMER32_0, 0);
  /* Enable the Timer0 Interrupt */
  NVIC_EnableIRQ(TIMER_32_0_IRQn);

  VCOM_Init();                              // VCOM Initialization

  USB_Init();                               // USB Initialization
  USB_Connect(TRUE);                        // USB Connect

  Chip_TIMER_CaptureEnableInt (LPC_TIMER32_0, 0);
  Chip_TIMER_Enable (LPC_TIMER32_0);

  while (1) {                               // Loop forever
	processOOK ();
    if (USB_Configuration) {
    	/* Switch off D2 Led on board */
    	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 0, 7, false);
    	VCOM_Usb2Serial();

    	if ((cur_ring = ring_message_to_fetch ()) > 0 && last_ring == 0) {
    		last_ring = cur_ring;
    		CDC_BulkIn ();
    	} else {
    		if (cur_ring == 0)
    			last_ring = 0;
    	}
    } else {
		last_ring = 0;
    	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, 0, 7, true);
    }
	Serial_Receive ();
	LPC_PMU->PCON |= 2;
	__WFI();
  } // end while											   
} // end main ()
