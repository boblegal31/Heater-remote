/*----------------------------------------------------------------------------
 *      Name:    serial.c
 *      Purpose: serial port handling for LPC134x
 *      Version: V1.10
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
#include "ring_buffer.h"                                   // LPC13xx definitions
#include "chip.h"                                   // LPC13xx definitions
#include "uart_13xx.h"                                   // LPC13xx definitions
#include "ViessMann.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* Transmit and receive ring buffers */
RINGBUFF_T txring;
RINGBUFF_T rx_buff_free_ring;
RINGBUFF_T rx_buff_used_ring;
RINGBUFF_T incomingCommands;

#define NUMRINGS	4
static RINGBUFF_T rxring_inuse;
static RINGBUFF_T rx_ring_list1[NUMRINGS];
static RINGBUFF_T rx_ring_list2[NUMRINGS];

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 32	/* Send */
#define UART_RRB_SIZE 64	/* Receive */

/* Transmit and receive buffers */
static uint8_t rxbuffs[NUMRINGS][UART_RRB_SIZE];
static uint8_t txbuff[UART_SRB_SIZE];
static uint8_t commands[UART_SRB_SIZE];

static uint32_t lostTelegrams;
/* Saved reference period foe standalone mode */
static uint32_t saved_period;

/* Saved total time in mS since timer was enabled */
volatile uint32_t systick_timems;

static unsigned short         ser_lineState;                  // ((msr << 8) | (lsr))

/* Current system clock rate, mainly used for sysTick */
extern uint32_t SystemCoreClock;

extern int receivedCompleteTelegram (void *pUART, unsigned char * telegram);

/**
 * @brief	UART 0 interrupt handler using ring buffers
 * @return	Nothing
 */
/* Enable LWIP tick and interrupt */
void SysTick_Enable(uint32_t period)
{
	/* Initialize System Tick with time interval */
//	SYSTICK_InternalInit(period); // FIXME
//	saved_period = period; // FIXME
//	systick_timems = 0; // FIXME

	/* Enable System Tick interrupt */
//	SYSTICK_IntCmd(ENABLE); // FIXME

	/* Enable System Tick Counter */
//	SYSTICK_Cmd(ENABLE); // FIXME

	saved_period = period;
	SysTick_Config((SystemCoreClock * period) / 1000);
}

/**
 * @brief	SysTick IRQ handler and timebase management
 * @return	Nothing
 * @note	This function keeps a timebase for LWIP that can be
 * used for other functions.
 */
void SysTick_Handler(void)
{
	/* Clear System Tick counter flag */
//	SYSTICK_ClearCounterFlag(); // FIXME

	/* Increment tick count */
	systick_timems += saved_period;
}

void UART_IRQHandler(void)
{
	register uint32_t curdate, bytecount;
	static uint8_t telegram_length = 0xFF;
	static uint32_t	lastbytereceiveddate;
	RINGBUFF_T  tmpptr;

	ser_lineState = LPC_USART->LSR & 0x1E;        // update linestate

	/* Want to handle any errors? Do it here. */
	if ((LPC_USART->IER & UART_IER_RBRINT) != 0)
	{
		curdate = systick_timems;
		if (curdate < lastbytereceiveddate)
		{
			/* This is the case when systick returns back to 0 */
			lastbytereceiveddate = 0xFFFFFFFF - lastbytereceiveddate;
		}
		if (((curdate - lastbytereceiveddate) > 20))
		{
			if (lastbytereceiveddate != 0 && RingBuffer_GetCount (& rxring_inuse) != 0)
			{
				/* Switch RX buffer, because a new frame is incoming... */
				if (RingBuffer_Insert (&rx_buff_used_ring, & rxring_inuse) == 0) {
					RingBuffer_Pop (&rx_buff_used_ring, & tmpptr);
					RingBuffer_Insert (&rx_buff_used_ring, & rxring_inuse);
					memcpy (& rxring_inuse, & tmpptr, sizeof (RINGBUFF_T));
					lostTelegrams++;
				} else {
					RingBuffer_Pop (&rx_buff_free_ring, & rxring_inuse);
				}
				RingBuffer_Flush (& rxring_inuse);
				telegram_length = 0xFF;
			}
		}
		lastbytereceiveddate = curdate;
	}
	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */

	Chip_UART_IRQRBHandler(LPC_USART, &rxring_inuse, &txring);
	bytecount = RingBuffer_GetCount(&rxring_inuse);
	if (telegram_length == 0xFF && bytecount > TELEGRAM_LENGTH_OFFSET)
		RingBuffer_GetItem(&rxring_inuse, &telegram_length, TELEGRAM_LENGTH_OFFSET);
	else
	{
		if (bytecount == telegram_length )
		{
			unsigned char * dataPtr = RingBuffer_GetDataPtr (& rxring_inuse);
			if (receivedCompleteTelegram (LPC_USART, dataPtr)) {
				/* Switch RX buffer, because a new frame is incoming... */
				if (RingBuffer_Insert (&rx_buff_used_ring, & rxring_inuse) == 0) {
					RingBuffer_Pop (&rx_buff_used_ring, & tmpptr);
					RingBuffer_Insert (&rx_buff_used_ring, & rxring_inuse);
					memcpy (& rxring_inuse, & tmpptr, sizeof (RINGBUFF_T));
					lostTelegrams++;
				} else {
					RingBuffer_Pop (&rx_buff_free_ring, & rxring_inuse);
				}
			}
			RingBuffer_Flush (& rxring_inuse);
			telegram_length = 0xFF;
		}
		else if (bytecount > telegram_length)
		{
			/* We should not reach here, but just in case, to avoid stopping the stream */
			unsigned char tmpbuf[UART_RRB_SIZE];

			RingBuffer_PopMult (& rxring_inuse, tmpbuf, telegram_length);
			if (receivedCompleteTelegram (LPC_USART, tmpbuf)) {
				if (RingBuffer_Pop (&rx_buff_free_ring, & tmpptr))
				{
					RingBuffer_InsertMult (& tmpptr, tmpbuf, telegram_length);
					RingBuffer_Insert (&rx_buff_used_ring, & tmpptr);
				}
				else

				{
					lostTelegrams++;
				}
			telegram_length = 0xFF;

		}
		else if (bytecount == UART_RRB_SIZE)
		{
			RingBuffer_Flush (& rxring_inuse);
			lostRxBytes += UART_RRB_SIZE;
		}
	}
}

/*----------------------------------------------------------------------------
  read the line state of the serial port
 *---------------------------------------------------------------------------*/
void ser_LineState (unsigned short *lineState) {

  *lineState = ser_lineState;
  ser_lineState = 0;

}

/* Initialize UART pins */
void Board_UART_Init()
{
	  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, IOCON_FUNC1);
	  Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, IOCON_FUNC1);
}

/**
 * @brief	Main UART program body
 * @return	Always returns 1
 */
int Serial_Init(void)
{
	RINGBUFF_T tmp;
	int i;

	Board_UART_Init();

	/* Setup UART for 1200 8E1 */
	Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 1200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_EN | UART_LCR_PARITY_EVEN));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0));
	Chip_UART_TXEnable(LPC_USART);

	/* Before using the ring buffers, initialize them using the ring
	   buffer init function */
	RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);
	RingBuffer_Init(&rx_buff_free_ring, rx_ring_list1, sizeof (RINGBUFF_T), NUMRINGS);
	RingBuffer_Init(&rx_buff_used_ring, rx_ring_list2, sizeof (RINGBUFF_T), NUMRINGS);
	for (i = 0; i < NUMRINGS; i++)
	{
		RingBuffer_Init(&tmp, rxbuffs[i], 1, UART_RRB_SIZE);
		RingBuffer_Insert(&rx_buff_free_ring, &tmp);
	}
	RingBuffer_Pop (&rx_buff_free_ring, & rxring_inuse);

	RingBuffer_Init(&incomingCommands, commands, 1, UART_SRB_SIZE);

	/* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_RX_RS |
							UART_FCR_TX_RS | UART_FCR_TRG_LEV0));

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(LPC_USART, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(UART0_IRQn, 1);
	NVIC_EnableIRQ(UART0_IRQn);

	return 1;
}

/**
 * @}
 */
