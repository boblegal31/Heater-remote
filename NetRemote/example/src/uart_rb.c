/*
 * @brief UART interrupt example with ring buffers
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2014
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "chip.h"
#include "board.h"
#include "arch/lpc_arch.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

#define UART_SELECTION 	LPC_UART3
#define IRQ_SELECTION 	UART3_IRQn
#define HANDLER_NAME 	UART3_IRQHandler

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

static uint32_t lostRxBytes;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void receivedCompleteTelegram (LPC_USART_T *pUART, unsigned char * telegram);

/**
 * @brief	UART 0 interrupt handler using ring buffers
 * @return	Nothing
 */
void HANDLER_NAME(void)
{
	register uint32_t curdate, bytecount;
	static uint8_t telegram_length = 0xFF;
	static uint32_t	lastbytereceiveddate;

	/* Want to handle any errors? Do it here. */
	if ((UART_SELECTION->IER & UART_IER_RBRINT) != 0)
	{
		curdate = SysTick_GetMS ();
		if (curdate < lastbytereceiveddate)
		{
			/* This is the case when systick returns back to 0 */
			lastbytereceiveddate = 0xFFFFFFFF - lastbytereceiveddate;
		}
		if (((curdate - lastbytereceiveddate) > 15))
		{
			if (lastbytereceiveddate != 0)
			{
			/* Switch RX buffer, because a new frame is incoming... */
				lostRxBytes -= RingBuffer_Insert (&rx_buff_used_ring, & rxring_inuse);
				lostRxBytes -= RingBuffer_Pop (&rx_buff_free_ring, & rxring_inuse);
				lostRxBytes += 2;
				RingBuffer_Flush (& rxring_inuse);
				telegram_length = 0xFF;
			}
		}
		lastbytereceiveddate = curdate;
	}
	/* Use default ring buffer handler. Override this with your own
	   code if you need more capability. */

	Chip_UART_IRQRBHandler(UART_SELECTION, &rxring_inuse, &txring);
	bytecount = RingBuffer_GetCount(&rxring_inuse);
	if (telegram_length == 0xFF && bytecount >= 4)
		RingBuffer_GetItem(&rxring_inuse, &telegram_length, 3);
	else
	{
		if (bytecount == telegram_length )
		{
			unsigned char * dataPtr = RingBuffer_GetDataPtr (& rxring_inuse);
			lostRxBytes -= RingBuffer_Insert (&rx_buff_used_ring, & rxring_inuse);
			lostRxBytes -= RingBuffer_Pop (&rx_buff_free_ring, & rxring_inuse);
			lostRxBytes += 2;
			RingBuffer_Flush (& rxring_inuse);
			telegram_length = 0xFF;
			receivedCompleteTelegram (UART_SELECTION, dataPtr);
		}
		else if (bytecount > telegram_length)
		{
			/* We should not reach here, but just in case, to avoid stopping the stream */
			RINGBUFF_T  tmpptr;
			unsigned char tmpbuf[UART_RRB_SIZE];

			RingBuffer_PopMult (& rxring_inuse, tmpbuf, telegram_length);
			receivedCompleteTelegram (UART_SELECTION, tmpbuf);
			if (RingBuffer_Pop (&rx_buff_free_ring, & tmpptr))
			{
				RingBuffer_InsertMult (& tmpptr, tmpbuf, telegram_length);
				lostRxBytes -= RingBuffer_Insert (&rx_buff_used_ring, & tmpptr);
				lostRxBytes += 1;
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

/**
 * @brief	Main UART program body
 * @return	Always returns 1
 */
int Serial_Init(void)
{
	RINGBUFF_T tmp;
	int i;

	Board_UART_Init(UART_SELECTION);

	/* Setup UART for 1200 8E1 */
	Chip_UART_Init(UART_SELECTION);
	Chip_UART_SetBaud(UART_SELECTION, 1200);
	Chip_UART_ConfigData(UART_SELECTION, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT | UART_LCR_PARITY_EN | UART_LCR_PARITY_EVEN));
	Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV0));
	Chip_UART_TXEnable(UART_SELECTION);

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
	Chip_UART_SetupFIFOS(UART_SELECTION, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | 
							UART_FCR_TX_RS | UART_FCR_TRG_LEV0));

	/* Enable receive data and line status interrupt */
	Chip_UART_IntEnable(UART_SELECTION, (UART_IER_RBRINT | UART_IER_RLSINT));

	/* preemption = 1, sub-priority = 1 */
	NVIC_SetPriority(IRQ_SELECTION, 1);
	NVIC_EnableIRQ(IRQ_SELECTION);

	return 1;
}

/**
 * @}
 */
