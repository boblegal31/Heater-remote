/*
 * ViessMann.c
 *
 *  Created on: 29 oct. 2018
 *      Author: francois
 */

#include "ring_buffer.h"
#include "ViessMann.h"
#include "string.h"
#include "usb.h"
#include "usbhw.h"
#include "cdcuser.h"
#include "uart_17xx_40xx.h"

extern RINGBUFF_T txring;
extern RINGBUFF_T rx_buff_free_ring;
extern RINGBUFF_T rx_buff_used_ring;
extern RINGBUFF_T incomingCommands;
extern uint32_t lostRxBytes;

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 32	/* Send */
#define UART_RRB_SIZE 64	/* Receive */

static uint32_t numTxBytesCmd;
static uint32_t droppedDuplicate;
static uint8_t txcmdbuff[UART_SRB_SIZE];

struct udp_pcb    * logger_pcb;
int					netopened = 0;
uint32_t 			sendmask = 0;
uint8_t 			lasttelegram[UART_RRB_SIZE] = {0,};
#define FILTER_DUPLICATE	0x8
#define FILTER_PING			0x4
#define SEND_STATS			0x2
#define ONLY_STATUSES		0x1
#if SENDSTATS
static uint32_t 	lastStatsSent;
#endif


void receivedCompleteTelegram (LPC_UART_TypeDef *pUART, unsigned char * telegram)
{
	t_telegram * theTelegram = (t_telegram *) telegram;
	static int numPongSent;

	if (RingBuffer_IsEmpty (& incomingCommands) == 0)
	{
		if (theTelegram->destClass == eCLASS_VITOTROL && theTelegram->intSlot == 2)
		{
			/* The telegram is for us ! */
			if (theTelegram->command == eCMD_PING)
			{
				/* PING request to us ! Reply with PONG or with command to send */
				if (numPongSent++ < 5)
					Chip_UART_SendRB (pUART, & txring, (void *) pongTelegram, sizeof (pongTelegram));
				else
					{
					int	command = 0;

					RingBuffer_Pop (& incomingCommands, & command);

					switch (command)
						{
						case SHUTDOWN:
							Chip_UART_SendRB (pUART, & txring, (void *) sendShutdownTelegram, sizeof (sendShutdownTelegram));
							break;
						case ECS_SEUL:
							Chip_UART_SendRB (pUART, & txring, (void *) sendWatonlyTelegram, sizeof (sendWatonlyTelegram));
							break;
						case ECS_CHF:
							Chip_UART_SendRB (pUART, & txring, (void *) sendHeatWatTelegram, sizeof (sendHeatWatTelegram));
							break;
						case ECOMODEON:
							Chip_UART_SendRB (pUART, & txring, (void *) sendEcoModeOnTelegram, sizeof (sendEcoModeOnTelegram));
							break;
						case ECOMODEOFF:
							Chip_UART_SendRB (pUART, & txring, (void *) sendEcoModeOffTelegram, sizeof (sendEcoModeOffTelegram));
							break;
						case PARTYMODEON:
							Chip_UART_SendRB (pUART, & txring, (void *) sendPartyModeOnTelegram, sizeof (sendPartyModeOnTelegram));
							break;
						case PARTYMODEOFF:
							Chip_UART_SendRB (pUART, & txring, (void *) sendPartyModeOffTelegram, sizeof (sendPartyModeOffTelegram));
							break;
						case CUSTOM:
							Chip_UART_SendRB (pUART, & txring, (void *) txcmdbuff, numTxBytesCmd);
							break;

						}
					numPongSent = 0;
					}
			}

			if (theTelegram->command == eCMD_RDN_REQ &&
				theTelegram->readNReq.readAddress == 0xF8 && theTelegram->readNReq.readLength == 4)
			{
				/* Register as VITOTROL unit on internal slot 2 */
				Chip_UART_SendRB (pUART, & txring, (void *) sendIdTelegram, sizeof (sendIdTelegram));
			}
			if (theTelegram->command == eCMD_RD1_REQ &&	theTelegram->readNReq.readAddress == 0x0)
			{
				/* Register as VITOTROL unit on internal slot 2 */
				Chip_UART_SendRB (pUART, & txring, (void *) sendAdd0Telegram, sizeof (sendAdd0Telegram));
			}
		}
	}
}

void Serial_Receive (void)
{

	int	bytesread, counter;
	uint8_t	buffer[UART_RRB_SIZE];
	uint8_t	outbuffer[2*UART_RRB_SIZE+6] = {'D','A','T',' ',0,};
	register uint8_t * bufptr = buffer;
	register uint8_t * outbufptr = &outbuffer[4];
	register uint8_t tmpchar;

	RINGBUFF_T tmp;

	if (sendmask != 0 && RingBuffer_IsEmpty(& rx_buff_used_ring) == 0)
	{
		RingBuffer_Pop (& rx_buff_used_ring, & tmp);
		bytesread = RingBuffer_GetCount (& tmp);

		bytesread = RingBuffer_PopMult(& tmp, buffer, UART_RRB_SIZE);
		RingBuffer_Insert(&rx_buff_free_ring, &tmp);

		if (sendmask & FILTER_DUPLICATE)
		{
			if (memcmp (lasttelegram, buffer, bytesread) != 0)
			{
				/* Only send data if different from previous telgram */
				memcpy (lasttelegram, buffer, bytesread);
				if (sendmask & FILTER_PING && buffer[2] == 0x00)
				{
					return;
				}
				for (counter = 0; counter < bytesread; counter++)
				{
					tmpchar = *bufptr  >> 4;
					*outbufptr++ = tmpchar>9? tmpchar + 0x37:tmpchar + 0x30;
					tmpchar = *bufptr++ & 0xf;
					*outbufptr++ = tmpchar>9? tmpchar + 0x37:tmpchar + 0x30;
				}
				*outbufptr++ = '\r';
				*outbufptr = '\n';
			    USB_WriteEP (CDC_DEP_IN, (unsigned char *)outbuffer, 2 * bytesread + 6);

			}
			else
			{
				droppedDuplicate++;
			}
		}
		else
		{
			if ((sendmask & FILTER_PING && buffer[2] == 0x00) ||
				((sendmask & ONLY_STATUSES && (buffer[2] != 0xBF)) ||
						(buffer[6] != 0x1D)))
			{
				return;
			}
			for (counter = 0; counter < bytesread; counter++)
			{
				tmpchar = *bufptr  >> 4;
				*outbufptr++ = tmpchar>9? tmpchar + 0x37:tmpchar + 0x30;
				tmpchar = *bufptr++ & 0xf;
				*outbufptr++ = tmpchar>9? tmpchar + 0x37:tmpchar + 0x30;
			}
			*outbufptr++ = '\r';
			*outbufptr = '\n';
		    USB_WriteEP (CDC_DEP_IN, (unsigned char *)outbuffer, 2 * bytesread + 6);
		}
	}
}

void ParseCommands(const register char *buffer, int length)
{
	char repbuffer[32] = {'S','T','S',' ',0};
	int count = 7;
	uint8_t command;
	register uint8_t tmpchar;

	if (buffer[0] == 0x30 && length >= 2) {
		sendmask |= buffer[1] - 0x30;
		repbuffer[4] = 0x30 | sendmask;
		repbuffer[5] = '\r';
		repbuffer[6] = '\n';
	} else if (buffer[0] == 0x31 && length >= 2) {
		sendmask &= ~ (buffer[1] - 0x30);
		repbuffer[4] = 0x30 | sendmask;
		repbuffer[5] = '\r';
		repbuffer[6] = '\n';
	} else if (length >= SWITCH_CMD_LENGTH &&
			   (memcmp(buffer, SWITCH_REQ, sizeof(SWITCH_REQ) - 1) == 0)) {
		if (memcmp(& buffer[4], SWITCH_SHUTDOWN, sizeof(SWITCH_SHUTDOWN) - 1)
				== 0)
			command = SHUTDOWN;
		else if (memcmp(& buffer[4], SWITCH_ECS, sizeof(SWITCH_ECS) - 1)
				== 0)
			command = ECS_SEUL;
		else if (memcmp(& buffer[4], SWITCH_FULL, sizeof(SWITCH_FULL) - 1)
				== 0)
			command = ECS_CHF;
		else if (memcmp(& buffer[4], SWITCH_ECO, sizeof(SWITCH_ECO) - 1)
				== 0) {
			if (length > 8)
				command = ECOMODEOFF;
			else
				command = ECOMODEON;
		} else if (memcmp(& buffer[4], SWITCH_PARTY,
				sizeof(SWITCH_PARTY) - 1) == 0) {
			if (length > 8)
				command = PARTYMODEOFF;
			else
				command = PARTYMODEON;
		} else
		{
			repbuffer[4] = 'E';
			repbuffer[5] = 'R';
			repbuffer[6] = 'R';
			repbuffer[7] = '\r';
			repbuffer[8] = '\n';
			count = 9;
		    USB_WriteEP (CDC_DEP_IN, (unsigned char *)repbuffer, count);
		    return;
		}
		RingBuffer_Insert(&incomingCommands, &command);
		count = RingBuffer_GetCount(&incomingCommands);
		repbuffer[4] = 'O';
		repbuffer[5] = 'K';
		repbuffer[6] = ' ';
		repbuffer[7] = (count / 10) | '0';
		repbuffer[8] = (count % 10) | '0';
		repbuffer[9] = '\r';
		repbuffer[10] = '\n';
		count = 11;

	} else if (length > sizeof(SWITCH_CUSTOM) &&
			   (memcmp(buffer, SWITCH_CUSTOM, sizeof(SWITCH_CUSTOM) - 1) == 0))
	{
		buffer += 4;
		length = (length - 4) / 2;

		for (numTxBytesCmd = 0; numTxBytesCmd < length; numTxBytesCmd++)
		{
			tmpchar = *buffer > 0x40 ? (*buffer - 0x37) << 4: (*buffer - 0x30) << 4;
			buffer++;
			tmpchar |= *buffer > 0x40 ? (*buffer - 0x37) & 0xf: (*buffer - 0x30) & 0xf;
			buffer++;
			txcmdbuff[numTxBytesCmd] = tmpchar;
		}
		numTxBytesCmd--;

		command = CUSTOM;
		RingBuffer_Insert(&incomingCommands, &command);
		count = RingBuffer_GetCount(&incomingCommands);
		repbuffer[4] = 'O';
		repbuffer[5] = 'K';
		repbuffer[6] = ' ';
		repbuffer[7] = (count / 10) | '0';
		repbuffer[8] = (count % 10) | '0';
		repbuffer[9] = '\r';
		repbuffer[10] = '\n';
		count = 11;

	} else {
		repbuffer[4] = 'E';
		repbuffer[5] = 'R';
		repbuffer[6] = 'R';
		repbuffer[7] = '\r';
		repbuffer[8] = '\n';
		count = 9;
	}
    USB_WriteEP (CDC_DEP_IN, (unsigned char *)repbuffer, count);
}
