/*
 * ViessMann.c
 *
 *  Created on: 29 oct. 2018
 *      Author: francois
 */

#include "chip.h"
#include "ViessMann.h"
#include "lwip/opt.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"
#include "string.h"

extern RINGBUFF_T txring;
extern RINGBUFF_T rx_buff_free_ring;
extern RINGBUFF_T rx_buff_used_ring;
extern RINGBUFF_T incomingCommands;
extern uint32_t lostRxBytes;

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 32	/* Send */
#define UART_RRB_SIZE 64	/* Receive */

static uint32_t numTxBytesCmd;
static uint32_t lostPbufPtr;
static uint32_t droppedDuplicate;
static uint8_t txcmdbuff[UART_SRB_SIZE];

struct udp_pcb    * logger_pcb;
ip_addr_t			dst_ip;
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


void receivedCompleteTelegram (LPC_USART_T *pUART, unsigned char * telegram)
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

void Serial_Send (void *arg, struct udp_pcb *pcb, struct pbuf *p, ip_addr_t *addr, u16_t port)
{

	if (p != NULL)
	{
		if (numTxBytesCmd == 0)
		{
			memcpy (txcmdbuff, p->payload, MIN(p->len, sizeof (txcmdbuff)));
			numTxBytesCmd = MIN(p->len, sizeof (txcmdbuff));
		}
		pbuf_free (p);
	}
}

void Serial_Set_Net_Args (void * pcb, void * dstip, int active)
{
	if ((netopened = active) == 1)
	{
		logger_pcb = (struct udp_pcb *) pcb;
		dst_ip.addr = ((ip_addr_t *)dstip)->addr;
		udp_recv(logger_pcb, Serial_Send, NULL);
	}
	else
	{
		udp_recv(logger_pcb, NULL, NULL);
		udp_disconnect (logger_pcb);
	}
}

void Serial_Receive (void)
{

	struct pbuf * pbuffptr;
	int	bytesread;
	RINGBUFF_T tmp;

	if (sendmask != 0 && RingBuffer_IsEmpty(& rx_buff_used_ring) == 0)
	{
#if SENDSTATS
		uint32_t curdate;

		curdate = SysTick_GetMS ();
		if (curdate < lastStatsSent)
		{
			/* This is the case when systick returns back to 0 */
			lastStatsSent = 0xFFFFFFFF - lastStatsSent;
		}
		if ((curdate - lastStatsSent) > 60000)
		{
				ptr = pbuf_alloc(PBUF_TRANSPORT, (u16_t) 16, PBUF_RAM);
				if (ptr == NULL)
					return;
				* (uint32_t *) & ptr->payload[0] = lostRxBytes;
				* (uint32_t *) & ptr->payload[1] = lostPbufPtr;
				* (uint32_t *) & ptr->payload[2] = droppedDuplicate;
				* (uint32_t *) & ptr->payload[3] = curdate;
				ptr->tot_len = ptr->len = 16;

				udp_sendto(logger_pcb, ptr, & dst_ip, 3999)
				pbuf_free (ptr);

				lastStatsSent = curdate;
		}
#endif

		RingBuffer_Pop (& rx_buff_used_ring, & tmp);
		bytesread = RingBuffer_GetCount (& tmp);

		pbuffptr = pbuf_alloc(PBUF_TRANSPORT, (u16_t) bytesread, PBUF_RAM);
		if (pbuffptr == NULL)
		{
			lostPbufPtr++;
			return;
		}

		bytesread = RingBuffer_PopMult(& tmp, pbuffptr->payload, UART_RRB_SIZE);
		RingBuffer_Insert(&rx_buff_free_ring, &tmp);

		if (sendmask & FILTER_DUPLICATE)
		{
			if (memcmp (lasttelegram, pbuffptr->payload, bytesread) != 0)
			{
				/* Only send data if different from previous telgram */
				//ptr->tot_len = ptr->len = bytesread;
				memcpy (lasttelegram, pbuffptr->payload, bytesread);
				if (sendmask & FILTER_PING && * ((uint8_t *) pbuffptr->payload + 2) == 0x00)
				{
					pbuf_free (pbuffptr);
					return;
				}

				udp_sendto(logger_pcb, pbuffptr, & dst_ip, 4000);
				pbuf_free (pbuffptr);

			}
			else
			{
				pbuf_free (pbuffptr);
				droppedDuplicate++;
			}
		}
		else
		{
			if ((sendmask & FILTER_PING && * ((uint8_t *) pbuffptr->payload + 2) == 0x00) ||
				(sendmask & ONLY_STATUSES && (* ((uint8_t *) pbuffptr->payload + 2) != 0xBF ||
											  * ((uint8_t *) pbuffptr->payload + 6) != 0x1D)))
			{
				pbuf_free (pbuffptr);
				return;
			}
			udp_sendto(logger_pcb, pbuffptr, & dst_ip, 4000);
			pbuf_free (pbuffptr);
		}
	}
}

