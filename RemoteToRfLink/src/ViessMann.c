/*
 * ViessMann.c
 *
 *  Created on: 29 oct. 2018
 *      Author: francois
 */

#include "ring_buffer.h"
#include "ViessMann.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "usb.h"
#include "usbhw.h"
#include "cdcuser.h"
#include "chip.h"
#include "uart_13xx.h"
#include "rflink.h"

extern RINGBUFF_T txring;
extern RINGBUFF_T rx_buff_free_ring;
extern RINGBUFF_T rx_buff_used_ring;
extern RINGBUFF_T incomingCommands;
extern uint32_t lostRxBytes;
extern volatile uint32_t systick_timems;

/* Transmit and receive ring buffer sizes */
#define UART_SRB_SIZE 32	/* Send */
#define UART_RRB_SIZE 64	/* Receive */

static uint32_t numTxBytesCmd;
#if 0
static uint32_t droppedDuplicate;
#endif
static uint8_t txcmdbuff[UART_SRB_SIZE];
static uint8_t curtemp;
static uint32_t lastReadTempDate;
static uint8_t curIntSlot = 1;
static uint8_t curEcoModeOn;
static uint16_t tempSensorId;
static int numCrcErrors;

uint32_t 			sendmask = 0;
uint8_t 			lasttelegram[UART_RRB_SIZE] = {0,};

#define FILTER_DUPLICATE	0x8
#define FILTER_PING			0x4
#define SEND_STATS			0x2
#define ONLY_STATUSES		0x1
#if SENDSTATS
static uint32_t 	lastStatsSent;
#endif

#define TIME_60SEC				(60 * 1000)
#define TIME_1HOUR				(24 * 60 * 60 * 1000)

#define SECS_IN_A_MINUTE		60
#define SECS_IN_AN_HOUR			(60 * 60)

//static const char lostTempSensor[] = "Temperature sensor lost !\r\n";

static unsigned char pongTelegram[] =					{0x00, 0x11, 0x80, 0x08, 0x01, 0x01, 0x00, 0x00};
static unsigned char sendAdd0Telegram[] =				{0x00, 0x11, 0xB1, 0x0a, 0x01, 0x01, 0x00, 0x02, 0x0, 0x0};
static unsigned char sendIdTelegram[] =					{0x00, 0x11, 0xB3, 0x10, 0x01, 0x01, 0xF8, 0x11, 0xF9, 0x38, 0xFA, 0x00, 0xFB, 0x11, 0x0, 0x0};

static unsigned char sendShutdownTelegram[] =			{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xA6, 0xAB, 0x62, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x0, 0x0};
static unsigned char sendHeatWatTelegram[] =			{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x60, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x0, 0x0};
static unsigned char sendWatonlyTelegram[] =			{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x63, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x0, 0x0};
static unsigned char sendEcoModeOnTelegram[] =			{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x76, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x0, 0x0};
static unsigned char sendEcoModeOffTelegram[] =			{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xA7, 0xAB, 0x77, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x0, 0x0};
static unsigned char sendPartyModeOnTelegram[] =		{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x61, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x0, 0x0};
static unsigned char sendPartyModeOffTelegram[] =		{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x66, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x0, 0x0};

static unsigned char sendNormTempSetPointTelegram[] =	{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x67, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x00};
static unsigned char sendRedTempSetPointTelegram[] =	{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x64, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x00};
static unsigned char sendPartyTemp1SetPointTelegram[] =	{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x65, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x00};
static unsigned char sendPartyTemp2SetPointTelegram[] =	{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0xAA, 0xAB, 0x66, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0x00, 0x00};
static unsigned char sendAmbiantTempTelegram[] =		{0x00, 0x11, 0xBF, 0x0C, 0x01, 0x01, 0x20, 0x00, 0xAA, 0xAA, 0x00, 0x00 };

static unsigned char sendTimeTelegram[] =				{0x00, 0x11, 0xBF, 0x11, 0x01, 0x01, 0x14, 0x00, 0xAA, 0x7e, 0x00, 0x00, 0xAA, 0xAA, 0xAA, 0x00, 0x00};

typedef struct {
	union {
		unsigned char * telegram;
		struct _telegram * theTelegram;
	};
	unsigned int    telegramlength;
} telegram_info;

static telegram_info telegrams[] = {
		{{pongTelegram}, sizeof (pongTelegram)},
		{{sendAdd0Telegram}, sizeof (sendAdd0Telegram)},
		{{sendIdTelegram}, sizeof (sendIdTelegram)},
		{{sendShutdownTelegram}, sizeof (sendShutdownTelegram)},
		{{sendHeatWatTelegram}, sizeof (sendHeatWatTelegram)},
		{{sendWatonlyTelegram}, sizeof (sendWatonlyTelegram)},
		{{sendEcoModeOffTelegram}, sizeof (sendEcoModeOffTelegram)},
		{{sendEcoModeOnTelegram}, sizeof (sendEcoModeOnTelegram)},
        {{sendPartyModeOffTelegram}, sizeof (sendPartyModeOffTelegram)},
		{{sendPartyModeOnTelegram}, sizeof (sendPartyModeOnTelegram)},
        {{sendNormTempSetPointTelegram}, sizeof (sendNormTempSetPointTelegram)},
		{{sendRedTempSetPointTelegram}, sizeof (sendRedTempSetPointTelegram)},
        {{sendPartyTemp1SetPointTelegram}, sizeof (sendPartyTemp1SetPointTelegram)},
		{{sendPartyTemp2SetPointTelegram}, sizeof (sendPartyTemp2SetPointTelegram)},
		{{sendAmbiantTempTelegram}, sizeof (sendAmbiantTempTelegram)},
		{{sendTimeTelegram}, sizeof (sendTimeTelegram)},
};

uint8_t reflect8(uint8_t data)
{
  const uint8_t bits = 8;
  unsigned long reflection = 0x00000000;
  // Reflect the data about the center bit.
  for (uint8_t bit = 0; bit < bits; bit++)
  {
    // If the LSB bit is set, set the reflection of it.
    if ((data & 0x01) != 0)
    {
      reflection |= (unsigned long)(1 << ((bits - 1) - bit));
    }

    data = (uint8_t)(data >> 1);
  }

  return reflection;
}
//-------------------------------------------------------
// Reflects bit in a uint16_t
//-------------------------------------------------------
uint16_t reflect16(uint16_t data)
{
  const uint8_t bits = 16;
  unsigned long reflection = 0x00000000;
  // Reflect the data about the center bit.
  for (uint8_t bit = 0; bit < bits; bit++)
  {
    // If the LSB bit is set, set the reflection of it.
    if ((data & 0x01) != 0)
    {
      reflection |= (unsigned long)(1 << ((bits - 1) - bit));
    }

    data = (uint16_t)(data >> 1);
  }

  return reflection;
}

unsigned int fastCrc(uint8_t data[], uint8_t start, uint16_t length, uint8_t reflectIn, uint8_t reflectOut, uint16_t polynomial, uint16_t xorIn, uint16_t xorOut, uint16_t msbMask, uint16_t mask)
{
  uint16_t crc = xorIn;

  int j;
  uint8_t c;
  unsigned int bit;

  if (length == 0) return crc;

  for (int i = start; i < (start + length); i++)
  {
    c = data[i];

    if (reflectIn != 0)
      c = (uint8_t) reflect8(c);

    j = 0x80;

    while (j > 0)
    {
      bit = (unsigned int)(crc & msbMask);
      crc <<= 1;

      if ((c & j) != 0)
      {
        bit = (unsigned int)(bit ^ msbMask);
      }

      if (bit != 0)
      {
        crc ^= polynomial;
      }

      j >>= 1;
    }
  }

  if (reflectOut != 0)
    crc = (unsigned int)((reflect16((uint16_t) crc) ^ xorOut) & mask);

  return crc;
}


void SetViessmannCRC(uint8_t data_p[], uint16_t length) {
  uint16_t ret = fastCrc(data_p, 0, length - 2, true, true, 0x1021, 0x000, 0x000, 0x8000, 0xffff);
  uint8_t lower = ret & 0xff;
  uint8_t higher = ret >> 8 & 0xff;
  data_p[length - 1] = higher;
  data_p[length - 2] = lower;
}


int receivedCompleteTelegram (void * pUART, unsigned char * telegram)
{
	t_telegram * theTelegram = (t_telegram *) telegram;
	uint32_t curdate;
	uint16_t crc;
	uint16_t * rcvdCrc = (uint16_t *) ((void *) telegram + theTelegram->length - 2);
	static uint32_t	lastsenttempdate;
	static int numPongSent;

	if (theTelegram->length > 2) {
		crc = fastCrc(telegram, 0, theTelegram->length - 2, true, true, 0x1021, 0x000, 0x000, 0x8000, 0xffff);
		if (crc != rcvdCrc)
			numCrcErrors++;
	}

	if (theTelegram->destClass == eCLASS_VITOTROL && theTelegram->intSlot == curIntSlot)
	{
		/* The telegram is for us ! */
		if (theTelegram->command == eCMD_PING)
		{
			/* PING request to us ! Reply with PONG or with command to send */
			if (numPongSent++ < 2)
				Chip_UART_SendRB (pUART, & txring, (void *) pongTelegram, sizeof (pongTelegram));
			else
				{
				int	command = 0;

				if (RingBuffer_Pop (& incomingCommands, & command) != 0)
					{

					if (command == CUSTOM)
						Chip_UART_SendRB (pUART, & txring, (void *) txcmdbuff, numTxBytesCmd);
					else if (command < CUSTOM)
						Chip_UART_SendRB (pUART, & txring, (void *) telegrams[command].telegram, telegrams[command].telegramlength);

					numPongSent = 0;
					}
				else
					{
					curdate = systick_timems;
					if (curdate < lastsenttempdate)
					{
						/* This is the case when systick returns back to 0 */
						lastsenttempdate = 0xFFFFFFFF - lastsenttempdate;
					}
					if ((curdate - lastsenttempdate) > TIME_60SEC)
					{
						Chip_UART_SendRB (pUART, & txring, (void *) sendAmbiantTempTelegram, sizeof (sendAmbiantTempTelegram));
						numPongSent = 0;
						lastsenttempdate = curdate;
					}
					else
					{
						Chip_UART_SendRB (pUART, & txring, (void *) pongTelegram, sizeof (pongTelegram));
					}
				}
			}
		}
		else if (theTelegram->command == eCMD_RDN_REQ &&
			theTelegram->ReqAddress == 0xF8 && theTelegram->ReqLength == 4)
		{
			/* Register as VITOTROL unit on internal slot 2 */
			Chip_UART_SendRB (pUART, & txring, (void *) sendIdTelegram, sizeof (sendIdTelegram));
		}
		else if (theTelegram->command == eCMD_RD1_REQ &&	theTelegram->ReqLength == 0x0)
		{
			/* Register as VITOTROL unit on internal slot 2 */
			Chip_UART_SendRB (pUART, & txring, (void *) sendAdd0Telegram, sizeof (sendAdd0Telegram));
		}
		else if (theTelegram->command == eCMD_WRR_DAT && theTelegram->ReqAddress == (eAddressMasterStatus + curIntSlot))
		{
			//curModeEcoOn =
		}
	}
	if (theTelegram->command == eCMD_WRR_DAT &&
	    (theTelegram->ReqAddress < eAddressMasterStatus || theTelegram->ReqAddress > eAddressCir3Status)) {
		return 0;
	} else {
		return 1;
	}
}

void Serial_Receive (void)
{
	uint8_t	buffer[UART_RRB_SIZE];
	RINGBUFF_T tmp;
#if 0
	int	counter;
	uint8_t	outbuffer[2*UART_RRB_SIZE+6] = {'D','A','T',' ',0,};
	uint32_t curdate = systick_timems;
	register uint8_t * bufptr = buffer;
	register uint8_t * outbufptr = &outbuffer[4];
	register uint8_t tmpchar;
	struct _telegram * theTelegram = (struct _telegram *) buffer;

	if (curdate < lastReadTempDate)
	{
		/* This is the case when systick returns back to 0 */
		lastReadTempDate = 0xFFFFFFFF - lastReadTempDate;
	}
	if (curdate - lastReadTempDate > TIME_1HOUR)
	    USB_WriteEP (CDC_DEP_IN, (unsigned char *) lostTempSensor, sizeof (lostTempSensor));

	while (sendmask != 0 && RingBuffer_IsEmpty(& rx_buff_used_ring) == 0)
	{
		RingBuffer_Pop (& rx_buff_used_ring, & tmp);
		bytesread = RingBuffer_GetCount (& tmp);

		bytesread = RingBuffer_PopMult(& tmp, buffer, UART_RRB_SIZE);
		RingBuffer_Insert(&rx_buff_free_ring, &tmp);

		if (theTelegram->command == eCMD_WRR_DAT && theTelegram->ReqAddress == (eAddressMasterStatus + curIntSlot)) {
			curEcoModeOn = buffer[STATUS_TELEGRAM_MODE_OFFSET] == MODE_BYTE_ECO;
		}
		if (sendmask & FILTER_DUPLICATE)
		{
			if (memcmp (lasttelegram, buffer, bytesread) != 0)
			{
				/* Only send data if different from previous telgram */
				memcpy (lasttelegram, buffer, bytesread);
				if (sendmask & FILTER_PING && theTelegram->command == eCMD_PING)
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
			if ((sendmask & FILTER_PING && theTelegram->command == eCMD_PING) ||
				(sendmask & ONLY_STATUSES && (theTelegram->command != eCMD_WRR_DAT ||
											   (theTelegram->command == eCMD_WRR_DAT &&
				  (theTelegram->ReqAddress < eAddressMasterStatus || theTelegram->ReqAddress > eAddressCir3Status)))))
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
#else
	t_statusRecord * HeaterStatus = (t_statusRecord *) buffer;
	uint8_t tmpval;

	while (USB_Configuration && RingBuffer_IsEmpty(& rx_buff_used_ring) == 0)
	{
		RingBuffer_Pop (& rx_buff_used_ring, & tmp);
		RingBuffer_PopMult(& tmp, buffer, UART_RRB_SIZE);
		RingBuffer_Insert(&rx_buff_free_ring, &tmp);

		if (HeaterStatus->recordNumber == (eAddressMasterStatus + curIntSlot)) {
			curEcoModeOn = buffer[STATUS_TELEGRAM_MODE_OFFSET] == MODE_BYTE_ECO;
		}
		if (HeaterStatus->recordNumber >= eAddressMasterStatus && HeaterStatus->recordNumber <= eAddressCir3Status) {

			rflink_encode_viessmann_temp (HeaterStatus->intSlot, TempInterneChaudiere, VIESSMANN_TEMP_CENTIDEGRE(HeaterStatus->tempChaudiere ^ XOR_WRR_DATA));
			rflink_encode_viessmann_temp (HeaterStatus->intSlot, TempEauChaudeSanitaire, VIESSMANN_TEMP_CENTIDEGRE(HeaterStatus->tempECS ^ XOR_WRR_DATA));
			rflink_encode_viessmann_temp (HeaterStatus->intSlot, TempConsigne, VIESSMANN_TEMP_CENTIDEGRE(HeaterStatus->tempConsigne ^ XOR_WRR_DATA));
			rflink_encode_viessmann_temp (HeaterStatus->intSlot, TempExterieure, VIESSMANN_TEMP_CENTIDEGRE(HeaterStatus->tempExterieure ^ XOR_WRR_DATA));
			rflink_encode_viessmann_temp (HeaterStatus->intSlot, TempDepartCircuit, VIESSMANN_TEMP_CENTIDEGRE(HeaterStatus->tempDepart ^ XOR_WRR_DATA));

			if (HeaterStatus->tbd5 == XOR_WRR_DATA) {
				tmpval = HeaterStatus->mode ^ XOR_WRR_DATA;
				rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusCommandMode, ModeOff, VIESSMANN_IS_MODE(tmpval, MODE_BYTE_OFF));
				rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusCommandMode, ModeEco, VIESSMANN_IS_MODE(tmpval, MODE_BYTE_ECO));
				rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusCommandMode, ModeParty, VIESSMANN_IS_MODE(tmpval, MODE_BYTE_PARTY));
				rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusCommandMode, ModeJour, VIESSMANN_IS_MODE(tmpval, MODE_BYTE_DAY));
				rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusCommandMode, ModeNuit, VIESSMANN_IS_MODE(tmpval, MODE_BYTE_NIGHT));
			}

			tmpval = HeaterStatus->status_chaudiere ^ XOR_WRR_DATA;
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusOrganes, Bruleur, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BURNER_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusOrganes, Pompe_chaudiere, VIESSMANN_IS_BIT_SET(tmpval, STATUS_MAIN_PUMP_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusOrganes, Pompe_bouclage, VIESSMANN_IS_BIT_SET(tmpval, STATUS_WATLOOP_PUMP_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusOrganes, bit0, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT0_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusOrganes, bit1, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT1_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusOrganes, bit3, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT3_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusOrganes, bit4, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT4_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusOrganes, bit5, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT5_ON));

			tmpval = HeaterStatus->status_pompe ^ XOR_WRR_DATA;
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusPompes, Bruleur, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BURNER_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusPompes, Pompe_chaudiere, VIESSMANN_IS_BIT_SET(tmpval, STATUS_MAIN_PUMP_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusPompes, Pompe_bouclage, VIESSMANN_IS_BIT_SET(tmpval, STATUS_WATLOOP_PUMP_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusPompes, bit0, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT0_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusPompes, bit1, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT1_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusPompes, bit3, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT3_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusPompes, bit4, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT4_ON));
			rflink_encode_viessmann_status (HeaterStatus->intSlot, StatusPompes, bit5, VIESSMANN_IS_BIT_SET(tmpval, STATUS_BIT5_ON));

		}
	}
#endif
}

void UpdateSlotTelegrams (uint8_t intSlot)
{
	int counter;
	int nbtelegrams = sizeof (telegrams) / sizeof (telegram_info) - 3;

	if (intSlot <= NUM_VITOTROL_INTSLOTS)
	{
		curIntSlot = intSlot;
		for (counter = 0; counter < nbtelegrams; counter++)
		{
			telegrams[counter].theTelegram->intSlot = intSlot;
			if (telegrams[counter].theTelegram->command == eCMD_WRR_DAT) {
				telegrams[counter].theTelegram->ReqCC = intSlot ^ XOR_WRR_DATA;
			} else if ((void *) telegrams[counter].theTelegram == (void *) sendAmbiantTempTelegram) {
				telegrams[counter].theTelegram->ReqAddress = intSlot + eAddressCir1AmbiantTemp - 1;
			}
			SetViessmannCRC (telegrams[counter].telegram, telegrams[counter].telegramlength);
		}
	}
}

void UpdateCCTelegram (uint8_t cc, uint8_t * message)
{
	struct _telegram * theTelegram = (struct _telegram *) message;
	theTelegram->ReqCC = cc ^ XOR_WRR_DATA;
	SetViessmannCRC (message, theTelegram->length);
}

void UpdateSetPointMessage (uint8_t cc, uint8_t * message, uint16_t temp)
{
	struct _telegram * theTelegram = (struct _telegram *) message;

	theTelegram->ReqCC = cc ^ XOR_WRR_DATA;
	message[SET_POINT_OFFSET] = temp ^ XOR_WRR_DATA;
	SetViessmannCRC (message, theTelegram->length);
}

void UpdateAmbTempMessage (uint16_t temp)
{
	lastReadTempDate = systick_timems;
	curtemp = temp;
	sendAmbiantTempTelegram[AMBIANT_TEMP_OFFSET] = (temp) ^ XOR_WRR_DATA;
	if (temp > 255)
		sendAmbiantTempTelegram[AMBIANT_TEMP_OFFSET + 1] = (temp >> 8) ^ XOR_WRR_DATA;

	SetViessmannCRC (sendAmbiantTempTelegram, sizeof (sendAmbiantTempTelegram));
}

void UpdateTimeMessage (uint8_t heures, uint8_t minutes)
{
	uint16_t	date = minutes + SECS_IN_A_MINUTE + heures * SECS_IN_AN_HOUR + VIESSMAN_TIME_OFFSET;
	struct _telegram * theTelegram = (struct _telegram *) sendTimeTelegram;

	theTelegram->ReqCmdArg1 = date;
	theTelegram->ReqCmdArg2 = date >> 8;
	SetViessmannCRC (sendTimeTelegram, sizeof (sendTimeTelegram));
}

void setIntSlot (uint8_t intslot)
{
    curIntSlot = intslot;
	UpdateSlotTelegrams (intslot);
}

void setTempSensorId (uint16_t id)
{
	tempSensorId = id;
}

void sendModeTelegram (uint8_t telegram, uint8_t circuit, uint8_t status)
{
	uint8_t command = telegram + status;

	UpdateCCTelegram (circuit, telegrams[command].telegram);
	RingBuffer_Insert(&incomingCommands, &command);
}

void sendSetPointTelegram (uint8_t telegram, uint8_t circuit, uint16_t setpoint)
{
	UpdateSetPointMessage (circuit, telegrams[telegram].telegram, setpoint);
	RingBuffer_Insert(&incomingCommands, &telegram);
}

void sendSetTimeTelegram (uint8_t heures, uint8_t minutes)
{
	uint8_t command = TIME;

	UpdateTimeMessage (heures, minutes);
	RingBuffer_Insert(&incomingCommands, &command);
}

#if 0
void ParseCommands(const register char *buffer, int length)
{
	char repbuffer[32] = {'S','T','S',' ',0};
	int count = 7;
	uint16_t temp;
	uint8_t command;
	uint8_t cc = curIntSlot;
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
	} else if (buffer[0] == 'T') {
		repbuffer[4] = 'T';
		count=4;
		itoa(curtemp, &repbuffer[5], 10);
	   	while (repbuffer[count++] != 0)
	   		;
		repbuffer[count++] = '\r';
		repbuffer[count++] = '\n';
	} else if (buffer[0] == 'I') {
		repbuffer[4] = 'I';
		repbuffer[5] = curIntSlot + '0';
		repbuffer[6] = '\r';
		repbuffer[7] = '\n';
		count=8;
	} else if (memcmp(& buffer[4], CONFIG_REQ, sizeof(CONFIG_REQ) - 1) == 0) {
		if ((temp = strtol (& buffer[8], NULL, 10)) != 0)
		{
		    curIntSlot = temp;
			UpdateSlotTelegrams (temp);
		}
		else
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
	} else if (length >= SWITCH_CMD_LENGTH &&
			   (memcmp(buffer, SWITCH_REQ, sizeof(SWITCH_REQ) - 1) == 0)) {
		if (length >= SWITCH_CMD_LENGTH + 2) {
			cc = buffer[8] - '0';
		}
		if (memcmp(& buffer[4], SWITCH_SHUTDOWN, sizeof(SWITCH_SHUTDOWN) - 1) == 0) {
			command = SHUTDOWN;
			UpdateCCTelegram (cc, sendShutdownTelegram);
			if (curEcoModeOn != 0) {
				RingBuffer_Insert(&incomingCommands, &command);
				UpdateCCTelegram (cc, sendEcoModeOffTelegram);
				command = ECOMODEOFF;
			}
		}
		else if (memcmp(& buffer[4], SWITCH_ECS, sizeof(SWITCH_ECS) - 1) == 0) {
			command = ECS_SEUL;
			UpdateCCTelegram (cc, sendWatonlyTelegram);
		} else if (memcmp(& buffer[4], SWITCH_FULL, sizeof(SWITCH_FULL) - 1) == 0) {
			command = ECS_CHF;
			UpdateCCTelegram (cc, sendHeatWatTelegram);
		} else if (memcmp(& buffer[4], SWITCH_ECO, sizeof(SWITCH_ECO) - 1) == 0) {
			if (length > 9) {
				command = ECOMODEOFF;
				UpdateCCTelegram (cc, sendEcoModeOffTelegram);
			} else {
				command = ECOMODEON;
				UpdateCCTelegram (cc, sendEcoModeOnTelegram);
			}
		} else if (memcmp(& buffer[4], SWITCH_PARTY, sizeof(SWITCH_PARTY) - 1) == 0) {
			if ((temp = strtol (& buffer[10], NULL, 10)) != 0){
				UpdateSetPointMessage (cc, sendPartyTemp1SetPointTelegram, temp);
				UpdateSetPointMessage (cc, sendPartyTemp2SetPointTelegram, temp);
				command = PARTYMODEON;
				UpdateCCTelegram (cc, sendPartyModeOnTelegram);
				RingBuffer_Insert(&incomingCommands, &command);
				if (curEcoModeOn != 0) {
					command = ECOMODEOFF;
					UpdateCCTelegram (cc, sendEcoModeOffTelegram);
					RingBuffer_Insert(&incomingCommands, &command);
				}
				command = SETPOINTPARTY1;
				RingBuffer_Insert(&incomingCommands, &command);
				command = SETPOINTPARTY2;
			} else {
				UpdateCCTelegram (cc, sendPartyModeOffTelegram);
				command = PARTYMODEOFF;
			}
		} else if (memcmp(& buffer[4], SWITCH_NORM_TEMP, sizeof(SWITCH_NORM_TEMP) - 1) == 0) {
			if ((temp = strtol (& buffer[8], NULL, 10)) != 0){
				UpdateSetPointMessage (cc, sendNormTempSetPointTelegram, temp);
				command = SETPOINTNORM;
			} else {
				repbuffer[4] = 'E';
				repbuffer[5] = 'R';
				repbuffer[6] = 'R';
				repbuffer[7] = '\r';
				repbuffer[8] = '\n';
				count = 9;
				USB_WriteEP (CDC_DEP_IN, (unsigned char *)repbuffer, count);
				return;
			}
		} else if (memcmp(& buffer[4], SWITCH_REDU_TEMP, sizeof(SWITCH_REDU_TEMP) - 1) == 0) {
			if ((temp = strtol (& buffer[8], NULL, 10)) != 0){
				UpdateSetPointMessage (cc, sendRedTempSetPointTelegram, temp);
				command = SETPOINTREDUCED;
			} else {
				repbuffer[4] = 'E';
				repbuffer[5] = 'R';
				repbuffer[6] = 'R';
				repbuffer[7] = '\r';
				repbuffer[8] = '\n';
				count = 9;
				USB_WriteEP (CDC_DEP_IN, (unsigned char *)repbuffer, count);
				return;
			}
		} else if (memcmp(& buffer[4], SWITCH_AMB_TEMP, sizeof(SWITCH_AMB_TEMP) - 1) == 0) {
			if ((temp = strtol (& buffer[8], NULL, 10)) != 0){
				UpdateAmbTempMessage (temp);
				command = AMBIANTTEMP;
			} else {
				repbuffer[4] = 'E';
				repbuffer[5] = 'R';
				repbuffer[6] = 'R';
				repbuffer[7] = '\r';
				repbuffer[8] = '\n';
				count = 9;
				USB_WriteEP (CDC_DEP_IN, (unsigned char *)repbuffer, count);
				return;
			}
		} else if (memcmp(& buffer[4], SWITCH_TIME, sizeof(SWITCH_TIME) - 1) == 0) {
			uint8_t hour, minutes;
			uint8_t * ptr;
			if ((hour = strtol (& buffer[8], (char **) &ptr, 10)) != 0 && (minutes = strtol ((char *) ptr + 1, (char **) &ptr, 10)) != 0){
				UpdateTimeMessage (hour, minutes);
				command = TIME;
			} else {
				repbuffer[4] = 'E';
				repbuffer[5] = 'R';
				repbuffer[6] = 'R';
				repbuffer[7] = '\r';
				repbuffer[8] = '\n';
				count = 9;
				USB_WriteEP (CDC_DEP_IN, (unsigned char *)repbuffer, count);
				return;
			}
		} else {
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
		struct _telegram * theTelegram = (struct _telegram *) txcmdbuff;
		buffer += 4;
		length = (length - 4) / 2;
		if (length < sizeof (struct _telegram)) {
			repbuffer[4] = 'E';
			repbuffer[5] = 'R';
			repbuffer[6] = 'R';
			repbuffer[7] = '\r';
			repbuffer[8] = '\n';
			count = 9;
		} else {
			/* Custom command is passed through interface without CRC */
			for (numTxBytesCmd = 0; numTxBytesCmd < length; numTxBytesCmd++)
			{
				tmpchar = *buffer > 0x40 ? (*buffer - 0x37) << 4: (*buffer - 0x30) << 4;
				buffer++;
				tmpchar |= *buffer > 0x40 ? (*buffer - 0x37) & 0xf: (*buffer - 0x30) & 0xf;
				buffer++;
				txcmdbuff[numTxBytesCmd] = tmpchar;
			}
			if (numTxBytesCmd < length)
				numTxBytesCmd--;

			/* Calculate command CRC, then add the CRC to byte count */
			SetViessmannCRC (txcmdbuff, theTelegram->length);
			numTxBytesCmd += 2;

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
		}
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
#endif

void SendAmbiantTemp (unsigned short id, unsigned short temp)
{
	uint8_t command;

	if (tempSensorId == 0)
		tempSensorId = id;

	if (id == tempSensorId) {
		UpdateAmbTempMessage (temp);
		command = AMBIANTTEMP;
		RingBuffer_Insert(&incomingCommands, &command);
	}
}
