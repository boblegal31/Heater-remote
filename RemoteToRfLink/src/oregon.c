/*
 * oregon.c
 *
 *  Created on: 27 nov. 2018
 *      Author: francois
 */

#include "chip.h"
#include "lpc_types.h"
#include "timer_13xx.h"
#include "gpio_13xx_2.h"
#include "usb.h"
#include "cdcuser.h"
#include "stdio.h"
#include "stdlib.h"
#include "usb.h"
#include "usbhw.h"
#include "cdcuser.h"

#include "rflink.h"

#define OUTPUT_MESSAGE_LENGTH	256

typedef struct {
	bool		pinStatus;
	uint32_t	timerValue;
} t_CaptureEvent;

static t_CaptureEvent	CaptureEvent;
static bool				GotCaptureEvent;
static uint32_t			NumCaptEvents;
static uint32_t			LastCaptureDate;

extern void SendAmbiantTemp (unsigned short id, unsigned short temp);

void DecodeV1Frame(const unsigned char * frame, unsigned char length);
void DecodeV2V3Frame(const unsigned char * frame, unsigned char length);

void init_version (int version);
bool nextPulse (unsigned short width);
unsigned char* getData (unsigned char * count);

void TIMER32_0_IRQHandler(void)
{
  register uint32_t	captureDate = Chip_TIMER_ReadCapture (LPC_TIMER32_0, 0);
  Chip_TIMER_ClearCapture (LPC_TIMER32_0, 0);

  CaptureEvent.pinStatus = Chip_GPIO_ReadPortBit (LPC_GPIO_PORT, 1, 5);
  CaptureEvent.timerValue = captureDate - LastCaptureDate;
  LastCaptureDate = captureDate;
  GotCaptureEvent = TRUE;
  NumCaptEvents++;

  return;
}

void processOOK (void)
{
	unsigned char length;
	unsigned char * frame;

	if (GotCaptureEvent)
	{
		GotCaptureEvent = FALSE;

		init_version (1);
		if (nextPulse(CaptureEvent.timerValue))
		{
			Chip_TIMER_Reset (LPC_TIMER32_0);
			frame = getData(& length);
			DecodeV1Frame (frame, length);
			Chip_TIMER_Enable (LPC_TIMER32_0);
		}
		init_version (2);
		if (nextPulse(CaptureEvent.timerValue))
		{
			Chip_TIMER_Reset (LPC_TIMER32_0);
			frame = getData(& length);
			DecodeV2V3Frame (frame, length);
			Chip_TIMER_Enable (LPC_TIMER32_0);
		}
		init_version (3);
		if (nextPulse(CaptureEvent.timerValue))
		{
			Chip_TIMER_Reset (LPC_TIMER32_0);
			frame = getData(& length);
			DecodeV2V3Frame (frame, length);
			Chip_TIMER_Enable (LPC_TIMER32_0);
		}
	}
}

static inline unsigned short decode_temp (const unsigned char * data)
{
	register unsigned short temp;

    temp = ((data[1] >> 4) * 100) + ((data[1] & 0x0F) * 10) + ((data[0] >> 4));
    if ((data[2] & 0x0F) >= 8)
    	temp = temp | 0x8000;

    return (temp);
}

static inline unsigned char decode_hum (const unsigned char * data)
{
	register unsigned char hum;

    hum = ((data[1] & 0x0F) * 10) | (data[0] >> 4);

    return (hum);
}
#if 0
static unsigned short decode_bcd1 (const unsigned char * data)
{
	register unsigned short bcd;

	bcd = ((data[1] >> 4) * 100) + ((data[1] & 0x0F) * 10) + ((data[0] >> 4));

    return (bcd);
}

static unsigned short decode_bcd2 (const unsigned char * data)
{
	register unsigned short bcd;

	bcd = ((data[1] >> 4) * 10) + ((data[1] & 0x0F) * 100) + ((data[0] & 0x0F));

    return (bcd);
}

static unsigned short decode_bcd3 (const unsigned char * data)
{
	register unsigned short bcd;

	bcd = ((data[1] >> 4) * 100) + ((data[0] & 0x0F) * 10) + ((data[0] >> 4));

    return (bcd);
}

static unsigned short decode_bcd4 (const unsigned char * data)
{
	register unsigned short bcd;

	bcd = ((data[1] & 0x0F) * 100) + ((data[0] >> 4) * 10) + ((data[0] & 0x0F));

    return (bcd);
}
#endif

unsigned char checksum (unsigned char type, const unsigned char * data, int count, unsigned char check)
{
	register unsigned char calc = 0;
	register int i;

	for (i = 0; i < count; i++)
	   calc += ((data[i] & 0xF0) >> 4) + (data[i] & 0xF);

	switch (type)
	{
		case 1:
		case 4:
			// type 1, add all nibbles, deduct 10
			calc -= 10;
			break;

		case 2:
			 // type 2, add all nibbles up to count, add the 13th nibble , deduct 10
			calc = calc + (data[6] & 0xF) - 10;
			break;

		case 3:
			// type 3, add all nibbles up to count, subtract 10 only use the low 4 bits for the compare
			calc = (calc - 10) & 0xF;
			break;

     }

	if (check == calc)
		return 0;
	else
		return 1;
}

void DecodeV1Frame(const unsigned char * frame, unsigned char length)
{
    int temp = 0;
    int id;
    int sum = frame[0] + frame[1] + frame[2];  // max. value is 0x2FD

    sum = (sum &0xff) + (sum>>8);              // add overflow to low byte
    if (frame[3] != (sum & 0xff) ) {
    	/* Frame CRC error */
    	return;
    }
    id = frame[0] & 0xcf;
    temp = ((frame[2] & 0x0F) * 100)  + ((frame[1] >> 4) * 10) + ((frame[1] & 0x0F));
    if ((frame[2] & 0x20) == 0x20)
    	temp = temp | 0x8000;  // bit 1 set when temp is negative, set highest bit on temp value

    SendAmbiantTemp (id, temp);

    rflink_encode_oregon_thr (id, temp, 0, frame[2] & 0x80);
}

void DecodeV2V3Frame(const unsigned char * frame, unsigned char length)
{
	unsigned short type = (frame[0] << 8) | frame[1];
	unsigned short id;
    int sum;
    unsigned short	temp;
#if 0
	unsigned short  rain, baro, rain_total, wind_dir, wind_speed, average_wind_speed;
    unsigned char	uv, comfort = 0, forecast = 0;
#endif
    unsigned char	hum;

	if(type == 0xea4c || type == 0xca48 || type == 0x0a4d) {
		sum = ((frame[7] & 0x0f) << 4) | (frame[6] >> 4);
        if (checksum(2, frame, 6, sum) != 0) {  // checksum = all nibbles 0-11+13 results is nibbles 15 <<4 + 12
			//Serial.println("CRC Error");
			return;
		}

        temp = decode_temp (& frame[4]);
        id = frame[3] << 8 | frame[2];
        SendAmbiantTemp (id, temp);

        rflink_encode_oregon_thr (id, temp, 0, frame[2] & 0x80);
    } else
		// ==================================================================================
		// 1a2d  Indoor Temp/Hygro: THGN122N, THGN123N, THGR122NX, THGR228N, THGR238, THGR268, THGR122X
		// 1a3d  Outside Temp/Hygro: THGR918, THGRN228NX, THGN500
		// fa28  Indoor Temp/Hygro: THGR810
		// *aac  Outside Temp/Hygro: RTGR328N
		// ca2c  Outside Temp/Hygro: THGR328N
		// fab8  Outside Temp/Hygro: WTGR800
		// TEMP + HUM sensor + BAT + CRC
		// ==================================================================================
		// OSV2 AACC13783419008250AD[RTGR328N,...] Id:78 ,Channel:0 ,temp:19.30 ,hum:20 ,bat:10
		// OSV2 1A2D40C4512170463EE6[THGR228N,...] Id:C4 ,Channel:3 ,temp:21.50 ,hum:67 ,bat:90
		// OSV2 1A2D1072512080E73F2C[THGR228N,...] Id:72 ,Channel:1 ,temp:20.50 ,hum:78 ,bat:90
		// OSV2 1A2D103742197005378E // THGR228N
		// OSV3 FA28A428202290834B46 //
		// OSV3 FA2814A93022304443BE // THGR810
		// OSV2 1A2D1002 02060552A4C
		//      1A3D10D91C273083..
		//      1A3D10D90C284083..
		//      01234567890123456789
		//      0 1 2 3 4 5
		// F+A+2+8+1+4+A+9+3+0+2+2+3+0+4+4=4d-a=43
    	if (type == 0xfa28 || type == 0x1a2d || type == 0x1a3d || (type&0xfff)==0xACC || type == 0xca2c || type == 0xfab8) {

            if (checksum(1, frame, 8, frame[8]) != 0) {  // checksum = all nibbles 0-11+13 results is nibbles 15 <<4 + 12
    			//Serial.println("CRC Error");
    			return;
    		}

            temp = decode_temp (& frame[4]);
            hum = decode_hum (& frame[6]);
            id = frame[1] << 8 | frame[3];

            SendAmbiantTemp (id, temp);

             rflink_encode_oregon_thr (id, temp, hum, frame[2] & 0x80);
#if 0
   	} else
		// ==================================================================================
		// 5a5d  Indoor Temp/Hygro/Baro: Huger - BTHR918
		// 5a6d  Indoor Temp/Hygro/Baro: BTHR918N, BTHR968. BTHG968
		// TEMP + HUM + BARO + FORECAST + BAT
		// NO CRC YET
		// ==================================================================================
		// 5A 6D 00 7A 10 23 30 83 86 31
		// 5+a+6+d+7+a+1+2+3+3+8+3=47 -a=3d  +8=4f +8+6=55
		// 5+a+6+d+7+a+1+2+3+3=3c-a=32
		// 5+a+6+d+7+a+1+2+3+3+0+8+3+8+6=55 -a =4b +3=4e !=1
		// 0  1  2  3  4  5  6  7  8  9
		if (type == 0x5a6d || type == 0x5a5d || type == 0x5d60) {

			temp = decode_temp (& frame[4]);
            hum = decode_hum (& frame[6]);

            //0: normal, 4: comfortable, 8: dry, C: wet
            switch (frame[7] >> 4)
            {
				case 0x00:
					comfort = 0;
					break;
				case 0x04:
					comfort = 1;
					break;
				case 0x08:
					comfort = 2;
					break;
				case 0x0C:
					comfort = 3;
					break;
            }

            baro = (frame[8] + 856);  // max value = 1111 / 0x457

            //2: cloudy, 3: rainy, 6: partly cloudy, C: sunny
            switch (frame[9] >> 4)
            {
				case 0x02:
					forecast = 3;
					break;
				case 0x03:
					forecast = 4;
					break;
				case 0x06:
					forecast = 2;
					break;
				case 0x0C:
					forecast = 1;
					break;
            }

            SendAmbiantTemp (temp);

           	buffer[0] = 'I';
           	itoa ((frame[0] & 0xCF) << 8 | frame[2], & buffer[1], 16);
           	while (buffer[counter++] != 0)
           		;
            buffer[counter++] = 'T';
           	itoa (temp, & buffer[counter], 10);
           	while (buffer[counter++] != 0)
           		;
           	buffer[counter++] = 'H';
           	itoa (hum, & buffer[counter], 10);
           	while (buffer[counter++] != 0)
           		;
           	buffer[counter++] = 'B';
           	itoa (baro, & buffer[counter], 10);
           	while (buffer[counter++] != 0)
           		;
           	buffer[counter++] = 'C';
           	buffer[counter++] = comfort + 0x30;
           	buffer[counter++] = 'F';
           	buffer[counter++] = forecast + 0x30;
       		buffer[counter++] = '\r';
       		buffer[counter++] = '\n';

       		USB_WriteEP (CDC_DEP_IN, (uint8_t *) buffer, counter);
	} else
		// ==================================================================================
		// 2914  Rain Gauge:
		// 2d10  Rain Gauge:
		// 2a1d  Rain Gauge: RGR126, RGR682, RGR918, RGR928, PCR122
		// 2A1D0065502735102063
		// 2+A+1+D+0+0+6+5+5+0+2+7+3+5+1+0+2+0=3e-a=34 != 63
		// 2+A+1+D+0+0+6+5+5+0+2+7+3+5+1+0+2+0+6=44-a=3A
		// ==================================================================================
		if (type == 0x2a1d || type == 0x2d10 || type == 0x2914) { // Rain sensor
              rain = decode_bcd1 (& frame[4]);
              rain_total = ((frame[7] >> 4) * 10) | (frame[6] >> 4);

              sum = snprintf (buffer, OUTPUT_MESSAGE_LENGTH, "RFL 20;%X;Oregon Rain;ID=%X%X;RAIN=%04x;RAINTOT=%04X;BAT=%s;\r\n", RfLinkSeqNumber++, frame[0] & 0xcf, frame[2], rain, rain_total, frame[4] & 0x80?"LOW":"OK");
              USB_WriteEP (CDC_DEP_IN, (uint8_t *) buffer, sum);
	} else
		// ==================================================================================
		// 2a19  Rain Gauge: PCR800
		// RAIN + BAT + CRC
		// ==================================================================================
		// OSV3 2A19048E399393250010
		//      01234567890123456789
		//      0 1 2 3 4 5 6 7 8 9
		// 2+A+1+9+0+4+8+E+3+9+9+3+9+3+2+5=5b-A=51 => 10
		if(type == 0x2a19) { // Rain sensor
            if (checksum(3, frame, 9, frame[9] >> 4) != 0) {  // checksum = all nibbles 0-11+13 results is nibbles 15 <<4 + 12
    			//Serial.println("CRC Error");
    			return;
    		}

            rain = decode_bcd1 (& frame[4]);

            sum = snprintf (buffer, OUTPUT_MESSAGE_LENGTH, "RFL 20;%X;Oregon Rain2;ID=%X%X;RAIN=%04x;BAT=%s;\r\n", RfLinkSeqNumber++, frame[0] & 0xcf, frame[4], rain, ((frame[3] & 0x0F) >= 4)?"LOW":"OK");
            USB_WriteEP (CDC_DEP_IN, (uint8_t *) buffer, sum);
	} else
		// ==================================================================================
		// 1a89  Anemometer: WGR800
		// WIND DIR + SPEED + AV SPEED + CRC
		// ==================================================================================
		// OSV3 1A89048800C026400543
		// OSV3 1A89048800C00431103B
		// OSV3 1a89048848c00000003e W
		// OSV3 1a890488c0c00000003e E
		//      1A89042CB0C047000147
		//      0 1 2 3 4 5 6 7 8 9
		// 1+A+8+9+0+4+8+8+0+0+C+0+0+4+3+1+1+0=45-a=3b
		if (type == 0x1a89) { // Wind sensor
            if (checksum(1, frame, 9, frame[9]) != 0) {  // checksum = all nibbles 0-11+13 results is nibbles 15 <<4 + 12
    			//Serial.println("CRC Error");
    			return;
    		}

            wind_dir = (frame[4] >> 4) & 0x0f;
            wind_speed = decode_bcd2 (& frame[5]);
            average_wind_speed = decode_bcd3 (& frame[7]);

			sum = snprintf (buffer, OUTPUT_MESSAGE_LENGTH, "RFL 20;%X;Oregon Wind;ID=%X%X;WINDIR=%0d;WINSP=%04X;AWINSP=%04X;BAT=%s;\r\n", RfLinkSeqNumber++, frame[0] & 0xcf, frame[2], wind_dir, wind_speed, average_wind_speed, ((frame[3] & 0x0F) >= 4)?"LOW":"OK");
			USB_WriteEP (CDC_DEP_IN, (uint8_t *) buffer, sum);

	} else
		// ==================================================================================
		// 3a0d  Anemometer: Huger-STR918, WGR918
		// 1984  Anemometer:
		// 1994  Anemometer:
		// WIND DIR + SPEED + AV SPEED + BAT + CRC
		// 3A0D006F400800000031
		// ==================================================================================
		if (type == 0x3A0D || type == 0x1984 || type == 0x1994) {
            if (checksum(1, frame, 9, frame[9]) != 0) {  // checksum = all nibbles 0-11+13 results is nibbles 15 <<4 + 12
    			return;
    		}

            wind_dir = (unsigned short)(decode_bcd1 (& frame[4]) / 22.5);
            wind_speed = decode_bcd4 (& frame[6]);
            average_wind_speed = decode_bcd1 (& frame[7]);

			sum = snprintf (buffer, OUTPUT_MESSAGE_LENGTH, "RFL 20;%X;Oregon Wind2;ID=%X%X;WINDIR=%0d;WINSP=%04X;AWINSP=%04X;BAT=%s;\r\n", RfLinkSeqNumber++, frame[0] & 0xcf, frame[2], wind_dir, wind_speed, average_wind_speed, ((frame[3] & 0x0F) >= 4)?"LOW":"OK");
			USB_WriteEP (CDC_DEP_IN, (uint8_t *) buffer, sum);

	} else
		// ==================================================================================
		// ea7c  UV Sensor: UVN128, UV138
		// UV + BAT
		// NO CRC YET
		// ==================================================================================
		if (type == 0xea7c) {
            uv = ((frame[5] & 0x0F) * 10) + (frame[4] >> 4);

  			sum = snprintf (buffer, OUTPUT_MESSAGE_LENGTH, "RFL 20;%X;Oregon UVN128/138;ID=%X%X;UV=%04X;BAT=%s;\r\n", RfLinkSeqNumber++, frame[0] & 0xcf, frame[2], uv, ((frame[3] & 0x0F) >= 4)?"LOW":"OK");
  			USB_WriteEP (CDC_DEP_IN, (uint8_t *) buffer, sum);

	} else
		// ==================================================================================
		// da78  UV Sensor: UVN800
		// UV
		// NO CRC YET
		// ==================================================================================
		if (type == 0xda78) {
            uv=(frame[6] & 0xf0) + (frame[5] &0x0f) ;

			sum = snprintf (buffer, OUTPUT_MESSAGE_LENGTH, "RFL 20;%X;Oregon UVN800;ID=%X%X;UV=%04X;BAT=%s;\r\n", RfLinkSeqNumber++, frame[0] & 0xcf, frame[2], uv, ((frame[3] & 0x0F) >= 4)?"LOW":"OK");
			USB_WriteEP (CDC_DEP_IN, (uint8_t *) buffer, sum);
	} else {
		sum = snprintf (buffer, OUTPUT_MESSAGE_LENGTH, "RFL 20;%X;Oregon Unknown;DEBUG=%X;\r\n", RfLinkSeqNumber++, frame[13]);
		USB_WriteEP (CDC_DEP_IN, (uint8_t *) buffer, sum);
#endif
	}
}
