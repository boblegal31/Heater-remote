#include "lpc_types.h"
#include "stdlib.h"
#include "string.h"
#include "rflink.h"
#include "ring_buffer.h"
#include "ViessMann.h"

#define MSG_CNT_OFFSET		3
#define THR_ID_OFFSET		26
#define THR_TEMP_OFFSET		36
#define THR_HUM_OFFSET  	45
#define THR_BAT_OFFSET		52

#define CMD_PROTOCOL_OFFSET					3
#define VIESSMAN_PROTO_CMD_ID_OFFSET		13
#define VIESSMAN_PROTO_SWITCH_OFFSET		18
#define VIESSMAN_PROTO_STATE_OFFSET			21
#define VIESSMAN_PROTO_ARG_OFFSET			21
#define VIESSMAN_PROTO_ARG_STATE_OFFSET		26

#define	ON_COMMAND	"ON"

//10;Viessmann;[Circuit][Command type];[SwitchNum];[ON/OFF];
//10;Viessmann;[Circuit][Command type];[Party];[Party temp];[ON/OFF];
//10;Viessmann;[Circuit][Command type];[SwitchNum];[Temp Sensor ID];[ON/OFF];

static uint8_t messagecounter;

static char thr_string[] = {'2', '0', ';', 0, 0, ';', 'O', 'r', 'e', 'g', 'o', 'n', ' ', 'T', 'e', 'm', 'p', 'H', 'y', 'g', 'r', 'o', ';',
		  'I', 'D', '=', 0, 0, 0, 0, ';',
		  'T', 'e', 'm', 'p', '=', 0, 0, 0, 0, ';',
		  'H', 'u', 'm', '=', 0, 0, ';',
		  'B', 'A', 'T', '=', 0, 0, 0, ';', 0, 0, 0};

const char viessmann_rflink_proto[] = "viessman";
#if 0
static char viessman_circuit_temp_string[] = {'2', '0', ';', 0, 0, ';', 'V', 'i', 'e', 's', 's', 'm', 'a', 'n', 'n', ';',
											  0, ';',									// Circuit
											  'T', 'e', 'm', 'p', '=', 0, 0, 0, 0, ';',	// Température chaudière
											  'T', 'e', 'm', 'p', '=', 0, 0, 0, 0, ';',	// Température ECS
											  'T', 'e', 'm', 'p', '=', 0, 0, 0, 0, ';',	// Température consigne
											  'T', 'e', 'm', 'p', '=', 0, 0, 0, 0, ';',	// Température extérieure
											  'T', 'e', 'm', 'p', '=', 0, 0, 0, 0, ';',	// Température départ
											  'C', 'm', 'd', '=', 0, 0, 0, 0, 0,   ';',	// PartyOn, EcoOn, DayOn, NightOn, Off
											  'H', 'w', '=', 0, 0, 0, ';', '\r', '\n', 0};			// xVanne ' ' xBruleur
#define VIESSMANN_STS_HEAT_CIRC_OFFSET	16
#define VIESSMANN_STS_HEAT_TEMP_OFFSET	23
#define VIESSMANN_STS_HWAT_TEMP_OFFSET	33
#define VIESSMANN_STS_SETP_TEMP_OFFSET	43
#define VIESSMANN_STS_OUT_TEMP_OFFSET	53
#define VIESSMANN_STS_EGRESS_TEMP_OFFSET	63
#define VIESSMANN_STS_MODE_OFFSET		72
#define VIESSMANN_STS_STATUS_OFFSET		81
#endif

static char viessmann_temp_string[] = {'2', '0', ';', 0, 0, ';', 'V', 'i', 'e', 's', 's', 'm', 'a', 'n', 'n', ';',
		 	 	 	 	 	 	 	  'I', 'D', '=', 0, 0, 0, 0, ';',
									  'T', 'e', 'm', 'p', '=', 0, 0, 0, ';', '\r', '\n'};

static char viessmann_mode_string[] = {'2', '0', ';', 0, 0, ';', 'V', 'i', 'e', 's', 's', 'm', 'a', 'n', 'n', ';',
										'I', 'D', '=', 0, 0, 0, 0, ';',
										'S', 'W', 'I', 'T', 'C', 'H', '=', 0, 0, ';',
										'C', 'M', 'D', '=', 0, 0, 0, ';', '\r', '\n'};

#define VIESSMANN_ID_OFFSET	19
#define VIESSMANN_TEMP_OFFSET	29
#define VIESSMANN_SWITCH_NUM_OFFSET	31
#define VIESSMANN_SWITCH_STS_OFFSET	38

void encode_int16_t (char * buffer, int16_t temp) {
	uint16_t	abstemp = abs(temp);
	uint8_t		msbyte;
	uint8_t		startoffset;

	if (temp < 0)
		msbyte = '8';
	else
		msbyte = '0';

	if (abstemp > 0xFFF) {
		startoffset = 0;
		abstemp |= 0x8000;
	} else if (abstemp > 0xFF) {
		buffer[0] = msbyte;
		startoffset = 1;
	} else if (abstemp > 0xF) {
		buffer[0] = msbyte;
		buffer[1] = '0';
		startoffset = 2;
	} else {
		buffer[0] = msbyte;
		buffer[1] = '0';
		buffer[2] = '0';
		startoffset = 3;
	}
	itoa (abstemp, & buffer[startoffset], 16);
	if (buffer[4] == 0)
		buffer[4] = ';';
}

void encode_uint16_t (char * buffer, uint16_t unsignedval) {
	uint8_t		startoffset;

	if (unsignedval > 0xFFF) {
		startoffset = 0;
	} else if (unsignedval > 0xFF) {
		buffer[0] = '0';
		startoffset = 1;
	} else if (unsignedval > 0xF) {
		buffer[0] = '0';
		buffer[1] = '0';
		startoffset = 2;
	} else {
		buffer[0] = '0';
		buffer[1] = '0';
		buffer[2] = '0';
		startoffset = 3;
	}
	itoa (unsignedval, & buffer[startoffset], 16);
	if (buffer[4] == 0)
		buffer[4] = ';';

}

void encode_temp (char * buffer, uint16_t unsignedval) {
	uint8_t		startoffset;

	if (unsignedval > 0x0FF) {
		startoffset = 0;
	} else if (unsignedval > 0x0F) {
		buffer[0] = '0';
		startoffset = 1;
	} else {
		buffer[0] = '0';
		buffer[1] = '0';
		startoffset = 2;
	}
	itoa (unsignedval, & buffer[startoffset], 16);
	if (buffer[3] == 0)
		buffer[3] = ';';

}

void encode_uint8_t (char * buffer, uint8_t unsignedval) {
	uint8_t		startoffset;

	if (unsignedval > 0xF) {
		startoffset = 0;
	} else {
		buffer[0] = '0';
		startoffset = 1;
	}
	itoa (unsignedval, & buffer[startoffset], 16);
	if (buffer[2] == 0)
		buffer[2] = ';';
}

void encode_humidity (char * buffer, uint8_t hum) {
	if (hum > 9) {
		if (hum > 99)
			hum = 99;
		itoa (hum, & buffer[0], 10);
	} else {
		buffer[0] = '0';
		itoa (hum, & buffer[1], 10);
	}
	if (buffer[2] == 0)
		buffer[2] = ';';
}

int encode_batt_status (char * buffer, const unsigned status) {
   	if (status != 0) {
   		buffer[0] = 'L';
   		buffer[1] = 'O';
   		buffer[2] = 'W';
   		buffer[3] = ';';
   		buffer[4] = '\r';
   		buffer[5] = '\n';
   		return 6;
    } else {
   		buffer[0] = 'O';
   		buffer[1] = 'K';
   		buffer[2] = ';';
   		buffer[3] = '\r';
   		buffer[4] = '\n';
   		return 5;
    }
}

int encode_boolean_status (char * buffer, const unsigned status) {
   	if (status == 0) {
   		buffer[0] = 'O';
   		buffer[1] = 'F';
   		buffer[2] = 'F';
   		buffer[3] = ';';
   		buffer[4] = '\r';
   		buffer[5] = '\n';
   		return 6;
    } else {
   		buffer[0] = 'O';
   		buffer[1] = 'N';
   		buffer[2] = ';';
   		buffer[3] = '\r';
   		buffer[4] = '\n';
   		return 5;
    }
}

void rflink_encode_oregon_thr (uint16_t id, int16_t temp, uint8_t hum, unsigned batstat) {
	int length, queuedbytes, ptr = 0;
	encode_uint8_t (& thr_string[MSG_CNT_OFFSET], messagecounter++);
    encode_uint16_t (& thr_string[THR_ID_OFFSET], id);
    encode_int16_t (& thr_string[THR_TEMP_OFFSET], temp);
    encode_humidity (& thr_string[THR_HUM_OFFSET], hum);
    length = THR_BAT_OFFSET + encode_batt_status (& thr_string[THR_BAT_OFFSET], batstat);
    while (length > 0) {
    	queuedbytes = ring_queue(& thr_string[ptr], length);
    	length -= queuedbytes;
    	ptr += queuedbytes;
    }
}

void rflink_encode_viessmann_temp (uint8_t circuit, uint8_t temp_id, uint16_t temp) {
	int length, queuedbytes, ptr = 0;
	encode_uint8_t (& viessmann_temp_string[MSG_CNT_OFFSET], messagecounter++);
    encode_uint16_t (& viessmann_temp_string[VIESSMANN_ID_OFFSET], circuit << 8 | temp_id);
    encode_temp (& viessmann_temp_string[VIESSMANN_TEMP_OFFSET], temp);
    length = sizeof (viessmann_temp_string);
    while (length > 0) {
    	queuedbytes = ring_queue(& viessmann_temp_string[ptr], length);
    	length -= queuedbytes;
    	ptr += queuedbytes;
    }
}

void rflink_encode_viessmann_status (uint8_t circuit, uint8_t status_type, uint8_t status_id, uint8_t value) {
	int length, queuedbytes, ptr = 0;
	encode_uint8_t (& viessmann_mode_string[MSG_CNT_OFFSET], messagecounter++);
    encode_uint16_t (& viessmann_mode_string[VIESSMANN_ID_OFFSET], circuit << 8 | status_type);
    encode_uint8_t (& viessmann_mode_string[VIESSMANN_SWITCH_NUM_OFFSET], status_id);
	length = VIESSMANN_SWITCH_STS_OFFSET + encode_boolean_status (& viessmann_mode_string[VIESSMANN_SWITCH_STS_OFFSET], value);
    while (length > 0) {
    	queuedbytes = ring_queue(& viessmann_mode_string[ptr], length);
    	length -= queuedbytes;
    	ptr += queuedbytes;
    }
}

#if 0
int rflink_encode_viess_sts_temp (char ** buffer, t_ViessTempSts * TempSts) {
	* buffer = viessman_circuit_temp_string;
	encode_uint8_t (& viessman_circuit_temp_string[MSG_CNT_OFFSET], messagecounter++);
	viessman_circuit_temp_string[VIESSMANN_STS_HEAT_CIRC_OFFSET] = TempSts->circuit + '0';
    encode_int16_t (& viessman_circuit_temp_string[VIESSMANN_STS_HEAT_TEMP_OFFSET], TempSts->tempChaudiere);
    encode_int16_t (& viessman_circuit_temp_string[VIESSMANN_STS_HWAT_TEMP_OFFSET], TempSts->tempECS);
    encode_int16_t (& viessman_circuit_temp_string[VIESSMANN_STS_SETP_TEMP_OFFSET], TempSts->tempConsigne);
    encode_int16_t (& viessman_circuit_temp_string[VIESSMANN_STS_OUT_TEMP_OFFSET], TempSts->tempExt);
    encode_int16_t (& viessman_circuit_temp_string[VIESSMANN_STS_EGRESS_TEMP_OFFSET], TempSts->tempDepart);
	viessman_circuit_temp_string[VIESSMANN_STS_MODE_OFFSET] = (TempSts->flags == MODE_BYTE_PARTY) + '0';
	viessman_circuit_temp_string[VIESSMANN_STS_MODE_OFFSET + 1] = (TempSts->flags == MODE_BYTE_ECO) + '0';
	viessman_circuit_temp_string[VIESSMANN_STS_MODE_OFFSET + 2] = (TempSts->flags == MODE_BYTE_DAY) + '0';
	viessman_circuit_temp_string[VIESSMANN_STS_MODE_OFFSET + 3] = (TempSts->flags == MODE_BYTE_NIGHT) + '0';
	viessman_circuit_temp_string[VIESSMANN_STS_MODE_OFFSET + 4] = (TempSts->flags == MODE_BYTE_OFF) + '0';
	viessman_circuit_temp_string[VIESSMANN_STS_STATUS_OFFSET] = ((TempSts->st_organes & STATUS_BURNER_ON) == STATUS_BURNER_ON) + '0';
	viessman_circuit_temp_string[VIESSMANN_STS_STATUS_OFFSET + 1] = ((TempSts->st_organes & STATUS_MAIN_PUMP_ON) == STATUS_MAIN_PUMP_ON) + '0';
	viessman_circuit_temp_string[VIESSMANN_STS_STATUS_OFFSET + 2] = ((TempSts->st_organes & STATUS_WATLOOP_PUMP_ON) == STATUS_WATLOOP_PUMP_ON) + '0';
	return sizeof (viessman_circuit_temp_string) - 1;
}
#endif

void rflink_decode_commands (const char * buffer, int length) {
	uint16_t tmpval;
	uint8_t circuit;
	uint8_t command;
	uint8_t type;
	uint8_t telegram;
	int status;
	uint16_t argument;
	char * statusptr;

	if (length < (CMD_PROTOCOL_OFFSET +  sizeof (viessmann_rflink_proto)) || strncasecmp (viessmann_rflink_proto, & buffer[CMD_PROTOCOL_OFFSET], sizeof (viessmann_rflink_proto) - 1) != 0)
		return;

	tmpval = strtol (& buffer[VIESSMAN_PROTO_CMD_ID_OFFSET] , NULL, 16);
	circuit = tmpval >> 8;
	type = tmpval & 0xff;
	command = strtol (& buffer[VIESSMAN_PROTO_SWITCH_OFFSET], NULL, 16);

	if (type == StatusCommandMode) {
		switch (command) {
			case ModeOff:
				telegram = SHUTDOWN;
				status = (strncasecmp (ON_COMMAND, & buffer[VIESSMAN_PROTO_STATE_OFFSET], sizeof(ON_COMMAND) - 1) != 0);
				sendModeTelegram (telegram, circuit, status);
				break;
			case ModeEco:
				telegram = ECOMODEOFF;
				status = (strncasecmp (ON_COMMAND, & buffer[VIESSMAN_PROTO_STATE_OFFSET], sizeof(ON_COMMAND) - 1) == 0);
				sendModeTelegram (telegram, circuit, status);
				break;

			case ConsigneNormal:
				telegram = SETPOINTNORM;
				argument = strtol (& buffer[VIESSMAN_PROTO_ARG_OFFSET], NULL, 16);
				sendSetPointTelegram (telegram, circuit, argument);
				break;
			case ConsigneReduit:
				telegram = SETPOINTREDUCED;
				argument = strtol (& buffer[VIESSMAN_PROTO_ARG_OFFSET], NULL, 16);
				sendSetPointTelegram (telegram, circuit, argument);
				break;

			case ModeParty:
				telegram = PARTYMODEOFF;
				statusptr = strchr (& buffer[VIESSMAN_PROTO_ARG_OFFSET], ';');
				if (statusptr != NULL) {
					statusptr++;
				} else {
					return;
				}
				argument = strtol (& buffer[VIESSMAN_PROTO_ARG_OFFSET], NULL, 16);
				status = (strncasecmp (ON_COMMAND, statusptr, sizeof(ON_COMMAND) - 1) == 0);
				sendModeTelegram (telegram, circuit, status);
				sendSetPointTelegram (SETPOINTPARTY1, circuit, argument);
				sendSetPointTelegram (SETPOINTPARTY2, circuit, argument);
				break;

			case AjusterHeure:
				argument = strtol (& buffer[VIESSMAN_PROTO_ARG_OFFSET], NULL, 16);
				sendSetTimeTelegram(argument >> 8, argument & 0xff);
				break;
		}
	} else if (type == ConfigureInterface) {
		switch (command) {
			case SetCircuitNumber:
				argument = strtol (& buffer[VIESSMAN_PROTO_ARG_OFFSET], NULL, 16);
				setIntSlot (argument);
				break;

			case SetTempSensorId:
				argument = strtol (& buffer[VIESSMAN_PROTO_ARG_OFFSET], NULL, 16);
				setTempSensorId (argument);
				break;
		}
	}

}
