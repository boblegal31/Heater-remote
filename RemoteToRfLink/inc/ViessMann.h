/*
 * ViessMann.h
 *
 *  Created on: 29 oct. 2018
 *      Author: francois
 */

#ifndef SRC_VIESSMANN_H_
#define SRC_VIESSMANN_H_

typedef enum {
	eCLASS_VITOTRONIC = 0x0,
	eCLASS_INT_EXTENSION = 0x04,
	eCLASS_VITOTROL = 0x11,
	eCLASS_BROADCAST = 0xFF,
} devClass;

typedef enum {
	 eCMD_PING = 0x00,
	 eCMD_PONG = 0x80,
	 eCMD_RD1_REQ = 0x31,
	 eCMD_RDN_REQ = 0x33,
	 eCMD_RDR_REQ = 0x3F,
	 eCMD_WR1_DAT = 0xB1,
	 eCMD_WRN_DAT = 0xB3,
	 eCMD_WRR_DAT = 0xBF,
} busCommands;

typedef enum {
	eWRR_Command_Mode_Off		= 0x62,
	eWRR_Command_Mode_Heat_Boil	= 0x60,
	eWRR_Command_Mode_Boil		= 0x63,
	eWRR_Command_Mode_Eco_On	= 0x76,
	eWRR_Command_Mode_Eco_Off	= 0x77,
	eWRR_Command_Mode_Party_On	= 0x61,
	eWRR_Command_Mode_Party_Off	= 0x66,

	eWRR_Command_SetPoint_Norm	= 0x67,
	eWRR_Command_SetPoint_Eco	= 0x64,
	eWRR_Command_SetPoint_Party1	= 0x65,
	eWRR_Command_SetPoint_Party2	= 0x66,

	/* Vitotrol envoie, avant de redémarrer, un message pour chaque circuit de chauffage associé quand on change l'association */
	eWRR_Command_Register_CC	= 0x5a,

	eWRR_Command_Set_Time		= 0x7e,
	/* Commande de mise à l'heure. Une heure compte pour 0x100e sur le mot de 16 bits suivant la commande */
	/* Les octets semblent inversé : 0xe10 = 3600 */
	eWRR_Command_Set_Date		= 0x7f,

} writeRecordCommandByte;

typedef enum {
	eAddressMasterStatus = 0x1c,
	eAddressCir1Status = 0x1d,
	eAddressCir2Status = 0x1e,
	eAddressCir3Status = 0x1f,
} reqStatusAddress;

typedef enum {
	eAddressMasterCmd = 0x14,
	eAddressCir1Cmd = 0x15,
	eAddressCir2Cmd = 0x16,
	eAddressCir3Cmd = 0x17,
} reqCmdAddress;

typedef enum {
	eAddressCir1AmbiantTemp = 0x20,
	eAddressCir2AmbiantTemp = 0x21,
	eAddressCir3AmbiantTemp = 0x22,
} reqAmbiantTempAddress;

struct _telegram {
	devClass		destClass;
	devClass		srcClass;
	busCommands		command;
	unsigned char	length;
	unsigned char	intSlot;
	unsigned char	srcSubClass;
	unsigned char	ReqAddress;
	unsigned char	ReqLength;
	unsigned char	ReqCC;
	unsigned char	ReqCommand;
	unsigned char	ReqCmdArg1;
	unsigned char	ReqCmdArg2;
};

#define STATUS_TELEGRAM_MODE_OFFSET		17

/* XOR value used on data in WRR commands */
#define XOR_WRR_DATA			0xAA

/* Number of internal slots. Corresponds to the maximum amount of heating circuits */
#define NUM_VITOTROL_INTSLOTS	3

/* Offset to the set point byte in the Write Record command */
#define TELEGRAM_LENGTH_OFFSET	3

/* Offset to the set point byte in the Write Record command */
#define SET_POINT_OFFSET		10

/* Offset to the ambiant temperature byte in the Write Record command */
#define AMBIANT_TEMP_OFFSET		7

/* Offset to the EcoMode status in status message */
#define STATUS_MSG_MODE_OFFSET	17

/* Offset added to the time */
#define VIESSMAN_TIME_OFFSET	0x4c00

typedef struct _telegram t_telegram;

#define CONFIG_REQ		"CFG"
#define SWITCH_REQ		"SWX"
#define STATUS_OK		"OK!"
#define SWITCH_SHUTDOWN	"OFF"
#define SWITCH_ECS		"ECS"
#define SWITCH_FULL		"ALL"
#define SWITCH_ECO		"ECO"
#define SWITCH_PARTY	"PTY"
#define SWITCH_NORM_TEMP	"NSP"
#define SWITCH_REDU_TEMP	"RSP"
#define SWITCH_AMB_TEMP	"TMP"
#define SWITCH_TIME		"TIM"
#define SWITCH_CUSTOM	"CST"
#define SWITCH_CMD_LENGTH 7

typedef enum {
	SHUTDOWN = 3,
	ECS_CHF,
	ECS_SEUL,
	ECOMODEOFF,
	ECOMODEON,
	PARTYMODEOFF,
	PARTYMODEON,
	SETPOINTNORM,
	SETPOINTREDUCED,
	SETPOINTPARTY1,
	SETPOINTPARTY2,
	AMBIANTTEMP,
	TIME,

	CUSTOM,	/* Keep at last position */
} commandes;

typedef struct _statusRecord {
	unsigned char	destClass;
	unsigned char	srcClass;
	unsigned char	command;
	unsigned char	length;
	unsigned char	intSlot;
	unsigned char	srcSubClass;
	unsigned char	recordNumber;
	unsigned char	status_chaudiere;	/* Bruleur sur LSB et vanne multivoie sur MSB */
	unsigned char	tbd1;
	unsigned char	tempChaudiere;
	unsigned char	tempECS;
	unsigned char	tempConsigne;
	unsigned char	tbd2;
	unsigned char	tempExterieure;
	unsigned char	status_pompe;
	unsigned char	tempDepart;
	unsigned char	tbd5;
	unsigned char	mode;
	unsigned char	tbd6;
	unsigned char	crcLow;
	unsigned char	crcHigh;
} t_statusRecord;

#define MODE_BYTE_NIGHT				0x08	/* was 0A */
#define MODE_BYTE_DAY				0x84	/* was C6 */
#define MODE_BYTE_ECO				0xC6    /* ou 0x0A ? was 86 */
#define MODE_BYTE_PARTY				0x86	/* was 06 */
#define MODE_BYTE_OFF				0x00	/*  */

/* Vue des modes :
 * Circuit 1 veille, Circuit 2 mode nuit (température réduite)
 * mode sur status circuit1 0x00 - variable
 * mode sur status circuit2 0x18
 *
 * Circuit 1 mode nuit (température réduite)Circuit 2 mode nuit (température réduite)
 * mode sur status circuit1 0x08
 * mode sur status circuit2 0x18
 *
 * Circuit 1 veille, Circuit 2 mode veille
 * mode sur status circuit1 0x00
 * mode sur status circuit2 0x10
 *
 * Circuit 1 veille, Circuit 2 mode jour
 * mode sur status circuit1 0x00
 * mode sur status circuit2 0x94
 *
 * Circuit 1 mode jour, Circuit 2 mode jour
 * mode sur status circuit1 0x84
 * mode sur status circuit2 0x94
 *
 * Circuit 1 mode jour/eco, Circuit 2 mode jour
 * mode sur status circuit1 0x84
 * mode sur status circuit2 0x
 *
 * Circuit 1 mode jour, Circuit 2 mode jour/eco
 * mode sur status circuit1 0x
 * mode sur status circuit2 0x
 *
 * Circuit 1 mode jour/eco, Circuit 2 mode jour/eco
 * mode sur status circuit1 0x
 * mode sur status circuit2 0x
 *
 */
#define STATUS_BIT0_ON			0x01
#define STATUS_BIT1_ON			0x02
#define STATUS_BURNER_ON			0x04
#define STATUS_BIT3_ON			0x08
#define STATUS_BIT4_ON			0x10
#define STATUS_BIT5_ON			0x20
#define STATUS_MAIN_PUMP_ON			0x80
#define STATUS_WATLOOP_PUMP_ON		0x40

#define VIESSMAN_TEMP_DEGRE(val)		((val) / 2.0)
#define VIESSMANN_TEMP_CENTIDEGRE(val)	((val) * 5)
#define VIESSMANN_MODE_MASK				0xCF
#define VIESSMANN_IS_MODE(val, mode)		(((val) & VIESSMANN_MODE_MASK) == (mode))
#define VIESSMANN_IS_BIT_SET(val, bit)		(((val) & (bit)) == (bit))

void sendSetPointTelegram (uint8_t telegram, uint8_t circuit, uint16_t setpoint);
void sendModeTelegram (uint8_t telegram, uint8_t circuit, uint8_t status);
void SendAmbiantTemp (unsigned short id, unsigned short temp);
void setTempSensorId (uint16_t id);
void setIntSlot (uint8_t intslot);
void sendSetTimeTelegram (uint8_t heures, uint8_t minutes);

#endif /* SRC_VIESSMANN_H_ */
