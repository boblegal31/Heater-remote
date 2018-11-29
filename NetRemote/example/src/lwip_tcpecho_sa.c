/*
 * @brief LWIP no-RTOS TCP Echo example
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

#include "lwip/init.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/memp.h"
#include "lwip/tcpip.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/timers.h"
#include "netif/etharp.h"

#if LWIP_DHCP
#include "lwip/dhcp.h"
#endif

#include "chip.h"
#include "board.h"
#include "lpc_phy.h"
#include "arch\lpc17xx_40xx_emac.h"
#include "arch\lpc_arch.h"
#include "echo.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/* NETIF data */
static struct netif lpc_netif;

/* Last sector address */
#define START_ADDR_LAST_SECTOR  0x00078000

/* Size of each sector */
#define SECTOR_SIZE             1024

/* LAST SECTOR */
#define IAP_LAST_SECTOR         29

/* Number of bytes to be written to the last sector */
#define IAP_NUM_BYTES_TO_WRITE  256

/* Number elements in array */
#define ARRAY_ELEMENTS          (IAP_NUM_BYTES_TO_WRITE / sizeof(uint32_t))

#define GPIO0_PIN2				2

typedef struct {
	uint32_t	ip;
	uint32_t	netmask;
	uint32_t	gateway;
	uint32_t	checksum;
} configData;

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

int Serial_Init(void);
void Serial_Receive (void);
void logger_init(void);

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* LED0 is used for the link status, on = PHY cable detected */
	/* Initial LED state is off to show an unconnected cable state */
	Board_LED_Set(0, false);

	Chip_GPIO_Init(LPC_GPIO);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO, 0, GPIO0_PIN2);
	
	/* Setup a 1mS sysTick for the primary time base */
	SysTick_Enable(1);
}

static uint32_t checksum (uint32_t * data, uint32_t length)
{
	uint64_t	checksum = 0;

	for (; length > 0; length -= sizeof (uint32_t))
		checksum += *data++;

	checksum = (checksum & 0xFFFFFFFF) + ((checksum >> 32) & 0xFFFFFFFF);
	return ((uint32_t) (checksum & 0xFFFFFFFF) + ((checksum >> 32) & 0xFFFFFFFF));
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

int writeFlashConfigData (uint32_t ip, uint32_t netmask, uint32_t gateway)
{
	uint8_t ret_code = 0;
	configData	theData;

	memcpy (& theData.ip, & ip, sizeof (uint32_t));
	memcpy (& theData.netmask, & netmask, sizeof (uint32_t));
	memcpy (& theData.gateway, & gateway, sizeof (uint32_t));
	theData.checksum = checksum ((uint32_t *)& theData, sizeof (configData) - sizeof (uint32_t));

	__disable_irq();

	ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT("Chip_IAP_PreSectorForReadWrite() failed to execute, return code is: %x\r\n", ret_code);
		return (ret_code);
	}

	/* Erase the last sector */
	ret_code = Chip_IAP_EraseSector(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT("Chip_IAP_EraseSector() failed to execute, return code is: %x\r\n", ret_code);
		return (ret_code);
	}

	/* Prepare to write/erase the last sector */
	ret_code = Chip_IAP_PreSectorForReadWrite(IAP_LAST_SECTOR, IAP_LAST_SECTOR);

	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT("Chip_IAP_PreSectorForReadWrite() failed to execute, return code is: %x\r\n", ret_code);
		return (ret_code);
	}

	/* Write to the last sector */
	ret_code = Chip_IAP_CopyRamToFlash(START_ADDR_LAST_SECTOR, (uint32_t *) & theData, IAP_NUM_BYTES_TO_WRITE);

	/* Error checking */
	if (ret_code != IAP_CMD_SUCCESS) {
		DEBUGOUT("Chip_IAP_CopyRamToFlash() failed to execute, return code is: %x\r\n", ret_code);
		return (ret_code);
	}

	/* Re-enable interrupt mode */
	__enable_irq();

	/* Start the signature generator for the last sector */
	Chip_FMC_ComputeSignatureBlocks(START_ADDR_LAST_SECTOR, (SECTOR_SIZE / 16));

	/* Check for signature geenration completion */
	while (Chip_FMC_IsSignatureBusy()) {}

	/* Get the generated FLASH signature value */
	DEBUGOUT("Generated signature for the last sector is: %x \r\n", Chip_FMC_GetSignature(0));
	return (ret_code);
	
}

/**
 * @brief	main routine for example_lwip_tcpecho_sa_17xx40xx
 * @return	Function should not exit.
 */
int main(void)
{
	uint32_t physts;
	ip_addr_t ipaddr, netmask, gw;
	static int prt_ip = 0;
	configData * flashData = (configData *) START_ADDR_LAST_SECTOR;
	uint32_t flashChecksum;

	prvSetupHardware();

	/* Initialize LWIP */
	lwip_init();

	LWIP_DEBUGF(LWIP_DBG_ON, ("Starting LWIP TCP echo server...\n"));

	/* Static IP assignment */
#if LWIP_DHCP
	IP4_ADDR(&gw, 0, 0, 0, 0);
	IP4_ADDR(&ipaddr, 0, 0, 0, 0);
	IP4_ADDR(&netmask, 0, 0, 0, 0);
#else
	flashChecksum = checksum (flashData, sizeof (configData) - sizeof (uint32_t));

	if ((flashChecksum != flashData->checksum && flashData->checksum != 0xFFFFFFFF) ||
		Chip_GPIO_ReadValue(LPC_GPIO0_BASE, GPIO0_PIN2) == 0)
		{
	    /* Using default IP Address if Port0 Pin2 is grounded (internally pulled high) or
		 * if FLASH config area checksum is invalid */
		IP4_ADDR(&gw, 0, 0, 0, 0);
		IP4_ADDR(&ipaddr, 192, 168, 0, 1);
		IP4_ADDR(&netmask, 255, 255, 255, 0);
		}
	else
		{
		memcpy (&ipaddr, &flashData->ip, sizeof (uint32_t));
		memcpy (&netmask, &flashData->netmask, sizeof (uint32_t));
		memcpy (&gw, &flashData->gateway, sizeof (uint32_t));
		}
#endif

	/* Add netif interface for lpc17xx_8x */
	netif_add(&lpc_netif, &ipaddr, &netmask, &gw, NULL, lpc_enetif_init,
			  ethernet_input);
	netif_set_default(&lpc_netif);
	netif_set_up(&lpc_netif);

#if LWIP_DHCP
	dhcp_start(&lpc_netif);
#endif

	Serial_Init ();

	/* Initialize and start application */
	logger_init();
	echo_init();

	/* This could be done in the sysTick ISR, but may stay in IRQ context
	   too long, so do this stuff with a background loop. */
	while (1) {
		/* Handle packets as part of this loop, not in the IRQ handler */
		lpc_enetif_input(&lpc_netif);

		Serial_Receive ();
		/* lpc_rx_queue will re-qeueu receive buffers. This normally occurs
		   automatically, but in systems were memory is constrained, pbufs
		   may not always be able to get allocated, so this function can be
		   optionally enabled to re-queue receive buffers. */
#if 0
		while (lpc_rx_queue(&lpc_netif)) {}
#endif

		/* Free TX buffers that are done sending */
		lpc_tx_reclaim(&lpc_netif);

		/* LWIP timers - ARP, DHCP, TCP, etc. */
		sys_check_timeouts();

		/* Call the PHY status update state machine once in a while
		   to keep the link status up-to-date */
		physts = lpcPHYStsPoll();

		/* Only check for connection state when the PHY status has changed */
		if (physts & PHY_LINK_CHANGED) {
			if (physts & PHY_LINK_CONNECTED) {
				Board_LED_Set(0, true);
				prt_ip = 0;

				/* Set interface speed and duplex */
				if (physts & PHY_LINK_SPEED100) {
					Chip_ENET_Set100Mbps(LPC_ETHERNET);
					NETIF_INIT_SNMP(&lpc_netif, snmp_ifType_ethernet_csmacd, 100000000);
				}
				else {
					Chip_ENET_Set10Mbps(LPC_ETHERNET);
					NETIF_INIT_SNMP(&lpc_netif, snmp_ifType_ethernet_csmacd, 10000000);
				}
				if (physts & PHY_LINK_FULLDUPLX) {
					Chip_ENET_SetFullDuplex(LPC_ETHERNET);
				}
				else {
					Chip_ENET_SetHalfDuplex(LPC_ETHERNET);
				}

				netif_set_link_up(&lpc_netif);
			}
			else {
				Board_LED_Set(0, false);
				netif_set_link_down(&lpc_netif);
			}

			DEBUGOUT("Link connect status: %d\r\n", ((physts & PHY_LINK_CONNECTED) != 0));
		}

		/* Print IP address info */
		if (!prt_ip) {
			if (lpc_netif.ip_addr.addr) {
				static char tmp_buff[16];
				DEBUGOUT("IP_ADDR    : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *) &lpc_netif.ip_addr, tmp_buff, 16));
				DEBUGOUT("NET_MASK   : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *) &lpc_netif.netmask, tmp_buff, 16));
				DEBUGOUT("GATEWAY_IP : %s\r\n", ipaddr_ntoa_r((const ip_addr_t *) &lpc_netif.gw, tmp_buff, 16));
				prt_ip = 1;
			}
		}
	}

	/* Never returns, for warning only */
	return 0;
}

/**
 * @}
 */
