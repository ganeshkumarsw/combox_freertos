/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : ethernetif.c
 * Description        : This file provides code for the configuration
 *                      of the ethernetif.c MiddleWare.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "lwip/opt.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "netif/etharp.h"
#include "lwip/ethip6.h"
#include "ethernetif.h"
#include <string.h>
#include "cmsis_os.h"
#include "lwip/tcpip.h"
#include "lwip/snmp.h"

/* Within 'USER CODE' section, code will be kept by default at each generation */
/* USER CODE BEGIN 0 */
#include "spi_eth.h"
/* USER CODE END 0 */

/* Private define ------------------------------------------------------------*/
/* The time to block waiting for input. */
#define TIME_WAITING_FOR_INPUT (portMAX_DELAY)
/* USER CODE BEGIN OS_THREAD_STACK_SIZE_WITH_RTOS */
/* Stack size of the interface thread */
#define INTERFACE_THREAD_STACK_SIZE (350)
/* USER CODE END OS_THREAD_STACK_SIZE_WITH_RTOS */
/* Network interface name */
#define IFNAME0 's'
#define IFNAME1 't'

/* ETH Setting  */

/* USER CODE BEGIN 1 */
#define ETH_MAX_PAYLOAD (1600)
#define ETH_MAX_PACKET_SIZE ETH_MAX_PAYLOAD
#undef ETH_RX_BUF_SIZE
#define ETH_RX_BUF_SIZE ETH_MAX_PAYLOAD
#define LINK_SPEED_OF_YOUR_NETIF_IN_BPS (100000000UL)
/* USER CODE END 1 */

/* Private variables ---------------------------------------------------------*/
/*
@Note: This interface is implemented to operate in zero-copy mode only:
        - Rx buffers will be allocated from LwIP stack memory heap,
          then passed to ETH HAL driver.
        - Tx buffers will be allocated from LwIP stack memory heap,
          then passed to ETH HAL driver.

@Notes:
  1.a. ETH DMA Rx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_RX_DESC_CNT in ETH GUI (Rx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h
  1.b. ETH DMA Tx descriptors must be contiguous, the default count is 4,
       to customize it please redefine ETH_TX_DESC_CNT in ETH GUI (Tx Descriptor Length)
       so that updated value will be generated in stm32xxxx_hal_conf.h

  2.a. Rx Buffers number must be between ETH_RX_DESC_CNT and 2*ETH_RX_DESC_CNT
  2.b. Rx Buffers must have the same size: ETH_RX_BUF_SIZE, this value must
       passed to ETH DMA in the init field (heth.Init.RxBuffLen)
  2.c  The RX Ruffers addresses and sizes must be properly defined to be aligned
       to L1-CACHE line size (32 bytes).
*/

/* Data Type Definitions */
typedef enum
{
  RX_ALLOC_OK = 0x00,
  RX_ALLOC_ERROR = 0x01
} RxAllocStatusTypeDef;

typedef struct
{
  struct pbuf_custom pbuf_custom;
  uint8_t buff[(ETH_RX_BUF_SIZE + 31) & ~31] __ALIGNED(32);
} RxBuff_t;

/* Memory Pool Declaration */
//#define ETH_RX_BUFFER_CNT 20U
//LWIP_MEMPOOL_DECLARE(RX_POOL, ETH_RX_BUFFER_CNT, sizeof(RxBuff_t), "Zero-copy RX PBUF pool");

/* Variable Definitions */
//static uint8_t RxAllocStatus;

/* USER CODE BEGIN 2 */

// start offset of next packet
static unsigned short next_pk_ptr;
unsigned char hw_enable;

/* USER CODE END 2 */

osSemaphoreId RxPktSemaphore = NULL; /* Semaphore to signal incoming packets */
osSemaphoreId TxPktSemaphore = NULL; /* Semaphore to signal transmit packet complete */

/* Global Ethernet handle */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

/* Private functions ---------------------------------------------------------*/
void pbuf_free_custom(struct pbuf *p);

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*******************************************************************************
                       LL Driver Interface ( LwIP stack --> ETH)
*******************************************************************************/
/**
 * @brief In this function, the hardware should be initialized.
 * Called from ethernetif_init().
 *
 * @param netif the already initialized lwip network interface structure
 *        for this ethernetif
 */
static void low_level_init(struct netif *netif)
{

  /* Start ETH HAL Init */

  uint8_t MACAddr[6];

  /* Reset and initialize the chip */
  enc424j600_hw_init();
  /* Update the MAC address to the one on the chip */
  enc424j600_get_hw_macaddr(MACAddr);
  /* Enable interrupts */
  enc424j600_hw_enable();

  /* Display in the console the MAC address */
  Console_DEBUG("MAC = %02x.%02x.%02x.%02x.%02x.%02x", MACAddr[0], MACAddr[1], MACAddr[2], MACAddr[3], MACAddr[4], MACAddr[5]);

  /* End ETH HAL Init */

  /* Initialize the RX POOL */
//  LWIP_MEMPOOL_INIT(RX_POOL);

#if LWIP_ARP || LWIP_ETHERNET

  /* set MAC hardware address length */
  netif->hwaddr_len = ETH_HWADDR_LEN;

  /* set MAC hardware address */
  netif->hwaddr[0] = MACAddr[0];
  netif->hwaddr[1] = MACAddr[1];
  netif->hwaddr[2] = MACAddr[2];
  netif->hwaddr[3] = MACAddr[3];
  netif->hwaddr[4] = MACAddr[4];
  netif->hwaddr[5] = MACAddr[5];

  /* maximum transfer unit */
  netif->mtu = ETH_MAX_PAYLOAD;

/* Accept broadcast address and ARP traffic */
/* don't set NETIF_FLAG_ETHARP if this device is not an ethernet one */
#if LWIP_ARP
  netif->flags |= NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET;
  MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);
#else
  netif->flags |= NETIF_FLAG_BROADCAST;
#endif /* LWIP_ARP */

  /* create a binary semaphore used for informing ethernetif of frame reception */
  RxPktSemaphore = osSemaphoreNew(1, 1, NULL);

  /* create a binary semaphore used for informing ethernetif of frame transmission */
  TxPktSemaphore = osSemaphoreNew(1, 1, NULL);

#endif /* LWIP_ARP || LWIP_ETHERNET */
  netif_set_up(netif);
  netif_set_link_up(netif);
  /* USER CODE BEGIN LOW_LEVEL_INIT */

  /* USER CODE END LOW_LEVEL_INIT */
}

/**
 * @brief This function should do the actual transmission of the packet. The packet is
 * contained in the pbuf that is passed to the function. This pbuf
 * might be chained.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @param p the MAC packet to send (e.g. IP packet including MAC addresses and type)
 * @return ERR_OK if the packet could be sent
 *         an err_t value if the packet couldn't be sent
 *
 * @note Returning ERR_MEM here if a DMA queue of your MAC is full can lead to
 *       strange results. You might consider waiting for space in the DMA queue
 *       to become available since the stack doesn't retry to send a packet
 *       dropped because of memory failure (except for the TCP timers).
 */

static err_t low_level_output(struct netif *netif, struct pbuf *p)
{
  struct pbuf *q = NULL;
  err_t errval = ERR_OK;

  //  APP_INFO("[enc_424] low_level_output")
  if (p != NULL)
  {
	Console_DEBUG("[enc_424] Pkt Tx len %d", p->tot_len);
  }

  for (q = p; q != NULL; q = q->next)
  {
    if (q->len > ETH_MAX_PACKET_SIZE)
    {
      errval = ERR_ARG;
    }

    /* Send buffer with Ethernet Chip ENC424  */
    enc424j600_hw_tx((unsigned char *)q->payload, q->len); /* send data */
  }

  if (p != NULL)
  {
	Console_DEBUG("[enc_424] Pkt Tx len %d", p->tot_len);
    /* Update SNMP stats (only if you use SNMP) */
    LINK_STATS_INC(link.recv);
    MIB2_STATS_NETIF_ADD(netif, ifinoctets, p->tot_len);
    if (((u8_t *)p->payload)[0] & 1)
    {
      MIB2_STATS_NETIF_INC(netif, ifinucastpkts);
    }
    else
    {
      MIB2_STATS_NETIF_INC(netif, ifinnucastpkts);
    }
  }
  else
  {
    errval = ERR_ARG;
    ;
  }

  pbuf_ref(p);
  pbuf_free(p);


  // while (osSemaphoreAcquire(TxPktSemaphore, TIME_WAITING_FOR_INPUT) != osOK)

  // {
  // }

  return errval;
}

/**
 * @brief Should allocate a pbuf and transfer the bytes of the incoming
 * packet from the interface into the pbuf.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return a pbuf filled with the received packet (including MAC header)
 *         NULL on memory error
 */
static struct pbuf *low_level_input(struct netif *netif)
{
//  uint8_t *buff = NULL;
  struct pbuf *p_pbuf = NULL;
//  struct pbuf_custom *p = LWIP_MEMPOOL_ALLOC(RX_POOL);
//
//  if (p)
//  {
    /* Get the buff from the struct pbuf address. */
//    buff = (uint8_t *)p + offsetof(RxBuff_t, buff);
//    p->custom_free_function = pbuf_free_custom;
    /* Initialize the struct pbuf.
     * This must be performed whenever a buffer's allocated because it may be
     * changed by lwIP or the app, e.g., pbuf_free decrements ref. */
//    p_pbuf = pbuf_alloced_custom(PBUF_RAW, 0, PBUF_REF, p, buff, ETH_RX_BUF_SIZE);

    p_pbuf = pbuf_alloc(PBUF_RAW, ETH_RX_BUF_SIZE, PBUF_POOL);
    if(p_pbuf)
    {
    	int len = enc_424j600_rcv_process(p_pbuf->payload);

		if (len <= 0)
		{
		  pbuf_free(p_pbuf);
		  p_pbuf = NULL;
		}
		else
		{
		  p_pbuf->len = p_pbuf->tot_len = len;
		  if(p_pbuf->len > 1000)
		  {
			  Console_DEBUG("[enc_424] Pkt Rx len %d", len);
		  }
		}
	  }

  return p_pbuf;
}

/**
 * @brief This function should be called when a packet is ready to be read
 * from the interface. It uses the function low_level_input() that
 * should handle the actual reception of bytes from the network
 * interface. Then the type of the received packet is determined and
 * the appropriate input function is called.
 *
 * @param netif the lwip network interface structure for this ethernetif
 */
void ethernetif_input(void *argument)
{
  struct pbuf *p = NULL;
  struct netif *netif = (struct netif *)argument;

  for (;;)
  {
    // if (osSemaphoreAcquire(RxPktSemaphore, TIME_WAITING_FOR_INPUT) == osOK)
    {
      do
      {
        p = low_level_input(netif);
        if (p != NULL)
        {
          if (netif->input(p, netif) != ERR_OK)
          {
            pbuf_free(p);
          }
        }
      } while (p != NULL);
      osDelay(1);
    }
  }
}

#if !LWIP_ARP
/**
 * This function has to be completed by user in case of ARP OFF.
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if ...
 */
static err_t low_level_output_arp_off(struct netif *netif, struct pbuf *q, const ip4_addr_t *ipaddr)
{
  err_t errval;
  errval = ERR_OK;

  /* USER CODE BEGIN 5 */

  /* USER CODE END 5 */

  return errval;
}
#endif /* LWIP_ARP */

/**
 * @brief Should be called at the beginning of the program to set up the
 * network interface. It calls the function low_level_init() to do the
 * actual setup of the hardware.
 *
 * This function should be passed as a parameter to netif_add().
 *
 * @param netif the lwip network interface structure for this ethernetif
 * @return ERR_OK if the loopif is initialized
 *         ERR_MEM if private data couldn't be allocated
 *         any other err_t on error
 */
err_t ethernetif_init(struct netif *netif)
{
  LWIP_ASSERT("netif != NULL", (netif != NULL));

#if LWIP_NETIF_HOSTNAME
  /* Initialize interface hostname */
  netif->hostname = "lwip";
#endif /* LWIP_NETIF_HOSTNAME */

  /*
   * Initialize the snmp variables and counters inside the struct netif.
   * The last argument should be replaced with your link speed, in units
   * of bits per second.
   */
  // MIB2_INIT_NETIF(netif, snmp_ifType_ethernet_csmacd, LINK_SPEED_OF_YOUR_NETIF_IN_BPS);

  netif->name[0] = IFNAME0;
  netif->name[1] = IFNAME1;
  /* We directly use etharp_output() here to save a function call.
   * You can instead declare your own function an call etharp_output()
   * from it if you have to do some checks before sending (e.g. if link
   * is available...) */

#if LWIP_IPV4
#if LWIP_ARP || LWIP_ETHERNET
#if LWIP_ARP
  netif->output = etharp_output;
#else
  /* The user should write its own code in low_level_output_arp_off function */
  netif->output = low_level_output_arp_off;
#endif /* LWIP_ARP */
#endif /* LWIP_ARP || LWIP_ETHERNET */
#endif /* LWIP_IPV4 */

#if LWIP_IPV6
  netif->output_ip6 = ethip6_output;
#endif /* LWIP_IPV6 */

  netif->linkoutput = low_level_output;

  /* initialize the hardware */
  low_level_init(netif);

  return ERR_OK;
}

/**
 * @brief  Custom Rx pbuf free callback
 * @param  pbuf: pbuf to be freed
 * @retval None
 */
//void pbuf_free_custom(struct pbuf *p)
//{
//  struct pbuf_custom *custom_pbuf = (struct pbuf_custom *)p;
//  LWIP_MEMPOOL_FREE(RX_POOL, custom_pbuf);
//
//  /* If the Rx Buffer Pool was exhausted, signal the ethernetif_input task to
//   * call HAL_ETH_GetRxDataBuffer to rebuild the Rx descriptors. */
//
//  if (RxAllocStatus == RX_ALLOC_ERROR)
//  {
//    RxAllocStatus = RX_ALLOC_OK;
//    osSemaphoreRelease(RxPktSemaphore);
//  }
//}

/* USER CODE BEGIN 6 */

/**
 * @brief  Returns the current time in milliseconds
 *         when LWIP_TIMERS == 1 and NO_SYS == 1
 * @param  None
 * @retval Current Time value
 */
u32_t sys_now(void)
{
  return HAL_GetTick();
}

/* USER CODE END 6 */

/*******************************************************************************
                       PHI IO Functions
*******************************************************************************/
/**
 * @brief  Check the ETH link state then update ETH driver and netif link accordingly.
 * @retval None
 */
void ethernet_link_thread(void *argument)
{
  // int32_t PHYLinkState = 0;
  // uint32_t linkchanged = 0U, speed = 0U, duplex = 0U;

  // struct netif *netif = (struct netif *)argument;
  // /* USER CODE BEGIN ETH link init */

  // /* USER CODE END ETH link init */

  for (;;)
  {
    osDelay(100);
  }
}

/* USER CODE BEGIN 8 */

void wait_until_register_written(void); // TODO INLINE

/*
 * Read data from chip SRAM.
 * window = 0 for Receive Buffer
 *       =  1 for User Defined area
 *       =  2 for General Purpose area
 */
int enc424j600_read_sram(unsigned char *dst, int len, unsigned short srcaddr)
{
  unsigned char command[4];

  if (len > (SPI_TRANSFER_BUF_LEN - 1) || len <= 0)
  {
    return -1;
  }

  spiEthAssertCs();

  /* First set the write pointer as per selected window */
  command[0] = WGPRDPT;
  command[1] = srcaddr & 0xFF;
  command[2] = srcaddr >> 8;
  command[3] = RGPDATA;
  spiEthTransmitBuffer(command, 4);
  /* read data */
  spiEthReceiveBuffer(dst, len);

  spiEthDeassertCs();

  return 0;
}

/**
 * Write data to chip SRAM.
 * \param priv The enc424j600 structure.
 * \param data Pointer to data buffer.
 * \param len Number of bytes to write.
 * \param dstaddr Address of the destination in the chip SRAM.
 * \return Zero on success, negative error code otherwise.
 */
int enc424j600_write_sram(const unsigned char *data, int len, unsigned short dstaddr)
{
  unsigned char command[4];

  if (len > SPI_TRANSFER_BUF_LEN - 1 || len <= 0)
    return -1;

  spiEthAssertCs();

  /* First set the general purpose write pointer */
  command[0] = WGPWRPT;
  command[1] = dstaddr & 0xFF;
  command[2] = (dstaddr >> 8) & 0xFF;
  command[3] = WGPDATA;
  spiEthTransmitBuffer(command, 4);

  /* Transfer the data */
  spiEthTransmitBuffer(data, len);

  spiEthDeassertCs();

  return 0;
}

/*
 * Set bits in an 16bit SFR.
 */
void enc424j600_set_bits(unsigned char addr, unsigned short mask)
{
  unsigned char command[4];

  spiEthAssertCs();

  command[0] = BFSU;
  command[1] = addr;
  command[2] = mask & 0xff;
  command[3] = mask >> 8;

  spiEthTransmitBuffer(command, 4);

  spiEthDeassertCs();
}

/*
 * Clear bits in an 16bit SFR.
 */
void enc424j600_clear_bits(unsigned char addr, unsigned short mask)
{
  unsigned char command[4];

  spiEthAssertCs();

  command[0] = BFCU;
  command[1] = addr;
  command[2] = mask & 0xff;
  command[3] = mask >> 8;

  spiEthTransmitBuffer(command, 4);

  spiEthDeassertCs();
}

/*
 * Write a 16bit special function register.
 * The @sfr parameters takes address of the low byte of the register.
 * Takes care of the endiannes & buffers.
 * Uses unbanked write instruction.
 */

void enc424j600_write_sfr(unsigned char sfr, unsigned short data)
{
  unsigned char command[4];

  spiEthAssertCs();

  command[0] = WCRU;
  command[1] = sfr;
  command[2] = (unsigned char)data;
  command[3] = (unsigned char)(data >> 8);

  spiEthTransmitBuffer(command, 4);

  spiEthDeassertCs();
}

/*
 * Read a 16bit special function register.
 * The @sfr parameters takes address of the low byte of the register.
 * Takes care of the endiannes & buffers.
 * Uses unbanked read instruction.
 */
unsigned short enc424j600_read_sfr(unsigned char sfr)
{
  unsigned char command[4];

  spiEthAssertCs();

  command[0] = RCRU;
  command[1] = sfr;
  command[2] = 0;
  command[3] = 0;

  spiEthTransmitReceiveBuffer(command, 4);

  spiEthDeassertCs();

  return command[2] | (command[3] << 8);
}

/*
 * Wait for bits in register to become equal to @readyMask, but at most 20ms.
 */
int checktimeout_16bit(unsigned char reg, unsigned short mask, unsigned short readyMask)
{
  unsigned short value;
  /* 20 msec timeout read */
  value = enc424j600_read_sfr(reg);
  while ((value & mask) != readyMask)
  {
    value = enc424j600_read_sfr(reg);
  }

  return 0;
}

/*
 * Reset the enc424j600.
 */
int enc424j600_soft_reset()
{

  unsigned short eudast;
  unsigned short clkReady;

  do
  {
    enc424j600_write_sfr(EUDASTL, EUDAST_TEST_VAL);
    eudast = enc424j600_read_sfr(EUDASTL);
  } while (eudast != EUDAST_TEST_VAL);

  do
  {
    clkReady = enc424j600_read_sfr(ESTATL);
  } while ((clkReady & CLKRDY) != CLKRDY);

  enc424j600_set_bits(ECON2L, ETHRST);
  HAL_Delay(1); // specification requires at least 25µs before next SPI operation

  eudast = enc424j600_read_sfr(EUDASTL);

  HAL_Delay(3);
  // specification requires at least 250µs to restore PHY registers

  if (eudast != 0)
  {
    return -1;
  }

  return 0;
}

/*
 * PHY register read
 * PHY registers are not accessed directly, but through the MII
 */
unsigned short enc424j600_phy_read(unsigned short address)
{
  enc424j600_write_sfr(MIREGADRL,
                       address | (MIREGADRH_VAL << 8));
  enc424j600_write_sfr(MICMDL, MIIRD);
  checktimeout_16bit(MISTATL, BUSY, 0);
  enc424j600_write_sfr(MICMDL, 0);
  return enc424j600_read_sfr(MIRDL);
}

void enc424j600_phy_write(unsigned short address,
                          unsigned short data)
{
  enc424j600_write_sfr(MIREGADRL,
                       address | (MIREGADRH_VAL << 8));
  enc424j600_write_sfr(MIWRL, data);
  checktimeout_16bit(MISTATL, BUSY, 0);
}

void enc424j600_hw_setlink()
{

  // unsigned short phcon1 = 0, phcon = 0, phane = 0, phstat3 = 0, phanlpa = 0, phana = 0;
  unsigned short phstat1, phcon1;
  phstat1 = enc424j600_phy_read(PHSTAT1);

  phcon1 = ANEN | RENEG;
  ;

  do
  {
    enc424j600_phy_write(PHCON1, phcon1);
    // TODO DEBUG enc424j600_phy_read(PHCON1,&phcon);
    // enc424j600_phy_read(PHANE,&phane);
    //  bit 0 : Link partner implement auto negotiation
    //  bit 1 : PHANLPA : register has been written with a new value from the link partner
    // enc424j600_phy_read(PHSTAT3,&phstat3);
    //  110 100Mbps, FULDPX
    //  101 10Mbps, FULDPX
    //  010 100Mbps, HALDPX
    //  001 10Mbps, HALDPX

    // enc424j600_phy_read(PHANA,&phana);

    // enc424j600_phy_read(PHANLPA,&phanlpa);
    /*
    15 Link partner phy supports autonegotiation
    14 Link patner has successfully reeived the local PHY
    13 Link partner has a fualt condition present
   11-0
    11 both symmetric and asymetric PAUSE
    10 asymmetric PAUSE
    01 symmetric PAUSE
    00 doesnt support PAUSE
    9 100Base T4 mode
    8 100Base TX FULDPX
    7 100Base TX HALDPX
    6 10Base TX FULDPX
    5 10Base TX HALDPX
    4-0 : should be equal to 1
    */
    HAL_Delay(20);
    phstat1 = enc424j600_phy_read(PHSTAT1);
    /*
    14 : FUL100
    13 : HALF100
    12 : FULL10
    11 : HALF10
    5 : ANDONE
    4 : Remote fault condition detected
    3 : ANABLE
    2 : Ethernet ling is established
    1 : EXTREGS

    */
  } while ((phstat1 & ANDONE) != ANDONE);

  /*
    unsigned short value;
    enc424j600_phy_read(PHCON2, &value);
    value |= FRCLNK;
    enc424j600_phy_write(PHCON2, value);
    */

  HAL_Delay(1);

  phstat1 = enc424j600_phy_read(PHSTAT1);
}

/*
 * Read the hardware MAC address to dev->dev_addr.
 */
int enc424j600_get_hw_macaddr(unsigned char dev_addr[6])
{
  unsigned short maadr1, maadr2, maadr3;

  maadr3 = enc424j600_read_sfr(MAADR3L);
  dev_addr[5] = maadr3 >> 8;
  dev_addr[4] = maadr3 & MAX_UINT8;
  maadr2 = enc424j600_read_sfr(MAADR2L);
  dev_addr[3] = maadr2 >> 8;
  dev_addr[2] = maadr2 & MAX_UINT8;
  maadr1 = enc424j600_read_sfr(MAADR1L);
  dev_addr[1] = maadr1 >> 8;
  dev_addr[0] = maadr1 & MAX_UINT8;

  return 0;
}

/*
 * Program the hardware MAC address from dev->dev_addr.
 */
int enc424j600_set_hw_macaddr(unsigned char *dev_addr)
{
  enc424j600_write_sfr(MAADR3L,
                       dev_addr[4] | dev_addr[5] << 8);
  enc424j600_write_sfr(MAADR2L,
                       dev_addr[2] | dev_addr[3] << 8);
  enc424j600_write_sfr(MAADR1L,
                       dev_addr[0] | dev_addr[1] << 8);

  return 0;
}

static void enc424j600_lowpower_disable()
{
  uint16_t phcon1;
  enc424j600_set_bits(ECON2L, ETHEN | STRCH);

  phcon1 = enc424j600_phy_read(PHCON1);
  phcon1 &= ~PSLEEP;
  enc424j600_phy_write(PHCON1, phcon1);
  enc424j600_set_bits(ECON1L, RXEN);
}

/*
 * TODO: Check the functionality
 * Low power mode shrinks power consumption about 100x, so we'd like
 * the chip to be in that mode whenever it's inactive. (However, we
 * can't stay in lowpower mode during suspend with WOL active.)
 */
// TODO TOCODE
void enc424j600_lowpower(unsigned char is_low)
{

#if 0

  if (is_low) {
    nolock_reg_bfclr(ECON1, ECON1_RXEN);
    checktimeout_8bit(ESTAT, ESTAT_RXBUSY, 0);
    checktimeout_8bit(ECON1, ECON1_TXRTS, 0);
    /* ECON2_VRPS was set during initialization */
    nolock_reg_bfset(ECON2, ECON2_PWRSV);
  } else {
    nolock_reg_bfclr(ECON2, ECON2_PWRSV);
    checktimeout_8bit(ESTAT, ESTAT_CLKRDY, ESTAT_CLKRDY);
    /* caller sets ECON1_RXEN */
  }

#endif
}

// TODO check with set_link
/* Waits for autonegotiation to complete. */
int enc424j600_wait_for_autoneg()
{
  unsigned int counter = 0;
  unsigned short value;
  /* 20 msec timeout read */
  value = enc424j600_phy_read(PHSTAT1);
  while ((value & ANDONE) == 0 && counter < 1000)
  {
    wait_until_register_written();
    value = enc424j600_phy_read(PHSTAT1);
    counter++;
  }

  if (counter < 1000)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/*
 * Reset and initialize the chip, but don't enable interrupts and don't
 * start receiving yet.
 */
int enc424j600_hw_init()
{
  unsigned int isLinkOK = 0;
  unsigned short macon2, value;
  int counter;

  // initialize SPI interface
  spiEthInit();

  hw_enable = 0;

  if (enc424j600_soft_reset() != 0)
  {
    return 0;
  }

  /* Set the tx pointer to start of general purpose SRAM area */
  enc424j600_write_sfr(ETXSTL, TXSTART);

  /* Set the rx pointer to start of general purpose SRAM area */
  enc424j600_write_sfr(ERXSTL, RXSTART);

  /* Sets the protected area in RX buffer to a smallest allowed value (2 bytes).*/
  next_pk_ptr = RXSTART;
  enc424j600_write_sfr(ERXTAILL, SRAMSIZE - 2);

  /* Set the filter in the chip */
  value = UCEN | BCEN | CRCEN | RUNTEN; // default value
  // value |= NOTMEEN;              		// promiscuoous mode
  value |= MCEN;
  enc424j600_write_sfr(ERXFCONL, value);

  /* Set auto-negotiation options */
  enc424j600_phy_write(PHANA, PHANA_DEFAULT); // Permet de faire fonctionner en connection directe //TODO : vérifier en déscativant AD100.*?

  while (0 == isLinkOK)
  {
    isLinkOK = enc424j600_wait_for_autoneg();
    counter++;
    osDelay(100);
    if (counter > 20)
    {
      break;
    }
  }

  Console_DEBUG("[enc424] Link Status %d", isLinkOK);
  Console_DEBUG("[enc424] ESTAT 0x%X", enc424j600_read_sfr(ESTATL));
  macon2 = MACON2_DEFER | PADCFG0 | TXCRCEN | HFRMEN | 0x2;

  enc424j600_write_sfr(MACON2L, macon2 | FULDPX);

  /* MAIPGL : Recomended values for inter packet gaps */
  enc424j600_write_sfr(MAIPGL, MAIPGL_VAL | (MAIPGH_VAL << 8));

  /* LED settings */
  // enc424j600_write_8b_sfr(EIDLEDH, (LED_A_SETTINGS << 4 | LED_B_SETTINGS));

  /*
   * Select enabled interrupts, but don't set the global
   * interrupt enable flag.
   */

  enc424j600_write_sfr(EIEL, LINKIE | PKTIE | DMAIE | TXIE | TXABTIE | RXABTIE);

#if (DEBUGG)
  /*
   * Check the device id and silicon revision id.
   */
  unsigned char eidledl;
  unsigned char eidledh;
  enc424j600_read_8b_sfr(EIDLEDL, &eidledl);
  enc424j600_read_8b_sfr(EIDLEDH, &eidledh);

  /* Get device ID for debugging purposes */
  unsigned char devId = ((eidledl & DEVID_MASK) >> DEVID_SHIFT);

  unsigned short txstart;
  enc424j600_read_16b_sfr(ETXSTL, &txstart);
  unsigned short rxstart = 0;
  enc424j600_read_16b_sfr(ERXSTL, &rxstart);
  unsigned short erxtaill;
  enc424j600_read_16b_sfr(ERXTAILL, &erxtaill);
  unsigned short erxfconl;
  enc424j600_read_16b_sfr(ERXFCONL, &erxfconl);
  unsigned short data;
  enc424j600_phy_read(PHANA, &data);
  unsigned short macon2l;
  enc424j600_read_16b_sfr(MACON2L, &macon2l);
  unsigned short maipgl;
  enc424j600_read_16b_sfr(MAIPGL, &maipgl);
  unsigned short eiel;
  enc424j600_read_16b_sfr(EIEL, &eiel);
#endif
  return isLinkOK;
}

void enc424j600_hw_enable()
{
  hw_enable = 0;

  /* Clear any pending interrupts */
  enc424j600_write_sfr(EIRL, 0);

  /* Enable global interrupt flag */
  enc424j600_set_bits(EIEL, INTIE);

  /* enable receive logic */
  enc424j600_set_bits(ECON1L, RXEN);

#if (DEBUGG)
  unsigned short eirl;
  enc424j600_read_16b_sfr(EIRL, &eirl);
  unsigned short intie;
  enc424j600_read_16b_sfr(EIEH, &intie);
  unsigned short econ1l;
  enc424j600_read_16b_sfr(ECON1L, &econ1l);
#endif
  hw_enable = 1;
}

void enc424j600_hw_disable()
{
  /* disable receive logic */
  enc424j600_clear_bits(ECON1L, RXEN);

  /* Disable global interrupt flag */
  enc424j600_clear_bits(EIEH, INTIE);

  hw_enable = 0;
}

int enc424j600_read_rx_area(unsigned char *dst, int len, unsigned short srcaddr)
{
  int ret = -1;

  // Adjust source address
  if (srcaddr >= SRAM_SIZE)
  {
    srcaddr -= RX_BUFFER_SIZE;
    Console_DEBUG("[enc424] srcaddr >= SRAM_SIZE 0x%X\r\n", srcaddr);
  }

  // Check if buffer to read is overlapped
  if ((srcaddr + len) < SRAM_SIZE)
  {
    ret = enc424j600_read_sram(dst, len, srcaddr);
  }
  else
  {
    int ret;
    int split = SRAM_SIZE - srcaddr;
    ret = enc424j600_read_sram(dst, split, srcaddr);
    ret += enc424j600_read_sram(dst + split, len - split, RXSTART);
  }

  return ret;
}

/**
 * Sets the protected area in RX buffer to a smallest allowed value (2 bytes).
 * Takes care of the buffer wrapping
 * \param priv The enc424j600 structure.
 */
void enc424j600_clear_unprocessed_rx_area()
{
  unsigned short tail = next_pk_ptr - 2;

  if (tail < RXSTART)
  {
	Console_DEBUG("[enc424] tail < RXSTART 0x%X \r\n", tail);
    tail = SRAM_SIZE - 2;
  }

  enc424j600_write_sfr(ERXTAILL, tail);
}

int enc_424j600_rcv_process(unsigned char *buff)
{
  int ret;
  unsigned short estat;

  ret = -1;

  estat = enc424j600_read_sfr(ESTATL) & 0xff;
  if (estat != 0)
  {
    ret = enc424j600_hw_rx(buff);
  }

  return ret;
}

int enc424j600_hw_rx(unsigned char *buff)
{
  int len = 0;
  int ret = 0;
  unsigned long rxstat;
  unsigned short next_packet;
  unsigned char rsv[RSV_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};

  (void)rxstat;
  enc424j600_read_rx_area(rsv, RSV_SIZE, next_pk_ptr);

  /* Read next packet pointer and rx status vector */

  // Compute start offset of next packet
  next_packet = (unsigned short)rsv[0] | ((unsigned short)rsv[1] << 8);

  // Compute length of received frame
  len = (unsigned short)rsv[2] | ((unsigned short)rsv[3] << 8);

#if CONSOLE_ENABLE_DEBUG_ETHERNET == CONSOLE__OPTION__ENABLED
  // CONSOLE("[enc424] Received packet %d \r\n", len);
#endif
  // Compute status of received frame
  rxstat = (unsigned long)rsv[4] | ((unsigned long)rsv[5] << 8) | ((unsigned long)rsv[6] << 16) | ((unsigned long)rsv[7] << 24);

  if (len >= MAX_FRAMELEN)
  {
	Console_DEBUG("[enc424] len >= MAX_FRAMELEN %d \r\n", len);
    return -1;
  }

  // TODO : vérifier le vecteur rxstatus aussi
  ret = enc424j600_read_rx_area(buff, len, next_pk_ptr + RSV_SIZE) < 0 ? -1 : len;

  /* Move the RX read pointer to the start of the next received packet.*/
  next_pk_ptr = next_packet;

  /* unprotect the area that was used by this packet. */
  enc424j600_clear_unprocessed_rx_area();

  /* we are done with this packet, decrement the packet counter */
  enc424j600_set_bits(ECON1L, PKTDEC);

  // memcpy(buff,spi_rx_buf,len);
  // if( ret != len)
  //{
  //     enc424j600_set_bits(ECON1H, PKTDEC);
  // }
  return ret;
}

/*
 * Access the PHY to determine link status
 */
int enc424j600_check_link_status()
{
  int ret = 0;
  unsigned short estat;

  estat = enc424j600_read_sfr(ESTATL);

  if (estat & PHYLNK)
  {
    ret = 1;
  }

  return ret;
}

/*
 * Hardware transmit function.
 * Fill the buffer memory and send the contents of the transmit buffer
 * onto the network
 */

unsigned short extlenl1;
unsigned short extlenl2;
unsigned short extlenl3;
int enc424j600_hw_tx(unsigned char *data, unsigned short len)
{
  int ret = 0;

  if (data == (void *)0 && len == 0)
  {
    return -1;
  }
  // TODO REturn 0 if len == ret 1 otherwise
  enc424j600_clear_bits(EIRL, TXIF);
  enc424j600_clear_bits(EIRL, TXABTIF);
  ret = enc424j600_write_sram(data, len, TXSTART);

  if (ret < 0)
  {
    return -1;
  }

  // CyDelayUs(1000);

  /* Write the transfer length */
  extlenl1 = 0;
  extlenl2 = 0;
  extlenl3 = len;
  extlenl1 = enc424j600_read_sfr(ETXLENL);
  enc424j600_write_sfr(ETXLENL, len);
  extlenl2 = enc424j600_read_sfr(ETXLENL);

  // CyDelayUs(250);

  /* set TX request flag */
  enc424j600_set_bits(ECON1L, TXRTS);

  return ret;
}

void enc424j600_init(void)
{
  // spiInit();
  // TODO RECUPERER ADRESS MAC ET IP
}

void wait_until_register_written()
{
  HAL_Delay(1);
}
/* USER CODE END 8 */
