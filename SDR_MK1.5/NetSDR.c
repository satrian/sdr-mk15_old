/*
 * NetSDR.c
 *
 * NetSDR protocol handler. This file contains routines necessary to handle NetSDR v1.03 protocol from RFspace on TCP/IP and UDP level
 *
 */

/*
 * We define the application state (struct hello_world_state) in the
 * hello-world.h file, so we need to include it here. We also include
 * uip.h (since this cannot be included in hello-world.h) and
 * <string.h>, since we use the memcpy() function in the code.
 */
#include "sdr_mk1.5.h"
#include "NetSDR.h"
#include "ksz8851.h"
#include "LM97593.h"

#include "power_clocks_lib.h"	//
#include "pdca.h"				//

#include "uip.h"
#include "dhcpc.h"

//#include "pgmspace.h"		wrapers moved to makefile definitions //pgmspace.h is just a wrapper for all the stuff associated with AVR8 pgmspace
#include "uip-conf.h"
#include "apps-conf.h"

#include "global-conf.h"
#include "gpio.h"
#include "eth_spi.h"
#include "spi.h"
#include "tx8m.h"

#include "TWI.h"



//#include "net_conf.h"
#include "clock-arch.h"

#include "timer.h"
#include "uip_arp.h"
#include "network.h"

//#include "memb.h"
//#include "shell.h"

/* Since this file will be included by uip.h, we cannot include uip.h
   here. But we might need to include uipopt.h if we need the u8_t and
   u16_t datatypes. */
#include "uipopt.h"

#include <string.h>

//#define USE_24BITNET	1
//#define NAMETEST ""
#define TX8MBUSMASTER 1

#if UIP_UDP
#else
 #error UIP_UDP must be enabled for network support to work properly!
#endif

#if (UIP_CONF_UDP_CONNS < 3)
 #error At least three UDP connections must be configured!
#endif


#define ETHBUF ((struct uip_eth_hdr *)&uip_buf[0])
#define BUF ((struct uip_tcpip_hdr *)&uip_buf[UIP_LLH_LEN])

/*
 * Declaration of the protosocket function that handles the connection
 * (defined at the end of the code).
 */
//static int handle_connection(struct NetSDR_state *s);

void Message(int16_t cdc, char* outstring, ...);
extern uint32_t f_adc;
extern volatile int32_t lastfreq_A;
extern volatile int32_t lastfreq_B;
extern uint8_t datamode;

struct uip_udp_conn *udp_conn = NULL;
struct uip_udp_conn *udp_conn_discovery = NULL;
//struct uip_udp_conn *udp_conn_dhcpbroadcast = NULL;
uip_ipaddr_t udp_addr;

extern volatile uint8_t ready2udp;
extern volatile int8_t nextisA;
uint16_t netsdrpktseq=0;

extern volatile uint8_t uipbuffA[UIP_LLH_LEN + UIP_IPUDPH_LEN + 2 + 2 + NETMAXDATA];			// ethernet headers + 2-byte netsdr data packet header + 2-byte sequence number + 6084-byte data
extern volatile uint8_t uipbuffB[UIP_LLH_LEN + UIP_IPUDPH_LEN + 2 + 2 + NETMAXDATA];			// ethernet headers + 2-byte netsdr data packet header + 2-byte sequence number + 6084-byte data

extern volatile uint8_t *netdmabuffA;
extern volatile uint8_t *netdmabuffB;

// we have defined UIP_CONF_EXTERNAL_BUFFER in uip-conf.h to be able to get it as pointer, rather than array, so create buffer here.
u8_t uip_buf_static[UIP_BUFSIZE + 2];
u8_t *uip_buf = uip_buf_static;

char udp_backaddress[4] = {0,0,0,0};		// this is the address for UDP data host, if reconfigured different from host IP using 0xC5 control command
uint16_t udp_backport=0;					// port address for UDP host, configured by 0xC5 message. used only if udp_backaddress[0] is not \0

uip_ipaddr_t lastconnected;					// this is used for tracking the incoming connections. If the incoming IP is changing, we must reset the udp_backaddress and udp_backport and close the existing UDP connection

extern u16_t uip_slen;
extern void *uip_sappdata;

extern int16_t GainCH_A;
extern int16_t GainCH_B;

extern struct dhcpc_state s;

extern int32_t SampleRate;

uint8_t ReverseEndian=1;					// if 1, the UDP sample data endian will be reversed
uint8_t netiqdump=0;

/*---------------------------------------------------------------------------*/
/*
 * The initialization function. We must explicitly call this function
 * from the system initialization code, some time after uip_init() is
 * called.
 */

uip_ipaddr_t ipaddr;

/*
Locally administered MAC addresses are in ranges:
x2-xx-xx-xx-xx-xx
x6-xx-xx-xx-xx-xx
xA-xx-xx-xx-xx-xx
xE-xx-xx-xx-xx-xx

We do use a combination of 0A-nn-nn-nn-nn-nn where nn-s are calculated from the processor unique ID

Trivia: the manufacturers OUI codes can be found from:

The IEEE public OUI listing available from:
http://standards.ieee.org/regauth/oui/index.shtml
http://standards.ieee.org/regauth/oui/oui.txt
http://www.cavebear.com/CaveBear/Ethernet/
ftp://ftp.cavebear.com/pub/Ethernet.txt

Compiled list from the sources above is at:

http://anonsvn.wireshark.org/wireshark/trunk/manuf
*/

static struct uip_eth_addr my_eth_addr = { .addr = {0,0,0,0,0,0}};
uint8_t _eth_addr[6]= {0,0,0,0,0,0};

/*
uint8_t _ip_addr[4]=	{192, 168, 100, 170};
uint8_t _net_mask[4]=	{255, 255, 255, 0};
uint8_t _gateway[4]=	{192, 168, 100, 1};

#define UDP_BACKADDRESS1	192
#define UDP_BACKADDRESS2	168
#define UDP_BACKADDRESS3	100
#define UDP_BACKADDRESS4	98


uint8_t _ip_addr[4]=	{192, 168, 0, 50};
uint8_t _net_mask[4]=	{255, 255, 255, 0};
uint8_t _gateway[4]=	{192, 168, 0, 1};

#define UDP_BACKADDRESS1	192
#define UDP_BACKADDRESS2	168
#define UDP_BACKADDRESS3	0
#define UDP_BACKADDRESS4	21
*/

uint8_t _ip_addr[4]=	{0, 0, 0, 0};
uint8_t _net_mask[4]=	{0, 0, 0, 0};
uint8_t _gateway[4]=	{0, 0, 0, 0};

struct timer dhcp_timer;
struct timer periodic_timer, arp_timer;

/*
#define STATE_NORMAL 0
#define STATE_IAC    1
#define STATE_WILL   2
#define STATE_WONT   3
#define STATE_DO     4
#define STATE_DONT   5
#define STATE_CLOSE  6
*/

extern nvram_data_t flash_nvram_data;

uint16_t NetDataPackets = NETDATAPACKETS16;
uint16_t NetPktLen = NETPKTLEN16;
uint16_t NetDataLen = NETDATALEN16;					// indicates, how much data in network mode SSC will fetch us for one buffer
uint16_t NetDataLenHalfwords = NETDATALEN16/2;		// indicates, how much data in network mode SSC will fetch us for one buffer (used because we need it inside IRQ and do not have to divide by two each time)

void dhcpc_configured(const struct dhcpc_state *s)
{
uip_ipaddr_t addr;

	UIP_LOG("Called dhcpc_configured()!\r\n");

    // byte swap the network info (note, that we are big endian here, so its different order than typical imlementations found around internet)
    _ip_addr[1] = (s->ipaddr[0]);
    _ip_addr[0] = (s->ipaddr[0]) >> 8;
    _ip_addr[3] = (s->ipaddr[1]);
    _ip_addr[2] = (s->ipaddr[1]) >> 8;

    _net_mask[1] = (s->netmask[0]);
    _net_mask[0] = (s->netmask[0]) >> 8;
    _net_mask[3] = (s->netmask[1]);
    _net_mask[2] = (s->netmask[1]) >> 8;

    _gateway[1] = (s->default_router[0]);
    _gateway[0] = (s->default_router[0]) >> 8;
    _gateway[3] = (s->default_router[1]);
    _gateway[2] = (s->default_router[1]) >> 8;

    // re-init just in case
	//uip_setethaddr(my_eth_addr);

    // set ip
    uip_ipaddr(&addr, _ip_addr[0], _ip_addr[1], _ip_addr[2], _ip_addr[3]);
    uip_sethostaddr(&addr);

    // set netmask
    uip_ipaddr(&addr,_net_mask[0], _net_mask[1], _net_mask[2], _net_mask[3]);
    uip_setnetmask(&addr);

    // set gateway
    uip_ipaddr(&addr,_gateway[0], _gateway[1], _gateway[2], _gateway[3]);
    uip_setdraddr(&addr);

//  code to use dhcp server lease time removed due to uint16_t overflow
//  issues with calculating the time.  Just use 5 minutes instead.
    timer_set(&dhcp_timer, 5 * 60 * CLOCK_SECOND);
}


/*---------------------------------------------------------------------------*/
/*
 * This is the protosocket function that handles the communication. A
 * protosocket function must always return an int, but must never
 * explicitly return - all return statements are hidden in the PSOCK
 * macros.
 */
/*
static int handle_connection(struct NetSDR_state *s)
{
  PSOCK_BEGIN(&s->p);

  PSOCK_SEND_STR(&s->p, "\0Hello. What is your name?\n");
  PSOCK_READTO(&s->p, '\n');
  strncpy(s->name, s->inputbuffer, sizeof(s->name));
  PSOCK_SEND_STR(&s->p, "\0Hello ");
  //PSOCK_SEND_STR(&s->p, s->name);
  PSOCK_CLOSE(&s->p);

  PSOCK_END(&s->p);
}
*/

/*
patch in DMA area fixed location with NetSDR protocol header fields. These do not need updating later (except when switching between 16 and 24-bit modes)
as they will not be overwritten, but for a sake of clarity it is brought here
2-byte NetSDR data packet header + 2-byte sequence number + payload
*/
void PatchNetSDRPktlen(uint16_t packetlen)
{
	uipbuffA[UIP_LLH_LEN + UIP_IPUDPH_LEN]=((2+2+packetlen)&0xFF);						//8-bit LSB of total length
	uipbuffA[UIP_LLH_LEN + UIP_IPUDPH_LEN+1]=(0x4<<5)|(((2+2+packetlen)&0x1FFF)>>8);

	uipbuffB[UIP_LLH_LEN + UIP_IPUDPH_LEN]=((2+2+packetlen)&0xFF);						//8-bit LSB of total length
	uipbuffB[UIP_LLH_LEN + UIP_IPUDPH_LEN+1]=(0x4<<5)|(((2+2+packetlen)&0x1FFF)>>8);
}

/*
Network init routine. This is called once from main() at SDR_MK1.5.c and has to take care
of all procedures related to setting up network stack (ports, IP addresses etc.).
It will also call network_init() what takes care of calling whatever hardware setup routines are left to be called after
general hardware initialization has been done at SDR_MK1.5.c startup routines
*/

void NetSDR_init(void)
{
	// Initialize network housekeeping timers
	timer_set(&periodic_timer, CLOCK_SECOND/2);
	timer_set(&arp_timer, CLOCK_SECOND * 10);
	timer_set(&dhcp_timer, 5 * CLOCK_SECOND * 60);

	// patch in DMA area fixed location with NetSDR protocol header fields. These do not need updating later (except when switching between 16 and 24-bit modes)
	// as they will not be overwritten, but for a sake of clarity it is brought here
	// 2-byte NetSDR data packet header + 2-byte sequence number + payload

	PatchNetSDRPktlen(NetPktLen);

	network_init();

	uip_init();
    // must be done or sometimes arp doesn't work
    uip_arp_init();

	uip_ipaddr(&lastconnected, 0,0,0,0);		// reset last connected address

	uint8_t* CPUSerial = (uint8_t*)0x80800204;		// internal serial start address, 120-bit (15 bytes)

	my_eth_addr.addr[0] = _eth_addr[0] = 0x0A;		// indicate that we are using locally managed MAC address
    my_eth_addr.addr[1] = _eth_addr[1] = (*(CPUSerial+1))^(*(CPUSerial+10))^(*(CPUSerial+5));	// randomize - one does not want to show what the UID bits are, if its used for crypto somewhere
    my_eth_addr.addr[2] = _eth_addr[2] = (*(CPUSerial+7))^(*(CPUSerial+9))^(*(CPUSerial+4));
    my_eth_addr.addr[3] = _eth_addr[3] = (*(CPUSerial+13))^(*(CPUSerial+2))^(*(CPUSerial+11));
    my_eth_addr.addr[4] = _eth_addr[4] = (*(CPUSerial+3))^(*(CPUSerial+6))^(*(CPUSerial+15));
    my_eth_addr.addr[5] = _eth_addr[5] = (*(CPUSerial+8))^(*(CPUSerial+14))^(*(CPUSerial+12));

	network_set_MAC(_eth_addr);
	uip_setethaddr(my_eth_addr);

	if (flash_nvram_data.dhcp)
	{
		dhcpc_init(_eth_addr/*&my_eth_addr*/, 6);
		dhcpc_request();		// this will actually ask timer to engage in address fetching on next firing
	}
	else
	{
		uip_ipaddr(&ipaddr, flash_nvram_data.ip[0], flash_nvram_data.ip[1], flash_nvram_data.ip[2], flash_nvram_data.ip[3]);
		uip_sethostaddr(ipaddr);

		uip_ipaddr(&ipaddr, flash_nvram_data.gw[0], flash_nvram_data.gw[1], flash_nvram_data.gw[2], flash_nvram_data.gw[3]);
		uip_setdraddr(ipaddr);
		uip_ipaddr(&ipaddr, flash_nvram_data.netmask[0], flash_nvram_data.netmask[1], flash_nvram_data.netmask[2], flash_nvram_data.netmask[3]);
		uip_setnetmask(ipaddr);
	}

	// We start to listen for connections on TCP port
	uip_listen(HTONS(flash_nvram_data.sdrport));

	//create UDP listener for SDR discovery protocol
	uip_ipaddr(&ipaddr, 255,255,255,255);
	udp_conn_discovery = uip_udp_new(&ipaddr, HTONS(SDR_UDP_DISCOVERY_HOST_PORT));
	if (udp_conn_discovery != NULL)
		uip_udp_bind(udp_conn_discovery, HTONS(SDR_UDP_DISCOVERY_SDR_PORT));
}

/*
This is our regular working routine what is called inside main loop of the SDR_MK1.5.c
*/

void NetSDR_Task(void)
{
uint16_t i, j, k;
uint16_t *uip_buf_short;
uint8_t *uip_buf_byte;
uint16_t tx8mpairs;
uint16_t tx8mbytes;

//uint8_t a, b;
/*
unsigned long a, b;
void *c;
unsigned long d;
*/
	if (ready2udp)	// DMA has filled data to send to network
	{
		// Why the hell are these exchanged?! Buffer nullification is obviously showing, that this order is correct, but why?
		// (MARR loading makes MAR one cycle off??)
		if (nextisA == 1)
			uip_buf=uipbuffB;					// have to send from A
		else
			uip_buf=uipbuffA;					// have to send from B

		// Since the LM97593 chip outputs data MSB first, we have wrong endian as far as NetSDR protocol is concerned!
		//
		// There seems no method so far to teach any of the AVR32 internal controllers we use on that data path (PDCA, SSC, SPI)
		// to swap endian (only can swap bit order in general ..) so we have to waste time here to do it manually ..
		//
		// Doing it as a background task while network DMA transfer is in progress, does not give much data rate penalty, since
		// CPU would be idle during that time anyway.

		uip_buf_short=(void*)&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + 4];		// used for 16-bit byteswap
		uip_buf_byte=(void*)&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + 4];			// used for 24-bit byteswap and for Tx

		// if someone has requested IQ bytes dump, output 60x4 words from buffer top
		if (netiqdump)
		{
		uint8_t *uipbuff;
			uipbuff=(void*)&uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + 4];

			for (i=0; i<60; i++)
			{
				Message(1, "{");
				for (j=0; j<4; j++)		// four I/Q words
				{
					for (k=0; k<(BitDepth/8); k++)
						Message(1, "%02X", uipbuff[(((i*4)+j)*(BitDepth/8))+k]);
					Message(1, "/");
				}
				Message(1, "}\r\n");
			}

			netiqdump=0;
		}

// Actual data transmission. If in MK1.5 mode, then we will use one of the two SSC DMA buffers (the one that is not being written at the moment)
// and set it as a source for network transfer. SPI DMA will the get the data and dump it to the network chip.

// For TX8M modes, we will ignore whatever is happening inside the radio, send the header to the network chip. We then disable 
// Atmel SPI MOSI pin and initiate TX8M board as data master. Now, when we activate network chip and tx8m chipselects and initialize DMA 
// transfer to SPI from memory (with bogus data), SPI clock will be output, but SPI data will be clocked out by TX8M.
// This sort of "virtual busmastering" gives us something what looks like external DMA and frees processor exactly the same way as it was for the
// local DMA transfers.


#if ((TX8MBUSMASTER == 1) && (TX8M == 1))
//#error tx8m busmaster enabled!
		
		tx8mpairs=twi_read(0x3B, 5, NULL);				// this will equal 2x IQ pairs
		
		if (BitDepth == _24BIT)
			tx8mbytes=2*tx8mpairs*2*3;
		else
			tx8mbytes=2*tx8mpairs*2*2;
			
		NetDataPackets=tx8mbytes/NetPktLen;
			
		for (i=0, j=0; i<(NetDataPackets); i++)
		{
			uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + 2]=netsdrpktseq&0xFF;
			uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + 3]=(netsdrpktseq>>8)&0xFF;

			if (!++netsdrpktseq)					// only the first/trigger packet is supposed to have seq# 0
				netsdrpktseq++;

			uip_slen=2+2+NetPktLen;					// have to patch this in manually, since uip_send() will also try to recopy data if udp_send() is used
			uip_udp_conn=udp_conn;					// give uIP our locally initialized UDP socket
			uip_process(UIP_UDP_SEND_CONN);			// processes all the header stuff and checksum, but does not transmit jet

			uip_arp_out();							// needed, otherwise arp decomposes!
			//network_send();							// actually transmit data

			ksz8851BeginPacketSend(uip_len);
			ksz8851SendPacketData((uint8_t *)uip_buf, UIP_LLH_LEN + UIP_IPUDPH_LEN + 2 + 2);
					
			//disable MOSI pin (make input to have it 3-stated)
			gpio_configure_pin(ETH_SPI_MOSI_PIN, GPIO_DIR_INPUT);
			
			//enable TX8M datamaster (must be done before we are enabling TX8M, otherwise MISO signals collide with network chip)
			twi_write(0x3B, 3, 0x10);	//00010000
										//||||||||
										//|||||||+-----	Do not clear error bits here
										//||||+++------	Q data marker (first 3 high order bits)
										//|||+--------- Enable TX8M data master
										//+++----------	I data marker (first 3 high order bits)
			//select TX8M
			gpio_set_pin_low(XILINX_SPICS&0xFF);
			//now perform the actual data transfer
			ksz8851SendPacketDataNonBlocking((uint8_t*)uip_buf_byte+(i*NetPktLen), NetPktLen);		// note the use of non-blocking function here!
			
			while (!spi_tx_completed())		// since non-blocking version is used, have to wait for completion before allowing new transfers, so in case endian swapping is finished already, we shall double-wait here just in case
			{
				/*
				if (!ReverseEndian)
				{
					//could actually do something useful here, like USBTask() or something, but no need at the moment.
				}
				*/
			}
			
			//unselect TX8M (must be done before datamaster is deactivated, otherwise MOSI of the cpu will collide with MISO of the tx8m)
			gpio_set_pin_high(XILINX_SPICS&0xFF);
			//disable TX8M datamastering
			twi_write(0x3B, 3, 0x0);	//00000000
										//||||||||
										//|||||||+-----	Do not clear error bits here
										//||||+++------	Q data marker (first 3 high order bits)
										//|||+--------- Disable TX8M data master
										//+++----------	I data marker (first 3 high order bits)
			//enable MOSI again
			gpio_enable_module(ETH_SPI_GPIO_MAP, sizeof(ETH_SPI_GPIO_MAP) / sizeof(ETH_SPI_GPIO_MAP[0]));

			ksz8851EndPacketSend();

			uip_slen=0;
		}
			
#else
		for (i=0, j=0; i<NetDataPackets; i++)
		{
			// If current packet is not fully endian-swapped yet, finish swapping first
			if (ReverseEndian)
			{
				if (BitDepth == _16BIT)
				{
					for (; j<(((i+1)*NetPktLen)/2); j++)
						uip_buf_short[j]=__builtin_bswap_16(uip_buf_short[j]);
				}
#ifdef USE_24BITNET
				else  //24-bit data
				{
					// We have to reverse only first and last bytes to get the "lsb first" format
					for (; j<(((i+1)*NetPktLen)); j+=3)
					{
					uint8_t b;

						b=uip_buf_byte[j];
						uip_buf_byte[j]=uip_buf_byte[j+2];
						uip_buf_byte[j+2]=b;
					}
				}
#endif
			}

			uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + 2]=netsdrpktseq&0xFF;
			uip_buf[UIP_LLH_LEN + UIP_IPUDPH_LEN + 3]=(netsdrpktseq>>8)&0xFF;

			if (!++netsdrpktseq)					// only the first/trigger packet is supposed to have seq# 0
				netsdrpktseq++;

			uip_slen=2+2+NetPktLen;					// have to patch this in manually, since uip_send() will also try to recopy data if udp_send() is used
			uip_udp_conn=udp_conn;					// give uIP our locally initialized UDP socket
			uip_process(UIP_UDP_SEND_CONN);			// processes all the header stuff and checksum, but does not transmit jet

			uip_arp_out();							// needed, otherwise arp decomposes!
			//network_send();							// actually transmit data

			ksz8851BeginPacketSend(uip_len);
			ksz8851SendPacketData((uint8_t *)uip_buf, UIP_LLH_LEN + UIP_IPUDPH_LEN + 2 + 2);
			ksz8851SendPacketDataNonBlocking((uint8_t*)uip_buf_byte+(i*NetPktLen), NetPktLen);		// note the use of non-blocking function here!

			//While the packet sends itself, go swap as much endians as we can in that time
			if (ReverseEndian)
			{
				if (BitDepth == _16BIT)
				{
					for(; (j<NetDataLenHalfwords)&&(!spi_tx_completed()); j++)
						uip_buf_short[j]=__builtin_bswap_16(uip_buf_short[j]);
				}
#if USE_24BITNET
				else
				{
					// We have to reverse only first and last bytes to get the "lsb first" format
					for(; (j<NetDataLen)&&(!spi_tx_completed()); j+=3)
					{
					uint8_t b;

						b=uip_buf_byte[j];
						uip_buf_byte[j]=uip_buf_byte[j+2];
						uip_buf_byte[j+2]=b;
					}
				}
#endif
			}

			while (!spi_tx_completed())		// since non-blocking version is used, have to wait for completion before allowing new transfers, so in case endian swapping is finished already, we shall double-wait here just in case
			{
				/*
				if (!ReverseEndian)
				{
					//could actually do something useful here, like USBTask() or something, but no need at the moment.
				}
				*/
			}

			ksz8851EndPacketSend();

			uip_slen=0;
		}
		
#endif //!tx8m busmaster

		ready2udp=0;
		uip_buf=uip_buf_static;					// restore uip's own buffer	(defined actually here, at the SDR_MK1.5.c)
	}

	uip_len = network_read();

	if(uip_len > 0)
	{
		UIP_LOG("network_read()=%d ETHBUF->type=0x%X\r\n", uip_len, ETHBUF->type);

		if(ETHBUF->type == htons(UIP_ETHTYPE_IP))	//?? BUF is defined in uip.c, but that does not seem to be the right one!
		{
			uip_arp_ipin(); // arp seems to have issues w/o this
			uip_input();
			if(uip_len > 0)
			{
				DEBUG_PRINTF("uip_input() returned, uip_len=%d\r\n", uip_len);
				uip_arp_out();
				DEBUG_PRINTF("uip_arp_out() done\r\n");
				network_send();
				DEBUG_PRINTF("network_send() done\r\n");
			}
		}
		else if(ETHBUF->type == htons(UIP_ETHTYPE_ARP))
		{
			uip_arp_arpin(); // this is correct
			if(uip_len > 0)
			{
				network_send();
			}
		}
	}
	else if(timer_expired(&periodic_timer))
	{
		//DEBUG_PRINTF("periodic_timer\r\n");

		timer_reset(&periodic_timer);

		for(i = 0; i < UIP_CONNS; i++)
		{
			uip_periodic(i);
			if(uip_len > 0)
			{
				uip_arp_out();
				network_send();
			}
		}

#if UIP_UDP
		for(i = 0; i < UIP_UDP_CONNS; i++)
		{
			uip_udp_periodic(i);
			if(uip_len > 0)
			{
				uip_arp_out();
				network_send();
			}
		}
#endif

		if(timer_expired(&arp_timer))
		{
			//DEBUG_PRINTF("arp_timer\r\n");

			timer_reset(&arp_timer);
			uip_arp_timer();
		}
	}
	else if (flash_nvram_data.dhcp && timer_expired(&dhcp_timer))
	{
		dhcpc_renew();
		timer_reset(&dhcp_timer);
	}
}


/*
 Our UDP appcall. Needed for both, SDR discovery protocol and DHCP to function
 There are currently two places where udp appcall is executed: timer and new data.
*/

void NetSDR_UDP_appcall(void)
{
DISCOVER_MSG dresp;
unsigned char sdripaddr[4];

	if (uip_poll())			// timer
	{
		if (uip_slen)
		{
			uip_arp_out();							// needed, otherwise arp decomposes!
			network_send();							// actually transmit data
		}
	}

	if (uip_udp_conn == s.conn)			// dhcp?
	{
		if (uip_newdata())
		{
			UIP_LOG("DHCP: New data\r\n");
		}

		if (flash_nvram_data.dhcp)
		{
			dhcpc_appcall();

			if (uip_slen)
			{
				UIP_LOG("dhcpc_apcall() rquested data send\r\n");
				uip_arp_out();							// needed, otherwise arp decomposes!
				network_send();							// actually transmit data
			}
		}
	}

	if (uip_newdata())
	{
		if (uip_udp_conn == udp_conn_discovery)
		{
			UIP_LOG("UDP Broadcast Discover Request Received!\r\n");

			//someone wants to discover us, so lets be discoverable and identify ourselves!

			dresp.len[0]=56;								//unsigned char len[2];		//length of total message in bytes (little endian byte order)
			dresp.len[1]=0;
			dresp.key[0]=0x5A;								//unsigned char key[2];		//fixed key key[0]==0x5A key[1]==0xA5
			dresp.key[1]=0xA5;
			dresp.op=1;										//unsigned char op;			//0==Request(to device) 1==Response(from device) 2 ==Set(to device)
#ifdef NAMETEST
			sprintf(dresp.name, NAMETEST);						// max 15bytes + /0
#else
			sprintf(dresp.name, "SDR MK1.5");				// max 15bytes + /0
#endif
			memmove(dresp.sn, SDRSerial, 9);				// max 15bytes + /0
			uip_gethostaddr(&sdripaddr);
			sprintf((char*)dresp.ipaddr, "%c%c%c%c", sdripaddr[3], sdripaddr[2], sdripaddr[1], sdripaddr[0]);	// max 15bytes + /0		// 170,100,168,192  //strange, but out of 15 fields only 4 are used!
			dresp.port[0]=SDR_PORT&0xFF;
			dresp.port[1]=SDR_PORT>>8;
			dresp.customfield=0;							//unsigned char customfield;	//Specifies a custom data field for a particular device
															//start of optional variable custom byte fields
															//unsigned char Custom[N];
			memmove(uip_sappdata, &dresp, sizeof(DISCOVER_MSG));
			uip_slen=sizeof(DISCOVER_MSG);

			// dont like to go redundant, so have to send on next timer call! (meanwhile, uip_process() will create UDP message for us into buffer when we exit this function)
		}
	}

	// if we exit this function with uip_slen having any value, udp_send: will be branched in uip_process() main loop (this is the function what called appcall)
	// This will, however, NOT execute network_send(), but only prepare data, so the next timer call will have to call network_send()
}



static void closed(void);
static void acked(void);
static void newdata(void);
static void senddata(void);

/*---------------------------------------------------------------------------*/
/*
 * In uipopt.h we have defined the UIP_APPCALL macro to
 * NetSDR_appcall so that this funcion is uIP's application
 * function. This function is called whenever an uIP event occurs
 * (e.g. when a new connection is established, new data arrives, sent
 * data is acknowledged, data needs to be retransmitted, etc.).

 We are running single-socket based for simplicity and speed, at the moment,
 so server side has to manage connections carefully and preferably
 not leave anything hanging unterminated.
 */

void NetSDR_appcall(void)
{

	DEBUG_PRINTF("NetSDR_appcall() entered\r\n");

	if (uip_connected())
	{
		UIP_LOG("Connected!\r\n");
		UIP_LOG("(destport(lport)=%d, srcport(rport)=%d, ripaddr=%d.%d.%d.%d)\r\n", BUF->destport, BUF->srcport,
															uip_ipaddr1(BUF->srcipaddr), uip_ipaddr2(BUF->srcipaddr), uip_ipaddr3(BUF->srcipaddr), uip_ipaddr4(BUF->srcipaddr));

		// we do not do anything here when the connection has just been established, although at the presence of the daughtercard
		// what have LED-s present it would make sense to light one of those up.
	}
/*
	if (s.state == STATE_CLOSE)
	{
		UIP_LOG("Connection closed!\r\n");
		uip_close();

		// If TCP connection is closed the radio should theoretically be returned to its default state.
		// However, we do not want data capture to break up because of network glitches, so we will keep it in whatever state it was!
		return;
	}
*/

	if (uip_closed() || uip_aborted() || uip_timedout())
	{
		closed();
	}

	if (uip_acked())
	{
		acked();
	}

	if (uip_newdata())
	{
		// process incoming command data
		newdata();
	}

	if (uip_rexmit() || uip_newdata() || uip_acked() || uip_connected() || uip_poll())
	{
		senddata();
	}
}

//---------------------------------------------------------------------------
static void closed(void)
{
	// we do not change state on connection close, but if we would do, then this is a place to run all the cleanup procedures
}


//---------------------------------------------------------------------------
static void acked(void)
{
	// Since there is no retry functionality at the moment, this is not processed
}

#if (UIP_BUFSIZE < 1500)
 #error uIP BUFSIZE too small (check uip-conf.h)
#endif

char netretdata[UIP_BUFSIZE-(UIP_IPTCPH_LEN+UIP_LLH_LEN)];
int netretlen;

void process_ctrl_item(uint16_t ctrlitem, uint8_t msgtype, char* payload, uint16_t payloadlen);

//---------------------------------------------------------------------------
static void newdata(void)
{
uint32_t i;
uint16_t len;
char *dataptr;
uint16_t msglen, ctrlitem, j;
uint8_t msgtype;

	// incoming command processor

	len = uip_datalen();
	dataptr = (char *)uip_appdata;

	netretlen=0;
	netretdata[0]=0;

	// process incoming messages (may be several per received packet!)

	if (len > 2)		// at least 16-bit header is required to do anything
	{
		for (i=0; i<len;)
		{
			msglen=0;
			ctrlitem=0;
			msgtype=0;

			msglen=(char)dataptr[i];
			msglen|=(dataptr[i+1]&0x1F)<<8;

			if (!msglen)
			{
				UIP_LOG("NetSDR: No message length!\r\n");
				break;
			}

			msgtype=(dataptr[i+1]&0xE0)>>5;

			if ((!(msgtype&0x4))&&(msglen>=4)&&((len-i)>=msglen))
			{
				// control message type with sufficient message length
				ctrlitem=dataptr[i+2];
				ctrlitem|=dataptr[i+3]<<8;

				UIP_LOG("NetSDR: uiplen=%d msglen=%d msgtype=0x%X ctrlitem=0x%04X ", len, msglen, msgtype, ctrlitem);

				if (msglen > 4)	//payload?
				{
					UIP_LOG("Data: [");
					for (j=0; j<(msglen-4); j++)
						UIP_LOG("%02X ", dataptr[i+4+j]);
					UIP_LOG("]");
				}

				UIP_LOG("\r\n");

				process_ctrl_item(ctrlitem, msgtype, dataptr+i+4, msglen-4);
			}
			else
			{
				// data item or invalid length, so stop processing
				UIP_LOG("NetSDR: Incomplete or Data Item\r\n");
			}

			i+=msglen;		// slightly dangerous to hacking, but this is why we do have i set up as 32-bit unsigned int
		}

		if (i<len)
		{
			UIP_LOG("NetSDR: %ld Remaining bytes: [", (len-i));
			for (j=0; j<(len-i); j++)
				UIP_LOG("%02X ", dataptr[i+j]);
			UIP_LOG("]\r\n");
		}
	}
}

void process_ctrl_item(uint16_t ctrlitem, uint8_t msgtype, char* payload, uint16_t payloadlen)
{
uint32_t nw_freq, nw_samplerate;
uint8_t regval;
uint16_t vercode;
static uint16_t SampleRateSet=0;

	// command codes as per NetSDR v1.03 standard (2011-11-01)
	// SDR MK1.5 specific modifications are marked with "[MK1.5]" tag
	switch(ctrlitem)
	{
		//*** General control items
		case 1:		//Returns an ASCII string describing the Target device.
#ifdef NAMETEST
			memmove(netretdata+netretlen, "\xB\0\1\0", 4);
			sprintf(netretdata+netretlen+4, NAMETEST);
			netretlen+=4+7;		// including terminating 0
#else
			memmove(netretdata+netretlen, "\x17\0\1\0", 4);
			sprintf(netretdata+netretlen+4, "SDR MK1.5 'Andrus'");
			netretlen+=4+19;		// including terminating 0
#endif
			break;

		case 2:		//Contains an ASCII string containing the Target device serial number.
			memmove(netretdata+netretlen, "\xD\0\2\0", 4);
			memmove(netretdata+netretlen+4, SDRSerial, 9);	// 8 bytes + terminating 0
			netretlen+=4+9;		// including terminating 0
			break;

		case 3:		//Contains the version number of the Host or Targets implemented Interface. This allows the Host or Target
					//to display or adapt to different versions of the interface.
			memmove(netretdata+netretlen, "\6\0\3\0", 4);
			memmove(netretdata+netretlen+4, "\x67\0", 2);	//0x67=103=1.03 interface spec
			netretlen+=4+2;
			break;

		case 4:		//Contains the Firmware or Hardware version information of the Target.
					//1-byte payload=ID
			memmove(netretdata+netretlen, "\7\0\4\0", 4);
			memmove(netretdata+netretlen+4, payload, 1);

			switch(payload[0])
			{
				case 0:		//ID=0 returns the boot code version.
					vercode=(VER_MAJOR*100)+VER_MINOR;
					netretdata[netretlen+5]=vercode&0xFF;
					netretdata[netretlen+6]=(vercode>>8)&0xFF;
					break;
				case 1:		//ID=1 returns the application firmware version.
					vercode=(VER_MAJOR*100)+VER_MINOR;
					netretdata[netretlen+5]=vercode&0xFF;
					netretdata[netretlen+6]=(vercode>>8)&0xFF;
					break;
				case 2:		//ID=2 returns the Hardware version.
					vercode=(1*100)+5;	// v1.5
					netretdata[netretlen+5]=vercode&0xFF;
					netretdata[netretlen+6]=(vercode>>8)&0xFF;
					break;
				case 3:		//ID=3 returns the FPGA Configuration version.
					vercode=0;	//no FPGA as of now
					netretdata[netretlen+5]=vercode&0xFF;
					netretdata[netretlen+6]=(vercode>>8)&0xFF;
					break;
			}

			netretlen+=4+3;
			break;

		case 5:		//Contains the Error/Status code(s) of the Target. This item is used to notify the Host of any error or status
					//change using a list of code values.

			//0x0B = SDR Idle
			//0x0C = SDR Busy(capturing data)
			//0x0E = SDR Boot mode Idle
			//0x0F = SDR Boot mode busy programming
			//0x20 = SDR A/D overload occurred
			//0x80 = SDR Boot mode programming error

			memmove(netretdata+netretlen, "\5\0\5\0", 4);
			if (datamode == DATA_NETWORK)
				memmove(netretdata+netretlen+4, "\xC", 1);		// show that the unit is capturing data
			else
				memmove(netretdata+netretlen+4, "\xB", 1);		// show idle for all other modes than network
			netretlen+=4+1;

			break;

		case 9:		//Returns the 4 byte product ID for the SDR used in firmware update validation.
			//SDR-Radio checks for this
			memmove(netretdata+netretlen, "\x8\0\x9\0", 4);
			memmove(netretdata+netretlen+4, "\x53\x44\x52\x4", 4);		// just bogus from spec example
			netretlen+=4+4;

			break;

		case 0xA:	//Returns information on installed options for the SDR.
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0xB:	//Returns 32 bit Security code based on 32 bit security key.
					//4-byte payload with 32-bit BCD security key
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0xC:	//Set or Request the current FPGA Configuration
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		//*** Receiver control items
		case 0x18:	//Controls the operational state of the SDR and specifies the data capture modes and formats. (Main "Start/Stop" command)
					//4-byte payload, see documentation
			if (payload[1] == 1)		// stop command
			{
				/// program multiplexer
				///regval=ReadRegister(6);
				///regval|=(1<<4);			// set MUX_MODE to 1, so both channels data is transmitted
				///WriteRegister(6, regval);
				//InitIQDataEngine(_2X16BIT_IQ);			// configure SSC engine to fetch 4 words since both channel data is delivered during single frame sync. Force 16-bit here
				// Note, that SampleMode() recalculates our ADC master clock to have the clocks dividing without jitter.
				SampleMode(48000, DUAL_CHANNEL, _16BIT, DATA_AUDIO);		// go with 16 bits for here and now (also recalculates LO frequencys)
				datamode=DATA_AUDIO;	// return whole radio to default mode
				memmove(netretdata+netretlen, "\x8\0\x18\0", 4);
				memmove(netretdata+netretlen+4, "\x80\x1\0\0", 4);
				netretlen+=4+4;
				//SampleRateSet=0;		// needed because cutesdr is not setting sample rate for non-netsdr radios, so we have to track if sample rate is set or not
			}
			else if (payload[1] == 2)	// start command
			{
				// see, if the incoming conection is from the same host IP as previously. If is a new host, then reset the UDP
				if (!(uip_ipaddr_cmp(&lastconnected, uip_conn->ripaddr)))
				{
					// reset existing UDP connection, if any
					if (udp_conn)
					{
						uip_udp_remove(udp_conn);
						udp_conn=NULL;			// this is only our local variable, the general UDP connections array is not touched by this
					}
				}

				if (udp_conn == NULL)
				{
					// the return address is generally the same as the host connecting, except when reconfigured by 0xC5 command.

					if (udp_backaddress[0] != 0)
					{
						uip_ipaddr(&udp_addr, udp_backaddress[0], udp_backaddress[1], udp_backaddress[2], udp_backaddress[3]);
						udp_conn = uip_udp_new(&udp_addr, HTONS(udp_backport));
					}
					else
					{
						uip_ipaddr_copy(&udp_addr, uip_conn->ripaddr);
						udp_conn = uip_udp_new(&udp_addr, HTONS(flash_nvram_data.udp_port));	// copy incoming TCP ripaddress to udp connection
					}

					uip_ipaddr_copy(&lastconnected, uip_conn->ripaddr);
				}

				/// program multiplexer
				/// taken care at Init_LM97593() now regval=ReadRegister(6);
				///regval&=~(1<<4);			// set MUX_MODE to 0, so only single channel data is transmitted
				///WriteRegister(6, regval);
#ifdef USE_24BITNET
				if (payload[2] & 0x80)	// 24-bit data requested?
				{
					if (!SampleRateSet)
						SampleMode(196078, SINGLE_CHANNEL, _24BIT, DATA_NETWORK);		// Patch for cutesdr non-se version. Go with 24 bits for here and now (also recalculates LO frequencys)
					else
						SampleMode(SampleRate, SINGLE_CHANNEL, _24BIT, DATA_NETWORK);	// sets everything, including NetPktLen etc. globals!

					PatchNetSDRPktlen(NetPktLen);
					netsdrpktseq=0;

					datamode=DATA_NETWORK;		// set radio to network mode
					memmove(netretdata+netretlen, "\x8\0\x18\0", 4);
					memmove(netretdata+netretlen+4, "\x80\x2\0x80\0", 4);
				}
				else
#endif
				{
					if (!SampleRateSet)
						SampleMode(196078, SINGLE_CHANNEL, _16BIT, DATA_NETWORK);		// Patch for cutesdr non-se version. Go with 16 bits for here and now (also recalculates LO frequencys)
					else
						SampleMode(SampleRate, SINGLE_CHANNEL, _16BIT, DATA_NETWORK);	// sets everything, including NetPktLen etc. globals!

					PatchNetSDRPktlen(NetPktLen);
					netsdrpktseq=0;

					datamode=DATA_NETWORK;		// set radio to network mode
					memmove(netretdata+netretlen, "\x8\0\x18\0", 4);
					memmove(netretdata+netretlen+4, "\x80\x2\0\0", 4);
				}

				netretlen+=4+4;
			}
			// do not update netretlen if invalid command number was given!
			break;

		case 0x19:	//Sets up the various multi-channel modes.
					//1-byte payload, see documentation
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0x20:	//Controls the SDR NCO center frequency.
					//6-byte payload, see documentation

			if ((payloadlen == 1)&&(msgtype == 2))		// only channel is given and msgtype=get/set range, so communicate back frequency boundaries
			{
				memmove(netretdata+netretlen, "\x15\x40\x20\0", 4);		// we have one set of frequency parameters, so message length is 4+1+1+(3*5)=21=0x15
				memmove(netretdata+netretlen+4, payload, 1);			// patch in channel number
				memmove(netretdata+netretlen+5, "\1", 1);				// we have only one frequency set/range

				memmove(netretdata+netretlen+6, "\0\0\0\0\0", 5);		// Freq Min = 0
				memmove(netretdata+netretlen+6+5, "\0\x48\xE8\1\0", 5);	// Freq Max = 32MHz at the moment
				memmove(netretdata+netretlen+6+10, "\0\0\0\0\0", 5);	// Downconverter VFO is not used

				netretlen+=4+2+(3*5);
			}
			else if ((payloadlen == 1)&&(msgtype == 1))	// only channel is given, but msgtype=get/set item, so return current frequency
			{
				memmove(netretdata+netretlen, "\xA\0\x20\0", 4);
				memmove(netretdata+netretlen+4, payload, 1);

				if (payload[0]==0)
				{
					netretdata[netretlen+5]=(char)lastfreq_A&0xFF;
					netretdata[netretlen+6]=(char)(lastfreq_A>>8)&0xFF;
					netretdata[netretlen+7]=(char)(lastfreq_A>>16)&0xFF;
					netretdata[netretlen+8]=(char)(lastfreq_A>>24)&0xFF;
					netretdata[netretlen+9]=0;

					netretlen+=4+6;
				}
				else if (payload[0]==2)
				{
					netretdata[netretlen+5]=(char)lastfreq_B&0xFF;
					netretdata[netretlen+6]=(char)(lastfreq_B>>8)&0xFF;
					netretdata[netretlen+7]=(char)(lastfreq_B>>16)&0xFF;
					netretdata[netretlen+8]=(char)(lastfreq_B>>24)&0xFF;
					netretdata[netretlen+9]=0;

					netretlen+=4+6;
				}

				// do not update netretlen if invalid channel number was given!
			}
			else if (payloadlen == 6)	// channel + payload means that we have to set new frequency
			{
				nw_freq=payload[1]|(payload[2]<<8)|(payload[3]<<16)|(payload[4]<<24);		// the most significant byte is not used, as we are currently only capable handling 32-bit frequency..!
				if ((payload[0]==0)||(payload[0]==0xFF))
					SetFreq(CH_A, nw_freq, 1, f_adc);
				else if ((payload[0]==1)||(payload[0]==0xFF))
					SetFreq(CH_B, nw_freq, 1, f_adc);

				memmove(netretdata+netretlen, "\xA\0\x20\0", 4);
				memmove(netretdata+netretlen+4, payload, 6);
				netretlen+=4+6;
			}
			break;

		case 0x22:	//Controls the SDR NCO phase offset when dual channels are used.
					//5-byte payload, see documentation
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0x23:	//Controls the SDR A/D Scaling value when dual channels are used.
					//3-byte payload, see documentation
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0x38:	//Controls the Level of RF gain( or attenuation) of the receiver.
					//2-byte payload
			//SDR-Radio checks for that
			memmove(netretdata+netretlen, "\x6\0\x38\0", 4);

			if (payloadlen == 1)
			{
			int16_t gain;

				// get current gain
				if (payload[0] == 0)
					gain=GetGain(CH_A);
				else
					gain=GetGain(CH_B);

				memmove(netretdata+netretlen+4, payload, 1);

				if (gain >= _0DB_GAIN)
					memmove(netretdata+netretlen+5, "\0", 1);	//0dB
				else if (gain >= (_0DB_GAIN-2))
					memmove(netretdata+netretlen+5, "\xF6", 1);	//-10dB
				else if (gain >= (_0DB_GAIN-4))
					memmove(netretdata+netretlen+5, "\xEC", 1);	//-20dB
				else //if (gain <= (_0DB_GAIN-5))
					memmove(netretdata+netretlen+5, "\xE2", 1);	//-30dB


			}
			else if (payloadlen == 2)
			{
			int16_t channel;

				if (payload[0] == 0)
					channel=CH_A;
				else
					channel=CH_B;
					
				//experimental gain control to see if it makes any difference
				
				Message(1, "Gain set: channel=%d, gain=%d(0x%02.2X)\r\n", payload[0], payload[1], payload[1]);
				
				switch(payload[1])
				{
					case 0:		//0dB - Enable AGC
						GainCH_A=_0DB_GAIN;
						WriteRegister(20, ReadRegister(20)&0xF7);	// enable AGC loop
						SetGain(CH_A, 7);							//highest IF gain with non-fixed exponent
						break;
					//below are all exponents fixed, bit shift 5 	
					case 0xF6:	//-10dB (-12dB actually)
						GainCH_A=_0DB_GAIN-2;
						SetGain(CH_A, 8);		

						//3 highest bits reversed, so 000=hignest gain and 111=lowest.
						//0		1110 =0000 =00
						//-6	1100 =0010 =20
						//-12	1010 =0100 =30
						//-18	1000 =0110 =60
						//-24	0110 =1000 =80
						//-30	0100 =1010 =A0
						//-36	0010 =1100 =C0
						//-42	0000 =1110 =E0
						WriteRegister(20, ReadRegister(20)&0xF7);	// enable AGC loop
						WriteRegister(23, 0x30);							
						WriteRegister(20, ReadRegister(20)|0x8);	//disable AGC loop and set integrator value
						break;

					case 0xEC:	//-20dB (-18dB actually)
						GainCH_A=_0DB_GAIN-4;
						SetGain(CH_A, 8);		
						WriteRegister(20, ReadRegister(20)&0xF7);	// enable AGC loop
						WriteRegister(23, 0x60);	
						WriteRegister(20, ReadRegister(20)|0x8);	//disable AGC loop and set integrator value
						break;

					case 0xE2:	//-30dB
						GainCH_A=_0DB_GAIN-5;
						SetGain(CH_A, 8);	
						WriteRegister(20, ReadRegister(20)&0xF7);	// enable AGC loop
						WriteRegister(23, 0xA0);	
						WriteRegister(20, ReadRegister(20)|0x8);	//disable AGC loop and set integrator value
						break;
					
					default:
						break;
				}
					
				/*

				switch(payload[1])
				{
					case 0:		//0dB
						SetGain(channel, _0DB_GAIN);
						break;

					case 0xF6:	//-10dB
						SetGain(channel, _0DB_GAIN-2);	// -12dB actually
						break;

					case 0xEC:	//-20dB
						SetGain(channel, _0DB_GAIN-4);	// -24dB actually
						break;

					case 0xE2:	//-30dB
						SetGain(channel, _0DB_GAIN-5);	// -30dB
						break;
				}
				*/
				// set gain
				memmove(netretdata+netretlen+4, payload, 2);	// payload 2 is a gain: 0x00=0dB, 0xF6=-10dB, 0xEC=-20dB, 0xE2=-30dB
			}

			netretlen+=4+2;
			break;

		case 0x40:	//UNDOCUMENTED. According to cutesdr source code, this is "RX_IF_GAIN"
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0x44:	//Controls the Analog RF Filter selection.
					//2-byte payload
			//SDR-Radio checks for that
			memmove(netretdata+netretlen, "\x6\0\x44\0", 4);
			memmove(netretdata+netretlen+4, payload, 2);
			netretlen+=4+2;
			break;

		case 0x8A:	//Controls various A/D Modes.
					//2-byte payload
			//SDR-Radio checks for that
			memmove(netretdata+netretlen, "\x6\0\x8A\0", 4);
			memmove(netretdata+netretlen+4, payload, 2);
			netretlen+=4+2;
			break;

		case 0xB8:	//Specifies the SDR I/Q data sample rate.
					//5-byte payload

			// payload[0] is ignored, since all channels go with the same sample rate
			nw_samplerate=payload[1]|(payload[2]<<8)|(payload[3]<<16)|(payload[4]<<24);

			// Note, that SampleMode() recalculates our ADC master clock to have the clocks dividing without jitter.
			SampleMode(nw_samplerate, SINGLE_CHANNEL, _16BIT, 0);	// go with 16 bits for here and now (also recalculates LO frequencys)
			
#if (TX8M == 1)
			//select appropriate sample rate for 24-bit ADC's
#endif

			memmove(netretdata+netretlen, "\x9\0\xB8\0", 4);
			memmove(netretdata+netretlen+4, payload, 5);
			netretlen+=4+5;

			SampleRateSet=1;		// cutesdr does not set sample rate, so we have to check if this function has been called or not.

			break;

		case 0xB0:	//Specifies the SDR A/D input sample rate for calibration purposes.
					//5-byte payload

			// In reason or another, sdr-radio v1.3 calls this function with only single parameter.
			// This is not by the spec (there is no calibration inquiry in the Rev1.03 spec), but lets answer.
			// note also, that whoever is aquiring this, expects us to respond with 80MHz ...
			memmove(netretdata+netretlen, "\x9\0\xB0\0", 4);
			memmove(netretdata+netretlen+4, "\0\0\xB4\xC4\x4", 5);		//80MHz
			//memmove(netretdata+netretlen+4, "\0\0\x90\xD0\x3", 5);		//64MHz
			netretlen+=4+5;
			//memmove(netretdata+netretlen, "\x9\0\xB0\0", 4);
			//memmove(netretdata+netretlen+4, payload, 5);
			//netretlen+=4+5;
			break;

		case 0xD0:	//Sets the DC offset value to be used by the A/D Converter.
					//3-byte payload
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		//*** Misc control items
		case 0xB6:	//Controls various Hardware Pulse output modes.(requires Hardware Option)
					//2-byte payload
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0xC4:	//Sets the UDP data packet size for the SDR.
					//1-byte payload:
					//	0 = (default) Large UDP packets (1444 bytes(24bit data) or 1028 bytes(16bit data)), little endian data
					//	1 = (not supported currently) Small UDP packets (388 bytes(24bit data) or 516 bytes(16bit data)), little endian data
					//	[MK1.5] 0x80 = Large UDP packets (1444 bytes(24bit data) or 1028 bytes(16bit data), BIG ENDIAN data
					//	[MK1.5] 0x81 = (not supported currently) Small UDP packets (388 bytes(24bit data) or 516 bytes(16bit data)), BIG ENDIAN data
					//	[MK1.5] 0x82 = (not supported currently) XtraLarge UDP packets (), BIG ENDIAN data

			memmove(netretdata+netretlen, "\5\0\xC4\0", 4);
			memmove(netretdata+netretlen+4, payload, 1);

			switch(payload[0])
			{
				case 0x80:				//[MK1.5] 0x80 = Large UDP packets (1444 bytes(24bit data) or 1028 bytes(16bit data), BIG ENDIAN data
					ReverseEndian=0;	//LM97593 already outputs bigendian data, so no need to swap bytes
					break;

				case 0:					//Large UDP packets (1444 bytes(24bit data) or 1028 bytes(16bit data) ) (default), little endian data
				default:
					ReverseEndian=1;	//by default we have to make data little-endian, so have to reverse
					break;
			}

			netretlen+=4+1;

			break;

		case 0xC5:	//Sets the UDP IP address and Port number for the SDR data output.
					//6-byte payload
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0x150:	//Specifies CW power on startup message for SDR.
					//12-byte payload
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0x200:	//Specifies and opens the SDR RS232 Serial port.
					//10-byte payload
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		case 0x201:	//Specifies and closes the SDR RS232 Serial port.
					//1-byte payload
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;

		default:
			memmove(netretdata+netretlen, "\2\0", 2);	//NAK
			netretlen+=2;
			break;
	}
}

//---------------------------------------------------------------------------
static void senddata(void)
{
uint16_t i;

	if (netretlen)
	{
		UIP_LOG("NetSDR: Responding %d bytes: [", netretlen);
		for (i=0; i<netretlen; i++)
			UIP_LOG("%02X ", netretdata[i]);
		UIP_LOG("]\r\n");
	}

	uip_send(netretdata, netretlen);
	netretlen=0;
	netretdata[0]=0;
}


