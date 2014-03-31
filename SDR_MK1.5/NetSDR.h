/*
 * NetSDR.h
 *
 * Created: 1/21/2012 5:58:19 PM
 *  Author: Laid
 */


#ifndef NETSDR_H_
#define NETSDR_H_

/**
 * \addtogroup apps
 * @{
 */

/**
 * \defgroup helloworld Hello, world
 * @{
 *
 * A small example showing how to write applications with
 * \ref psock "protosockets".
 */

/**
 * \file
 *         Header file for an example of how to write uIP applications
 *         with protosockets.
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

/*
NOTE about the networking part of the code:

The original network implementation for prototype MK1.5 was based on ENC28J60. While this is working OK and the MK1.5 boards do have ENC28J60
footprint on board, it is deprecated from the MK1.5 support, as the speed of 10Mbit (and SPI interface even slower) does not permit
any useful use for this chip.

There are several AVR/Arduino implementations around for Microchip ENC28J60 chip, most prominent of which seem to be the versions
distributed by ekitszone.com, nuelectronics.com and the one created by Andrew D. Lindsay as found at http://blog.thiseldo.co.uk/?p=329

Andrew's version combines the original EtherShield stuff from the Nuelectronics written by Xing Yu and TCP/IP stack from Tuxgraphics.org
by Guido Socher and Pascal Stang, licensed as GPL2. See http://www.gnu.org/licenses/gpl.html

One of the managed C++ offspring from early Andrew's code seems to be https://github.com/turicas/Ethernet_ENC28J60 although this is
trying to manage both, ENC28J60 shields and Arduino original WIZnet W5100 controllers. I like the usability logic behind this library better
than the ones I am using, but Andrew's and Tuxgraphics stuff is more up to date (including DHCP, for one).

SDR MK1.5 implementation was originally ported for AVR32UC3B0256 SPI interface from from Andrew's v1.1 library and later reworked to version 1.6.
The updated code from Andy can be found from https://github.com/thiseldo/EtherShield

I am not exactly sure what branch is better to use - Andrew's or Tuxgraphics's. Andrew's code seems to be a cut-down version from Tuxgraphics,
and works, so this is what is used at the moment.

Just-in-case reference to Tuxgraphics: http://tuxgraphics.org/electronics/200905/embedded-tcp-ip-stack.shtml#0lfindex3
Download: http://tuxgraphics.org/common/src2/article09051/

This support is, however, deprecated now, as the 10Base Ethernet is too slow to serve for anything useful (500kHz IF takes 16Mbit on single channel
even if 16-bit bit depth is used..) The SPI speed is even slower than that (8Mbit), so theoretical maximum would be only around 200kHz IF.

The production version is therefore using the KSZ8851SNL chip from Micrel, which is 10/100 chip and also has SPI bandwidth of 40MHz.

The Networking code is also changed therefore, as tuxgraphics spinoffs did not have support for KSZ8851 and uIP had. The code this implementation is
based on port of uIP tcp/ip stack from Adam Dunkels to use with AVR microcontrollers
http://code.google.com/p/avr-uip/
http://code.google.com/p/avr-uip/source/browse/trunk/

The included uIP folder is r152 (checkout on December 5, 2011)

The ENC28J60 part of uIP is currently not ported nor tested for MK1.5.
*/

/*
#include "enc28j60\enc28j60.h"
#include "enc28j60\net.h"
#include "enc28j60\ip_arp_udp_tcp.h"
#include "enc28j60\websrv_help_functions.h"
*/

#include "psock.h"

/* First, we define the uip_tcp_appstate_t datatype. This is the state
   of our application, and the memory required for this state is
   allocated together with each TCP connection. One application state
   for each TCP connection. */

typedef struct NetSDR_state
{
	struct psock p;
	char inputbuffer[100];
	char name[40];
} uip_tcp_appstate_t;


#ifndef TELNETD_CONF_LINELEN
  #define TELNETD_CONF_LINELEN 40
#endif

#ifndef TELNETD_CONF_NUMLINES
  // orig value #define TELNETD_CONF_NUMLINES 16
  #define TELNETD_CONF_NUMLINES 8
#endif

struct telnetd_state
{
	char *lines[TELNETD_CONF_NUMLINES];
	char buf[TELNETD_CONF_LINELEN];
	char bufptr;
	u8_t numsent;
	u8_t state;
};

/*
NetSDR UDP Broadcast Discover Request Response
*/
typedef struct
{
	//fixed common 56 byte fields
	unsigned char len[2];		//length of total message in bytes (little endian byte order)
	unsigned char key[2];		//fixed key key[0]==0x5A key[1]==0xA5
	unsigned char op;			//0==Request(to device) 1==Response(from device) 2 ==Set(to device)
	char name[16];				//Device name string null terminated
	char sn[16];				//Serial number string null terminated
	unsigned char ipaddr[16];	//device IP address (little endian byte order)
	unsigned char port[2];		//device Port number (little endian byte order)
	unsigned char customfield;	//Specifies a custom data field for a particular device

	//start of optional variable custom byte fields
	//unsigned char Custom[N];
	/*
	//start of optional variable custom byte fields for NetSDR
	unsigned char macaddr[6];	//HW mac address (little endian byte order) (read only)
	unsigned char hwver[2];		//Hardware version*100  (little endian byte order) (read only)
	unsigned char fwver[2];		//Firmware version*100 (little endian byte order)(read only)
	unsigned char btver[2];		//Boot version*100 (little endian byte order) (read only)
	unsigned char fpgaid;		//FPGA ID (read only)
	unsigned char fpgarev;		//FPGA revision (read only)
	unsigned char opts;			//Options (read only)
	unsigned char mode;			//0 == Use DHCP 1==manual  2==manual Alternate data address
	unsigned char subnet[4];	//IP subnet mask (little endian byte order)
	unsigned char gwaddr[4];	//gateway address (little endian byte order)
	unsigned char dataipaddr[4];// Alternate data IP address for UDP data  (little endian byte order)
	unsigned char dataport[2];	// Alternate data Port address for UDP (little endian byte order)
	unsigned char fpga;			//0 == default cfg   1==custom1    2==custom2
	unsigned char status;		//bit 0 == TCP connected   Bit 1 == running  Bit 2-7 not defined
	unsigned char future[15];	//future use
	*/
} DISCOVER_MSG;


void NetSDR_init(void);
void NetSDR_Task(void);

#define _0DB_GAIN		7								// gain value for SetGain() what coresponds to 0dB RF Gain setting

#define NETMAXDATA (12*1024)							// data area max size is two full 6K buffers (network chip maximum with one transfer)

// NetSDR modes

#define NETDATAPACKETS24	8
#define NETPKTLEN24			1440								// for 24-bit samples this has to be 1440 for CuteSDR to recognize correctly
#define NETDATALEN24		(NETPKTLEN24*NETDATAPACKETS24)

#define NETDATAPACKETS16	12
#define NETPKTLEN16			1024
#define NETDATALEN16		(NETPKTLEN16*NETDATAPACKETS16)		// the size of the buffer filled by SSC DMA

// SDR MK1.5 native modes

#define NETDATAPACKETS24XL	2
#define NETPKTLEN24XL		(4*1440)  							// for 24-bit samples this has to be 1440 for CuteSDR to recognize correctly
#define NETDATALEN24XL		(NETPKTLEN24XL*NETDATAPACKETS24XL)

#define NETDATAPACKETS16XL	2
#define NETPKTLEN16XL		(5*1024)
#define NETDATALEN16XL		(NETPKTLEN16XL*NETDATAPACKETS16XL)		// the size of the buffer filled by SSC DMA

#define SDR_PORT	50000
#define SDR_UDP_DISCOVERY_HOST_PORT	48322
#define SDR_UDP_DISCOVERY_SDR_PORT	48321


#endif /* NETSDR_H_ */