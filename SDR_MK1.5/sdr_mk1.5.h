
 
/** \file
 *
 *  Header file for AudioInput.c.
 */

#define	VERINFO		"v1.99"
#define VER_MAJOR	1
#define VER_MINOR	99


#ifndef _SDR_MK15_H_
#define _SDR_MK15_H_

	/* Includes: */

//xxx#include <LUFA/Platform/UC3/InterruptManagement.h>
//xxx#include <LUFA/Platform/UC3/ClockManagement.h>
#include <avr32/io.h>

//#include <wdt.h>
//#include <power.h>
//#include <interrupt.h>

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

//#include <LUFA/Common/Common.h>
//#include <LUFA/Version.h>

#include <LUFA/Platform/UC3/InterruptManagement.h>
#include <LUFA/Platform/UC3/ClockManagement.h>
#include <LUFA/Drivers/USB/USB.h>
#include <LUFA/Drivers/USB/Core/USBInterrupt.h>

#include "Descriptors.h"

int16_t SetupHardware(void);

void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);

void Message(int16_t cdc, char* outstring, ...);

void delayms(uint16_t delms);
void delayMicroseconds(uint32_t delus);

void SampleMode(uint32_t samplefreq, uint16_t channels, uint16_t numbits, uint16_t ifcmode);

void InitIQDataEngine(uint16_t bytesperframe, uint16_t ifcmode, uint16_t resynconly);
void Init_SSC(uint16_t bytespeframe, int enablerx);

#define		DATA_AUDIO				1
#define		DATA_LIBUSB				2
#define		DATA_NETWORK			3
#define		DATA_SDRIQ				4
#define		DATA_LIBUSB_SPEEDTEST	5
#define		DATA_LIBUSB_XPY			6
#define		DATA_LIBUSB_XMY			7
#define		DATA_TX8M_LIBUSB		8
#define		DATA_TX8M_NETWORK		9


// libusb commands to control radio through interface 06 control interface (CDC2)

#define		LIBUSB_MODE				14
//#define		LIBUSB_MODE				15
#define		LIBUSB_READREG			16
#define		LIBUSB_WRITEREG			17
#define		LIBUSB_SETFREQ			18
#define		LIBUSB_GETFREQ			19
#define		LIBUSB_SAMPLEMODE		20
#define		LIBUSB_GETVER			21
#define		LIBUSB_SETGAIN			22
#define		LIBUSB_SETPHASE			23
#define		LIBUSB_PANTABLE			24

#define		LIBUSB_CHA			0
#define		LIBUSB_CHB			1

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// LIBUSB_MODE flags. NB!! These must be kept in sync with ExtIO DLL radio button values!

#define		LIBMODE_16A			0	//A only
#define		LIBMODE_16B			1	//B only

#define		LIBMODE_16APB		2	//A+B
#define		LIBMODE_16AMB		3	//A-B
#define		LIBMODE_16BMA		4	//B-A
#define		LIBMODE_16ABPAN		5	//A, B=panscan
#define		LIBMODE_16BAPAN		6	//A=panscan, B
//#define		LIBMODE_16AB		7	//A,B
#define		LIBMODE_16IAQB		7	//I= ch_A I,  Q=ch_B I

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#define		LIBMODE_OFF			10
#define		LIBMODE_SPEEDTEST	11
#define		LIBMODE_TXTEST		12

#define		LIBMODE_24A			20
#define		LIBMODE_24B			21

#define		LIBMODE_24AB		25

#define	FREQ_63744000	63744000
#define FREQ_64000000	64000000
#define FREQ_59904000	59904000
#define FREQ_58750000	58750000
// NB! If freq is equal or lower than 58MHz, the LM97593 divisor is set to /2 by Init_LM97593() (/4 for all frequencys above that)
#define FREQ_57500000	57500000
#define FREQ_40000000	40000000
#define FREQ_30000000	30000000

extern uint8_t	datamode;

extern unsigned char SDRSerial[9];		// SDR unique serial number stored here

#define SINGLE_CHANNEL	1
#define DUAL_CHANNEL	2

#define _16BIT	16
#define _24BIT	24
extern uint16_t BitDepth;				// I/Q data bit length. 16 or 24, dependent on configuration

// how many bytes to clock in from LM97593 depending on bit depth and channel modes
#define _1X16BIT_IQ		((1*16*2)/8)
#define _1X24BIT_IQ		((1*24*2)/8)
#define _2X16BIT_IQ		((2*16*2)/8)
#define _2X24BIT_IQ		((2*24*2)/8)

#define NVRAMSIZE	256	// we have reserved us 256 bytes worth of data at flash through linker definition file avr32elf_uc3b0512.x
//! Structure type containing variables to store in NVRAM using a specific
//! memory map.

typedef const struct
{
	uint8_t		dhcp;		// 0=dhcp off, 1=dhcp on
	uint8_t		ip[4];		// manually configured ip address
	uint8_t		gw[4];		// manually configured gateway
	uint8_t		netmask[4];	// manually configured netmask
	uint16_t	sdrport;	// SDR incoming TCP connections port
	uint16_t	udp_port;	// SDR outgoing UDP data connections port

	// always keep as last two bytes of the area
	uint16_t crc;		// 16-bit CRC of flash parameter area
} nvram_data_t;

// structure for running the panoramic scan by
typedef struct
{
	uint32_t	startfreq;	// boundary frequency start (Hz)
	uint32_t	samples;	// how many samples to fetch (note, that this is only indicative -- the actual sample count varies (is somewhat higher), since we can not guarantee that good latency.
							// Parsing on the host side has to go always by magic word, NOT by counting samples.
	uint32_t	stepfreq;	// frequency increment (Hz)
	uint32_t	steps;		// how many steps to increment before going to next table entry
	uint16_t	magic_I;	// magic token - if three consecutive I and Q pairs are equal to these values, it indicates the beginning of the panoramic packet of particular kind
	uint16_t	magic_Q;
	uint16_t	skip;		// how many IQ pairs to skip after frequency change
	uint16_t	dummy;		// unpacked structure, so have it multiples of 32-bit (what it is anyway)
} PANENTRY;

#endif

