
 /*

 Revisions:

	v0.7b				-	Initial revision

	v1.0	13.06.2011	-	Data capture IRQ vector completly rewritten in assembler
						- 	Diversity mode support added
						-	Corrected FIR filter tables to prevent bleeding on waterfall edges
							(narrowed from 0.2375 bandwidth to 0.2175)
							[well, actally a standard table is used for production now]
						-	FIR filter tables are now gain-normalized to G=1.0 (was seriously overgain, G=2.3-2.3 earlier)
						-	Added 1/16 FIR tables to support 12kHz second channel when CH A is runing on 48kHz
						-	12kHz CH B is now working
						-	Added manual gain control option (agc, rfga, rfgb commands)
						-	ch a and b software gain control is now separated (attn! Exponent bit is common for both channels)
						-	Added command 'compressor', what fixes the exponent and sets gain to 1 (default is compressor off, gain fixed to 4)iqpktsize


	v1.1	28.06.2011	-	Audio type changed from type 0x5 to 0x1 (no sync) to get rid of sample skipping artifacts
						-	Fixed ugly bug with audio data endian. Noise floor is now below -70dB, AGC works.
							(There is still an artifact in audio above 10kHz what has to be figured out, although it is not audible)

=====================================================

	V1.5	06.09.2011	-	First cut for 32UC3B0256 running on EVK1101 (Direct port from V1.1) (not really doing anything useful other than compiles)
	V1.51	22.11.2011	-	Fixed LUFA bug, so CDC driver now works on 0256 chip
	V1.52	25.11.2011	-	Port to 0512 chip (First MK1.5 assembled! (Its 2.51 am :) ))
	V1.53	14.12.2011	-	(Entry added later 18.dec.2011)
						-	Microchip Ethernet (tuxgraphics) replaced with uIP, 100Base ethernet now works!
						-	All features on board tested, everything seems to work. Interrups are, however, too undeterministic to allow serial rates of F_ADC/2 (only /4 works (reg 5 3))
						-	Lot of minor fixes and tweaks. Still nothing more than answers to ping and transmits audio on single channel, but what works, works solid (except the routine ping miss after every minute)
	V1.54	19.12.2011	-	SSC data transfer is once again re-written and is DMA-based now.
	V1.55	21.01.2012	-	Added UDP support
						-	Started to clean up source code and splitting functionality between NetSDR.c, SDR-IQ.c and LM97593.c
						-	Descriptors_audio.c renamed to Descriptors.c
	V1.56	05.03.2012	-	Branched to new LUFA codebase (LUFA120219), removed all sorts of hooks to be compatible with "straight out of the box" LUFA
	V1.57	18.03.2012	-	Ugly bug in USB descriptor table finally found what prevented second audio card to work (Audio2_AudioFormatSampleRates missed '2' in a name ..). Works now (took 3 months and 4 days!!!)!!
						-	Removed USB audio sample rate control features for time being (USB attribute AUDIO_EP_SAMPLE_FREQ_CONTROL) to be on a safe side with Linux and MAC

	V1.57a	16.09.2012	-	Bulk data transfer finally works as supposed to after a lot of setbacks. Took a libusb to implement that and had to totally rework the transfer routine.
							The gain is 7.57Mbit/s steady maximum transfer rate. Still, there are frame synchronization problems what cause channels and I/Q to swap unexpectedly from time to time.

	// NB! 1.58 is the minimum version to be working with ExtIO DLL current release!
	V1.58	17.09.2012	-	Fixed the I/Q alignment issues, hopefully for good! (was SSC interface problem - had to be reset instead of just disabling and re-enabling)
						-	Added phase, gain and firmware version checks USB control messages
						-	SSC interrupt now deprecated
						-	LIBUSB_GETSYNC control message deprecated (was not used anywhere anyway)
						!	Diversity mode hooks missing for libusb mode
	V1.59	17.09.2012	-	Cleaned the correctionoffset hooks from firmware as redundant. Data rate is back on 7.57Mbit/s now.
						-	Fixed F_ADC frequencies for audio frequencies (calculation logic was erratic before, see SampleMode() comments)

	// =================
	// NB! 1.70 is the minimum version to be working with current ExtIO DLL release!
	// (Hopefully this is the last time when backward compatibility gets destroyed!)

	V1.70	20.09.2012	-	Redefined LIBMODE_xxxx flags to different values to match radio button states
	V1.71	23.09.2012	-	Added ResyncSI() function (a rework of SampleMode() code)
						-	Phase change is now followed by ResyncSI()
						-	Gain and Diversity stuff now works for libusb mode

	V1.80	29.10.2012	-	Networking mode works with CuteSDR! No network setup tho.
	V1.81	30.10.2012	-	Fixed bug what caused cutesdr to start up with wrong LO frequency at SDR side (frequency was not recalculated after sample rate change)
	V1.82	31.10.2012	-	Implemented UDP broadcast discover request response
	V1.83	08.11.2012	-	Added 256-byte NVRAM flash area at the end of flash (also updated linker relocation at avr32elf_uc3b0512.x acordingly)
						-	"nvram" command added to terminal for dumping NVRAM content
						-	Message() now calls CDC tasks to prevent data loss on long loops calling it
						-	Added commands dhcp {1|0}, ip {ipaddr}, gw {gateway}, netmask {netmask} and ipconfig
						-	tcpip config is now nvram-based
	V1.84	10.11.2012	-	UDP outbound setup is now dynamic (either uses ripaddress or address configured by control message 0xC5)
						-	DHCP works!
						-	Added unique serial number generation from CPU ID
						-	Endian swapping for UDP packets moved to background of SPI DMA transfer (ksz8851 has now non-blocking DMA packet transfer capability)
	V1.85	11.11.2012	-	Reworked SSC transfer engine to always use byte transfers (to simplify transitions between 16 and 24-bit modes for ntwork)
						-	24-bit mode is temporarily deprecated for audio and libusb modes (to be supported by network mode first)
						-	Added dynamic CDCE913 frequency calculation for SetADCClock() (were using pre-defined values and fixed tables before)
						-	Added command 'clock' to display current settings for CDCE913 chip

	V1.86	12.11.2012	-	Added default sample rate 196078 Hz for network modes to fix the cutesdr startup problem if no netsdr registry patch is applied
							(cutesdr does not set SDR sample rate for non-netsdr radios in some reason)

	V1.87	13.11.2012	-	Implemented netsdr control message 0xC4 and added MK1.5 specific data mode 0x80 (large packet, big endian mode)
	v1.88	14.11.2012	-	Implemented RF gain control message 0x38 for NetSDR protocol
						-	Shell commands gain control logic revisited

*/

#define	VERINFO		"v1.88"
#define VER_MAJOR	1
#define VER_MINOR	88

/*
 To Do:

		* SSC Frame syncing
		- VFO A/B callback (not yet implemented inside HDSDR either ...)
		* Synchronous tuning (ExtIO DLL change)
		* Clean help
		- Commands to case-insensitive (write generic parser)
		* manual gain control
		* Diversity Mode to finish
		* Diversity mode for libusb mode
		- Clean up source for publishing
		* separate ch a and ch b gain software gain control and add 'exp' command for forcing max gain through exponent
		? Figure out, why there is an intermodulation in 10kHz+ sinewawe
		x Implement sample rate feedback
		x Implement audio data capture and control through CDC device (SDR-IQ and ExtIO DLL compatible)
		- Finish SDR-IQ compatible prtocol (or deprecate officially?)
		* SampleMode() shall recalculate clock synthesizer to find best match for any given sample frequency
		* DHCP
		* NetSDR UDP broadcast discovery of radio
		- Connection dropping/cleanup if host is gone
		! Init_SSC() gets wrong frame length somewhere (garbage, actually!) during the init. This is likely because memory is overwritten somewhere,
		  but has to be tracked down!!
		* When first started, cutesdr selects frequency which is slightly off
		x SDR-IQ and SDR-14 sample frequency setup (works now, because all sample rates are dynamically calculated now)
		- MK1.5 specific udp packet mode (8k and reverse endian)
		- Wideband scan mode
		- 24-bit network modes support
		- Dual channel network modes
		! UDP packet source port is ignored at the moment (see uip.c line 1187), since we keep getting our DHCP response from DHCP server port 676 rahther than 67
		  This may be either a endianness bug somewhere or something else, but for time being we do not check source port addresses for UDP connections.
		! DHCP lease renews itself every 5 minutes. This is no good, but as a workaround, the ip address nullification is disabled, so the lease is likely going to be rnewed to a same address.
		  Works as a workaround for time being, but has to be reworked.
		- IPv6 support (far in the future ..)
		- NetSDR small and MK1.5 XLarge packet length support
 */

/** \file
 *
 *  Header file for AudioInput.c.
 */

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

void SampleMode(uint32_t samplefreq, uint16_t channels, uint16_t numbits);

void InitIQDataEngine(uint16_t bytsperframe);
void Init_SSC(uint16_t bytespeframe, int enablerx);

#define		DATA_AUDIO				1
#define		DATA_LIBUSB				2
#define		DATA_NETWORK			3
#define		DATA_SDRIQ				4
#define		DATA_LIBUSB_SPEEDTEST	5
#define		DATA_LIBUSB_XPY			6
#define		DATA_LIBUSB_XMY			7


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

#define		LIBUSB_CHA			0
#define		LIBUSB_CHB			1

// LIBUSB_MODE flags

#define		LIBMODE_16A			0	//A only
#define		LIBMODE_16B			1	//B only
#define		LIBMODE_16APB		2	//A+B
#define		LIBMODE_16AMB		3	//A-B
#define		LIBMODE_16BMA		4	//B-A
#define		LIBMODE_16AB		5	//A,B

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

#endif

