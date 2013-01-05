
/*

SDR MK1.5 'Andrus' SDR Firmware main body.

Written by Andrus Aaslaid (ES1UVB) (c)2010-2012, andrus.aaslaid(6)gmail.com

See revision history and comments at the sdr_mk1.5.h

Repository:
-----------
http://code.google.com/p/sdr-mk15/

License:
--------
Creative Commons Attribution-NonCommercial-ShareAlike (CC BY-NC-SA) License.
http://creativecommons.org/licenses/by-nc-sa/3.0/
Basically, you are free to remix, tweak, and build upon this work non-commercially, as long as you
credit the original work properly and license your new creations under the identical terms.
You have to, however, ask for permission if you are planning to use the material or
inherited works commercially.

*/

// This is from ASF package. Include this first, so we can undef the LUFA-incompatible macros
#include "compiler.h"

// Undef enidan stuff coming from ASF, as we need the ones from LUFA in this code
#undef le16_to_cpu
#undef cpu_to_le16
#undef LE16_TO_CPU
#undef CPU_TO_LE16
#undef le32_to_cpu
#undef cpu_to_le32
#undef LE32_TO_CPU
#undef CPU_TO_LE32

// ISR is also incompatible between LUFA and ASF, so discard the ASF version of it
//
//!! NOTE !! LUFA and ASF have similar Exception.S handlers, but use different calling convention.
// As we do use LUFA handlers for all exception / ISR stuff, no part of the ASF can be used what uses ASF interrupt handlers.
#undef ISR

#include "sdr_mk1.5.h"

//workaround of LUFA120219 bug
ISR(USB_GEN_vect);

#include "uip.h"
#include "network.h"
#include "uip_arp.h"
#include "dhcpc.h"

#include "NetSDR.h"
#include "SDR-IQ.h"
#include "terminal.h"
#include "omnirig.h"

#include "clock-arch.h"

// Remaining stuff from AFS
#include "compiler.h"
#include "gpio.h"
#include "spi.h"
#include "ssc_i2s.h"
#include "power_clocks_lib.h"
#include "pdca.h"
#include "cycle_counter\cycle_counter.h"
#include "flashc.h"

#include "pm.h"

#include "twi.h"
#include "eth_spi.h"
#include "LM97593.h"

#include "cdce913_coeff.h"

#include <string.h>
#include <stdarg.h>

//#include "adOS\adOS.h"


/** LUFA Audio Class driver interface configuration and state information. This structure is
 *  passed to all Audio Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_Audio_Device_t Main_Audio_Interface =
	{
		.Config =
			{
				.StreamingInterfaceNumber = AUDIO_STREAM_A_INTERFACE+1,

				.DataINEndpointNumber     = AUDIO_STREAM_A_EPNUM,
				.DataINEndpointSize       = AUDIO_STREAM_A_EPSIZE,
			},
	};

USB_ClassInfo_Audio_Device_t AudioB_Audio_Interface =
	{
		.Config =
			{
				.StreamingInterfaceNumber = AUDIO_STREAM_B_INTERFACE+1,

				.DataINEndpointNumber     = AUDIO_STREAM_B_EPNUM,
				.DataINEndpointSize       = AUDIO_STREAM_B_EPSIZE,
			},
	};



/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another. This is for the first CDC interface,
 *  which sends strings to the host for each joystick movement.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial1_CDC_Interface =
{
	.Config =
		{
			.ControlInterfaceNumber           = SER_FIRST,

			.DataINEndpointNumber             = CDC1_TX_EPNUM,
			.DataINEndpointSize               = CDC1_TX_EPSIZE,
			.DataINEndpointDoubleBank         = false,

			.DataOUTEndpointNumber            = CDC1_RX_EPNUM,
			.DataOUTEndpointSize              = CDC1_RX_EPSIZE,
			.DataOUTEndpointDoubleBank        = false,

			.NotificationEndpointNumber       = CDC1_NOTIFICATION_EPNUM,
			.NotificationEndpointSize         = CDC1_NOTIFICATION_EPSIZE,
			.NotificationEndpointDoubleBank   = false,
		},
};

USB_ClassInfo_CDC_Device_t VirtualSerial2_CDC_Interface =
{
	.Config =
		{
			.ControlInterfaceNumber           = SER_SECOND,

			.DataINEndpointNumber             = CDC2_TX_EPNUM,
			.DataINEndpointSize               = CDC2_TX_EPSIZE,
			.DataINEndpointDoubleBank         = true,

			.DataOUTEndpointNumber            = CDC2_RX_EPNUM,
			.DataOUTEndpointSize              = CDC2_RX_EPSIZE,
			.DataOUTEndpointDoubleBank        = false,

			.NotificationEndpointNumber       = CDC2_NOTIFICATION_EPNUM,
			.NotificationEndpointSize         = CDC2_NOTIFICATION_EPSIZE,
			.NotificationEndpointDoubleBank   = false,
		},
};

void Message(int16_t cdc, char* outstring, ...)
{
va_list argptr;
char lineout[128];		// not safe, but we are embedded, so there is a little chance anyone is going to hack us this way ..
uint8_t PrevEndpoint;
static uint16_t bytesgiven=0;

	PrevEndpoint = Endpoint_GetCurrentEndpoint();		// always has to be first and in the primary scope!

	va_start(argptr, outstring);
	vsprintf(lineout,outstring, argptr);
	va_end(argptr);

	if (cdc==1)
	{
		bytesgiven+=strlen(lineout);

		CDC_Device_SendString(&VirtualSerial1_CDC_Interface, lineout/*, strlen(lineout)*/);

		// do CDC interface tasks, so we will not loose data on case of long messages (in some reason Hercules still looses symbols if the queue is longer than 20 bytes!)
		if (bytesgiven > 20)
		{
			CDC_Device_USBTask(&VirtualSerial1_CDC_Interface);
			bytesgiven=0;
		}
	}

	Endpoint_SelectEndpoint(PrevEndpoint);
}

volatile bool reginit_done=false;
volatile bool usbinit_done=false;

volatile bool radio_ready=false;
volatile uint32_t testsamples=0;
volatile bool external_power=false;
volatile unsigned char LSB=0;

volatile bool omnirig=false;					// omnirig command mode
volatile bool firstsync=true;					// used for syncing with SDR-RADIO

volatile uint16_t gain=5;						// total from 0..15

volatile unsigned char pollmode=0;

volatile uint8_t AGCMode=1;

volatile unsigned char activeVFO='0';			// used for sdr-radio VFO focus keeping

int32_t phase_B=0;

int16_t fixedgain_A=6, fixedgain_B=6;					// used just for displaying

uint16_t plen;

uint8_t datamode = DATA_AUDIO;					// default to audio interface

uint16_t panadapter = 0;						// will be set to LIBMODE_16ABPAN or LIBMODE_16BAPAN in case panadapter is enabled
uint16_t panentries = 0;						// number of entries (PANENTRY) on panoramic scan table
char* pantable = NULL;							// this will host our panoramic scan regions table (PANENTRY list)
PANENTRY* panentry;

uint16_t stepsdone = 0;
uint16_t currentpanentry = 0;

volatile uint32_t SSCSFSIntCounter=0;
volatile uint32_t IntTime=0;					// first IRQ execution time (milliseconds from start)

int32_t SampleRate=SDR_SAMPLE_FREQUENCY;		// start as 48k by default
//int16_t AudioBits=AUDIOBITS;
//int16_t WordSize=AUDIOBITS/8;					// these are 1:1 with AudioBits, but we are keeping two constants to avoid calculations on the fly

uint32_t f_adc=0;								// global variable to hold ADC clock value

uint32_t samplesadded=0;						// number of samples added to audio buffer (if the calculated sample rate is less than expected)

int iqpktsize=-1;								// in SDR-IQ mode, tracks the 8192 byte payload.

extern struct uip_udp_conn *uip_udp_conn;		// this is our master UDP connection what uIP is usingf
extern struct uip_udp_conn *udp_conn;			// this is a UDP connection what we have initialized at NetSDR.c at control item code 0x0018

extern struct dhcpc_state s;	// used for debug logging only

unsigned char SDRSerial[9];		// SDR Unique serial number

extern uint8_t netiqdump;		// if set to 1, NetSDR_Task() will print out head of the IQ data buffer

extern uint16_t NetDataPackets;
extern uint16_t NetPktLen;
extern uint16_t NetDataLen;					// indicates, how much data in network mode SSC will fetch us for one buffer
extern uint16_t NetDataLenHalfwords;		// indicates, how much data in network mode SSC will fetch us for one buffer (used because we need it inside IRQ and do not have to divide by two each time)

extern uint8_t ReverseEndian;				// indicates if IQ data endian has to be swaooed at NetSDR_Task()

uint16_t BitDepth = AUDIOBITS;							// 16 or 24, dependent on configuration, but start with 16-bit to be compatible with audio

union pll_conf pllconf;							// clock parameters calculated by SetADCClock()
uint32_t real_f_adc;
uint32_t f_adc_divider;

//This a 256 byte data area at flash (reserved through linker definition file avr32elf_uc3b0512.x)
__attribute__((__section__(".flash_nvram")))
nvram_data_t flash_nvram_data
;

//
// CRC Calculation for storing TCPIP parameters in flash.
// The particular implementation is CCITT-16 (0x1021, x^16 + x^12 + x^5 + 1)

uint16_t UpdateCRC16(uint16_t crc, uint8_t databyte)
{
int i;

	crc ^= databyte << 8;
	for (i = 0; i < 8; ++i)
	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
	return crc;
}


void delayms(uint16_t ms)
{
	cpu_delay_ms(ms, F_CPU);
}

// debug info from Endpoint_UC3.c
extern uint8_t eporder[];
extern int __i;

// debug info from Descriptors_Audio.c
extern uint8_t windexes[];
extern int __j;

void ShowHelp(void)
{
//int i;

	Message(1, "       Commands:\n\r");
	Message(1, "-----------------\n\r");
	//Message(1, "            pon - turn on power for radio circuit\r\n");
	//Message(1, "           poff - turn off radio circuit power\r\n");
	Message(1, "        intcount - display audio interrupt counter\r\n");
	Message(1, "         regdump - dump LM97593 registers\r\n");
	Message(1, "       audiotest - output test samples\r\n");
	Message(1, " reg <reg> <val> - set register reg to value val\r\n");
	Message(1, "        defaults - refresh registers to default state\r\n");
	Message(1, "        scancode - displays the scan code of the next key pressed\r\n");
	//Message(1, "          reset - assert reset on LM97593\r\n");
	//Message(1, "             mc - perform Montecarlo on LM97593 registers\r\n");
	Message(1, " fa/fb <freq_hz> - Set Channel A/B tuning frequency in Hz\r\n");
	Message(1, "            freq - Display channel frequencies\r\n");
	Message(1, "    ga/gb <gain> - Set Channel A/B Software gain (0..15; default=7)\r\n");
	Message(1, "rfga/rfgb <gain> - Set Channel A/B RF gain (0..7; default=6)\r\n");
	Message(1, "       agc <1|0> - Turn Automatic Gain Control ON or OFF (default=OFF)\r\n");
	Message(1, "compressor <1|0> - Fixes the exponent and sets gain to 1 (default=ON (gain=8))\r\n");
	Message(1, "              si - Assert xSI line to get the LM97593 internals synced\r\n");
	Message(1, " diversity <1|0> - Enable or Disable diversity mode (default=OFF)\r\n");
	Message(1, "  pollmode <1|0> - Enable or Disable CAT polling mode (default=OFF)\r\n");
	//Message(1, "     complement - Change input data conversion from two's complement\r\n");
	Message(1, "\r\n");
	Message(1, "            help - display this help screen\r\n");
	Message(1, "         7/4;8/5 - Ch.A freq up/down; Ch.B freq up/down\r\n");
	Message(1, "             9/6 - Change Channel B frequency angle for divereity mode\r\n");

	Message(1, "\r\n");
	Message(1, "Current parameters:\r\n");
	Message(1, "------------------------------------\r\n");
	Message(1, "           LO CH_A = %lu Hz\r\n", lastfreq_A);
	Message(1, "           LO CH_B = %lu Hz\r\n", lastfreq_B);
	Message(1, "               AGC = %s\r\n", (AGCMode)?"ON":"OFF");
	Message(1, "        Compressor = %s\r\n", (ReadRegister(20)&1)?"ON":"OFF");

	if (!AGCMode)
	{
	Message(1, "      RF Gain CH_A = %d\r\n", fixedgain_A);
	Message(1, "      RF Gain CH_B = %d\r\n", fixedgain_B);
	}

	Message(1, "Software Gain CH_A = %d (exponent %s)\r\n", ReadRegister(3)+((ReadRegister(20)&1)*8),(ReadRegister(20)&1)?"fixed":"passed");
	Message(1, "Software Gain CH_B = %d (exponent %s)\r\n", ReadRegister(4)+((ReadRegister(20)&1)*8),(ReadRegister(20)&1)?"fixed":"passed");
	Message(1, "  CAT polling mode = %s\r\n", (pollmode)?"ON":"OFF");

	Message(1, "Endpoint A: %d Endpoint B: %d MaxEP: %d\r\n", AUDIO_STREAM_A_EPNUM, AUDIO_STREAM_B_EPNUM, ENDPOINT_TOTAL_ENDPOINTS);
/*
	Message(1, "Endpoint config order: ");

	for (i=0; ((i<__i)) && (i<16); i++)
		Message(1, "%d ", eporder[i]);

	Message(1, "(%d calls total)\r\n", __i);
	if (__i > 16)
		Message(1, "ERROR!! Endpoint order buffer overflow! (%d instead of 16 max)\r\n", __i);

	Message(1, "wIndexes accessed: ");

	for (i=0; ((i<__j)) && (i<64); i++)
		Message(1, "%d ", windexes[i]);

	Message(1, "(%d accesses total)\r\n", __j);
	if (__j > 64)
		Message(1, "ERROR!! wIndexes access buffer overflow! (%d instead of 64 max)\r\n", __j);
*/
}

void ShowMenu(void)
{
static bool firsttime=true;

	if (firsttime)
	{
		firsttime=false;
		Message(1, "SDR MK1.5 Console %s (%s)\n\r", VERINFO, __DATE__);
		Message(1, "(c)UVB-76.net 2011\n\r");
		Message(1, "---------------------------------\n\r\n\r");

		ShowHelp();
	}
}

void DumpRegisters(void)
{
int16_t i, j;

	Message(1, "  Register        Addr.     Value\r\n");
	Message(1, "-------------------------------------------------\r\n");

	for (i=0; i<REGENTRIES; i++)
	{
		if (regstruct[i].flags & R_DISPLAY)	// only dump registers what have flag set as 1
		{
			Message(1, "%s | %3.3d (0x%02.2X) | ", regstruct[i].description, regstruct[i].regaddr, regstruct[i].regaddr);

			for (j=0; j<regstruct[i].datalen; j++)
			{
				Message(1, "%02.2X ", ReadRegister(regstruct[i].regaddr+j));
				if (!((j+1)%8))
					Message(1, "\r\n             |            | ");
			}

			Message(1, "\r\n");
		}
	}
}


// SSC interface (will be initialized through SetupHardware())
volatile avr32_ssc_t *ssc = &AVR32_SSC;

static const gpio_map_t TWI_GPIO_MAP =
{
	{AVR32_TWI_SDA_0_0_PIN, AVR32_TWI_SDA_0_0_FUNCTION},
	{AVR32_TWI_SCL_0_0_PIN, AVR32_TWI_SCL_0_0_FUNCTION}
};

//
// Thanx to jkuusama, gchapman, noob64 and Commanderbob for posting their EVK110x SPARE_SPI code snippets to the avrfreaks!
// The one supplied with the ASF as spi master example does not seem to work at all
//

static const gpio_map_t ETH_SPI_GPIO_MAP =
{
	{ETH_SPI_SCK_PIN,          ETH_SPI_SCK_FUNCTION         },  // SPI Clock.
	{ETH_SPI_MISO_PIN,         ETH_SPI_MISO_FUNCTION        },  // MISO.
	{ETH_SPI_MOSI_PIN,         ETH_SPI_MOSI_FUNCTION        },  // MOSI.
	{ETH_SPI_NPCS_PIN,		   ETH_SPI_NPCS_FUNCTION		}	// Chip Select NPCS0
};


static spi_options_t spiOptions =
{
	.reg          = 0,	// 2 for CS2 does not make any sense, but works! //SPI to setup, so 0 for SPI0, 1 for SPI1 etc..
	.baudrate     = SPI_ETH_BAUDRATE,		//Be aware, if this is too high or too low the compiler won't complain, many SPI functions will run without failing, even though the SPI is not working.
	.bits         = 8,						//8 bits for sending chars, 16 bits for shorts, I think it can be set to 11 or 13 etc. don't see why you'd want to do that though.
	.spck_delay   = 4,						//Delay before first clock pulse after selecting slave (in PBA clock periods). (50ns specified for enc28j60 = 1/60MHz*4 = 66ns)
	.trans_delay  = 0,						//Delay between each transfer/character (in PBA clock periods) (not needed for enc28j60)
	.stay_act     = 1,						//Sets this chip to stay active after last transfer to it.
	.spi_mode     = 0,						//Which SPI mode to use when transmitting.
	.modfdis      = 1						//Errata Workaround: Disable mode fault detection by writing a one to MR.MODFDIS
};


uint16_t SetADCClock(uint32_t adcclock)
{
uint16_t retval;
uint32_t clock;
uint32_t divider, bestdivider;
uint32_t freqdiff;

	retval=0;

	//Config Registers, common
	//twi_write(0, 0x1); do not overwrite
	//twi_write(1, 0); do not overwrite -- eeprom lock bit is here!
	twi_write(2, 0xB4);
	twi_write(4, 0x2);
	//twi_write(5, 12<<2); already set
	//twi_write(6, 0x40); eeprom write

	// PLL registers, common
	twi_write(16, 0);		//x10
	twi_write(17, 0);		//x11
	twi_write(18, 0);		//x12
	twi_write(19, 0);		//x13
	twi_write(20, 0x4D);	//x14
	twi_write(21, 0x2);		//x15
	twi_write(22, 0);		//x16
	twi_write(23, 0);		//x17

	twi_write(28, 0);		//x1C
	twi_write(29, 0x40);	//x1D
	twi_write(30, 0x2);		//x1E
	twi_write(31, 0x8);		//x1F

	// Fvco can be only in a range between 80 and 230MHz. Therefore we can not always use output divider 1, but have to find the divider what will
	// put the Fvco in one of the correct ranges: 0: 80MHz >= Fvco < 125MHz; 1: 125MHz <= Fvco < 150MHz; 2: 150MHz <= Fvco < 175MHz; 3: 175MHz <= Fvco < 230MHz;

	for (bestdivider=1, freqdiff=0xFFFFFFFF, divider=(1+(80000000/adcclock)); divider<=(230000000/adcclock); divider++)
	{
		pllconf=find_coeffs(adcclock*divider, &real_f_adc);
		if (freqdiff > uabssub(adcclock, real_f_adc/divider))
		{
			freqdiff = uabssub(adcclock, real_f_adc/divider);
			bestdivider=divider;
		}
	}

	f_adc_divider=bestdivider;
	pllconf=find_coeffs(adcclock*f_adc_divider, &real_f_adc);

	real_f_adc/=f_adc_divider;

	//Config Registers
	twi_write(3, f_adc_divider);
	// PLL registers
	twi_write(24, pllconf.darr[0]);	//x18
	twi_write(25, pllconf.darr[1]);	//x19
	twi_write(26, pllconf.darr[2]);	//x1A
	twi_write(27, pllconf.darr[3]);	//x1B

	f_adc=adcclock;

	return 1;
}

#define OSC0_STARTUP    AVR32_PM_OSCCTRL0_STARTUP_2048_RCOSC  //!< Osc0 startup time: RCOsc periods.

static pcl_freq_param_t pcl_freq_param =
{
	.cpu_f=F_CPU,
	.pba_f=F_PBA_SPEED,
	.osc0_f=FOSC0,
	.osc0_startup=OSC0_STARTUP
};

int16_t SetupHardware(void)
{
	//for early versions, we had CPClare solid state relay on board. The idea was to allow board come up in minimalist power mode and then
	//gradually power it up. However, this was not really making any sense, as there was so much power leaking through all the clamping
	//diodes, that the relay was more or less selecting between a full power and brown-out region, not on and off. Therefore, for new
	//boards the relay is bypassed and for older (rev 1.03) boards we will enable the relay as a first thing after power-up.
	//
	//The side-effect is, that external power drains to USB interface if it exceeds the USB voltage level, so for newer yet boards
	//the relay is put back in action (CPC1020). The code here has to be modified therefore at some point to detect external power!
	//(the new boards have bootstrap pull-up at A3 address pin to indicate that the relay is on the board!)
	gpio_set_gpio_pin(BUSPOWER);

	//clear SDRENABLE, so BUSPOWER would not power the entire board immediately
	//this is also important to do immediately for a reason, that if we do not configure it as output immediately, the
	//RED LED will drain enough current from the pin, that it will become high and the whole board starts fetching power
	//through protection diodes, which is not healthy.
	gpio_set_gpio_pin(SDRENABLE);

	//pcl_switch_to_osc(PCL_OSC0, FOSC0, OSC0_STARTUP);

	pcl_configure_clocks(&pcl_freq_param);

	/*
	while (!(AVR32_PM.poscsr & AVR32_PM_CKRDY_MASK)) {};			// wait for access
	AVR32_PM.cksel&=~(AVR32_PM_PBADIV_MASK|AVR32_PM_PBASEL_MASK);	// we may be overclocking the CPU to 66MHz and have to override PBA divisor therefore, as its not clear
																	// what ASF will set the divisor if the frequency exceeds maximum defined by AVR32_PM_PBA_MAX_FREQ (=60MHz)
	*/

	//pm_cksel(&AVR32_PM, 0, 0, 0, 0, 0, 0	);	// all clocks undivided

	pcl_configure_usb_clock();

	// Initialize interrupt subsystem
	INTC_Init();
	INTC_RegisterGroupHandler(INTC_IRQ_GROUP(AVR32_USBB_IRQ), AVR32_INTC_INT0, USB_GEN_vect);

	GlobalInterruptEnable();

	// TWI gpio pins configuration
	gpio_enable_module(TWI_GPIO_MAP, sizeof(TWI_GPIO_MAP) / sizeof(TWI_GPIO_MAP[0]));
	twi_init();

	// Now when the processor and DVGA amps definitely do have proper power,
	// allow power to stabilize and caps to get loaded.
	// Also, prevent obsessive looping if USB can not provide all the power necessary if
	// full power is enabled.
	delayms(1000);

	//gpio_set_pin_high(SDRENABLE);		//SDRENABLE high - enable all power now
	//delayms(10);						// allow power to stabilize somewhat before programming clock synth

	// Program the clock synth as first thing after power-up to avoid running LM97593 without clock after reset.
	// We should be done before the LM97593 chip comes out of reset

	twi_write(5, 11<<2);		// set crystal loading capacitor value
	//twi_write(5, 0);
	//twi_write(1, 0x9);				// set external LVCMOS clock as input. ATTN!!!! Do not touch bit 5, as this will permanently lock the EEPROM config!

	SetADCClock(FREQ_64000000);		// set default ADC clock to 64MHz

	// allow power to stabilize some more and everything boot up after power is applied

	delayms(500);

	powered=true;

	gpio_configure_group(PORTA, address_bitmask, GPIO_DIR_INPUT);						// configure addresses as inputs for now, as we have button connected to the HWBE line and therefore
																						// we will configure it as output only on data transfer to minimize possibilities for shorting the output
	gpio_configure_group(PORTA, data_bitmask, GPIO_DIR_INPUT);							// configure data as inputs for now
	gpio_configure_group(PORTB, ctrl_bitmask, GPIO_DIR_OUTPUT|GPIO_INIT_HIGH);			// configure control pins for LM97593 control data and set high (no chipselect, read or write)
	gpio_set_pin_low(xCE);																// keep the LM97593 always chipselected, as there is nothing else on the bus

	gpio_set_gpio_pin(SI);																// set SI as output and set high

    // Assign I/Os to SPI.
	gpio_enable_module(ETH_SPI_GPIO_MAP, sizeof(ETH_SPI_GPIO_MAP) / sizeof(ETH_SPI_GPIO_MAP[0]));

	// Initialize as master.
	spi_initMaster(ETH_SPI, &spiOptions);
	spi_setupChipReg(ETH_SPI, &spiOptions, F_PBA_SPEED);
	// AVR32_SPI.CSR0.scbr = 2;		// we need 30MHz - not exactly sure what the clock calculator was giving us at the SPI init, so overwriting the parameters here
	// Set SPI selection mode: [variable_ps (address slave for every byte), pcs_decode (external decoder), delay (chipselect delay)].
	spi_selectionMode(ETH_SPI, 0, 0, 0);
	spi_enable(ETH_SPI);

	//enc28j60Init(mymac);

	//webservertest();

	return(0);
}


int16_t errled=0;
int16_t cfgled=0;

/*

Experimental stuff for using adOS cooperative multitasker. Although this has never been tested, its left here for future experimentation

#define USB_TASK_PRIO	0
volatile ados_tcb_t g_USBTcb;
unsigned char g_USBTaskStack[1024]   = {0xE7};

#define DEMO_TASK_PRIO	128
volatile ados_tcb_t g_demoTcb;
unsigned char g_demoTaskStack[128]  = {0xE7};


void USBTask(void)
{
	ados_sleep(500); //give time for rest of tasks to start
	while(1)
	{
		CDC_Device_USBTask(&VirtualSerial1_CDC_Interface);
		Audio_Device_USBTask(&Main_Audio_Interface);
		Audio_Device_USBTask(&AudioB_Audio_Interface);

		ados_sleep(0);		// trigger immediately
	}
}

void demoTask(void)
{
	ados_sleep(500);	// allow other tasks to start

	while(1)
	{
		gpio_toggle_pin(LED);
		ados_sleep(1000);		// sleep 1 sec
	}
}
*/

/*

 a full cycle, 16-bit, 2's complement sine wave lookup table
 for a test tone generator

 Table from DAC1_fgen1.c
  -----------------------------------------------------------------------------

   AUTH: BW,FB
   DATE: 2 OCT 01

*/
/*
volatile int16_t SINE_TABLE[256] = {

   0x0000, 0x0324, 0x0647, 0x096a, 0x0c8b, 0x0fab, 0x12c8, 0x15e2,
   0x18f8, 0x1c0b, 0x1f19, 0x2223, 0x2528, 0x2826, 0x2b1f, 0x2e11,
   0x30fb, 0x33de, 0x36ba, 0x398c, 0x3c56, 0x3f17, 0x41ce, 0x447a,
   0x471c, 0x49b4, 0x4c3f, 0x4ebf, 0x5133, 0x539b, 0x55f5, 0x5842,
   0x5a82, 0x5cb4, 0x5ed7, 0x60ec, 0x62f2, 0x64e8, 0x66cf, 0x68a6,
   0x6a6d, 0x6c24, 0x6dca, 0x6f5f, 0x70e2, 0x7255, 0x73b5, 0x7504,
   0x7641, 0x776c, 0x7884, 0x798a, 0x7a7d, 0x7b5d, 0x7c29, 0x7ce3,
   0x7d8a, 0x7e1d, 0x7e9d, 0x7f09, 0x7f62, 0x7fa7, 0x7fd8, 0x7ff6,

   0x7fff, 0x7ff6, 0x7fd8, 0x7fa7, 0x7f62, 0x7f09, 0x7e9d, 0x7e1d,
   0x7d8a, 0x7ce3, 0x7c29, 0x7b5d, 0x7a7d, 0x798a, 0x7884, 0x776c,
   0x7641, 0x7504, 0x73b5, 0x7255, 0x70e2, 0x6f5f, 0x6dca, 0x6c24,
   0x6a6d, 0x68a6, 0x66cf, 0x64e8, 0x62f2, 0x60ec, 0x5ed7, 0x5cb4,
   0x5a82, 0x5842, 0x55f5, 0x539b, 0x5133, 0x4ebf, 0x4c3f, 0x49b4,
   0x471c, 0x447a, 0x41ce, 0x3f17, 0x3c56, 0x398c, 0x36ba, 0x33de,
   0x30fb, 0x2e11, 0x2b1f, 0x2826, 0x2528, 0x2223, 0x1f19, 0x1c0b,
   0x18f8, 0x15e2, 0x12c8, 0x0fab, 0x0c8b, 0x096a, 0x0647, 0x0324,

   0x0000, 0xfcdc, 0xf9b9, 0xf696, 0xf375, 0xf055, 0xed38, 0xea1e,
   0xe708, 0xe3f5, 0xe0e7, 0xdddd, 0xdad8, 0xd7da, 0xd4e1, 0xd1ef,
   0xcf05, 0xcc22, 0xc946, 0xc674, 0xc3aa, 0xc0e9, 0xbe32, 0xbb86,
   0xb8e4, 0xb64c, 0xb3c1, 0xb141, 0xaecd, 0xac65, 0xaa0b, 0xa7be,
   0xa57e, 0xa34c, 0xa129, 0x9f14, 0x9d0e, 0x9b18, 0x9931, 0x975a,
   0x9593, 0x93dc, 0x9236, 0x90a1, 0x8f1e, 0x8dab, 0x8c4b, 0x8afc,
   0x89bf, 0x8894, 0x877c, 0x8676, 0x8583, 0x84a3, 0x83d7, 0x831d,
   0x8276, 0x81e3, 0x8163, 0x80f7, 0x809e, 0x8059, 0x8028, 0x800a,

   0x8000, 0x800a, 0x8028, 0x8059, 0x809e, 0x80f7, 0x8163, 0x81e3,
   0x8276, 0x831d, 0x83d7, 0x84a3, 0x8583, 0x8676, 0x877c, 0x8894,
   0x89bf, 0x8afc, 0x8c4b, 0x8dab, 0x8f1e, 0x90a1, 0x9236, 0x93dc,
   0x9593, 0x975a, 0x9931, 0x9b18, 0x9d0e, 0x9f14, 0xa129, 0xa34c,
   0xa57e, 0xa7be, 0xaa0b, 0xac65, 0xaecd, 0xb141, 0xb3c1, 0xb64c,
   0xb8e4, 0xbb86, 0xbe32, 0xc0e9, 0xc3aa, 0xc674, 0xc946, 0xcc22,
   0xcf05, 0xd1ef, 0xd4e1, 0xd7da, 0xdad8, 0xdddd, 0xe0e7, 0xe3f5,
   0xe708, 0xea1e, 0xed38, 0xf055, 0xf375, 0xf696, 0xf9b9, 0xfcdc,
};

uint32_t sinetable_ptr = (uint32_t)&SINE_TABLE;
volatile uint32_t sineptra=0, sineptrb=0;

*/


//#if (AUDIOBITS == 16)
//volatile int16_t dmabuff_1[512] __attribute__((aligned(0x8)));
//#else
//volatile int32_t dmabuff_1[512] __attribute__((aligned(0x10)));
//#endif

#define DMAMASK		0x3FFFU	//0x1FFF	//0x7FF
//volatile uint16_t dmabuff_1[(2*(DMAMASK+1))+2] __attribute__((aligned(0x20)));		// Align by 0x20. Makes it 0x8 aligned as well as needed by 16-bit mode, but at 16-bit single channel bulk mode we need to be
																					// sure that we have 32 bytes ahead of us. Get int32_t*(DMAMASK+1) worth of memory, but allocate as int16_t to make the
																					// dmabuff_1[] addressing simple for 16-bit samples for audio output (24-bit (using 32-bit width) goes bizarre tho ... )
																					//
																					// Also, note the 2-word safety margin here, see SampleMode() toget an idea, why!

volatile uint16_t dmabuff_1[DMAMASK+1] __attribute__((aligned(0x20)));			// Audio data buffer Allocated as 16-bit ints, so it makes life easier for buffer addressing when outputing audio data.
																				// To prevent partial transfers, we are aligning the buffer by 32 bytes (the longest look-ahead we shall have on case we are
																				// sending 4 consecutive 16-bit I/Q pairs from channel B is 16 words)

// these are used as displacement pointers to speed up working loops
volatile uint16_t* dmabuff_w1;
volatile uint16_t* dmabuff_w2;
volatile uint16_t* dmabuff_w3;
volatile uint16_t* dmabuff_w4;

volatile uint16_t* dmabuff_x1;
volatile uint16_t* dmabuff_x2;
volatile uint16_t* dmabuff_x3;
volatile uint16_t* dmabuff_x4;
volatile uint16_t* dmabuff_x5;
volatile uint16_t* dmabuff_x6;
volatile uint16_t* dmabuff_x7;
volatile uint16_t* dmabuff_x8;

volatile uint16_t* dmabuff_y1;
volatile uint16_t* dmabuff_y2;
volatile uint16_t* dmabuff_y3;
volatile uint16_t* dmabuff_y4;
volatile uint16_t* dmabuff_y5;
volatile uint16_t* dmabuff_y6;
volatile uint16_t* dmabuff_y7;
volatile uint16_t* dmabuff_y8;

// indexes are 32-bit to make work easier with registers
volatile uint32_t woffsetx;
volatile uint32_t roffseta=0;
volatile uint32_t roffsetb=0;
volatile uint32_t displacement=8;

volatile avr32_pdca_channel_t *pdca_channel;

uint32_t setsync=0, syncerr=0;

// These DMA buffers are used to transmit data through UDP. Since we need to do a DMA from SSC directly to this buffer and we need two of them,
// we will create auxiliary uip_sdata

// have to align netdmabuffs 4, since we use some uint32 mathematics on them later to speed up byte swapping (well, actually we do not today ...)
volatile uint8_t uipbuffA[UIP_LLH_LEN + UIP_IPUDPH_LEN + 2 + 2 + NETMAXDATA] __attribute__((aligned(0x4)));			// ethernet headers + 2-byte netsdr data packet header + 2-byte sequence number + 6084-byte data
volatile uint8_t uipbuffB[UIP_LLH_LEN + UIP_IPUDPH_LEN + 2 + 2 + NETMAXDATA] __attribute__((aligned(0x4)));			// ethernet headers + 2-byte netsdr data packet header + 2-byte sequence number + 6084-byte data

volatile uint8_t *netdmabuffA = &uipbuffA[UIP_LLH_LEN + UIP_IPUDPH_LEN + 2 + 2];	 // netdmabuffs become aligned, since the UDP header stuff, netsdr header and sequence are alltogether 32 bytes long
volatile uint8_t *netdmabuffB = &uipbuffB[UIP_LLH_LEN + UIP_IPUDPH_LEN + 2 + 2];

volatile int8_t nextisA=-1;

volatile uint8_t ready2udp=0;

// happens with every frame sync
/*
Deprecated, as syncing is now taken care ny SampleMode()

ISR(ssc_int_handler)
{
	SSCSFSIntCounter++;							// increment sample IRQ count

	ssc->idr=(unsigned long)0xFFFFFFFF;			// cant afford running the interrupt all the time, so disable
	AVR32_SSC.sr;								// end interrupt
}
*/

static const gpio_map_t SSC_GPIO_MAP =
{
	{AVR32_SSC_RX_CLOCK_0_0_PIN,      AVR32_SSC_RX_CLOCK_0_0_FUNCTION     },
	{AVR32_SSC_RX_DATA_0_0_PIN,       AVR32_SSC_RX_DATA_0_0_FUNCTION      },
	{AVR32_SSC_RX_FRAME_SYNC_0_0_PIN, AVR32_SSC_RX_FRAME_SYNC_0_0_FUNCTION}
};

// PDCA channel options
static pdca_channel_options_t PDCA_OPTIONS =
{
	.addr = (void *)dmabuff_1,						// memory address
	.pid = AVR32_PDCA_PID_SSC_RX,					// select peripheral
	.size = (DMAMASK+1)*2,							// transfer counter
	.r_addr = (void *)dmabuff_1,					// next memory address (use the same, so we do have a ring buffer)
	.r_size = (DMAMASK+1)*2,						// next transfer counter
	.transfer_size = PDCA_TRANSFER_SIZE_BYTE		// select size of the transfer
};

ISR(pdca_int_handler_0)
{
	// When TCR reaches zero, it will be reloaded with TCRV if TCRV has a positive value. If TCRV is zero, no more transfers
	// will be performed for the channel. When TCR is reloaded, the TCRR register is cleared, so we have to reload it inside interrupt.
	// Same applies to marr

	if (datamode == DATA_NETWORK)
	{
		// For network we are using different buffering system than all other modes.
		// There are two different buffers in use, both incorporating 6088 bytes of data (the largest multiple of 4 fitting inside (6*1024)-54 (max KSZ8851 buffer - header lenght))

		if (nextisA == 1)
		{
			// update SCC DMA immediately to not loose any data
			// note, that tcrr has to be reloaded after marr, since loading tcrr will automatically update tcr and this will trigger transfer immediately
			pdca_channel->marr = (unsigned long)netdmabuffA;
			pdca_channel->tcrr = NetDataLen;
			nextisA=0;
			ready2udp=1;		//signal NetSDR_Task() that it is time to transmit
		}
		else if (nextisA == 0)
		{
			pdca_channel->marr = (unsigned long)netdmabuffB;
			pdca_channel->tcrr = NetDataLen;
			nextisA=1;
			ready2udp=1;
		}
		else // first pass, so set up DMA but do not transmit
		{
			pdca_channel->marr = (unsigned long)netdmabuffA;
			pdca_channel->tcrr = NetDataLen;
			nextisA=0;
		}
	}
	else
	{
		// note, that tcrr has to be reloaded after marr, since loading tcrr will automatically update tcr and this will trigger transfer immediately
		pdca_channel->marr = (unsigned long)dmabuff_1;
		pdca_channel->tcrr = (DMAMASK+1)*2;
	}

	//ssc->ier=AVR32_SSC_IER_RXSYN_MASK;			// once per buffer fill enable SSC SOF interrupt (ISR is currently deprecated and vector is not set either!)

	pdca_channel->isr;
}

void Init_SSC(uint16_t bytesperframe, int enablerx)
{
	// Enable internal pullups for SSC signals
	gpio_enable_pin_pull_up(AVR32_SSC_RX_DATA_0_0_PIN);
	gpio_enable_pin_pull_up(AVR32_SSC_RX_CLOCK_0_0_PIN);
	gpio_enable_pin_pull_up(AVR32_SSC_RX_FRAME_SYNC_0_0_PIN);

	// Assign GPIO to SSC.
	gpio_enable_module(SSC_GPIO_MAP, sizeof(SSC_GPIO_MAP) / sizeof(SSC_GPIO_MAP[0]));

	// SSC init
	//
	// Note that in order this function to work, the LM97593 chip registers 6 and 248 have to be set up as following:
	//
	//	reg	6		bit	value	desc
	//	---------------------------
	//	SOUT_EN		0	1	Enable serial signals
	//	SCK_POL		1	0	data changes on SCK rising edge
	//	SFS_POL		2	0	SFS is active high
	//	RDY_POL		3	0	RDY is active high (do not care really, was used earlier for parallel data sample period detecting)
	//	MUX_MODE	4	1	AOUT outputs both channels
	//	PACKED		5	1	SFS is sent only once per I/Q pair from both channels (we do not have RDY signal connected, so otherwise we do not know which channel is A and which is B)
	//	FORMAT		6	0					1
	//				7	1	24-bit data		0	16-bit data
	//	(reg 6 0xb1)=24-bit (reg 6 0x71)=16-bit
	//
	//	reg 248		bit	value	desc
	//	---------------------------
	//	SFS_MODE	0	1	SFS asserted at the beginning of each sample period
	//	SDC_EN		1	0	No serial daisy-chain mode
	//
	//	(reg 248 1)
	//
	//	This gives the following signal format: MSB first, words 4, word length 24bit, frame size 96bit
	//
	//	SCK		______/--\__/--\__/.../--\__/--\__/--\__/.../--\__/--\__/--\__/.../--\__/--\__/--\__/.../--\__/--\__/--\__
	//	SFS		______/-----\___________________________________________________________________________________________
	//	AOUT	xxxxxxxxxxxxX=IAn=X...X=IA1=X=IA0=X=QAn=X...X=QA1=X=QA0=X=IBn=X...X=IB1=X=IB0=X=QBn=X...X=QB1=X=QB0=X
	//
	//	NOTE, that the LSB of the last transmission is clocked out during the SFS cycle if transmission is continuous, therefore
	//	we shall start frame from falling edge of the SFS and have the clock rate high enough that all data gets clocked
	//	out before next SFS happens!
	//
	//	Oddly enough, looking at the AT32UC3B0256 datasheet, there seems to be no way of disabling the initial frame sync data clocking
	//	of at least one clock cycle (1 bit). However, it seems that because of either having a FSDEN set to 0 on trasmit control or
	//	because of using the frame sync signal falling edge, this bit does not get clocked, which is a lucky case, as otherwise all our
	//	data would be one bit left-shifted.
	//
	//	The RATE register (reg 5) determines the SCK rate (Fsck = Fck/(RATE+1)) which has to be set up the way that all the bits will be
	//  clocked out during the sample period. We are clocking out for maximum of 800kHz bandwidth and have to clock out alltogether 1+(4*24) bits,
	//	so the absolutely minimum bitrate is 77.6mbit which is more than our Fck .. therefore at maximum bandwidth we have to change to
	//	single-channel mode.
	//	The required minimum bitrates and clock dividers for Fck=64MHz to support dual channel operation for audio sample rates are:
	//		48k		4.656 Mbit	Fck/8=8Mbit
	//		96k		9.312 Mbit	Fck/4=16Mbit
	//		192k	18.624 Mbit	Fck/2=32Mbit
	//
	//	Lets opt in therefore for clock divider Fck/2, so reg 5 has to be 1, as actual rate will be RATE+1. This also seems to be a fastest clocking rate we can use with
	//	AT32UC3B0256, what gives us single-channel operation of 653kHz 24bit and 969kHz 16-bit what is well aligned with a 820kHz
	//
	//	(reg 5 1) (reg 5 3 when  working on cable)
	//

	// Perform reset
    ssc->cr = AVR32_SSC_CR_SWRST_MASK;
	cpu_delay_us(100, F_CPU);

	ssc->cmr = AVR32_SSC_CMR_DIV_NOT_ACTIVE			<< AVR32_SSC_CMR_DIV_OFFSET;				// Maximum SSC bitrate is CLK_SSC/2, so use undivided clock. Gives us RX on F_ADC/2 rate from DRC chip
    ssc->tcmr = 0;		// no TX setup
    ssc->tfmr = 0;		// no TX setup

	// Normaly, there is at least 1 bit Receive Sync Data between frame sync and first bit received. We cant have this, so here is the sentence from datasheet what helps us out:
	// "Concerning the Receive Frame Sync Data operation, if the Frame Sync Length is equal to or lower than the delay between the start event and the actual data reception,
	// the data sampling operation is performed in the RSHR through the receive shift register."

	ssc->rcmr =((AVR32_SSC_RCMR_CKS_RK_PIN			<< AVR32_SSC_RCMR_CKS_OFFSET)			|	// RX clock is taken from the RX_CLOCK
				(AVR32_SSC_RCMR_CKO_INPUT_ONLY		<< AVR32_SSC_RCMR_CKO_OFFSET)			|	// clock is input-only
				(0									<< AVR32_SSC_RCMR_CKI_OFFSET)			|	// FS and data is shifted out on clock rising edge and sampled on falling edge
				(AVR32_SSC_RCMR_CKG_NONE			<< AVR32_SSC_RCMR_CKG_OFFSET)			|	// we do not generate RX clock ourselves, so do not care
				(/*3*/4								<< AVR32_SSC_RCMR_START_OFFSET)			|	// start receiving frame on the  /*5=rising edge*/ /*3=high level*/ /*4=falling edge*/ /*2=low level*/ of the SFS signal (both, high level (3) and falling edge (4) seem to be ending up with correct timing)
				(0									<< AVR32_SSC_RCMR_STTDLY_OFFSET)		|	// no receive start delay
				(0									<< AVR32_SSC_RCMR_PERIOD_OFFSET));			// do not generate FS signals

	ssc->rfmr =(((8 - 1)							<< AVR32_SSC_RFMR_DATLEN_OFFSET)        |	// word length in bits for each symbol to clock in.
                (1                                  << AVR32_SSC_RFMR_MSBF_OFFSET)          |	// most significant bit first
                ((bytesperframe - 1)				<< AVR32_SSC_RFMR_DATNB_OFFSET)         |	// number of symbols to clock in after FS activates the transfer. Could be just IA and QA or IA QA IB QB depending on mode. Also, depending on 16 or 24-bit mode, the frame lenght differs.
                ((1 - 1)							<< AVR32_SSC_RFMR_FSLEN_OFFSET)			|	// FS is 1 SCK cycle
                (AVR32_SSC_RFMR_FSOS_INPUT_ONLY     << AVR32_SSC_RFMR_FSOS_OFFSET)          |	// RX only, so no FS outputting
                (0                                  << AVR32_SSC_RFMR_FSEDGE_OFFSET));			// SR.RXSYN interrupt on FS rising edge (0)

	ssc->idr=(unsigned long)0xFFFFFFFF;

	//INTC_RegisterGroupHandler(INTC_IRQ_GROUP(AVR32_SSC_IRQ), AVR32_INTC_INT0, ssc_int_handler);
	//ssc->ier=AVR32_SSC_IER_RXSYN_MASK;

	// Enable receiver
    if (enablerx)
		ssc->cr = AVR32_SSC_CR_RXEN_MASK;
	else
		ssc->cr = AVR32_SSC_CR_RXDIS_MASK;
}


/*
Blink LED for error code three times
*/

void errcode (uint16_t ecode)
{
int i, j;

	for (j=0; j<3; j++)
	{
		gpio_clr_gpio_pin(LED);
		delayms(1000);

		for (i=0; i<ecode; i++)
		{
			gpio_set_gpio_pin(LED);
			delayms(200);
			gpio_clr_gpio_pin(LED);
			delayms(300);
		}
	}
}

// Important piece of code. This will guarantee, that we are having I/Q alignment sync between radio chip and PDCA buffers.
// Note, that one has to reset SSC interface completely, otherwise all weirdness breaks loose ...

void InitIQDataEngine(uint16_t bytesperframe, uint16_t _ifcmode)
{
uint16_t ifcmode;

	if (_ifcmode)
		ifcmode=_ifcmode;
	else
		ifcmode=datamode;		// assume current mode, if not specifically set!

	// disable interrupt and let the pdca controller run to the end
	pdca_disable_interrupt_reload_counter_zero(PDCA_CHANNEL_0);
	while(!(pdca_channel->isr&AVR32_PDCA_TRC_MASK))					// may be needed, may be not ..
		{}

	pdca_disable(PDCA_CHANNEL_0);

	// PDCA options will be reloaded at ISR() anyway, but just to make a reasonable code,
	// we will initialize here as well.
	if (ifcmode == DATA_NETWORK)
	{
		if (BitDepth == _16BIT)
		{
			NetDataLen=NETDATALEN16;
			NetPktLen=NETPKTLEN16;
			NetDataPackets=NETDATAPACKETS16;
			NetDataLenHalfwords=NETDATALEN16/2;			// used at endian swapping
		}
		else
		{
			NetDataLen=NETDATALEN24;
			NetPktLen=NETPKTLEN24;
			NetDataPackets=NETDATAPACKETS24;
			//NetDataLenHalfwords=NETDATALEN24/2;			// used at endian swapping for 16-bit mode only!
		}

		PDCA_OPTIONS.addr = (unsigned long)netdmabuffA;						// memory address
		PDCA_OPTIONS.r_addr = (unsigned long)netdmabuffA;					// next memory address (use the same, so we do have a ring buffer)
		PDCA_OPTIONS.size = NetDataLen;										// transfer counter
		PDCA_OPTIONS.r_size = NetDataLen;									// next transfer counter
	}
	else
	{
		PDCA_OPTIONS.addr = (void *)dmabuff_1;						// memory address
		PDCA_OPTIONS.r_addr = (void *)dmabuff_1;					// next memory address (use the same, so we do have a ring buffer)
		PDCA_OPTIONS.size = (DMAMASK+1)*2;							// transfer counter
		PDCA_OPTIONS.r_size = (DMAMASK+1)*2;						// next transfer counter
	}

	pdca_init_channel(PDCA_CHANNEL_0, &PDCA_OPTIONS);				// init PDCA channel with options.
	pdca_enable_interrupt_reload_counter_zero(PDCA_CHANNEL_0);

	SetSI(0);
	Init_SSC(bytesperframe, 0);				// init, but keep disabled

	cpu_delay_us(10, F_CPU);				// SI is clocked in on a rising edge of CK, so we are probably OK, but just in case make a short delay!
	//ssc->cr=AVR32_SSC_CR_RXEN_MASK;
	pdca_enable(PDCA_CHANNEL_0);
	ssc->cr=AVR32_SSC_CR_RXEN_MASK;		// should not matter if this is before or after pdca enable, since SI is asserted anyway and data is not running
	SetSI(1);
}


/*
Set the new sample frequency and bitrate on the fly. Note, that if audio interfaces are used, this has always to be
48KHz/16-bit. For network- and serial modes the sample rate must be something what F_ADC divides with, otherwise there
will be artifacts.

numbits could only be 16 or 24

The function does full PDCA and SSC interface reset, as otherwise the frame sync with DMA buffers get lost.

Note, that this function is also getting called within USB control request, so current endpoint global variable shall not be changed.

To avoid phase artifacts, the ADC clock to be set has to divide without a reminder with 4*sample_rate.

Example 1:
----------
For 250kSps single-channel 16-bit mode, we can use maximum F_ADC=64MHz, since bitstream can easily fit to F_ADC/4 speed.
Therefore, we will test it against the 64MHz first: 64000000/(4*250000)=64. Therefore, go with maximum supported F_ADC

Example 2:
----------
For 625kSps single-channel 16-bit mode we can only use speed divider /2 for LM97593 (register 5).
Since SSC clocked on F_CPU speed, which is 60MHz, and SSC max clock is F_CPU/2, the maximum safe working F_ADC clock to be used with /2 divider
seems to be 58.75MHz (based on actual tests).

Also, because each frame has its frame sync bit, in 16-bit single-channel mode we are getting 32/33=0.9697 of bits effectiveness.

We can therefore test it first: 58750000/(4*625000)=23.5. The reminder is unaceptable, so the correct F_ADC clock speed is therefore
23*(4*625000)=57500000. This is giving us the upper frequency limit to only 28.75MHz, but thats the penalty of wider bandwidth with that particular
sample rate.

Example 3:
----------
For 231884Sps dual-channel operation, the actual required bitrate is (231884*4*8*2)+231884=15.07246Mbit. This is fitting inside
the 64000000/4 maximum (16Mbit), so the maximum F_ADC of 64MHz can be used with divisor left on 4.
Fist test shows, that 64000000/(4*231884)=69.00001725, which is close enough to have F_ADC operating at its max 64MHz frequency!

*/

void SampleMode(uint32_t samplefreq, uint16_t channels, uint16_t numbits, uint16_t ifcmode)
{
uint32_t oldsamplefreq;

uint32_t bitrate;
uint32_t maxadcclock;
uint32_t fadc;

	// Calculate required serial output rate needed by LM97593.
	// The rate is bits + one frame sync bit per frame.
	bitrate=samplefreq;
	bitrate*=numbits*2;
	bitrate*=channels;
	bitrate+=samplefreq;					// add frame sync bits as well

	if (bitrate > (64000000/4))				// 480ksps in 16-bit single-channel mode max
		maxadcclock = MAX_SAFE_DIV2_FADC;
	else
		maxadcclock = 64000000;

	//calculate maximum fadc what divides integer with 4*samplefreq
	fadc=(maxadcclock/(samplefreq*4));
	fadc*=(samplefreq*4);

	SetADCClock(fadc);

	oldsamplefreq=SampleRate;

	SampleRate=samplefreq;								//update globals
	BitDepth=numbits;

	Init_LM97593(samplefreq, numbits, channels, false, f_adc);

	UpdateRegisters(1);

	if (oldsamplefreq != SampleRate)
	{
		// new sample rate requires also frequency to be recalculated
		SetFreq(CH_A, lastfreq_A, 1, f_adc);
		SetFreq(CH_B, lastfreq_B, 1, f_adc);
	}

	InitIQDataEngine((2*numbits*channels)/8, ifcmode);
}

void UpdateFlashCRC(void)
{
uint16_t crc;
int i;
void* flashdata;

	crc=0;
	flashdata=(void*)&flash_nvram_data;

	for (i=0; i<(sizeof(nvram_data_t)-2); i++)
		crc=UpdateCRC16(crc, *(uint8_t*)(flashdata+i));

	flashc_memcpy((void *)&flash_nvram_data.crc, &crc, sizeof(flash_nvram_data.crc), true);
}

void SetupFlashNVRAM(void)
{
uint16_t crc;
int i;
void* flashdata;
uint16_t port;

	crc=0;
	flashdata=(void*)&flash_nvram_data;
	for (i=0; i<(sizeof(nvram_data_t)-2); i++)
		crc=UpdateCRC16(crc, *(uint8_t*)(flashdata+i));

	if (crc!=flash_nvram_data.crc)
	{
		port = SDR_PORT;

		flashc_memcpy((void *)&flash_nvram_data.dhcp, "\1", sizeof(flash_nvram_data.dhcp), true);
		//flashc_memcpy((void *)&flash_nvram_data.ip, "\xC0\xA8\x64\xAA", sizeof(flash_nvram_data.ip), true);				//192.168.100.170
		//flashc_memcpy((void *)&flash_nvram_data.gw, "\xC0\xA8\x64\x1", sizeof(flash_nvram_data.gw), true);				//192.168.100.1
		//flashc_memcpy((void *)&flash_nvram_data.netmask, "\xFF\xFF\xFF\x0", sizeof(flash_nvram_data.netmask), true);		//255.255.255.0

		flashc_memcpy((void *)&flash_nvram_data.ip, "\0\0\0\0", sizeof(flash_nvram_data.ip), true);
		flashc_memcpy((void *)&flash_nvram_data.gw, "\0\0\0\0", sizeof(flash_nvram_data.gw), true);
		flashc_memcpy((void *)&flash_nvram_data.netmask, "\0\0\0\0", sizeof(flash_nvram_data.netmask), true);

		flashc_memcpy((void *)&flash_nvram_data.sdrport, &port, sizeof(flash_nvram_data.sdrport), true);				// set incoming TCP port to default
		flashc_memcpy((void *)&flash_nvram_data.udp_port, &port, sizeof(flash_nvram_data.udp_port), true);				// set UDP outbound port to default
	}

	UpdateFlashCRC();
}

//
// Parse "xxx.xxx.xxx.xxx" string to a IP address into ipaddr array (4 bytes)
//
uint8_t ParseIPAddress(unsigned char* ipaddr, char* carg)
{
int i, j;
int dotcount=0;
unsigned long token;

	for (i=0, j=0; i<strlen(carg); i++)
	{
		if (carg[i]==' ')	// strip spaces
			continue;

		token=strtol(carg+i, NULL, 0);

		if (token > 255)
			return 1;		// fail, if number is too large

		ipaddr[j++]=token;	// save token

		// parse past next dot
		for (; i<strlen(carg); i++)
		{
			if (carg[i]=='.')
			{
				dotcount++;
				break;			// note, that i is incremented once we re-iterate main loop, so no need to do it here
			}
		}
	}

	if (dotcount!=3)
		return 2;	// must have three dots in IP address!

	return 0;		// show success
}


/*
Main program entry point. This routine contains the overall program flow, including initial
setup of all components and the main program loop.
*/

int main(void)
{
char cmdbuffer[64+1];
int16_t recbyte;
int16_t bufflen=0;
bool scancode=false;
//volatile unsigned char mcubyte;		// if not volatile, double updates may fail
int16_t i;	// MUST be kept as signed!!!

int16_t a;
uint32_t tuningfreq;
uint32_t dummy;

char omnibuff[120];
int16_t omnioff=0;

//uint64_t avgrate;

char *cmdname, *carg1, *carg2, *carg3;

bool showecho=true;
bool createzlp=false;
int bytessent=0;

uint32_t dmaoffset, last_dmaoffset=0, worddiff;
uint32_t panfreq;

uint8_t* CPUSerial = (uint8_t*)0x80800204;		// internal serial start address, 120-bit (15 bytes)

	// always keep this as a first thing in main()
	SetupHardware();

	// check flash nvram area and if incorrect CRC, init with defaults
	SetupFlashNVRAM();

	clock_init();					// this gives us 1ms resolution timer interrupt

	// Initialize USB
	USB_Init();

	//Compose a unique serial number from CPU serial (actually the same as MAC numbers [1]..[3]
	sprintf((char*)SDRSerial, "%c%c%02X%02X%02X", '1', '5', (*(CPUSerial+1))^(*(CPUSerial+10))^(*(CPUSerial+5)), (*(CPUSerial+7))^(*(CPUSerial+9))^(*(CPUSerial+4)), (*(CPUSerial+13))^(*(CPUSerial+2))^(*(CPUSerial+11)));

	// Wait for USB to start. Test for 5 seconds, if no USB, then we are probably getting external power and shall go ahead with network
	for (i=0; ((i<5000) && (!usbinit_done)); i++)
	{
		cpu_delay_us(1000, F_CPU);
	}

	GlobalInterruptDisable();

	//clear SSC status and empty buffers before enabling interrupt

	while(ssc->sr & (AVR32_SSC_SR_RXRDY_MASK|AVR32_SSC_SR_RXSYN_MASK))
	{
		dummy=ssc->rhr;
		dummy=ssc->sr;	// in case the code optimization found a way of checking the SR register status what does not count as read for interrupt subsystem
	}

	GlobalInterruptEnable();
/*
	// Initialize DMA now for transferring bytes automatically to memory
	if (BitDepth == _16BIT)
		PDCA_OPTIONS.transfer_size = PDCA_TRANSFER_SIZE_HALF_WORD;
	else
		PDCA_OPTIONS.transfer_size = PDCA_TRANSFER_SIZE_WORD;
*/
	pdca_init_channel(PDCA_CHANNEL_0, &PDCA_OPTIONS); // init PDCA channel with options.

	INTC_RegisterGroupHandler(INTC_IRQ_GROUP(AVR32_PDCA_IRQ_0), AVR32_INTC_INT0, pdca_int_handler_0);
	pdca_enable_interrupt_reload_counter_zero(PDCA_CHANNEL_0);

	pdca_enable(PDCA_CHANNEL_0);
	pdca_channel=pdca_get_handler(PDCA_CHANNEL_0);

	// theoretically we shall be ok now!
	StartRadio();

	if (!reginit_done)
	{
		Init_LM97593(SampleRate, BitDepth, DUAL_CHANNEL, true, 64000000);		// just init the register pool with default values and 64MHz ADC clock
		// NB! Only enable SSC and associated interrupt after PDCA is initialized (interrupt will set the PDCA enable bit)
		Init_SSC(_2X16BIT_IQ, 1);
		reginit_done=true;
	}

	ResetRadio();			// Assert reset. May or may not be needed, but nice to do.

	UpdateRegisters(1);		// Initialize radio registers with whatever we have for defaults

	radio_ready=true;		// show audio interrupt that its OK to fetch data.

	// Purge all data on CDC interface
	while (CDC_Device_ReceiveByte(&VirtualSerial1_CDC_Interface)>=0) {};

	memset(cmdbuffer, 0, 64+1);
	memset(omnibuff, 0, 120);

	gpio_set_gpio_pin(LED);			// indicate that setup process is over

	delayms(1000);					// allow some more stabilization before network is going to be activated (needed for old 1.03 boards)

	// Initialize network
	NetSDR_init();


/*
	// start tasks!
	ados_init();
	//ados_eventReset(&g_event);
	ados_addTask(&g_demoTcb, demoTask, g_demoTaskStack, sizeof(g_demoTaskStack), DEMO_TASK_PRIO);
	ados_addTask(&g_USBTcb, USBTask, g_USBTaskStack, sizeof(g_USBTaskStack), USB_TASK_PRIO);

	ados_start(); //no return from here
*/

	// buffer at last 500 words/halfwords worth of data locally.
	// do not stall radio tho, as we need to be able to do diagnostics about the situation, if this is happening

	i=0;
	a=0;

	while (pdca_channel->mar < (unsigned long)dmabuff_1+((BitDepth/8)*500)/*&dmabuff_1[500]*/)
	{
		i++;
		delayms(10);

		if (i==100)
		{
			if (!a)
			{
				//program CDCE913 chip to accept LVCMOS input and see, if we are better off with this - may be we do have external clock or 27MHz generator instead of crystal

				a=1;	// indicate that we are already trying external LVCMOS clock
				i=0;	// reset loop counter

				twi_write(1, 0x9);				// do not touch bit 5, as this will permanently lock the EEPROM config!
				twi_write(5, 0);				// 0pF loading caps
				//wait for LM97593 to start up
				delayms(10);

				// re-init radio chip
				Init_LM97593(SampleRate, BitDepth, DUAL_CHANNEL, true, 64000000);		// just init the register pool with default values and 64MHz sample clock
				ResetRadio();			// Assert reset. May or may not be needed, but nice to do. (This function is deprecated actually ...)
				UpdateRegisters(1);		// Initialize radio registers with whatever we have for default

				continue;
			}

			errcode(5);	// blink error code (3x5), do USB activities meanwhile
			break;		// no luck, LM97593 chip is likely stalled :(
		}
	}

	i=0;

	GlobalInterruptDisable();
	SSCSFSIntCounter=0;			//reset samples injection counters
	IntTime=millis();
	samplesadded=0;
	GlobalInterruptEnable();

	// default parameters for 16-bt AB mode.
	// These will be reset at USB control processing where the LIBUSB_MODE will actually be set
	roffseta=0;					// depending on if we are transmitting samples from channel A, B or both in LIBUSB mode, the reset value will be changed to reflect correct sample offset
	displacement=8;				// how many words (16-bit) to advance in buffer on each pass

	dmabuff_w1=&dmabuff_1[0];
	dmabuff_w2=&dmabuff_1[2];
	dmabuff_w3=&dmabuff_1[4];
	dmabuff_w4=&dmabuff_1[6];
	////

	//set samplemode to I/Q sync once more, just in case
	SampleMode(48000, DUAL_CHANNEL, _16BIT, 0);		// we are starting up in audio mode, so set sample mode appropriately. this function will also sync the PDCA buffers with SSC to get a grip on I/Q frame alignment

	if (!pantable)
	{
		pantable=malloc(sizeof(PANENTRY));
		panentry=pantable;


		panentry->magic_I=0x55AA;
		panentry->magic_Q=0xF0F0;
		panentry->samples=32;
		panentry->startfreq=0;
		panentry->stepfreq=64000;
		panentry->steps=500;
		panentry->skip=8;

		panentry->samples*=4;		// multiply by 4, as each sample in our context is 2 IQ pairs, i.e. 4 words
		panentry->skip*=4;

		/*
		panentry->magic_I=0x55AA;
		panentry->magic_Q=0xF0F0;
		panentry->samples=1000;
		panentry->startfreq=4635000;
		panentry->stepfreq=64000;
		panentry->steps=1;
		*/
	}

	while(1)
	{
		if (datamode != DATA_NETWORK)			// update variables only in non-network mode to save some time
		{
			woffsetx=pdca_channel->mar;			// highwater mark - this is where the next data is placed into buffer
			dmaoffset=woffsetx;					// make dmaoffset pointing the same address as woffsetx does

			woffsetx&=~(0x1FU);					// Eliminate partial transfers.
												// The longest data to fetch is 4 I/Q sample pairs form channel B, what accounts alltogether 32 bytes (4 full I/Q A and B pairs), thus 1F.
		}

		if (datamode == DATA_LIBUSB)		// transfer data through bulk endpoint
		{
			// if we are in panadapter mode, go update scanner parameters according to table (datamode is DATA_LIBUSB, data is compatible with LIBMODE_16AB)
			// (the same is theoretically applicable for network dual channel mode as well, but we are not supporting it at this moment)

			if (panadapter)
			{
				panentry=pantable+(currentpanentry*sizeof(PANENTRY));			// point to the correct panentry record

				//dmaoffset=pdca_channel->mar-(unsigned long)dmabuff_1;			// offset of the next write address inside the buffer
				dmaoffset-=(unsigned long)dmabuff_1;
				dmaoffset/=2;													// convert to words
				dmaoffset&=~(3U);										// always point to IA

				if (last_dmaoffset < dmaoffset)
					worddiff=dmaoffset-last_dmaoffset;
				else
					worddiff=dmaoffset+(DMAMASK+1)-last_dmaoffset;

				if ((worddiff >= (panentry->samples+16+panentry->skip)))	// at least requested sample count + 16 words to store magic info and frequency + skip area for frequency change
				{
					last_dmaoffset=dmaoffset;					// reset current pointer

					panfreq=panentry->startfreq;
					panfreq+=stepsdone*panentry->stepfreq;		// new scan starting freq

					stepsdone++;

					if (stepsdone >= panentry->steps)
					{
						stepsdone=0;
						//advance by one in the pantable
						currentpanentry++;
						if (currentpanentry >= panentries)
							currentpanentry=0;
					}

					// Now change frequency. Note that registry write etc. takes some time, so we will have to ignore some of the data after the magic!

					if (panadapter == LIBMODE_16ABPAN)
					{
						// key in magic in current position for chB IQ data
						dmabuff_1[(dmaoffset-16+2)&DMAMASK]=panentry->magic_I;
						dmabuff_1[(dmaoffset-16+3)&DMAMASK]=panentry->magic_Q+1;
						dmabuff_1[(dmaoffset-16+6)&DMAMASK]=panentry->magic_I+2;
						dmabuff_1[(dmaoffset-16+7)&DMAMASK]=panentry->magic_Q+3;
						// key in frequency
						dmabuff_1[(dmaoffset-16+10)&DMAMASK]=panfreq&0xFFFF;
						dmabuff_1[(dmaoffset-16+11)&DMAMASK]=(panfreq>>16)&0xFFFF;
						// key in trailer
						dmabuff_1[(dmaoffset-16+14)&DMAMASK]=panentry->magic_I+4;
						dmabuff_1[(dmaoffset-16+15)&DMAMASK]=panentry->magic_Q+5;

						// channel A is IQ data, channel B is panscan
						SetFreq_Fast(CH_B, panfreq, 1, f_adc);
					}
					else	//LIBMODE_16BAPAN
					{
						// key in magic in current position for chA IQ data
						dmabuff_1[(dmaoffset-16+0)&DMAMASK]=panentry->magic_I;
						dmabuff_1[(dmaoffset-16+1)&DMAMASK]=panentry->magic_Q+1;
						dmabuff_1[(dmaoffset-16+4)&DMAMASK]=panentry->magic_I+2;
						dmabuff_1[(dmaoffset-16+5)&DMAMASK]=panentry->magic_Q+3;
						// key in frequency
						dmabuff_1[(dmaoffset-16+8)&DMAMASK]=panfreq&0xFFFF;
						dmabuff_1[(dmaoffset-16+9)&DMAMASK]=(panfreq>>16)&0xFFFF;
						// key in trailer
						dmabuff_1[(dmaoffset-16+12)&DMAMASK]=panentry->magic_I+4;
						dmabuff_1[(dmaoffset-16+13)&DMAMASK]=panentry->magic_Q+5;

						// channel B is IQ data, channel A is panscan
						SetFreq_Fast(CH_A, panfreq, 1, f_adc);
					}
				}
			}

			Endpoint_SelectEndpoint(CDC2_TX_EPNUM);

			// Little obscure on a first glance, this routine is actually producing a code, what compiled with no optimization (OPT=0 in makefile)
			// is capable of transfer rate of 7.5+ Mbit if nothing additional is added to a loop and there are no excess system interrupts!

			if (Endpoint_IsINReady())
			{
				while (Endpoint_IsReadWriteAllowed())
				{
					if ((uint32_t)&dmabuff_1[(roffseta+16)&DMAMASK]==woffsetx)			// reached the end of buffer, so break. (use woffset+1 to test the max speed, as on this case the transfers are not terminated at all)
						break;															// also, leave last 16 bytes in buffer, since we are patching in the panscanner header information there.
																						// The +16 is needed to establish a no-transfer area to patch the panscanner header into, if needed

					// note, that to prevent pointer mathematics inside loop for various modes (A/B/AB), we are using pre-offset pointers here!

					Endpoint_Write_16_BE(dmabuff_w1[(roffseta)&DMAMASK]);			//I
					Endpoint_Write_16_BE(dmabuff_w1[(roffseta+1)&DMAMASK]);			//Q
					Endpoint_Write_16_BE(dmabuff_w2[(roffseta)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w2[(roffseta+1)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w3[(roffseta)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w3[(roffseta+1)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w4[(roffseta)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w4[(roffseta+1)&DMAMASK]);

					roffseta+=displacement;

					/*
					Endpoint_Write_16_BE(dmabuff_1[roffseta&DMAMASK]);			//I
					Endpoint_Write_16_BE(dmabuff_1[(roffseta+1)&DMAMASK]);		//Q
					roffseta+=4;
					*/
				}

				// Send the full packet to the host
				if (!Endpoint_IsReadWriteAllowed())
					Endpoint_ClearIN();
			}
		}
		else if (datamode == DATA_NETWORK)
		{
			// do nothing, but prevent crawling through entire if-else tree below to save some time
		}
		else if (datamode == DATA_LIBUSB_XPY)		// transfer data through bulk endpoint
		{
			Endpoint_SelectEndpoint(CDC2_TX_EPNUM);

			// Little obscure on a first glance, this routine is actually producing a code, what compiled with no optimization (OPT=0 in makefile)
			// is capable of transfer rate of 7.5+ Mbit if nothing additional is added to a loop and there are no excess system interrupts!

			if (Endpoint_IsINReady())
			{
				while (Endpoint_IsReadWriteAllowed())
				{
					if ((uint32_t)&dmabuff_1[roffseta&DMAMASK]==woffsetx)				// reached the end of buffer, so break. (use woffset+1 to test the max speed, as on this case the transfers are not terminated at all)
						break;

					// note, that to prevent pointer mathematics inside loop for various modes (A/B/AB), we are using pre-offset pointers here!

					roffsetb=(roffseta)&DMAMASK;

					Endpoint_Write_16_BE(dmabuff_x1[roffsetb]+dmabuff_y1[roffsetb]);		//I
					Endpoint_Write_16_BE(dmabuff_x2[roffsetb]+dmabuff_y2[roffsetb]);		//Q
					Endpoint_Write_16_BE(dmabuff_x3[roffsetb]+dmabuff_y3[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x4[roffsetb]+dmabuff_y4[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x5[roffsetb]+dmabuff_y5[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x6[roffsetb]+dmabuff_y6[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x7[roffsetb]+dmabuff_y7[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x8[roffsetb]+dmabuff_y8[roffsetb]);

					roffseta+=displacement;

				}

				// Send the full packet to the host
				if (!Endpoint_IsReadWriteAllowed())
					Endpoint_ClearIN();
			}
		}
		else if (datamode == DATA_LIBUSB_XMY)		// transfer data through bulk endpoint
		{
			Endpoint_SelectEndpoint(CDC2_TX_EPNUM);

			// Little obscure on a first glance, this routine is actually producing a code, what compiled with no optimization (OPT=0 in makefile)
			// is capable of transfer rate of 7.5+ Mbit if nothing additional is added to a loop and there are no excess system interrupts!

			if (Endpoint_IsINReady())
			{
				while (Endpoint_IsReadWriteAllowed())
				{
					if ((uint32_t)&dmabuff_1[roffseta&DMAMASK]==woffsetx)				// reached the end of buffer, so break. (use woffset+1 to test the max speed, as on this case the transfers are not terminated at all)
						break;

					// note, that to prevent pointer mathematics inside loop for various modes (A/B/AB), we are using pre-offset pointers here!

					roffsetb=(roffseta)&DMAMASK;

					Endpoint_Write_16_BE(dmabuff_x1[roffsetb]-dmabuff_y1[roffsetb]);		//I
					Endpoint_Write_16_BE(dmabuff_x2[roffsetb]-dmabuff_y2[roffsetb]);		//Q
					Endpoint_Write_16_BE(dmabuff_x3[roffsetb]-dmabuff_y3[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x4[roffsetb]-dmabuff_y4[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x5[roffsetb]-dmabuff_y5[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x6[roffsetb]-dmabuff_y6[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x7[roffsetb]-dmabuff_y7[roffsetb]);
					Endpoint_Write_16_BE(dmabuff_x8[roffsetb]-dmabuff_y8[roffsetb]);

					roffseta+=displacement;
				}

				// Send the full packet to the host
				if (!Endpoint_IsReadWriteAllowed())
					Endpoint_ClearIN();
			}
		}

		// Seems that absolutely max speed we can get from AT32UC3B is 7.5Mbit
		else if (datamode == DATA_LIBUSB_SPEEDTEST)
		{
			Endpoint_SelectEndpoint(CDC2_TX_EPNUM);

			if (Endpoint_IsINReady())
			{
				while (Endpoint_IsReadWriteAllowed())
				{
					// !!!!!!!!
					// this condition will never be met, but is included to test the actual speed with all the mathematics inside the loop!
					if ((uint32_t)&dmabuff_1[roffseta&DMAMASK]==woffsetx+1)				// reached the end of buffer, so break. (use woffset+1 to test the max speed, as on this case the transfers are not terminated at all)
						break;

					// note, that to prevent pointer mathematics inside loop for various modes (A/B/AB), we are using pre-offset pointers here!

					Endpoint_Write_16_BE(dmabuff_w1[(roffseta)&DMAMASK]);			//I
					Endpoint_Write_16_BE(dmabuff_w1[(roffseta+1)&DMAMASK]);		//Q
					Endpoint_Write_16_BE(dmabuff_w2[(roffseta)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w2[(roffseta+1)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w3[(roffseta)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w3[(roffseta+1)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w4[(roffseta)&DMAMASK]);
					Endpoint_Write_16_BE(dmabuff_w4[(roffseta+1)&DMAMASK]);

					roffseta+=displacement;
				}

				// Send the full packet to the host
				if (!Endpoint_IsReadWriteAllowed())
					Endpoint_ClearIN();
			}

			// if the main worker loop is optimal enough, both methods should give a steady 7.57Mbit data rate!
			/*
			Endpoint_SelectEndpoint(CDC2_TX_EPNUM);
			if (Endpoint_IsINReady())
			{
				while (Endpoint_IsReadWriteAllowed())
				{
					Endpoint_Write_32_BE(0x15AA00FF);	//Subtract channel B samples from channel A for diversity mode
					Endpoint_Write_32_BE(0x25AA00FF);	//Subtract channel B samples from channel A for diversity mode
					Endpoint_Write_32_BE(0x35AA00FF);	//Subtract channel B samples from channel A for diversity mode
					Endpoint_Write_32_BE(0x45AA00FF);	//Subtract channel B samples from channel A for diversity mode
				}

				Endpoint_ClearIN();
			}
			*/


		}
		else if (datamode == DATA_AUDIO)
		{
			//Saving previous endpoint
			//uint8_t PrevEndpoint = Endpoint_GetCurrentEndpoint();	// always has to be first and in the primary scope!

			//read bytes aligned for channel A data
			if (Main_Audio_Interface.State.InterfaceEnabled)
			{
				Endpoint_SelectEndpoint(AUDIO_STREAM_A_EPNUM);			// CH_A audio

				// Note, that we are supporting only 16-bit samples through audio interface, so we relay on array indexing and do not manipulate pointers ourselves.
				while (Endpoint_IsINReady() && Endpoint_IsReadWriteAllowed() && (((uint32_t)&dmabuff_1[(roffseta)&DMAMASK])!=woffsetx))
				{
					if (!diversity)
					{
						Endpoint_Write_16_BE(dmabuff_1[(roffseta)&DMAMASK]);
						roffseta++;
						Endpoint_Write_16_BE(dmabuff_1[(roffseta)&DMAMASK]);
						roffseta++;
					}
					else
					{
						Endpoint_Write_16_BE(dmabuff_1[(roffseta)&DMAMASK]-dmabuff_1[(roffseta+2)&DMAMASK]);	//Subtract channel B samples from channel A for diversity mode
						roffseta++;
						Endpoint_Write_16_BE(dmabuff_1[(roffseta)&DMAMASK]-dmabuff_1[(roffseta+2)&DMAMASK]);
						roffseta++;
					}

					roffseta+=2;	//skip trailing 2 words for channel A (belonging to channel B)
				}

				// Check to see if the bank is now full
				if (!(Endpoint_IsReadWriteAllowed()))
				{
					// Send the full packet to the host
					Endpoint_ClearIN();
				}
			}

			//read bytes aligned for channel B data
			if (AudioB_Audio_Interface.State.InterfaceEnabled)
			{
				if (scanner)
				{

				}
				else
				{
					Endpoint_SelectEndpoint(AUDIO_STREAM_B_EPNUM);			// CH_B audio

					while (Endpoint_IsINReady() && Endpoint_IsReadWriteAllowed() && (((uint32_t)&dmabuff_1[(roffsetb)&DMAMASK])!=woffsetx))
					{
						roffsetb+=2;	//skip leading 2 words for channel B (did belong to channel A)

						Endpoint_Write_16_BE(dmabuff_1[(roffsetb)&DMAMASK]);
						roffsetb++;
						Endpoint_Write_16_BE(dmabuff_1[(roffsetb)&DMAMASK]);
						roffsetb++;
					}

					// Check to see if the bank is now full
					if (!(Endpoint_IsReadWriteAllowed()))
					{
						// Send the full packet to the host
						Endpoint_ClearIN();
					}
				}
			}

			//Endpoint_SelectEndpoint(PrevEndpoint);		// always has to be last and in the primary scope!
		}
		else if (datamode == DATA_SDRIQ)		// SDR-IQ emulation mode works only with
		{
			//if (!serialstreamok)
			//{
			//	CDC_Device_CreateStream(&VirtualSerial1_CDC_Interface, &USBSerialStream);
			//	serialstreamok=true;
			//}

			//process incoming data/commands
			if (IQ_ProcessByte(&VirtualSerial1_CDC_Interface))
			{
				//Output audio data. We are using SDR-IQ sample format, so each packet starts with \x00 \x80 and then 8192 bytes of IQ data
				//   +------+------+--------+--------+--------+--------+--------+-------
				//   | 0x00 | 0x80 | I1_lsb | I1_msb | Q1_lsb | Q1_msb | I2_lsb | ....
				//   +------+------+--------+--------+--------+--------+--------+-------
				//
				// As the SDR-IQ format is only describing 16-bit format, the 24-bit format is specific to the SDR MK1.5 and consists
				// of 1365 3-byte words in little endian order, therefore the packet is 2-byte header + 8190 bytes of payload with same layout
				//
				//

				Endpoint_SelectEndpoint(CDC1_TX_EPNUM);

				if ((USB_DeviceState != DEVICE_STATE_Configured) || !(VirtualSerial1_CDC_Interface.State.LineEncoding.BaudRateBPS))
				{}
				else
				{
					if (!iqpktsize)
					{
						//have to send header data
						CDC_Device_SendByte(&VirtualSerial1_CDC_Interface, 0);
						CDC_Device_SendByte(&VirtualSerial1_CDC_Interface, 0x80);		// note, that these two sends block, until the CDC interface is ready to accept data!
					}


					if (BitDepth == 16)
					{
						//if ((createzlp)&&(Endpoint_IsReadWriteAllowed()))
						//{
						//	createzlp=false;
						//	Endpoint_ClearIN();
						//}

						// Quite same as audio data transfer routine
						while (Endpoint_IsReadWriteAllowed() && (((uint32_t)&dmabuff_1[(roffseta)&DMAMASK])!=woffsetx)/* && (!createzlp)*/)
						{
							Endpoint_Write_16_BE(dmabuff_1[(roffseta)&DMAMASK]);
							roffseta++;
							Endpoint_Write_16_BE(dmabuff_1[(roffseta)&DMAMASK]);
							roffseta++;

							roffseta+=2;				//skip trailing 2 words for channel A (belonging to channel B)
							iqpktsize+=4;

							bytessent+=4;

							if (iqpktsize > 8192)
								iqpktsize=0;			// indicate that we have to send header again
						}

						// Check to see if the bank is now full
						//if (!(Endpoint_IsReadWriteAllowed()))
						//if (Endpoint_BytesInEndpoint() == CDC_TX_EPSIZE)		//
						//if (bytessent == CDC_TX_EPSIZE)
						if (bytessent >= 1024)
						{
							// Send the full packet to the host
							Endpoint_ClearIN();
							createzlp=true;
							bytessent=0;
							//Endpoint_ClearIN();		//ZLP
						}
					}
					else
					{
						// 24-bit logic goes here
					}
				}
			}
		}
/*
Since SSCSFIntCounter is not running (interrupt is deprecated), this function has no meaning anyway.

		if ((datamode == DATA_AUDIO) || (datamode == DATA_SDRIQ))
		{
			// if we do have a sample rate lag, repeat last samples until the rate matches again. This is not as effective as the USB audio throttling would be,
			// but still better than have a skipping audio.

			if (roffseta && (SSCSFSIntCounter > SampleRate))
			{
				avgrate=SSCSFSIntCounter;
				avgrate*=1000L;
				avgrate/=(millis()-IntTime);

				if (avgrate < SampleRate)
				{
					roffsetb-=4;
					roffseta-=4;
					GlobalInterruptDisable();
					SSCSFSIntCounter++;
					GlobalInterruptEnable();
					samplesadded++;
				}
			}
		}
*/
		if ((datamode == DATA_AUDIO) || (datamode == DATA_NETWORK))
		{

			// network interface, if configured, is active in all modes to be able to answer pings except bulk USB (waists about 1.2Mbit otherwise ..)

			NetSDR_Task();

			// check if we have to run on poll mode
			if (pollmode)
			{
				CAT_Poll();
			}

			// process console interface packets

			recbyte=CDC_Device_ReceiveByte(&VirtualSerial1_CDC_Interface);

			if ((!scancode)&&((recbyte == 0xD)))		// CR pressed or _ character
			{
				if (!bufflen)		// just an empty cr
				{
					ShowMenu();
				}
				else				// process command
				{
					if (showecho)
						Message(1, "\r\n");

					showecho = true;	// re-enable echo if we have had it diabled

					CMD_Condition(cmdbuffer);		//clean up comand line
					CMD_Parse(cmdbuffer, &cmdname, &carg1, &carg2, &carg3);

					if (strcmp(cmdname, "help") == 0)
					{
						ShowHelp();
					}
					else if (strcmp(cmdname, "pon") == 0)
					{
						StartRadio();
					}
					else if (strcmp(cmdname, "poff") == 0)
					{
						StopRadio();
					}
					else if (strcmp(cmdname, "sysinfo") == 0)
					{
						unsigned char sdripaddr[4];
						//avgrate=SSCSFSIntCounter;
						//avgrate*=1000L;
						//avgrate/=(millis()-IntTime);

						Message(1, "SFS/PDCA_reload Interrupt counter = %lu\r\n", SSCSFSIntCounter);
						Message(1, "PBA speed = %ld\r\n", pcl_freq_param.pba_f);
						Message(1, "CKSEL Register= %08lX\r\n", AVR32_PM.cksel);
						uip_gethostaddr(&sdripaddr);
						Message(1, "IP Address: %d.%d.%d.%d\r\n", sdripaddr[0], sdripaddr[1], sdripaddr[2], sdripaddr[3]);
						Message(1, "Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", uip_ethaddr.addr[0], uip_ethaddr.addr[1], uip_ethaddr.addr[2], uip_ethaddr.addr[3], uip_ethaddr.addr[4], uip_ethaddr.addr[5]);
						//Message(1, "Total time since first interrupt = %lu\r\n", ((uint32_t)millis()-(uint32_t)IntTime));
						//Message(1, "  Calculated Average Sample Rate = %luHz\r\n", (uint32_t)avgrate);
						//Message(1, "         Samples added to buffer = %lu\r\n", (uint32_t)samplesadded);
					}
					else if (strcmp(cmdname, "netiqdump") == 0)
					{
						Message(1, "\r\n");
						Message(1, "BitDepth = %d\r\n", BitDepth);
						Message(1, "NetDataLen = %d\r\n", NetDataLen);
						Message(1, "NetPktLen = %d\r\n", NetPktLen);
						Message(1, "NetDataPackets = %d\r\n", NetDataPackets);
						Message(1, "ReverseEndian = %d\r\n\r\n", ReverseEndian);

						Message(1, "Head of Data Buffer (Before endian swap):\r\n");

						netiqdump=1;
					}
					else if (strcmp(cmdname, "ir") == 0)
					{
						GlobalInterruptDisable();
						SSCSFSIntCounter=0;
						GlobalInterruptEnable();
					}
					else if (strcmp(cmdname, "clock") == 0)
					{
						Message(1, "CDCE913 VCXO Settings:\r\n");
						Message(1, "Fvco=%u Hz; p=%u; q=%u; r=%u; n=%u; Pdiv=%u; VCO Range: %u; Valid: %s; Error: %u Hz\r\n",
										f_adc, pllconf.p, pllconf.q, pllconf.r, pllconf.n, f_adc_divider, pllconf.vco_range, coeffs_are_valid(pllconf) ? "yes" : "no", uabssub(real_f_adc, f_adc));
						Message(1, "Registers: %02X %02X %02X %02X\r\n", pllconf.darr[0], pllconf.darr[1], pllconf.darr[2], pllconf.darr[3]);
					}
					else if (strcmp(cmdname, "regdump") == 0)
					{
						DumpRegisters();
					}
					else if (strcmp(cmdname, "audiotest") == 0)
					{
						testsamples=!testsamples;
					}
					else if (strcmp(cmdname, "defaults") == 0)
					{
						UpdateRegisters(1);
					}
					else if (strcmp(cmdname, "scancode") == 0)
					{
						scancode=true;
					}
					else if (strcmp(cmdname, "mc") == 0)
					{
						MonteCarlo();
					}
					else if (strcmp(cmdname, "si") == 0)
					{
						AssertSI();
					}
					else if (strcmp(cmdname, "freq") == 0)
					{
						Message(1, "\r\n");
						Message(1, "Channel A Frequency = %lu\r\n", lastfreq_A);
						Message(1, "Channel B Frequency = %lu\r\n", lastfreq_B);
					}
					else if (strncmp(cmdname, "fa ", 3) == 0)
					{
					uint32_t tuningfreq;

						if (carg1)
						{
							tuningfreq=strtol(carg1, NULL, 0);
							SetFreq(CH_A, tuningfreq, 1, f_adc);
						}
						else
							Message(1, "Invalid command line arguments!\r\n");
					}

					else if (strncmp(cmdname, "fb ", 3) == 0)
					{
					uint32_t tuningfreq;

						if (carg1)
						{
							tuningfreq=strtol(carg1, NULL, 0);
							SetFreq(CH_B, tuningfreq, 1, f_adc);
						}
						else
							Message(1, "Invalid command line arguments!\r\n");
					}

					else if (strncmp(cmdname, "ga ", 3) == 0)
					{
						if (carg1)
						{
							gain=strtol(carg1, NULL, 0);
							if (gain > 15)
							{
								Message(1, "Gain value %d out of boundaries (has to be 0..15)\r\n", gain);
							}
							else
							{
								switch(SetGain(CH_A, gain))
								{
									case 1:
											Message(1, "ATTN! Exponent is now cleared for both channels!\r\n");
											break;
									case 2:
											Message(1, "ATTN! Exponent is now forced to 111 for both channels!\r\n");
											break;
									case 0:
									default:
											break;
								}
								Message(1, "CH_A Software Gain is now set to %d\r\n", gain);
							}
						}
						else
							Message(1, "Invalid command line arguments!\r\n");
					}
					else if (strncmp(cmdname, "gb ", 3) == 0)
					{
						if (carg1)
						{
							gain=strtol(carg1, NULL, 0);
							if (gain > 15)
							{
								Message(1, "Gain value %d out of boundaries (has to be 0..15)\r\n", gain);
							}
							else
							{
								switch(SetGain(CH_B, gain))
								{
									case 1:
											Message(1, "ATTN! Exponent is now cleared for both channels!\r\n");
											break;
									case 2:
											Message(1, "ATTN! Exponent is now forced to 111 for both channels!\r\n");
											break;
									case 0:
									default:
											break;
								}
								Message(1, "CH_B Software Gain is now set to %d\r\n", gain);
							}
						}
						else
							Message(1, "Invalid command line arguments!\r\n");
					}
					else if (strncmp(cmdname, "pollmode ", 9) == 0)
					{
						if (carg1)
						{
							pollmode=strtol(carg1, NULL, 0);
							Message(1, "CAT polling mode %s\r\n", (pollmode)?"ON":"OFF");
						}
						else
							Message(1, "Invalid command line arguments!\r\n");
					}
					else if (strncmp(cmdname, "diversity ", 10) == 0)
					{
						if (carg1)
							Message(1, "Diversity mode %s\r\n", (DiversityMode(strtol(carg1, NULL, 0),f_adc))?"ON":"OFF");
						else
							Message(1, "Invalid command line arguments!\r\n");
					}
					else if (strncmp(cmdname, "agc ", 4) == 0)
					{
						if (carg1)
						{
							AGCMode=strtol(carg1, NULL, 0);
							Message(1, "AGC %s\r\n", (AGCMode)?"ON":"OFF");

							if (AGCMode)
							{
								WriteRegister(20, ReadRegister(20)&0xF7);	// enable AGC
							}
							else
							{
								WriteRegister(20, ReadRegister(20)|0x8);		// 00xxx01x
																				//   ||||||
																				//   |||||+- EXP_INH 0=allow exponent to pass into FLOAT TO FIXED converter 1=Force exponent in DDC channel to a 7 (max digital gain)
																				//   |||++-- Reserved, do not use
																				//   ||+---- AGC_HOLD_IC 0=normal closed loop operation 1=Hold integrator at initial condition.
																				//   ++----- Bit shift value for AGC loop. Valid range is from 0 to 3
								Message(1, "Use 'rfgs and rfgb commands to set gain manually.\r\n");
							}
						}
						else
							Message(1, "Invalid command line arguments!\r\n");
					}
					else if (strncmp(cmdname, "compressor ", 11) == 0)
					{
						if (carg1)
						{
							if (strtol(carg1, NULL, 0))
							{
								// turn on compressor (fix exponent) and set gain to 1
								Message(1, "Compressor ON (exponent fixed), gainA=ganB=1\r\n");
								WriteRegister(20, ReadRegister(20)|1);	// force exponent
																			// 00xxx01x
																			//   ||||||
																			//   |||||+- EXP_INH 0=allow exponent to pass into FLOAT TO FIXED converter 1=Force exponent in DDC channel to a 7 (max digital gain)
																			//   |||++-- Reserved, do not use
																			//   ||+---- AGC_HOLD_IC 0=normal closed loop operation 1=Hold integrator at initial condition.
																			//   ++----- Bit shift value for AGC loop. Valid range is from 0 to 3
								WriteRegister(3, 1);	// gain A =1
								WriteRegister(4, 1);	// gain B =1

							}
							else
							{
								// turn off compressor (release exponent) and set gain to 4
								Message(1, "Compressor OFF (exponent passed), gainA=ganB=4\r\n");
								WriteRegister(20, ReadRegister(20)&0xFE);	// pass exponent
								SetGain(CH_A, 4);
								SetGain(CH_B, 4);
								//WriteRegister(3, 4);	// gain A =4
								//WriteRegister(4, 4);	// gain B =4
							}
						}
						else
							Message(1, "Invalid command line arguments!\r\n");
					}
					else if (strncmp(cmdname, "rfga ", 5) == 0)
					{
					uint16_t rfgain;

						if (AGCMode)
						{
							Message(1, "RF Gain can be manually set only if AGC is disabled (cmd 'agc 0')\r\n");
						}
						else
						{
							if (carg1)
							{
								rfgain=strtol(carg1, NULL, 0);

								if (rfgain<8)
								{
									Message(1, "RF (DVGA) Gain for CH_A set to %d (%s%ddB gain)\r\n", rfgain, (rfgain)?"+":"", rfgain*6);
									fixedgain_A=rfgain;
									rfgain<<=5;
									rfgain|=0x1F;
									WriteRegister(23, ~(rfgain));
								}
								else
									Message(1, "RF Gain value has to be between 0 and 7\r\n");
							}
							else
								Message(1, "Invalid command line arguments!\r\n");
						}
					}
					else if (strncmp(cmdname, "rfgb ", 5) == 0)
					{
					uint16_t rfgain;

						if (AGCMode)
						{
							Message(1, "RF Gain can be manually set only if AGC is disabled (cmd 'agc 0')\r\n");
						}
						else
						{
							if (carg1)
							{
								rfgain=strtol(carg1, NULL, 0);

								if (rfgain<8)
								{
									fixedgain_B=rfgain;
									Message(1, "RF (DVGA) Gain for CH_B set to %d (%s%ddB gain)\r\n", rfgain, (rfgain)?"+":"", rfgain*6);
									rfgain<<=5;
									rfgain|=0x1F;
									WriteRegister(24, ~(rfgain));
								}
								else
									Message(1, "RF Gain value has to be between 0 and 7\r\n");
							}
							else
								Message(1, "Invalid command line arguments!\r\n");
						}
					}
					else if (strcmp(cmdname, "reset") == 0)
					{
						if (powered)
						{
							ResetRadio();
						}
						else
							Message(1, "Chip is not powered!\r\n");
					}
					else if (strcmp(cmdname, "omnibuff") == 0)
					{
						Message(1, "%s", omnibuff);
						memset(omnibuff, 0, 120);
						omnioff=0;
					}
					else if (strncmp(cmdname, "reg ", 4) == 0)
					{
					unsigned char regno, regval;

						if (carg1 && carg2)
						{
							regno=strtol(carg1, NULL, 0);
							regval=strtol(carg2, NULL, 0);

							WriteRegister(regno, regval);
							Message(1, "Register %d (0x%02.2X) set to 0x%02.2X\r\n", regno, regno, regval);
						}
						else
							Message(1, "Invalid command line arguments\r\n");

					}
					// New extio DLL commands
					else if (strncmp(cmdname, "_iqmode ", 8) == 0)
					{
					uint32_t samplerate;
					uint16_t samplebits;

						if (carg1 && carg2)
						{
							samplerate=strtol(carg1, NULL, 0);
							samplebits=strtol(carg2, NULL, 0);

							SampleMode(samplerate, DUAL_CHANNEL, samplebits, 0);		// IQ mode is single channel, but we will interleave samples ourselves to be able to fetch both channels data
							datamode=DATA_SDRIQ;
						}
						else
							Message(1, "Invalid command line arguments\r\n");
					}
					else if (strncmp(cmdname, "samplemode ", 11) == 0)
					{
					uint32_t samplerate;
					uint16_t samplebits;

						if (carg1 && carg2)
						{
							samplerate=strtol(carg1, NULL, 0);
							samplebits=strtol(carg2, NULL, 0);

							SampleMode(samplerate, DUAL_CHANNEL, samplebits, 0);
							Message(1, "Sample mode is now %ldHz/%dbit\r\n", samplerate, samplebits);
						}
						else
							Message(1, "Invalid command line arguments\r\n");
					}
					else if (strncmp(cmdname, "nvram", 5) == 0)
					{
					int a, b;
					void* flashdata;

						Message(1, "NVRAM Dump:\r\n");

						flashdata=(void*)&flash_nvram_data;

						for (a=0; a<(NVRAMSIZE/16); a++)
						{
							Message(1, "0x%08X  ", (unsigned long)flashdata+(a*16));
							for (b=0; b<16; b++)
								Message(1, "%02X ", *(uint8_t*)(flashdata+(a*16)+b));
							Message(1, "\r\n");
						}
					}
					else if (strncmp(cmdname, "ipconfig", 8) == 0)
					{
					unsigned char sdripaddr[4];

						Message(1, "Ethernet MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", uip_ethaddr.addr[0], uip_ethaddr.addr[1], uip_ethaddr.addr[2], uip_ethaddr.addr[3], uip_ethaddr.addr[4], uip_ethaddr.addr[5]);
						Message(1, "        DHCP: %s (state=%d)\r\n", (flash_nvram_data.dhcp)?"On":"Off", s.state);
						uip_gethostaddr(&sdripaddr);
						Message(1, "          IP: %d.%d.%d.%d\r\n", sdripaddr[0], sdripaddr[1], sdripaddr[2], sdripaddr[3]);
						uip_getdraddr(&sdripaddr);
						Message(1, "     Gateway: %d.%d.%d.%d\r\n", sdripaddr[0], sdripaddr[1], sdripaddr[2], sdripaddr[3]);
						uip_getnetmask(&sdripaddr);
						Message(1, "     Netmask: %d.%d.%d.%d\r\n", sdripaddr[0], sdripaddr[1], sdripaddr[2], sdripaddr[3]);
						Message(1, "\r\n");
					}
					else if (strncmp(cmdname, "ip ", 3) == 0)
					{
					unsigned char ipaddr[4];

						if (carg1)
						{
							if (ParseIPAddress(ipaddr, carg1))
								Message(1, "Invalid IP Address Format!\r\n");
							else
							{
								flashc_memcpy((void *)flash_nvram_data.ip, &ipaddr, sizeof(flash_nvram_data.ip), true);
								UpdateFlashCRC();
								Message(1, "     New IP: %d.%d.%d.%d\r\n", flash_nvram_data.ip[0], flash_nvram_data.ip[1], flash_nvram_data.ip[2], flash_nvram_data.ip[3]);
							}
						}
						else
							Message(1, "Invalid command line arguments\r\n");
					}
					else if (strncmp(cmdname, "gw ", 3) == 0)
					{
					unsigned char ipaddr[4];

						if (carg1)
						{
							if (ParseIPAddress(ipaddr, carg1))
								Message(1, "Invalid IP Address Format!\r\n");
							else
							{
								flashc_memcpy((void *)flash_nvram_data.gw, &ipaddr, sizeof(flash_nvram_data.gw), true);
								UpdateFlashCRC();
								Message(1, "New Gateway: %d.%d.%d.%d\r\n", flash_nvram_data.gw[0], flash_nvram_data.gw[1], flash_nvram_data.gw[2], flash_nvram_data.gw[3]);
							}
						}
						else
							Message(1, "Invalid command line arguments\r\n");
					}
					else if (strncmp(cmdname, "netmask ", 8) == 0)
					{
					unsigned char ipaddr[4];

						if (carg1)
						{
							if (ParseIPAddress(ipaddr, carg1))
								Message(1, "Invalid IP Address Format!\r\n");
							else
							{
								flashc_memcpy((void *)flash_nvram_data.netmask, &ipaddr, sizeof(flash_nvram_data.netmask), true);
								UpdateFlashCRC();
								Message(1, "New Netmask: %d.%d.%d.%d\r\n", flash_nvram_data.netmask[0], flash_nvram_data.netmask[1], flash_nvram_data.netmask[2], flash_nvram_data.netmask[3]);
							}
						}
						else
							Message(1, "Invalid command line arguments\r\n");
					}
					else if (strncmp(cmdname, "dhcp ", 5) == 0)
					{
					uint8_t dhcpmode;

						if (carg1)
						{
							dhcpmode=strtol(carg1, NULL, 0);

							if (dhcpmode == 1)
							{
								flashc_memcpy((void *)&flash_nvram_data.dhcp, "\1", sizeof(flash_nvram_data.dhcp), true);
								UpdateFlashCRC();
								Message(1, "DHCP mode ON. Please reset radio to obtain IP address from network.\r\n");
							}
							else if (dhcpmode == 0)
							{
								flashc_memcpy((void *)&flash_nvram_data.dhcp, "\0", sizeof(flash_nvram_data.dhcp), true);
								UpdateFlashCRC();
								Message(1, "DHCP mode OFF. Currently configured network parameters are:\r\n");
								Message(1, "          IP: %d.%d.%d.%d\r\n", flash_nvram_data.ip[0], flash_nvram_data.ip[1], flash_nvram_data.ip[2], flash_nvram_data.ip[3]);
								Message(1, "     Gateway: %d.%d.%d.%d\r\n", flash_nvram_data.gw[0], flash_nvram_data.gw[1], flash_nvram_data.gw[2], flash_nvram_data.gw[3]);
								Message(1, "     Netmask: %d.%d.%d.%d\r\n", flash_nvram_data.netmask[0], flash_nvram_data.netmask[1], flash_nvram_data.netmask[2], flash_nvram_data.netmask[3]);
								Message(1, "\r\n");
								Message(1, "Use commands 'ip' , 'gw' and 'netmask' to set appropriate values,\r\n");
								Message(1, "then reset radio for new settings to become active.\r\n");
							}
							else
								Message(1, "Invalid command line arguments\r\n");
						}
						else
							Message(1, "Invalid command line arguments\r\n");
					}
					// New extio DLL commands without echo
					else if (strncmp(cmdname, "_ga ", 4) == 0)
					{
						if (carg1)
						{
							gain=strtol(carg1, NULL, 0);
							SetGain(CH_A, gain);
						}
					}
					else if (strncmp(cmdname, "_gb ", 4) == 0)
					{
						if (carg1)
						{
							gain=strtol(carg1, NULL, 0);
							SetGain(CH_B, gain);
						}
					}
					else if (strncmp(cmdname, "_fa ", 4) == 0)
					{
						if (carg1)
						{
							tuningfreq=strtol(carg1, NULL, 0);
							SetFreq(CH_A, tuningfreq, 1, f_adc);
						}
					}
					else if (strncmp(cmdname, "_fb ", 4) == 0)
					{
						if (carg1)
						{
							tuningfreq=strtol(carg1, NULL, 0);
							SetFreq(CH_B, tuningfreq, 1, f_adc);
						}
					}
					else if (strncmp(cmdname, "_pa ", 4) == 0)		// phase word from 0 to 2^16
					{
						if (carg1)
						{
							phase_B=strtol(carg1, NULL, 0);
							SetPhase(CH_A, phase_B);
						}
					}
					else if (strncmp(cmdname, "_pb ", 4) == 0)		// phase word from 0 to 2^16
					{
						if (carg1)
						{
							phase_B=strtol(carg1, NULL, 0);
							SetPhase(CH_B, phase_B);
						}
					}
					else
					{
						Message(1, "Unknown Command [%s]\r\n", cmdbuffer);
					}

					memset(cmdbuffer, 0, 64+1);
					bufflen=0;
				}

				if (datamode!=DATA_SDRIQ)
					Message(1, "\r\n> ");		// display new prompt
			}
			else if ((recbyte == ';')&&(!scancode)&&(!omnirig))		// purge for omnirig in case we have some sort of garbage in buffer
			{
				Message(1, "E;");
				memset(cmdbuffer, 0, 64+1);
				bufflen=0;
			}
			else if (recbyte>=0)	// string entered
			{
				if (scancode)
				{
					Message(1, "Scan code is 0x%02.2X (%d)\r\n", recbyte, recbyte);
					if (scancode != 0xA)		// if LF, get another one
						scancode=false;
					Message(1, "\r\n> ");		// display new prompt
				}
				/**
				Omnirig processing starts here!
				**/
				else if ((omnirig)||((recbyte>='A')&&(recbyte<='Z')&&(bufflen<2)))		// already in omnirig mode or uppercase character received
				{
					omnirig=true;

					switch(recbyte)
					{
					case ';':	// process command
						if ((strncmp(cmdbuffer, "FB", 2)==0)||(strncmp(cmdbuffer, "FC", 2)==0))
						{
							if (strlen(cmdbuffer)==2)		// frequency readout
							{
								Message(1, "%c%c%11.11lu;", cmdbuffer[0], cmdbuffer[1], lastfreq_B);
							}
							else						// frequency selection
							{
								if (strlen(cmdbuffer)!=13)
									Message(1, "E;");
								else
								{
									for (a=2; a<strlen(cmdbuffer); a++)
										if (cmdbuffer[a] != '0')
											break;
									tuningfreq=strtol(cmdbuffer+a, NULL, 0);

									/******
									Now, the CAT interface on SDR-Radio is actually exhibiting a useful phenomena: It allows reading the actual
									VFO frequency, while setting the frequency is setting the center frequency.
									This allows us to syncronize LO frequency with the radio!
									******/


									if ((!pollmode)||(firstsync))		// working with HDSDR likely, so no gimmicks (or is the first message syncing with SDR-RADIO)
									{
										SetFreq(CH_A, tuningfreq, 1, f_adc);
										firstsync=false;				// do it only once if in poll mode
									}
									else
									{
										// only change the center frequency in case the new tuning frequency is more than SampleRate/2 apart from last
										if (tuningfreq>(lastfreq_A+(SampleRate/2)))
										{
											tuningfreq=lastfreq_A+(tuningfreq-(lastfreq_A+(SampleRate/2)));
											SetFreq(CH_A, tuningfreq, 1, f_adc);				// set new center freq, but only by the necessary increment
											Message(1, "FA%11.11lu;", tuningfreq);		// write back to the SDR-RADIO what we are using for a new center frequency
										}
										else if (tuningfreq<(lastfreq_A-(SampleRate/2)))
										{
											tuningfreq=lastfreq_A-((lastfreq_A-(SampleRate/2))-tuningfreq);
											SetFreq(CH_A, tuningfreq, 1, f_adc);				// set new center freq, but only by the necessary increment
											Message(1, "FA%11.11lu;", tuningfreq);		// write back to the SDR-RADIO what we are using for a new center frequency
										}
									}

									/*
									// to get the center frequency in sync, we have to set the center frequency correctly.
									if ((pollmode)&&(firstsync))
									{
										Message(1, "FA%11.11lu;", 100000);				// move to 1Hz
										lastfreq_A=100000;
										firstsync=false;
										delayms(300);
									}


									*/

									// in pollmode, sets ch A frequency
									//
									if ((pollmode)&&(lastfreq_A!=tuningfreq))
									{
										if ((tuningfreq > lastfreq_A) || (firstsync))
										{
											Message(1, "FB10000000000;");				// behave so that the frequency set will always be lower than exising
											if (firstsync)
												firstsync=false;
										}

										Message(1, "FB%11.11lu;", tuningfreq);		// write back to the SDR-RADIO what we are using for a new center frequency
										//not supported?! Message(1, "VS0;");							// set VFO-B the same value as VFO-A
										Message(1, "FR%c;", activeVFO);				// restore focus to active VFO
										//Message(1, "IF;");							// request actual tuning frequency with IF message (can be done with FA as well, but
																					// processing is easier this way)
										SetFreq(CH_A, tuningfreq, 1, f_adc);					// set new freq,
									}
									else if (!pollmode)
									{
										SetFreq(CH_B, tuningfreq, 1, f_adc);					// set new freq,
									}
								}
							}
						}
						else if (strncmp(cmdbuffer, "FA", 2)==0)
						{
							if (strlen(cmdbuffer)==2)		// frequency readout
							{
								Message(1, "FA%11.11lu;", lastfreq_A);
							}
							else						// frequency selection
							{
								if (strlen(cmdbuffer)!=13)
									Message(1, "E;");
								else
								{
									for (a=2; a<strlen(cmdbuffer); a++)
										if (cmdbuffer[a] != '0')
											break;
									tuningfreq=strtol(cmdbuffer+a, NULL, 0);
									SetFreq(CH_A, tuningfreq, 1, f_adc);
									//Message(1, "%s;", cmdbuffer);
								}
							}
						}
						else if (strncmp(cmdbuffer, "RG", 2)==0)		// note, that we can adjust the gain between 0 and 15, so we have to convert it to 0..255
						{
						int16_t lgain;

							if (strlen(cmdbuffer)==2)		// gain readout only
							{
								lgain=gain*17;		// normalize gain to 0..255
								Message(1, "RG%3.3d;", lgain);
							}
							else						// set gain message
							{
								if (strlen(cmdbuffer)!=5)
									Message(1, "E;");
								else
								{
									for (a=2; a<strlen(cmdbuffer); a++)
										if (cmdbuffer[a] != '0')
											break;
									gain=strtol(cmdbuffer+a, NULL, 0);
									gain/=17;			// make from 0..15 to store in global variable
									SetGain(CH_A, gain);
									SetGain(CH_B, gain);
									//Message(1, "%s;", cmdbuffer);
								}
							}
						}
						// "AG" is the compatibility mesage for SDR-RADIO device, as this allows us to remotely adjust RF gain.
						else if (strncmp(cmdbuffer, "AG", 2)==0)		// We are getting values between 0 and 100%, what we have to normalize to 0..15 to use as gain
						{
						uint16_t lgain;

							if (strlen(cmdbuffer)==2)					// gain readout only
							{
								lgain=(gain*100)/15;					// normalize gain to 0..100%
								Message(1, "AG%3.3d;", lgain);
							}
							else										// set gain message
							{
								if (strlen(cmdbuffer)!=5)
									Message(1, "E;");
								else
								{
									for (a=2; a<strlen(cmdbuffer); a++)
										if (cmdbuffer[a] != '0')
											break;
									gain=strtol(cmdbuffer+a, NULL, 0);
									gain*=15;			// make from 0..15 to store in global variable
									gain/=100;
									SetGain(CH_A, gain);
									SetGain(CH_B, gain);
									//Message(1, "%s;", cmdbuffer);
								}
							}
						}
						/*
						else if (strncmp(cmdbuffer, "IF", 2)==0)		// We are getting the actual tuning frequency of the VFO-A here after new center setting and have to calculate offset from that
						{
							if (strlen(cmdbuffer)==2)		// readout
							{
								Message(1, "?;");
							}
							else							// Parse IF frequency
							{
								if (strlen(cmdbuffer)<13)
									Message(1, "E;");
								else
								{
									cmdbuffer[13]=0;							// only use the frequency part for parsing
									for (a=2; a<strlen(cmdbuffer); a++)
										if (cmdbuffer[a] != '0')
											break;
									lastfreq_A=strtol(cmdbuffer+a, NULL, 0);	//
								}
							}
						}
						*/
						else if (strncmp(cmdbuffer, "FR", 2)==0)		// We are getting the actual tuning frequency of the VFO-A here after new center setting and have to calculate offset from that
						{
							if (strlen(cmdbuffer)==2)		// readout
							{
								Message(1, "?;");
							}
							else							// Parse IF frequency
							{
								if (strlen(cmdbuffer)!=3)
									Message(1, "E;");
								else
								{
									activeVFO=cmdbuffer[2];	// update active VFO
								}
							}
						}
						else
						{
							if (!pollmode)				// avoid feedback in poll mode
								Message(1, "?;");		// show syntax error or command unavailability
						}

						// store omnirig debugging data
						if (omnioff<(119-bufflen))
						{
							cmdbuffer[bufflen++]=';';
							memmove(omnibuff+omnioff, cmdbuffer, bufflen);
							omnioff+=bufflen;
						}

						// kill buffer
						memset(cmdbuffer, 0, 64+1);
						bufflen=0;
						omnirig=false;

						break;

					default:
						if (bufflen<64)
						{
							cmdbuffer[bufflen]=recbyte;
							bufflen++;
						}
						else
						{
							memset(cmdbuffer, 0, 64+1);
							bufflen=0;
							omnirig=false;
							Message(1, "E;");		// show communication error
						}

						break;
					}
				}
				/** Omnirig processing ends here **/
				else
				{
					switch(recbyte)
					{

					case 0x7F:		// backspace on putty terminal
					case 0x8:		// backspace on hercules terminal
						if (bufflen)
						{
							bufflen--;
							cmdbuffer[bufflen]=0;
							CDC_Device_SendByte(&VirtualSerial1_CDC_Interface, (uint8_t)recbyte);
						}
						break;

					case 0xA:		// LF
						if (showecho)
							CDC_Device_SendByte(&VirtualSerial1_CDC_Interface, (uint8_t)recbyte);
						break;		// ignore character, although echo back.

					case '7':		// if the first character of the line is number, we are using it for tuning up and down and changing channel B phase angle!
					case '4':
					case '8':
					case '5':
					case '9':
					case '6':
					case '+':
					case '-':
						if (!bufflen)
						{
							switch(recbyte)
							{
							/*
							case '+':
									skipvalue++;
									Message(1, "%d\r\n", skipvalue);
									break;

							case '-':
									skipvalue--;
									Message(1, "%d\r\n", skipvalue);
									break;
							*/
							case '7':
								if (lastfreq_A <= (64000000-TUNINGSTEP))
								{
									lastfreq_A+=TUNINGSTEP;
									SetFreq(CH_A, lastfreq_A, 1, f_adc);
								}
								Message(1, "A=%lu\r\n", lastfreq_A);
								break;

							case '4':
								if (lastfreq_A >= TUNINGSTEP)
								{
									lastfreq_A-=TUNINGSTEP;
									SetFreq(CH_A, lastfreq_A, 1, f_adc);
								}
								Message(1, "A=%lu\r\n", lastfreq_A);
								break;

							case '8':
								if (lastfreq_B <= (64000000-TUNINGSTEP))
								{
									lastfreq_B+=TUNINGSTEP;
									SetFreq(CH_B, lastfreq_B, 1, f_adc);
								}
								Message(1, "B=%lu\r\n", lastfreq_B);
								break;

							case '5':
								if (lastfreq_B >= TUNINGSTEP)
								{
									lastfreq_B-=TUNINGSTEP;
									SetFreq(CH_B, lastfreq_B, 1, f_adc);
								}
								Message(1, "B=%lu\r\n", lastfreq_B);
								break;

							case '9':
								if ((long)phase_B <= (0xFFFF-182))
								{
									phase_B+=182;

									if (phase_B >= 0)
										SetPhase(CH_B, phase_B);
									if (phase_B <=0)
										SetPhase(CH_A, 0-phase_B);		// note, that on 0 phase, both channels get updated!
								}

								Message(1, "phaseB=%d deg\r\n", phase_B/182);
								break;

							case '6':
								if ((long)phase_B >= (long)(-65535L+182L))
								{
									phase_B-=182;

									if (phase_B >= 0)
										SetPhase(CH_B, phase_B);
									if (phase_B <=0)
										SetPhase(CH_A, 0-phase_B);		// note, that on 0 phase, both channels get updated!
								}
								Message(1, "phaseB=%d deg\r\n", phase_B/182);
								break;
							}

							break;
						}

					default:		// any other characters
						if ((!bufflen)&&(recbyte == '_'))		// disable echo if first character is '_'
							showecho = false;

						if (bufflen<64)
						{
							cmdbuffer[bufflen]=recbyte;
							bufflen++;

							if (showecho)
								CDC_Device_SendByte(&VirtualSerial1_CDC_Interface, (uint8_t)recbyte);
						}
						break;
					}
				}
			}
		}	// network or audio interface

		switch (datamode)
		{
			case DATA_NETWORK:
				CDC_Device_USBTask(&VirtualSerial1_CDC_Interface);
				break;

			case DATA_LIBUSB:
			case DATA_LIBUSB_SPEEDTEST:
			case DATA_LIBUSB_XMY:
			case DATA_LIBUSB_XPY:
				CDC_Device_USBTask(&VirtualSerial2_CDC_Interface);
				break;

			case DATA_AUDIO:
			default:
				CDC_Device_USBTask(&VirtualSerial1_CDC_Interface);
				CDC_Device_USBTask(&VirtualSerial2_CDC_Interface);
				Audio_Device_USBTask(&Main_Audio_Interface);
				Audio_Device_USBTask(&AudioB_Audio_Interface);
				break;
		}

		USB_USBTask();
	}
}


/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	//LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
	usbinit_done=true;		// show that timer interrupt may start using the USB functions now
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	/* Stop the sample reload timer */
	//TCCR0B = 0;

	//LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	usbinit_done=false;
}

/*
Replacement function to Audio_Device_ConfigureEndpoints() in C:\engineering\Software\LUFA120219\LUFA\Drivers\USB\Class\Device\AudioClassDevice.c

We do not have enough endpoint memory to have two audio channels both running double-banked, but LUFA always configures audio endpoints as
double-banked. Therefore we are configuring audio endpoints ourselves.

*/

bool Audio_Device_ConfigureEndpoints_singlebank(USB_ClassInfo_Audio_Device_t* const AudioInterfaceInfo)
{
	memset(&AudioInterfaceInfo->State, 0x00, sizeof(AudioInterfaceInfo->State));

	for (uint8_t EndpointNum = 1; EndpointNum < ENDPOINT_TOTAL_ENDPOINTS; EndpointNum++)
	{
		uint16_t Size;
		uint8_t  Type;
		uint8_t  Direction;
		bool     DoubleBanked;

		if (EndpointNum == AudioInterfaceInfo->Config.DataINEndpointNumber)
		{
			Size         = AudioInterfaceInfo->Config.DataINEndpointSize;
			Direction    = ENDPOINT_DIR_IN;
			Type         = EP_TYPE_ISOCHRONOUS;
			DoubleBanked = false;
		}
		else if (EndpointNum == AudioInterfaceInfo->Config.DataOUTEndpointNumber)
		{
			Size         = AudioInterfaceInfo->Config.DataOUTEndpointSize;
			Direction    = ENDPOINT_DIR_OUT;
			Type         = EP_TYPE_ISOCHRONOUS;
			DoubleBanked = false;
		}
		else
		{
			continue;
		}

		if (!(Endpoint_ConfigureEndpoint(EndpointNum, Type, Direction, Size,
		                                 DoubleBanked ? ENDPOINT_BANK_DOUBLE : ENDPOINT_BANK_SINGLE)))
		{
			return false;
		}
	}

	return true;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	//!! ATTENTION !!
	// As we are using the ORDERED_EP_CONFIG build option (other does not seem to work for UC3B, although the code looks solid?!), the endpoints have to be configured in ascending order here!
	//
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial1_CDC_Interface);
	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial2_CDC_Interface);
	ConfigSuccess &= Audio_Device_ConfigureEndpoints_singlebank(&Main_Audio_Interface);
	ConfigSuccess &= Audio_Device_ConfigureEndpoints_singlebank(&AudioB_Audio_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial1_CDC_Interface);
	CDC_Device_ProcessControlRequest(&VirtualSerial2_CDC_Interface);
	Audio_Device_ProcessControlRequest(&Main_Audio_Interface);
	Audio_Device_ProcessControlRequest(&AudioB_Audio_Interface);

	// process additional control request stuff for CDC interface (as EVENT_USB_Device_UnhandledControlRequest() seems to be deprecated ..)

	if (!(Endpoint_IsSETUPReceived()))
	  return;

	if (USB_ControlRequest.wIndex == VirtualSerial2_CDC_Interface.Config.ControlInterfaceNumber)
	{
		// On LIBUSB mode, all command functions are routed through USB control requests for CDC2 interface (06), as this is the only way we can achieve the required 6.2MBit speed

		switch (USB_ControlRequest.bRequest)
		{
			case LIBUSB_MODE:		// bulk test requested by bulk.c libusb test routine

					if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQREC_DEVICE))
					{

						Endpoint_ClearSETUP();

						// just reply with one byte of data or Endpoint_ClearStatusStage() will stall
						while (!(Endpoint_IsINReady()));
						Endpoint_Write_8(0);

						Endpoint_ClearIN();
						Endpoint_ClearStatusStage();

						// in any case, set channel A and channel B to their respective RF frontends first
						WriteRegister(19, 0x4);
						panadapter = 0;

						switch (USB_ControlRequest.wValue)
						{
							case LIBMODE_OFF:
									gpio_set_pin_high(LED);
									datamode = DATA_AUDIO;
									break;

							case LIBMODE_SPEEDTEST:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB_SPEEDTEST;

									// Init variables for speed test. Not used really, but as we are using dmabuff_wN buffers, they have to point somewhere!
									roffseta=0;					// buffer start
									displacement=8;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_w1=&dmabuff_1[0];		//IA
									dmabuff_w2=&dmabuff_1[2];		//IB
									dmabuff_w3=&dmabuff_1[4];		//IA
									dmabuff_w4=&dmabuff_1[6];		//IB
									break;

							case LIBMODE_TXTEST:		// reserved for bulk.c write transfer test
									break;

							case LIBMODE_16ABPAN:
							case LIBMODE_16BAPAN:		// panadapter modes need to deliver 16-bit AB data
									panadapter = USB_ControlRequest.wValue;
									stepsdone = 0;			// reset panadapter frequency step counter
									currentpanentry = 0;	// start from the beginning of table

							case LIBMODE_16AB:
									if (USB_ControlRequest.wValue == LIBMODE_16AB)		// panadapter?
									{
										panadapter = 0;
									}
									else if (USB_ControlRequest.wValue == LIBMODE_16ABPAN)
									{
										// set channel A and B use channel A RF frontend
										WriteRegister(19, 0x0);
									}
									else // LIBMODE_16BAPAN
									{
										// set channel A and B use channel B RF frontend
										WriteRegister(19, 0x5);
									}

									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB;

									roffseta=0;					// buffer start
									displacement=8;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_w1=&dmabuff_1[0];		//IA
									dmabuff_w2=&dmabuff_1[2];		//IB
									dmabuff_w3=&dmabuff_1[4];		//IA
									dmabuff_w4=&dmabuff_1[6];		//IB
									break;

							case LIBMODE_16A:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB;

									roffseta=0;						// buffer start
									displacement=16;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_w1=&dmabuff_1[0];		//IA
									dmabuff_w2=&dmabuff_1[4];		//IA
									dmabuff_w3=&dmabuff_1[8];		//IA
									dmabuff_w4=&dmabuff_1[12];		//IA

									break;

							case LIBMODE_16B:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB;

									roffseta=0;						// buffer start shifted by
									displacement=16;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_w1=&dmabuff_1[2];		//IB
									dmabuff_w2=&dmabuff_1[6];		//IB
									dmabuff_w3=&dmabuff_1[10];		//IB
									dmabuff_w4=&dmabuff_1[14];		//IB

									break;

							case LIBMODE_16APB:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB_XPY;

									roffseta=0;						// buffer start shifted by
									displacement=16;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_x1=&dmabuff_1[0];		//IA
									dmabuff_x2=&dmabuff_1[1];		//QA
									dmabuff_x3=&dmabuff_1[4];		//IA
									dmabuff_x4=&dmabuff_1[5];		//QA
									dmabuff_x5=&dmabuff_1[8];		//IA
									dmabuff_x6=&dmabuff_1[9];		//QA
									dmabuff_x7=&dmabuff_1[12];		//IA
									dmabuff_x8=&dmabuff_1[13];		//QA

									dmabuff_y1=&dmabuff_1[2];		//IB
									dmabuff_y2=&dmabuff_1[3];		//QB
									dmabuff_y3=&dmabuff_1[6];		//IB
									dmabuff_y4=&dmabuff_1[7];		//QB
									dmabuff_y5=&dmabuff_1[10];		//IB
									dmabuff_y6=&dmabuff_1[11];		//QB
									dmabuff_y7=&dmabuff_1[14];		//IB
									dmabuff_y8=&dmabuff_1[15];		//QB

									break;

							case LIBMODE_16AMB:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB_XMY;

									roffseta=0;						// buffer start shifted by
									displacement=16;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_x1=&dmabuff_1[0];		//IA
									dmabuff_x2=&dmabuff_1[1];		//QA
									dmabuff_x3=&dmabuff_1[4];		//IA
									dmabuff_x4=&dmabuff_1[5];		//QA
									dmabuff_x5=&dmabuff_1[8];		//IA
									dmabuff_x6=&dmabuff_1[9];		//QA
									dmabuff_x7=&dmabuff_1[12];		//IA
									dmabuff_x8=&dmabuff_1[13];		//QA

									dmabuff_y1=&dmabuff_1[2];		//IB
									dmabuff_y2=&dmabuff_1[3];		//QB
									dmabuff_y3=&dmabuff_1[6];		//IB
									dmabuff_y4=&dmabuff_1[7];		//QB
									dmabuff_y5=&dmabuff_1[10];		//IB
									dmabuff_y6=&dmabuff_1[11];		//QB
									dmabuff_y7=&dmabuff_1[14];		//IB
									dmabuff_y8=&dmabuff_1[15];		//QB

									break;

							case LIBMODE_16BMA:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB_XMY;

									roffseta=0;						// buffer start shifted by
									displacement=16;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_y1=&dmabuff_1[0];		//IA
									dmabuff_y2=&dmabuff_1[1];		//QA
									dmabuff_y3=&dmabuff_1[4];		//IA
									dmabuff_y4=&dmabuff_1[5];		//QA
									dmabuff_y5=&dmabuff_1[8];		//IA
									dmabuff_y6=&dmabuff_1[9];		//QA
									dmabuff_y7=&dmabuff_1[12];		//IA
									dmabuff_y8=&dmabuff_1[13];		//QA

									dmabuff_x1=&dmabuff_1[2];		//IB
									dmabuff_x2=&dmabuff_1[3];		//QB
									dmabuff_x3=&dmabuff_1[6];		//IB
									dmabuff_x4=&dmabuff_1[7];		//QB
									dmabuff_x5=&dmabuff_1[10];		//IB
									dmabuff_x6=&dmabuff_1[11];		//QB
									dmabuff_x7=&dmabuff_1[14];		//IB
									dmabuff_x8=&dmabuff_1[15];		//QB

									break;

							case LIBMODE_24AB:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB;

									roffseta=0;					// buffer start
									displacement=8;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_w1=&dmabuff_1[0];		//IA
									dmabuff_w2=&dmabuff_1[2];		//QA
									dmabuff_w3=&dmabuff_1[4];		//IB
									dmabuff_w4=&dmabuff_1[6];		//QB

									break;

							case LIBMODE_24A:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB;

									roffseta=0;						// buffer start
									displacement=16;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_w1=&dmabuff_1[0];		//IA
									dmabuff_w2=&dmabuff_1[2];		//QA
									dmabuff_w3=&dmabuff_1[8];		//IA
									dmabuff_w4=&dmabuff_1[10];		//QA

									break;

							case LIBMODE_24B:
									gpio_set_pin_low(LED);
									datamode = DATA_LIBUSB;

									roffseta=0;						// buffer start
									displacement=16;				// how many words (16-bit) to advance in buffer on each pass

									dmabuff_w1=&dmabuff_1[4];		//IB
									dmabuff_w2=&dmabuff_1[6];		//QB
									dmabuff_w3=&dmabuff_1[12];		//IB
									dmabuff_w4=&dmabuff_1[14];		//QB

									break;

							default:
									break;
						}
					}

					break;

			case LIBUSB_SAMPLEMODE:			// Set appropriate LM97593 sampling rate and bit depth

					if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_DEVICE))
					{
					uint32_t libusb_samplerate;
					uint8_t numbits;

						if (USB_ControlRequest.wLength == 5)
						{
							Endpoint_ClearSETUP();

							while (!(Endpoint_IsOUTReceived()));

							libusb_samplerate = Endpoint_Read_32_LE();
							numbits = Endpoint_Read_8();

							//Message(1, "LIBUSB_SAMPLEMODE: SampleRate=%ld Bits=%d\n", libusb_samplerate, numbits);
							SampleMode(libusb_samplerate, DUAL_CHANNEL, numbits, 0);		// libusb mode can be single- or dual channel, but we are interleaving it ourselves to be able to access both channels

							Endpoint_ClearOUT();
							Endpoint_ClearStatusStage();

							Endpoint_ClearSETUP();

							//SampleMode(libusb_samplerate, numbits); cant be here somehow, or the whole system hangs up (de-stalls on next successful write to whatever edpoint, but thats nonsense)
						}
					}

					break;

			case LIBUSB_WRITEREG:

					if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_DEVICE))
					{
					uint16_t i, firstreg;

						Endpoint_ClearSETUP();

						while (!(Endpoint_IsOUTReceived()));

						firstreg=USB_ControlRequest.wValue;

						for (i=0; i<USB_ControlRequest.wLength; i++)
						{
							WriteRegister(firstreg++, Endpoint_Read_8());
						}

						//AssertSI();

						Endpoint_ClearOUT();
						Endpoint_ClearStatusStage();

						Endpoint_ClearSETUP();
					}

					break;

			case LIBUSB_READREG:

					if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQREC_DEVICE))
					{
					uint16_t i, firstreg;

						Endpoint_ClearSETUP();

						while (!(Endpoint_IsINReady()));

						firstreg=USB_ControlRequest.wValue;

						for (i=0; i<USB_ControlRequest.wLength; i++)
						{
							Endpoint_Write_8(ReadRegister(firstreg++));
						}

						Endpoint_ClearIN();
						Endpoint_ClearStatusStage();
					}

					break;

			case LIBUSB_PANTABLE:

					// fetch global table for running panoramic scan on secondary channel
					if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_DEVICE))
					{
					uint16_t i, regions;

						Endpoint_ClearSETUP();

						while (!(Endpoint_IsOUTReceived()));

						panentries = USB_ControlRequest.wLength/sizeof(PANENTRY);

						if (panentries == USB_ControlRequest.wValue)
						{
							if (pantable)
								pantable = realloc(pantable, panentries*sizeof(PANENTRY));
							else
								pantable = malloc(panentries*sizeof(PANENTRY));

							for (i=0; i<USB_ControlRequest.wLength; i++)
							{
								//WriteRegister(firstreg++, Endpoint_Read_8());
								pantable[i]=Endpoint_Read_8();
							}

							// multiply sample count by 4 for each entry, so we do not have to do this for each cycle

							for (i=0; i<panentries; i++)
							{
								panentry=pantable+(i*sizeof(PANENTRY));
								panentry->samples*=4;		// multiply by 4, as each sample in our context is 2 IQ pairs, i.e. 4 words
								panentry->skip*=4;
							}

							stepsdone=0;		//reset step scanner
						}

						//AssertSI();

						Endpoint_ClearOUT();
						Endpoint_ClearStatusStage();

						Endpoint_ClearSETUP();
					}

					break;

			// return 8 bytes starting from dmabuff_1[0] to allow host to control if the I/Q bulk stream is correctly aligned.
			case LIBUSB_GETVER:

					if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_VENDOR | REQREC_DEVICE))
					{
						if (USB_ControlRequest.wLength == 4)
						{
							Endpoint_ClearSETUP();

							while (!(Endpoint_IsINReady()));

							Endpoint_Write_16_BE(VER_MAJOR);
							Endpoint_Write_16_BE(VER_MINOR);

							Endpoint_ClearIN();
							Endpoint_ClearStatusStage();
						}
					}

					break;

			case LIBUSB_SETFREQ:

					if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_DEVICE))
					{
					uint32_t libusb_freq;

						if (USB_ControlRequest.wLength == 4)
						{
							Endpoint_ClearSETUP();

							while (!(Endpoint_IsOUTReceived()));

							libusb_freq = Endpoint_Read_32_LE();

							if (USB_ControlRequest.wValue == LIBUSB_CHA)
							{
								SetFreq(CH_A, libusb_freq, 1, f_adc);
							}
							else if (USB_ControlRequest.wValue == LIBUSB_CHB)
							{
								SetFreq(CH_B, libusb_freq, 1, f_adc);
							}

							//Message(1, "LIBUSB_SETFREQ: Freq=%ld Channel=%d\n", libusb_freq, USB_ControlRequest.wValue);

							Endpoint_ClearOUT();
							Endpoint_ClearStatusStage();

							Endpoint_ClearSETUP();
						}
					}

					break;

			case LIBUSB_SETGAIN:

					if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_DEVICE))
					{
					uint16_t libusb_gain;

						if (USB_ControlRequest.wLength == 2)
						{
							Endpoint_ClearSETUP();

							while (!(Endpoint_IsOUTReceived()));

							libusb_gain = Endpoint_Read_16_LE();

							//Message(1, "Gain value received=%04X\n", libusb_gain);

							if (USB_ControlRequest.wValue == LIBUSB_CHA)
							{
								SetGain(CH_A, libusb_gain);
							}
							else if (USB_ControlRequest.wValue == LIBUSB_CHB)
							{
								SetGain(CH_B, libusb_gain);
							}

							//Message(1, "LIBUSB_SETFREQ: Freq=%ld Channel=%d\n", libusb_freq, USB_ControlRequest.wValue);

							Endpoint_ClearOUT();
							Endpoint_ClearStatusStage();

							Endpoint_ClearSETUP();
						}
					}

					break;

			case LIBUSB_SETPHASE:

					if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_VENDOR | REQREC_DEVICE))
					{
					uint16_t libusb_phase;

						if (USB_ControlRequest.wLength == 2)
						{
							Endpoint_ClearSETUP();

							while (!(Endpoint_IsOUTReceived()));

							libusb_phase = Endpoint_Read_16_LE();

							//Message(1, "Phase value received=%04X\n", libusb_phase);

							if (USB_ControlRequest.wValue == LIBUSB_CHA)
							{
								SetPhase(CH_A, libusb_phase);
							}
							else if (USB_ControlRequest.wValue == LIBUSB_CHB)
							{
								SetPhase(CH_B, libusb_phase);
							}

							//Message(1, "LIBUSB_SETFREQ: Freq=%ld Channel=%d\n", libusb_freq, USB_ControlRequest.wValue);

							Endpoint_ClearOUT();
							Endpoint_ClearStatusStage();

							Endpoint_ClearSETUP();
						}
					}

					break;

			case LIBUSB_GETFREQ:

					break;

			/*

			case CDC_REQ_GetLineEncoding:
				if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
				{
					Endpoint_ClearSETUP();

					while (!(Endpoint_IsINReady()));

					Endpoint_Write_32_LE(CDCInterfaceInfo->State.LineEncoding.BaudRateBPS);
					Endpoint_Write_8(CDCInterfaceInfo->State.LineEncoding.CharFormat);
					Endpoint_Write_8(CDCInterfaceInfo->State.LineEncoding.ParityType);
					Endpoint_Write_8(CDCInterfaceInfo->State.LineEncoding.DataBits);

					Endpoint_ClearIN();
					Endpoint_ClearStatusStage();
				}

				break;
			case CDC_REQ_SetLineEncoding:
				if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
				{
					Endpoint_ClearSETUP();

					while (!(Endpoint_IsOUTReceived()));

					CDCInterfaceInfo->State.LineEncoding.BaudRateBPS = Endpoint_Read_32_LE();
					CDCInterfaceInfo->State.LineEncoding.CharFormat  = Endpoint_Read_8();
					CDCInterfaceInfo->State.LineEncoding.ParityType  = Endpoint_Read_8();
					CDCInterfaceInfo->State.LineEncoding.DataBits    = Endpoint_Read_8();

					Endpoint_ClearOUT();
					Endpoint_ClearStatusStage();

					EVENT_CDC_Device_LineEncodingChanged(CDCInterfaceInfo);
				}

				break;
			case CDC_REQ_SetControlLineState:
				if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
				{
					Endpoint_ClearSETUP();
					Endpoint_ClearStatusStage();

					CDCInterfaceInfo->State.ControlLineStates.HostToDevice = USB_ControlRequest.wValue;

					EVENT_CDC_Device_ControLineStateChanged(CDCInterfaceInfo);
				}

				break;
			case CDC_REQ_SendBreak:
				if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
				{
					Endpoint_ClearSETUP();
					Endpoint_ClearStatusStage();

					EVENT_CDC_Device_BreakSent(CDCInterfaceInfo, (uint8_t)USB_ControlRequest.wValue);
				}

				break;
			*/
			default:
				break;
		}
	}
}

/*
void EVENT_USB_Host_DeviceEnumerationComplete(void)
{

}
*/

