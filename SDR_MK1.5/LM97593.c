/*
 * LM97593.c
 *
 * Radio chipset related functionality
 *
 */

#include "sdr_mk1.5.h"
#include "LM97593.h"
#include "gpio.h"
#include "clock-arch.h"		// gives definition for millis()
#include "cycle_counter\cycle_counter.h"

#include "spi.h"
#include "eth_spi.h"
#include "tx8m.h"

#include <string.h>

// Build mask for Address pins
uint32_t address_bitmask =	(1<<(AVR32_PIN_PA12 & 0x1F))|		//A0
							(1<<(AVR32_PIN_PA13 & 0x1F))|		//A1, Also HWBE. Keep as input if idle, as this is also a button

							(1<<(AVR32_PIN_PA26 & 0x1F))|		//A2
							(1<<(AVR32_PIN_PA27 & 0x1F))|		//A3
							(1<<(AVR32_PIN_PA28 & 0x1F))|		//A4
							(1<<(AVR32_PIN_PA29 & 0x1F))|		//A5
							(1<<(AVR32_PIN_PA30 & 0x1F))|		//A6
							(1<<(AVR32_PIN_PA31 & 0x1F));		//A7
// Build mask for Data pins
uint32_t data_bitmask =		(1<<(AVR32_PIN_PA06 & 0x1F))|		//D0
							(1<<(AVR32_PIN_PA07 & 0x1F))|		//D1
							(1<<(AVR32_PIN_PA08 & 0x1F))|		//D2

							(1<<(AVR32_PIN_PA20 & 0x1F))|		//D3
							(1<<(AVR32_PIN_PA21 & 0x1F))|		//D4
							(1<<(AVR32_PIN_PA22 & 0x1F))|		//D5
							(1<<(AVR32_PIN_PA23 & 0x1F))|		//D6
							(1<<(AVR32_PIN_PA24 & 0x1F));		//D7
// Build mask for Control pins
uint32_t ctrl_bitmask =		(1<<(AVR32_PIN_PB09 & 0x1F))|		//xCE
							(1<<(AVR32_PIN_PB10 & 0x1F))|		//xWR
							(1<<(AVR32_PIN_PB11 & 0x1F));		//xRD




// LM97593 register value definitions
REGSTRUCT regstruct[REGENTRIES] = {
	{DEC, 			1, 	"         DEC", R_DISPLAY|R_INIT},	// last digit bitmap shows, what the routines shall do with the entry:
	{DEC_BY_4,		1, 	"    DEC_BY_4", R_DISPLAY|R_INIT},	//		bit:0 = Display in register du
	{SCALE, 		1,	"       SCALE", R_DISPLAY|R_INIT|R_MONTECARLO},	//		bit:1 = normal read-write field
	{GAIN_A, 		1, 	"      GAIN_A", R_DISPLAY|R_INIT|R_MONTECARLO},	//		2 = field what is used only during the initial upload for writing
	{GAIN_B, 		1, 	"      GAIN_B", R_DISPLAY|R_INIT|R_MONTECARLO},
	{RATE, 			1, 	"        RATE", R_DISPLAY|R_INIT|R_MONTECARLO},
	{SERIAL_CTRL, 	1,	" SERIAL_CTRL", R_DISPLAY|R_INIT|R_MONTECARLO},
	{FREQ_A, 		4, 	"      FREQ_A", R_DISPLAY|R_INIT|R_MONTECARLO},
	{PHASE_A, 		2, 	"     PHASE_A", R_DISPLAY|R_INIT|R_MONTECARLO},
	{FREQ_B, 		4, 	"      FREQ_B", R_DISPLAY|R_INIT|R_MONTECARLO},
	{PHASE_B, 		2,	"     PHASE_B", R_DISPLAY|R_INIT|R_MONTECARLO},
	{SOURCE, 		1,	"      SOURCE", R_DISPLAY|R_INIT|R_MONTECARLO},
	{AGC_CTRL, 		1, 	"    AGC_CTRL", R_DISPLAY|R_INIT|R_MONTECARLO},
	{AGC_IC_A, 		1, 	"    AGC_IC_A", R_DISPLAY|R_INIT|R_MONTECARLO},
	{AGC_IC_B, 		1, 	"    AGC_IC_B", R_DISPLAY|R_INIT|R_MONTECARLO},
	{AGC_RB_A, 		1,	"    AGC_RB_A", R_DISPLAY||R_MONTECARLO},
	{AGC_RB_B, 		1, 	"    AGC_RB_B", R_DISPLAY||R_MONTECARLO},
	{TEST_REG, 		2,	"    TEST_REG", R_DISPLAY|R_INIT},
	{DEBUG, 		1, 	"       DEBUG", R_DISPLAY|R_INIT},
	{AGC_TABLE, 	32,	"   AGC_TABLE", R_DISPLAY|R_INIT|R_MONTECARLO},

	{F1_CTRL, 		1, 	"     F1_CTRL", R_INIT},
	{F2_CTRL, 		1, 	"     F2_CTRL", R_INIT},
	{F1_COEFF, 		22,	"    F1_COEFF", R_INIT},
	{F2_COEFF, 		64,	"    F2_COEFF", R_INIT},

	{F1_CTRL, 		1, 	"     F1_CTRL", R_INIT|R_MONTECARLO},
	{F2_CTRL, 		1, 	"     F2_CTRL", R_INIT|R_MONTECARLO},
	{F1_COEFF, 		22,	"    F1_COEFF", R_DISPLAY|R_INIT},
	{F2_COEFF, 		64,	"    F2_COEFF", R_DISPLAY|R_INIT},

	{F1_CTRL, 		1, 	"     F1_CTRL", R_DISPLAY|R_INIT},
	{F2_CTRL, 		1, 	"     F2_CTRL", R_DISPLAY|R_INIT},
	{SERIAL_CTRL2,	1,	"SERIAL_CTRL2", R_DISPLAY|R_INIT},
	{AGC_CTRL2, 	1, 	"   AGC_CTRL2", R_DISPLAY|R_INIT},

	{SERIAL_CTRL, 	1,	" SERIAL_CTRL", R_INIT|R_ASSERT_SI}							// seems that sample format has to be programmed as a last thing all over, otherwise the filters will not boot up
};


REGVALS reg;						// register values. This one goes to memory
volatile bool powered=false;		// tracking the radio chip power status.
volatile uint8_t diversity=0;		// tracking diversity mode - set to 1 for diversity mode
volatile uint8_t scanner=0;			// channel B in scanning mode

int16_t GainCH_A=0;
int16_t	GainCH_B=0;

/*
How to calculate filter coefficients:

Go to web page http://www-users.cs.york.ac.uk/~fisher/mkfilter/racos.html

Enter sample frequency and corner frequency. (f.ex. 400000 and 88000). Note, that F1 and F2
filters are decimating by 2 by default, so the output sample rate is half the input rate
and filtering point is normally kept slightly below that.

The beta of the filter is ok to keep at 0.3

Impulse length is the number of taps, use 21 for F1 and 63 for F2.

You can truncate coefficients to 16 bit if you wish.

The resulting numbers have to be multiplied by 32767 (this is the max value)
*/


//Initialize LM97593 Register table values for SDR MK1

// default filter table values.
int16_t _F1_TABLE_STD[11] = {29, -85, -308, -56, 1068, 1405, -2056, -6009, 1303, 21121, 32703};			// standard set, see datasheet pg.30
int16_t _F2_TABLE_STD[32] = {-14, -20, 19, 73, 43, -70, -82, 84, 171, -49, -269, -34, 374, 192, -449,	// standard set, see datasheet pg.30
					-430, 460, 751, -357, -1144, 81, 1581, 443, -2026, -1337, 2437, 2886,
					-2770, -6127, 2987, 20544, 29647};

// used 1/16 tables used for delivering 12kHz signal on CH B when CH A is in 48kHz mode
// ------
// calculated set: http://www-users.cs.york.ac.uk/~fisher/mkfilter/racos.html
// sr=400000, fq=87000 b=0.3 63-tap raised cos, no sqr, no comp, 16 bit, -100db
// then multiply the resulting values with 65535. Note, that gain has to be normalized to 1,
// so filter coefficients have to be divided or multiplied appropriately

int16_t _F1_TABLE_SDR[11] = {184,-116,-769,-173,1752,1432,-2924,-4924,3891,20137,28555};
int16_t _F2_TABLE_SDR[32] = {10,2,-10,-3,1,-5,5,20,0,-27,-10,11,-3,13,48,-10,-96,-29,78,26,33,184,
						 -116,-767,-173,1749,1429,-2919,-4915,3884,20102,28506};

// vigane
//int16_t _F1_TABLE_WIDE[11] = {37, 61, 187, -594, -576, 1947, 1037, -5194, -1415, 19349, 32766};			// same as below, 21-tap
//int16_t _F2_TABLE_WIDE[32] = {5, -3, -1, -2, -4, 10, 5, -11, -1, -2, -1, 23, -2, -30, 5, 3, 10,			// sr=400000, fq=100000 b=0.3 63-tap raised cos, no sqr, no comp, 16 bit, -100db
//					55, -50, -95, 71, 37, 61, 187, -594, -576, 1947, 1037, -5194, -1415, 19349,		// then multiply the resulting values with 32767
//					32766};


//vigane
//int16_t _F1_TABLE_NORMAL[11] = {72, 199, -427, -864, 942, 2403, -1497, -5986, 1929, 21684, 32766};			// sr=400000, fq=94000 b=0.3 21-tap raised cos, no sqr, no comp, 16 bit, -100db, then multiply the resulting values with 32767
//int16_t _F2_TABLE_NORMAL[32] = {-4, -2, 10, 6, -10, -5, 1, -6, 10, 22, -12, -29, 5, 6, 1, 50, 13,
//							105, -40, 91, 19, 72, 199, -427, -864, 942, 2403, -1497, -5986, 1929, 21684, 32766};	// 63-tap, same as above

//vigane
//int16_t _F1_TABLE_QUARTER[11] = {2403, 1171, -1497, -4501, -5986, -4122, 1929, 11344, 21684, 29731, 32766};			// sr=400000, fq=47000 b=0.3 21-tap raised cos, no sqr, no comp, 16 bit, -100db, then multiply the resulting values with 32767
//int16_t _F2_TABLE_QUARTER[32] = {52, 13, -53, -105, -103, -40, 44, 91, 72, 19, 3, 72, 182, 199, -7, -427, -835, 		// 63-tap, same as above
//							-864, -240, 942, 2108, 2403, 1171, -1497, -4501, -5986, -4122, 1929, 11344,
//							21684, 29731, 32766};

// used 1/16 tables used for delivering 12kHz signal on CH B when CH A is in 48kHz mode
// ------
// calculated set: http://www-users.cs.york.ac.uk/~fisher/mkfilter/racos.html
// sr=400000, fq=21750 b=0.3 63-tap raised cos, no sqr, no comp, 16 bit, -100db
// then multiply the resulting values with 65535. Note, that gain has to be normalized to 1,
// so filter coefficients have to be divided or multiplied appropriately

// int16_t _F1_TABLE_16[11] = {-447, 124, 847, 1687, 2596, 3516, 4384, 5138, 5722, 6091, 6217};
// int16_t _F2_TABLE_16[32] = {-198, -178, -126, -43,67,194,322,433,508,528,479,354,154,-109,
//						-412,-722,-1001,-1204,-1289,-1217,-961,-507,141,961,1915,2947,3991,4976,5831,
//						6494,6913,7056};


//int8_t _AGC_TABLE[32] =	{-102, -102, -88, -80, -75, -70, -66, -63, -61, -56, -53, -50, -47, -42, -39, -36, -33, -29, -25,	// 6dB below full scale, deadband 8dB, loop gain 0.108. See datasheet pg.42
//						 -22, -19, -15, -11, -0, 0, 0, 0, 0, 0, 13, 17, 20};

// test table
//int8_t _AGC_TABLE[32] =	{-102, -102, -88, -80, -75, -70, -66, -63, -61, -56, -53, -50, -47, -42, -39, -36, -33, -29, -25,
//						 -22, -19, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20};

#define AGC_SHIFT	(2<<4)
int8_t _AGC_TABLE[32] =	{-85,-85,-74,-67,-63,-59,-56,-54,-52,-48,-45,-43,-41,-37,-34,-32,-29,-26,-23,-20,-18,-15,-12,0/*-9*/,0,0,0,0,0,0,0/*11*/,13};

void AssertSI(void)
{

	//if (!powered)		// dont touch if chip is not powered
	//	return;

	gpio_set_pin_low(SI);
	cpu_delay_us(1000, F_CPU);
	gpio_set_pin_high(SI);
}

void SetSI(int state)
{
	if (state)
		gpio_set_pin_high(SI);
	else
		gpio_set_pin_low(SI);
}


void WriteRegister(unsigned char regno, unsigned char regval)
{
uint32_t data_low, data_high, addr_low, addr_high;

	if (!powered)		// don't touch if chip is not powered
		return;

	//check, if A1 or A3 are low (means that HWBE or BTN1 (optional) is pressed and we shall not drive address bus
	if ((!gpio_get_pin_value(HWBE)) /*|| (!gpio_get_pin_value(BTN1)) stalls system when R67 is not populated! */)
		return;

	gpio_configure_group(PORTA, address_bitmask|data_bitmask, GPIO_DIR_OUTPUT|GPIO_INIT_LOW);	// enable address and bus and clear bits

	data_low=(regval&7);				// D0,D1,D2
	data_high=(regval&0xF8);			// D3,D4,D5,D6,D7

	data_low<<=AVR32_PIN_PA06;		// D0 is now at the place of PA06
	data_high<<=AVR32_PIN_PA20-3;		// D3 is now at the place of PA20

	addr_low=(regno&3);				// A0,A1
	addr_high=(regno&0xFC);			// A2,A3,A4,A5,A6,A7

	addr_low<<=AVR32_PIN_PA12;		// A0 is now at the place of PA12
	addr_high<<=AVR32_PIN_PA26-2;		// A2 is now at the place of PA26

	gpio_set_group_high(PORTA, data_low|data_high|addr_low|addr_high);		// output address and data

	cpu_delay_us(10, F_CPU);		// 2000 the address bus is slow, as we do have 10K resistors between shared signals. Therefore needed delay is longer than normal

	gpio_set_pin_low(xWR);
	cpu_delay_us(10, F_CPU);
	gpio_set_pin_high(xWR);

	gpio_configure_group(PORTA, address_bitmask, GPIO_DIR_INPUT);		// free address bus for input (buttons etc.)
	gpio_configure_group(PORTA, data_bitmask, GPIO_DIR_INPUT);		// set data bus as input (no need really to do that)
}

/*
// For using with panadapter frequency writes
void WriteRegister_Fast(unsigned char regno, unsigned char regval)
{
uint32_t data_low, data_high, addr_low, addr_high;

	//check, if A1 or A3 are low (means that HWBE or BTN1 (optional) is pressed and we shall not drive address bus
	if ((!gpio_get_pin_value(HWBE)))	//|| (!gpio_get_pin_value(BTN1)) stalls system when R67 is not populated! 
		return;

	gpio_configure_group(PORTA, address_bitmask|data_bitmask, GPIO_DIR_OUTPUT|GPIO_INIT_LOW);	// enable address and bus and clear bits

	data_low=(regval&7);				// D0,D1,D2
	data_high=(regval&0xF8);			// D3,D4,D5,D6,D7

	data_low<<=AVR32_PIN_PA06;		// D0 is now at the place of PA06
	data_high<<=AVR32_PIN_PA20-3;		// D3 is now at the place of PA20

	addr_low=(regno&3);				// A0,A1
	addr_high=(regno&0xFC);			// A2,A3,A4,A5,A6,A7

	addr_low<<=AVR32_PIN_PA12;		// A0 is now at the place of PA12
	addr_high<<=AVR32_PIN_PA26-2;		// A2 is now at the place of PA26

	gpio_set_group_high(PORTA, data_low|data_high|addr_low|addr_high);		// output address and data

	cpu_delay_us(1, F_CPU);		// was 10 // the address bus is slow, as we do have 10K resistors between shared signals. Therefore needed delay is longer than normal

	gpio_set_pin_low(xWR);
	cpu_delay_us(1, F_CPU);
	gpio_set_pin_high(xWR);

	gpio_configure_group(PORTA, address_bitmask, GPIO_DIR_INPUT);	// free address bus for input (buttons etc.)
	//gpio_configure_group(PORTA, data_bitmask, GPIO_DIR_INPUT);		// set data bus as input (no need really to do that)
}
*/

unsigned char ReadRegister(unsigned char regno)
{
unsigned char retval;
uint32_t data_low, data_high, addr_low, addr_high;
volatile avr32_gpio_port_t *gpio_port = &AVR32_GPIO.port[PORTA];

	if (!powered)
		return(0xFF);

	//check, if A1 or A3 are low (means that HWBE or BTN1 (optional) is pressed and we shall not drive address bus
	if ((!gpio_get_pin_value(HWBE)) /*|| (!gpio_get_pin_value(BTN1)) stalls system when R67 is not populated! */)
		return(0xFF);

	gpio_configure_group(PORTA, address_bitmask, GPIO_DIR_OUTPUT|GPIO_INIT_LOW);	// enable addres bus and clear bits
	gpio_configure_group(PORTA, data_bitmask, GPIO_DIR_INPUT);						// data as input

	addr_low=(regno&3);				// A0,A1
	addr_high=(regno&0xFC);			// A2,A3,A4,A5,A6,A7
	addr_low<<=AVR32_PIN_PA12;		// A0 is now at the place of PA12
	addr_high<<=AVR32_PIN_PA26-2;		// A2 is now at the place of PA26

	gpio_set_group_high(PORTA, addr_low|addr_high);	// output new address

	cpu_delay_us(10, F_CPU);		// the address bus is slow, as we do have 10K resistors between shared signals. Therefore needed delay is longer than normal

	gpio_set_pin_low(xRD);			// assert read
	cpu_delay_us(10, F_CPU);
	data_low=gpio_port->pvr;
	gpio_set_pin_high(xRD);			// release read

	gpio_configure_group(PORTA, address_bitmask, GPIO_DIR_INPUT);		// free address bus for input (buttons etc.)

	data_high=data_low;
	data_high>>=AVR32_PIN_PA20-3;		// D3 is now bit 3
	data_low>>=AVR32_PIN_PA06;		// D0 is now at bit 0
	data_high&=0xF8;				// strip surplus bits
	data_low&=7;

	retval=(data_high|data_low);
	return retval;
}


uint8_t DiversityMode(uint8_t mode, uint32_t adcfreq)
{
	diversity=mode;

	if (diversity)
	{
		SetFreq(CH_B, lastfreq_A, 1, adcfreq);		// sync frequencys
		WriteRegister(246, 0);
		WriteRegister(247, 0);					// use filter set A for both channels, as both have to be full-bandwidth
	}
	else
	{
		WriteRegister(246, 2);
		WriteRegister(247, 2);					// use filter set A for channel A and filter set B for channel B
	}
	
	InitIQDataEngine(0,0,1);						// re-sync phase

	return diversity;
}

volatile int32_t lastfreq_A=0L;
volatile int32_t lastfreq_B=0L;

/*
Calculates the correct FREQ_x value based on input frequency in Hz

The tuning is made by setting the FREQ_x=(2^32)F/Fck where F is the desired tuning frequency inside the band,
Ranging from -FCK/2 to +FCK/2 (Well, to +FCK(1-2^-31)/2 actually ..)

Our function takes a absolute frequency as a parameter, ranging therefore from 0Hz to 32MHz

*/

int TX8Mengaged=-1;

void SetFreq(int16_t channel, int32_t _tunefreq, int16_t write, uint32_t adcfreq)
{
int64_t full;
int64_t clockfreq;
int64_t freq;
int32_t freqval, lofreq;
char* freqstore;
int16_t i;


#if (TX8M == 1)		// do we have downconverter option ready for us?
uint32_t tx8mfreq, iffreq;
uint32_t steppedfreq;

	if (_tunefreq >= ((f_adc/2)))// - 1000000))		// skip 1 meg from the end, since this will likely start having some mirror artifacts already!
	{
		if (TX8Mengaged != 1)
		{

			//connect downconverter
			expander_set(RELAYS);

			TX8Mengaged=1;	
		}
		

		iffreq=7890000;							// just a number somewhere quiet where MK1.5 internal birdies are minimal within +/-1MHz
		tx8mfreq=_tunefreq-iffreq;				// actual downconverter frequency

		steppedfreq=tx8mfreq/5000;				// we are using 10khz stepping for ADF4351. Since the LO will be doubled for mixer, divide discriminate only by 5kHz here
		steppedfreq*=5000;

		iffreq+=tx8mfreq-steppedfreq;			// advances our IF frequncy up to 10kHz since downconverter frequency is possibly lower than needed
		lofreq=iffreq;

		steppedfreq*=2;							// mixer needs double the mixing frequency. This is now in 10kHz stepping
		WSS_SetFreq(steppedfreq);
	}
	else
	{
		if (TX8Mengaged != 0)
		{			
			//disconnect downconverter
			expander_clr(RELAYS);
					
			TX8Mengaged=0;
		}
		
		lofreq=_tunefreq;
	}
#else				// if not, then make lofreq always the same as
	lofreq=_tunefreq;
#endif

	clockfreq=adcfreq;

	full=1;
	full<<=32;		//2^32

	freq=0-lofreq;
	freqval=(full*freq/clockfreq);

	if (channel == CH_A)
	{
		freqstore=reg.freq_a;
		lastfreq_A=_tunefreq;
	}
	else
	{
		freqstore=reg.freq_b;
		lastfreq_B=_tunefreq;
	}

	freqstore[0]=freqval&0xFF;
	freqstore[1]=(freqval>>8)&0xFF;
	freqstore[2]=(freqval>>16)&0xFF;
	freqstore[3]=(freqval>>24)&0xFF;

	if (write)
	{
		if (channel == CH_A)
		{
			for (i=0; i<4; i++)
				WriteRegister(FREQ_A+i, reg.freq_a[i]);
		}
		else
		{
			for (i=0; i<4; i++)
				WriteRegister(FREQ_B+i, reg.freq_b[i]);
		}

		if ((diversity) && (channel == CH_A))		// sync A and B frequencys for diversity math
		{
			for (i=0; i<4; i++)
				WriteRegister(FREQ_B+i, reg.freq_a[i]);

			lastfreq_B=lastfreq_A;
		}
	}
}

/*
//A faster version for using with panadapter scanner only!

void SetFreq_Fast(int16_t channel, int32_t _freq, int16_t write, uint32_t adcfreq)
{
int64_t full;
int64_t clockfreq;
int64_t freq;
int32_t freqval;
char* freqstore;
int16_t i;

	clockfreq=adcfreq;

	full=1;
	full<<=32;		//2^32

	freq=0-_freq;
	freqval=(full*freq/clockfreq);

	if (channel == CH_A)
	{
		freqstore=reg.freq_a;
		lastfreq_A=_freq;
	}
	else
	{
		freqstore=reg.freq_b;
		lastfreq_B=_freq;
	}

	freqstore[0]=freqval&0xFF;
	freqstore[1]=(freqval>>8)&0xFF;
	freqstore[2]=(freqval>>16)&0xFF;
	freqstore[3]=(freqval>>24)&0xFF;

	//if (write)
	//{
		if (channel == CH_A)
		{
			for (i=0; i<4; i++)
				WriteRegister_Fast(FREQ_A+i, reg.freq_a[i]);
		}
		else
		{
			for (i=0; i<4; i++)
				WriteRegister_Fast(FREQ_B+i, reg.freq_b[i]);
		}

		//AssertSI(); do not use, as DMA buffer sync gets broken this way!
	//}
}
*/

void SetPhase(int16_t channel, uint16_t phaseword)
{
char* phasestore;
int16_t i;

	if (channel == CH_A)
	{
		phasestore=reg.phase_a;
	}
	else
	{
		phasestore=reg.phase_b;
	}

	phasestore[0]=phaseword&0xFF;
	phasestore[1]=(phaseword>>8)&0xFF;

	if (channel == CH_A)
	{
		for (i=0; i<2; i++)
			WriteRegister(PHASE_A+i, reg.phase_a[i]);
	}
	else
	{
		for (i=0; i<2; i++)
			WriteRegister(PHASE_B+i, reg.phase_b[i]);
	}


	// SI must be touched. However, we cant just do AssertSI(), as this breaks the sample sync with DMA buffer.
	// Therefore we are using same process as for the SampleMode() routine.
	InitIQDataEngine(0,0,1);		//reinit with existing parameters - SI will be assrted in the process!
}

uint8_t SetGain(int16_t channel, int16_t _gain)
{
uint8_t expfixed=0;

	if (_gain < 8)
	{
		if (channel == CH_A)
			WriteRegister(3, _gain);
		else
			WriteRegister(4, _gain);

		if (ReadRegister(20)&1)
		{
			expfixed=1;	//Message(1, "ATTN! Exponent is now cleared for both channels!\r\n");
			WriteRegister(20, ReadRegister(20)&0xFE);
		}
	}
	else if (_gain <=15)
	{
		if (channel == CH_A)
			WriteRegister(3, _gain-8);
		else
			WriteRegister(4, _gain-8);

		if ((ReadRegister(20)&1) == 0)
		{
			expfixed=2;	//Message(1, "ATTN! Exponent is now forced to 111 for both channels!\r\n");
			WriteRegister(20, ReadRegister(20)|1);		// force exponent
		}
	}

	if (channel == CH_A)
		GainCH_A=_gain;
	else if (channel == CH_B)
		GainCH_B=_gain;

	return expfixed;
}

int16_t GetGain(int16_t channel)
{
	if (channel == CH_A)
		return GainCH_A;
	else if (channel == CH_B)
		return GainCH_B;
	else
		return 0;
}

void Int2Phase(char* phasestore, int16_t phase)
{
	phasestore[0]=phase&0xFF;
	phasestore[1]=(phase>>8)&0xFF;
}


uint64_t pow2(int16_t power)
{
uint64_t retval;

	retval=1;
	retval<<=power;
	return retval;
}

uint64_t powX(uint64_t numvalue, int16_t power)
{
uint64_t result;
int16_t i;

	result=numvalue;
	for (i=1; i<power; i++)
		result*=numvalue;

	return(result);
}

// copy 16-bit array of integers as char, swapping high and low bytes
void memmove_BE(uint8_t* dest, uint8_t* src, uint16_t count)
{
uint16_t i;

	for (i=0; i<count; i+=2)
	{
		dest[i]=src[i+1];
		dest[i+1]=src[i];
	}
}

void Init_LM97593(uint32_t sdr_sample_freq, uint16_t bits_per_word, uint16_t channelmode, bool freqandphase, uint32_t adcfreq)
{
uint32_t decvalue=0L;
uint32_t divider;
int16_t j;
uint8_t i;
int16_t F2decFactor=2;		// Can be only 2 or 4. NB! === Leave it to 2 ===. If changed to 4, F_ADC calculating algorithm at
							// SampleMode() has to be reworked to find a F_ADC rate what would divide with samplerate*8, or otherwise phase artifacts will appear
							// because of sample rate mismatch
uint64_t cicgain;

	// LM97593 default register values.

	// DEC is quite simply a ((Fck/(sample rate))/F1dec.fctr/F2dec.fctr)+1, spread across two registers

	decvalue=adcfreq;						// clock frequency

	// multiply divider first, as otherwise we loose digits after comma every time we divide each element separately
	divider=sdr_sample_freq*			// desired output frequency
			F2decFactor*				// we are using factor 2 normally
			2;							// F1 decimalization factor is always 2

	decvalue/=divider;

	decvalue--;								// Actual decimalization N=DEC+1, so subtract one

	/***********/

	reg.dec=decvalue&0xFF;					// CIC decimation control LSB. N=DEC+1, valid range is from 7 to 2047. Unsigned Integer. (332=0x14C)
	reg.dec_by_4=(decvalue>>8)&0x7;			// Bits 0..2 are MSB of DEC value. Bit 4 Controls the decimation factor in F2. 0=decimate by 2, 1=decimate by 4

	if (F2decFactor == 4)
		reg.dec_by_4 |= 0x10;					// set decimate-by-4 bit

	// scale factor is calculated to compensate the CIC filter gain. The shift-up circuit divides the signal by
	// 2^(SCALE-44), wheras CIC filter has gain of N^4. The SCALE value is shosen so, that 2^(SCALE-44) * N^4 <= 1

	// easier to do iteratively :)

	decvalue+=1;//2;								// decvalue is the same as N now
	cicgain=powX(decvalue, 4);

	/*
	for (i=0; i<40; i++)
	{
		if ((pow2(i-44) * cicgain) >=1)
			break;
	}

	if ((pow2(i-44) * cicgain) > 1)		// do not allow gain above 1
		i--;
	*/
	// conversion of above to use without floating point.
	// 2^-a * B = B / 2^a
	for (j=44,i=0; j>=4; j--,i++)
	{
		if ((cicgain/pow2(j)) >= 1)
			break;
	}

	if ((cicgain/pow2(j)) > 1)
		i--;

	reg.scale=i;			// CIC scale parameter. Unsigned integer, representing the number of left bit shifts to perform on the data prior to the CIC filter. Valid range from 0 to 40

	reg.gain_a=1;			// Value of left bit shift prior to F1 channel A
	reg.gain_b=1;			// Value of left bit shift prior to F1 channel B

	if (adcfreq > MAX_SAFE_DIV2_FADC)
		reg.rate=3;			// Determines the serial output clock. The output rate is FCK/(RATE+1). Unsigned integer values 0,1,3,7,15 and 31 are allowed.
							// NB! The parallel data will not work either, if this is too fast! (Seems that any other value besides 0 will do)
							// For MK1.5 serial, use 3 for 96kHz rate and 1 for 192+ rate. 0 is untested.
	else
		reg.rate=1;

	reg.serial_ctrl_dummy=0;		// this is used as a placeholder in the register table

	if (bits_per_word == _16BIT)
		reg.serial_ctrl=0x71;
	else	//24-bit
		reg.serial_ctrl=0xB1;	// xxxxxxxx		// 0x71=Serial output, round parallel and serial to 16-bit, 0xB1=Serial, round to 24-bit
								// ||||||||
								// |||||||+- (1)Enable serial signals								SOUT_EN Enables the serial output pins AOUT, BOUT, SCK and SFS 0=tristate, 1=enabled
								// ||||||+-- (0)data changes on SCK rising edge						SCK_POL Determines polarity of the SCK output. 0=AOUT,BOUT,SFS change on the rising edge of SCK, 1=on the falling edge
								// |||||+--- (0)SFS is active high									SFS_POL Determines the polarity of SFS output 0=active high, 1=active low
								// ||||+---- (0)RDY is active high (not used)						RDY_POL Determines the polarity of RDY output 0=active high, 1=active low
								// |||+----- (1)AOUT outputs both channels							MUX_MODE Determines the mode of the serial output pins. 0=each channel is output on its respective pin, 1=both channels multiplexed on AOUT
								// ||+------ (1)SFS is only once per I/Q pair for both channels		PACKED Controls when SFS goes active. 0=SFS pulses prior to the start of the I and Q words. 1=SFS pulses only prior the start of each I/Q pair. (I word first)
								// ||
								// ++------- FORMAT 	0=Truncate serial output to 8, parallel to 32 bits
								//						1=round serial and parallel to 16 bits, all others set to 0
								//						2=round serial and parallel to 24 bits, all others set to 0
								//						3=output floating point. 8-bit mantissa, 4-bit exponent. All other bits are set to 0

	if (channelmode == SINGLE_CHANNEL)
		reg.serial_ctrl&=~0x10;	// set to single-channel mode

if (freqandphase)
{
	SetFreq(CH_A, 4625000, 0, adcfreq); 	// Frequency word for channel A. Format is a 32-bit 2's complement number spread across 4 registers. The LSB's are in the lower registers.
									// NCO frequency F is F/Fck=FREQ_A/2^32, valid range is from

	Int2Phase(reg.phase_a, 0);		// Phase word for channel A. Format is a 16-bit unsigned magnitude number spread across 2 registers. The LSB's are in the lower registers.
										// The NCO phase PHI is PHI=2*pi*PHASE_A/2^16

	SetFreq(CH_B, 5505000, 0, adcfreq);		// Frequency word for channel B. Format is a 32-bit 2's complement number spread across 4 registers. The LSB's are in the lower registers.
										// NCO frequency F is F/Fck=FREQ_B/2^32

	Int2Phase(reg.phase_b, 0);		// Phase word for channel B. Format is a 16-bit unsigned magnitude number spread across 2 registers. The LSB's are in the lower registers.
										// The NCO phase PHI is PHI=2*pi*PHASE_B/2^16
}
	reg.source=4; //0x82;	// 0000xxxx
							//     ||||
							//     ||++- A_SOURCE 0=AIN as channel A input source 1=BIN 2=3=select TEST_REG as channel input source
							//     ++--- B_SOURCE 0=AIN as channel B input source 1=BIN 2=3=select TEST_REG as channel input source

	//reg.agc_ctrl=1;			// 00xxx01x
	reg.agc_ctrl=0x0|AGC_SHIFT;		// 00xxx01x
							//   ||||||
							//   |||||+- EXP_INH 0=allow exponent to pass into FLOAT TO FIXED converter 1=Force exponent in DDC channel to a 7 (max digital gain)
							//   |||++-- Reserved, do not use
							//   ||+---- AGC_HOLD_IC 0=normal closed loop operation 1=Hold integrator at initial condition.
							//   ++----- Bit shift value for AGC loop. Valid range is from 0 to 3

							// Note: Set AGC_HOLD_IC to 1 and AGC_IC_A/B to some value to fix DVGA gain

	reg.agc_ic_a=0x20;		// AGC fixed gain for channel A. 8-bit unsigned magnitude number. The channel A DVGA gain will be set to the inverted three MSB-s
	reg.agc_ic_b=0x20;		// AGC fixed gain for channel B. 8-bit unsigned magnitude number. The channel B DVGA gain will be set to the inverted three MSB-s

	reg.agc_rb_a=0;			// AGC integrator feedback readback value for channel A. 8-bit unsigned magnitude number. The user can read the magnitude MSB's in this register.
	reg.agc_rb_b=0;			// AGC integrator feedback readback value for channel A. 8-bit unsigned magnitude number. The user can read the magnitude MSB's in this register.

	memmove(reg.test_reg, 	"\x12\x34", // Test input source. Instead of AIN and BIN pins, the value from this register is used when in test mode.
							2);

	//reg.debug=0x1|(8<<1);	//debug, NCO A cosine output,
	reg.debug=0xC0;			// xxxxxxxx		// enable phase dithering
							// ||||||||
							// |||||||+- 0=normal, 1=Enables access to the internal probe points
							// ||+++++-- Selects internal node tap for debugging (see datasheet pg.38 for details)
							// |+------- 0=disable NCO dither source for channel A 1=enable
							// +-------- 0=disable NCO dither source for channel B 1=enable

	memmove(reg.agc_table, 	_AGC_TABLE, 32);	// RAM space that defines key AGC loop parameters. 32 separate 8-bit 2's complement numbers. Common to both channels.


	reg.f1_ctrl_A=0;		// 00000xxx
							//      |||
							//      ||+- COEF_SEL_F1A Channel A F1 coefficient select register 0=memory A, 1=memory B
							//      |+-- COEF_SEL_F1B Channel B F1 coefficient select register 0=memory A, 1=memory B
							//      +--- F1 coefficient page select register. 0=memory A, 1=memory B

	reg.f2_ctrl_A=0;		// 00000xxx
							//      |||
							//      ||+- COEF_SEL_F2A Channel A F2 coefficient select register 0=memory A, 1=memory B
							//      |+-- COEF_SEL_F2B Channel B F2 coefficient select register 0=memory A, 1=memory B
							//      +--- F2 coefficient page select register. 0=memory A, 1=memory B


	memmove_BE((uint8_t*)reg.f1_coeff_A, 	(uint8_t*)_F1_TABLE_STD, 22);	// Coefficients for F1. 11 separate 16-bit 2's complement numbers, each spread around 2 registers, LSB's in lower register.
													// PAGE_SEL_F1=1 maps these addresses to coefficient memory B

	memmove_BE((uint8_t*)reg.f2_coeff_A, 	(uint8_t*)_F2_TABLE_STD, 64);	// Coefficients for F2. 32 separate 16-bit 2's complement numbers, each spread across 2 registers, LSB's in lower register.
													// PAGE_SEL_F2=1 maps these addresses to coefficient memory B

	// NOTE! Filter coefficients are doubled here in order to get both pages loaded during the initialization
	// Note, that both channels use coefficient bank A as a default.

	reg.f1_ctrl_B=4;		// 00000xxx
							//      |||
							//      ||+- COEF_SEL_F1A Channel A F1 coefficient select register 0=memory A, 1=memory B
							//      |+-- COEF_SEL_F1B Channel B F1 coefficient select register 0=memory A, 1=memory B
							//      +--- F1 coefficient page select register. 0=memory A, 1=memory B

	reg.f2_ctrl_B=4;		// 00000xxx
							//      |||
							//      ||+- COEF_SEL_F2A Channel A F2 coefficient select register 0=memory A, 1=memory B
							//      |+-- COEF_SEL_F2B Channel B F2 coefficient select register 0=memory A, 1=memory B
							//      +--- F2 coefficient page select register. 0=memory A, 1=memory B

	memmove_BE((uint8_t*)reg.f1_coeff_B, 	(uint8_t*)_F1_TABLE_SDR, 22);	// Coefficients for F1. 11 separate 16-bit 2's complement numbers, each spread around 2 registers, LSB's in lower register.
													// PAGE_SEL_F1=1 maps these addresses to coefficient memory B

	memmove_BE((uint8_t*)reg.f2_coeff_B, 	(uint8_t*)_F2_TABLE_SDR, 64);	// Coefficients for F2. 32 separate 16-bit 2's complement numbers, each spread across 2 registers, LSB's in lower register.
													// PAGE_SEL_F2=1 maps these addresses to coefficient memory B

	// Final values what define what filters to use.

	reg.f1_ctrl=0;//2;		// 00000xxx
							//      |||
							//      ||+- COEF_SEL_F1A Channel A F1 coefficient select register 0=memory A, 1=memory B
							//      |+-- COEF_SEL_F1B Channel B F1 coefficient select register 0=memory A, 1=memory B
							//      +--- F1 coefficient page select register. 0=memory A, 1=memory B

	reg.f2_ctrl=0;//2;		// 00000xxx
							//      |||
							//      ||+- COEF_SEL_F2A Channel A F2 coefficient select register 0=memory A, 1=memory B
							//      |+-- COEF_SEL_F2B Channel B F2 coefficient select register 0=memory A, 1=memory B
							//      +--- F2 coefficient page select register. 0=memory A, 1=memory B

	reg.serial_ctrl2=1;		// 000000xx
							//       ||
							//       |+- SFS_MODE 0=SFS asserted at the start of each output word when PACKED=1 or each I/Q pair when PACKED=0; 1=SFS asserted at te start of each output sample period
							//       +-- SDC_EN 0=normal serial mode; 1=serial daisy-chain master mode

	reg.agc_ctrl2=0x22;		// 0xxxxxxx		//0x20 was used until V1.95, what did not work properly!
							//  |||||||
							//  |||||++- Enable reduced bandwidth AGC power detector. 0=2'nd order decimate-by-8 CIC; 1=1-tap comb added to CIC; 2=4-tap comb added to CIC
							//  +++++--- Number of CK period delays needed to align DVGA gain step with the digital gain compensation step. Set this register to 7 if xASTROBE and xBSTROBE are not used. Otherwise set to 8

}


int16_t UpdateRegisters(uint16_t touchsi)
{

int16_t i, j, k;
char* regpool;

	if (!powered)
		return -1;		// no access to chip allowed if radio part is not powered

	regpool=&reg.dec;	// get the start of register data

	for (i=0, k=0; i<REGENTRIES; i++)
	{
		for (j=0; j<regstruct[i].datalen; j++, k++)
		{
			if (regstruct[i].flags & R_INIT)
			{
				if (regstruct[i].flags & R_ASSERT_SI)
					WriteRegister(regstruct[i].regaddr+j, regpool[k]);
				else
					WriteRegister(regstruct[i].regaddr+j, regpool[k]);
			}
		}
	}

	if (touchsi)
		AssertSI();

	return(0);
}

void StartRadio(void)
{
/*
	PORTE|=0x80;	// rise SDRENABLE
	delayms(500);	// allow some time for power to stabilize

	// write appropriate ports as outputs towards radio chip

	DDRA=0xFF;		// address lines

	PORTF&=0x80;		// clear everything except pwrdet
	PORTF|=0x4E;		// rise xSI, xWR, xRD, xPOUT_EN(gpio3); clear xCE and xMR(gpio2)
	DDRF|=0x7F;			// set all optputs except pwrdet

	delayms(500);	// keep reset
	PORTF|=0x20;	// release reset

	PORTE&=~(0x7);	// clear PSEL[0..2]
	//PORTE|=0x10;	// set pullup for RDY

	DDRE|=0x7;		// set PSEL signals as outputs

	PORTE|=LSB;		// pre-set LSB bit. NB - this is 0 by default and shall stay this way for rounded 16-bit output.

	PORTF&=~(0x40);	// assert xPOUT_EN by default to conserve timer interrupt time

	// Enable INT4 for RDY signal

	powered=true;	// show everybody that chip communication is now ok
*/
}

void StopRadio(void)
{
/*
	// kill INT4
	//cli();
	//EICRB &= ~(1 << ISC41);
	//EIMSK &= ~(1<<INT4);
	//sei();

	powered=false;		// show globally that radio is no more powered and no IO shall be performed towards the chip
	radio_ready=false;	// show audio interrupt that the chip is not valid any more (vector will start outputting silence)

	// make outputs to inputs, so processor does not send anything towards the radio chip (otherwise the signals will start leaking through clamping diodes
	// when the chip is not powered and it is not nice)

	DDRA=0;
	DDRC=0;
	DDRF&=~(0x6F);
	DDRE&=~(0x7);

	PORTE&=~(0x80);	// clear SDRENABLE
*/
}

void ResetRadio(void)
{
/*
	if (powered)
	{
		PORTF&=~(0x20);	// assert reset
		delayms(100);	// keep reset
		PORTF|=0x20;	// release reset
		delayms(200);	// wait to complete
	}
*/
}


/*
This is the legacy piece of code what was used at the very beginning when radio chip did not show any sign of life and
only way out was trashing the registers randomly hoping to get lucky. What we did! So the code is today totally obsolete,
but left here as an important landmark in development! :)
*/

//Desperate measure for trying to figure out what is wrong with the chip - program all the registers having R_MONTECARLO flag set with ranodm values!

static   int32_t   randx;
static   char   randf;

void xsrand(unsigned x)
{
   randx = x;
   randf = 1;
}

int16_t xrand(void)
{
   if(!randf)
      xsrand(1);
   return((int)((randx = randx*1103515245L + 12345)>>16) & 077777);
}

void MonteCarlo(void)
{
int16_t i,j;

	xsrand(millis());

	for (i=0; i<REGENTRIES; i++)
	{
		if (regstruct[i].flags & R_MONTECARLO)
		{
			for (j=0; j<regstruct[i].datalen; j++)
			{
				WriteRegister(regstruct[i].regaddr+j, xrand() / 257);
			}
		}
	}
}

