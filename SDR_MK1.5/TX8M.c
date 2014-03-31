#include "gpio.h"
#include "twi.h"
#include "spi.h"
#include "cycle_counter\cycle_counter.h"

#include "gpio.h"
#include "spi.h"
#include "eth_spi.h"
#include "tx8m.h"



#if (TX8M == 1)

/***************************************************************************************
Signal Synthesizer
***************************************************************************************/

uint32_t wssr0=0, wssr1=0, wssr2=0, wssr3=0, wssr4=0, wssr5=0;		// register mirrors

//
// Set frequency in Hz (from 35MHz to 4.4GHz)
//
void WSS_SetFreq(uint64_t freq)
{
uint64_t vcofreq;
uint16_t divider;		// can be up to 12-bit in real life
uint64_t wssint, wssfrac;

	//VCO has to be between 2.2 and 4.4GHz. Calculate divider first, so our VCO freq will not drop below 2.2GHz
	for (divider=0; (vcofreq=freq<<divider) < 2200000000; divider++)
	{};

	//calculate INT value
	wssint=vcofreq/10000000;

	//With 10MHz clock signal and running in low-spur mode, we can have reasonable channel spacing 10MHz / 1000 = 10kHz, so MOD=1000
	//calculate FRAC value, given that our MOD is 1000
	wssfrac=vcofreq/10000;  //divide by Fclk*1000 so we do not loose the fractional part
	wssfrac-=wssint*1000;	//subtract INT*1000, so we will have only FRAC. Since our MOD is 1000, there is no need to do anything else to the FRAC here.

	//set new values to mirrors
	wssr4&=~(0x7<<R4_RFDIV);
	wssr4|=divider<<R4_RFDIV;

	wssr1&=~(0xFFF<<R1_MOD);
	wssr1|=1000<<R1_MOD;

	wssr0&=~(0xFFF<<R0_FRAC);
	wssr0&=~(0xFFFF<<R0_INT);
	wssr0|=wssfrac<<R0_FRAC;
	wssr0|=wssint<<R0_INT;

	//Program registers. Note, that R0 is written last because this will cause double buffered values in other registers to be updated
	WSS_Write(wssr4);		//R4
	WSS_Write(wssr1);		//R1
	WSS_Write(wssr0);		//R0
}


void WSS_Write(uint32_t dssdword)
{
uint16_t i;

	// be sure the clock is low initially
	gpio_set_pin_low(WSS_CLK);

	// enable loading
	gpio_set_pin_low(WSS_LDEN);
	cpu_delay_us(1, F_CPU);			// probably not needed, since it has to be only 20ns, but added for safety

	//clock out data bits starting from MSB
	for (i=0; i<32; i++)
	{
		if ((dssdword << i) & 0x80000000)		// highest bit set?
			gpio_set_pin_high(WSS_DIN);
		else
			gpio_set_pin_low(WSS_DIN);

		// transfer bit
		cpu_delay_us(1, F_CPU);				// for increased safety
		gpio_set_pin_high(WSS_CLK);
		cpu_delay_us(1, F_CPU);				// for increased safety
		gpio_set_pin_low(WSS_CLK);
	}

	// latch register
	gpio_set_pin_high(WSS_LDEN);
	cpu_delay_us(1, F_CPU);			// probably not needed
	gpio_set_pin_low(WSS_LDEN);
}

void WSS_Init(void)
{
	// write registers starting from register 5 (datasheet rev0 pg.28)
	// defaults set here will give us 500MHz setup

	// enable ADF4351 and charge pump
	gpio_set_pin_high(WSS_CE);

	// reset register loading mechanism
	gpio_set_pin_high(WSS_LDEN);
	cpu_delay_us(1, F_CPU);			// probably not needed, since it has to be only 20ns, but added for safety

	// set defaults in mirrors for 500MHz, low spur mode, FRAC-N
	wssr5=0x580005;			//R5 LED=lock detect
	//wssr5=0xD80005; 		//R5 LED=high (just for testing)
	wssr4=0xB501FC;			//R4
	wssr3=0x4B3;			//R3
	wssr2=0x60004E42;		//R2
	wssr1=0x8008009;		//R1
	wssr0=0xC80000;			//R0

	WSS_Write(wssr5);		//R5
	WSS_Write(wssr4);		//R4
	WSS_Write(wssr3);		//R3
	WSS_Write(wssr2);		//R2
	WSS_Write(wssr1);		//R1
	WSS_Write(wssr0);		//R0
}

/***************************************************************************************
Programmable Filters
***************************************************************************************/
void FLT_SetFreq(uint32_t _freq, uint8_t filter)
{
uint16_t i;
uint32_t freq;

	freq=_freq/1000000;				// convert to MHz
	freq++;							// 0=1MHz by datasheet
	freq&=0x1F;						// leave only 5 bits for frequency
	freq<<=1;						// we are clocking LSB first, but first bit has to be write condition bit, so create space for one

	freq|=1;						// patch in write condition bit

	gpio_set_pin_low(WSS_LDEN);		// just in case to be sure that we do not mess with WSS by accident
	gpio_set_pin_low(WSS_CLK);		// set clock to initial state

	// enable loading
	if (filter&FLT_LOWPASS)
		expander_clr(LPASSLDEN);
	if (filter&FLT_HIGHPASS)
		expander_clr(HPASSLDEN);

	for (i=0; i<6; i++)
	{
		//set bit
		if ((freq>>i)&1)
			gpio_set_pin_high(WSS_DIN);
		else
			gpio_set_pin_low(WSS_DIN);

		//clock in data now
		cpu_delay_us(1, F_CPU);				// for increased safety
		gpio_set_pin_high(WSS_CLK);
		cpu_delay_us(1, F_CPU);				// for increased safety
		gpio_set_pin_low(WSS_CLK);
	}

	expander_set(LPASSLDEN|HPASSLDEN);		//disable filter loading again
}

/***************************************************************************************
I2C IO Expander
***************************************************************************************/

uint16_t expander=0, expcfg=0xFFFF;

//set pin directions and default levels for outputs
//
void expander_init(void)
{
	// set all IO-s as inputs (should be default as well on startup)
	expander_setasinput(0xFFFF);

	// reset polarity inversion bits
	twi_write(0x20, IO_PINV0, 0);
	twi_write(0x20, IO_PINV1, 0);

	// preset output pin values. Start with SPI memory loading enabled for FPGA, but FPGA in reset.
	expander_clr(0xFFFF);		// clear all
	// set pins
	expander_set(ROMCSO_B);		// clear SPI memory chipselect for time being
	// expander_set(EXP_IO1_7);	// test

	expander_set(LPASSLDEN);	// disable lowpass filter data loading
	expander_set(HPASSLDEN);	// disable highpass filter data loading

	// configure pins what we need as inputs to inputs, rest of them will be outputs
	// since most pins are outputs, it is easier to use reversal
	expander_setasoutput(0xFFFF&(~(FPGADONE|FPGAINIT)));
}

//set expander pins as inputs
void expander_setasinput(uint16_t pinmask)
{
	expcfg|=pinmask;
	twi_write(0x20, IO_CFG0, expcfg&0xFF);
	twi_write(0x20, IO_CFG1, (expcfg>>8)&0xFF);
}

//set expander pins as inputs
void expander_setasoutput(uint16_t pinmask)
{
	expcfg&=~pinmask;
	twi_write(0x20, IO_CFG0, expcfg&0xFF);
	twi_write(0x20, IO_CFG1, (expcfg>>8)&0xFF);
}

//read all inputs
uint16_t expander_read(uint16_t pinmask)
{
uint16_t ioval, stat;

	ioval=twi_read(0x20, IO_IN1, &stat);
	ioval<<=8;
	ioval|=twi_read(0x20, IO_IN0, &stat);

	return(ioval&pinmask);
}

//write pin 1
void expander_set(uint16_t pinmask)
{
	expander|=pinmask;
	twi_write(0x20, IO_OUT0, expander&0xFF);
	twi_write(0x20, IO_OUT1, (expander>>8)&0xFF);
}

//write pin 0
void expander_clr(uint16_t pinmask)
{
	expander&=~pinmask;
	twi_write(0x20, IO_OUT0, expander&0xFF);
	twi_write(0x20, IO_OUT1, (expander>>8)&0xFF);
}

/***************************************************************************************
Xilinx config SPI Memory
***************************************************************************************/

// Read Identification
void spimem_rdid(uint16_t bufflen, uint8_t* buff)
{
uint16_t i;

	if (bufflen<20)
		return;

	select_tx8mspi(ROMCSO_B, 0, 15000000, 0, 8);		//20MHz seems to be maximum for the M25P16 variety we are using

	spi_write8(0x9F);		//RDID instruction

	for (i=0; i<bufflen; i++)
		buff[i]=spi_read8();

	unselect_tx8mspi(ROMCSO_B, 0);
}

//Re-Program SPI memory
void spimem_pgm(uint8_t* buff, uint16_t bufflen)
{
	select_tx8mspi(ROMCSO_B, 0, 15000000, 0, 8);		//20MHz seems to be maximum for the M25P16 variety we are using

	unselect_tx8mspi(ROMCSO_B, 0);
}



/***************************************************************************************
Xilinx functionality
***************************************************************************************/

#endif