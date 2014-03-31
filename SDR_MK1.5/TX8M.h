#define TX8M 0		// set to 0 to exclude entire TX8M support

#if (TX8M == 1)

#define SPI (&AVR32_SPI)

/****************** WSS definitions ***************************/

// GPIO connector pin definitions

#define WSS_CLK		AVR32_PIN_PA04
#define WSS_DIN		AVR32_PIN_PA03
#define WSS_LDEN	AVR32_PIN_TMS
#define WSS_CE		AVR32_PIN_TDO
#define WSS_LOCK	AVR32_PIN_TDI

#define XILINX_SPICS	(AVR32_PIN_PB05|0xF0000000)
///#define RELAYS			AVR32_PIN_PA20	 //UART1_CLK				//!!! multiplexed with DRC DATA! replaced with io expander line
//AVR32_PIN_PB03	UART1_RXD	UART_RXD
//AVR32_PIN_PB02	UART1_TXD	UART_TXD
#define EXC_IQTHROTTLE	AVR32_PIN_PB04	 //UART1_CTS

//WSS register selection (control bits DB2..0)
#define WSS_R0	0
#define WSS_R1	1
#define WSS_R2	2
#define WSS_R3	3
#define WSS_R4	4
#define WSS_R5	5

//Register 0 shift values
#define R0_INT		15		//16-bit integer value
#define R0_FRAC		3		//12-bit fractional value

//Register 1 shift values
#define R1_PHASEADJ		28		//1-bit phase adjust
#define R1_PRESCALER	27		//1-bit prescaler
#define R1_PHASE		15		//12-bit phase
#define R1_MOD			3		//12-bit modulus value

//Register 2 shift values
#define R2_LNLSMOD		29		//2-bit low noise and low spur modes
#define R2_MUXOUT		26		//3-bit
#define R2_REFDOUBLER	25		//1-bit reference doubler
#define R2_RDIV2		24		//1-bit
#define R2_R			14		//10-bit R counter
#define R2_DOUBLEBUFF	13		//1-bit
#define R2_CHGPMPCURRNT	9		//4-bit
#define R2_LDF			8		//1-bit
#define R2_LDP			7		//1-bit
#define R2_PDPOL		6		//1-bit
#define R2_PWRDWN		5		//1-bit
#define R2_CPTHRSTATE	4		//1-bit
#define R2_COUNTERRST	3		//1-bit

//Register 3 shift values
#define R3_BANDSELCLK	23		//1-bit
#define R3_ABP			22		//1-bit
#define R3_CHARGECANCEL	21		//1-bit
#define R3_CSR			18		//1-bit
#define R3_CLKDIVMODE	15		//2-bit
#define R3_CLKDIV		3		//12-bit clock divider value

//Register 4 shift values
#define R4_FEEDBACKSEL	23		//1-bit
#define R4_RFDIV		20		//3-bit
#define R4_BNDSELCLKDIV	12		//8-bit
#define R4_VCOPWRDWN	11		//1-bit
#define R4_MTLD			10		//1-bit
#define R4_AUXOUTSEL	9		//1-bit
#define R4_AUXOUTEN		8		//1-bit
#define R4_AUXOUTPWR	6		//2-bit
#define R4_RFOUTEN		5		//1-bit
#define R4_RFOUTPWR		3		//2-bit

//Register 5 shift values
#define R5_LDPINMODE	22		//2-bit

//WSS Public Functions
void WSS_Write(uint32_t dssdword);
void WSS_SetFreq(uint64_t freq);
void WSS_Init(void);

/****************** Programmable Filters definitions ***************************/

#define FLT_LOWPASS		1
#define FLT_HIGHPASS	2

void FLT_SetFreq(uint32_t freq, uint8_t filter);


/****************** I2C Expander definitions ***************************/

// pin definitions
#define FPGAM2			(1<<0)	//IO0_0
#define FPGAM1			(1<<1)	//IO0_1
#define FPGAPROGB		(1<<2)	//IO0_2
#define RFOUTSEL		(1<<3)	//IO0_3
#define DETINSEL		(1<<4)	//IO0_4
#define DETREFPHASE		(1<<5)	//IO0_5
#define DETREFSEL		(1<<6)	//IO0_6
#define LOSEL			(1<<7)	//IO0_7

#define RELAYS			(1<<15)	//IO1_7
#define PTT				(1<<14)	//IO1_6
#define FPGAINIT		(1<<13)	//IO1_5
#define ROMCSO_B		(1<<12)	//IO1_4
#define FPGADONE		(1<<11)	//IO1_3
#define HPASSLDEN		(1<<10)	//IO1_2
#define LPASSLDEN		(1<<9)	//IO1_1
#define EXP_IO1_0		(1<<8)	//IO1_0

// command byte definitions
#define IO_IN0		0
#define IO_IN1		1
#define IO_OUT0		2
#define IO_OUT1		3
#define IO_PINV0	4
#define IO_PINV1	5
#define IO_CFG0		6
#define IO_CFG1		7

//Expander Public Functions
void expander_init(void);
void expander_mode(uint16_t pin, uint16_t mode);
uint16_t expander_read(uint16_t pinmask);
void expander_set(uint16_t pinmask);
void expander_clr(uint16_t pinmask);
void expander_setasinput(uint16_t pinmask);
void expander_setasoutput(uint16_t pinmask);

/***************************************************************************************
TX8M SPI generic functionality
***************************************************************************************/

extern spi_options_t spiOptions;
static uint32_t spioldbaudrate=20000000;
static uint8_t spioldmode=0;
static uint8_t spioldbits=8;

static inline void select_tx8mspi(uint32_t xcssignal, uint16_t csdelay, uint32_t baudrate, uint8_t spimode, uint8_t bits) __attribute__ ((always_inline));
static inline void select_tx8mspi(uint32_t xcssignal, uint16_t csdelay, uint32_t baudrate, uint8_t spimode, uint8_t bits)
{
	// Other SPI chipselets may have left the last transfer active (if the CSAAT bit is set),
	// so do not do anything before current transfer is complete
	while(!(AVR32_SPI.sr & AVR32_SPI_SR_TDRE_MASK)) {};			// Wait until any non-DMA operation is complete. Should not block .. theoretically ...

	// Just in case unselect everything
	AVR32_SPI.mr |= AVR32_SPI_MR_PCS_MASK;						// Assert all lines; no peripheral is selected.
	AVR32_SPI.cr = AVR32_SPI_CR_LASTXFER_MASK;					// Last transfer, so deassert the current NPCS if CSAAT is set.

	//disconnect NPCS0 from IO pins and set pin high (deactivate chipselect)
	gpio_set_gpio_pin(ETH_SPI_NPCS_PIN);

	//store existing spi parameters
	spioldbaudrate=spiOptions.baudrate;		// store original baudrate
	spioldmode=spiOptions.spi_mode;
	spioldbits=spiOptions.bits;

	//set appropriate values for tx8m
	spiOptions.baudrate=baudrate;
	spiOptions.spi_mode=spimode;
	spiOptions.bits=bits;						// we are transfering data in 24-bit mode, since this is the sample width for ADC. (Loosing 1/3 bandwidth for Exciter because of that, but that is not a problem, at least for a moment)
	spi_setupChipReg(SPI, &spiOptions, F_PBA_SPEED);

	//assert SPI memory chipselect at IO expander
	if (!(xcssignal&0xF0000000))
		expander_clr(xcssignal);
	else if (xcssignal!=0xFFFFFFFF)
		gpio_set_pin_low(xcssignal&0xFF);

	if (csdelay)
		cpu_delay_us(csdelay, F_CPU);

	//now select NPCS0 to make the SPI active (although the NPCS0 pin is disconnected)
	AVR32_SPI.mr &= ~(1 << (AVR32_SPI_MR_PCS_OFFSET + 0));		// always select NPCS0
}

static inline void unselect_tx8mspi(uint32_t xcssignal, uint16_t csdelay) __attribute__ ((always_inline));
static inline void unselect_tx8mspi(uint32_t xcssignal, uint16_t csdelay)
{
	//clear SPI memory chipselect at IO expander
	if (!(xcssignal&0xF0000000))
		expander_set(xcssignal);
	else if (xcssignal!=0xFFFFFFFF)
		gpio_set_pin_high(xcssignal&0xFF);

	if (csdelay)
		cpu_delay_us(csdelay, F_CPU);

	// Assert all lines; no peripheral is selected.
	AVR32_SPI.mr |= AVR32_SPI_MR_PCS_MASK;
	// Last transfer, so deassert the current NPCS if CSAAT is set.
	AVR32_SPI.cr = AVR32_SPI_CR_LASTXFER_MASK;

	//restore spi parameters to previous setting
	spiOptions.baudrate=spioldbaudrate;			//SPI_ETH_BAUDRATE;
	spiOptions.spi_mode=spioldmode;
	spiOptions.bits=spioldbits;
	spi_setupChipReg(SPI, &spiOptions, F_PBA_SPEED);

	//reconnect NPCS0 to the SPI0 engine for ethernet
	gpio_enable_module_pin(ETH_SPI_NPCS_PIN, ETH_SPI_NPCS_FUNCTION);
}

static inline void spi_write8(uint8_t b) __attribute__ ((always_inline));
static inline void spi_write8(uint8_t b)
{
	while (!(AVR32_SPI.sr & AVR32_SPI_SR_TDRE_MASK)) {};		// should not block .. theoretically ...
	AVR32_SPI.tdr = b << AVR32_SPI_TDR_TD_OFFSET;

	//block until transmission ends
	while ((AVR32_SPI.sr & (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) != (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) {};
}

static inline uint8_t spi_read8(void) __attribute__ ((always_inline));
static inline uint8_t spi_read8(void)
{
	// write dummy byte to clock in data
	while (!(AVR32_SPI.sr & AVR32_SPI_SR_TDRE_MASK)) {};		// should not block .. theoretically ...
	AVR32_SPI.tdr = 0xFF << AVR32_SPI_TDR_TD_OFFSET;

	// return result. If the SPI block is not operational somehow, this function is blocking and the whole system dies here!
	while ((AVR32_SPI.sr & (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) != (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) {};	// wait for character to appear
	return (AVR32_SPI.rdr >> AVR32_SPI_RDR_RD_OFFSET);
}

static inline void spi_write16(uint16_t w) __attribute__ ((always_inline));
static inline void spi_write16(uint16_t w)
{
	while (!(AVR32_SPI.sr & AVR32_SPI_SR_TDRE_MASK)) {};		// should not block .. theoretically ...
	AVR32_SPI.tdr = w << AVR32_SPI_TDR_TD_OFFSET;

	//block until transmission ends
	while ((AVR32_SPI.sr & (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) != (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) {};
}

static inline uint16_t spi_read16(void) __attribute__ ((always_inline));
static inline uint16_t spi_read16(void)
{
	// write dummy byte to clock in data
	while (!(AVR32_SPI.sr & AVR32_SPI_SR_TDRE_MASK)) {};		// should not block .. theoretically ...
	AVR32_SPI.tdr = 0xFFFF << AVR32_SPI_TDR_TD_OFFSET;

	// return result. If the SPI block is not operational somehow, this function is blocking and the whole system dies here!
	while ((AVR32_SPI.sr & (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) != (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) {};	// wait for character to appear
	return (AVR32_SPI.rdr >> AVR32_SPI_RDR_RD_OFFSET);
}

static inline uint16_t spi_readwrite16(uint16_t w) __attribute__ ((always_inline));
static inline uint16_t spi_readwrite16(uint16_t w)
{
	// write dummy byte to clock in data
	while (!(AVR32_SPI.sr & AVR32_SPI_SR_TDRE_MASK)) {};		// should not block .. theoretically ...
	AVR32_SPI.tdr = w << AVR32_SPI_TDR_TD_OFFSET;

	// return result. If the SPI block is not operational somehow, this function is blocking and the whole system dies here!
	while ((AVR32_SPI.sr & (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) != (AVR32_SPI_SR_RDRF_MASK | AVR32_SPI_SR_TXEMPTY_MASK)) {};	// wait for character to appear
	return (AVR32_SPI.rdr >> AVR32_SPI_RDR_RD_OFFSET);
}

/***************************************************************************************
Xilinx Host configuration
***************************************************************************************/
//note, that SPI CLK and MISO lines have to be configured as GPIO first!
static inline void xilinx_writecfgbyte(uint8_t xbyte)  __attribute__ ((always_inline));
static inline void xilinx_writecfgbyte(uint8_t xbyte)
{
int16_t ix;

	//clock in bits MSB first
	for (ix=7; ix>=0; ix--)
	{
		gpio_set_pin_low(ETH_SPI_SCK_PIN);

		if ((xbyte>>ix)&1)
			gpio_set_pin_high(ETH_SPI_MISO_PIN);
		else
			gpio_set_pin_low(ETH_SPI_MISO_PIN);
		gpio_set_pin_high(ETH_SPI_SCK_PIN);
		cpu_delay_us(1, F_CPU);
	}

	gpio_set_pin_low(ETH_SPI_SCK_PIN);
}

/****************** SPI Memory definitions ***************************/

void spimem_rdid(uint16_t bufflen, uint8_t* buff);

/****************** FPGA programming binary definitions **************/
//#ifdef TX8MXILINX
#if (1)

extern const char fpga_bitstream_start[];
extern const char fpga_bitstream_end[];
extern const char fpga_bitstream_size_sym[];

#define fpga_bitstream_size ((uint32_t)fpga_bitstream_size_sym)

/*
Xilinx .bit file format:

http://www.fpga-faq.com/FAQ_Pages/0026_Tell_me_about_bit_files.htm

00000000:  00 09 0f f0 0f f0 0f f0 0f f0 00 00 01 61 00 0a  .............a..
00000010:  78 66 6f 72 6d 2e 6e 63 64 00 62 00 0c 76 31 30  xform.ncd.b..v10
00000020:  30 30 65 66 67 38 36 30 00 63 00 0b 32 30 30 31  00efg860.c..2001
00000030:  2f 30 38 2f 31 30 00 64 00 09 30 36 3a 35 35 3a  /08/10.d..06:55:
00000040:  30 34 00 65 00 0c 28 18 ff ff ff ff aa 99 55 66  04.e..(.......Uf

Field 1
2 bytes          length 0x0009           (big endian)
9 bytes          some sort of header

Field 2
2 bytes          length 0x0001
1 byte           key 0x61                (The letter "a")

Field 3
2 bytes          length 0x000a           (value depends on file name length)
10 bytes         string design name "xform.ncd" (including a trailing 0x00)

Field 4
1 byte           key 0x62                (The letter "b")
2 bytes          length 0x000c           (value depends on part name length)
12 bytes         string part name "v1000efg860" (including a trailing 0x00)

Field 4
1 byte           key 0x63                (The letter "c")
2 bytes          length 0x000b
11 bytes         string date "2001/08/10"  (including a trailing 0x00)

Field 5
1 byte           key 0x64                (The letter "d")
2 bytes          length 0x0009
9 bytes          string time "06:55:04"    (including a trailing 0x00)

Field 6
1 byte           key 0x65                 (The letter "e")
4 bytes          length 0x000c9090        (value depends on device type,
                                           and maybe design details)
8233440 bytes    raw bit stream starting with 0xffffffff aa995566 sync
                 word documented below.

(For Spartan-3A and 3AN families sync word is 16-bit and is 0xAA 0x99 (0xAA99 big endian) [Laid]

/---/

Note the "Enable .bit File Compression" option doesn't change the file
format at all.  It only changes how the bitstream is interpreted by the
configuration state machine inside the Xilinx part. (and of course the length of the file)
*/
#endif //TX8MXILINX

#endif //TX8M==1

