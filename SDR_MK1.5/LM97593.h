


#ifndef LM97593_H_
#define LM97593_H_

#include "compiler.h"

// Register names for LM97593

#define	DEC				0
#define	DEC_BY_4		1
#define SCALE			2
#define	GAIN_A			3
#define GAIN_B			4
#define	RATE			5
#define	SERIAL_CTRL		6

#define	FREQ_A			7	//7..10

#define	PHASE_A			11	//11..12

#define	FREQ_B			13	//13..16

#define	PHASE_B			17	//17..18

#define	SOURCE			19
#define AGC_CTRL		20
//#define AGC_COUNT		21	reserved
//#define RESERVED		22	reserved

#define AGC_IC_A		23
#define AGC_IC_B		24
#define AGC_RB_A		25
#define AGC_RB_B		26

#define TEST_REG		27	//27..28

#define	DEBUG			31

#define AGC_TABLE		128	//128..159

#define	F1_COEFF		160	//160..181
#define F2_COEFF		182	//182..245
#define F1_CTRL			246
#define F2_CTRL			247
#define	SERIAL_CTRL2	248
#define	AGC_CTRL2		249

#define	R_DISPLAY		1		// display register in regdump
#define	R_WRITE			2		// not used
#define R_INIT			4		// update register during UpdateDefaults()
#define	R_MONTECARLO	8		// include register in montecarlo analysis
#define R_ASSERT_SI		16		// assert xSI signal after programming this register

#define TUNINGSTEP	2500L		// incremet of frequency tuning when done manually from numeric keypad

#define PDCA_CHANNEL_0	0

#define	xCE			AVR32_PIN_PB09
#define xWR			AVR32_PIN_PB10
#define xRD			AVR32_PIN_PB11

#define HWBE		AVR32_PIN_PA13
#define BTN1		AVR32_PIN_PA27

#define BUSPOWER	AVR32_PIN_PB00
#define SDRENABLE	AVR32_PIN_PB01
#define LED			AVR32_PIN_PA11
#define SI			AVR32_PIN_PA17
#define PWRDET		AVR32_PIN_PA31

#define PORTA		(AVR32_PIN_PA31 >> 5)
#define PORTB		(AVR32_PIN_PB11 >> 5)

#define CH_A	1
#define CH_B	2

#define REGENTRIES		33		// total number of entries for register structure table;

#define MAX_SAFE_DIV2_FADC	58500000L

typedef struct {
	unsigned char regaddr;
	unsigned char datalen;
	char* description;
	char flags;
} REGSTRUCT;

typedef struct {
	char dec;
	char dec_by_4;
	char scale;
	char gain_a;
	char gain_b;
	char rate;
	char serial_ctrl_dummy;
	char freq_a[4];
	char phase_a[2];
	char freq_b[4];
	char phase_b[2];
	char source;
	char agc_ctrl;
	char agc_ic_a;
	char agc_ic_b;
	char agc_rb_a;
	char agc_rb_b;
	char test_reg[2];
	char debug;
	char agc_table[32];

	char f1_ctrl_A;
	char f2_ctrl_A;
	char f1_coeff_A[22];
	char f2_coeff_A[64];

	char f1_ctrl_B;
	char f2_ctrl_B;
	char f1_coeff_B[22];
	char f2_coeff_B[64];

	char f1_ctrl;
	char f2_ctrl;
	char serial_ctrl2;
	char agc_ctrl2;

	char serial_ctrl;
} REGVALS;

extern uint32_t address_bitmask;
extern uint32_t data_bitmask;
extern uint32_t ctrl_bitmask;
extern REGVALS reg;
extern REGSTRUCT regstruct[REGENTRIES];
extern int16_t _F1_TABLE_STD[11];
extern int16_t _F2_TABLE_STD[32];
extern int16_t _F1_TABLE_SDR[11];
extern int16_t _F2_TABLE_SDR[32];
extern int8_t _AGC_TABLE[32];

extern volatile uint8_t diversity;
extern volatile uint8_t scanner;
extern volatile int32_t lastfreq_A;
extern volatile int32_t lastfreq_B;
extern volatile bool powered;

unsigned char ReadRegister(unsigned char regno);
void WriteRegister(unsigned char regno, unsigned char regval, unsigned char toggle_si);
void AssertSI(void);
void SetSI(int state);
uint8_t DiversityMode(uint8_t mode, uint32_t adcfreq);
void SetFreq(int16_t channel, int32_t _freq, int16_t write, uint32_t adcfreq);
void SetPhase(int16_t channel, uint16_t phaseword);
uint8_t SetGain(int16_t channel, int16_t _gain);
int16_t GetGain(int16_t channel);
void Int2Phase(char* phasestore, int16_t phase);
uint64_t pow2(int16_t power);
uint64_t powX(uint64_t numvalue, int16_t power);
void memmove_BE(uint8_t* dest, uint8_t* src, uint16_t count);
void Init_LM97593(uint32_t sdr_sample_freq, uint16_t bits_per_word, bool freqandphase, uint32_t adcfreq);
int16_t UpdateRegisters(void);

void StartRadio(void);
void StopRadio(void);
void ResetRadio(void);

void xsrand(unsigned x);
int16_t xrand(void);
void MonteCarlo(void);

#endif /* LM97593_H_ */

