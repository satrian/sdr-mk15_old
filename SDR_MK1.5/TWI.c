/*
 * TWI.c
 *
 * Created: 1/22/2012 10:26:49 PM
 *  Author: Laid
 */

#include "compiler.h"	//ASF
#include "TWI.h"
#include "cycle_counter\cycle_counter.h"

// TWI interface (will be initialized through SetupHardware()
volatile avr32_twi_t *twi = &AVR32_TWI;

/*
Copied from Atmel ASF code.
Fixed: speed and c_lh_div modified to be 32-bit
*/
int16_t twi_set_speed(volatile avr32_twi_t *twi, uint32_t speed, uint32_t pba_hz)
{
uint16_t ckdiv = 0;
uint32_t c_lh_div;

	c_lh_div = pba_hz / (speed * 2) - 4;

	// cldiv must fit in 8 bits, ckdiv must fit in 3 bits
	while ((c_lh_div > 0xFF) && (ckdiv < 0x7))
	{
		// increase clock divider
		ckdiv++;
		// divide cldiv value
		c_lh_div /= 2;
	}

	// set clock waveform generator register
	twi->cwgr = ((c_lh_div << AVR32_TWI_CWGR_CLDIV_OFFSET) |
				(c_lh_div << AVR32_TWI_CWGR_CHDIV_OFFSET) |
				(ckdiv << AVR32_TWI_CWGR_CKDIV_OFFSET));

	return	0;	// TWI_SUCCESS;
}

/*
Initialize TWI. As the ASF code uses interrupt system what clashes with the LUFA stuff,
we have to implement our own interrupt-less master here.
*/
void twi_init(void)
{
	twi->cr = AVR32_TWI_CR_SWRST_MASK;		// perform software reset to TWI interface
	cpu_delay_us(1000, F_CPU);
	twi_set_speed(twi, 100000, F_PBA_SPEED);
}

/*
Read single byte as master from TWI from specified address
*/
uint8_t twi_read(uint8_t addr, uint16_t *statbuff)
{
int16_t i, j;
uint16_t s;
uint16_t *statptr;
uint8_t retval;

	if (!statbuff)
		statptr=&s;				// put local in place in case we were not given any
	else
		statptr=statbuff;

	twi->cr = AVR32_TWI_SVDIS_MASK;			// disable slave
	i=twi->sr;	// dummy read to clear status

	twi->mmr = ((0x65		<< AVR32_TWI_DADR_OFFSET)				|	// Device addres to access. We have only one TWI device, so we can hardcode it here
				(1			<< AVR32_TWI_MREAD_OFFSET)				|	// Master is read direction
				(1			<< AVR32_TWI_IADRSZ_OFFSET));				// 1-byte device internal address

	twi->iadr=addr|0x80;		// byte read/write, not block operation, so rise the first bit

	twi->cr = AVR32_TWI_MSEN_MASK;			// enable master

	twi->cr = AVR32_TWI_START_MASK;			// init transfer
	twi->cr = AVR32_TWI_STOP_MASK;			// stop condition after master read is complete for current byte

	for (i=0; i<50; i++)
	{
		*statptr=twi->sr;
		if ((*statptr) & AVR32_TWI_RXRDY_MASK)
		{
			retval=twi->rhr;

			for (j=0; j<10; j++)
			{
				*statptr=twi->sr;
				if ((*statptr) & AVR32_TWI_TXCOMP_MASK)
					return retval;

				cpu_delay_us(100, F_CPU);
			}

			*statptr|=0x1000;
			return retval;
		}
		cpu_delay_us(100, F_CPU);
	}
	*statptr|=0x2000;
	return twi->rhr;
}

/*
Write single byte as master to TWI to specified address
*/
uint8_t twi_write(uint8_t addr, uint8_t databyte)
{
int16_t i, j;
uint16_t stat;

	twi->cr = AVR32_TWI_SVDIS_MASK;			// disable slave
	i=twi->sr;	// dummy read to clear status

	twi->mmr = ((0x65		<< AVR32_TWI_DADR_OFFSET)				|	// Device addres to access. We have only one TWI device, so we can hardcode it here
				(0			<< AVR32_TWI_MREAD_OFFSET)				|	// Master is write direction
				(1			<< AVR32_TWI_IADRSZ_OFFSET));				// 1-byte device internal address

	twi->iadr=addr|0x80;		// byte read/write, not block operation, so rise the first bit

	twi->cr = AVR32_TWI_MSEN_MASK;			// enable master

	twi->thr=databyte;						// write now
	//twi->cr = AVR32_TWI_START_MASK;			// init transfer
	//twi->cr = AVR32_TWI_STOP_MASK;			// stop condition after master read is complete for current byte

	for (i=0; i<50; i++)
	{
		stat=twi->sr;
		if (stat & AVR32_TWI_TXRDY_MASK)
		{
			for (j=0; j<10; j++)
			{
				stat=twi->sr;
				if (stat & AVR32_TWI_TXCOMP_MASK)
					return stat;

				cpu_delay_us(100, F_CPU);
			}

			stat=0x1000;
			return stat;
		}
		cpu_delay_us(100, F_CPU);
	}
	stat|=0x2000;
	return stat;
}