#include "uip-conf.h"
//#include "dhcpc.h"
#include "global-conf.h"
//#include <stdint.h>
//#include <stdlib.h>
//#include <stdio.h>
//#include <avr/interrupt.h>
//#include <avr/io.h>
//#include <avr/sfr_defs.h>

//from ASF
#include "tc.h"
//#include "eth_spi.h"
//#include "pm.h"
//#include "power_clocks_lib.h"

#include <LUFA/Platform/UC3/InterruptManagement.h>		//ISR() macro and interrupt setup


#include "clock-arch.h"

#define TC_CHANNEL		0
#define FALSE			0

//Counted time
static uint32_t clock_datetime = 0;


ISR(tc_irq)
{
  // Increment the ms counter
  clock_datetime++;

  // Clear the interrupt flag. This is a side effect of reading the TC SR.
  tc_read_sr(&AVR32_TC, TC_CHANNEL);
}

volatile avr32_tc_t *tc = &AVR32_TC;

// Options for waveform genration.
static const tc_waveform_opt_t WAVEFORM_OPT =
{
	.channel  = TC_CHANNEL,                        // Channel selection.

	.bswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOB.
	.beevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOB.
	.bcpc     = TC_EVT_EFFECT_TOGGLE,                // RC compare effect on TIOB.
	.bcpb     = TC_EVT_EFFECT_TOGGLE,                // RB compare effect on TIOB.

	.aswtrg   = TC_EVT_EFFECT_NOOP,                // Software trigger effect on TIOA.
	.aeevt    = TC_EVT_EFFECT_NOOP,                // External event effect on TIOA.
	.acpc     = TC_EVT_EFFECT_TOGGLE,                // RC compare effect on TIOA: toggle.
	.acpa     = TC_EVT_EFFECT_TOGGLE,                // RA compare effect on TIOA: toggle (other possibilities are none, set and clear).

	.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
	.enetrg   = FALSE,                             // External event trigger enable.
	.eevt     = 0 ,       						// External event selection.
	.eevtedg  = TC_SEL_NO_EDGE,                    // External event edge selection.
	.cpcdis   = FALSE,                             // Counter disable when RC compare.
	.cpcstop  = FALSE,                             // Counter clock stopped with RC compare.

	.burst    = FALSE,                             // Burst signal selection.
	.clki     = FALSE,                             // Clock inversion.
	.tcclks   = TC_CLOCK_SOURCE_TC3                // Internal source clock 3, connected to fPBA / 8.
};

static const tc_interrupt_t TC_INTERRUPT =
{
	.etrgs = 0,
	.ldrbs = 0,
	.ldras = 0,
	.cpcs  = 1,
	.cpbs  = 0,
	.cpas  = 0,
	.lovrs = 0,
	.covfs = 0
};

//Initialize the clock
void clock_init(void)
{
	GlobalInterruptDisable();
	INTC_RegisterGroupHandler(INTC_IRQ_GROUP(AVR32_TC_IRQ0), AVR32_INTC_INT0, tc_irq);
	GlobalInterruptEnable();

	// Initialize the timer/counter.
	tc_init_waveform(tc, &WAVEFORM_OPT);         // Initialize the timer/counter waveform.

	// Set the compare triggers.
	// Remember TC counter is 16-bits, so counting second is not possible with fPBA = 12 MHz.
	// We configure it to count ms.
	// We want: (1/(fPBA/8)) * RC = 0.001 s, hence RC = (fPBA/8) / 1000 = 1500 to get an interrupt every 1 ms.
	tc_write_rc(tc, TC_CHANNEL, (F_PBA_SPEED / 8) / 1000); // Set RC value.
	//tc_write_ra(tc, TC_CHANNEL, 0);					// Set RA value.
	//tc_write_rb(tc, TC_CHANNEL, 1900);				// Set RB value.

	//gpio_enable_module_pin(AVR32_TC_A0_0_1_PIN, AVR32_TC_A0_0_1_FUNCTION);
	//gpio_enable_module_pin(AVR32_TC_B0_0_1_PIN, AVR32_TC_B0_0_1_FUNCTION);

	tc_configure_interrupts(tc, TC_CHANNEL, &TC_INTERRUPT);

	// Start the timer/counter.
	tc_start(tc, TC_CHANNEL);                    // And start the timer/counter.

}

//Return time
uint32_t clock_time(void)
{
uint32_t time;

	GlobalInterruptDisable();
	time = clock_datetime;
	GlobalInterruptEnable();

	return time;
}

uint32_t millis(void)
{
	return(clock_time());
}
