/*
 * omnirig.c
 *
 * Created: 3/11/2012 8:31:00 PM
 *  Author: Laid
 */
#include "sdr_mk1.5.h"
#include "omnirig.h"
#include "clock-arch.h"

uint32_t pollCounter=0L;

void CAT_Poll(void)
{
	if (pollCounter > millis())
	{
		pollCounter=millis()+500;

		Message(1, "FR;");		// fetch current ctive VFO
		Message(1, "FB;");		// fetch VFO-B (which is our LFO frequency)
		Message(1, "AG;");		// fetch Audio Gain, what we are using as the RF Gain really!
	}
}