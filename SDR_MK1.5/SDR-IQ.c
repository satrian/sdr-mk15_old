/*
 * SDR_IQ.c
 *
 * Functionality for handling SDR-IQ protocol v1.01 from RFspace
 *
 */

#include "SDR-IQ.h"

bool outputsamples=false;
char* cmdbuffer[16];		// we do not support command blocks longer than 16 bytes, as we do not expect to have any block transmissions happening towards us.
int bytesincmdbuffer=0;

extern int iqpktsize;

/*
 Command bytes processor.
 
 SDR-IQ protocol is somewhat crippled, as it does not have any fields for determining command block header. 
 Therefore we are just starting to parse from the beginning of the received data and hope that miracles happen.
 Should there be a garbage byte somewhere, chances are that the comm sync is lost forever. Unfortunately, this is
 what the protocol looks like, so no can do.
*/
 
bool IQ_ProcessByte(USB_ClassInfo_CDC_Device_t* const CDCInterfaceInfo)
{
int16_t recbyte;

	recbyte=CDC_Device_ReceiveByte(CDCInterfaceInfo);

	if (recbyte!=-1)
	{
		CDC_Device_SendByte(CDCInterfaceInfo, recbyte);
		
		if (recbyte == 1)		// hack - monitor for our control message last byte
		{
			iqpktsize=0;		// reset sync
			outputsamples=true;
		}			
			
		if (recbyte == 0xFF)	// hack - stop stream if 0xFF is received
			outputsamples=false;
	}

	return outputsamples;
}