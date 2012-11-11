/*
 * terminal.c
 *
 * Created: 3/11/2012 8:26:41 PM
 *  Author: Laid
 */

#include "sdr_mk1.5.h"
#include "terminal.h"

//clean up the command line by removing unnecessary whitespace, convert everything to lowercase etc.

void CMD_Condition(char* cmdbuffer)
{
uint16_t i;

	// strip trailing spaces
	for (i=strlen(cmdbuffer)-1; i>0; i--)
	{
		if (cmdbuffer[i]==' ')
			cmdbuffer[i]=0;
		else
			break;
	}
}

//seek the command line argument pointers to the correct offsets inside cmdbuffer string

void CMD_Parse(char* cmdbuffer, char** cmdname, char** carg1, char** carg2, char** carg3)
{
uint16_t a, b;

	*cmdname=cmdbuffer;
	*carg1=NULL;
	*carg2=NULL;
	*carg3=NULL;

	for (b=0, a=0; b<4; b++)
	{
		// parse first whitespace, stopping at the end of string
		for (; a<strlen(cmdbuffer); a++)
		{
			if (cmdbuffer[a] == ' ')
				break;
			if (cmdbuffer[a] == 0)
				return;
		}
		// parse till the end of whitespace (in case input was not conditioned string)
		for (; a<strlen(cmdbuffer); a++)
		{
			if (cmdbuffer[a] != ' ')
				break;
			if (cmdbuffer[a] == 0)
				return;
		}

		switch (b)
		{
			case 0:
				*carg1=cmdbuffer+a;
				break;
			case 1:
				*carg2=cmdbuffer+a;
				break;
			case 2:
				*carg3=cmdbuffer+a;
				break;
			default:
				break;	// shall never happen
		}
	}
}