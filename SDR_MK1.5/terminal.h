/*
 * terminal.h
 *
 * Created: 3/11/2012 8:27:05 PM
 *  Author: Laid
 */


#ifndef TERMINAL_H_
#define TERMINAL_H_


void CMD_Condition(char* cmdbuffer);
void CMD_Parse(char* cmdbuffer, char** cmdname, char** carg1, char** carg2, char** carg3);


#endif /* TERMINAL_H_ */