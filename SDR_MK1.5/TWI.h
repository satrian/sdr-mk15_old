/*
 * TWI.h
 *
 * Created: 1/22/2012 10:27:05 PM
 *  Author: Laid
 */ 


#ifndef TWI_H_
#define TWI_H_


int16_t twi_set_speed(volatile avr32_twi_t *twi, uint32_t speed, uint32_t pba_hz);
void twi_init(void);
uint8_t twi_read(uint8_t addr, uint16_t *statbuff);
uint8_t twi_write(uint8_t addr, uint8_t databyte);


#endif /* TWI_H_ */