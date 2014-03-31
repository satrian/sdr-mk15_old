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
uint8_t twi_read(uint8_t devaddr, uint8_t command, uint16_t *statbuff);
uint8_t twi_write(uint8_t devaddr, uint8_t command, uint8_t databyte);

unsigned long twi_read_blk(uint8_t devaddr, uint8_t iaddrlen, uint8_t iaddr, uint8_t bytes, uint8_t *databuff);
unsigned long twi_write_blk(uint8_t devaddr, uint8_t iaddrlen, uint8_t iaddr, uint8_t bytes, uint8_t *databuff);


#endif /* TWI_H_ */