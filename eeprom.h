/*
 * eeprom.h
 *
 *  Created on: 18 Nov 2020
 *      Author: Ruthya Gowda
 */

#ifndef EEPROM_H_
#define EEPROM_H_

void initeeprom();
void write_eeprom(uint16_t add,uint32_t data);
uint32_t readeeprom(uint16_t add);



#endif /* EEPROM_H_ */
