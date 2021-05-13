/*
 * eeprom.c
 *
 *  Created on: 18 Nov 2020
 *      Author: Ruthya Gowda
 */
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "eeprom.h"

void initeeprom()
{
    SYSCTL_RCGCEEPROM_R |= SYSCTL_RCGCEEPROM_R0;
    _delay_cycles(3);
    while(EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

void write_eeprom(uint16_t add,uint32_t data)
{
    EEPROM_EEBLOCK_R = add>>4;
    EEPROM_EEOFFSET_R = add & 0xF;
    EEPROM_EERDWR_R = data;
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

uint32_t readeeprom(uint16_t add)
{
    EEPROM_EEBLOCK_R = add>>4;
    EEPROM_EEOFFSET_R = add & 0xF;
    return EEPROM_EERDWR_R;
}
