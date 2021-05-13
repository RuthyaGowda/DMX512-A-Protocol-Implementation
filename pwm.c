/*
 * pwm.c
 *
 *  Created on: 23 Nov 2020
 *      Author: Ruthya Gowda
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "pwm.h"


#define BLUE_BL_LED_MASK          4             //Port F pin2


void initpwmlight()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;                      //Clock is enabled for PWM module1
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;                    //Clock is enabled for PortF
    _delay_cycles(3);

    // Configure three backlight LEDs
    GPIO_PORTF_DIR_R |= BLUE_BL_LED_MASK;  // make bits 3,4 and 5 outputs
    GPIO_PORTF_DR2R_R |= BLUE_BL_LED_MASK; // set drive strength to 2mA
    GPIO_PORTF_DEN_R |=  BLUE_BL_LED_MASK;  // enable digital
    GPIO_PORTF_AFSEL_R |= BLUE_BL_LED_MASK;// select auxilary function
    GPIO_PORTF_PCTL_R &=  GPIO_PCTL_PF2_M;    // enable PWM
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF2_M1PWM6;

    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ZERO | PWM_1_GENA_ACTLOAD_ONE;    // output 2 on PWM1, gen 2b, cmpb
    PWM1_3_LOAD_R = 256;                             // set period to 40 MHz sys clock / 2 / 256 = 78.125 kHz
    PWM1_INVERT_R = PWM_INVERT_PWM6INV;             // invert outputs so duty cycle increases with increasing compare values
    PWM1_3_CMPA_R = 0;                               // blue off
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM6EN;               // enable outputs
}

void setpwmColor(uint16_t blue)
{
    PWM1_3_CMPA_R = blue;
}
