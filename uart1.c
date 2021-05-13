// UART1 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart1.h"
#include "eeprom.h"
#include "pwm.h"

// PortA masks
#define UART_TX_MASK 32
#define UART_RX_MASK 16

// PortF masks
#define RED_LED_MASK            2
#define BLUE_LED_MASK           4
#define GREEN_LED_MASK          8
#define PC4_MASK                32
#define Data_MASK               16        //PC5
#define Data_Enable_Mask        128       //PC7

#define RED_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define Data          (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
#define Data_Enable   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define PC4           (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart1()
{
    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1 | SYSCTL_RCGCTIMER_R2 | SYSCTL_RCGCTIMER_R3;
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Configure UART1 pins
    GPIO_PORTC_DATA_R |= (UART_TX_MASK|Data_Enable_Mask);
    GPIO_PORTC_DIR_R |= (UART_TX_MASK |Data_Enable_Mask);                   // enable output on UART1 TX pin and PC7
    GPIO_PORTC_DIR_R &= ~UART_RX_MASK;                                      // enable input on UART1 RX pin
    GPIO_PORTC_DR2R_R |= (UART_TX_MASK |Data_Enable_Mask);                  // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R |= UART_TX_MASK | UART_RX_MASK |Data_Enable_Mask;      // enable digital on UART1 pins

    //Configure LED pin
       GPIO_PORTF_DIR_R |= RED_LED_MASK | BLUE_LED_MASK;                    // make bit an output
       GPIO_PORTF_DR2R_R |= RED_LED_MASK | BLUE_LED_MASK;                   // set drive strength to 2mA (not needed since default configuration -- for clarity)
       GPIO_PORTF_DEN_R |= RED_LED_MASK | BLUE_LED_MASK;                    // enable LED

    // Configure UART1 to 250k baud, 8N2 format
    UART1_CTL_R = 0;                                                        // turn-off UART0 to allow safe programming
    UART1_CC_R |= UART_CC_CS_SYSCLK;                                        // use system clock (40 MHz)
    UART1_IBRD_R = 10;                                                      // r = 40 MHz / (Nx250kkHz), set floor(r)=10, where N=16
    UART1_FBRD_R = 00;                                                      // round(fract(r)*64)=00
    UART1_LCRH_R |= UART_LCRH_WLEN_8|UART_LCRH_STP2;                        // configure for 8N2 w/ 16-level FIFO
    UART1_CTL_R |= UART_CTL_EOT|UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                                            // enable TX, RX, and module
}

// Set baud rate as function of instruction cycle frequency
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;                       // calculate divisor (r) in units of 1/128,
                                                                            // where r = fcyc / 16 * baudRate
    UART1_IBRD_R = (divisorTimes128 + 1) >> 7;                              // set integer value to floor(r)
    UART1_FBRD_R = (((divisorTimes128 + 1)) >> 1) & 63;                     // set fractional value to round(fract(r)*64)
}

void starttimer176us()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                                        // turn-off timer before reconfiguring
    TIMER1_CFG_R |= TIMER_CFG_32_BIT_TIMER;                                 // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT;                                // configure for periodic mode (count down)
    TIMER1_TAILR_R = 7040;                                                  // set load value to 7040 for 5681.8 Hz interrupt rate
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;                                       // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);                                    // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void starttimer12us()
{
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                                        // turn-off timer before reconfiguring
    TIMER1_CFG_R |= TIMER_CFG_32_BIT_TIMER;                                 // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R |= TIMER_TAMR_TAMR_1_SHOT;                                // configure for periodic mode (count down)
    TIMER1_TAILR_R = 480;                                                   // set load value to 480 for 83333.3 Hz interrupt rate
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;                                       // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A-16);                                    // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void starttimer10ms()
{
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                                        // turn-off timer before reconfiguring
    TIMER3_CFG_R |= TIMER_CFG_32_BIT_TIMER;                                 // configure as 32-bit timer (A+B)
    TIMER3_TAMR_R |= TIMER_TAMR_TAMR_PERIOD;                                // configure for periodic mode (count down)
    TIMER3_TAILR_R = 400000;                                                // set load value to 400000 for 100 Hz interrupt rate
    TIMER3_IMR_R |= TIMER_IMR_TATOIM;                                       // turn-on interrupts
    NVIC_EN1_R |= 1 << (INT_TIMER3A-16-32);                                 // turn-on interrupt 51 (TIMER3A)
    TIMER3_CTL_R |= TIMER_CTL_TAEN;
}

void starttimer8us()
{
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN;                                        // turn-off timer before reconfiguring
    TIMER2_CFG_R |= TIMER_CFG_32_BIT_TIMER;                                 // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R |= TIMER_TAMR_TAMR_PERIOD;                                // configure for periodic mode (count down)
    TIMER2_TAILR_R = 320;                                                   // set load value to 320 for 125000 Hz interrupt rate
    TIMER2_IMR_R |= TIMER_IMR_TATOIM;                                       // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER2A-16);                                    // turn-on interrupt 39 (TIMER2A)
    TIMER2_CTL_R |= TIMER_CTL_TAEN;
}

void StartDmxTx()
{
    GPIO_PORTC_AFSEL_R &= ~UART_TX_MASK;
    Data_Enable = 1;
    Data=0;                                                                 // PC5 is set to 0
    phase = 0;                                                              // Break is set
    UART1_IM_R |= UART_IM_TXIM;                                             // turn-on UART1 TX interrupt
    NVIC_EN0_R |= 1<<(INT_UART1-16);                                        // turn-on interrupt 22
    starttimer176us();                                                      // Timer is fired for 176us
}

void StartDmxRx()
{
    GPIO_PORTC_AFSEL_R |= UART_RX_MASK;                                     // use peripheral to drive PC4
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;                                  // clear bits 0-7
    GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC4_U1RX;                                // select UART1 to drive pins PC4: default, added for clarity
    UART1_IM_R |= UART_IM_RXIM;                                             // turn-on UART1 RX interrupt
    NVIC_EN0_R |= 1<<(INT_UART1-16);                                        // turn-on interrupt 22
    Data_Enable = 0;                                                        // PC7 is set to 0
    PC4 = 1;                                                                // PC4 is set to 1
}

void TimerIsr()
{
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;                                     // clear the timer1 interrupt
    if(phase == 0)
    {
       Data=1;                                                              // PC5 is set to 1
       phase=1;                                                             // Mark after break is set
       starttimer12us();                                                    // Timer is fired for 12us

    }
    if(phase == 1)
    {
        TIMER1_ICR_R |= TIMER_ICR_TATOCINT;                                 // clear the timer1 interrupt
        GPIO_PORTC_AFSEL_R |= UART_TX_MASK;                                 // use peripheral to drive PC5
        GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC5_M;                              // clear bits 0-7
        GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX;                            // select UART1 to drive pins PC5: default, added for clarity
        phase=2;
        if(flagpoll)
        {
//            pollphase = 0;
            UART1_DR_R = (0xF6);                                            // Start code set as F6h for polling
        }
        else
        {
            flagpoll = false;
            UART1_DR_R = 0;                                                  //Start code set as 0 for transmission
        }

    }
}

void Uart1Isr()
{
    //mode = readeeprom(0x00);
    if(mode==0x00)                                                           // controller mode
    {
    UART1_ICR_R |= UART_ICR_TXIC;                                            // clear the UART1 TX interrupt
    if(phase-2<MAX)
    {
        UART1_DR_R =datatx[phase-2];                                         // store 512 bytes data in UART1 data register
        phase++;                                                             // increase the phase count
    }
    else
    {
            if(run==1)
            {
            StartDmxTx();
            }
    }

    UART1_ICR_R |= UART_ICR_TXIC;                                            // clear the UART1 TX interrupt
    }


    if(mode==0x01)                                                           //device mode
    {
        //address_1 = readeeprom(0x00);
        if(!(UART1_FR_R & UART_FR_RXFE))                                     // check if the UART RX fifo is not empty
        {
            UART1_ICR_R |= UART_ICR_RXIC;                                    // clear the UART1 RX interrupt
            data2 = UART1_DR_R;                                              // store the data in UART data register to data2
            UART1_ICR_R |= UART_ICR_RXIC;
            if(data2 & UART_DR_BE)                                           //checking if BE is set to 1
            {
                rx_phase = 0;

                data2 = UART1_DR_R;
                data2 &= (0xFF);
                if(UART1_DR_R == (0xF6))                                     // checks if F6h is stored in UART dataregister
                {
                    rxflagpoll = true;
                    quo=(address_1-1)/8;                                     // calculating the bytes position
                    rem=(address_1-1)%8;                                     // calculating the bits position
                }
                else
                {
                    rxflagpoll = false;
                }
                UART1_ICR_R |= UART_ICR_RXIC;
            }
            else
            {
                data2 = UART1_DR_R;                                         // store the data in UART data register to data2 variable
                datarx[rx_phase] =  data2  & 0xFF;
                if(rxflagpoll)
                {
                    if(rx_phase == quo)
                    {
                        if(data2 & (1<<rem))
                        {
                            RED_LED = 1;
                        }
                    }
                    rx_phase++;
                    UART1_ICR_R |= UART_ICR_RXIC;
                }
                else
                {
                    if(rx_phase == address_1)                                 // check if rxphase matches with address_1 given in set command
                    {
                        setpwmColor(data2);                                   // set the pwm for blue led with the data stored in address_1 given in set command
                    }
                    rx_phase++;                                               // increment the rxphase for 512 bytes
                }
            }
            UART1_ICR_R |= UART_ICR_RXIC;
        }
        }
}

void Timer3Isr()
{
    TIMER3_ICR_R |= TIMER_ICR_TATOCINT;

    uint16_t timeraddr, timerdata;
 if (checkpattern<patterncount)
{
    if(patterndata[checkpattern].type == ramptype)                               // check if it's a ramp function
    {
        if(patterndata[checkpattern].delN<patterndata[checkpattern].N)           // check if deltaN is less than N
        {
            patterndata[checkpattern].delN++;                                    // increment deltaN
            timeraddr = patterndata[checkpattern].add;                           // store the address given in ramp command
            timerdata = patterndata[checkpattern].value1+(((patterndata[checkpattern].value2-patterndata[checkpattern].value1)*patterndata[checkpattern].delN)/patterndata[checkpattern].N);
                                                                                 // store the calculated data
            datatx[timeraddr-1] = timerdata;                                     // send the calculated data to the buffer
            StartDmxTx();                                                        // start transmission in DMX
        }
        else
        {
            patterndata[checkpattern].delN = 0;                                  // reset the deltaN to 0
            checkpattern++;                                                      // increment the checkpattern

        }
    }
    else if(patterndata[checkpattern].type == pulsetype)                         // check if it's in pulse function
    {
        if(patterndata[checkpattern].delN<=patterndata[checkpattern].N)          // check if deltaN is less than N
        {
            if(patterndata[checkpattern].delN == 0)                              // check if deltaN is 0
            {
                timeraddr = patterndata[checkpattern].add;                       // store the address given in pulse command
                timerdata = patterndata[checkpattern].value1;                    // store the value1 given in pulse command
                datatx[timeraddr-1] = timerdata;                                 // store the value1 in the buffer
                StartDmxTx();                                                    // start transmission in DMX
            }
            if(patterndata[checkpattern].delN == patterndata[checkpattern].N)    // check if deltaN and N(interrupts) are equal
            {
                timeraddr = patterndata[checkpattern].add;                       // store the address given in pulse command
                timerdata = patterndata[checkpattern].value2;                    // store the value2 given in pulse command
                datatx[timeraddr-1] = timerdata;                                 // store the value2 in the buffer
                StartDmxTx();                                                    // start transmission in DMX
            }
            patterndata[checkpattern].delN++;                                    // increment deltaN
        }
        else
        {
            patterndata[checkpattern].delN = 0;                                  // reset the deltaN to 0
            checkpattern++;                                                      // increment the checkpattern
        }
    }
}
else
{
    checkpattern = 0;                                                            // reset the checkpattern to 0
    TIMER3_CTL_R &= ~TIMER_CTL_TAEN;                                             // turn-off the timer3 interrupt
    timerflag = false;
}
}

void Timer2Isr()
{

    TIMER2_ICR_R |= TIMER_ICR_TATOCINT;
    if(pollcount<512)
    {
        quo = pollcount/8;                                                        // set the quotient to the bytes of buffer
        rem = pollcount%8;                                                        // set the reminder to the bits of buffer
        datatx[quo] |= (1<<rem);                                                  // store the value to 64bytes(512bits)
        pollcount++;                                                              // increment the pollcount till it reaches 512 bits
        GREEN_LED = 1;
        starttimer10ms();
        StartDmxTx();

    }
    else
    {
        TIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    }
    if(timeflag)
    {
        if(TIMEOUT>0)
        {
            TIMEOUT--;
            if(TIMEOUT == 0)
            {
                GREEN_LED = 0;
            }
        }
    }

}
