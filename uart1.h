// UART0 Library
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

#ifndef UART_H_
#define UART_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
#define MAX               512
#define Maxpollphase      64
#define ramptype          20
#define pulsetype         10
uint16_t datatx[MAX];
uint16_t address_1;
uint32_t data2;
uint8_t datarx[MAX];
uint16_t address;
uint16_t mode;
uint16_t polldatatx[Maxpollphase];
uint16_t polldatarx[Maxpollphase];
uint16_t pollcount,pollphase;
uint16_t rx_phase;
uint32_t data;
uint8_t run;
uint16_t phase;
uint16_t data1,patterncount,checkpattern;
uint16_t delncount;
uint8_t quo,rem,i,j;
bool flagpoll;
bool rxflagpoll,timeflag,rampflag1,timerflag;
int TIMEOUT;
struct{
    uint16_t add;
    uint16_t value1;
    uint16_t value2;
    uint16_t time;
    uint16_t N;
    uint16_t delN;
    uint16_t type;
}patterndata[10];



void initUart1();
void setUart1BaudRate(uint32_t baudRate, uint32_t fcyc);
void starttimer176us();
void starttimer12us();
void starttimer8us();
void StartDmxTx();
void StartDmxRx();
void starttimer10ms();
void startpolling();


#endif
