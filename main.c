#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart1.h"
#include "uart0.h"
#include "eeprom.h"
#include "pwm.h"

// Bitband alias
#define RED_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define Data_Enable   (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 7*4)))
#define Data          (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 5*4)))
//#define Receive       (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))
#define PC4           (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))

#define MAX_CHARS               80
#define MAX_ARGS                5
#define SIZE_OF_BUFFER          1024
#define MAX                     512
// PortF masks
#define RED_LED_MASK            2
#define BLUE_LED_MASK           4
#define GREEN_LED_MASK          8
#define PC4_MASK                32

#define CHECK_ALPHA(x) ((x>=65 && x<=90) || (x>=97 && x<=122) ? 1:0)
#define CHECK_NUM(x) ((x>=48 && x<=57) || (x==45) || (x==46) ? 1:0)

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

     //Configure LED pin
    GPIO_PORTF_DIR_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;  // make bit an output
    GPIO_PORTF_DR2R_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= RED_LED_MASK | BLUE_LED_MASK | GREEN_LED_MASK;  // enable LED
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------
//bool pollingflag = false;
typedef struct _USER_DATA{
    char strBuffer[MAX_CHARS+1];
    uint8_t arg_count;
    uint8_t index[MAX_ARGS];
    char type[MAX_ARGS];
}USER_DATA;
//Stores Uart buffer in String(strBuffer)

struct _Ringbuffer
{
    char strBuffer[SIZE_OF_BUFFER];
    uint16_t read_index;
    uint16_t write_index;
}ringbuffer;

uint16_t len_str(const char strBuffer[])
{
    uint16_t length=0;
    for(length=0;length<MAX_CHARS;length++)
    {
        if(strBuffer[length] == '\0')
         return length;
    }
    return length;
}

void getsUart0(USER_DATA* dataptr)
{
    char c;
    uint8_t count = 0;
    while(count<MAX_CHARS+1)
    {
       c = getcUart0();
       if(c == 8 || c == 127)                                                  //8 is for backspace and 127 is for delete
       {
           if(count>0)
           {
               count--;
           }
           continue;
       }
       if(c == 10 || c == 13)                                                  // 10 is for line feed and 13 is for carriage return
       {
           dataptr->strBuffer[count] = 0;
           break;
       }
       if(c >=' ')                                                             // checks if c is greater than space(32)
       {
            dataptr->strBuffer[count++] = c;                                   // store the c value in the buffer
           // dataptr->strBuffer[(wrindex+1)%N]
            if(count == MAX_CHARS)                                             // checks if count is more than 80
            {
                dataptr->strBuffer[count] = 0;                                 // then set the buffer to 0 and exit
                break;
            }
            continue;
       }

    }
}


void parse(USER_DATA* dataptr)      //step 3
{
    uint8_t i,len;
    dataptr->arg_count=0;
    len=len_str(dataptr->strBuffer);
    for(i=0;i<=len;i++)
    {
      if(i==0)
      {
          if(CHECK_ALPHA(dataptr->strBuffer[i])==1)                        // check if the first character in buffer is an alpha
          {
             dataptr->index[dataptr->arg_count]=i;                         // write the first character
             dataptr->type[dataptr->arg_count]='a';                        // first character represents alpha
             dataptr->arg_count++;                                         // increment the argcount
          }
          if(CHECK_NUM(dataptr->strBuffer[i])==1)                          // check if the first character in buffer is an number
          {
              dataptr->index[dataptr->arg_count]=i;                        // write the first character
              dataptr->type[dataptr->arg_count]='n';                       // first character represents alpha
              dataptr->arg_count++;                                        // increment the argcount
          }
          continue;
      }
      if(CHECK_ALPHA(dataptr->strBuffer[i]) || CHECK_NUM(dataptr->strBuffer[i]))          // check if the pointed character is number or alpha
      {
          if(!(CHECK_ALPHA(dataptr->strBuffer[i-1]) || CHECK_NUM(dataptr->strBuffer[i-1]))) // checks the previous character if it is number or alpha
                  {
                         dataptr->index[dataptr->arg_count]=i;                              // stores the value of the character
                         if(CHECK_ALPHA(dataptr->strBuffer[i]))                             // checks if it's an alpha or number
                             {
                                 dataptr->type[dataptr->arg_count]='a';                     // indicates it an a alpha
                             }
                         else
                             {
                                 dataptr->type[dataptr->arg_count]='n';                     // indicates it as a number
                             }
                         dataptr->arg_count++;

                  }
      }

      else
      {
          dataptr->strBuffer[i]='\0';                                     // setting the delimiters to null value
      }
      }
}

bool strMatch(char* x,char* y)
{
    bool flag = true;
    while(*x!='\0' || *y!='\0')
    {
        if(*x == *y)
        {
            x++;
            y++;
        }
        else if((*x!=*y) || (*x!='\0' && *y=='\0') || (*x=='\0' && *y!='\0'))
        {
            flag=false;
            break;
        }
    }
    return flag;
}

bool isCommand(char* str,uint8_t extraargs,USER_DATA* dataptr)
{

    if(strMatch(str,&dataptr->strBuffer[dataptr->index[0]]))                     //strmatch=first argument in strBuffer
    {
        if(extraargs<dataptr->arg_count)                                         //extraargs<argcount
        {
            return true;                                                         //return true if both condition are true
        }
    }
    return false;
}

int Atoi(char* str)
{
    int i=0;
    int res = 0;
    for(i=0;str[i]!='\0';i++)
    {
        if(CHECK_NUM(str[i]))
        {
            res=((res*10)+(str[i]-48));

        }

    }
    return res;
}

uint32_t getValue(uint8_t arg,USER_DATA* dataptr)
{
       if(dataptr->type[arg]=='n')                                        // check if it's number or alpha or delimiter
       {
           return Atoi(&dataptr->strBuffer[dataptr->index[arg]]);         // return the ASCII value to interger value
       }
       else
       {
           return 0;
       }
}

char* getString(uint8_t arg,USER_DATA* dataptr)
{
    return(&dataptr->strBuffer[dataptr->index[arg]]);
}

bool Empty(struct _Ringbuffer* ringptr)
{
    if(ringptr->read_index == ringptr->write_index)                        // checks the buffer is Empty
    {
        return true;
    }
    return false;
}

bool Full(struct _Ringbuffer* ringptr)
{
    if((ringptr->write_index+1)%SIZE_OF_BUFFER == ringptr->read_index)     // checks the buffer is full
    {
        return true;
    }
    return false;
}

void bufferinit(struct _Ringbuffer* ringptr)
{
    ringptr->read_index = SIZE_OF_BUFFER-1;                                // size of readindex = 1023
    ringptr->write_index = SIZE_OF_BUFFER-1;                               // size of readindex = 1023
}


void displayUart0(char* str)
{
    uint16_t length = len_str(str);
    uint16_t i = 0;
    while(!Full(&ringbuffer))                                                 // checks if the buffer is not full
    {
       ringbuffer.write_index = (ringbuffer.write_index+1)%SIZE_OF_BUFFER;
       ringbuffer.strBuffer[ringbuffer.write_index] = str[i];                 // writing the characters to the ringbuffer
       i++;
       if(i>length)
       {
           break;
       }
    }
    if(UART_FR_TXFE)
   {
       if(!Empty(&ringbuffer))                                                // checks is the ringbuffer is not empty
       {
           ringbuffer.read_index=(ringbuffer.read_index+1)%SIZE_OF_BUFFER;    // read the first character stored inringbuffer
           UART0_DR_R = ringbuffer.strBuffer[ringbuffer.read_index];          // store that character in data register of UART0

       }
   }
}

void Uart0Isr()
{
    UART0_ICR_R |= UART_ICR_TXIC;
    if(!Empty(&ringbuffer))
    {
        ringbuffer.read_index=(ringbuffer.read_index+1)%SIZE_OF_BUFFER;        // read the other characters in ringbuffer
        UART0_DR_R = ringbuffer.strBuffer[ringbuffer.read_index];              // store it in data register of UART0
        //ringbuffer.read_index++;
    }
    //clear interrupt flag

}


int main(void)
{

    USER_DATA data;

    flagpoll = false;
    rxflagpoll = false;
    timeflag = false;
    rampflag1 = false;
    timerflag = false;
    patterncount = 0;
    checkpattern = 0;
    delncount = 0;
    rx_phase = 0;
    phase = 0;
    TIMEOUT = 20;

    // Initialize hardware
    initHw();
    initUart0();
    initUart1();
    initeeprom();

    setUart0BaudRate(115200, 40e6);       //baud rate of 115200 for UART0 with 40MHz Frequency

    setUart1BaudRate(250000, 40e6);       //baud rate of 25000 for UART1 with 40MHz Frequency

    // Toggle blue LED every second
    BLUE_LED = 1;
    waitMicrosecond(500000);
    BLUE_LED = 0;
    waitMicrosecond(500000);

    // displaying in putty using displayUart instead of putsuart
    displayUart0("Hello\r\n");
    displayUart0("HI\n");
    displayUart0("no\n");


    mode=readeeprom(0x00);                     //reading the data stored in 0x00 address
    while(true)
    {

        getsUart0(&data);
        parse(&data);
        bool valid = false;

        if(isCommand("controller",0,&data))                 // controller command
        {
            write_eeprom(0x00,0X00);                        // writing the data as 0 in address 0 of eeprom
            mode=0;
            valid = true;
        }

        if(isCommand("on",0,&data))                         // on command
        {
            run=1;
            RED_LED = 1;
            StartDmxTx();
            valid = true;
        }

        if(isCommand("off",0,&data))                        // off command
        {
            run=0;
            RED_LED = 0;
            valid = true;
        }

        if(isCommand("device",1,&data))                    // device mode
        {
            address_1 = getValue(1,&data);
            write_eeprom(0x01,address_1);                  // writing the data as the address_1 in the address 1 of eeprom
            write_eeprom(0x00,1);
            mode=1;
            StartDmxRx();
            initpwmlight();
            valid = true;
        }

        if(isCommand("set",2,&data))                      // set command having 2 extra args
        {
            flagpoll = false;
            uint16_t address = getValue(1,&data);        // first arg address
            uint16_t data1 = getValue(2,&data);          // second arg data1
            datatx[address-1] = data1;                   // sending the data1 to the datatx[address-1]
            StartDmxTx();
            valid = true;
        }

        if(isCommand("clear",0,&data))                   // clear command
        {
            int i;
            for(i=0;i<MAX+1;i++)
            {
            datatx[i] = 0;                              // setting all the data in the buffer as 0
            }
            valid = true;
        }

        if(isCommand("poll",0,&data))
        {
            //GREEN_LED = 1;
            flagpoll = true;

            int i;
            for(i=0;i<513;i++)
            {
                datatx[i]=0;
            }
            pollcount = 0;
            starttimer8us();

            valid = true;
        }

        if(isCommand("ramp",4,&data))                    // ramp command with 4 args
        {
            rampflag1 = true;
            if(patterncount<10)
            {
                patterndata[patterncount].add = getValue(1,&data);              // first arg add
                patterndata[patterncount].value1 = getValue(2,&data);           // second arg value1
                patterndata[patterncount].value2 = getValue(3,&data);           // third arg value2
                patterndata[patterncount].time = getValue(4,&data);             // forth arg time
                patterndata[patterncount].N = patterndata[patterncount].time/0.01; // N interrupts
                patterndata[patterncount].delN=0;                               // deltaN
                patterndata[patterncount].type=ramptype;                        // type to indicate it's in ramp function
                patterncount++;                                                 //increment the patterncount upto 10 so that 10 patterns are created
                if(!timerflag)
                {
                    timerflag = true;
                    starttimer10ms();
                }

            }
            else
            {
                displayUart0("patterncount is already 10\n");                   // if the patterncount is more that 10 then displayuart
                continue;
            }
            valid = true;
        }

        if(isCommand("pulse",4,&data))                                         // pulse command with 4 args
        {

            if(patterncount<10)
            {
                patterndata[patterncount].add = getValue(1,&data);            // first arg add
                patterndata[patterncount].value1 = getValue(2,&data);         // second arg value1
                patterndata[patterncount].value2 = getValue(3,&data);         // third arg value2
                patterndata[patterncount].time = getValue(4,&data);           // forth arg time
                patterndata[patterncount].N = patterndata[patterncount].time/1;// N interrupts
                patterndata[patterncount].delN=0;                              // deltaN
                patterndata[patterncount].type=pulsetype;                      // type to indicate it's in pulse function
                patterncount++;                                                //increment the patterncount upto 10 so that 10 patterns are created
                if(!timerflag)
                {
                    timerflag = true;
                    starttimer10ms();
                }
            }
            else
            {
                displayUart0("patterncount is already 10\n");                 // if the patterncount is more that 10 then displayuart
                continue;
            }
            valid = true;
        }

       if(!valid)
       {
           displayUart0("error\n");                                          // if the commands are not valid then error
       }
    }
}

