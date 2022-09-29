# PIC-Frequency-Meter
Project includes PIC18F452 C code for Frequency Meter. The frequency meter employs multiple seven segment displays and uses interrupts and timers to find the freqeuency of signals

## Objectives
* To understand about PIC Timers and how to use them.
* To understand what are interrupts and how to use them.
* To write a code for frequency measurement of a pulse and display it on multiplexed 7-segment displays.
* To simulate the circuit on the proteus.
## Theory
### Timers
In PIC Timers are used to generate time delays in programs or to count an external event on application of high-low or low-high signal depending upon the mode of timer used. There are five timers available in PIC18 these are,
* Timer 0
* Timer 1
* Timer 2
* Timer 3
* Timer 4
These timers are used in different cases differently in different modes depending upon the application of PIC. Timers can be used to generate time delay by using PIC’s internal clock signal and to count some external events while using some external clock source like a pulse.
There are several registers associated with a timer and these registers are used to program the timers for a given application. These registers include the mode of timer in which it will work, the clock source (internal or external) and timer on and off. All of the registers associated with timers are 8-bit registers. Following text shows how to program timer 0 and timer 1.
#### TIMER 0 PROGRAMMING
Timer 0 can be used as an 8-bit or 16-bit timer, the 16-bit timer register is accessed as Lower and Higher bytes. These are TMR0H and TMR0L and used to store the values up to which the timer will count. Another timer control register is associated with timer 0 which is T0CON register which is an 8-bit register which is used to program the timer for its modes, for the use of clock and all the things needed.
Figure 5.1: T0CON Register’s Bits
Another bit is TMR0IF which is associated with timer 0 and it belongs to register INTCON register, the bit raises HIGH when timer 0 overflows. In 16-bit mode timer overflows if TMR0 registers contains (216 – 1) value and in case of 8-bits it overflows if TMR0L contains (28 – 1) value in it. The steps to program timer 0 are explained below: -
* Timer control register T0CON for this is loaded with specific values as shown in above table. But keeping the TMR0ON a LOW
* The timer value is loaded in the TMR0H and TMR0L the value can be 0x0000 to 0xFFFF in case of 16-bit mode and 0x00 to 0xFF in 8-bit mode.
* The timer interrupt control bit is cleared by clearing the bit TMR0IF so if it overflowed it should come to normal state.
* The timer is started by switching the T0CON’s bit TMR0ON.
* The timer has started and will start the counting or produce the delay depending upon the value of T0CON’s T0CS bit.
#### TIMER 1 PROGRAMMING
Timer 1 can be used as 16-bit timer only. These are TMR1H and TMR1L and used to store the values up to which the timer will count. Another timer control register is associated with timer 1 which is T1CON register which is an 8-bit register which is used to program the timer for its modes, for the use of clock and all the things needed.
 
Figure 5.2: T1CON Register’s Bits
Another bit is TMR1IF which is associated with timer 1 and it belongs to register PIR1 register, the bit raises HIGH when timer 0 overflows. In 16-bit mode timer overflows if TMR0 registers contains (216 – 1) value. Steps to program timer 1 are explained below: -
* Timer control register T0CON for this is loaded with specific values as shown in above table. But keeping the TMR0ON a LOW
* The timer value is loaded in the TMR0H and TMR0L the value can be 0x0000 to 0xFFFF.
* The timer interrupt control bit is cleared by clearing the bit TMR1IF so if it overflowed it should come to normal state.
* The timer is started by switching the T1CON’s bit TMR1ON.
* The timer has started and will start the counting or produce the delay depending upon the value of T1CON’s TMR1CS bit.
 
### Interrupt
There are two methods to write a program normally one is polling and other is interrupts. Both methods are different from each other. To understand this one should consider an example.
Let a letter is coming for you, to check if the latter has arrived you check your letter box every two minutes, in those two minutes if the latter have not come you are wasting your time. You can do something meaning full in that time. This is polling that the checking of condition again and again to do a specific task.
Now let’s assume the condition explained above but this timer you are not checking the box every two minutes but you have created such a system that when the letter came you will get notified let say an alarm, then you can do something meaning full in those two minutes and when letter came it will cause to ring the alarm and you can come to receive it. This method is called interrupt method. It means when a condition becomes true the task is done with the other tasks are also being executed. A very nice way to do something.
There are some internal and external interrupts on PIC, here our topic is internal interrupts. These interrupts are associated with timers and we will use timer 0 ad timer 1 to count the pulses in one second which is actually frequency of the applied signal.
#### INTERRUPT SERVICE ROUTINE (ISR)
ISR (Interrupt Service Routine) is a sub program or a function or sub-routine whatever it is called is a combination of codes that will be executed when an interrupt cause. This is a normal code just to ensure it is attached with interrupt it is named as ISR or Interrupt Service Routine but one can use different name in coding it will cause no change in final output.
## Code
/*
 * What is frequency?
 * "Frequency is number of waves per second."
 *
 * My concept is to start two interrupts at the same time
 * ---> Counter
 * ---> Timer
 *
 * When a pulse is given to the Timer0 external clock input pin.
 * A one second timer will start at timer_1.
 * At the same time timer0 will count the number of pulses.
 * At the end of interrupt at timer1 we will have number of waves in on second.

 * Which was required.
 */

// PIC18F452 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = XT  // Oscillator Selection bits (XT oscillator)
#pragma config OSCS = ON // Oscillator System Clock Switch Enable bit (Oscillator system clock switch option is enabled (oscillator switching is enabled))

// CONFIG2L
#pragma config PWRT = OFF // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = OFF  // Brown-out Reset Enable bit (Brown-out Reset disabled)
#pragma config BORV = 20  // Brown-out Reset Voltage bits (VBOR set to 2.0V)

// CONFIG2H
#pragma config WDT = OFF   // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 128 // Watchdog Timer Postscale Select bits (1:128)

// CONFIG3H
#pragma config CCP2MUX = ON // CCP2 Mux bit (CCP2 input/output is multiplexed with RC1)

// CONFIG4L
#pragma config STVR = ON // Stack Full/Underflow Reset Enable bit (Stack Full/Underflow will cause RESET)
#pragma config LVP = ON  // Low Voltage ICSP Enable bit (Low Voltage ICSP enabled)

// CONFIG5L
#pragma config CP0 = OFF // Code Protection bit (Block 0 (000200-001FFFh) not code protected)
#pragma config CP1 = OFF // Code Protection bit (Block 1 (002000-003FFFh) not code protected)
#pragma config CP2 = OFF // Code Protection bit (Block 2 (004000-005FFFh) not code protected)
#pragma config CP3 = OFF // Code Protection bit (Block 3 (006000-007FFFh) not code protected)

// CONFIG5H
#pragma config CPB = OFF // Boot Block Code Protection bit (Boot Block (000000-0001FFh) not code protected)
#pragma config CPD = OFF // Data EEPROM Code Protection bit (Data EEPROM not code protected)

// CONFIG6L
#pragma config WRT0 = OFF // Write Protection bit (Block 0 (000200-001FFFh) not write protected)
#pragma config WRT1 = OFF // Write Protection bit (Block 1 (002000-003FFFh) not write protected)
#pragma config WRT2 = OFF // Write Protection bit (Block 2 (004000-005FFFh) not write protected)
#pragma config WRT3 = OFF // Write Protection bit (Block 3 (006000-007FFFh) not write protected)

// CONFIG6H
#pragma config WRTC = OFF // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write protected)
#pragma config WRTB = OFF // Boot Block Write Protection bit (Boot Block (000000-0001FFh) not write protected)
#pragma config WRTD = OFF // Data EEPROM Write Protection bit (Data EEPROM not write protected)

// CONFIG7L
#pragma config EBTR0 = OFF // Table Read Protection bit (Block 0 (000200-001FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR1 = OFF // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR2 = OFF // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from Table Reads executed in other blocks)
#pragma config EBTR3 = OFF // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from Table Reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF // Boot Block Table Read Protection bit (Boot Block (000000-0001FFh) not protected from Table Reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <pic18f452.h>

// Declarations
void T0_isr();                      // Timer0 Interrupt Service Routine > Count number of waves
void T1_isr();                      // Timer1 Interrupt Service Routine > 1 sec timer
void num2dig(void);                 // Convert Number to Digits
unsigned int dig2hex(unsigned int); // Convert Digits to Hexadecimal Code
void display(void);                 // Drive Displays using concept of multiplexing
void seg_Delay(void);               // Delay for Multiplexing (120 usec)
void my_Delay();

unsigned int ovrCount;   // Stores how many times pulse counter overflows
unsigned int timerCount; // Stores number of timer cycles

unsigned int dig[8];   // Stores 8 digits for 8 displays
unsigned char code[8]; // Stores hex code for displays
unsigned long freq;    // Stores measured frequency

/*
 * Following function checks which interrupt is being invoked by controller
 * ---> Counter
 * ---> Timer
 * 
 * Note: There is a reserved keyword __interrupt() which is used for interrupts only.
 */

void __interrupt() chk_int()
{
    if (INTCONbits.TMR0IF == 1)
        T0_isr();
    if (PIR1bits.TMR1IF == 1)
        T1_isr();
}

int main(void)
{
    unsigned short long L_Byte, H_Byte;
    TRISB = TRISD = 0; // Setting ports as Output
    PORTB = PORTD = 0; // Clearing ports as Output
    TRISAbits.RA4 = 1; // Setting PortA (Pin4) as input > Timer0 External Clock

    T0CON = 0x28;          // Loading timer0 control register , Timer off, 16-bit, External Clock, No pre-scale
    T1CON = 0x30;          // Loading timer1 control register , 16-bit, 1:8 pre-scale, Internal Clock, Timer off
    INTCONbits.TMR0IF = 0; // Clearing timer0 flag-bit
    PIR1bits.TMR1IF = 0;   // Clearing timer1 flag-bit
    INTCONbits.TMR0IE = 1; // Setting timer0 Interrupt Enabler-bit
    PIE1bits.TMR1IE = 1;   // Setting timer1 Interrupt Enabler-bit
    INTCONbits.PEIE = 1;   // On Peripheral Interrupts
    INTCONbits.GIE = 1;    // On Global interrupts

    while (1)
    { // Forever loop
        freq = 0;
        TMR0L = 0x00; // Loading timer0 lower byte
        TMR0H = 0x00; // Loading timer0 higher byte
        TMR1H = 0x0B; // Loading timer1 lower byte
        TMR1L = 0xDC; // Loading timer1 higher byte

        // timer1 is loaded for 200 msec delay > 200 x 5 = 1000 msec (milli seconds))

        ovrCount = 0;
        timerCount = 0;

        T1CONbits.TMR1ON = 1; // timer1 On > Start 1 sec timer
        T0CONbits.TMR0ON = 1; // timer0 On > Start pulse counting

        while (timerCount <= 4)
            ; // repeat timer 5 times

        T0CONbits.TMR0ON = 0; // timer0 Off
        T1CONbits.TMR1ON = 0; // timer1 Off

        L_Byte = TMR0L;
        H_Byte = TMR0H;

        freq = (unsigned long)((256 * H_Byte) + L_Byte); // Converting Lower and Higher bytes to a single value
        freq = (65536 * ovrCount) + freq;                // Calculating total frequency

        // 65536 * ovrCount shows how many time counter overflows

        // Successfully calculated frequency below steps to display the frequncy

        num2dig(); // Separating digits of calculated frequency
        for (int i = 0; i < 8; i++)
            code[i] = dig2hex(dig[i]); // Converting each digit to hexadecimal code

        while (1)
        {
            display(); // calling display forever
        }

        my_Delay();
    }
    return 0;
}

// Invokes if interrupt_0 invokes < for pulse counting

void T0_isr()
{
    if (INTCONbits.TMR0IF == 1)
    {
        TMR0L = 0;
        TMR0H = 0;
        ovrCount++;
        INTCONbits.TMR0IF == 0;
    }
}

// Invoked if interrupt_1 invokes > for 1 sec delay

void T1_isr()
{
    if (PIR1bits.TMR1IF = 1)
    {
        TMR1H = 0x0B;
        TMR1L = 0xDC;
        timerCount++;
        PIR1bits.TMR1IF = 0;
    }
}

void num2dig()
{
    unsigned int i;
    while (freq > 0)
    { // Separate digits until frequency becomes zero
        dig[i] = freq % 10;
        freq = freq / 10;
        i++;
    }
}

// Returns corresponding Hexadecimal code of digits from 0-9

unsigned int dig2hex(unsigned int targetDig)
{
    switch (targetDig)
    {
    case 0:
        return 0x3F;
        break;
    case 1:
        return 0x06;
        break;
    case 2:
        return 0x5B;
        break;
    case 3:
        return 0x4F;
        break;
    case 4:
        return 0x66;
        break;
    case 5:
        return 0x6D;
        break;
    case 6:
        return 0x7D;
        break;
    case 7:
        return 0x07;
        break;
    case 8:
        return 0x7F;
        break;
    case 9:
        return 0x6F;
        break;
    }
    return 0;
}

// Display Function

void display()
{
    PORTB = code[0];
    PORTDbits.RD0 = 1;
    seg_Delay();
    PORTDbits.RD0 = 0;

    PORTB = code[1];
    PORTDbits.RD1 = 1;
    seg_Delay();
    PORTDbits.RD1 = 0;

    PORTB = code[2];
    PORTDbits.RD2 = 1;
    seg_Delay();
    PORTDbits.RD2 = 0;

    PORTB = code[3];
    PORTDbits.RD3 = 1;
    seg_Delay();
    PORTDbits.RD3 = 0;

    PORTB = code[4];
    PORTDbits.RD4 = 1;
    seg_Delay();
    PORTDbits.RD4 = 0;

    PORTB = code[5];
    PORTDbits.RD5 = 1;
    seg_Delay();
    PORTDbits.RD5 = 0;

    PORTB = code[6];
    PORTDbits.RD6 = 1;
    seg_Delay();
    PORTDbits.RD6 = 0;

    PORTB = code[7];
    PORTDbits.RD7 = 1;
    seg_Delay();
    PORTDbits.RD7 = 0;
}

// 120 usec delay

void seg_Delay()
{
    unsigned char time_1;
    unsigned char time_2;
    for (time_1 = 0; time_1 < 150; time_1++)
        for (time_2 = 0; time_2 < 2; time_2++)
        {
        }
}

void my_Delay()
{
    unsigned char i;
    unsigned char j;
    unsigned char k;
    for (i = 0; i < 10; i++)
        for (j = 0; j < 238; j++)
            for (k = 0; k < 250; k++)
            {
            }
}