/*
 * What is frequency ?
 * "Frequency is number of waves per second."

 * My concept is to start two interrupts at the same time
 * ---> Counter
 * ---> Timer
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