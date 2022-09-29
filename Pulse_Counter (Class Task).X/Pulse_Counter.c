#include <pic18f452.h>
#include <xc.h>

void Delay(unsigned int);
void cnvrsion2dec();
void cnvrsion2svn(unsigned int);
void Display();

#define mybit PORTAbits.RA4
unsigned int T1_ovrflo, total_time, upr_three_dig;
unsigned int x, d0, d1, d2, d3, temp = 0;
unsigned char time_L, time_H;

int main() {
    TRISAbits.TRISA4 = 1;
    TRISB = TRISD = 0;
    T0CON = 0x68;
    T1CON = 0x00;

    while (1) {
        TMR1H = 0x02;
        TMR1L = 0x00;
        PIR1bits.TMR1IF = 0;
        INTCONbits.TMR0IF = 0;
        TMR0L = 0xFE;
        while (TMR0L == 0xFE)
        {
//            TMR0L=0xFF;
        }
        T1CONbits.TMR1ON = 1;
        while (INTCONbits.TMR0IF == 0) {
            if (PIR1bits.TMR1IF == 1) {
                T1_ovrflo++;
                PIR1bits.TMR1IF = 0;
            }
//            INTCONbits.TMR0IF=1;
        }
    
    T1CONbits.TMR1ON = 0;
    time_L = TMR1L;
    time_H = TMR1H;
    INTCONbits.TMR0IF = 0;
    cnvrsion2dec();
    Display();
    }
    return 0;
}

void cnvrsion2dec() {
    total_time = 256 * time_H;
    total_time = total_time + time_L;
    upr_three_dig = total_time / 10;
    
    d0=(total_time/10)%10;
    cnvrsion2svn(d0);
    d0=temp;
    
    x = upr_three_dig / 10;
    
    d1 = upr_three_dig % 10;
    cnvrsion2svn(d1);
    d1 = temp;
    d2 = x % 10;
    cnvrsion2svn(d2);
    d2 = temp;
    d3 = x / 10;
    cnvrsion2svn(d3);
    d3 = temp;
}

void cnvrsion2svn(unsigned int T) {
    switch (T) {
        case 0:
            temp = 0x3F;
            break;
        case 1:
            temp = 0x06;
            break;
        case 2:
            temp = 0x5B;
            break;
        case 3:
            temp = 0x4F;
            break;
        case 4:
            temp = 0x66;
            break;
        case 5:
            temp = 0x6D;
            break;
        case 6:
            temp = 0x7D;
            break;
        case 7:
            temp = 0x07;
            break;
        case 8:
            temp = 0x7F;
            break;
        case 9:
            temp = 0x6F;
            break;
    }
}

void Display() {
    PORTB = PORTD = 0;
    
    PORTB = d0;
    PORTDbits.RD3 = 1;
    Delay(10);
    PORTDbits.RD3 = 0;

    PORTB = d1;
    PORTDbits.RD2 = 1;
    Delay(10);
    PORTDbits.RD2 = 0;

    PORTB = d2;
    PORTDbits.RD1 = 1;
    Delay(10);
    PORTDbits.RD1 = 0;

    PORTB = d3;
    PORTDbits.RD0 = 1;
    Delay(10);
    PORTDbits.RD2 = 0;
}

void Delay(unsigned int itime) {
    unsigned int i;
    unsigned char j;
    for (i = 0; i < itime; i++)
        for (j = 0; j < 165; j++) {
        }
}