#pragma config FOSC = INTOSCIO
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF

#define _XTAL_FREQ 4000000

#include <xc.h>
 

// Register defines
// PORTA RA-registers
#define BLUE_BUTTON     2   // INPUT
#define YELLOW_BUTTON   4   // INPUT
#define BKOUT           5   // INPUT
// PORTC RC-registers
#define B_PIN           0   // OUTPUT
#define MAXMIN          1   // OUTPUT
#define BKLIT           2   // OUTPUT
#define D_PIN           3   // OUTPUT
#define RS232           4   // OUTPUT
#define BKLED           5   // OUTPUT


void setupRegisters(void);


void main() {
    setupRegisters();

    while(1) {
        SLEEP();
        NOP();

        T1CONbits.TMR1ON = 1;
        __delay_ms(40);

        switch(PORTA & (1<<BLUE_BUTTON | 1<<YELLOW_BUTTON)) {
            case 16:  //BLUE button pressed
                while(0==(PORTA&(1<<BLUE_BUTTON)));
                if (1==TMR1IF) {
                    PORTC &= ~(1<<MAXMIN);
                } else {
                    PORTC &= ~(1<<B_PIN);
                }
                __delay_ms(50);
                PORTC |= ((1<<MAXMIN) | (1<<B_PIN));
                break;
            case 4:  // YELLOW button pressed
                while (0==(PORTA&(1<<YELLOW_BUTTON)));
                if (1==TMR1IF) {
                    PORTC &= ~(1<<BKLIT);
                } else {
                    PORTC &= ~(1<<D_PIN);
                }
                __delay_ms(50);
                PORTC |= ((1<<BKLIT) | (1<<D_PIN));
                break;
            case 0:  // BOTH buttons pressed
                PORTC ^= (1<<RS232);
                break;
            default:
                break;
        }

// Mirror BKOUT pin on ES51922A to BKLED pin on PIC to drive LEDS from PIC
// By mirroring the port I can protect output on ES51922A
        if (1==PORTAbits.RA5) {
            PORTC |= (1<<BKLED);
        } else {
            PORTC &= ~(1<<BKLED);
        }

        T1CONbits.TMR1ON = 0;
        TMR1L = 0;
        TMR1H = 0;
        TMR1IF = 0;

        __delay_ms(200);

        RAIF = 0;
    }
}



void setupRegisters() {
    ANSEL = 0x00;   // Digital I/O
// Setup of I/O-pins
    PORTC = (1<<B_PIN | 1<<MAXMIN | 1<<BKLIT | 1<<D_PIN);
    TRISC = 0x00;
    PORTA = (1<<BLUE_BUTTON | 1<<YELLOW_BUTTON);
    WPUA = (1<<BLUE_BUTTON | 1<<YELLOW_BUTTON); // Enable weak pullups
    IOCA = (1<<BLUE_BUTTON | 1<<YELLOW_BUTTON | 1<<BKOUT);
// Setup the timer used to detect long button press
    TMR1L = 0x00;
    TMR1H = 0x00;
    T1CON = 0x30; // Prescaler 8 to be able to time 0.52s
    TMR1IF = 0;
// Setup of the interrupt registers to wake up PIC at button activity
    RAIF = 0;
    RAIE = 1;
// Setup oscillator
    OPTION_REG = 0x00;
    CMCON0 = 0x07;  // Disable comparator to save power
}

