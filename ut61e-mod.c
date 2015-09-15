/* 
 * Software written for PIC16F688.
 * The PIC can be installed in a DMM UT61E to enable
 * unused features on the ES51922 by introducing more
 * options to the function buttons at the front of the DMM.
 *
 * Added features:
 * -Long push DC-AC-button -> Enables the MAX/MIN feature
 * -Long push V-Hz-button -> LCD background light ON/OFF (60s)
 * -Both DCAC-button and VHz-button pushed -> Toggle RS232 ON/OFF
 * -Pushing and holding down the DC-AC button for 1/2 second
 * during power-on, enables the Low Pass Filter AC mode.
 * The filter is by default turned off for patent infringement
 * reasons.
 *
 * Notes and tweaking the tweak:
 * -The background light on-time can be changed from 60s to
 *  180s by connecting BKSEL (pin 113) to VB_ (-3V).
 * -When the low pass filter mode is enabled, only AC Volt
 *  and AC current can be measured.
 * -RS232 is turned off by default.
 * -Auto Power Off (APO) is enabled by default. It can be
 *  disabled by holding down any of the push functions at
 *  Power-on.
 *  The DMM will APO after 15 minutes. APO time can be changed
 *  from 15 minutes to 30 minutes by connecting APOSEL (pin 112)
 *  to VB_ (-3V).
 */


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
#define DCAC            2   // INPUT Hooks in to the Blue button
#define VHZ             4   // INPUT Hooks in to the Yellow button
#define BKOUT           5   // INPUT This signal is taken from a voltage divider (BKOUT(VC+))-100k+100k-(VB_)
#define LPF             0   // OUTPUT to FC5 Pin 118
#define SLACDC          1   // OUTPUT to SLACDC Pin 117
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

// Check Yellow button at startup
    // Yellow button trigger LPF-AC mode
    if (0==(PORTA&(1<<VHZ))) {
        T1CONbits.TMR1ON = 1;
        __delay_ms(40);
        while( 0==(PORTA&(1<<VHZ)) && 0==TMR1IF );
        if (1==TMR1IF) {
            PORTA &= ~(1<<LPF); // Enable Low Pass Filter for better AC measures
            PORTA |= ((1<<SLACDC));

            T1CONbits.TMR1ON = 0;
            TMR1IF = 0;

            while(1) {
                SLEEP();
                NOP();

                __delay_ms(40);
                if (0==(PORTA&(1<<DCAC))) {
                    while( 0==(PORTA&(1<<DCAC)) );
                    PORTC &= ~(1<<B_PIN);
                    __delay_ms(50);
                    PORTC |= (1<<B_PIN);
                }
            }
        }
        // We get here if VHZ was not pressed long enough at start-up
        T1CONbits.TMR1ON = 0;
        TMR1L = 0x00;
        TMR1H = 0x00;
        TMR1IF = 0;
    }

// We get here only if LPF-AC mode is not selected
    while(1) {
        SLEEP();
        NOP();

        T1CONbits.TMR1ON = 1;
        __delay_ms(40);

        switch(PORTA & (1<<DCAC | 1<<VHZ)) {
            case 16:  // DC/AC button was pressed
                while( 0==(PORTA&(1<<DCAC)) && 0==TMR1IF );
                if (1==TMR1IF) {
                    PORTC &= ~(1<<MAXMIN);
                } else {
                    PORTC &= ~(1<<B_PIN);
                }
                __delay_ms(50);
                PORTC |= ((1<<MAXMIN) | (1<<B_PIN));
                break;
            case 4:  // V/Hz button was pressed
                while ( 0==(PORTA&(1<<VHZ)) && 0==TMR1IF );
                if (1==TMR1IF) {
                    PORTC &= ~(1<<BKLIT);
                } else {
                    PORTC &= ~(1<<D_PIN);
                }
                __delay_ms(50);
                PORTC |= ((1<<BKLIT) | (1<<D_PIN));
                break;
            case 0:  // BOTH buttons was pressed
                while( 0==(PORTA&(1<<DCAC | 1<<VHZ)) && 0==TMR1IF );
                if (1==TMR1IF) {
                    // Add triggering another feature here if needed.
                } else {
                    PORTC ^= (1<<RS232);
                }
                break;
            default:
                break;
        }

// Mirror BKOUT pin on ES51922A to BKLED pin on PIC to drive LEDs from PIC.
// By mirroring the port, the ES51922 port is not only protected, but the
// LED auto-off feature in the ES51922 is kept.
        if (1==PORTAbits.RA5) {
            PORTC |= (1<<BKLED);
        } else {
            PORTC &= ~(1<<BKLED);
        }

        T1CONbits.TMR1ON = 0;
        TMR1L = 0x00;
        TMR1H = 0x00;
        TMR1IF = 0;

        __delay_ms(200);

        RAIF = 0;
    }
}



void setupRegisters() {
    ANSEL = 0x00;   // Make I/O-pins digital
// Setup of I/O-pins PORTA and PORTC
    PORTC = (1<<B_PIN | 1<<MAXMIN | 1<<BKLIT | 1<<D_PIN | 1<<RS232);
    TRISC = 0x00;
    PORTA = (1<<DCAC | 1<<VHZ | 1<< LPF);
    TRISA = (1<<DCAC | 1<<VHZ | 1<<BKOUT);
    WPUA = (1<<DCAC | 1<<VHZ); // Enable weak pullups
    IOCA = (1<<DCAC | 1<<VHZ | 1<<BKOUT);
// Setup the timer used to detect long button press
    TMR1L = 0x00;
    TMR1H = 0x00;
    T1CON = 0x30; // The prescaler is set to 8 to be able to time 0.52s
    TMR1IF = 0;
// Setup of the interrupt registers to wake up PIC at button activity
// Since the Global Interrupt Enable is not set, the execution will
// proceed immediately after the SLEEP instruction.
    RAIF = 0;
    RAIE = 1;
// Setup the oscillator
    OPTION_REG = 0x00;
    CMCON0 = 0x07;  // Disable comparator to save power
}

