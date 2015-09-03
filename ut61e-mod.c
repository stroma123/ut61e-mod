/* This code can handle several different modes.
 * Short DC-AC-button -> Default function
 * Long DC-AC-button -> Enables the MAX/MIN feature
 * Short V-Hz-button -> Default function
 * Long V-Hz-button -> LCD background light ON/OFF
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
#define DCAC            2   // INPUT
#define VHZ             4   // INPUT
#define BKOUT           5   // INPUT This need a voltage divider (BKOUT(VC+))-100k+100k-(VB_)
#define LPF             0   // OUTPUT
#define SDACDC          1   // OUTPUT
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

    // Check buttons at start-up
    if (0==(PORTA&(1<<VHZ))) {
      T1CONbits.TMR1ON = 1;
      __delay_ms(40);
      while( 0==(PORTA&(1<<VHZ)) && 0==TMR1IF );
      if (1==TMR1IF) {
        PORTA &= ~(1<<LPF); // Enable Low Pass Filter for better AC measures
      }
      __delay_ms(50);
      PORTA |= ((1<<LPF));

      T1CONbits.TMR1ON = 0;
      TMR1L = 0x00;
      TMR1H = 0x00;
      TMR1IF = 0;

      while(1) {
        SLEEP();
        NOP();
      }
    }



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
// By mirroring the port, the ES51922A port is not only protected, but the
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
    PORTA = (1<<DCAC | 1<<VHZ);
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

