/*
 * File:   main.c
 * Author: rschaub
 *
 *                   PIC16F1825
 *                   __________________
 *           VDD ---|1               14|--- VSS
 *               ---|2 / RA5   RA0 / 13|---
 *               ---|3 / RA4   RA1 / 12|---
 *               ---|4 / RA3   RA2 / 11|--- potentiometer "input"
 *       clk out ---|5 / RC5   RC0 / 10|--- 
 *               ---|6 / RC4   RC1 /  9|--- 
 *               ---|7 / RC3   RC2 /  8|--- "on" light
 *                   ------------------
 * 
 * This program drives a clock for four BBD chips
 *
 * Created on March 22, 2020, 12:48 AM
 */

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL Enable (4x PLL disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (Low-voltage programming disabled)

#include <xc.h>

unsigned int adc_result = 0;    //this is where the ADC value will be stored
int tmp = 0;

void adc_init(void){
    // sets up the ADC
    ADCON0bits.ADON = 0;        //turn ADC off for config
    
    ADCON1bits.ADPREF = 0b00;   //ADC positive reference is set to VDD
    ADCON1bits.ADNREF = 0;      //ADC negative reference is set to VSS
    ADCON0bits.CHS = 0b00010;   //selecting the AN2 analog channel
    ADCON1bits.ADCS = 0b100;    //ADC clock set to FOSC/4
    ADCON1bits.ADFM = 1;        //ADC result is right justified

    ADCON0bits.ADON = 1;        //turn ADC on
}

void timer1_init(void){
    // timer 1 is used to drive the compare module
    T1CONbits.TMR1ON = 0;       //turn off timer1 for config
    PIR1bits.TMR1IF = 0;        //clear the timer1 overflow interrupt flag
    
    T1CONbits.T1CKPS = 0b00;    //set timer1's prescaler (set to 1:1)
    T1CONbits.TMR1CS = 0b00;    //set timer1's clock source (set to Fosc/4)
    T1CONbits.T1OSCEN = 0;      //dedicated timer1 oscillator disabled
    
    // the above sets timer1 overflow interrupts frequency to
    // (Fosc / 4) * prescaler * (1/ (2^16))) = 61.04Hz if Fosc = 16MHz
    T1CONbits.TMR1ON = 1;       //turn on timer1
}

void compare1_init(void){
    // the compare1 module is used to define the output clock rate by toggling
    // an output pin based on timer1's state
    CCP1CONbits.CCP1M = 0b0010; //CCP1 set to compare mode (output pin toggles)
    CCPR1 = 0xFFFF;             //compare reg set to max as default (~30Hz output)
}

void tmr1_interrupt_handler(void){
    //read the potentiometer
    adc_result = ADC_Convert();
}

int ADC_Convert(void){
    ADCON0bits.GO_nDONE = 1;               //start ADC
    while (ADCON0bits.GO_nDONE == 1);      //wait for ADC to finish
    return (ADRESL + (ADRESH * 256));      //return full 10-bit number
}

void main(void) {
    // configure the internal clock to run at 16MHz. this means that the
    // instruction clock will run at 4MHz (Fosc/4)
    OSCCONbits.SPLLEN = 0b0;    //4xPLL disabled (also disable by config word)
    OSCCONbits.IRCF = 0b1111;   //HFINTOSC set to 16MHz
    OSCCONbits.SCS = 0b00;      //clock source set by FOSC config word

    // configure the watchdog timer
    WDTCONbits.WDTPS = 0b01011; //set to 2s timer
    
    // configure the inputs and outputs
    TRISAbits.TRISA2 = 1;       //set RA2 (pin 11) as input (for analog input)
    TRISCbits.TRISC0 = 0;       //set RC0 (pin 10) as output
    TRISCbits.TRISC1 = 0;       //set RC1 (pin 9) as output
    TRISCbits.TRISC2 = 0;       //set RC2 (pin 8) as output
    TRISCbits.TRISC3 = 0;       //set RC3 (pin 7) as an output
    TRISCbits.TRISC4 = 0;       //set RC4 (pin 6) as an output
    TRISCbits.TRISC5 = 0;       //set RC5 (pin 5) as an output
    ANSELA = 0b00000100;        //set RA2 (pin 11) as an analog input (AN2 channel))
    ANSELC = 0b00000000;        //nothing on port C is an analog input
    
    // turn on interrupts
    PIE1bits.TMR1IE = 0;      //disable timer1 overflow interrupt
    PIE1bits.CCP1IE = 1;      //enable compare1 match interrupt
    INTCONbits.PEIE = 1;      //enable peripheral interrupts
    INTCONbits.GIE = 1;       //general interrupts enabled
    
    // configure the timers, the adc, and the compare peripheral
    adc_init();
    timer1_init();
    compare1_init();
    
    PORTCbits.RC2 = 1;          //just to tell the user that the program started
        
    while(1){
        adc_result = ADC_Convert();
        CCPR1 = (adc_result >> 1) + 1;
        CLRWDT();               //clear the Watchdog Timer to keep the PIC from
                                //resetting
    }
    return;
}

void interrupt ISR(void){
    // check for compare1 match interrupt
    if(PIR1bits.CCP1IF == 1){
        TMR1 = 0;                           //reset timer1
        PIR1bits.CCP1IF = 0;                //reset the interrupt flag
    }
    
    // check for timer1 overflow interrupt (this interrupt shouldn't happen)
    if(PIR1bits.TMR1IF == 1){
        PIR1bits.TMR1IF = 0;                 //reset the interrupt flag
    }
    
   return;
}
