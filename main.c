/*
 * File:   main.c
 * Author: rschaub
 *
 *                   PIC16F18326
 *                   __________________
 *           VDD ---|1               14|--- VSS
 *               ---|2 / RA5   RA0 / 13|---
 *               ---|3 / RA4   RA1 / 12|---
 *               ---|4 / RA3   RA2 / 11|--- potentiometer input
 *     clk out+  ---|5 / RC5   RC0 / 10|--- tap tempo button input
 *     clk out-  ---|6 / RC4   RC1 /  9|--- 
 *               ---|7 / RC3   RC2 /  8|--- "on" light
 *                   ------------------
 * 
 * This program drives a clock for four BBD chips
 *
 * Created on March 22, 2020, 12:48 AM
 */

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with 2x PLL (32MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = OFF      // Clock Switch Enable bit (The NOSC and NDIV bits cannot be changed by user software)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = ON        // Watchdog Timer Enable bits (WDT enabled, SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = HIGH      // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.7V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger enabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

unsigned int adc_result = 0;            //this is where the ADC value will be stored

//variables to handle tap tempo button debouncing
unsigned int DB_INTEGRATOR_MAX = 6;
unsigned int DB_INTEGRATOR_THRESHOLD = 3;
unsigned int db_input = 0;
unsigned int db_integrator = 0;
unsigned int db_output = 0;

void adc_init(void){
    // sets up the ADC
    ADCON0bits.ADON = 0;        //turn ADC off for config
    
    ADCON1bits.ADPREF = 0b00;   //ADC positive reference is set to VDD
    ADCON1bits.ADNREF = 0;      //ADC negative reference is set to VSS
    ADCON0bits.CHS = 0b00010;   //selecting the AN2 analog channel
    ADCON1bits.ADCS = 0b010;    //ADC clock set to FOSC/32 (2us convertion time with Fosc = 32MHz)
    ADCON1bits.ADFM = 1;        //ADC result is right justified

    ADCON0bits.ADON = 1;        //turn ADC on
}

void  NCO1_init(void){
    // sets up the NCO module
    NCO1CONbits.N1EN = 0;       //turn off the NCO for config
    
    NCO1CONbits.N1PFM = 0;      //set the NCO output to a fixed 50% duty cycle
    NCO1CLKbits.N1CKS = 0x00;   //set the NCO clock to the HFINTOSC (16MHz)
    
    // set the NCO increment value to 255 just to start. this should initialize
    // the NCO overflow frequency to 3.891kHz. therefore the real output
    // frequency should be 1.945kHz
    NCO1INCU = 0x0;
    NCO1INCH = 0x00;
    NCO1INCL = 0xFF;
    
    NCO1CONbits.N1EN = 1;       //turn on the NCO
}

void CWG_init(void){
    // set up the complementary waveform generator
    CWG1CON0bits.CWG1EN = 0;        //disable the CWG
    
    CWG1CON0bits.CWG1MODE = 0b100;  //CWG set to half-bridge mode
    CWG1CON1bits.POLA = 0;          //output A is normal
    CWG1CON1bits.POLB = 0;          //output B is inverted
    CWG1DATbits.DAT = 0b1001;       //the NCO is the CWG data input
    
    // shutdown settings
    CWG1AS0bits.CWG1LSAC = 0b01;    //output pins will tri-state in shutdown
    CWG1AS0bits.CWG1LSBD = 0b01;    //output pins will tri-state in shutdown
    CWG1AS0bits.REN = 1;            //auto restart enabled
    CWG1AS1bits.AS0E = 0;           //auto shutdown from PPS disabled
    CWG1AS1bits.AS1E = 0;           //auto shutdown from comp 1 disabled
    CWG1AS1bits.AS2E = 0;           //auto shutdown from comp 2 disabled
    CWG1AS1bits.AS3E = 0;           //auto shutdown from CLC 2 disabled
    CWG1AS1bits.AS4E = 0;           //auto shutdown from CLC 3 disabled
    
    CWG1DBRbits.DBR = 0b000010;     //rising dead band is 1-2 HFINTOSC periods
    CWG1DBFbits.DBF = 0b000010;     //falling dead band is 1-2 HFINTOSC periods
    CWG1CLKCONbits.CS = 1;          //the HFINTOSC drives the dead band time
    CWG1CON0bits.CWG1EN = 1;        //enable the CWG
    
    TRISCbits.TRISC4 = 0;           //set RC4 (pin 6) as an output
    TRISCbits.TRISC5 = 0;           //set RC5 (pin 5) as an output
    
    CWG1AS0bits.SHUTDOWN = 0;       //disable shutdown
}

void PPS_init(void){
    // set up the pin assignments for the various peripherals
    PPSLOCKbits.PPSLOCKED = 0;      //unlock the peripheral control
    
    RC5PPSbits.RC5PPS = 0b01000;    //RC5 is attached to the CWG1A output
    RC4PPSbits.RC4PPS = 0b01001;    //RC4 is attached to the CWG1B output
}

void timer2_init(void){
    // timer 2 is used to schedule the ADC readings. the clock source is
    // Fosc / 4
    T2CONbits.TMR2ON = 0;       //turn off timer 2 for conifg
    PIR1bits.TMR2IF = 0;        //clear the timer 2 match interrupt flag
    
    T2CONbits.T2CKPS = 0b11;    //prescaler set to 1:64
    T2CONbits.T2OUTPS = 0b1111; //postscaler set to 1:16
    PR2 = 194;                  //match register set to give a "round" interrupt freq
    
    // the above sets the timer 2 match interrupt frequency to
    // (Fosc / 4) * prescaler * (1 / (PR2)) * postscaler = interrupt freq.
    // (2MHz) * (1/64) * (1 / (194)) * 1/16 = ~10Hz
    T2CONbits.TMR2ON = 1;       //turn on timer2
}

void timer4_init(void){
    // timer 4 is used to run the tap tempo button debouncing. the clock source
    // is Fosc / 4
    T4CONbits.TMR4ON = 0;       //turn off timer 4 for config
    PIR2bits.TMR4IF = 0;        //clear the timer 4 match interrupt flag
    
    T4CONbits.T4CKPS = 0b10;    //prescaler set to 1:16
    T4CONbits.T4OUTPS = 0b0111; //postscaler set to 1:8
    PR4 = 78;
    
    // the above sets the timer 4 match interrupt frequency to
    // (Fosc / 4) * prescaler * (1 / (PR2)) * postscaler = interrupt freq.
    // (2MHz) * (1/16) * (1 / (78)) * (1/8) = ~200Hz
    T4CONbits.TMR4ON = 1;       //turn on timer 4
}

void tmr2_interrupt_handler(void){
    // if the ADC has completed a conversion, write the value into the "results"
    // register
    if (ADCON0bits.GO_nDONE == 0){
        adc_result = ADRES;
    }
    ADCON0bits.GO_nDONE = 1;               //start ADC
    
    // use the ADC value to drive the NCO frequency by adjusting the increment
    // register (using only the most-significant 10 bits for now)
    NCO1INCU = adc_result >> 8;
    NCO1INCH = adc_result & 0xFF;
    NCO1INCL = 0xFF;
}

void tmr4_interrupt_handler(void){
    // check the status of the tap tempo button, and set the tap tempo positive
    // edge flag appropriately
    
    // run the debounce integrator to filter out mechanical switch bounce
    if (PORTCbits.RC0 == 0){
        if (db_integrator > 0){
            db_integrator--;
        }
    } else if (db_integrator < DB_INTEGRATOR_MAX){
        db_integrator++;
    }
    
    // turn the LED on if the debounce integrator exceeds the threshold
    if (db_integrator >= DB_INTEGRATOR_THRESHOLD){
        LATCbits.LATC2 = 1;        
    } else {
        LATCbits.LATC2 = 0;
    }
        
}

void main(void) {
    // configure the internal clock to run at 16MHz. this means that the
    // instruction clock will run at 2MHz (Fosc/4)
    OSCCON1bits.NOSC = 0b000;   //set the "new osc" source to HFINTC w/ 2x PLL
    OSCFRQbits.HFFRQ = 0b0100;  //set the HFINTOSC to 8MHz

    // configure the watchdog timer
    WDTCONbits.WDTPS = 0b01011; //set to 2s timer
    
    // configure the inputs and outputs
    TRISAbits.TRISA2 = 1;       //set RA2 (pin 11) as input (analog input for "speed" pot)
    TRISCbits.TRISC0 = 1;       //set RC0 (pin 10) as input (for tap tempo button)
    TRISCbits.TRISC1 = 0;       //set RC1 (pin 9) as output
    TRISCbits.TRISC2 = 0;       //set RC2 (pin 8) as output
    TRISCbits.TRISC3 = 0;       //set RC3 (pin 7) as an output
    TRISCbits.TRISC4 = 0;       //set RC4 (pin 6) as an output
    TRISCbits.TRISC5 = 0;       //set RC5 (pin 5) as an output
    ANSELA = 0b00000100;        //set RA2 (pin 11) as an analog input (AN2 channel))
    ANSELC = 0b00000000;        //nothing on port C is an analog input
    
    // configure the internal peripherals
    adc_init();               //configure the ADC
    NCO1_init();              //configure the NCO
    CWG_init();               //configure the complementary waveform generator
    PPS_init();               //assign peripherals to pins
    timer2_init();            //turn on timer2
    timer4_init();            //turn on timer4
    
     // turn on interrupts
    PIE1bits.TMR2IE = 1;      //enable timer 2 to PR2 match interrupt
    PIE2bits.TMR4IE =  1;     //enable timer 4 to PR2 match interrupt
    INTCONbits.PEIE = 1;      //enable peripheral interrupts
    INTCONbits.GIE = 1;       //general interrupts enabled
    
    LATC = 0x00;              //init the port c latches
    LATCbits.LATC2 = 1;       //just to tell the user that the program started        
        
    while(1){
        CLRWDT();               //clear the Watchdog Timer to keep the PIC from
                                //resetting
    }
    return;
}

void interrupt ISR(void){
    // check for timer 2 to PR2 match interrupt
    if(PIR1bits.TMR2IF == 1){
        tmr2_interrupt_handler();
        PIR1bits.TMR2IF = 0;                 //reset the interrupt flag
    }
    
    // check fir time 4 to PR4 match interrupt
    if(PIR2bits.TMR4IF == 1){
        tmr4_interrupt_handler();
        PIR2bits.TMR4IF = 0;                 //reset the interrupt flag
    }
    
   return;
}
