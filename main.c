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
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT enabled, SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = HIGH      // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.7V)
#pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = ON       // Debugger enable bit (Background debugger enabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (High Voltage on MCLR/VPP must be used for programming.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>

// variables for the ADC
volatile bit adc_timing_flag = 0;       //signal to tell the program to run the ADC routine
unsigned int current_adc_result = 0;    //this is where the ADC value will be stored
unsigned int old_adc_result = 0;       //the last ADC reading

// variables for the tap tempo delay time calculation
volatile bit calc_tap_tempo_flag = 0;   //sign that we need to compute tempo
volatile bit tap_tempo_mode = 0;        //are we in tap tempo mode?
unsigned int last_tap_time = 0;
unsigned int current_tap_time = 0;
unsigned int avg_delay_time = 0;

unsigned short long NCO_increment = 0;


void adc_init(void){
    // sets up the ADC
    ADCON0bits.ADON = 0;        //turn ADC off for config
    
    ADCON1bits.ADPREF = 0b00;   //ADC positive reference is set to VDD
    ADCON1bits.ADNREF = 0;      //ADC negative reference is set to VSS
    ADCON0bits.CHS = 0b00010;   //selecting the AN2 analog channel
    ADCON1bits.ADCS = 0b010;    //ADC clock set to FOSC/32 (1us convertion time with Fosc = 32MHz)
    ADCON1bits.ADFM = 1;        //ADC result is right justified

    ADCON0bits.ADON = 1;        //turn ADC on
}

void  NCO1_init(void){
    // sets up the NCO module. the output clock frequency of the module as it
    // is setup below is:
    //
    // NCO_out_freq = ((NCO_clock_freq * NCO_increment_value) / (2^20)) / 2
    // the divide by two factor above comes from the 50% duty cycle calc
    //
    // clock frequency relates to delay time in the memory man:
    // delay time = (total_BBD_stages / (2 * BBD_clock_freq)
    NCO1CONbits.N1EN = 0;       //turn off the NCO for config
    
    NCO1CONbits.N1PFM = 0;      //set the NCO output to a fixed 50% duty cycle
    NCO1CLKbits.N1CKS = 0x01;   //set the NCO clock to the FOSC (32MHz)
    
    // set the NCO increment value to 300 just to start. this should initialize
    // the NCO overflow frequency to 9.155kHz. therefore the real output
    // frequency should be 4.578kHz
    NCO1INCU = 0x0;
    NCO1INCH = 0x01;
    NCO1INCL = 0x2C;
    
    NCO1CONbits.N1EN = 1;       //turn on the NCO
}

void CWG_init(void){
    // set up the complementary waveform generator
    CWG1CON0bits.CWG1EN = 0;        //disable the CWG
    
    CWG1CON0bits.CWG1MODE = 0b100;  //CWG set to half-bridge mode
    CWG1CON1bits.POLA = 0;          //output A is normal
    CWG1CON1bits.POLB = 0;          //output B is inverted
    CWG1DATbits.DAT = 0b1001;       //the NCO is the CWG data input
    
    // shutdown settings. the defaults in CWG1AS1 are fine for this application
    CWG1AS0bits.CWG1LSAC = 0b01;    //output pins will tri-state in shutdown
    CWG1AS0bits.CWG1LSBD = 0b01;    //output pins will tri-state in shutdown
    CWG1AS0bits.REN = 1;            //auto restart enabled
    
    CWG1DBRbits.DBR = 0b000010;     //rising dead band is 1-2 HFINTOSC periods
    CWG1DBFbits.DBF = 0b000010;     //falling dead band is 1-2 HFINTOSC periods
    CWG1CLKCONbits.CS = 1;          //the HFINTOSC drives the dead band time
    CWG1CON0bits.CWG1EN = 1;        //enable the CWG 1
    
    CWG1AS0bits.SHUTDOWN = 0;       //disable shutdown
}

void PPS_init(void){
    // set up the pin assignments for the various peripherals
    PPSLOCKbits.PPSLOCKED = 0;      //unlock the peripheral control
    
    RC5PPSbits.RC5PPS = 0b01000;    //RC5 is attached to the CWG1A output
    RC4PPSbits.RC4PPS = 0b01001;    //RC4 is attached to the CWG1B output
}

void timer0_init(void){
    // timer 0 is used for the tap tempo timing. the clock source is the
    // internal 31kHz oscillator LFINTOSC. the increment rate should be
    // 1ms, and the timer will overflow after ~65 seconds.
    
    T0CON0bits.T0EN = 0;        //turn off timer 0 for config
    PIR0bits.TMR0IF = 0;        //clear the timer 0 overflow interrupt flag
    
    T0CON0bits.T016BIT = 1;     //timer 0 runs in 16bit mode
    T0CON1bits.T0CKPS = 0b0101; //timer 0 prescaler set to 1:32
    T0CON0bits.T0OUTPS = 0b0000;//timer 0 post scaler set to 1:1
    T0CON1bits.T0CS = 0b100;    //timer 0 clock source set to LFINTOSC
    
    T0CON0bits.T0EN = 1;        //turn on timer 0
}

void timer4_init(void){
    // timer 4 is used to run the tap tempo button debouncing and it schedules
    // the ADC readings. the clock source is Fosc / 4
    T4CONbits.TMR4ON = 0;       //turn off timer 4 for config
    PIR2bits.TMR4IF = 0;        //clear the timer 4 match interrupt flag
    
    T4CONbits.T4CKPS = 0b10;    //prescaler set to 1:16
    T4CONbits.T4OUTPS = 0b1111; //postscaler set to 1:16
    PR4 = 156;
    
    // the above sets the timer 4 match interrupt frequency to
    // (Fosc / 4) * prescaler * (1 / (PR2)) * postscaler = interrupt freq.
    // (8MHz) * (1/16) * (1 / (156)) * (1/16) = ~200Hz 
    T4CONbits.TMR4ON = 1;       //turn on timer 4
}

void perform_adc_tasks(void){
    // if the ADC has completed a conversion, write the value into the "results"
    // register
    if (ADCON0bits.GO_nDONE == 0){
        current_adc_result = ADRES;
    }
    ADCON0bits.GO_nDONE = 1;               //start ADC

    // no need to write to the NCO from the ADC if we're in tap tempo mode. but
    // if we are in tap tempo mode and the knob position changes, then we should
    // exit tap tempo mode and prioritize defining the delay time with the knob
    if (tap_tempo_mode){
        if ( (current_adc_result >= (old_adc_result + 10)) | (current_adc_result <= (old_adc_result - 10)) ){
            tap_tempo_mode = 0;
        } else {
        return;
        }
    }
    
    // right here, there should be a way to map the raw ADC readings to the
    // BBD clock frequencies used by the original DMM...

    // use the ADC value to drive the NCO frequency by adjusting the increment
    // register
    NCO1INCU = 0;
    NCO1INCH = current_adc_result >> 8 ;
    NCO1INCL = current_adc_result;
    
    old_adc_result = current_adc_result;
}

bit tt_pos_edge_detected(){
    // this algo by jack ganssle. returns true if a positive edge has been
    // detected from the tap tempo button. 
    static unsigned char debounce_status = 0;       //current debounce status
    debounce_status = (debounce_status << 1) | !PORTCbits.RC0 | 0xF0;
    if (debounce_status == 0xF3){
        return 1;
    }
    return 0;
}

void compute_write_new_nco_freq(unsigned int time){
    // this line converts the delay time between tap tempo button presses into
    // the appropriate NCO increment value. the line also compensates for the
    // fact that each tick of timer 0 only counts for 0.969ms - the input clock
    // for timer 0 runs at 31KHz, not 32KHz!
    NCO_increment = (unsigned short long) 260111.88 / time;
    
    // establish a lower frequency limit. this limit is, for now, set to just
    // below the lower clock frequency limit on the stock DMM
    if (NCO_increment < 450){
        NCO_increment = 450;
    }
    
    // establish an upper frequency limit. this limit is, for now, set to just
    // above the upper clock frequency limit on the stock DMM
    if (NCO_increment > 36000){
        NCO_increment = 36000;
    }
    
    // split this increment value up and put it in the NCO increment register
    NCO1INCU = NCO_increment >> 16;
    NCO1INCH = NCO_increment >> 8;
    NCO1INCL = NCO_increment;
}

void calc_tap_tempo(void){
    // here is where we figure out how to set the NCO clock based on the tap
    // tempo signal
    // when the button is pushed for the first time in a while, we have to turn 
    // on timer 0 and tell the rest of the program we're doing a tap tempo
    if (tap_tempo_mode == 0){
        // reset timer 0. one must write to TMR0H first and then TMR0L due to
        // the way the timer 0 high byte buffer works
        TMR0H = 0;              
        TMR0L = 0;
        tap_tempo_mode = 1;         //set the tap tempo flag
        last_tap_time = 0;          //since we're starting from scratch
        return;                     //leave early on the first button push
    }

    // after the first button push, we have to store how long it has been
    // since the last button push
    current_tap_time = ((TMR0H << 8 ) + TMR0L);
    
    // average the last two tap times to smooth the results a bit
    avg_delay_time = (current_tap_time + last_tap_time) / 2;
    
    // ignore the delay time if it's too long (limit set to 5 seconds for now).
    // otherwise write a new value to the NCO
    if (avg_delay_time < 5000){
        compute_write_new_nco_freq(avg_delay_time);
    } else {
        tap_tempo_mode = 0;         //exit tap tempo mode
        return;
    }
    
    //overwrite the last tap time with the new one
    last_tap_time = current_tap_time;
    
    // reset our millisecond counter
    TMR0H = 0;              
    TMR0L = 0;
}

void main(void) {
    // configure the internal clock to run at 32MHz. this means that the
    // instruction clock will run at 8MHz (Fosc/4)
    OSCCON1bits.NOSC = 0b000;   //set the "new osc" source to HFINTOSC w/ 2x PLL
    OSCFRQbits.HFFRQ = 0b0110;  //set the HFINTOSC to 16MHz

    // configure the watchdog timer
    //WDTCONbits.WDTPS = 0b01011; //set to 2s timer
    
    // configure the inputs and outputs
    TRISAbits.TRISA2 = 1;       //set RA2 (pin 11) as input (analog input for "speed" pot)
    TRISCbits.TRISC0 = 1;       //set RC0 (pin 10) as input (for tap tempo button)
    TRISCbits.TRISC1 = 0;       //set RC1 (pin 9) as output
    TRISCbits.TRISC2 = 0;       //set RC2 (pin 8) as output
    TRISCbits.TRISC3 = 0;       //set RC3 (pin 7) as an output
    TRISCbits.TRISC4 = 0;       //set RC4 (pin 6) as an output (for pos clock out)
    TRISCbits.TRISC5 = 0;       //set RC5 (pin 5) as an output (for neg clock out)
    ANSELA = 0b00000100;        //set RA2 (pin 11) as an analog input (AN2 channel))
    ANSELC = 0b00000000;        //nothing on port C is an analog input
    
    // configure the internal peripherals
    adc_init();               //configure the ADC
    NCO1_init();              //configure the NCO
    CWG_init();               //configure the complementary waveform generator
    PPS_init();               //assign peripherals to pins
    timer0_init();            //setup timer 0 and turn it on
    timer4_init();            //setup and turn on timer 4
    
    // turn on interrupts
    PIE2bits.TMR4IE =  1;     //enable the timer 4 to PR4 match interrupt
    INTCONbits.PEIE = 1;      //enable peripheral interrupts
    INTCONbits.GIE = 1;       //general interrupts enabled
    
    LATC = 0x00;              //init the port c latches
    LATCbits.LATC2 = 0;       //just to tell the user that the program started        
    
    // below is the main loop
    while(1){
        CLRWDT();               //clear the Watchdog Timer to keep the PIC from
                                //resetting
        
        // if the debounce routine found a positive edge, then we must do the
        // tap tempo math
        if (calc_tap_tempo_flag){
            LATCbits.LATC2 = 1;             //i want to see it!
            calc_tap_tempo();
            LATCbits.LATC2 = 0;
            calc_tap_tempo_flag = 0;       //reset the flag
        }
        
        // if it's time to read the ADC, do so and see if we need to update
        // the NCO based on the ADC reading.
        if (adc_timing_flag){
            perform_adc_tasks();
            adc_timing_flag = 0;            //reset the flag
        }
    }
    return;
}

void interrupt ISR(void){
  
    // check for timer 4 to PR4 match interrupt
    if(PIR2bits.TMR4IF == 1){
        // tell the main loop that it's time to read the tap tempo button input
        if (tt_pos_edge_detected()){
            calc_tap_tempo_flag = 1;
        }
    
        // tell the main loop that it's time to read the ADC
        adc_timing_flag = 1;
        
        PIR2bits.TMR4IF = 0;                 //reset the interrupt flag
    }
    
   return;
}
