/*
 * File:   main.c
 * Author: rschaub
 *
 *                   PIC16F18326
 *                   __________________
 *           VDD ---|1               14|--- VSS
 *               ---|2 / RA5   RA0 / 13|---
 *               ---|3 / RA4   RA1 / 12|---
 *               ---|4 / RA3   RA2 / 11|--- delay time knob input
 *     clk out+  ---|5 / RC5   RC0 / 10|--- chorus amount knob input
 *     clk out-  ---|6 / RC4   RC1 /  9|--- tap tempo button input
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
volatile bit new_dly_knb_result = 0;            //flag to let us a knob if there's a new dly knob reading
unsigned int dly_knb_adc_result = 0;            //holding register for the delay knob adc result
unsigned int current_dly_knb_adc_result = 0;    //latest value from delay knob
unsigned int old_dly_knb_adc_result = 0;        //old value from delay knob
unsigned int cho_knb_adc_result = 0;            //holding register for the chorus knob adc result

//variables for the chorus LFO
signed float chorus_LFO = 0;              //holds the current value of the chorus LFO
                                          //sort of a phase accumulator
signed float chorus_LFO_dir = 0.01;       //controls the chorus LFO wave's direction

// variables for the tap tempo delay time calculation
volatile bit calc_tap_tempo_flag = 0;   //sign that we need to compute tempo
volatile bit tap_tempo_mode = 0;        //are we in tap tempo mode?
unsigned int last_tap_time = 0;
unsigned int current_tap_time = 0;

// variables for the NCO
unsigned int delay_time = 0;
unsigned short long NCO_increment = 0;


void adc_init(void){
    // sets up the ADC
    ADCON0bits.ADON = 0;        //turn ADC off for config
    
    ADCON1bits.ADPREF = 0b00;   //ADC positive reference is set to VDD
    ADCON1bits.ADNREF = 0;      //ADC negative reference is set to VSS
    ADCON0bits.CHS = 0b00010;   //selecting A2 as the input analog channel
    ADCON1bits.ADCS = 0b110;    //ADC clock set to FOSC/64 (2us convertion time with Fosc = 32MHz)
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
    NCO1CLKbits.N1CKS = 0x00;   //set the NCO clock to the HFINTOSC (16MHz)
    
    // set the NCO increment value to 300 just to start. this should initialize
    // the NCO overflow frequency to ~4.58kHz. therefore the real output
    // frequency should be ~2.29kHz
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
    
    CWG1DBRbits.DBR = 0b000001;     //rising dead band is 1-2 HFINTOSC periods
    CWG1DBFbits.DBF = 0b000001;     //falling dead band is 1-2 HFINTOSC periods
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

void timer2_init(void){
    // timer 2 is used to run the chorus LFO
    T2CONbits.TMR2ON = 0;       //turn off timer 2 for config
    PIR1bits.TMR2IF = 0;        //clear timer 2 match interrupt flag
    
    T2CONbits.T2CKPS = 0b11;    //prescaler set to 1:64
    T2CONbits.T2OUTPS = 0b0001; //postscaler set to 1:2
    PR2 = 125; 
    
    // the above sets the timer 4 match interrupt frequency to
    // (Fosc / 4) * prescaler * (1 / (PR2)) * postscaler = interrupt freq.
    // (8MHz) * (1/64) * (1 / (125)) * (1/2) = ~500Hz
    T2CONbits.TMR2ON = 1;       //turn on timer 2
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

void update_adc_readings(){
    // if the ADC has completed a conversion, write the result to whichever
    // holding variable is "up next" depending on what channel has been 
    // prepared. also prepare the "other" channel if a conversion on one channel
    // is ready
    
    // has the ADC finished a conversion?
    if (ADCON0bits.GO_nDONE == 0){
        // if the delay knob has been chosen as the input
        if (ADCON0bits.CHS == 0b00010){
            dly_knb_adc_result = ADRES;
            ADCON0bits.CHS = 0b010000;  //switch the channel to the chorus amount
            new_dly_knb_result = 1;     //let us know a new delay knob value is available
        // if the chorus amount knob has been chosen as the input
        } else if (ADCON0bits.CHS == 0b010000){
            cho_knb_adc_result = ADRES;
            ADCON0bits.CHS = 0b00010;   //switch the channel to the delay time
        }
    }
}

void update_chorus_LFO() {
    //creates the triangle-wave LFO needed to emulate the DMM's chorus LFO
    
    chorus_LFO += chorus_LFO_dir;
    
    if (chorus_LFO >= 1.0) {
        chorus_LFO_dir = -0.01;
    } else if (chorus_LFO <= -1.0){
        chorus_LFO_dir = 0.01;
    }
}

bit tt_pos_edge_detected(){
    // this algo by jack ganssle. returns true if a positive edge has been
    // detected from the tap tempo button. 
    static unsigned char debounce_status = 0;       //current debounce status
    debounce_status = (debounce_status << 1) | !PORTCbits.RC1 | 0xF0;
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
    NCO_increment = (unsigned short long) 520223.76 / time;
    
    // add in the impact from the chorus LFO, depending on the value of the
    // chorus knob
    NCO_increment += (unsigned short long) (chorus_LFO * cho_knb_adc_result);
    
    // establish a lower frequency limit. this limit is, for now, set to just
    // below the lower clock frequency limit on the stock DMM
    if (NCO_increment < 450){
        NCO_increment = 450;
    }
    
    // establish an upper frequency limit. this limit is, for now, set to just
    // above the upper clock frequency limit on the stock DMM
    if (NCO_increment > 18000){
        NCO_increment = 18000;
    }
    
    // split this increment value up and put it in the NCO increment register
    NCO1INCU = NCO_increment >> 16;
    NCO1INCH = NCO_increment >> 8;
    NCO1INCL = NCO_increment;
}

void init_tap_tempo(void){
    // reset timer 0. one must write to TMR0H first and then TMR0L due to
    // the way the timer 0 high byte buffer works
    TMR0H = 0;              
    TMR0L = 0;
    tap_tempo_mode = 1;         //set the tap tempo flag
    last_tap_time = 0;          //since we're starting from scratch
}

unsigned int calc_tap_tempo(){
    // here is where we figure out how to set the NCO clock based on the tap
    // tempo signal
    
    unsigned int new_delay_time;

    // after the first button push, we have to store how long it has been
    // since the last button push
    current_tap_time = (unsigned int) ((TMR0H << 8 ) + TMR0L);
    
    // average the last two tap times to smooth the results a bit
    new_delay_time = (current_tap_time + last_tap_time) / 2;
    
    // ignore the delay time if it's too long (limit set to 5 seconds for now).
    // otherwise write a new value to the NCO
    if (new_delay_time > 5000){
        tap_tempo_mode = 0;         //exit tap tempo mode
        return delay_time;          //we return the old delay time
    }
    
    //overwrite the last tap time with the new one
    last_tap_time = current_tap_time;
    
    // reset our millisecond counter (timer 0). well millisecond ish...
    TMR0H = 0;              
    TMR0L = 0;
    
    return new_delay_time;
}

unsigned int delay_time_from_delay_knob(unsigned int adc_reading){
    // scale the ADC readings into NCO increment values that correspond to the
    // DMM's original delay time knob range
    return 542 - (adc_reading / 2);
}

void main(void) {
    // configure the internal clock to run at 32MHz. this means that the
    // instruction clock will run at 8MHz (Fosc/4)
    OSCCON1bits.NOSC = 0b000;   //set the "new osc" source to HFINTOSC w/ 2x PLL
    OSCFRQbits.HFFRQ = 0b0110;  //set the HFINTOSC to 16MHz

    // configure the watchdog timer
    //WDTCONbits.WDTPS = 0b01011; //set to 2s timer
    
    // configure the inputs and outputs
    TRISAbits.TRISA2 = 1;       //set RA2 (pin 11) as input (analog input for "delay time" pot)
    TRISCbits.TRISC0 = 1;       //set RC0 (pin 10) as input (analog input for "chorus amount" pot)
    TRISCbits.TRISC1 = 1;       //set RC1 (pin 9) as input (digital for tap tempo button)
    TRISCbits.TRISC2 = 0;       //set RC2 (pin 8) as output
    TRISCbits.TRISC3 = 0;       //set RC3 (pin 7) as an output
    TRISCbits.TRISC4 = 0;       //set RC4 (pin 6) as an output (for pos clock out)
    TRISCbits.TRISC5 = 0;       //set RC5 (pin 5) as an output (for neg clock out)
    ANSELA = 0b00000100;        //set RA2 (pin 11) as an analog input (AN2 channel))
    ANSELC = 0b00000001;        //set RC0 (pin 10) as an analog input
    
    // configure the internal peripherals
    adc_init();               //configure the ADC
    NCO1_init();              //configure the NCO
    CWG_init();               //configure the complementary waveform generator
    PPS_init();               //assign peripherals to pins
    timer0_init();            //setup timer 0 and turn it on
    timer2_init();            //setup timer 2 and turn it on
    timer4_init();            //setup and turn on timer 4
    
    // turn on interrupts
    PIE1bits.TMR2IE = 1;      //enable the timer 2 to PR2 match interrupt
    PIE2bits.TMR4IE =  1;     //enable the timer 4 to PR4 match interrupt
    INTCONbits.PEIE = 1;      //enable peripheral interrupts
    INTCONbits.GIE = 1;       //general interrupts enabled
    
    LATC = 0x00;              //init the port c latches
    LATCbits.LATC2 = 1;       //just to tell the user that the program started        
    
    // below is the main loop
    while(1){
        CLRWDT();               //clear the Watchdog timer to keep the PIC from
                                //resetting
 
        // if the debounce routine found a positive edge, then we must do the
        // tap tempo math
        if (calc_tap_tempo_flag){
            // if we caught a positive edge but are not in tap tempo mode, then
            // we need to initialize the tap tempo bits
            if (!tap_tempo_mode){
                init_tap_tempo();
            // if we caught a positive edge and are already in tap tempo mode,
            // then we can calculate the delay time
            } else {
               delay_time = calc_tap_tempo(); 
            }
            calc_tap_tempo_flag = 0;       //reset the flag
        }        
        
        // if there's a new reading available from the delay time knob, check
        // to see if we need to use and act accordingly
        if (new_dly_knb_result) {
            current_dly_knb_adc_result = dly_knb_adc_result;
            // if we're in tap tempo mode, we should only use the delay time
            // knob to change the delay time if the knob has turned since the
            // last time we looked
            if (tap_tempo_mode) {
                if ( (current_dly_knb_adc_result >= (old_dly_knb_adc_result + 5)) 
                        | (current_dly_knb_adc_result <= (old_dly_knb_adc_result - 5)) ){
                    tap_tempo_mode = 0;     //exit tap tempo if the knob moved
                    delay_time = delay_time_from_delay_knob(current_dly_knb_adc_result);
                }
            // if we're not in tap tempo mode, than we can directly write a new
            // delay time from the delay time knob reading
            } else {
                delay_time = delay_time_from_delay_knob(current_dly_knb_adc_result);
            }
            old_dly_knb_adc_result = current_dly_knb_adc_result;
            new_dly_knb_result = 0;            //reset the flag
        }

        // update the NCO based on the delay time
        compute_write_new_nco_freq(delay_time);
    }
    return;
}

void interrupt ISR(void){
    
    // check for timer 2 to PR2 match interrupt
    if (PIR1bits.TMR2IF == 1) {
        update_chorus_LFO();
        
        PIR1bits.TMR2IF = 0;                //reset the interrupt flag
    }
  
    // check for timer 4 to PR4 match interrupt
    if(PIR2bits.TMR4IF == 1){
        // tell the main loop that it's time to read the tap tempo button input
        if (tt_pos_edge_detected()){
            calc_tap_tempo_flag = 1;
        }
    
        // read the ADC if it's time. note that the language below causes each
        // knob to be read at 50Hz due to the ADC channel switching and the way
        // ADC reads are initiated every other time timer4 matches PR4
        if (adc_timing_flag) {
            update_adc_readings();
            adc_timing_flag = 0;
        } else {
            ADCON0bits.GO_nDONE = 1;         //start ADC
            adc_timing_flag = 1;             //so that the next time we do a read
        }
   
        PIR2bits.TMR4IF = 0;                 //reset the interrupt flag
    }
    
   return;
}
