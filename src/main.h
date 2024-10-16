/*
 * main.h
 *
 *  Created on: Apr 14, 2014
 *      Author: michael
 */

#ifndef MAIN_H_
#define MAIN_H_

//--------------------------------------------------
// Global Defines
//--------------------------------------------------
// PWM LEDs
#define NLEDS                   12          //How many PWM channels to handle
// ADC, voltage measurement
#define VCAPDIVH                549.0       //[kOhm]
#define VCAPDIVL                160.0       //[kOhm]
#define VCAP_CONV_FACT_MV       (uint16_t)( 10000.0 / 1023 * 1.1 * ( VCAPDIVL + VCAPDIVH ) / VCAPDIVL ) //to get [10*mV] from the ADC value
#define ADC_MUX_VCAP            0x06
#define ADC_MUX_SOLAR           0x07
#define ADC_MUX_TEMP            0x08
#define ADC_MUX_VREF            0x0E
#define ADC_MUX_GND             0x0F
// Sleeping behaviour
#define LBATT_THRESHOLD         3300        //[mV] go to deep sleep mode to save battery
#define HBATT_THRESHOLD         4200        //[mV] switch on LEDs to burn overvoltage
#define CHARGING_THRESHOLD      3000        //Sun is present if the solar cell voltage is more than that [mV]
#define WDT_N_WK_ADC_LBATT      113/4       //Only measure voltage every 113/4 * 32 = 15 min if battery is down
#define MANUAL_CONTROL_TIMEOUT  113/4       //Go back to automode if no new command received for n 32s intervals
// Status flags
#define FLAG_wakeWDT            1           //WDT timeout occurred (8 seconds passed)
#define FLAG_putNewFrame        2           //Tell main to render a new PWM frame
#define FLAG_nRF_RX_DR          3           //We received something over the air!
#define FLAG_PWM_IS_ON          4           //Timer 1 is running, pwm is on

// Pseudo functions
#define SBI(reg, bit) ( reg |=  ( 1 << bit ) )
#define CBI(reg, bit) ( reg &= ~( 1 << bit ) )
#define IBI(reg, bit) ( reg &   ( 1 << bit ) )
#define SLEEP()             { asm volatile("sleep"); }
#define SLEEP_SET_ADC()     { SMCR=0b00000011; }    //For ADC measurements
#define SLEEP_SET_IDLE()    { SMCR=0b00000001; }    //When T0 still needs to run
#define SLEEP_SET_STANDBY() { SMCR=0b00001101; }    //When only the crystal needs to run
#define SLEEP_SET_PDOWN()   { SMCR=0b00000101; }    //When everything except the WDT is off

//--------------------------------------------------
// Global Variables
//--------------------------------------------------
#define WDT_N_WK_FAST_MODE      240                 //Do housekeeping every 240 wakeups (8s), when in FAST_MODE
#define FAST_MODE               0x80                //When currentState>=FAST_MODE, wdt is triggered with 30 Hz, otherwise every 8 s
#define WDT_SLOW_MODE()         { wdt_reset(); WDTCSR|=(1<<WDE)|(1<<WDCE); WDTCSR=(1<<WDIE)|(1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0); }  //8.0 s
#define WDT_FAST_MODE()         { wdt_reset(); WDTCSR|=(1<<WDE)|(1<<WDCE); WDTCSR=(1<<WDIE)|(0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0); }  //32 ms, 31.25 Hz


typedef enum { STATE_LOW_BATT, STATE_HIGH_BATT, STATE_CHARGING, STATE_GLOW_MANUAL_CONTROL_SLOW, STATE_GLOW_GLUEH=FAST_MODE, STATE_GLOW_FULLSCREEN, STATE_GLOW_MANUAL_CONTROL } currentState_t;
currentState_t currentState;
extern volatile uint8_t flags;
extern uint16_t vCap, vSolar, temperature;

//--------------------------------------------------
// Functions
//--------------------------------------------------
void init(void);
void onWDT();
void onRX();

uint16_t readVCAP();
uint16_t readVSolar();
uint16_t readADC( uint8_t mux, uint8_t n );
void houseKeeping();




#endif /* MAIN_H_ */
