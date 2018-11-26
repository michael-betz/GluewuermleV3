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
#define NLEDS 					12			//How many PWM channels to handle
// ADC, voltage measurement
#define VCAPDIVH				560.0		//[kOhm]
#define VCAPDIVL				160.0		//[kOhm]
//#define	VCAP_CONV_FACT_MV		(uint16_t)( 10000.0 / 1023 * 1.1 * ( VCAPDIVL + VCAPDIVH ) / VCAPDIVL )	//to get [10*mV] from the ADC value
#define	VCAP_CONV_FACT_MV		47			//Fits better for Glueh4
#define WDT_N_WK_ADC_LBATT		113			//Only measure voltage every 113 * 8 = 15 min if battery is down
#define ADC_MUX_VCAP			0x07
#define ADC_MUX_SOLAR			0x06
#define ADC_MUX_TEMP			0x08
#define ADC_MUX_VREF			0x0E
#define ADC_MUX_GND				0x0F
// Sleeping behaviour
#define LBATT_THRESHOLD			3300		//[mV] go to deep sleep mode to save battery
#define HBATT_THRESHOLD			4190		//[mV] switch on LEDs to burn overvoltage
#define CHARGING_THRESHOLD		2000		//Sun is present if the solar cell voltage is more than that [mV]
// Status flags
#define FLAG_wakeWDT 			1			//WDT timeout occurred (8 seconds passed)
#define FLAG_putNewFrame		2			//Tell main to render a new PWM frame
#define FLAG_nRF_RX_DR			3			//We received something over the air!
#define FLAG_PWM_IS_ON			4			//Timer 1 is running, pwm is on

// Pseudo functions
#define SBI(reg, bit) ( reg |=  ( 1 << bit ) )
#define CBI(reg, bit) ( reg &= ~( 1 << bit ) )
#define IBI(reg, bit) ( reg &   ( 1 << bit ) )
#define SLEEP()				{ asm volatile("sleep"); }
#define SLEEP_SET_ADC()     { SMCR=0b00000011; }	//For ADC measurements
#define SLEEP_SET_IDLE()    { SMCR=0b00000001; }	//When T0 still needs to run
#define SLEEP_SET_STANDBY() { SMCR=0b00001101; }	//When only the crystal needs to run
#define SLEEP_SET_PDOWN() 	{ SMCR=0b00000101; }	//When everything except the WDT is off

//--------------------------------------------------
// Global Variables
//--------------------------------------------------
#define	WDT_N_WK_FAST_MODE		240					//Do housekeeping every 240 wakeups (8s), when in FAST_MODE
#define FAST_MODE 				0x80				//When currentState>=FAST_MODE, wdt is triggered with 30 Hz, otherwise every 8 s
#define WDT_SLOW_MODE()			{ wdt_reset(); WDTCSR|=(1<<WDE)|(1<<WDCE); WDTCSR=(1<<WDIE)|(1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0); }	//8.0 s
#define WDT_FAST_MODE()			{ wdt_reset(); WDTCSR|=(1<<WDE)|(1<<WDCE); WDTCSR=(1<<WDIE)|(0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0); }	//32 ms, 31.25 Hz

//WDP3 WDP2 WDP1 WDP0 Number of WDT Oscillator
//Cycles Typical Time-out at
//V CC = 5.0V
//0 0 0 0 2K (2048) cycles 16ms
//0 0 0 1 4K (4096) cycles 32ms
//0 0 1 0 8K (8192) cycles 64ms
//0 0 1 1 16K (16384) cycles 0.125 s
//0 1 0 0 32K (32768) cycles 0.25 s
//0 1 0 1 64K (65536) cycles 0.5 s
//0 1 1 0 128K (131072) cycles 1.0 s
//0 1 1 1 256K (262144) cycles 2.0 s
//1 0 0 0 512K (524288) cycles 4.0 s
//1 0 0 1 1024K (1048576) cycles 8.0 s

typedef enum { STATE_LOW_BATT, STATE_HIGH_BATT, STATE_CHARGING, STATE_GLOW_GLUEH=FAST_MODE, STATE_GLOW_FULLSCREEN } currentState_t;
currentState_t currentState;
extern volatile uint8_t flags;
extern uint16_t vCap, vSolar, temperature, wdtCountSinceCharging;

//--------------------------------------------------
// Functions
//--------------------------------------------------
void init(void);
void onWDT();
void onRX();

uint16_t readVCAP();
uint16_t readVSolar();
uint16_t readADC( uint8_t mux, uint8_t n );




#endif /* MAIN_H_ */
