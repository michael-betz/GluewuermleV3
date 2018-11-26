/*
 * main.c
 *
 *  Created on: Apr 14, 2014
 *      Author: michael
 * 
 * We will have 4 or 5 fixed glowing patterns as wavetable in flash memory
 * 		One is about 4 or 5 s long and rather dim and continous
 *		One is < 1 s long and short but intense (but less likely to be played)
 * WDT measures light conditions and battery voltage every 8 seconds 	-- in onWDT()
 *	--> If dark and full, switch WDT to 0.5 s interrupts
 * If less than 4 Blinky sequences are in progress:
 * 		There is a certain change that WDT starts a new Blinky sequence -- in onWDT()
 * Blinky
 * 	- int8_t pwmChannel					//Set to a random and FREE LED channel, -1 = inactive Blinky slot
 *  - uint8_t *glowingPatternPointer	//Set to a random pattern with preference for more dim patterns
 *  - uint8_t samplesLeftInPattern		//How many samples left to show in this pattern
 *  - uint8_t blinkySpeed				//Delay between samples, set to a random value with reasonable limits
 * Blinky is updated and shown on the LEDs by setting curPWMvalues[]	-- in onNewFrame()
 * 		if samplesLeftInPattern==0:  pwmChannel=-1   --> Blinky slot is inactive
 *
 * Things to do
 * --------------
 * All pwm values are iterated during every wakeup in the PWM routine which is rather inefficient
 * (even if the OCR register is used for precise wakeups)
 *
 * Better: 8-10 Wakeups per PWM cycle, arranged in a 2^N way
 *
 * Each wakeup uses precalculated port settings which are output from ram to the port pins
 * For 10 bit PWM we have the arrays uint8_t pwmPrecalcPortB[10], pwmPrecalcPortC[10], pwmPrecalcPortD[10]
 *
 * there is a setPwmValue( uint8_t channel, uint16_t value ) routine, which does the precalculation
 * from main()
 *
 * it chooses a pwmPrecalcPortX[] array and a pinNumber, depending on the channel.
 * Then it does:
 * uint8_t *precalc = pwmPrecalcPortX;
 * for( uint8_t pwmStep=0; pwmStep<10; pwmStep++ ){
 * 		if( IBI( value, pwmStep ) ){
 * 			SBI( *precalc, pinNumber );
 * 		} else {
 * 			CBI( *precalc, pinNumber );
 * 		}
 *		precalc++;
 * }
 *
 * This avoids having to sort the array to find the next wakeup
 *
 * We could propably get away with 11 bit PWM and 100 Hz rate
 */

#include <inttypes.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include "rprintf.h"
#include "lfsr32.h"
#include "main.h"
#include "glowPatterns.h"
#include "pulseCode.h"

//--------------------------------------------------
// Global variables
//--------------------------------------------------
volatile uint8_t flags = 0;
uint16_t cycleCount = 0;
uint16_t vCap=0, vSolar=0;


void init(void) {
//--------------------------------------------------
// LED Ports
//--------------------------------------------------
	DDRC   = 0b00111111;	//Set LED pins as output
	DDRD   = 0b11111100;
//--------------------------------------------------
// Power Reduction, switch off every peripheral not needed
//--------------------------------------------------
	PRR    = 0xFF;
	SBI( MCUCR, PUD );						//Disable Pull-ups globally
//--------------------------------------------------
// Serial debugging at 38400 baud/s
//--------------------------------------------------
	SBI( DDRD, PD1 );						//Set UART TX pin as output
	odDebugInit();
//-----------------------------------------------------------------------
// ADC for measuring CAP voltage with divider: 560k, 160k, on PC5 = ADC5
//-----------------------------------------------------------------------
    ACSR   = (1 << ACD);					//Switch off Analog comparator
    ADCSRA = 0;								//Switch off ADC for now
    ADCSRB = 0;
    DIDR0  = 0b00111111;					//Switch off all digitial port functions on PORTC0 - PORTC5
//-----------------------------------------------------------------------
// Timer1, HiSpeed PWM, Compare Match = Overflow ISR used for LED output
//-----------------------------------------------------------------------
    TCCR1A = 0b00000000;					//No Port change on compare match
    TIFR1   = (1<<OCF1B)|(1<<OCF1A)|(1<<TOV1);	//Clear interrupt flags
    TIMSK1  = (1 << OCIE1A); 				//Enable Interrupt on compare match OCR1A
	OCR1A = 0x00FF;
//----------------------------------------------------------------
// Init independent Watchdog timer to fire Interrupt every  8.0 s
//----------------------------------------------------------------
//Note that WDTON Fuse (force Watchdog) must be == 1 --> unprogrammed
//    wdt_reset();
//    WDTCSR |= (1 << WDE) | (1 << WDCE);		//Unlock safety
//    WDTCSR  = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);  							//Set prescaler to 128 kHz / 1024k = 8s, interrupt mode
//    WDTCSR  = (1 << WDIE) | (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);  //Set prescaler to 1s, interrupt mode
	WDT_SLOW_MODE();
//------------------------------------------------------
// init PRNG, load last seed from EEPROM, calculate next seed and store back to EEPROM, use another polynom as in lfsr()
//------------------------------------------------------
    eeprom_read_block((uint8_t *)&seed, (uint8_t *)(0), 4);
    lfsr_poly(32, 0x8140C9D5);
    eeprom_write_block((uint8_t *)&seed, (uint8_t *)(0), 4);
}

void ledTest(){
	uint8_t nled, iter, value=0xFF;
	for( iter=0; iter<=4; iter++ ){
		for( nled=0; nled<NLEDS; nled++){
			setPwmValue( nled, value );
			_delay_ms( 100 );
		}
		if( value == 0xFF ){
			value = 0x01;
		} else {
			value = 0xFF;
		}
	}
	for( nled=0; nled<NLEDS; nled++){
		setPwmValue( nled, 0 );
	}
}

int main(){
	init();				//Now Timer0 is running and will generate an interrupt on its first overflow
	rprintf("\n\n---------------------------------------\n");
	rprintf(" Hello world, this is Gluewuermle 3 ! \n");
	rprintf("---------------------------------------\n");
	rprintf(" MAX_PWM_VALUE = %d\n", MAX_PWM_VALUE);
	pwmTimerOn();
	sei();
	rprintf(" LED - test ... ");
	ledTest();
	rprintf("OK\n");
	currentState = STATE_GLOW_GLUEH;
	WDT_FAST_MODE();
	while(1){       	//We woke up. Figure out what to do (Wake-up sources: T0 compare match, WDT overflow)
        if ( IBI( flags, FLAG_wakeWDT ) ){		//This is called every PWM_CYCLES_PER_FRAME
            onWDT();		                	//Update the PWM values, switch OFF T0 to save energy if finished to glow
												//(Then we have less wake up interrupts and can do deep power down)
            CBI( flags, FLAG_wakeWDT );     	//Clear flag
        }
        cycleCount++;							//Count wakeups
		#if DEBUG_LEVEL > 0
			_delay_ms( 1 );
		#endif
		SLEEP();
	}
	return 0;
}

// Executed every 8 s by WDT to keep track of voltage levels
// Executed in any state except STATE_LOW_BATT
// Save the battery from over / undervoltage and switch states
void houseKeeping(){
	uint8_t temp;
	vSolar = readVCAP();
	vCap = readVSolar();
//		temperature = readADC( ADC_MUX_TEMP, 64 );	//max value: 65472
	if ( vCap < LBATT_THRESHOLD ){					//Go comatose!
		rprintf("currentState = STATE_LOW_BATT  (Battery undervoltage!)  zzZZzzZZ\n");
		currentState = STATE_LOW_BATT;
		WDT_SLOW_MODE();
		pwmTimerOff();
	} else if ( vCap > HBATT_THRESHOLD ){			//Battery is full, avoid overvoltage
		rprintf("currentState = STATE_HIGH_BATT  (Battery overvoltage!)  BUURN!\n");
		currentState = STATE_HIGH_BATT;
		WDT_SLOW_MODE();
		pwmTimerOn();
		for( temp=0; temp<NLEDS; temp++){
			setPwmValue( temp, MAX_PWM_VALUE );
		}
	}
	rprintf( "houseKeeping()   Vs = %4d   Vc = %4d   State = %d\n", vSolar, vCap, currentState );
}

//--------------------------------------------------
// Watch dog Interrupt
//---------------------
// if PWM_ouptut is on
//		if a LED is on
//			trigger every 0.03 s for new frame, Sleep = Idle (keep PWM timer running)
//		if no LED is on
//			trigger every 0.03 s for new frame, Sleep = Power down (Stop PWM timer)
// if PWM_output is off
//		trigger every 8.0 s for housekeeping, Sleep = Power down
//---------------------
// Housekeeping
//---------------------
// Decide if we should be in standby (during day) or glowing (during night)
// If vBatt is too low, go to low power mode (wakeup every ten minutes)
// If vBatt is too high, switch on all LEDs
//--------------------------------------------------
void onWDT(){
	static uint16_t wdtWakeupCounter=0x0000;	//Used to do something only every n wakeups
	uint8_t temp;

	if( currentState == STATE_LOW_BATT ){		//Special case, we exit as fast as possible
		if ( wdtWakeupCounter > WDT_N_WK_ADC_LBATT ){
			//--------------------------------------------------------------------
			// Everything below is executed every n min. if juice is low
			//--------------------------------------------------------------------
			wdtWakeupCounter = 0;
			vCap = readVCAP();
			if ( vCap > LBATT_THRESHOLD + 250 ){//We have charged a bit, wake up a bit
				currentState = STATE_CHARGING;
				WDT_SLOW_MODE();
				pwmTimerOff();
			}
		}
		wdtWakeupCounter++;
		return;
	} else {									//Normal operation
		if ( currentState >= FAST_MODE ) {		//WDT triggers with 30 fps
			if ( wdtWakeupCounter > WDT_N_WK_FAST_MODE ) {
				wdtWakeupCounter = 0;
				//--------------------------------------------------------------------
				// Everything below is executed every 8 s, if blinking
				//--------------------------------------------------------------------
				houseKeeping();					//Check for over / undervoltage and react
			}
		} else {								//WDT triggers once every 8 s
			//--------------------------------------------------------------------
			// Everything below is executed every 8 s, if not blinking
			//--------------------------------------------------------------------
			houseKeeping();						//Check for over / undervoltage and react
		}
	}

	switch( currentState ){
	//--------------------------------------------------------------------
	// Everything below is specific to a certain state
	//--------------------------------------------------------------------
	case STATE_GLOW_FULLSCREEN:
		newFrameFullscreen( -1 );
		if( wdtWakeupCounter == 0 ){						//Triggered every 8 s if glowing
			temp = lfsr(8);
			newFrameFullscreen( temp&0x0F );				//Change glowing mode to temp
			if( temp <= 15 ){								//Get out of Fullscreen glowing mode
				currentState = STATE_GLOW_GLUEH;
				rprintf("currentState = STATE_GLOW_GLUEH  (Enough Fullscreen)\n");
				for( temp=0; temp<NLEDS; temp++){
					setPwmValue( temp, 0 );
				}
			}
			houseKeepingWhileGlowing();
		}
		break;
	case STATE_GLOW_GLUEH:				//Triggered every 0.03 s
		newFrameGlueh();
		if( wdtWakeupCounter == 0 ){	//every 8 s if glowing
			if( (lfsr(8)<=2) && (vCap>3800) ){	//If vCap > 3800 mV
				currentState = STATE_GLOW_FULLSCREEN;
				pwmTimerOff();
				newFrameFullscreen( 0 );//Switch to BLACK mode
				rprintf("currentState = STATE_GLOW_FULLSCREEN  (Enough Glueh)\n");
			}
			houseKeepingWhileGlowing();
		}
		break;
	case STATE_CHARGING:				//Triggered every 8 s
		if( vSolar < CHARGING_THRESHOLD ){
			currentState = STATE_GLOW_GLUEH;
			pwmTimerOn();
			WDT_FAST_MODE();
		}
		break;
	case STATE_HIGH_BATT:				//Triggered every 8 s
		if( vCap <= HBATT_THRESHOLD - 50 ){		//Check if Battery has discharged to a save value
			rprintf("currentState = STATE_GLOW_GLUEH  (Battery back to normal voltage!)\n");
			for( temp=0; temp<NLEDS; temp++){
				setPwmValue( temp, 0 );
			}
			currentState = STATE_GLOW_GLUEH;
			pwmTimerOn();
			WDT_FAST_MODE();
		}
		break;
	case STATE_LOW_BATT:				//Has been handeled as special case above already
		;
	}
	wdtWakeupCounter++;
}

// Internal readADC routine, does n ADC readings (10 bit each) and returns the sum result
uint16_t readADC( uint8_t mux, uint8_t n ){
	uint16_t result=0;
	uint8_t i, sleepSave;
	CBI( PRR, PRADC );											//Power UP
	ADCSRA = (1<<ADEN)  | (1<<ADIE)  | (1<<ADPS2)  | (1<<ADPS1) | (1<<ADPS0); //Prescaler = 128
	ADMUX =  (1<<REFS1) | (1<<REFS0) | (mux & 0x0F);			//Internal 1.1 V reference
	//CBI( TIMSK0, OCIE0A ); 									//Switch off timer interrupt which could cause preliminary wakeup
	sleepSave = SMCR & 0x0F;									//Backup sleep mode register
	if( !IBI(flags, FLAG_PWM_IS_ON) ){							//Dont change sleep mode if T1 is running (we would deactivate it)
		SLEEP_SET_ADC();
	}
	for( i=0; i<n; i++ ){
		SLEEP();
		while( IBI(ADCSRA,ADSC) ){								//ADC still running, preliminary wakeup, sleep again
			SLEEP();
		}
		result += ADC;
	}
	SMCR = sleepSave;											//Restore sleep mode register
	CBI( ADCSRA, ADEN );										//disable ADC
	SBI( PRR, PRADC );											//Power DOWN
//	SBI( TIMSK0, OCIE0A ); 										//Switch timer interrupt back on
	return result;
}

//Returns storage capacitor voltage in [mV]
uint16_t readVCAP(){
	return( readADC( ADC_MUX_VCAP, 1 ) * VCAP_CONV_FACT_MV / 10 );
}

//Returns solar cell voltage in [mV]
uint16_t readVSolar(){
	return( readADC( ADC_MUX_SOLAR, 1 ) * VCAP_CONV_FACT_MV / 10 );
}

ISR( ADC_vect ){  }
