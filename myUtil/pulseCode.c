/*
 * pulseCode.c
 *
 *  Created on: May 7, 2014
 *      Author: michael
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "rprintf.h"
#include "../main.h"
#include "pulseCode.h"

volatile uint16_t pwmPortRaw[ NBITS ];    //N Precalculated Pin values for 16 ports to achieve N bit pulse code modulation
//we start this array at bit6! the lower bits are handled by the ASM routine
uint16_t delayValues[] = { 2*64-5-36, 2*128, 2*256, 2*512, 2*1024, 2*2048, 2*4096, 2*8192, 2*16384L };
// !!! NEVER EVER Access the following global variables from main, they are only for the ISR !!!
volatile uint16_t *portDataPointer = pwmPortRaw;                                    //16 bit pointer to portData entry of interest
volatile uint16_t *portDelayPointer = delayValues;                                  //16 bit pointer to next DELAY value
volatile uint8_t currentBit = NBITS;

//Channel = 0 - 9
void setPwmValue( uint8_t channel, uint16_t value ){
    uint8_t curBit;
	volatile uint16_t *outPointer=pwmPortRaw;
    if( channel >= 6 ){
    	channel += 4;
    }
	for( curBit=0; curBit<NBITS; curBit++ ){
		if( IBI( value, curBit ) ){
			SBI( *outPointer, channel );
		} else {
			CBI( *outPointer, channel );
		}
		outPointer++;
	}
}

void pwmTimerOff(void) {
	if( !IBI(flags, FLAG_PWM_IS_ON) ){
		return;
	}
	CBI(flags, FLAG_PWM_IS_ON);
	rprintf(" PL ");
	PWM_TIMER_OFF();						//Prescaler: 0 Hz --> Timer Off
	PRR |= (1 << PRTIM1);					//Switch Off clock to Timer1
	PORTC = 0;
	PORTD = 0;
	SLEEP_SET_PDOWN();
}

void pwmTimerOn(void) {
	if( IBI(flags, FLAG_PWM_IS_ON) ){
		return;
	}
	SBI(flags, FLAG_PWM_IS_ON);
	rprintf(" PH ");
	PRR &= ~( (1<<PRTIM1) );				//Switch On clock to Timer 1
	TCCR1A = 0b00000000;					//No Port change on compare match
	TIFR1   = (1<<OCF1B)|(1<<OCF1A)|(1<<TOV1);	//Clear interrupt flags
	TIMSK1  = (1 << OCIE1A); 				//Enable Interrupt on compare match OCR1A
	OCR1A = 0x00FF;
	PWM_TIMER_ON();
	SLEEP_SET_IDLE();
}
