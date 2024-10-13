/*
 * pulseCode.h
 *
 *  Created on: May 7, 2014
 *      Author: michael
 */

#ifndef PULSECODE_H_
#define PULSECODE_H_

//--------------------------------------------------
// Global defines for ASM and C
//--------------------------------------------------
#define NBITS                   (13)        //Do Pulse code modulation with N bits resolution
#define MAX_PWM_VALUE          ((1<<NBITS)-1)//Do Pulse code modulation with N bits resolution
#define TCCR1B_VALUE            0b00001001  //For timing pwm periods, CTC mode, CLK = Tsys /  1

#ifdef __ASSEMBLER__

#define FLAG_wakeWDT        1               //Tell main to render a new PWM frame
//-------------------------------------------
// Register definitions
//-------------------------------------------
//This is an ISR, so we have to clean up
//We will not call any subroutines so we can use any register we want
//For caching the first 2 bits for fast output (this is only used when a new cycle is started)
#define _sRegSave   r0
#define _cachePC0   r18
#define _cachePD0   r19
#define _cachePC1   r20
#define _cachePD1   r21
//General purpose
#define _temp0      r22  //Must be push/popped
#define _temp1      r23  //Must be push/popped
#define _temp2      r24  //Must be push/popped
//Reserved for indirect addressing
#define _y          r28
#define _yl         r28
#define _yh         r29

//-------------------------------------------
// ASM macros definitions
//-------------------------------------------
.MACRO DELAY_CYCLES DEL=0
// ---- DELAY start ---
ldi _temp2, \DEL                     // delay for exactly DEL*3 instructions
dec _temp2                           // r0--
brne .-4                             // if not zero, repeat loop
// ---- DELAY end ---
.ENDM

#else


//--------------------------------------------------
// Global variables for C
//--------------------------------------------------
extern volatile uint16_t pwmPortRaw[ NBITS ];    //N Precalculated Pin values for 16 ports to achieve N bit pulse code modulation

//--------------------------------------------------
// Functions
//--------------------------------------------------
// Pseudo functions
#define PWM_TIMER_OFF() {TCCR1B=0;}                 //Switch off timer1
#define PWM_TIMER_ON()  {TCCR1B=TCCR1B_VALUE;}      //Switch on timer1

void pwmTimerOn( void );                            //Callable functions
void pwmTimerOff( void );

void setPwmValue( uint8_t channel, uint16_t value );


#endif
#endif /* PULSECODE_H_ */
