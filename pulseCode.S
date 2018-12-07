#include <avr/io.h>
#include "./myUtil/pulseCode.h"

//-------------------------------------------
// GCC global variable imports
//-------------------------------------------
.extern portDataPointer             //These values are GCC global variables
.extern portDelayPointer            //which I want to access from
.extern currentBit                  //within this ASM file
.extern delayValues
.extern pwmPortRaw
.extern flags

//-------------------------------------------
// A T1 compare match and overflow happened!
//-------------------------------------------
// This is the basic plan of attack:
//
//  if( currentBit < NBITS) {
//      PORTC, PORTD = *xxx //Branch and blast some bits to the ports
//      currentBit++
//  } else {
//      PORTC = 0           //Don;t branch and start a new cycle
//      currentBit = 0
//  }
.global TIMER1_COMPA_vect
TIMER1_COMPA_vect:
    // Save only the most basic context first (SREG save inspired by GCC)
    push    _sRegSave
    in      _sRegSave, _SFR_IO_ADDR(SREG)
    push    _temp0
    push    _temp1
    push    _yl
    push    _yh

    //-------------------------------------------
    // Branch if( currentBit >= NBITS-1 ) to start a new cycle
    //-------------------------------------------
    lds   _temp0, currentBit
    cpi   _temp0, NBITS-1
    brge  isrStartNewCycle               //If condition is true, Branch to the startNewCycle routine

 isrOutputBits:
    //-------------------------------------------
    // currentBit++
    //-------------------------------------------
    inc   _temp0
    sts   currentBit, _temp0

    //-------------------------------------------
    // PORTC = *portDataPointer;
    // PORTD = (*portDataPointer)>>8;
    // portDataPointer++;
    //-------------------------------------------
    // do 16 bit indirect data read into PORTC, PORTD
    lds   _yl, portDataPointer          //Load pointer variable into Y (LOW)
    lds   _yh, portDataPointer+1        //Load pointer variable into Y (HIGH)
    ld    _temp0,   Y+                  //Indirectly access pointer contents (Port Data)
    ld    _temp1,   Y+                  //Indirectly access pointer contents (Port Data)
    out  _SFR_IO_ADDR(PORTC), _temp0    //Output the bits to the PORTS
    out  _SFR_IO_ADDR(PORTD), _temp1    //
    // Store incremented pointer back to global variable
    sts   portDataPointer  , _yl
    sts   portDataPointer+1, _yh

    //-------------------------------------------
    // OCR1A = *nextDelayPointer++;
    //-------------------------------------------
    // do 16 bit indirect data read into OCR1A
    lds   _yl, portDelayPointer         //Load pointer variable into Y (LOW)
    lds   _yh, portDelayPointer+1       //Load pointer variable into Y (HIGH)
    ld    _temp0,   Y+                  //Indirectly access pointer contents (Delay Data)
    ld    _temp1,   Y+                  //Indirectly access pointer contents (Delay Data)
    sts   OCR1AH, _temp1                // Set OCR1A highbyte
    sts   OCR1AL, _temp0                // Set OCR1A lowbyte
    // Store incremented pointer back to global variable
    sts   portDelayPointer  , _yl
    sts   portDelayPointer+1, _yh

endOfISR:
    // Restore the most basic context
    pop     _yh
    pop     _yl
    pop     _temp1
    pop     _temp0
    out     _SFR_IO_ADDR(SREG), _sRegSave
    pop     _sRegSave
    reti

isrStartNewCycle:
    //We have to do some more content saving
    push _cachePC0
    push _cachePD0
    push _cachePC1
    push _cachePD1
    push _temp2
    //-------------------------------------------
    // Otherwise start a new PWM cycle!!!
    //-------------------------------------------
    // PORTC = 0; PORTD = 0; PWM_TIMER_OFF();
    clr _temp2                          //Now _temp2 is our ZERO register for a while
    out  _SFR_IO_ADDR(PORTC), _temp2
    out  _SFR_IO_ADDR(PORTD), _temp2
    sts TCCR1B, _temp2                  //Switch OFF T1
    // TCNT1 = 0;
    sts   TCNT1H, _temp2
    sts   TCNT1L, _temp2
    //TIFR1 = (1<<OCF1A)|(1<<TOV1);     //Clear pending T1 interrupt flags
    ldi   _temp0, (1<<OCF1A)|(1<<TOV1)
    sts   TIFR1, _temp0
    // currentBit = 6;
    ldi   _temp2, 6
    sts   currentBit, _temp2
    // portDelayPointer = delayValues;
    ldi   _yl, lo8(delayValues)         //Load delayValues (address of start of the array)
    ldi   _yh, hi8(delayValues)
    // OCR1A = *portDelayPointer++;     //Delay after the 4th bit has been output (28 cycles)
    ld    _temp0,   Y+                  //Indirectly access pointer contents (Delay Data)
    ld    _temp1,   Y+                  //Indirectly access pointer contents (Delay Data)
    sts   OCR1AH, _temp1                // Set OCR1A highbyte
    sts   OCR1AL, _temp0                // Set OCR1A lowbyte
    sts   portDelayPointer  , _yl       // Store incremented pointer back to global variable
    sts   portDelayPointer+1, _yh

    //-------------------------------------------------
    // fckt. to output the timing critical bit0 - bit6
    //-------------------------------------------------
    //What follows is the ASM routine, which outputs the first 7 bits,
    // then switches on the timer and exits the ISR
    // Okay, we are at the very beginning of a pwm cycle, the LEDs are currently all off
    // And we have some spare time to set up everything we need for the ultrafast pulse
    // code modulation routine. The portCpointer points to an array with the PCM values
    // Which we will cache to make it faster
    //Start by caching the first 2 Bits
    ldi   _yl, lo8(pwmPortRaw)             //Load start of Port-data array in y
    ldi   _yh, hi8(pwmPortRaw)
    ld   _cachePC0,   Y+                   //   read bit0
    ld   _cachePD0,   Y+
    ld   _cachePC1,   Y+                   //   read bit1
    ld   _cachePD1,   Y+
    ld   _temp0,      Y+                   //   read bit2
    ld   _temp1,      Y+
    //Start output routine
    // -----------------
    // bit0, 2 cycles
    // -----------------
    out  _SFR_IO_ADDR(PORTC), _cachePC0   //[1] out cached bit0
    out  _SFR_IO_ADDR(PORTD), _cachePD0   //[2]
    // -----------------
    // bit1, 4 cycles
    // -----------------
    out  _SFR_IO_ADDR(PORTC), _cachePC1  //[1]  out cached bit1
    out  _SFR_IO_ADDR(PORTD), _cachePD1  //[2]
    nop                                  //[3]
    nop                                  //[4]
    // -----------------
    // bit2, 8 cycles
    // -----------------
    out  _SFR_IO_ADDR(PORTC), _temp0     //[1]  out cached bit2
    out  _SFR_IO_ADDR(PORTD), _temp1     //[2]
    ld   _temp0,      Y+                 //[4] read bit3
    ld   _temp1,      Y+                 //[6]
    nop                                  //[7]
    nop                                  //[8]
    // -----------------
    // bit3, 16 cycles
    // -----------------
    out  _SFR_IO_ADDR(PORTC), _temp0     //[1] out bit3
    out  _SFR_IO_ADDR(PORTD), _temp1     //[2]
    ld   _temp0,      Y+                 //[4]read bit4
    ld   _temp1,      Y+                 //[6]
    //Now we got 16-6 = 10 cycles to burn until the next output
    DELAY_CYCLES 3                       // delay for 3*3 = 9 instructions
    nop                                  //[16]
    // -----------------
    // bit4, 32 cycles
    // -----------------
    out  _SFR_IO_ADDR(PORTC), _temp0      //[1] out bit4
    out  _SFR_IO_ADDR(PORTD), _temp1      //[2]
    ld   _temp0,      Y+                 //[4]read bit5
    ld   _temp1,      Y+                 //[6]
    //We got 32-6 = 26 cycles to burn until the next output
    DELAY_CYCLES 8                       // delay for 8*3 = 24 instructions
    nop                                  //[31]
    nop                                  //[32]
    // -----------------
    // bit5, 64 cycles
    // -----------------
    out  _SFR_IO_ADDR(PORTC), _temp0      //[1] out bit5
    out  _SFR_IO_ADDR(PORTD), _temp1      //[2]
    ld   _temp0,      Y+                  //[4]read bit6
    ld   _temp1,      Y+                  //[6]
    //We got 64-6 = 58 cycles to burn until the next output
    DELAY_CYCLES 19                       // delay for 19*3 = 57 instructions
    nop                                   //[64]
    // --------------------------------
    // bit6, 128 cycles (Switch to T1-mode)
    // --------------------------------
    out  _SFR_IO_ADDR(PORTC), _temp0      //[1] out bit5
    out  _SFR_IO_ADDR(PORTD), _temp1      //[2]
    LDI  _temp0,  TCCR1B_VALUE            //[3] Switch on T1, fclk / 1
    sts TCCR1B, _temp0                    //[5] Switch on T1, fclk / 1
    //Timer should cause interrupt after 128-5 = 123 cycles from now
    sts   portDataPointer  , _yl          // store incremented portDataPointer in global variable
    sts   portDataPointer+1, _yh
    //Restore the extended content save
    pop _temp2
    pop _cachePD1
    pop _cachePC1
    pop _cachePD0
    pop _cachePC0

    rjmp endOfISR



// Once every 8 s. Set flag to notify Main routine
    .global WDT_vect
    WDT_vect:       //Trigger new WDT interrupt flag!
    // Save only the most basic context first (SREG save inspired by GCC)
    push    _sRegSave
    in      _sRegSave, _SFR_IO_ADDR(SREG)
    push    _temp0

    lds     _temp0, flags
    sbr     _temp0, (1<<FLAG_wakeWDT)
    sts     flags, _temp0

    pop     _temp0
    out     _SFR_IO_ADDR(SREG), _sRegSave
    pop     _sRegSave
    reti
