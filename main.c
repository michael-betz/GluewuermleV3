/*
 * main.c
 *
 *  Created on: Apr 14, 2014
 *      Author: michael
 *
 *  !!! IOT version with nRF module !!!
 *
 * We will have 4 or 5 fixed glowing patterns as wavetable in flash memory
 *      One is about 4 or 5 s long and rather dim and continous
 *      One is < 1 s long and short but intense (but less likely to be played)
 * WDT measures light conditions and battery voltage every 8 seconds    -- in onWDT()
 *  --> If dark and full, switch WDT to 0.5 s interrupts
 * If less than 4 Blinky sequences are in progress:
 *      There is a certain change that WDT starts a new Blinky sequence -- in onWDT()
 * Blinky
 *  - int8_t pwmChannel                 //Set to a random and FREE LED channel, -1 = inactive Blinky slot
 *  - uint8_t *glowingPatternPointer    //Set to a random pattern with preference for more dim patterns
 *  - uint8_t samplesLeftInPattern      //How many samples left to show in this pattern
 *  - uint8_t blinkySpeed               //Delay between samples, set to a random value with reasonable limits
 * Blinky is updated and shown on the LEDs by setting curPWMvalues[]    -- in onNewFrame()
 *      if samplesLeftInPattern==0:  pwmChannel=-1   --> Blinky slot is inactive
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
 *      if( IBI( value, pwmStep ) ){
 *          SBI( *precalc, pinNumber );
 *      } else {
 *          CBI( *precalc, pinNumber );
 *      }
 *      precalc++;
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
#include "myNRF24.h"
#include "mySPI_avr.h"
#include "nRF24L01.h"

//--------------------------------------------------
// Global variables
//--------------------------------------------------
volatile uint8_t flags = 0;
uint16_t vCap=0, vSolar=0, temperature=0, wdtCountSinceManual=0;

void init(void) {
//--------------------------------------------------
// LED Ports
//--------------------------------------------------
    DDRC   = 0b00001111;    //Set LED pins as output
    DDRD   = 0b11111100;
//--------------------------------------------------
// Power Reduction, switch off every peripheral not needed
//--------------------------------------------------
    PRR    = 0xFF;
    CBI( PRR, PRSPI );                      //Enable SPI
    SBI( MCUCR, PUD );                      //Disable Pull-ups globally
//--------------------------------------------------
// Serial debugging at 38400 baud/s
//--------------------------------------------------
    SBI( DDRD, PD1 );                       //Set UART TX pin as output
    odDebugInit();
//--------------------------------------------------
// Pin change interrupt PCINT1 for nRF24 status
//--------------------------------------------------
    SBI( PCICR, PCIE0 );                    //Enable PCINT0 - PCINT7
    SBI( PCMSK0, PCINT1 );                  //Enable specifically PCINT1
//-----------------------------------------------------------------------
// ADC for measuring CAP voltage with divider: 560k, 160k, on PC5 = ADC5
//-----------------------------------------------------------------------
    ACSR   = (1 << ACD);                    //Switch off Analog comparator
    ADCSRA = 0;                             //Switch off ADC for now
    ADCSRB = 0;
    DIDR0  = 0b00111111;                    //Switch off all digitial port functions on PORTC0 - PORTC5
//-----------------------------------------------------------------------
// Timer1, HiSpeed PWM, Compare Match = Overflow ISR used for LED output
//-----------------------------------------------------------------------
    TCCR1A = 0b00000000;                    //No Port change on compare match
    TIFR1   = (1<<OCF1B)|(1<<OCF1A)|(1<<TOV1);  //Clear interrupt flags
    TIMSK1  = (1 << OCIE1A);                //Enable Interrupt on compare match OCR1A
    OCR1A = 0x00FF;
//----------------------------------------------------------------
// Init independent Watchdog timer to fire Interrupt every  8.0 s
//----------------------------------------------------------------
//Note that WDTON Fuse (force Watchdog) must be == 1 --> unprogrammed
//    wdt_reset();
//    WDTCSR |= (1 << WDE) | (1 << WDCE);       //Unlock safety
//    WDTCSR  = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);                            //Set prescaler to 128 kHz / 1024k = 8s, interrupt mode
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
    for(uint8_t iter=0; iter<=5; iter++){
        for(uint8_t nled=0; nled<NLEDS; nled++){
            setPwmValue(nled, 0xFF);
            _delay_ms(100);
        }
        _delay_ms(3000);
        for(uint8_t nled=0; nled<NLEDS; nled++){
            setPwmValue(nled, 0);
        }
    }
}

int main(){
    init();             //Now Timer0 is running and will generate an interrupt on its first overflow
    rprintf("\n\n---------------------------------------\n");
    rprintf(" Hello world, this is Gluewuermle 2 ! \n");
    rprintf("---------------------------------------\n");
    nRfInitTX();
    nRfHexdump();
    pwmTimerOn();
    sei();
    rprintf(" LED - test ... ");
    ledTest();
    rprintf("OK\n");
    currentState = STATE_GLOW_GLUEH;
    WDT_FAST_MODE();

    while(1){           //We woke up. Figure out what to do (Wake-up sources: T0 compare match, WDT overflow)
        if ( IBI( flags, FLAG_wakeWDT ) ){      //This is called every PWM_CYCLES_PER_FRAME
            onWDT();                            //Update the PWM values, switch OFF T0 to save energy if finished to glow
                                                //(Then we have less wake up interrupts and can do deep power down)
            CBI( flags, FLAG_wakeWDT );         //Clear flag
        }
        if( IBI( flags, FLAG_nRF_RX_DR ) ){     //Read payload from RF module
            onRX();
            CBI( flags, FLAG_nRF_RX_DR );
        }
#if DEBUG_LEVEL > 0
        _delay_ms( 1 );
#endif
        SLEEP();
    }
    return 0;
}

static void sendState()
{
    uint8_t sendBuffer[12];
    int16_t energySum=0;
    sendBuffer[0] = vSolar&0x00FF;
    sendBuffer[1] = (vSolar&0xFF00)>>8;
    sendBuffer[2] = vCap&0x00FF;
    sendBuffer[3] = (vCap&0xFF00)>>8;
    sendBuffer[4] = ticksSinceBlinking&0x00FF;
    sendBuffer[5] = (ticksSinceBlinking&0xFF00)>>8;
    sendBuffer[6] = temperature&0x00FF;
    sendBuffer[7] = (temperature&0xFF00)>>8;
    sendBuffer[8] = currentState;
    sendBuffer[9] = currentFSFlashMode;
    for( uint8_t temp=0; temp<NLEDS; temp++){
        energySum += bugs[temp].availableEnergy;
    }
    sendBuffer[10] = energySum&0x00FF;
    sendBuffer[11] = (energySum&0xFF00)>>8;
    nRfSendBytes( sendBuffer, sizeof(sendBuffer), 0 );  //After sending something we could receive an ACK with payload!
}

//We have received something from the RF module (An Acc payload!)
void onRX(){
    uint8_t nBytes, readBuffer[32], i;
    while( nRfIsRxDataReady() ){
        nBytes = nRfGet_RX_Payload_Width();
        nRfRead_payload( readBuffer, nBytes );
        rprintf("Received %d bytes, rx[0] = %x, ", nBytes, readBuffer[0] );
        if (nBytes == 1) {
            wdtCountSinceManual = 0;
            currentState = STATE_GLOW_MANUAL_CONTROL;
            WDT_FAST_MODE();
            pwmTimerOff();
            newFrameFullscreen( readBuffer[0] );
            sendState();
        } else if (nBytes == 2) {
            g_maximumCoffeeLevel  = readBuffer[0] << 8;
            g_maximumCoffeeLevel |= readBuffer[1] & 0xFF;
            rprintf("g_maximumCoffeeLevel = %d", g_maximumCoffeeLevel);
        } else if (nBytes == NLEDS) {
            wdtCountSinceManual = 0;
            for( i=0; i<nBytes; i++){
                setPwmValue( i, readBuffer[i] );
            }
            currentState = STATE_GLOW_MANUAL_CONTROL_SLOW;
            WDT_SLOW_MODE();
            pwmTimerOn();
        }
        rprintf("\n");
    }
}


// Executed every 4 * 8 s by WDT to keep track of voltage levels
// Executed in any state except STATE_LOW_BATT
// Save the battery from over / undervoltage and switch states
void houseKeeping(){
    static uint8_t callCount=0;
    if( callCount++ < 3 ){
        return;
    }
    callCount = 0;
    vSolar = readVCAP();
    vCap = readVSolar();
    if( !IBI(flags, FLAG_PWM_IS_ON) ){
        temperature = readADC( ADC_MUX_TEMP, 64 );      //max value: 65472
    }
    sendState();
    if ( vCap < LBATT_THRESHOLD ){                      //Go comatose!
        rprintf("currentState = STATE_LOW_BATT  (Battery undervoltage!)  zzZZzzZZ\n");
        currentState = STATE_LOW_BATT;
        WDT_SLOW_MODE();
        pwmTimerOff();
    } else if ( vCap > HBATT_THRESHOLD ){           //Battery is full, avoid overvoltage
        rprintf("currentState = STATE_HIGH_BATT  (Battery overvoltage!)  BUURN!\n");
        currentState = STATE_HIGH_BATT;
        WDT_SLOW_MODE();
        pwmTimerOn();
        for( uint8_t temp=0; temp<NLEDS; temp++){
            setPwmValue( temp, MAX_PWM_VALUE );
        }
    }
    rprintf( "houseKeeping()   Vs = %4d  Vc = %4d  Temp = %4d  State = %d\n", vSolar, vCap, temperature, currentState );
}

//--------------------------------------------------
// WDT = Housekeeping (every 8.0 s)
// Decide if we should be in standby (during day) or glowing (during night)
//  Glowing:  start T0 PWM interrupt and switch to "Idle" sleep mode (Only CPU core is off during sleep)
//  Standby:  stop  T0 PWM interrupt and switch to "Power-down" mode (Main oscillator is off during sleep)
//--------------------------------------------------
void onWDT(){
    static uint16_t wdtWakeupCounter=0x0000;    //Used to do something only every n wakeups
    uint8_t temp;

    if( currentState == STATE_LOW_BATT ){       //Special case, we exit as fast as possible
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
    } else {                                    //Normal operation
        if ( currentState >= FAST_MODE ) {      //WDT triggers with 30 fps
            if ( wdtWakeupCounter > WDT_N_WK_FAST_MODE ) {
                wdtWakeupCounter = 0;
                //--------------------------------------------------------------------
                // Everything below is executed every 4*8 s, if blinking
                //--------------------------------------------------------------------
                houseKeeping();                 //Check for over / undervoltage and react
            }
        } else {                                //WDT triggers once every 8 s
            //--------------------------------------------------------------------
            // Everything below is executed every 4*8 s, if not blinking
            //--------------------------------------------------------------------
            houseKeeping();                     //Check for over / undervoltage and react
            wdtWakeupCounter = 0;
        }
    }

    switch( currentState ){
    //--------------------------------------------------------------------
    // Everything below is specific to a certain state
    //--------------------------------------------------------------------
    case STATE_GLOW_FULLSCREEN:
        newFrameFullscreen( -1 );
        if( wdtWakeupCounter == 0 ){                        //Triggered every 32 s if glowing
            temp = lfsr(8);
            newFrameFullscreen( temp&0x0F );                //Change glowing mode to temp
            if( temp <= 12 ){                               //Get out of Fullscreen glowing mode
                rprintf("currentState = STATE_GLOW_GLUEH  (Enough Fullscreen)\n");
                currentState = STATE_GLOW_GLUEH;
                for( temp=0; temp<NLEDS; temp++){
                    setPwmValue( temp, 0 );
                }
            }
            houseKeepingWhileGlowing();
        }
        break;
    case STATE_GLOW_GLUEH:              //Triggered every 0.03 s
        newFrameGlueh();
        if( wdtWakeupCounter == 0 ){    //every 8 s if glowing
            if( (lfsr(8)<=1) && (vCap>3700) ){  //If vCap > 3700 mV
                currentState = STATE_GLOW_FULLSCREEN;
                pwmTimerOff();
                newFrameFullscreen( 0 );//Switch to BLACK mode
                rprintf("currentState = STATE_GLOW_FULLSCREEN  (Enough Glueh)\n");
            }
            houseKeepingWhileGlowing();
        }
        break;
    case STATE_GLOW_MANUAL_CONTROL:
        newFrameFullscreen( -1 );
    case STATE_GLOW_MANUAL_CONTROL_SLOW:
        if( wdtWakeupCounter == 0 ){    //every 8 s if glowing
            if( wdtCountSinceManual++ > MANUAL_CONTROL_TIMEOUT ){
                rprintf("currentState = STATE_GLOW_GLUEH  (Enough Manual control)\n");
                currentState = STATE_GLOW_GLUEH;
                WDT_FAST_MODE();
                for( temp=0; temp<NLEDS; temp++){
                    setPwmValue( temp, 0 );
                }
            }
        }
        break;
    case STATE_CHARGING:                //Triggered every 8 s
        if( vSolar < CHARGING_THRESHOLD ){
            currentState = STATE_GLOW_GLUEH;
            pwmTimerOn();
            WDT_FAST_MODE();
        }
        break;
    case STATE_HIGH_BATT:               //Triggered every 8 s
        if( vCap <= HBATT_THRESHOLD - 50 ){     //Check if Battery has discharged to a save value
            rprintf("currentState = STATE_GLOW_GLUEH  (Battery back to normal voltage!)\n");
            for( temp=0; temp<NLEDS; temp++){
                setPwmValue( temp, 0 );
            }
            currentState = STATE_GLOW_GLUEH;
            pwmTimerOn();
            WDT_FAST_MODE();
        }
        break;
    case STATE_LOW_BATT:                //Has been handeled as special case above already
        ;
    }
    wdtWakeupCounter++;
}

// Internal readADC routine, does n ADC readings (10 bit each) and returns the sum result
uint16_t readADC( uint8_t mux, uint8_t n ){
    uint16_t result=0;
    uint8_t i, sleepSave;
    CBI( PRR, PRADC );                                          //Power UP
    ADCSRA = (1<<ADEN)  | (1<<ADIE)  | (1<<ADPS2)  | (1<<ADPS1) | (1<<ADPS0); //Prescaler = 128
    ADMUX =  (1<<REFS1) | (1<<REFS0) | (mux & 0x0F);            //Internal 1.1 V reference
    //CBI( TIMSK0, OCIE0A );                                    //Switch off timer interrupt which could cause preliminary wakeup
    sleepSave = SMCR & 0x0F;                                    //Backup sleep mode register
    if( !IBI(flags, FLAG_PWM_IS_ON) ){                          //Dont change sleep mode if T1 is running (we would deactivate it)
        SLEEP_SET_ADC();
    }
    for( i=0; i<n; i++ ){
        SLEEP();
        while( IBI(ADCSRA,ADSC) ){                              //ADC still running, preliminary wakeup, sleep again
            SLEEP();
        }
        result += ADC;
    }
    SMCR = sleepSave;                                           //Restore sleep mode register
    CBI( ADCSRA, ADEN );                                        //disable ADC
    SBI( PRR, PRADC );                                          //Power DOWN
//  SBI( TIMSK0, OCIE0A );                                      //Switch timer interrupt back on
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

// nRF24 interrupt, something happened ( triggered on PCINT1 )
ISR( PCINT0_vect ){
    uint8_t status;
    // uint8_t temp, dataPipe;
    if( IBI(PINB,nRF_PIN_IRQ) ) {   //If the pin is high this was a low to high transition which we must Ignore
        return;
    }
    status = nRfGet_status();   //Otherwise the nRF24 module really triggered an interrupt
    if( IBI(status, TX_DS) ){   //TX finished interrupt
        // temp = nRfRead_register( OBSERVE_TX ) & 0x0F;
        // rprintf("TX_DS: %d retries\n", temp);
//      Data Sent TX FIFO interrupt. Asserted when
//      packet transmitted on TX. If AUTO_ACK is acti-
//      vated, this bit is set high only when ACK is
//      received.
//      Write 1 to clear bit.
    }
    if( IBI(status, RX_DR) ){   //RX data received
        rprintf("RX_DR\n");
//      dataPipe = ( status & 0b00001110 ) >> 1;
//      The RX_DR IRQ is asserted by a new packet arrival event. The procedure for handling this interrupt should
//      be: 1) read payload through SPI, 2) clear RX_DR IRQ, 3) read FIFO_STATUS to check if there are more
//      payloads available in RX FIFO, 4) if there are more data in RX FIFO, repeat from step 1)
        SBI( flags, FLAG_nRF_RX_DR );   //Let main handle the job!
    }
    if( IBI(status, MAX_RT) ){  //Maximum number of TX retries reached
        rprintf("MAX_RT\n");
//      Maximum number of TX retransmits interrupt
//      Write 1 to clear bit. If MAX_RT is asserted it must
//      be cleared to enable further communication.
        // if( nRfIsTxFifoFull() ){        //Okay screw this, enough playing around will empty the FIFO
        //     rprintf("FIFO_CLEAR\n");
        nRfFlush_tx();
        // }
    }
    nRfWrite_register( STATUS, (1<<TX_DS)|(1<<RX_DR)|(1<<MAX_RT) ); //Clear all interrupt flags
    NRF_PWR_DOWN();
}
