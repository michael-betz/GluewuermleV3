/*
 * mySPI_avr.h
 *
 *  Created on: May 8, 2014
 *      Author: michael
 */
//#ifdef __AVR_ATmega328P__   //AVR version
#ifndef MYSPI_AVR_H_
#define MYSPI_AVR_H_
#include <avr/io.h>

#define SBI(reg, bit) ( reg |=  ( 1 << bit ) )
#define CBI(reg, bit) ( reg &= ~( 1 << bit ) )
#define IBI(reg, bit) ( reg &   ( 1 << bit ) )

//----------------------------------------
// Defines for the nRF24 module
//----------------------------------------
#define nRF_PIN_IRQ         PB1         // Interrupt notification       (IN)
#define nRF_PIN_CNS         PB2         // SPI inverted chip select     (OUT)
#define nRF_PIN_EN          PB0         // Transceiver enable           (OUT)

#define NRF_CHIP_SELECT()   CBI( PORTB, nRF_PIN_CNS );
#define NRF_CHIP_DESELECT() SBI( PORTB, nRF_PIN_CNS );
#define NRF_CE_ON()         SBI( PORTB, nRF_PIN_EN );
#define NRF_CE_OFF()        CBI( PORTB, nRF_PIN_EN );

void initSPI( void );   //Must be called before doing other stuff
uint8_t nRfRead_registers(uint8_t reg, uint8_t *buf, uint8_t len);
uint8_t nRfRead_register( uint8_t reg );
uint8_t nRfWrite_registers( uint8_t reg, const uint8_t* buf, uint8_t len );
uint8_t nRfWrite_register(uint8_t reg, uint8_t value);
uint8_t nRfWrite_payload( const void* buf, uint8_t len, uint8_t command );
uint8_t nRfRead_payload( void* buf, uint8_t len );
uint8_t nRfFlush_rx(void);
uint8_t nRfFlush_tx(void);
uint8_t nRfGet_status(void);
uint8_t nRfGet_RX_Payload_Width(void);

#endif /* MYSPI_AVR_H_ */
//#endif
