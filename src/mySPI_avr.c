/*
 * mySPI_avr.c
 *
 *  Created on: May 8, 2014
 *      Author: michael
 */

#ifdef __AVR_ATmega328P__ // AVR version

#include "mySPI_avr.h"
#include "nRF24L01.h"
#include "rprintf.h"
#include <avr/io.h>
#include <inttypes.h>
#include <util/delay.h>

//------------------------------------------------
// Low level functions for SPI communication
//------------------------------------------------

//--------------------------------------------------
// Init SPI and GPIO for nRF24 module
//--------------------------------------------------
void initSPI(void) {
	SBI(DDRB, PB3);			// OUT (MOSI)
	CBI(DDRB, PB4);			// IN  (MISO)
	SBI(DDRB, PB5);			// OUT (SCK)
	CBI(DDRB, nRF_PIN_IRQ); // IN  (IRQ)
	SBI(DDRB, nRF_PIN_CNS); // OUT (CSN)
	SBI(DDRB, nRF_PIN_EN);	// OUT (EN)
	// Enable SPI, Master, set clock divider /4 = 2 MHz
	SPCR = (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (0 << SPR0);
}

// Send and receive a single byte over SPI
static inline uint8_t spiTXRXbyte(uint8_t cData) {
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while (!IBI(SPSR, SPIF)) {
		;
	}
	return SPDR;
}

// Do a generalized command and data read from the nRF module
static uint8_t nRfRead_transaction(uint8_t command, uint8_t *readData,
								   uint8_t len) {
	uint8_t status;
	NRF_CHIP_SELECT();
	status = spiTXRXbyte(command);
	while (len--) {
		*readData = spiTXRXbyte(0xFF);
		readData++;
	}
	NRF_CHIP_DESELECT();
	return status;
}

// Do a generalized command and data write to the nRF module
static uint8_t nRfWrite_transaction(uint8_t command, const uint8_t *writeData,
									uint8_t len) {
	uint8_t status;
	NRF_CHIP_SELECT();
	status = spiTXRXbyte(command);
	while (len--) {
		spiTXRXbyte(*writeData++);
	}
	NRF_CHIP_DESELECT();
	return status;
}

// Read several registers, Returns status
uint8_t nRfRead_registers(uint8_t reg, uint8_t *buf, uint8_t len) {
	return nRfRead_transaction(R_REGISTER | (REGISTER_MASK & reg), buf, len);
}

// Read a single registers, Returns its value
uint8_t nRfRead_register(uint8_t reg) {
	uint8_t temp = 0xFF;
	nRfRead_transaction(R_REGISTER | (REGISTER_MASK & reg), &temp, 1);
	return temp;
}

// Write several registers, Returns status
uint8_t nRfWrite_registers(uint8_t reg, const uint8_t *buf, uint8_t len) {
	return nRfWrite_transaction(W_REGISTER | (REGISTER_MASK & reg), buf, len);
}

// Write a single register, Returns status
uint8_t nRfWrite_register(uint8_t reg, uint8_t value) {
	return nRfWrite_transaction(W_REGISTER | (REGISTER_MASK & reg), &value, 1);
}

// Do a command like write data payload, return status
uint8_t nRfWrite_payload(const void *buf, uint8_t len, uint8_t command) {
	return nRfWrite_transaction(command, buf, len);
}

// Read data payload
uint8_t nRfRead_payload(void *buf, uint8_t len) {
	return nRfRead_transaction(R_RX_PAYLOAD, buf, len);
}

uint8_t nRfFlush_rx(void) { return nRfWrite_transaction(FLUSH_RX, 0, 0); }

uint8_t nRfFlush_tx(void) { return nRfWrite_transaction(FLUSH_TX, 0, 0); }

uint8_t nRfGet_status(void) { return nRfWrite_transaction(NOP, 0, 0); }

uint8_t nRfGet_RX_Payload_Width(void) {
	uint8_t plw;
	nRfRead_transaction(R_RX_PL_WID, &plw, 1);
	;
	if (plw > 32) { // Datasheet says: Flush RX FIFO if the read value is larger
					// than 32 bytes.
		nRfFlush_rx();
		return 0;
	}
	return plw;
}

#endif
