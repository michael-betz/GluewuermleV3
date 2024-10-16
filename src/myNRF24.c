/*
 * myNRF24.c
 *
 *  Created on: May 8, 2014
 *      Author: michael
 */

#include <inttypes.h>
#include <stdio.h>
#ifdef __AVR_ATmega328P__ // AVR version
#define AVR_VERSION
#include "mySPI_avr.h"
#include "rprintf.h"
#include <avr/interrupt.h>
#include <util/delay.h>
#else // Raspi version
#define RASPI_VERSION
#include "mySPI_raspi.h"
#include <bcm2835.h> //Raspi GPIO library
#endif
#include "myNRF24.h"
#include "nRF24L01.h"

uint8_t
    cacheCONFIG; // We cache the config register as global variable, so it does
                 // not need to be read over SPI each time it is changed

void nRfInit() { // Init with default config
  const uint8_t rxTxAdr[5] = {0xE1, 0xE7, 0xE7, 0xE7, 0xE7};
  initSPI();
  NRF_CE_OFF();
  NRF_CHIP_DESELECT();
  _delay_ms(5);
  cacheCONFIG = 0b00001100; // 3xIRQs on, 16bit CRC, PDown, PTX mode
  nRfWrite_register(CONFIG, cacheCONFIG);
  nRfWrite_register(EN_AA, 0b00111111); // Enable auto ACK on pipe0 - pipe5
  nRfWrite_register(EN_RXADDR,
                    0b00000001); // Only enable data pipe ERX_P0 for the start
  nRfWrite_register(SETUP_AW, 0b00000011); // 5 byte address width
  nRfWrite_register(
      SETUP_RETR,
      0x3F); // Automatic retransmit up to 15 times with delay of 1000 us
  nRfWrite_register(RF_CH, 64); // Set RF channel to 64
  nRfWrite_register(RF_SETUP,
                    0b00001111); // 2 Mbps data rate, 0dBm power, LNA_HCURR=1
  nRfWrite_registers(RX_ADDR_P0, rxTxAdr, 5); // Set RX pipe address
  nRfWrite_registers(TX_ADDR, rxTxAdr, 5);    // Set transmit pipe address
  // nRfWrite_register( RX_PW_P0,  6 );            //6 byte static RX payload
  // length (only needed for RX when dyn payload is off)
  nRfWrite_register(DYNPD,
                    0b00111111); // Enable dynamic payload length on all pipes
  nRfWrite_register(
      FEATURE,
      0b00000111); // Enable features: Dynamic payload length, Ack packets with
                   // payload, Disabled Ack for some packets
  nRfFlush_rx();
  nRfFlush_tx();
  nRfWrite_register(STATUS, (1 << TX_DS) | (1 << RX_DR) |
                                (1 << MAX_RT)); // Clear all interrupt flags
}

void nRfInitTX() { // Init for sleeping and sending on demand
  nRfInit();
}

void nRfInitRX() { // Init for continuously receiving data
  nRfInit();
  NRF_RX_MODE();
  NRF_PWR_UP();
  _delay_ms(3); // Wait for Powerup
  NRF_CE_ON();
}

// The RX module monitors the air for packets which match its address
// This is done for 6 data pipes in parallel, which have individual settings
// pipeNumber: ERX_P0 - ERX_P5
void nRfSetupRXPipe(uint8_t pipeNumber, uint8_t *rxAddr) {
  if (pipeNumber <= 5) {
    //      Enable the pipe
    //      ----------------
    uint8_t temp =
        nRfRead_register(EN_RXADDR); // We have to do a read modify write
    SBI(temp, pipeNumber);
    nRfWrite_register(EN_RXADDR, temp); // Enable the data pipe
    //      Set the pipe address
    //      --------------------
    if (pipeNumber <= 1) { // First 2 pipes got a 5 byte address
      nRfWrite_registers(RX_ADDR_P0 + pipeNumber, rxAddr, 5);
    } else { // The other 4 pipes got a 1 byte address
      nRfWrite_registers(RX_ADDR_P0 + pipeNumber, rxAddr, 1);
    }
  }
}

// Transfers data from bytesToSend to the TX FIFO
// len = 1 ... 32
// if noAck == 1 then the sender will not wait for an ack packet, even if ack is
// enabled
void nRfSendBytes(uint8_t *bytesToSend, uint8_t len, uint8_t noAck) {
  NRF_PWR_UP()
  if (noAck) {
    nRfWrite_payload(bytesToSend, len, W_TX_PAYLOAD_NO_ACK);
  } else {
    nRfWrite_payload(bytesToSend, len, W_TX_PAYLOAD);
  }
  _delay_ms(1); // Wait for Powerup
  NRF_CE_ON();  // Strange, this fu<ks up the nRF, propably hardware defect
  _delay_us(50);
  NRF_CE_OFF(); // The nRF will be powered down by the ISR
}

// Transfers data from bytesToSend to the ACC payload FIFO
// Note that ACC payloads are only transmitted in RX mode
// len = 1 ... 32
void nRfSendAccPayload(uint8_t *bytesToSend, uint8_t len, uint8_t pipeNumber) {
  if (pipeNumber > 5) {
    return;
  }
  nRfWrite_payload(bytesToSend, len, (W_ACK_PAYLOAD | pipeNumber));
}

uint8_t nRfIsRxDataReady() {
  //  return( nRfGet_status()&(1<<RX_DR) );                      //Check
  //  interrupt flag
  return (
      !(nRfRead_register(FIFO_STATUS) & (1 << RX_EMPTY))); // Check FIFO status
}

uint8_t nRfIsTxFifoFull() { return ((nRfGet_status() & 0b00000001)); }

// Returns -1 if no new data available (RX FIFO empty),
// otherwise returns the pipe number of current RX FIFO entry where new data is
// available
int8_t nRfgetNextRxPipeNo() {
  uint8_t temp = (nRfGet_status() & 0b00001110) >> 1;
  if (temp >= 7) {
    return (-1);
  }
  return (temp);
}

//------------------------------------------------
// Debug functions
//------------------------------------------------
void nRfHexdump(void) {
  uint8_t x;
  rprintf("All nRF24 registers:\n");
  rprintf("0x00:  ");
  for (x = 0; x <= 0x1D; x++) {
    rprintf("%02x ", nRfRead_register(x));
    if (((x + 1) % 8) == 0) {
      rprintf("\n");
      rprintf("0x%02x:  ", x + 1);
    }
  }
  rprintf("\n\n");
}
