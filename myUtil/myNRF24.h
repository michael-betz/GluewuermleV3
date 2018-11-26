/*
 * myNRF24.c
 *
 *  Created on: May 8, 2014
 *      Author: michael
 */

#ifndef MYNRF24_C_
#define MYNRF24_C_
#include <inttypes.h>

extern uint8_t cacheCONFIG;
#define NRF_PWR_UP()   { SBI( cacheCONFIG, PWR_UP);  nRfWrite_register( CONFIG, cacheCONFIG ); }
#define NRF_PWR_DOWN() { CBI( cacheCONFIG, PWR_UP);  nRfWrite_register( CONFIG, cacheCONFIG ); }
#define NRF_RX_MODE()  { SBI( cacheCONFIG, PRIM_RX); nRfWrite_register( CONFIG, cacheCONFIG ); }
#define NRF_TX_MODE()  { CBI( cacheCONFIG, PRIM_RX); nRfWrite_register( CONFIG, cacheCONFIG ); }

void nRfInit();
void nRfInitTX();
void nRfInitRX();
void nRfSendBytes( uint8_t *bytesToSend, uint8_t len, uint8_t noAck );
void nRfSendAccPayload( uint8_t *bytesToSend, uint8_t len, uint8_t pipeNumber );
uint8_t nRfIsRxDataReady();
uint8_t nRfIsTxFifoFull();
int8_t nRfgetNextRxPipeNo();
void nRfHandleISR();

void nRfHexdump( void );

#endif /* MYNRF24_C_ */
