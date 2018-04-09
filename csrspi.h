/*
 * csrspi.h
 *
 *  Created on: 2 jan. 2013
 *      Author: Frans-Willem
 */

#ifndef __CSRSPI_H_
#define __CSRSPI_H_

extern volatile uint32_t delaycycles;

extern uint16_t g_nSpeed;
extern uint16_t g_nReadBits;
extern uint16_t g_nWriteBits;
extern uint16_t g_nBcA;
extern uint16_t g_nBcB;

void CsrInit();
void CsrSpiDelay();
int CsrSpiRead(uint16_t nAddress, uint16_t nLength, uint16_t *pnOutput);
void CsrSpiWrite(uint16_t nAddress, uint16_t nLength, uint16_t *pnInput);
int CsrSpiIsStopped();
int CsrSpiBcOperation(uint16_t nOperation);
int CsrSpiBcCmd(uint16_t nLength, uint16_t *pnData);

#endif /* CSRSPI_H_ */
