/*
 * csrspi.c
 *
 *  Created on: 2 jan. 2013
 *      Author: Frans-Willem
 * 
 *  Tweaked on: 2 Apr. 2018
 * 		Author: Konsgn
 * 		Now With more STM32F103 targeted
 */
#include "misc.h"
#include "csrspi.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/stm32/gpio.h>

//CSR SPI PINS match BMP
#define PIN_ALL_OUT		PIN_CS|PIN_MOSI|PIN_CLK|PIN_UNK
#define PIN_ALL_IN		PIN_MISO

#define SET_CS()	gpio_set(CSR_PORT_BASE, PIN_CS)
#define CLR_CS()	gpio_clear(CSR_PORT_BASE, PIN_CS)
#define SET_CLK()	gpio_set(CSR_PORT_BASE, PIN_CLK)
#define	CLR_CLK()	gpio_clear(CSR_PORT_BASE, PIN_CLK)
#define SET_MOSI()	gpio_set(CSR_PORT_BASE, PIN_MOSI)
#define CLR_MOSI()	gpio_clear(CSR_PORT_BASE, PIN_MOSI)
#define GET_MISO()	gpio_get(CSR_PORT_BASE, PIN_MISO)

#define g_nFastSpiMode 0 //test to see if it will work

volatile uint32_t delaycycles = 313;
uint16_t g_nSpeed = 393;
uint16_t g_nReadBits = 0;
uint16_t g_nWriteBits = 0;
uint16_t g_nBcA = 0;
uint16_t g_nBcB = 0;
uint16_t g_nUseSpecialRead = 0;

//remapping to stm32lib vocabulary
void GPIOPinWrite(uint32_t GPIOx, uint16_t PinsRelevant, uint16_t PinsSet){
	//gah, this is silly 
	//   uint16_t portout = gpio_port_read(GPIOx); //take outupt values
	//   uint16_t portout = GPIO_ODR(GPIOx);
	//   gpio_port_write(GPIOx,(portout&(~PortClr))|PortSet); // clear the clear bits, and set the set bits
	GPIO_BSRR(GPIOx) = (((PinsRelevant^PinsSet) << 16)|PinsSet); 
		//PinsRelevant^PinsSet gets all the relevant pins that won't be set
}

uint8_t GPIOPinRead(uint32_t GPIOx, uint16_t PortRd){
	return gpio_get(GPIOx,PortRd);
}


void CsrInit() {
	//start of delay testing
	//   rcc_periph_clock_enable(RCC_GPIOC);
	//   gpio_set(GPIOC,GPIO13);
	
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		GPIO2); //PA2 is used for the spi_enable line if needed
	
	gpio_set(GPIOA,GPIO2);
	
	gpio_set_mode(CSR_PORT_BASE, GPIO_MODE_OUTPUT_50_MHZ,
		GPIO_CNF_OUTPUT_PUSHPULL,
		PIN_CS | PIN_MOSI | PIN_CLK | PIN_UNK);
	
	gpio_clear(CSR_PORT_BASE,PIN_UNK);
	
	gpio_set_mode(CSR_PORT_BASE, GPIO_MODE_INPUT,
		GPIO_CNF_INPUT_PULL_UPDOWN,
		PIN_MISO);
		
	gpio_clear(CSR_PORT_BASE, PIN_MISO);
}

void CsrSpiDelay() {
	if (g_nSpeed==4 && g_nFastSpiMode) return; //if usb host has specified 'fast' mode (Setspeed 0x0004) use practically no delay - works fine for me, but YMMV!
	uint32_t tempd = delaycycles;
	while (tempd--)asm("nop");
}

//Equivalent of function at 022D
void CsrSpiStart() {
//022D
//Unsets all outputs pins, sets unk and cs.
	GPIOPinWrite(CSR_PORT_BASE, PIN_ALL_OUT, PIN_UNK | PIN_CS);
	CsrSpiDelay();
//0239
	GPIOPinWrite(CSR_PORT_BASE, PIN_ALL_OUT, PIN_UNK | PIN_CS | PIN_CLK);
	CsrSpiDelay();
//023C
	GPIOPinWrite(CSR_PORT_BASE, PIN_ALL_OUT, PIN_UNK | PIN_CS);
	CsrSpiDelay();
//023F
	GPIOPinWrite(CSR_PORT_BASE, PIN_ALL_OUT, PIN_UNK | PIN_CS | PIN_CLK);
	CsrSpiDelay();
//0242
	GPIOPinWrite(CSR_PORT_BASE, PIN_ALL_OUT, PIN_UNK | PIN_CS);
	CsrSpiDelay();
//0245
	GPIOPinWrite(CSR_PORT_BASE, PIN_ALL_OUT, PIN_UNK);
	CsrSpiDelay();
//0247
	return;
}

//Equivalent of function at 0248
void CsrSpiStop() {
	//Clears MOSI and CLK, sets CS and UNK
	GPIOPinWrite(CSR_PORT_BASE, PIN_ALL_OUT, PIN_UNK|PIN_CS);
}

//Equivalent of function at 0250
//R10 is number of bits
//R0 is what to send?
//Normally R7 and R8 hold the first and second delay function, but I opted for a bool instead
void CsrSpiSendBits(unsigned int nData, unsigned int nBits) {
	//CsrTransfer(nData, (1<<(nBits-1)));
	//return;
	uint16_t nClkLow = 0; //R2
	uint16_t nClkHigh = PIN_CLK; //R3
	nData <<= 24 - nBits; //Could've gone with 32, but hey, who cares
	while (nBits--) {
		GPIOPinWrite(CSR_PORT_BASE, PIN_CLK|PIN_MOSI|PIN_CS, nClkLow);
		if (nData & 0x800000) nClkHigh |= PIN_MOSI;
		nClkLow = nClkHigh - PIN_CLK;
		CsrSpiDelay();
		GPIOPinWrite(CSR_PORT_BASE, PIN_MOSI|PIN_CS, nClkHigh);
		GPIOPinWrite(CSR_PORT_BASE, PIN_CLK|PIN_MOSI|PIN_CS, nClkHigh);
		CsrSpiDelay();
		nClkHigh = PIN_CLK;
		nData <<= 1;
	}
}

//Equivalent of function at 0263
//nBits = R10?
unsigned int CsrSpiReadBits(unsigned int nBits) {
	//return CsrTransfer(0, (1<<(nBits-1)));
	unsigned int nRetval = 0;
	//CSR code disables interrupts here
	while (nBits--) {
		GPIOPinWrite(CSR_PORT_BASE, PIN_CLK, 0);
		CsrSpiDelay();
		nRetval <<= 1;
		GPIOPinWrite(CSR_PORT_BASE, PIN_CLK, PIN_CLK);
		CsrSpiDelay();
		if (GPIOPinRead(CSR_PORT_BASE, PIN_MISO))
			nRetval |= 1;
	}
	//CSR code re-enables interrupts here
	return nRetval & 0xFFFF;
}

//Equivalent of function at 0279
int CsrSpiReadBasic(uint16_t nAddress, uint16_t nLength, uint16_t *pnOutput) {
	nLength--; //Do one loop less, last word is read differently
	CsrSpiStart();
	unsigned int nCommand = ((g_nReadBits | 3) << 16) | nAddress;
	unsigned int nControl;
	CsrSpiSendBits(nCommand, 24);
	/*
	 * Inbetween these commands is an if-statement with some high-speed optimizations for if the spi delay = 0
	 */
	nControl = CsrSpiReadBits(16);
	if (nControl != ((nCommand >> 8) & 0xFFFF)) {
		//In CSR's code, this is at the bottom, 02B7
		CsrSpiStart(); //Not sure why we call this again
		CsrSpiStop();
		return 0;
	}
	//Loop at 296 - 029B
	while (nLength--) {
		*(pnOutput++)=CsrSpiReadBits(16);
	}
	//From 19D to 2A2 some more speed optimized code that we don't need
	uint16_t nLast = CsrSpiReadBits(15);
	GPIOPinWrite(CSR_PORT_BASE, PIN_CLK, 0);
	CsrSpiDelay(); CsrSpiDelay(); CsrSpiDelay(); CsrSpiDelay();
	nLast <<= 1;
	if (GPIOPinRead(CSR_PORT_BASE, PIN_MISO))
		nLast |= 1;
	*(pnOutput++) = nLast;
	CsrSpiStart(); //Not sure why we call this again
	CsrSpiStop();
	return 1;
}

//Equivalent of function at 02BB
int CsrSpiRead(uint16_t nAddress, uint16_t nLength, uint16_t *pnOutput) {
	int nRet = 1;
	if (!g_nUseSpecialRead)
		return CsrSpiReadBasic(nAddress,nLength,pnOutput);
	nLength--;
	nRet &= CsrSpiReadBasic(nAddress, nLength, pnOutput);
	g_nReadBits |= 0x20;
	nRet &= CsrSpiReadBasic(nAddress + nLength, 1, &pnOutput[nLength]);
	g_nReadBits &= ~0x20;
	return nRet;
}

//Equivalent of 02CE
void CsrSpiWrite(uint16_t nAddress, uint16_t nLength, uint16_t *pnInput) {
	CsrSpiStart();
	// Some speed optimizing code from 02D2 to 02D8
	unsigned int nCommand = ((g_nWriteBits | 2) << 16) | nAddress;
	CsrSpiSendBits(nCommand, 24);
	while (nLength--)
		CsrSpiSendBits(*(pnInput++), 16);
	CsrSpiStart();
	CsrSpiStop();
}

int CsrSpiIsStopped() {
	uint16_t nOldSpeed = g_nSpeed;
	g_nSpeed += 32;
	CsrSpiStart();
	g_nSpeed = nOldSpeed;
	unsigned char nRead = GPIOPinRead(CSR_PORT_BASE, PIN_MISO);
	CsrSpiStop();
	if (nRead) return 1;
	return 0;
}

//0x47F
//R1 seems to be used as return value
int CsrSpiBcOperation(uint16_t nOperation /* R2 */) {
	uint16_t var0, var1 = 0;
	int i;
	CsrSpiRead(g_nBcA, 1, &var0);
	CsrSpiWrite(g_nBcB, 1, &nOperation);
	CsrSpiWrite(g_nBcA, 1, &var0);
	for (i=0; i<30; i++) {
		CsrSpiRead(g_nBcB, 1, &var1);
		if (nOperation != var1)
			return var1;
	}
	return var1;
}

int CsrSpiBcCmd(uint16_t nLength, uint16_t *pnData) {
	uint16_t var1,var2;
	int i;
	if (CsrSpiBcOperation(0x7) != 0)
		return 0;
	CsrSpiWrite(g_nBcB + 1, 1, &nLength);
	if (CsrSpiBcOperation(0x1) != 2)
		return 0;
	if (!CsrSpiRead(g_nBcB + 2, 1, &var1))
		return 0;
	CsrSpiWrite(var1, nLength, pnData);
	CsrSpiBcOperation(0x4);
	for (i=0; i<30; i++) {
		if (CsrSpiRead(g_nBcB, 1, &var2) && var2 == 0x6) {
			CsrSpiRead(var1, nLength, pnData);
			CsrSpiBcOperation(0x7);
			return 1;
		}
	}
	return 0;
}
/*
void CsrReset() {
	SET_CS(); CsrDelay();
	SET_CLK(); CsrDelay();
	CLR_CLK(); CsrDelay();
	SET_CLK(); CsrDelay();
	CLR_CLK(); CsrDelay();
}

void CsrStart() {
	CsrReset();
	CLR_CS(); CsrDelay();
}

void CsrStop() {
	SET_CS();
}

unsigned int CsrTransfer(unsigned int nInput, unsigned int msb) {
	unsigned int nOutput = 0;
	while (msb) {
		if (nInput & msb) SET_MOSI();
		else CLR_MOSI();
		msb >>= 1;
		CsrDelay();
		SET_CLK();
		CsrDelay();
		nOutput <<= 1;
		if (GET_MISO()) nOutput |= 1;
		CLR_CLK();
	}
	return nOutput;
}

int CsrRead(uint16_t nAddress, uint16_t nLength, uint16_t *pnOutput) {
	unsigned int nCommand;
	unsigned int nControl;
	nCommand = 3 | (g_nReadBits & ~3);
	CsrStart();
	CsrTransfer(nCommand, 0x80);
	CsrTransfer(nAddress, 0x8000);
	nControl = CsrTransfer(0, 0x8000);
	unsigned int nExpected = ((nCommand & 0xFF) << 8) | ((nAddress >> 8) & 0xFF);
	//USARTSend("Expected: %04X, Control: %04X\n",nExpected,nControl);
	if (nExpected != nControl) {
		CsrStop();
		return 0;
	}
	while (nLength--) {
		*(pnOutput++) = CsrTransfer(0, 0x8000);
	}
	CsrStop();
	return 1;
}

int CsrWrite(uint16_t nAddress, uint16_t nLength, uint16_t *pnInput) {
	unsigned int nCommand;
	nCommand = 2;
	CsrStart();
	CsrTransfer(nCommand, 0x80);
	CsrTransfer(nAddress, 0x8000);
	while (nLength--)
		CsrTransfer(*(pnInput++), 0x8000);
	CsrStop();
	return 1;
}

int CsrStopped() {
	unsigned int nControl, nCheck;
	CsrStart();
	nCheck = CsrTransfer(3, 0x80);
	CsrTransfer(0xFF9A, 0x8000);
	nControl = CsrTransfer(0, 0x8000);
	if (nControl != 0x3FF) {
		return -1;
	}
	return (nCheck ? 1 : 0);
}*/
