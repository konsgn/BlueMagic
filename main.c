#include "misc.h"
#include "csrspi.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

//Printf debug info?
const bool pDEBUG = 0;

#define MODE_SPI	0
#define MODE_JTAG	0xFFFF;
uint16_t g_nMode = MODE_SPI;

#define CMD_READ		0x0100
#define CMD_WRITE		0x0200
#define CMD_SETSPEED	0x0300
#define CMD_GETSTOPPED	0x0400
#define CMD_GETSPEED	0x0500
#define CMD_UPDATE		0x0600
#define CMD_GETSERIAL	0x0700
#define CMD_GETVERSION	0x0800
#define CMD_SETMODE		0x0900
#define CMD_SETBITS		0x0F00
#define CMD_BCCMDINIT	0x4000
#define CMD_BCCMD		0x4100

unsigned char pReceiveBuffer[BUFFER_SIZE];
volatile size_t nReceiveLength;
volatile int bReceiving = 1;

unsigned char pTransmitBuffer[BUFFER_SIZE];
volatile size_t nTransmitLength;
volatile size_t nTransmitOffset;
volatile int bTransmitting = 0;


void WriteWord(unsigned char *szOffset, uint16_t n) {
	szOffset[1] = n & 0xFF;
	szOffset[0] = (n >> 8) & 0xFF;
}

uint16_t ReadWord(unsigned char *szOffset) {
	return (szOffset[0] << 8) | szOffset[1];
}

void TransmitWord(uint16_t n) {
	if (nTransmitLength + 2 < BUFFER_SIZE) {
		WriteWord(&pTransmitBuffer[nTransmitLength], n);
		nTransmitLength += 2;
	}
}

void TransmitDWord(uint32_t n) {
	TransmitWord((n >> 16) & 0xFFFF);
	TransmitWord(n & 0xFFFF);
}

int CmdRead(uint16_t nAddress, uint16_t nLength);
int CmdWrite(uint16_t nAddress, uint16_t nLength, unsigned char *pData);
int CmdSetSpeed(uint16_t nSpeed);
int CmdGetStopped();
int CmdGetSpeed();
int CmdUpdate();
int CmdGetSerial();
int CmdGetVersion();
int CmdSetMode(uint16_t nMode);
int CmdSetBits(uint16_t nWhich, uint16_t nValue);
void CmdBcCmdInit(uint16_t nA, uint16_t nB);
int CmdBcCmd(uint16_t nLength, unsigned char *pData);

//   void WaitTransmit() {
	//   while (bTransmitting);
	//   nTransmitLength = nTransmitOffset = 0;
//   }


int _write(int file, char *ptr, int len);

/**
 * Use USART1 as a console.
 * This is a syscall for newlib
 * @param file
 * @param ptr
 * @param len
 * @return
 */
int _write(int file, char *ptr, int len)
{
	int i;

	if (file == STDOUT_FILENO || file == STDERR_FILENO) {
		for (i = 0; i < len; i++) {
			if (ptr[i] == '\n') {
				usart_send_blocking(USART1, '\r');
			}
			usart_send_blocking(USART1, ptr[i]);
		}
		return i;
	}
	errno = EIO;
	return -1;
}


void exti15_10_isr(void)
{
	
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	
	//   if (gpio_get(USB_VBUS_PORT, USB_VBUS_PIN)) {
		//   /* Drive pull-up high if VBUS connected */
		//   gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_10_MHZ,
				//   GPIO_CNF_OUTPUT_PUSHPULL, USB_PU_PIN);
	//   } else {
		//   /* Allow pull-up to float if VBUS disconnected */
		//   gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT,
				//   GPIO_CNF_INPUT_FLOAT, USB_PU_PIN);
	//   }
	exti_reset_request(USB_VBUS_PIN);
}

/*
 * main.c
 */
int main(void) {
	size_t nOffset;
	uint16_t nCommand, nArgs[4];
	
	//TAKEN FROM BLACKMAGIC NATIVE
	SCS_DEMCR |= SCS_DEMCR_VC_MON_EN;
	
	//enable 72 mHz clock
	rcc_clock_setup_in_hse_8mhz_out_72mhz();

	/* Enable peripherals */
	rcc_periph_clock_enable(RCC_USB);
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_AFIO); 
	rcc_periph_clock_enable(RCC_CRC);
	rcc_periph_clock_enable(RCC_USART1);


	/* Setup GPIO ports */
	gpio_clear(USB_PU_PORT, USB_PU_PIN);
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_INPUT, GPIO_CNF_OUTPUT_PUSHPULL,
			USB_PU_PIN);

	//   /* This needs some fixing... */
	//   /* Toggle required to sort out line drivers... */
	//   gpio_port_write(GPIOA, 0x8102);
	//   gpio_port_write(GPIOB, 0x2000);

	//   gpio_port_write(GPIOA, 0x8182);
	//   gpio_port_write(GPIOB, 0x2002);
	//can we get away without?? 

	gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_2_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL,
			GPIO_PIN_LED_R | GPIO_PIN_LED_G | GPIO_PIN_LED_B);

	/* Enable power on PWR_BR so that we do drive
	   TPWR locally & advertently supply power to the target. */
	gpio_clear(PWR_BR_PORT, PWR_BR_PIN);
	gpio_set_mode(PWR_BR_PORT, GPIO_MODE_OUTPUT_2_MHZ,
				  GPIO_CNF_OUTPUT_OPENDRAIN, PWR_BR_PIN);

	gpio_clear(GPIOB, GPIO0);
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO0);

	// set GPIO of usb to low so that the device resets from usb initialization
	gpio_clear(GPIOA, GPIO11);
	gpio_clear(GPIOA, GPIO12);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
			GPIO_CNF_OUTPUT_PUSHPULL, GPIO11);
			
	/* Relocate interrupt vector table here */
	extern int vector_table;
	SCB_VTOR = (uint32_t)&vector_table;

	//   platform_timing_init();
	//   cdcacm_init();

	//setup_vbus_irq();
	nvic_set_priority(USB_VBUS_IRQ, IRQ_PRI_USB_VBUS);
	nvic_enable_irq(USB_VBUS_IRQ);

	gpio_set(USB_VBUS_PORT, USB_VBUS_PIN);

	gpio_set_mode(USB_VBUS_PORT, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_PULL_UPDOWN, USB_VBUS_PIN);

	/* Configure EXTI for USB VBUS monitor */
	exti_select_source(USB_VBUS_PIN, USB_VBUS_PORT);
	exti_set_trigger(USB_VBUS_PIN, EXTI_TRIGGER_BOTH);
	exti_enable_request(USB_VBUS_PIN);

	exti15_10_isr();
	//setup_vbus_irq();
	
	/* Setup GPIO pin GPIO_USART1_TX. */
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
		      GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);	
	
	//Do some output!
	/* Setup UART parameters. */
	usart_set_baudrate(USART1, 115200);
	usart_set_databits(USART1, 8);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_parity(USART1, USART_PARITY_NONE);
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
	usart_set_mode(USART1, USART_MODE_TX);
	usart_enable(USART1);
    if(pDEBUG)printf("Hello.\r\nUSART1 is ready.\r\n");
    
	//Csr SPI init
	CsrInit();
	if(pDEBUG)printf( "CSR initialized\n");

	//Status LEDs Init
	gpio_clear(LED_PORT, GPIO_PIN_LED_R);
	gpio_clear(LED_PORT, GPIO_PIN_LED_G);
	gpio_clear(LED_PORT, GPIO_PIN_LED_B);
	if(pDEBUG)printf( "LEDs initialized\n");


	/* Setup GPIO ports */
	gpio_set(USB_PU_PORT, USB_PU_PIN);
	gpio_set_mode(USB_PU_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
			USB_PU_PIN);
			
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO12);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT,
			GPIO_CNF_INPUT_FLOAT, GPIO11);
	USB_Init();
	  
    if(pDEBUG)printf( "Waiting for host...\n");

	while (1) {
		while (bReceiving){
			//testing code:
			//   WaitTransmit();
			//   CsrSpiDelay();
			//   gpio_toggle(GPIOC,GPIO13);
			}; //Wait until we're ready receiving data
		if (nReceiveLength == 0) {
			if(pDEBUG)printf( "Empty packet...\n");
		} else {
			WaitTransmit(); 
			if (pReceiveBuffer[0] != '\0') {
				if(pDEBUG)printf( "Invalid start byte\n");
			} else {
				nOffset = 1;
				while (nReceiveLength >= nOffset + 2) {
					nCommand = ReadWord(&pReceiveBuffer[nOffset]);
					
					if(pDEBUG)printf( "Command: 0x%04X\n",nCommand);
					
					nOffset += 2;
					switch (nCommand) {
					case CMD_READ:
						if (nReceiveLength < nOffset + 4) {
							if(pDEBUG)printf( "Too few arguments to read\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							nArgs[1] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							CmdRead(nArgs[0], nArgs[1]);
						}
						break;
					case CMD_WRITE:
						if (nReceiveLength < nOffset + 4) {
							if(pDEBUG)printf( "Too few arguments to write\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							nArgs[1] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							if (nReceiveLength < nOffset + (nArgs[1] * 2)) {
								if(pDEBUG)printf( "Too few arguments to write\n");
							} else {
								CmdWrite(nArgs[0], nArgs[1], &pReceiveBuffer[nOffset]);
								nOffset+=nArgs[1] * 2;
							}
						}
						break;
					case CMD_SETSPEED:
						if (nReceiveLength < nOffset + 2) {
							if(pDEBUG)printf( "Too few arguments to set speed\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							CmdSetSpeed(nArgs[0]);
						}
						break;
					case CMD_GETSTOPPED:
						CmdGetStopped();
						break;
					case CMD_GETSPEED:
						CmdGetSpeed();
						break;
					case CMD_UPDATE:
						CmdUpdate();
						break;
					case CMD_GETSERIAL:
						CmdGetSerial();
						break;
					case CMD_GETVERSION:
						CmdGetVersion();
						break;
					case CMD_SETMODE:
						if (nReceiveLength < nOffset + 2) {
							if(pDEBUG)printf( "Too few arguments to set mode\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							CmdSetMode(nArgs[0]);
						}
						break;
					case CMD_SETBITS:
						if (nReceiveLength < nOffset + 4) {
							if(pDEBUG)printf( "Too few arguments to set bits\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							nArgs[1] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							CmdSetBits(nArgs[0], nArgs[1]);
						}
						break;
					case CMD_BCCMDINIT:
						if (nReceiveLength < nOffset + 4) {
							if(pDEBUG)printf( "Too few arguments to init bccmd\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							nArgs[1] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							CmdBcCmdInit(nArgs[0], nArgs[1]);
						}
						break;
					case CMD_BCCMD:
						if (nReceiveLength < nOffset + 2) {
							if(pDEBUG)printf( "Too few arguments to bccmd\n");
						} else {
							nArgs[0] = ReadWord(&pReceiveBuffer[nOffset]);
							nOffset += 2;
							if (nReceiveLength < nOffset + (nArgs[0] * 2)) {
								if(pDEBUG)printf( "Too few arguments to bccmd\n");
							} else {
								CmdBcCmd(nArgs[0], &pReceiveBuffer[nOffset]);
								nOffset+=nArgs[0] * 2;
							}
						}
						break;
					default:
						if(pDEBUG)printf( "Unknown command: %04X\n", nCommand);
						nOffset = nReceiveLength;
						continue;
					}
					nOffset += 2; //Read the two inbetween bytes
				}
			}
			if (nTransmitLength){
				StartTransmit();
				WaitTransmit();
			}
		}
		//Get ready for the next packet
		if(pDEBUG)printf("PacketProcessed\n");
		nReceiveLength = 0;
		bReceiving = 1;
		RxFlow(0);//enable reception of data.
	}
}

//   unsigned long
//   TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
          //   void *pvMsgData)
//   {
    //   if(ulEvent == USB_EVENT_TX_COMPLETE)
    //   {
    	//   TransmitCallback();
    //   }
    //   return(0);
//   }

uint16_t pCsrBuffer[1024];


int CmdRead(uint16_t nAddress, uint16_t nLength) {
	uint16_t *pCurrent;
	gpio_set(LED_PORT, GPIO_PIN_LED_G);
	if (g_nMode == MODE_SPI && nLength < 1024 && CsrSpiRead(nAddress,nLength,pCsrBuffer)) {
		gpio_clear(LED_PORT, GPIO_PIN_LED_G);
		TransmitWord(CMD_READ);
		TransmitWord(nAddress);
		TransmitWord(nLength);
		pCurrent = pCsrBuffer;
		while (nLength--) {
			TransmitWord(*(pCurrent++));
		}
	} else {
		gpio_clear(LED_PORT, GPIO_PIN_LED_G);
		if(pDEBUG)printf( "Read Failed\n");
		TransmitWord(CMD_READ + 1);
		TransmitWord(nAddress);
		TransmitWord(nLength);
		while (nLength--)
			TransmitWord(0);
	}
	return 1;
}

int CmdWrite(uint16_t nAddress, uint16_t nLength, unsigned char *pData) {
	if (nLength > 1024 || g_nMode != MODE_SPI)
		return 0;
	uint16_t i;
	for (i = 0; i < nLength; i++) {
		pCsrBuffer[i] = ReadWord(&pData[i * 2]);
	}
	gpio_set(LED_PORT, GPIO_PIN_LED_R);
	CsrSpiWrite(nAddress, nLength, pCsrBuffer);
	gpio_clear(LED_PORT, GPIO_PIN_LED_R);
	return 1;
}
int CmdSetSpeed(uint16_t nSpeed) {
    //   if(pDEBUG)printf( "Set speed:0d%02d\n",nSpeed);
	g_nSpeed = nSpeed;
	int kHz=0;
	kHz = 1000000 / (126 * g_nSpeed + 434);
	uint32_t count = ((72000000/2000)/kHz); //was 6000 now 2000
	delaycycles = count;
    //   if(pDEBUG)printf( "Delay Cycles:0d%04d\n",(unsigned int)delaycycles);
	return 1;
}
int CmdGetStopped() {
	TransmitWord(CMD_GETSTOPPED);
	TransmitWord(g_nMode != MODE_SPI || CsrSpiIsStopped()); //TODO
	return 1;
}
int CmdGetSpeed() {
	TransmitWord(CMD_GETSPEED);
	TransmitWord(g_nSpeed);
	return 1;
}
int CmdUpdate() {
	return 1;
}
int CmdGetSerial() { //command 0x0700
	TransmitWord(CMD_GETSERIAL);
	TransmitDWord(31337);
	return 1;
}
int CmdGetVersion() { // command 0x0800
	TransmitWord(CMD_GETVERSION);
	TransmitWord(0x119);
	return 1;
}
int CmdSetMode(uint16_t nMode) {
	g_nMode = nMode;
	return 1;
}
int CmdSetBits(uint16_t nWhich, uint16_t nValue) {
	if (nWhich) g_nWriteBits = nValue;
	else g_nReadBits = nValue;
	return 1;
}

void CmdBcCmdInit(uint16_t nA, uint16_t nB) {
	if(pDEBUG)printf( "BCCMD Init: %04X %04X\n", nA, nB);
	g_nBcA = nA;
	g_nBcB = nB;
}

int CmdBcCmd(uint16_t nLength, unsigned char *pData) {
	uint16_t i;
	//if(pDEBUG)printf( "BCCMD input:\n");
	for (i = 0; i < nLength; i++) {
		pCsrBuffer[i] = ReadWord(&pData[i*2]);
	}
	gpio_set(LED_PORT, GPIO_PIN_LED_B);
	if (CsrSpiBcCmd(nLength, pCsrBuffer)) {
		TransmitWord(CMD_BCCMD);
	} else {
		TransmitWord(CMD_BCCMD + 1);
	}
	gpio_clear(LED_PORT, GPIO_PIN_LED_B);
	TransmitWord(nLength);
	for (i=0; i<nLength; i++) {
		TransmitWord(pCsrBuffer[i]);
	}
	return 1;
}

//   unsigned long RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
//   {
    //   switch(ulEvent)
    //   {
        //   case USB_EVENT_CONNECTED:
        //   {
        	//   if (bReceiving) {
        		//   nReceiveLength = 0;
        	//   }
            //   if(pDEBUG)printf( "Connected...\n");
            //   break;
        //   }
        //   case USB_EVENT_DISCONNECTED:
        //   {
        	//   if(pDEBUG)printf( "Disconnected...\n");
            //   break;
        //   }
        //   case USB_EVENT_RX_AVAILABLE:
        //   {
        	//   if (bReceiving) {
        		//   if (nReceiveLength + ulMsgValue > BUFFER_SIZE) {
        			//   if(pDEBUG)printf( "Buffer overflow\n");
        			//   return 0;
        		//   }
        		//   nReceiveLength += USBDBulkPacketRead((void *)&g_sBulkDevice, &pReceiveBuffer[nReceiveLength], ulMsgValue, true);
        		//   if (ulMsgValue < 64) {
        			//   bReceiving = 0;
        		//   }
        		//   return ulMsgValue;
        	//   }
        	//   return 0;
        //   }
        //   case USB_EVENT_SUSPEND:
        //   case USB_EVENT_RESUME:
        //   {
            //   break;
        //   }
        //   default:
        //   {
            //   break;
        //   }
    //   }

    //   return(0);
//   }
