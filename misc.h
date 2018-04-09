#ifndef __MISC_H_
#define __MISC_H_


#define _GNU_SOURCE 
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <inttypes.h>
#include <errno.h>
#include <stdio.h>
#include <unistd.h>

extern const bool pDEBUG; 

//usb stuff
#define usb_incoming_ep_addr 0x01
#define usb_outgoing_ep_addr 0x81

#define USB_DRIVER st_usbfs_v1_usb_driver
#define USB_IRQ    NVIC_USB_LP_CAN_RX0_IRQ
#define USB_ISR    usb_lp_can_rx0_isr

//usb buffers
#define BUFFER_SIZE	1024

//usb protocol
void WaitTransmit(void);
void StartTransmit(void);
void USB_Init(void);

typedef unsigned int size_t;

extern unsigned char pReceiveBuffer[BUFFER_SIZE];
extern volatile size_t nReceiveLength;
extern volatile int bReceiving ;

extern unsigned char pTransmitBuffer[BUFFER_SIZE];
extern volatile size_t nTransmitLength;
extern volatile size_t nTransmitOffset;
extern volatile int bTransmitting ;

/* Interrupt priorities.  Low numbers are high priority.
 * For now USART1 preempts USB which may spin while buffer is drained.
 * TIM3 is used for traceswo capture and must be highest priority.
 */
#define IRQ_PRI_USB             (2 << 4)
#define IRQ_PRI_USBUSART        (1 << 4)
#define IRQ_PRI_USBUSART_TIM    (3 << 4)
#define IRQ_PRI_USB_VBUS        (14 << 4)
#define IRQ_PRI_TRACE           (0 << 4)

//pins used on the board: targets the BlackmagicProbe native hardware:
/* Define the STM32F10x hardware depending on the used board 
 *  *
 * LED0 = 	PB2		(Blue LED : Running) //my blackmagic has blue here
 * LED1 = 	PB10	(Yellow LED : Idle)
 * LED2 = 	PB11	(Red LED    : Error)
 *
 * TPWR = 	RB0 (input) -- analogue on mini design ADC1, ch8
 * nTRST = 	PB1 (output) [blackmagic] enable power out
 * PWR_BR = 	PB1 (output) [blackmagic_mini] -- supply power to the target, active low
 * SRST_OUT = SPIEN = 	PA2 (output)
 * TDI = MOSI =	PA3 (output) 
 * TMS = CS	=	PA4 (input/output for SWDIO)
 * TCK = CLK =	PA5 (output SWCLK)
 * TDO = MISO =	PA6 (input)
 *
 * USB cable pull-up: PA8
 * USB VBUS detect:  PB13 -- New on mini design.
 *                           Enable pull up for compatibility.
 * Force DFU mode button: PB12 TODO: enable fast bit-bang csr spi
 */

/* Hardware definitions... */
#define PWR_BR_PORT	GPIOB
#define PWR_BR_PIN	GPIO1

#define USB_PU_PORT	GPIOA
#define USB_PU_PIN	GPIO8

#define USB_VBUS_PORT	GPIOB
#define USB_VBUS_PIN	GPIO13
#define USB_VBUS_IRQ	NVIC_EXTI15_10_IRQ

#define LED_PORT	GPIOB
#define LED_PORT_UART	GPIOB
//   #define LED_0		GPIO2
//   #define LED_1		GPIO10
//   #define LED_2		GPIO11
//   #define LED_UART	LED_0
//   #define LED_IDLE_RUN	LED_1
//   #define LED_ERROR	LED_2
#define GPIO_PIN_LED_R		GPIO11
#define GPIO_PIN_LED_G		GPIO10 //yellow on black magic
#define GPIO_PIN_LED_B		GPIO2

#define GPIO_PIN_TX			GPIO9
#define GPIO_PIN_RX			GPIO10 

//CSR SPI redefines
#define CSR_PORT_BASE	GPIOA
#define PIN_CS			GPIO4 //Equivalent of pin 0x1 in CSR code
#define PIN_MISO		GPIO6 //Equivalent of pin 0x2 in CSR code
#define PIN_MOSI		GPIO3 //Equivalent of pin 0x4 in CSR code
#define PIN_CLK			GPIO5 //Equivalent of pin 0x8 in CSR code
#define PIN_UNK			GPIO2 //Equivalent to pin 0x10 in CSR code


#define ALIGN(x, n) (((x) + (n) - 1) & ~((n) - 1))
#undef MIN
#define MIN(x, y)  (((x) < (y)) ? (x) : (y))
#undef MAX
#define MAX(x, y)  (((x) > (y)) ? (x) : (y))

#endif
