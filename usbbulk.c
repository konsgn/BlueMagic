//
#include "misc.h"

#include <libopencm3/usb/usbd.h>
#include <libopencm3/cm3/nvic.h>

static usbd_device *usbd_dev;

const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0110,
	.bDeviceClass = USB_CLASS_VENDOR,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0a12,
	.idProduct = 0x0042,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor data_endp[] = {{
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = usb_incoming_ep_addr,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, 
	.bInterval = 1,
}, {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = usb_outgoing_ep_addr,
	.bmAttributes = USB_ENDPOINT_ATTR_BULK,
	.wMaxPacketSize = 64, 
	.bInterval = 1,
}};

static const struct usb_interface_descriptor data_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 2,
	.bInterfaceClass = USB_CLASS_VENDOR,
	.bInterfaceSubClass = 0,
	.bInterfaceProtocol = 0,
	.iInterface = 4,

	.endpoint = data_endp,
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &data_iface,
}};

static const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 5,
	.bmAttributes = (USB_CONFIG_ATTR_DEFAULT | USB_CONFIG_ATTR_SELF_POWERED),
	.bMaxPower = 0xFA,// technically should only ask for 100mA but no-one cares anymore//spec redacted?

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Cambridge Silicon Radio",
	"CSR Programmer",
	"12345678",
	"Bulk Data Interface",
	"Bulk Data Configuration",
};


uint8_t usbd_in_buff[64];

void TransmitCallback(usbd_device *usbd_dev, uint8_t ep){
	(void)ep;
	size_t nPacket;
	if (bTransmitting) {
		nPacket = nTransmitLength - nTransmitOffset;
		if (nPacket > 64)
			nPacket = 64;
		nTransmitOffset += usbd_ep_write_packet(usbd_dev, usb_outgoing_ep_addr, &pTransmitBuffer[nTransmitOffset], nPacket);
		//   USBDBulkPacketWrite((void *)&g_sBulkDevice, &pTransmitBuffer[nTransmitOffset], nPacket, true);
		if (nTransmitOffset >= nTransmitLength)
			bTransmitting = 0;
	}
}
void StartTransmit() {
	nTransmitOffset = 0;
	bTransmitting = 1;
	TransmitCallback(usbd_dev,usb_outgoing_ep_addr);
}

//   void TxHandler(usbd_device *usbd_dev, uint8_t ep){
	//   (void)ep;
	//   if(bTransmitting){
		//   TransmitCallback(usbd_dev,usb_outgoing_ep_addr);
	//   }
//   }

void WaitTransmit() {
	while (bTransmitting){
		TransmitCallback(usbd_dev,usb_outgoing_ep_addr);
		};
	nTransmitLength = nTransmitOffset = 0;
}

void RxFlow(uint8_t disable){
	usbd_ep_nak_set(usbd_dev,usb_incoming_ep_addr,disable); // disable rx if true
}

void RxHandler(usbd_device *usbd_dev, uint8_t ep){
	(void)ep;
	
	if (!bReceiving)return;
	
	int len = usbd_ep_read_packet(usbd_dev, usb_incoming_ep_addr, usbd_in_buff, 64);
	
	if (nReceiveLength + len > BUFFER_SIZE) {
		printf( "Buffer overflow\n");
		return;
	}
	
	if (len < 64) {
		bReceiving = 0;
		RxFlow(1); // disable reception of data from host.
		//   if ((len>0)){
		if (pDEBUG&&(len>0)){
			printf("RX buf :");
			for (int i=len;i>0;i--)printf("%02X.",usbd_in_buff[len-i]);
			printf("\n");
		}
	}
	
	memcpy(&pReceiveBuffer[nReceiveLength],usbd_in_buff,len);
	nReceiveLength += len;
}

static void usbbulk_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;
	usbd_ep_setup(usbd_dev, usb_incoming_ep_addr, USB_ENDPOINT_ATTR_BULK, 64, RxHandler);
	usbd_ep_setup(usbd_dev, usb_outgoing_ep_addr, USB_ENDPOINT_ATTR_BULK, 64, NULL); //set this up?
	
}

/* We shouldn't need a special large control buffer for this device: */
uint8_t usbd_control_buffer[64];

void USB_Init(){
	nReceiveLength = 0;
	 
	usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config, usb_strings, 5, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, usbbulk_set_config);
	
	nvic_set_priority(USB_IRQ, IRQ_PRI_USB);
	nvic_enable_irq(USB_IRQ);
}

void USB_ISR(void)
{
	usbd_poll(usbd_dev);
}
