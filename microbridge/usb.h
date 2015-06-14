/*
Copyright 2011 Niels Brouwers

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.#include <string.h>
*/
#ifndef __usb__
#define __usb__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*************** USB Descriptors ***********************/

// Device descriptor.
//		General info about a USB device (vendor ID, etc).
//		Contains info that applies globally to the device.
//		Only one device descriptor.

typedef struct {
	uint8_t  bLength;				// Length of this descriptor.
	uint8_t  bDescriptorType;		// DEVICE descriptor type (USB_DESCRIPTOR_DEVICE).
	uint16_t bcdUSB;				// USB Spec Release Number (BCD).
	uint8_t  bDeviceClass;			// Class code (assigned by the USB-IF). 0xFF-Vendor specific.
	uint8_t  bDeviceSubClass;		// Subclass code (assigned by the USB-IF).
	uint8_t  bDeviceProtocol;		// Protocol code (assigned by the USB-IF). 0xFF-Vendor specific.
	uint8_t  bMaxPacketSize0;		// Maximum packet size for endpoint 0.
	uint16_t idVendor;				// Vendor ID (assigned by the USB-IF).
	uint16_t idProduct;				// Product ID (assigned by the manufacturer).
	uint16_t bcdDevice;				// Device release number (BCD).
	uint8_t  iManufacturer;			// Index of String Descriptor describing the manufacturer.
	uint8_t  iProduct;				// Index of String Descriptor describing the product.
	uint8_t  iSerialNumber;			// Index of String Descriptor with the device's serial number.
	uint8_t  bNumConfigurations;	// Number of possible configurations.
}  __attribute__((packed)) usb_deviceDescriptor;

// Configuration descriptor.
//		USB devices can have multiple configurations
//		Each configuration contains one or more interfaces
//		All associated interface and endpoint descriptors get loaded with a request from the host for the configuration descriptor
//		Contains fields like remote wake-up capability and max power requirements

typedef struct
{
	uint8_t  bLength;				// Length of this descriptor.
	uint8_t  bDescriptorType;		// CONFIGURATION descriptor type (USB_DESCRIPTOR_CONFIGURATION).
	uint16_t wTotalLength;			// Total length of all descriptors for this configuration.
	uint8_t  bNumInterfaces;		// Number of interfaces in this configuration.
	uint8_t  bConfigurationValue;	// Value of this configuration (1 based).
	uint8_t  iConfiguration;		// Index of String Descriptor describing the configuration.
	uint8_t  bmAttributes;			// Configuration characteristics.
	uint8_t  bMaxPower;				// Maximum power consumed by this configuration.
}  __attribute__((packed)) usb_configurationDescriptor;

// Interface descriptor structure (Generic).
//		Lists the endpoint descriptors for the interface
//		Identifies if the interface belongs to a predefined Class (such as as the Human Interface Device or HID)

typedef struct
{
	uint8_t bLength;				// Length of this descriptor.
	uint8_t bDescriptorType;		// INTERFACE descriptor type (USB_DESCRIPTOR_INTERFACE).
	uint8_t bInterfaceNumber;		// Number of this interface (0 based).
	uint8_t bAlternateSetting;		// Value of this alternate interface setting.
	uint8_t bNumEndpoints;			// Number of endpoints in this interface.
	uint8_t bInterfaceClass;		// Class code (assigned by the USB-IF).  0xFF-Vendor specific.
	uint8_t bInterfaceSubClass;		// Subclass code (assigned by the USB-IF).
	uint8_t bInterfaceProtocol;		// Protocol code (assigned by the USB-IF).  0xFF-Vendor specific.
	uint8_t iInterface;				// Index of String Descriptor describing the interface.
}  __attribute__((packed)) usb_interfaceDescriptor;

// Endpoint descriptor structure (Generic).
//		Info required by host to determine bandwidth requirements.
//		Describes endpoint number and address, IN or OUT endpoint and the transfer types requested.

typedef struct
{
	uint8_t  bLength;				// Length of this descriptor.
	uint8_t  bDescriptorType;		// ENDPOINT descriptor type (USB_DESCRIPTOR_ENDPOINT).
	uint8_t  bEndpointAddress;		// Endpoint address. Bit 7 indicates direction (0=OUT, 1=IN).
	uint8_t  bmAttributes;			// Endpoint transfer type.
	uint16_t wMaxPacketSize;		// Maximum packet size.
	uint8_t  bInterval;				// Polling interval in frames.
}  __attribute__((packed)) usb_endpointDescriptor;


/* USB Setup Packet. */

typedef struct
{
    union { // offset   description
            uint8_t bmRequestType;	//   0 Bit-map of request type

            struct {
                    uint8_t recipient : 5; // Recipient of the request
                    uint8_t type 	  : 2; // Type of request
                    uint8_t direction : 1; // Direction of data transfer
            } __attribute__((packed));
    } ReqType_u;

    uint8_t bRequest;               //   1 Request

    union {
            uint16_t wValue;        //   2 Depends on bRequest

            struct {
                    uint8_t wValueLo;
                    uint8_t wValueHi;
            } __attribute__((packed));
    } wVal_u;

    uint16_t wIndex;                //   4 Depends on bRequest
    uint16_t wLength;               //   6 Depends on bRequest

} __attribute__((packed)) usb_setupPacket;


/* USB endpoint */

typedef struct
{
    // Endpoint address. Bit 7 indicates direction (out=0, in=1).
	uint8_t address;

	// Endpoint transfer type.
	uint8_t attributes;

	// Maximum packet size.
    uint16_t maxPacketSize;

    // The max3421e uses these bits to toggle between DATA0 and DATA1.
    uint8_t sendToggle;
    uint8_t receiveToggle;

}  __attribute__((packed)) usb_endpoint;


/* USB device */

typedef struct
{
	// Device address.
	uint8_t address;

	// Indicates whether this device is active.
	uint8_t active;

	// Endpoints.
	usb_endpoint control;
	usb_endpoint bulk_in, bulk_out;

	// First supported language (for retrieving Strings)
	uint16_t firstStringLanguage;

}  __attribute__((packed)) usb_device;


/* USB device record */

typedef struct
{
    usb_endpoint * epinfo;      // device end point information
    uint8_t devclass;           // device class
}  __attribute__((packed)) usb_deviceRecord;


/* Events arising from the USB layer.
 * Things get connected, disconnected or have wrong connections.
 */

typedef enum
{
	USB_CONNECT,
	USB_DISCONNECT,
	USB_ADRESSING_ERROR
} usb_eventType;



typedef void(usb_eventHandler)(usb_device * device, usb_eventType event);

void usb_init();			/* Initialises the USB layer. */
void usb_poll(); 			/* USB main task. Performs enumeration/cleanup. */
void usb_setEventHandler(usb_eventHandler * handler)  __attribute__ ((flatten));
void usb_fireEvent(usb_device * device, usb_eventType event)  __attribute__ ((flatten));

int8_t usb_getDeviceDescriptor(usb_device * device, usb_deviceDescriptor * descriptor) __attribute__ ((flatten));
int8_t usb_printDeviceInfo(usb_device * device);
int8_t usb_initDevice(usb_device * device, uint8_t configuration);

usb_device * usb_getDevice(uint8_t address)  __attribute__ ((flatten));

void usb_initEndPoint(usb_endpoint * endpoint, uint8_t address);

int16_t usb_bulkRead(usb_device * device, uint16_t length, uint8_t * data, uint8_t poll) __attribute__ ((flatten));
int16_t usb_bulkWrite(usb_device * device, uint16_t length, uint8_t * data) __attribute__ ((flatten));


#ifdef __cplusplus
}
#endif

#endif
