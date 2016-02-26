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

/* USB functions */
#ifndef _usb_h_
#define _usb_h_

#ifdef __cplusplus
extern "C" {
#endif

#include "../usb.h"

#define MAX_BUF_SIZE		255		// Maximum size for descriptor buffers using UNICODE in some cases = 126 characters.

#define USB_XFER_TIMEOUT    5000    // USB transfer timeout in milliseconds, per section 9.2.6.1 of USB 2.0 spec
#define USB_NAK_LIMIT       32000   // NAK limit for a transfer. 0 means NAKs are not counted
#define USB_RETRY_LIMIT     3       // retry limit for a transfer
#define USB_SETTLE_DELAY    200     // settle delay in milliseconds
#define USB_NAK_NOWAIT      1       // used in PS2 and Wiimote code

#define USB_NUMDEVICES  	2       // Number of USB devices


/* Common setup data constant combinations  */
#define bmREQ_GET_DESCR     USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE     // get descriptor request type
#define bmREQ_SET           USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE     // set request type for all request types, excluding 'set feature' and 'set interface'
#define bmREQ_CL_GET_INTF   USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE     // get interface request type

/* HID requests */

#define bmREQ_HIDOUT        USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE
#define bmREQ_HIDIN         USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE
#define bmREQ_HIDREPORT     USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_INTERFACE


/* USB Device Classes */

#define USB_CLASS_USE_CLASS_INFO        0x00    // Use Class Info in the Interface Descriptors
#define USB_CLASS_AUDIO                 0x01    // Audio
#define USB_CLASS_COM_AND_CDC_CTRL      0x02    // Communications and CDC Control
#define USB_CLASS_HID                   0x03    // HID
#define USB_CLASS_PHYSICAL              0x05    // Physical
#define USB_CLASS_IMAGE                 0x06    // Image
#define USB_CLASS_PRINTER               0x07    // Printer
#define USB_CLASS_MASS_STORAGE          0x08    // Mass Storage
#define USB_CLASS_HUB                   0x09    // Hub
#define USB_CLASS_CDC_DATA              0x0a    // CDC-Data
#define USB_CLASS_SMART_CARD            0x0b    // Smart-Card
#define USB_CLASS_CONTENT_SECURITY      0x0d    // Content Security
#define USB_CLASS_VIDEO                 0x0e    // Video
#define USB_CLASS_PERSONAL_HEALTH       0x0f    // Personal Healthcare
#define USB_CLASS_DIAGNOSTIC_DEVICE     0xdc    // Diagnostic Device
#define USB_CLASS_WIRELESS_CTRL         0xe0    // Wireless Controller
#define USB_CLASS_MISC                  0xef    // Miscellaneous
#define USB_CLASS_APP_SPECIFIC          0xfe    // Application Specific
#define USB_CLASS_VENDOR_SPECIFIC       0xff    // Vendor Specific


/* USB state machine states */

#define USB_STATE_MASK                                   0xf0

typedef enum
{
	USB_STATE_DETACHED                                 = 0x10,	//  16 in Decimal / Char
	USB_DETACHED_SUBSTATE_INITIALIZE                   = 0x11,  //  17
	USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE              = 0x12,	//  18
	USB_DETACHED_SUBSTATE_ILLEGAL                      = 0x13,	//  19
	USB_ATTACHED_SUBSTATE_SETTLE                       = 0x20,	//  32
	USB_ATTACHED_SUBSTATE_RESET_DEVICE                 = 0x30,	//  48
	USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE          = 0x40,	//  64
	USB_ATTACHED_SUBSTATE_WAIT_SOF                     = 0x50,	//  80
	USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE   = 0x60,	//  96
	USB_STATE_ADDRESSING                               = 0x70,	// 112
	USB_STATE_CONFIGURING                              = 0x80,	// 128
	USB_STATE_RUNNING                                  = 0x90,	// 144
	USB_STATE_ERROR                                    = 0xa0	// 160
} usb_state_t;


/* Additional USB Error Codes */

typedef enum
{
	USB_ERROR_NIL                                   = 0x00,
	USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED       = 0xD1,
	USB_DEV_CONFIG_ERROR_DEVICE_INIT_INCOMPLETE     = 0xD2,
	USB_ERROR_UNABLE_TO_REGISTER_DEVICE_CLASS       = 0xD3,
	USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL          = 0xD4,
	USB_ERROR_HUB_ADDRESS_OVERFLOW                  = 0xD5,
	USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL             = 0xD6,
	USB_ERROR_EPINFO_IS_NULL                        = 0xD7,
	USB_ERROR_INVALID_ARGUMENT                      = 0xD8,
	USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE         = 0xD9,
	USB_ERROR_INVALID_MAX_PKT_SIZE                  = 0xDA,
	USB_ERROR_EP_NOT_FOUND_IN_TBL                   = 0xDB,
	USB_ERROR_CONFIG_REQUIRES_ADDITIONAL_RESET      = 0xE0,
	USB_ERROR_FailGetDevDescr                       = 0xE1,
	USB_ERROR_FailSetDevTblEntry                    = 0xE2,
	USB_ERROR_FailGetConfDescr                      = 0xE3,
	USB_ERROR_TRANSFER_TIMEOUT                      = 0xFF
} usb_error_t;


usb_state_t usb_getUsbTaskState(void) __attribute__ ((flatten));
void usb_setUsbTaskState(usb_state_t state) __attribute__ ((flatten));

usb_endpoint * usb_getDevTableEntry(uint8_t addr, uint8_t ep);
void usb_setDevTableEntry(uint8_t addr, usb_endpoint * eprecord_ptr);
int8_t usb_controlRequest(usb_device * device, uint8_t requestType, uint8_t request, uint8_t valueLow, uint8_t valueHigh, uint16_t index, uint16_t length, uint8_t * data);
int8_t usb_getString(usb_device *device, uint8_t index, uint16_t languageId, uint8_t length, uint8_t * str);
int16_t usb_getConfigurationDescriptor(usb_device * device, uint8_t conf, uint16_t length, uint8_t * data);
uint8_t usb_getStringDescriptor(usb_device * device, uint16_t nbytes, uint8_t index, uint16_t langid, uint8_t * dataptr, uint16_t nak_limit);
int8_t usb_setAddress(usb_device * device, uint8_t address) __attribute__ ((flatten));
int8_t usb_setConfiguration(usb_device * device, uint8_t configuration) __attribute__ ((flatten));

#ifdef __cplusplus
}
#endif

#endif //_usb_h_
