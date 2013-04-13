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

/* Common setup data constant combinations  */
#define bmREQ_GET_DESCR     USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE     //get descriptor request type
#define bmREQ_SET           USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_DEVICE     //set request type for all but 'set feature' and 'set interface'
#define bmREQ_CL_GET_INTF   USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE     //get interface request type

/* HID requests */
/*
#define bmREQ_HIDOUT        USB_SETUP_HOST_TO_DEVICE|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE
#define bmREQ_HIDIN         USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_CLASS|USB_SETUP_RECIPIENT_INTERFACE
#define bmREQ_HIDREPORT     USB_SETUP_DEVICE_TO_HOST|USB_SETUP_TYPE_STANDARD|USB_SETUP_RECIPIENT_INTERFACE
*/

#define USB_XFER_TIMEOUT    5000    // USB transfer timeout in milliseconds, per section 9.2.6.1 of USB 2.0 spec
#define USB_NAK_LIMIT       32000   // NAK limit for a transfer. 0 means NAKs are not counted
#define USB_RETRY_LIMIT     3       // retry limit for a transfer
#define USB_SETTLE_DELAY    200     // settle delay in milliseconds
#define USB_NAK_NOWAIT      1       // used in Richard's PS2/Wiimote code

#define USB_NUMDEVICES  2           // Number of USB devices

/* USB state machine states */

#define USB_STATE_MASK                                      0xf0

#define USB_STATE_DETACHED                                  0x10	//  16 in Decimal / Char
#define USB_DETACHED_SUBSTATE_INITIALIZE                    0x11    //  17
#define USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE               0x12	//  18
#define USB_DETACHED_SUBSTATE_ILLEGAL                       0x13	//  19
#define USB_ATTACHED_SUBSTATE_SETTLE                        0x20	//  32
#define USB_ATTACHED_SUBSTATE_RESET_DEVICE                  0x30	//  48
#define USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE           0x40	//  64
#define USB_ATTACHED_SUBSTATE_WAIT_SOF                      0x50	//  80
#define USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE    0x60	//  96
#define USB_STATE_ADDRESSING                                0x70	// 112
#define USB_STATE_CONFIGURING                               0x80	// 128
#define USB_STATE_RUNNING                                   0x90	// 144
#define USB_STATE_ERROR                                     0xa0	// 160

// USB Device
typedef struct
{
    usb_endpoint * epinfo;      //device end point information
    uint8_t devclass;          //device class
} usb_deviceRecord;

uint8_t usb_getUsbTaskState(void);
void usb_setUsbTaskState(uint8_t state);

usb_endpoint * usb_getDevTableEntry(uint8_t addr, uint8_t ep);
void usb_setDevTableEntry(uint8_t addr, usb_endpoint * eprecord_ptr);
int usb_controlRequest(usb_device * device, uint8_t requestType, uint8_t request, uint8_t valueLow, uint8_t valueHigh, uint16_t index, uint16_t length, uint8_t * data);
int usb_getString(usb_device *device, uint8_t index, uint8_t languageId, uint16_t length, char * str);
int usb_getConfigurationDescriptor(usb_device * device, uint8_t conf, uint16_t length, uint8_t * data);
uint8_t usb_getStringDescriptor(usb_device * device, unsigned int nbytes, uint8_t index, unsigned int langid, char * dataptr, unsigned int nak_limit);
int usb_setAddress(usb_device * device, uint8_t address);
int usb_setConfiguration(usb_device * device, uint8_t configuration);

#ifdef __cplusplus
}
#endif

#endif //_usb_h_
