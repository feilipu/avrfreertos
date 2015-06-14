/*
Copyright 2015 Phillip Stevens

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
#ifndef __hid_h__
#define __hid_h__

#ifdef __cplusplus
extern "C" {
#endif

#define HID_USB_PACKETSIZE 			8

#define HID_CONNECTION_RETRY_TIME	1000 / portTICK_PERIOD_MS // should be 1000mS

/*-------------------------------------------------------------------------*/

/* HID constants.
 *
 * Not part of chapter 9 of the USB 2.0 specification
 */

/* HID Interface Class Code */
#define HID_INTF                    0x03

/* HID Interface Class SubClass Codes */
#define BOOT_INTF_SUBCLASS          0x01

/* HID Interface Class Protocol Codes */
#define HID_PROTOCOL_NONE           0x00
#define HID_PROTOCOL_KEYBOARD       0x01
#define HID_PROTOCOL_MOUSE          0x02

/* Class Descriptor Types */
#define HID_DESCRIPTOR_HID     		0x21
#define HID_DESCRIPTOR_REPORT   	0x22
#define HID_DESRIPTOR_PHY       	0x23

/* Class-Specific Requests */
#define HID_REQUEST_GET_REPORT      0x01
#define HID_REQUEST_GET_IDLE        0x02
#define HID_REQUEST_GET_PROTOCOL    0x03
// Reserved 0x04 - 0x08
#define HID_REQUEST_SET_REPORT      0x09
#define HID_REQUEST_SET_IDLE        0x0A
#define HID_REQUEST_SET_PROTOCOL    0x0B

/* Protocol Selection */
#define BOOT_PROTOCOL  				0x00
#define RPT_PROTOCOL    			0x01


/* Keyboard META Keys */
#define CTRL_L						0x01
#define CTRL_R						0x10
#define SHFT_L						0x02
#define SHFT_R						0x20
#define ALT_L						0x04
#define ALT_R						0x40
#define META_L						0x08
#define META_R						0x80


/* HID Descriptor Keyboard & Mouse */

typedef struct {
        uint8_t  bLength;
        uint8_t  bDescriptorType;
        uint16_t bcdHID; // HID class specification release
        uint8_t  bCountryCode;
        uint8_t  bNumDescriptors; // Number of additional class specific descriptors
        uint8_t  bDescrType; // Type of class descriptor
        uint16_t wDescriptorLength; // Total size of the Report descriptor
} __attribute__((packed)) usb_HIDDescriptor;


/* HID Class Descriptor Keyboard & Mouse */

typedef struct {
        uint8_t  bDescrType; // Type of class descriptor
        uint16_t wDescriptorLength; // Total size of the Report descriptor
} __attribute__((packed)) hid_classDescriptorLengthType;


/* HID Boot - generalised input report 8 bytes Keyboard & Mouse */
typedef union {

	struct {
		struct {
			uint8_t one:1;
			uint8_t two:1;
			uint8_t three:1;
			uint8_t :5;
			} __attribute__((packed)) button;
		uint8_t Xdispl;
		uint8_t Ydispl;
		uint8_t bytes3to7[ 5 ] ;   //optional bytes
	} __attribute__((packed)) mouse;

	struct {
		struct {
			uint8_t LCtrl:1;
			uint8_t LShift:1;
			uint8_t LAlt:1;
			uint8_t LWin:1;
			/**/
			uint8_t RCtrl:1;
			uint8_t RShift:1;
			uint8_t RAlt:1;
			uint8_t RWin:1;
			} __attribute__((packed)) modkey;
			uint8_t vendorReserved;
			uint8_t keycode[ 6 ];
	} __attribute__((packed)) keyboard;

	uint8_t generic[ 8 ];

} __attribute__((packed)) hid_bootReport;


/* HID Boot keyboard output report 1 byte */
typedef struct {
    struct {
    	uint8_t NumLk:1;
    	uint8_t CapsLock:1;
    	uint8_t ScrLk:1;
    	uint8_t Compose:1;
    	uint8_t Kana:1;
    	uint8_t :3;
        } __attribute__((packed)) leds;

    uint8_t generic;
} __attribute__((packed)) hid_bootKeyboardOutputReport;


typedef struct
{
	uint8_t address;
	uint8_t configuration;
	uint8_t interface;
	uint8_t interfaceProtocol; 			// defines what is attached, either Keyboard or Mouse will report their protocol here.
	uint8_t inputEndPointAddress;
	uint8_t outputEndPointAddress;
} __attribute__((packed)) hid_usbConfiguration;

typedef enum
{
	HID_UNUSED = 0,
	HID_CLOSED,
	HID_OPEN,
	HID_RECEIVING,
	HID_WRITING,
	HID_ERROR
} hid_connectionStatus;

typedef enum
{
	HID_DISCONNECT = 0,
	HID_CONNECTION_OPEN,
	HID_CONNECTION_CLOSE,
	HID_CONNECTION_FAIL,
	HID_CONNECTION_RECEIVE
} hid_eventType;


// Event handler
typedef void(hid_eventHandler)( hid_connectionStatus * connection, hid_eventType event, uint16_t length, hid_bootReport * data );


void hid_init( hid_connectionStatus * connection );
void hid_poll( void );

void hid_setEventHandler(hid_eventHandler * handler );
void hid_HIDCodeASCII(hid_bootReport * data); // converts HID Boot Protocol Report Codes into ASCII codes (in place).

int16_t hid_write( uint8_t data );

#ifdef __cplusplus
}
#endif

#endif
