/* Copyright 2015 Phillip Stevens

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"

#include "serial.h"

#include "ch9.h"
#include "max3421e/max3421e_usb.h"
#include "usb.h"

#include "hid.h"
#include "hid_codes.h"

//#define DEBUG // XXX define this to turn on debug printing to serial port


static usb_device * hidDevice;
static hid_connectionStatus * theConnection;
static uint8_t connected;

// Event handler callback function.
static hid_eventHandler * eventHandler;

// Private Functions
static void hid_fireEvent( hid_eventType type, uint16_t length, hid_bootReport * data );
static int16_t hid_writeMessage( usb_device * device, uint8_t data );
static uint8_t hid_pollMessage(hid_bootReport * message, uint8_t poll );
static void hid_handleOkay( hid_bootReport * message );
static void hid_closeAll();
static uint8_t usb_isHIDInterface(usb_interfaceDescriptor * interface);
static uint8_t hid_isHIDDevice(usb_device * device, uint8_t protocol, hid_usbConfiguration * handle);
static void hid_initUsb(usb_device * device, hid_usbConfiguration * handle);
static void hid_setIdle(usb_device * device);
void hid_put_dump (const uint8_t *buffer, uint16_t cnt);

/**
 * Sets the HID event handler function. This function will be called by the HID layer
 * when interesting events occur, such as HID connect/disconnect, connection open/close, and
 * connection writes from the HID device.
 *
 * @param HID event handler function.
 */
void hid_setEventHandler(hid_eventHandler * handler)
{
	eventHandler = handler;
}

/**
 * Fires an HID event.
 * @param connection HID connection. May be NULL in case of global connect/disconnect events.
 * @param type event type.
 * @param length payload length or zero if no payload.
 * @param data payload data if relevant or NULL otherwise.
 */
static void hid_fireEvent( hid_eventType type, uint16_t length, hid_bootReport * data)
{
	// Fire the global event handler, if set.
	if (eventHandler!=NULL)
		eventHandler(theConnection, type, length, data);
}


/**
 * Prints an HID_boot_report, for debugging purposes.
 * @param message HID message to print.
 */
#ifdef DEBUG
static void hid_printMessage(hid_bootReport * message)
{
	xSerialPrint_P(PSTR("HID message\r\n"));
	hid_put_dump (message->generic, sizeof(hid_bootReport));
}
#endif


/**
 * Writes an HID message with payload to the HID device using the interrupt endpoint
 *
 * @param device USB device handle.
 * @param data command payload.
 * @return error code or 0 for success.
 */
static int16_t hid_writeMessage( usb_device * device, uint8_t data )
{
	hid_bootKeyboardOutputReport message;
	int16_t rcode;

	// Fill out the message record.
	message.generic = data;

	if (*theConnection == HID_OPEN)
	{
		*theConnection = HID_WRITING;
#ifdef DEBUG
		xSerialPrintf_P(PSTR("\r\nOUT << 0x%02x"), message.generic);
#endif

		rcode = usb_bulkWrite(device, sizeof(hid_bootKeyboardOutputReport), (uint8_t*)&message);
		if (rcode < 0)
			return rcode;

		rcode = usb_bulkWrite(device, 1, (uint8_t *)&data);
		if (rcode < 0)
			return rcode;
	}
	*theConnection = HID_OPEN;
	return 0;
}


/**
 * Poll an HID message.
 * @param message on success, the HID message will be returned in this struct.
 * @param poll true to poll for a packet on the input endpoint, false to wait for a packet. Use false here when a packet is expected (i.e. OKAY in response to WRTE)
 * @return true if a packet was successfully received, false otherwise.
 */
static uint8_t hid_pollMessage(hid_bootReport * message, uint8_t poll)
{
	int16_t bytesRead;
	uint8_t buf[HID_USB_PACKETSIZE];

	// Poll a packet from the USB
	bytesRead = usb_bulkRead(hidDevice, HID_USB_PACKETSIZE, buf, poll);

	// Check if the USB in transfer received a message. False if error or no message, otherwise contains bytes read.
	if (bytesRead<0) return false;

	// Check if the buffer contains a valid message
	memcpy((void*)message, (void*)buf, sizeof(hid_bootReport));

	// Check if the received number of bytes matches our expected 8 bytes of HID message header.
	if (bytesRead != sizeof(hid_bootReport)) return false;

	return true;
}

/**
 * Handles first received message, which represents a transition in the connection state machine.
 *
 * @param connection HID connection
 * @param message HID message.
 */
static void hid_handleOkay( hid_bootReport * message )
{
	// Check if the OKAY message was a response to a CONNECT message.
	if (*theConnection != HID_OPEN)
	{
		*theConnection = HID_OPEN;
		hid_fireEvent( HID_CONNECTION_OPEN, 0, NULL );
	}
}

/**
 * Handles received messages.
 *
 * @param connection HID connection
 * @param message HID message.
 */
static void hid_handleMessage(hid_bootReport * message)
{

	if (*theConnection == HID_OPEN)
	{
		hid_fireEvent( HID_CONNECTION_RECEIVE, sizeof(hid_bootReport), message );
	}
}

/**
 * Close all HID connections.
 *
 * @param connection HID connection
 * @param message HID message struct.
 */
static void hid_closeAll()
{
	// Check if the CLOSE message was a response to a CONNECT message.
	if (*theConnection == HID_OPEN)
		hid_fireEvent( HID_CONNECTION_CLOSE, 0, NULL);
	else
		hid_fireEvent(  HID_CONNECTION_FAIL, 0, NULL);

	*theConnection = HID_CLOSED;
}


/**
 * This method is called periodically to check for new messages on the USB bus and process them.
 */
void hid_poll(void)
{
	hid_bootReport message;

	// Poll the USB layer.
	usb_poll();

	// If no USB device, there's no work for us to be done, so just return.
	if (hidDevice==NULL)
		return;

	// If not connected just return.
	if (!connected)
		return;

	// Check for an incoming HID message. don't wait for incoming packet so set "true"
	if ( !hid_pollMessage(&message, true) )
		{return;}

#ifdef DEBUG
	else
		{ xSerialPrint_P(PSTR("\r\nIN >> ")); hid_printMessage(&message);}
#endif

	// Handle messages
	if (*theConnection == HID_OPEN )
		hid_handleMessage( &message );
	else
		hid_handleOkay( &message ); // we got a character, so set the Connection to OPEN.

}

/**
 * Helper function for usb_isHIDDevice to check whether an interface is a valid HID interface.
 * @param interface interface descriptor struct.
 */
static uint8_t usb_isHIDInterface(usb_interfaceDescriptor * interface)
{

	// Check if the interface has exactly one endpoint (excluding the control endpoint 0).
	if (interface->bNumEndpoints!=1) return false;

#ifdef DEBUG
    xSerialPrintf_P(PSTR("Interface class: %d\r\n"), interface->bInterfaceClass);
    xSerialPrintf_P(PSTR("Interface subclass: %d\r\n"), interface->bInterfaceSubClass);
    xSerialPrintf_P(PSTR("Interface protocol: %d\r\n"), interface->bInterfaceProtocol);
#endif

	// Check if the endpoint supports interrupt transfer.
	if (interface->bInterfaceClass != HID_INTF) return false;
	if (interface->bInterfaceSubClass != BOOT_INTF_SUBCLASS) return false;
	if ( !(interface->bInterfaceProtocol && (HID_PROTOCOL_KEYBOARD | HID_PROTOCOL_MOUSE)) ) return false;


	return true;
}

/**
 * Checks whether the a connected USB device is an HID device and populates a configuration record if it is.
 *
 * @param device USB device.
 * @param protocol desired protocol for endpoint (keyboard or mouse).
 * @param handle pointer to a configuration record. The endpoint device address, protocol (keyboard or mouse), and endpoint information will be stored here.
 * @return true if the device is an HID device. Assume Configuration 0 exactly 1 configuration is allowed.
 */
static uint8_t hid_isHIDDevice(usb_device * device, uint8_t protocol, hid_usbConfiguration * handle)
{
	uint8_t ret = false;
	uint8_t buf[MAX_BUF_SIZE];
	int16_t bytesRead;

	// Read the length of the configuration descriptor.
	bytesRead = usb_getConfigurationDescriptor(device, 0, MAX_BUF_SIZE, buf);
	if (bytesRead<0) return false;

	uint16_t pos = 0;
	uint8_t descriptorLength;
	uint8_t descriptorType;

	usb_configurationDescriptor * config = NULL;
	usb_interfaceDescriptor * interface = NULL;
	usb_endpointDescriptor * endpoint = NULL;

	while (pos < bytesRead)
	{
		descriptorLength = buf[pos];
		descriptorType = buf[pos + 1];

		switch (descriptorType)
		{
		case (USB_DESCRIPTOR_CONFIGURATION):
			config = (usb_configurationDescriptor *)(buf + pos);
			break;

		case (USB_DESCRIPTOR_INTERFACE):
			interface = (usb_interfaceDescriptor *)(buf + pos);

			if ( usb_isHIDInterface(interface) && (interface->bInterfaceProtocol == protocol))
			{

				handle->configuration = config->bConfigurationValue;
				handle->interface = interface->bInterfaceNumber;

				// Detected requested HID interface!
				ret = true;

#ifdef DEBUG
				xSerialPrintf_P(PSTR("HID Interface Descriptor parsed. 0x%02X\r\n"), interface->bInterfaceProtocol);
#endif

			}
			break;

		case (USB_DESCRIPTOR_ENDPOINT):
			endpoint = (usb_endpointDescriptor *)(buf + pos);

			// If this endpoint descriptor is found right after the HID interface descriptor, it belongs to that interface.
			if (interface->bInterfaceNumber == handle->interface)
			{
				if (endpoint->bEndpointAddress & 0x80)
				{
					handle->inputEndPointAddress = endpoint->bEndpointAddress & ~0x80;
#ifdef DEBUG
					xSerialPrintf_P(PSTR("HID INPUT Endpoint Descriptor parsed. 0x%02X\r\n"), handle->inputEndPointAddress);
#endif
				}else{
					handle->outputEndPointAddress = endpoint->bEndpointAddress;
#ifdef DEBUG
					xSerialPrintf_P(PSTR("HID OUTPUT Endpoint Descriptor parsed. 0x%02X\r\n"), handle->outputEndPointAddress);
#endif
				}
			}


			break;

		default:
			break;
		}

		pos += descriptorLength;
	}

	return ret;

}

/**
 * Initialises an HID device.
 *
 * @param device the USB device.
 * @param handle configuration information.
 */
static void hid_initUsb(usb_device * device, hid_usbConfiguration * handle)
{
	// Initialise/configure the USB device.
	usb_initDevice(device, handle->configuration);

#ifdef DEBUG
	usb_printDeviceInfo( device );
#endif

	// Initialise bulk / interrupt input endpoint.
	usb_initEndPoint(&(device->bulk_in), handle->inputEndPointAddress);
	device->bulk_in.attributes = USB_ENDPOINT_XFER_INT;
	device->bulk_in.maxPacketSize = HID_USB_PACKETSIZE;

	// Initialise bulk / interrupt output endpoint.
	usb_initEndPoint(&(device->bulk_out), handle->outputEndPointAddress);
	device->bulk_out.attributes = USB_ENDPOINT_XFER_INT;
	device->bulk_out.maxPacketSize = HID_USB_PACKETSIZE;

#ifdef DEBUG
    xSerialPrintf_P(PSTR("handle->inputEndPointAddress: 0x%02x\r\n"), handle->inputEndPointAddress);
    xSerialPrintf_P(PSTR("handle->outputEndPointAddress: 0x%02x\r\n"), handle->outputEndPointAddress);
    xSerialPrintf_P(PSTR("bulk_in.address: 0x%02x\r\n"), device->bulk_in.address);
    xSerialPrintf_P(PSTR("bulk_out.address: 0x%02x\r\n"), device->bulk_out.address);
#endif
}

/**
 * Handles events from the USB layer.
 *
 * @param device USB device that generated the event.
 * @param event USB event.
 */
static void hid_usbEventHandler(usb_device * device, usb_eventType event)
{
	hid_usbConfiguration handle;

	switch (event)
	{
	case USB_CONNECT:
		xSerialPrint_P(PSTR("HID_USB_CONNECT\r\n"));
		// Check if the newly connected device is an HID device, and initialise it if so.
		if (hid_isHIDDevice(device, HID_PROTOCOL_KEYBOARD, &handle))
		{
			hid_initUsb(device, &handle);

			// Success, signal that we are now connected by setting the global device handler to the connected device.
			hidDevice = device;
			connected = true;

			*theConnection = HID_OPEN;

			// set the device to idle so that it (particularly for a keyboard) only reports when there is an event.
			hid_setIdle(device);
		}
		break;

	case USB_DISCONNECT:
		xSerialPrint_P(PSTR("HID_USB_DISCONNECT\r\n"));
		// Check if the device that was disconnected is the HID device we've been using.
		if (device == hidDevice)
		{
			// Close all open HID connections.
			hid_closeAll();

			// Signal that we're no longer connected by setting the global device handler to NULL;
			hidDevice = NULL;
			connected = false;

			*theConnection = HID_CLOSED;
		}
		break;

	case 	USB_ADRESSING_ERROR:
		xSerialPrint_P(PSTR("HID_USB_ADRESSING_ERROR\r\n"));
		// Check if the device that was disconnected is the HID device we've been using.
		if (device == hidDevice)
		{
			// Close all open HID connections.
			hid_closeAll();

			// Signal that we're no longer connected by setting the global device handler to NULL;
			hidDevice = NULL;
			connected = false;

			*theConnection = HID_ERROR;
		}
		break;

	default:
		// ignore
		break;
	}
}

/**
 * Write a byte to an open HID connection.
 *
 * @param connection HID connection to write the data to.
 * @param data data to send.
 * @return number of transmitted bytes, or -1 on failure.
 */
int16_t hid_write( uint8_t data )
{
	int16_t ret;

	// First check if we have a working HID connection
	if (hidDevice==NULL || !connected) return -1;

	// Check if the connection is open for writing.
	if (*theConnection != HID_OPEN) return -2;
	*theConnection = HID_WRITING;

	// Write payload
	ret = hid_writeMessage(hidDevice, data);
	if (ret==0)

	*theConnection = HID_OPEN;
	return ret;
}


/**
 * Initialises the HID protocol. This function initialises the USB layer underneath so no further setup is required.
 */
void hid_init( hid_connectionStatus * connection )
{
	// Signal that we are not connected.
	hidDevice = NULL;
	connected = false;

	*connection = HID_UNUSED; // the HID device is unused at this stage.
	theConnection = connection; // store the connection for local reference.

	// Initialise the USB layer and attach a HID event handler.
	usb_setEventHandler(hid_usbEventHandler);
	usb_init();
}


/**
 * Sets the HID protocol not to report until an event occurs i.e set to idle.
 */
static void hid_setIdle(usb_device * device)
{
	usb_controlRequest(device, bmREQ_HIDOUT, HID_REQUEST_SET_IDLE, 0x00, 0x00, 0x0000, 0x0000, NULL);
}

void hid_put_dump(const uint8_t *buffer, uint16_t cnt)
{
	uint8_t i;

	for(i = 0; i < cnt; ++i)
		xSerialPrintf_P(PSTR(" %02X"), buffer[i]);

	xSerialPutChar( &xSerialPort, ' ' );
	for(i = 0; i < cnt; ++i)
	{
		xSerialPutChar( &xSerialPort, (buffer[i] >= ' ' && buffer[i] <= '~') ? buffer[i] : '.' );
	}
	xSerialPrint((uint8_t *)"\r\n");
	vTaskDelay( 8 / portTICK_PERIOD_MS ); // Whoa... too fast.
}

// converts HID Boot Protocol Report Codes into ASCII codes (in place).
void hid_HIDCodeASCII(hid_bootReport * data)
{
	for (uint8_t i = 0; i < 6; ++i) // up to 6 key codes can be returned in a Boot Protocol Report
	{
		if (*(uint8_t*)&data->keyboard.keycode[i] == '\0') // if the key code is NULL, then there are no more keys
			return;

		switch(*(uint8_t*)&data->keyboard.modkey)  // switch the key case based on the Modkey held during the key stroke.
		{
		case (SHFT_L): // this are the upper case keys
		case (SHFT_R):
			*(uint8_t*)&data->keyboard.keycode[i] = pgm_read_byte(hid_uppercase + (*(uint8_t*)&data->keyboard.keycode[i] & 0x7f ));
			break;

		case (CTRL_L):
		case (CTRL_R):
			*(uint8_t*)&data->keyboard.keycode[i] = '\f';
			break;

		case (META_L):
		case (META_R):
			*(uint8_t*)&data->keyboard.keycode[i] = '\f';
			break;

		default: // these are the lower case keys
			*(uint8_t*)&data->keyboard.keycode[i] = pgm_read_byte(hid_lowercase + (*(uint8_t*)&data->keyboard.keycode[i] & 0x7f ));
			break;
		}
	}
}

