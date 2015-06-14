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

#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "serial.h"

#include "max3421e/max3421e_constants.h"
#include "max3421e/max3421e_usb.h"

#include "usb.h"

static usb_eventHandler * eventHandler;

void usb_setEventHandler(usb_eventHandler * handler)
{
	eventHandler = handler;
}

/**
 * Fires a USB event. This calls the callback function set by usb_setEventHandler.
 *
 * @param device the device the events relates to.
 * @param event event type (i.e. connected, disconnected)
 */
void usb_fireEvent(usb_device * device, usb_eventType event)
{
	eventHandler(device, event);
}

void usb_initEndPoint(usb_endpoint * endpoint, uint8_t address)
{
	endpoint->address = address;
	endpoint->sendToggle = bmSNDTOG0;
	endpoint->receiveToggle = bmRCVTOG0;
}


/**
 * Print USB device information.
 */
int8_t usb_printDeviceInfo(usb_device * device)
{
	int8_t rcode;
    uint8_t buf[MAX_BUF_SIZE];
    usb_deviceDescriptor deviceDescriptor;

    xSerialPrintf_P(PSTR("\n\nDevice address: %d\r\n"), device->address);
    xSerialPrintf_P(PSTR("Device active: %d\r\n"), device->active);
    xSerialPrintf_P(PSTR("Device firstStringLanguage: %d\r\n\n"), device->firstStringLanguage);

    // Read the device descriptor
    rcode = usb_getDeviceDescriptor(device, &deviceDescriptor);
    if (rcode)
    {
    	xSerialPrintf_P(PSTR("getDeviceDescriptor read error: %x\r\n\n"), rcode);
    	return rcode;
    }

    xSerialPrintf_P(PSTR("Descriptor Length: %d\r\n"), deviceDescriptor.bLength);
    xSerialPrintf_P(PSTR("Descriptor Type: %d\r\n"), deviceDescriptor.bDescriptorType);
    xSerialPrintf_P(PSTR("USB BCD: %d\r\n"), deviceDescriptor.bcdUSB);

    xSerialPrintf_P(PSTR("Device class: %d\r\n"), deviceDescriptor.bDeviceClass);
    xSerialPrintf_P(PSTR("Device subclass: %d\r\n"), deviceDescriptor.bDeviceSubClass);
    xSerialPrintf_P(PSTR("Device protocol: %d\r\n"), deviceDescriptor.bDeviceProtocol);
    xSerialPrintf_P(PSTR("Maximum Packet: %d\r\n"), deviceDescriptor.bMaxPacketSize0);

    xSerialPrintf_P(PSTR("Vendor ID: 0x%x\r\n"), deviceDescriptor.idVendor);
    xSerialPrintf_P(PSTR("Product ID: 0x%x\r\n"), deviceDescriptor.idProduct);
    xSerialPrintf_P(PSTR("Configuration count: %d\r\n\n"), deviceDescriptor.bNumConfigurations);

    if (usb_getString(device, deviceDescriptor.iManufacturer, device->firstStringLanguage, MAX_BUF_SIZE, buf) > 0)
    	xSerialPrintf_P(PSTR("Manufacturer: %d : %s\r\n"), deviceDescriptor.iManufacturer, buf);

    if (usb_getString(device, deviceDescriptor.iProduct, device->firstStringLanguage, MAX_BUF_SIZE, buf) > 0)
    	xSerialPrintf_P(PSTR("Product: %d : %s\r\n"), deviceDescriptor.iProduct, buf);

    if (usb_getString(device, deviceDescriptor.iSerialNumber, device->firstStringLanguage, MAX_BUF_SIZE, buf) > 0)
    	xSerialPrintf_P(PSTR("Serial number: %d : %s\r\n\n"), deviceDescriptor.iSerialNumber, buf);

    return 0;
}

int8_t usb_initDevice(usb_device * device, uint8_t configuration)
{
	uint8_t buf[4];

	uint8_t rcode;

	// Set the configuration for this USB device.
	rcode = usb_setConfiguration(device, configuration);
	if (rcode < 0) return rcode;

	// Get the first supported language.
	rcode = usb_getString(device, 0, 0, 4, buf);
	if (rcode < 0) return rcode;
    device->firstStringLanguage = (buf[3] << 8) | buf[2];

    return rcode;
}
