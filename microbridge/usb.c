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
#include "avr.h"

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
int usb_printDeviceInfo(usb_device * device)
{
	int rcode;
    // char buf[128];

    // Read the device descriptor
	usb_deviceDescriptor deviceDescriptor;
    rcode = usb_getDeviceDescriptor(device, &deviceDescriptor);
    if (rcode) return rcode;

    xSerialPrintf_P(PSTR("\r\nVendor ID: %x\r\n"), deviceDescriptor.idVendor);
    xSerialPrintf_P(PSTR("Product ID: %x\r\n"), deviceDescriptor.idProduct);
    xSerialPrintf_P(PSTR("Configuration count: %d\r\n"), deviceDescriptor.bNumConfigurations);
    xSerialPrintf_P(PSTR("Device class: %d\r\n"), deviceDescriptor.bDeviceClass);
    xSerialPrintf_P(PSTR("Device subclass: %d\r\n"), deviceDescriptor.bDeviceSubClass);
    xSerialPrintf_P(PSTR("Device protocol: %d\r\n"), deviceDescriptor.bDeviceProtocol);

    /*
    usb_getString(device, deviceDescriptor.iManufacturer, device->firstStringLanguage, 128, buf);
    avr_serialPrintf("Manufacturer: %d %s\n", deviceDescriptor.iManufacturer, buf);

    usb_getString(device, deviceDescriptor.iProduct, device->firstStringLanguage, 128, buf);
    avr_serialPrintf("Product: %s\n", buf);

    usb_getString(device, deviceDescriptor.iSerialNumber, device->firstStringLanguage, 128, buf);
    avr_serialPrintf("Serial number: %s\n", buf);
    */

    return 0;
}

int usb_initDevice(usb_device * device, int configuration)
{
	char buf[4];

	uint8_t rcode;

	// Set the configuration for this USB device.
	rcode = usb_setConfiguration(device, configuration);
	if (rcode<0) return rcode;

	// Get the first supported language.
	rcode = usb_getString(device, 0, 0, 4, buf);
	if (rcode<0) return rcode;
    device->firstStringLanguage = (buf[3] << 8) | buf[2];

    return rcode;
}
