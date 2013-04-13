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

#include "../avr.h"
#include "max3421e_usb.h"
#include "../ch9.h"

#include <util/delay.h>

static uint8_t usb_error = 0;
static uint8_t usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;

usb_device deviceTable[USB_NUMDEVICES + 1];

/**
 * Initialises the USB layer.
 */
void usb_init()
{
 	max3421e_init();
	max3421e_powerOn();

	uint8_t i;

	// Initialise the USB state machine.
	usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;

	// Initialise the device table.
	for (i = 0; i < (USB_NUMDEVICES + 1); i++)
		deviceTable[i].active = false;

	// Address 0 is used to configure devices and assign them an address when they are first plugged in
	deviceTable[0].address = 0;
	usb_initEndPoint(&(deviceTable[0].control), 0);

}

uint8_t usb_getUsbTaskState()
{
	return (usb_task_state);
}

void usb_setUsbTaskState(uint8_t state)
{
	usb_task_state = state;
}

usb_device * usb_getDevice(uint8_t address)
{
	if (address>USB_NUMDEVICES+1) return NULL;

	return &(deviceTable[address]);
}

int usb_dispatchPacket(uint8_t token, usb_endpoint * endpoint, unsigned int nakLimit)
{
	uint32_t timeout = avr_millis() + USB_XFER_TIMEOUT;
	uint8_t tmpdata;
	uint8_t rcode = 0;
	unsigned int nak_count = 0;
	char retry_count = 0;

	while (timeout > avr_millis())
	{
		// Analyze transfer result.

		// Launch the transfer.
		max3421e_write(MAX_REG_HXFR, (token | endpoint->address));
		rcode = 0xff;

		// Wait for interrupt
		while (timeout > avr_millis())
		{
			tmpdata = max3421e_read(MAX_REG_HIRQ);
			if (tmpdata & bmHXFRDNIRQ)
			{
				// Clear the interrupt.
				max3421e_write(MAX_REG_HIRQ, bmHXFRDNIRQ);

				rcode = 0x00;
				break;
			}
		}

		// Exit if timeout.
		if (rcode != 0x00)
			return (rcode);

		// Wait for HRSL
		while (timeout > avr_millis())
		{
			rcode = (max3421e_read(MAX_REG_HRSL) & 0x0f);
			if (rcode != hrBUSY)
				break;
//			else
//				avr_serialPrintf("busy!\n");
		}


		switch (rcode)
		{
			case hrNAK:
				nak_count++;
				if (nak_count == nakLimit)
					return (rcode);
				break;
			case hrTIMEOUT:
				retry_count++;
				if (retry_count == USB_RETRY_LIMIT)
					return (rcode);
				break;
			default:
				return (rcode);
		}
	}

	return (rcode);
}

/**
 * USB main task. Performs enumeration/cleanup
 */
void usb_poll(void)
{
	uint8_t i;
	uint8_t rcode;
	uint8_t tmpdata;
	static unsigned long delay = 0;
	usb_deviceDescriptor deviceDescriptor;

	// Poll the MAX3421E device.
	max3421e_poll();

	/* modify USB task state if Vbus changed */
	tmpdata = max3421e_getVbusState();

	switch (tmpdata)
	{
	case SE1: //illegal state
		usb_task_state = USB_DETACHED_SUBSTATE_ILLEGAL;
		break;
	case SE0: //disconnected
		if ((usb_task_state & USB_STATE_MASK) != USB_STATE_DETACHED)
		{
			usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;
		}
		break;
	case FSHOST: //attached
	case LSHOST:
		if ((usb_task_state & USB_STATE_MASK) == USB_STATE_DETACHED)
		{
			delay = avr_millis() + USB_SETTLE_DELAY;
			usb_task_state = USB_ATTACHED_SUBSTATE_SETTLE;
		}
		break;
	}// switch( tmpdata

	//Serial.print("USB task state: ");
	//Serial.println( usb_task_state, HEX );

	switch (usb_task_state)
	{
	case USB_DETACHED_SUBSTATE_INITIALIZE:

		// TODO right now it looks like the USB board is just reset on disconnect. Fire disconnect for all connected
		// devices.
		for (i = 1; i < USB_NUMDEVICES; i++)
			if (deviceTable[i].active)
				usb_fireEvent(&(deviceTable[i]), USB_DISCONNECT);

		usb_init();
		usb_task_state = USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE;
		break;
	case USB_DETACHED_SUBSTATE_WAIT_FOR_DEVICE: //just sit here
		break;
	case USB_DETACHED_SUBSTATE_ILLEGAL: //just sit here
		break;
	case USB_ATTACHED_SUBSTATE_SETTLE: //setlle time for just attached device
		if (delay < avr_millis())
		{
			usb_task_state = USB_ATTACHED_SUBSTATE_RESET_DEVICE;
		}
		break;

	case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
		// Issue bus reset.
		max3421e_write(MAX_REG_HCTL, bmBUSRST);
		usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE;
		break;

	case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE:
		if ((max3421e_read(MAX_REG_HCTL) & bmBUSRST) == 0)
		{
			tmpdata = max3421e_read(MAX_REG_MODE) | bmSOFKAENAB; //start SOF generation
			max3421e_write(MAX_REG_MODE, tmpdata);
			//                  max3421e_regWr( rMODE, bmSOFKAENAB );
			usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_SOF;
			delay = avr_millis() + 20; //20ms wait after reset per USB spec
		}
		break;

	case USB_ATTACHED_SUBSTATE_WAIT_SOF: //todo: change check order
		if (max3421e_read(MAX_REG_HIRQ) & bmFRAMEIRQ)
		{ //when first SOF received we can continue
			if (delay < avr_millis())
			{ //20ms passed
				usb_task_state
						= USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE;
			}
		}
		break;

	case USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE:
		// toggle( BPNT_0 );

		deviceTable[0].control.maxPacketSize = 8;

		rcode = usb_getDeviceDescriptor(&deviceTable[0], &deviceDescriptor);
		if (rcode == 0)
		{
			deviceTable[0].control.maxPacketSize = deviceDescriptor.bMaxPacketSize0;
			usb_task_state = USB_STATE_ADDRESSING;
		} else
		{
			usb_error = USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE;
			usb_task_state = USB_STATE_ERROR;
		}
		break;

	case USB_STATE_ADDRESSING:

		// Look for an empty spot
		for (i = 1; i < USB_NUMDEVICES; i++)
		{
			if (!deviceTable[i].active)
			{
				// Set correct MaxPktSize
				// deviceTable[i].epinfo = deviceTable[0].epinfo;

				deviceTable[i].address = i;
				deviceTable[i].active = true;

				usb_initEndPoint(&(deviceTable[i].control), 0);

				//temporary record
				//until plugged with real device endpoint structure
				rcode = usb_setAddress(&deviceTable[0], i);

				if (rcode == 0)
				{
					usb_fireEvent(&deviceTable[i], USB_CONNECT);
					// usb_task_state = USB_STATE_CONFIGURING;
					// NB: I've bypassed the configuring state, because configuration should be handled
					// in the usb event handler.
					usb_task_state = USB_STATE_RUNNING;
				} else
				{
					usb_fireEvent(&deviceTable[i], USB_ADRESSING_ERROR);

					// TODO remove usb_error at some point?
					usb_error = USB_STATE_ADDRESSING;
					usb_task_state = USB_STATE_ERROR;
				}
				break; //break if address assigned or error occured during address assignment attempt
			}
		}

		// If no vacant spot was found in the device table, fire an error.
		if (usb_task_state == USB_STATE_ADDRESSING)
		{
			usb_fireEvent(&deviceTable[i], USB_ADRESSING_ERROR);

			// No vacant place in devtable
			usb_error = 0xfe;
			usb_task_state = USB_STATE_ERROR;
		}

		break;
	case USB_STATE_CONFIGURING:
		break;
	case USB_STATE_RUNNING:
		break;
	case USB_STATE_ERROR:
		break;
	}
}

/**
 * Convenience method for getting a string. This is useful for printing manufacturer names, serial numbers, etc.
 * Language is defaulted to zero. Strings are returned in 16-bit unicode from the device, so this function converts the
 * result to ASCII by ignoring every second byte. However, since the target buffer is used as a temporary storage during
 * this process it must be twice as large as the desired maximum string size.
 *
 * @param device USB device.
 * @param index string index.
 * @param languageId language ID.
 * @param length buffer length.
 * @param str target buffer.
 * @return 0 on success, error code otherwise.
 */
int usb_getString(usb_device * device, uint8_t index, uint8_t languageId, uint16_t length, char * str)
{
	uint8_t stringLength = 0;
	int i, ret = 0;

    // Get string length;
	ret = usb_controlRequest(device, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, languageId, sizeof(uint8_t), &stringLength);
    if (ret<0) return -1;

    // Trim string size to fit the target buffer.
    if (stringLength>length) stringLength = length;

	// Get the whole thing.
	ret = usb_controlRequest(device, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, languageId, stringLength, (uint8_t *)str);
    if (ret<0) return -2;

	// Convert to 8-bit ASCII
	stringLength = (stringLength - 2) / 2;
	for (i=0; i<stringLength; i++) str[i] = str[2+i*2];
	str[stringLength] = 0;

	return 0;
}

/**
 * Performs an in transfer from a USB device from an arbitrary endpoint.
 *
 * @param device USB bulk device.
 * @param device length number of bytes to read.
 * @param data target buffer.
 * @return number of bytes read, or error code in case of failure.
 */
int usb_read(usb_device * device, usb_endpoint * endpoint, uint16_t length, uint8_t * data, unsigned int nakLimit)
{
	uint16_t rcode, bytesRead;
	uint16_t maxPacketSize = endpoint->maxPacketSize;

	unsigned int totalTransferred = 0;

	// Set device address.
	max3421e_write(MAX_REG_PERADDR, device->address);

	// Set toggle value.
	max3421e_write(MAX_REG_HCTL, endpoint->receiveToggle);

	while (1)
	{

		// Start IN transfer
		rcode = usb_dispatchPacket(tokIN, endpoint, nakLimit);

		if (rcode)
		{
//			if (rcode != hrNAK)
//				avr_serialPrintf("usb_read: dispatch error %d\n", rcode);

			return -1;
		}

		// Assert that the RCVDAVIRQ bit in register MAX_REG_HIRQ is set.
		if ((max3421e_read(MAX_REG_HIRQ) & bmRCVDAVIRQ) == 0)
		{
//			avr_serialPrintf("usb_read: toggle error? %d\n", rcode);

			// TODO: the absence of RCVDAVIRQ indicates a toggle error. Need to add handling for that.
			return -2;
		}

		// Obtain the number of bytes in FIFO.
		bytesRead = max3421e_read(MAX_REG_RCVBC);

		// Read the data from the FIFO.
		data = max3421e_readMultiple(MAX_REG_RCVFIFO, bytesRead, data);

		// Clear the interrupt to free the buffer.
		max3421e_write(MAX_REG_HIRQ, bmRCVDAVIRQ);

		totalTransferred += bytesRead;

		// Check if we're done reading. Either we've received a 'short' packet (<maxPacketSize), or the
		// desired number of bytes has been transferred.
		if ((bytesRead < maxPacketSize) || (totalTransferred >= length))
		{
			// Remember the toggle value for the next transfer.
			if (max3421e_read(MAX_REG_HRSL) & bmRCVTOGRD)
				endpoint->receiveToggle = bmRCVTOG1;
			else
				endpoint->receiveToggle = bmRCVTOG0;

			// Break out of the loop.
			break;
		}
	}

	// Report success.
	return totalTransferred;
}


/**
 * Performs a bulk in transfer from a USB device.
 *
 * @param device USB bulk device.
 * @param device length number of bytes to read.
 * @param data target buffer.
 *
 * @return number of bytes read, or error code in case of failure.
 */
int usb_bulkRead(usb_device * device, uint16_t length, uint8_t * data, boolean poll)
{
	return usb_read(device, &(device->bulk_in), length, data, poll ? 1 : USB_NAK_LIMIT);
}


/**
 * Performs ab out transfer to a USB device on an arbitrary endpoint.
 *
 * @param device USB bulk device.
 * @param device length number of bytes to read.
 * @param data target buffer.
 * @return number of bytes written, or error code in case of failure.
 */
int usb_write(usb_device * device, usb_endpoint * endpoint, uint16_t length, uint8_t * data)
{
	uint8_t rcode = 0, retry_count;

	// Set device address.
	max3421e_write(MAX_REG_PERADDR, device->address);

	// Local copy of the data pointer.
	uint8_t * data_p = data;

	unsigned int bytes_tosend, nak_count;
	unsigned int bytes_left = length;
	unsigned int nak_limit = USB_NAK_LIMIT;

	uint32_t timeout = avr_millis() + USB_XFER_TIMEOUT;

	uint8_t maxPacketSize = endpoint->maxPacketSize;

	// If maximum packet size is not set, return.
	if (!maxPacketSize) return 0xFE;

	max3421e_write(MAX_REG_HCTL, endpoint->sendToggle); //set toggle value

	while (bytes_left)
	{
		retry_count = 0;
		nak_count = 0;

		bytes_tosend = (bytes_left >= maxPacketSize) ? maxPacketSize : bytes_left;

		// Filling output FIFO
		max3421e_writeMultiple(MAX_REG_SNDFIFO, bytes_tosend, data_p);

		// Set number of bytes to send.
		max3421e_write(MAX_REG_SNDBC, bytes_tosend);

		// Dispatch packet.
		max3421e_write(MAX_REG_HXFR, (tokOUT | endpoint->address));

		// Wait for completion.
		while (!(max3421e_read(MAX_REG_HIRQ) & bmHXFRDNIRQ));

		// Clear IRQ.
		max3421e_write(MAX_REG_HIRQ, bmHXFRDNIRQ);

		rcode = (max3421e_read(MAX_REG_HRSL) & 0x0f);

		while (rcode && (timeout > avr_millis()))
		{
			switch (rcode)
			{
			case hrNAK:
				nak_count++;
				if (nak_limit && (nak_count == USB_NAK_LIMIT))
				{
					return (rcode); //return NAK
				}
				break;
			case hrTIMEOUT:
				retry_count++;
				if (retry_count == USB_RETRY_LIMIT)
				{
					return (rcode); //return TIMEOUT
				}
				break;
			default:
				return (rcode);
			}

			// Process NAK according to Host out NAK bug.
			max3421e_write(MAX_REG_SNDBC, 0);
			max3421e_write(MAX_REG_SNDFIFO, *data_p);
			max3421e_write(MAX_REG_SNDBC, bytes_tosend);
			max3421e_write(MAX_REG_HXFR, (tokOUT | endpoint->address)); //dispatch packet

			// Wait for the completion interrupt.
			while (!(max3421e_read(MAX_REG_HIRQ) & bmHXFRDNIRQ));

			// Clear interrupt.
			max3421e_write(MAX_REG_HIRQ, bmHXFRDNIRQ);

			rcode = (max3421e_read(MAX_REG_HRSL) & 0x0f);
		}

		bytes_left -= bytes_tosend;
		data_p += bytes_tosend;
	}

	endpoint->sendToggle = (max3421e_read(MAX_REG_HRSL) & bmSNDTOGRD) ? bmSNDTOG1 : bmSNDTOG0; //update toggle

	// Should be 0 in all cases.
	return (rcode);
}

/**
 * Performs a bulk out transfer to a USB device.
 *
 * @param device USB bulk device.
 * @param device length number of bytes to read.
 * @param data target buffer.
 * @return number of bytes read, or error code in case of failure.
 */
int usb_bulkWrite(usb_device * device, uint16_t length, uint8_t * data)
{
	return usb_write(device, &(device->bulk_out) , length, data);
}

/**
 * Read/write data to/from the control endpoint of a device.
 *
 * @param device USB device.
 * @param direction true for input, false for output.
 * @param length number of bytes to transfer.
 * @param data data buffer.
 */
uint8_t usb_ctrlData(usb_device * device, boolean direction, uint16_t length, uint8_t * data)
{
	if (direction)
	{
		// IN transfer
		device->control.receiveToggle = bmRCVTOG1;
		return usb_read(device, &(device->control), length, data, USB_NAK_LIMIT);

	} else
	{
		// OUT transfer
		device->control.sendToggle = bmSNDTOG1;
		return usb_write(device, &(device->control), length, data);
	}
}

/**
 * Sends a control request to a USB device.
 *
 * @param device USB device to send the control request to.
 * @param requestType request type (in/out).
 * @param request request.
 * @param valueLow low byte of the value parameter.
 * @param valueHigh high byte of the value parameter.
 * @param index index.
 * @param length number of bytes to transfer.
 * @param data data to send in case of output transfer, or reception buffer in case of input. If no data is to be exchanged this should be set to NULL.
 * @return 0 on success, error code otherwise
 */
int usb_controlRequest(
		usb_device * device,
		uint8_t requestType,
		uint8_t request,
		uint8_t valueLow,
		uint8_t valueHigh,
		uint16_t index,
		uint16_t length,
		uint8_t * data)
{
	boolean direction = false; //request direction, IN or OUT
	uint8_t rcode;
	usb_setupPacket setup_pkt;

	// Set device address.
	max3421e_write(MAX_REG_PERADDR, device->address);

	if (requestType & 0x80)
		direction = true; //determine request direction

	// Build setup packet.
	setup_pkt.bmRequestType = requestType;
	setup_pkt.bRequest = request;
	setup_pkt.wValue = valueLow | (valueHigh << 8);
	setup_pkt.wIndex = index;
	setup_pkt.wLength = length;

	// Write setup packet to the FIFO and dispatch
	max3421e_writeMultiple(MAX_REG_SUDFIFO, 8, (uint8_t *) &setup_pkt);
	rcode = usb_dispatchPacket(tokSETUP, &(device->control), USB_NAK_LIMIT);

	// Print error in case of failure.
	if (rcode)
	{
//		avr_serialPrintf("Setup packet error: 0x%02x\n", rcode);
		return -1;
	}

	// Data stage, if present
	if (data != NULL)
	{
		rcode = usb_ctrlData(device, direction, length, data);

		// If unsuccessful, return error.
		if (rcode<0)
		{
//			avr_serialPrintf("Data packet error: 0x%02x\n", rcode);
			return -2;
		}
	}

	// Status stage.
	if (direction)
		rcode = usb_dispatchPacket(tokOUTHS, &(device->control), USB_NAK_LIMIT);
	else
		rcode = usb_dispatchPacket(tokINHS, &(device->control), USB_NAK_LIMIT);

	if (rcode)
		return -3;
	else
		return 0;
}

/**
 * Gets the device descriptor of a USB device.
 * @param device USB device
 * @param descriptor pointer to a usb_deviceDescriptor record that will be filled with the requested data.
 * @return 0 in case of success, error code otherwise
 */
int usb_getDeviceDescriptor(usb_device * device, usb_deviceDescriptor * descriptor)
{
	return(usb_controlRequest(device, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, 0x00, USB_DESCRIPTOR_DEVICE, 0x0000, sizeof(usb_deviceDescriptor), (uint8_t *)descriptor));
}

/**
 * Gets the configuration descriptor of a USB device as a byte array.
 * @param device USB device
 * @param conf configuration number
 * @param length length of the data buffer. This method will not write beyond this boundary.
 * @return number of bytes read, or negative number in case of error.
 */
int usb_getConfigurationDescriptor(usb_device * device, uint8_t conf, uint16_t length, uint8_t * data)
{
	uint16_t descriptorLength;
	int rcode;

	// Read the length of the configuration descriptor.
	rcode = (usb_controlRequest(device, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, 4, data));
	if (rcode) return -1;

	descriptorLength = (data[3] << 8) | data[2];
	if (descriptorLength<length) length = descriptorLength;

	// Read the length of the configuration descriptor.
	rcode = (usb_controlRequest(device, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, length, data));
	if (rcode) return -2;

	return length;
}

/**
 * Sets the address of a newly connected USB device.
 *
 * @param device the 'zero' usb device (address 0, endpoint 0)
 * @param address the address to set for the newly connected device
 * @return 0 in case of success, error code otherwise
 */
int usb_setAddress(usb_device * device, uint8_t address)
{
    return(usb_controlRequest(device, bmREQ_SET, USB_REQUEST_SET_ADDRESS, address, 0x00, 0x0000, 0x0000, NULL));
}

//set configuration
int usb_setConfiguration(usb_device * device, uint8_t configuration)
{
    return(usb_controlRequest(device, bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, configuration, 0x00, 0x0000, 0x0000, NULL));
}
