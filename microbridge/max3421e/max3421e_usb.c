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
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "serial.h"
#include "spi.h"

#include "../ch9.h"
#include "../usb.h"

#include "max3421e_constants.h"
#include "max3421e.h"

#include "max3421e_usb.h"

static usb_state_t usb_task_state;

usb_device deviceTable[USB_NUMDEVICES + 1];

/* Declare a binary Semaphore flag for the SPI Bus, in spi.c. To ensure only single access to SPI Bus. */
extern SemaphoreHandle_t xSPISemaphore;

/* Private functions */

uint8_t usb_dispatchPacket(uint8_t token, usb_endpoint * endpoint, uint16_t nakLimit);
int16_t usb_read(usb_device * device, usb_endpoint * endpoint, uint16_t length, uint8_t * data, uint16_t nakLimit);
int16_t usb_write(usb_device * device, usb_endpoint * endpoint, uint16_t length, uint8_t * data);
int16_t usb_ctrlData(usb_device * device, uint8_t direction, uint16_t length, uint8_t * data);


/**
 * Initialises the USB layer.
 *
 * Takes xSPISemaphore - reserving SPI bus
 *
 */
void usb_init()
{
 	max3421e_init();

	if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
	{
		max3421e_powerOn();
		xSemaphoreGive( xSPISemaphore);
	}else
		return;

	// Initialise the USB state machine.
	usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;

	// Initialise the device table.
	for (uint8_t i = 0; i < (USB_NUMDEVICES + 1); ++i)
		deviceTable[i].active = false;

	// Address 0 is used to configure devices and assign them an address when they are first plugged in
	deviceTable[0].address = 0;
	usb_initEndPoint(&(deviceTable[0].control), 0);
}

usb_state_t usb_getUsbTaskState()
{
	return (usb_task_state);
}

void usb_setUsbTaskState(usb_state_t state)
{
	usb_task_state = state;
}

usb_device * usb_getDevice(uint8_t address)
{
	if (address>USB_NUMDEVICES+1) return NULL;

	return &(deviceTable[address]);
}

uint8_t usb_dispatchPacket(uint8_t token, usb_endpoint * endpoint, uint16_t nakLimit)
{
	uint8_t rcode = hrSUCCESS;
	uint8_t retry_count = 0;
	uint16_t nak_count = 0;
	TickType_t timeout = xTaskGetTickCount() + USB_XFER_TIMEOUT / portTICK_PERIOD_MS;

	while (timeout > xTaskGetTickCount() )
	{
		// Analyze transfer result

		// Launch the transfer.
		max3421e_write(MAX_REG_HXFR, (token | endpoint->address));

		// Wait for interrupt
		while (timeout > xTaskGetTickCount() )
		{
			if (max3421e_read(MAX_REG_HIRQ) & bmHXFRDNIRQ)
			{
				// Clear the interrupt.
				max3421e_write(MAX_REG_HIRQ, bmHXFRDNIRQ);
				break;
			}
		}

		// Wait for HRSL
		while (timeout > xTaskGetTickCount() )
		{
			rcode = (max3421e_read(MAX_REG_HRSL) & 0x0f);
			if (rcode != hrBUSY)
				break;
			else
				xSerialPrint_P(PSTR("USB dispatch busy!\r\n"));
		}

		switch (rcode)
		{
			case hrSUCCESS:
				return (rcode);
				break;

			case hrNAK:
				nak_count++;
				if (nakLimit && (nak_count == nakLimit))
					return (rcode);
				break;

			case hrTIMEOUT:
				retry_count++;
				if (retry_count == USB_RETRY_LIMIT)
					return (rcode);
				break;

			default:
				xSerialPrint_P(PSTR("USB dispatch error!\r\n"));
				break;
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
	vbusState_t tmpvbus;
	uint8_t tmpdata;
	static TickType_t delay = 0;
	usb_deviceDescriptor deviceDescriptor;

	if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
	{
		// Poll the MAX3421E device. Check the interrupt status.
		max3421e_poll();
		xSemaphoreGive( xSPISemaphore);
	}else
		return;

	/* modify USB task state if Vbus changed */
	tmpvbus = (vbusState_t) max3421e_getVbusState();

	switch (tmpvbus)
	{

	case SE0: //disconnected
		if ((usb_task_state & USB_STATE_MASK) != USB_STATE_DETACHED)
		{
			usb_task_state = USB_DETACHED_SUBSTATE_INITIALIZE;
		}
		break;

	case SE1: //illegal state
		usb_task_state = USB_DETACHED_SUBSTATE_ILLEGAL;
		break;

	case LSHOST: //low speed device attached

	case FSHOST: //full speed device attached
		if ((usb_task_state & USB_STATE_MASK) == USB_STATE_DETACHED)
		{
			delay = xTaskGetTickCount() + USB_SETTLE_DELAY / portTICK_PERIOD_MS;
			usb_task_state = USB_ATTACHED_SUBSTATE_SETTLE;
		}
		break;

	}// switch( tmpdata


	switch (usb_task_state)
	{
	case USB_STATE_DETACHED: //just sit here
		break;

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
		xSerialPrint_P(PSTR("USB_DETACHED_SUBSTATE_ILLEGAL\r\n"));
		break;

	case USB_ATTACHED_SUBSTATE_SETTLE: //settle time for just attached device
		if (delay < xTaskGetTickCount() )
		{
			usb_task_state = USB_ATTACHED_SUBSTATE_RESET_DEVICE;
		}
		break;

	case USB_ATTACHED_SUBSTATE_RESET_DEVICE:
		xSerialPrint_P(PSTR("USB_ATTACHED_SUBSTATE_RESET_DEVICE\r\n"));
		if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
		{
			// Issue bus reset.
			max3421e_write(MAX_REG_HCTL, bmBUSRST);
			xSemaphoreGive( xSPISemaphore);
		}else
			break;

		usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE;
		break;

	case USB_ATTACHED_SUBSTATE_WAIT_RESET_COMPLETE:
		if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
		{
			if ((max3421e_read(MAX_REG_HCTL) & bmBUSRST) == 0)
			{
				tmpdata = ( max3421e_read(MAX_REG_MODE) | bmSOFKAENAB ); //start SOF generation
				max3421e_write(MAX_REG_MODE, tmpdata);

				usb_task_state = USB_ATTACHED_SUBSTATE_WAIT_SOF;
				delay = xTaskGetTickCount() + 20 / portTICK_PERIOD_MS; //20ms wait after reset per USB spec
			}
			xSemaphoreGive( xSPISemaphore);
		}
		break;

	case USB_ATTACHED_SUBSTATE_WAIT_SOF: //todo: change check order
		if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
		{
			if (max3421e_read(MAX_REG_HIRQ) & bmFRAMEIRQ)
			{ //when first SOF received we can continue
				if (delay < xTaskGetTickCount() )
				{ //20ms passed
					usb_task_state = USB_ATTACHED_SUBSTATE_GET_DEVICE_DESCRIPTOR_SIZE;
				}
			}
			xSemaphoreGive( xSPISemaphore);
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
					xSerialPrint_P(PSTR("USB_STATE_CONFIGURING\r\n"));
					usb_fireEvent(&deviceTable[i], USB_CONNECT);
					// usb_task_state = USB_STATE_CONFIGURING;
					// NB: I've bypassed the configuring state, because configuration should be handled
					// in the usb event handler.
					usb_task_state = USB_STATE_RUNNING;
					xSerialPrint_P(PSTR("USB_STATE_RUNNING\r\n"));
				} else
				{
					usb_fireEvent(&deviceTable[i], USB_ADRESSING_ERROR);

					usb_task_state = USB_STATE_ERROR;
				}
				break; //break if address assigned or error occurred during address assignment attempt
			}
		}

		// If no vacant spot was found in the device table, fire an error.
		if (usb_task_state == USB_STATE_ADDRESSING)
		{
			usb_fireEvent(&deviceTable[i], USB_ADRESSING_ERROR);

			// No vacant place in devtable
			usb_task_state = USB_STATE_ERROR;
		}

		break;
	case USB_STATE_CONFIGURING:
		break;
	case USB_STATE_RUNNING:
		break;
	case USB_STATE_ERROR:
		xSerialPrint_P(PSTR("USB_STATE_ERROR\r\n"));
		break;
	}
}

/**
 * Convenience method for getting a string. This is useful for printing manufacturer names, serial numbers, etc.
 * Language is defaulted to zero. Strings are returned in 16-bit unicode from the device, so this function converts the
 * result to ASCII by ignoring every second byte. However, since the target buffer is used as a temporary storage during
 * this process it must be twice as large as the desired maximum string size.
 * Maximum ASCII string length is 127 bytes
 *
 * @param device USB device.
 * @param index string index.
 * @param languageId language ID.
 * @param length buffer length.
 * @param str target buffer.
 * @return retrieved string length on success, error code otherwise.
 */
int8_t usb_getString(usb_device * device, uint8_t index, uint16_t languageId, uint8_t length, uint8_t * str)
{
	uint8_t stringLength = 0;
	int8_t ret = 0;

    // Get string length;
	ret = usb_controlRequest(device, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, languageId, sizeof(uint8_t), &stringLength);
    if (ret<0) return -1;

    // Trim string size to fit the target buffer.
    if (stringLength>length) stringLength = length;

	// Get the whole thing.
	ret = usb_controlRequest(device, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, index, USB_DESCRIPTOR_STRING, languageId, stringLength, (uint8_t *)str);
    if (ret<0) return -2;

    if (stringLength != 0 && str[0] != 0x00) // make sure we're not processing a null string
    {
		// Convert Unicode to 8-bit ASCII
		stringLength = (stringLength - 2) / 2;
		for (uint8_t i=0; i < stringLength; ++i)
		{
			str[i] = str[i*2 + 2];
		}
    }
	str[stringLength] = 0x00; // Null terminated string.

	return stringLength;
}

/**
 * Performs an in transfer from a USB device from an arbitrary endpoint.
 *
 * Takes xSPISemaphore - reserving SPI bus
 *
 * @param device USB bulk device.
 * @param device length number of bytes to read.
 * @param data target buffer.
 * @return number of bytes read, or error code in case of failure.
 */
int16_t usb_read(usb_device * device, usb_endpoint * endpoint, uint16_t length, uint8_t * data, uint16_t nakLimit)
{
	uint8_t rcode;
	uint16_t bytesRead;
	uint16_t maxPacketSize = endpoint->maxPacketSize;

	int16_t totalTransferred = 0;

	if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
	{

		// Set device address.
		max3421e_write(MAX_REG_PERADDR, device->address);

		// Set toggle value.
		max3421e_write(MAX_REG_HCTL, endpoint->receiveToggle);

		while (1)
		{

			// Start IN transfer
			rcode = usb_dispatchPacket(tokIN, endpoint, nakLimit);

			if (rcode == hrTOGERR)
			{
				// Yes, we flip the receive toggle wrong here so that next time it is actually correct!
				endpoint->receiveToggle = (max3421e_read(MAX_REG_HRSL) & bmRCVTOGRD) ? bmRCVTOG0 : bmRCVTOG1;
				max3421e_write(MAX_REG_HCTL, endpoint->receiveToggle ); //set toggle value
				continue;
			}

			if (rcode != hrSUCCESS)
			{
				if (rcode != hrNAK)
					xSerialPrintf_P(PSTR("usb_read: dispatch error 0x%02x @ Tick: %u\r\n"), rcode, xTaskGetTickCount());

				xSemaphoreGive( xSPISemaphore);
				return -1;
			}

			// Check that the RCVDAVIRQ bit in register MAX_REG_HIRQ is set.
			if ( rcode == hrSUCCESS && ((max3421e_read(MAX_REG_HIRQ) & bmRCVDAVIRQ) == 0) )
			{
//				xSerialPrintf_P(PSTR("usb_read: no RCVDAVIRQ rcode: 0x%02x @ Tick: %u\r\n"), rcode, xTaskGetTickCount());

				// The absence of RCVDAVIRQ indicates a toggle error.
				// Yes, we flip the receive toggle wrong here so that next time it is actually correct!
				endpoint->receiveToggle = (max3421e_read(MAX_REG_HRSL) & bmRCVTOGRD) ? bmRCVTOG0 : bmRCVTOG1;
				max3421e_write(MAX_REG_HCTL, endpoint->receiveToggle ); //set toggle value

				xSemaphoreGive( xSPISemaphore);
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
				endpoint->receiveToggle = (max3421e_read(MAX_REG_HRSL) & bmRCVTOGRD) ? bmRCVTOG1 : bmRCVTOG0;


				// Break out of the loop.
				break;
			}
		}
		xSemaphoreGive( xSPISemaphore);

	} else
		return -3;

	// Report success.
	return totalTransferred;

}


/**
 * Performs a bulk or interrupt in transfer from a USB device.
 *
 * @param device USB bulk device.
 * @param device length number of bytes to read.
 * @param data target buffer.
 *
 * @return number of bytes read, or error code in case of failure.
 */
int16_t usb_bulkRead(usb_device * device, uint16_t length, uint8_t * data, uint8_t poll)
{
	return usb_read(device, &(device->bulk_in), length, data, poll ? 1 : USB_NAK_LIMIT);
}



/**
 * Performs an out transfer to a USB device on an arbitrary endpoint.
 *
 * Takes xSPISemaphore - reserving SPI bus
 *
 * @param device USB bulk device.
 * @param device length number of bytes to read.
 * @param data target buffer.
 * @return number of bytes written, or error code in case of failure.
 */
int16_t usb_write(usb_device * device, usb_endpoint * endpoint, uint16_t length, uint8_t * data)
{
	uint8_t rcode = 0;
	uint8_t retry_count;

	int16_t totalTransferred = 0;

	uint16_t bytes_tosend, nak_count;
	uint16_t bytes_left = length;
	uint16_t nak_limit = USB_NAK_LIMIT;

	// Local copy of the data pointer.
	uint8_t * data_p = data;

	uint8_t maxPacketSize = endpoint->maxPacketSize;

	TickType_t timeout = xTaskGetTickCount() + USB_XFER_TIMEOUT / portTICK_PERIOD_MS;

	// If maximum packet size is not set, return.
	if (!maxPacketSize) return 0xFE;

	if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
	{

		// Set device address.
		max3421e_write(MAX_REG_PERADDR, device->address);

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

			while (rcode && (timeout > xTaskGetTickCount()) )
			{
				switch (rcode)
				{
				case hrNAK:
					nak_count++;
					if (nak_limit && (nak_count == USB_NAK_LIMIT))
					{
						xSemaphoreGive( xSPISemaphore);
						return (rcode); //return NAK
					}
					break;
				case hrTIMEOUT:
					retry_count++;
					if (retry_count == USB_RETRY_LIMIT)
					{
						xSemaphoreGive( xSPISemaphore);
						return (rcode); //return TIMEOUT
					}
					break;
                case hrTOGERR:
                        // yes, we flip it wrong here so that next time it is actually correct!
                        endpoint->sendToggle = (max3421e_read(MAX_REG_HRSL) & bmSNDTOGRD) ? bmSNDTOG0 : bmSNDTOG1;
                        max3421e_write(MAX_REG_HCTL, endpoint->sendToggle); //set toggle value
                        break;
				default:
					xSemaphoreGive( xSPISemaphore);
					return (rcode);
					break;
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
			totalTransferred += bytes_tosend;
		}

		endpoint->sendToggle = (max3421e_read(MAX_REG_HRSL) & bmSNDTOGRD) ? bmSNDTOG1 : bmSNDTOG0; //update toggle

		xSemaphoreGive( xSPISemaphore);

	}

	return totalTransferred;
}

/**
 * Performs a bulk or interrupt out transfer to a USB device.
 *
 * @param device USB bulk device.
 * @param device length number of bytes to read.
 * @param data target buffer.
 * @return number of bytes read, or error code in case of failure.
 */
int16_t usb_bulkWrite(usb_device * device, uint16_t length, uint8_t * data)
{
	return usb_write(device, &(device->bulk_out), length, data);
}


/**
 * Read/write data to/from the control endpoint of a device.
 *
 * @param device USB device.
 * @param direction true for input, false for output.
 * @param length number of bytes to transfer.
 * @param data data buffer.
 */
int16_t usb_ctrlData(usb_device * device, uint8_t direction, uint16_t length, uint8_t * data)
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
 * Takes xSPISemaphore - reserving SPI bus
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
int8_t usb_controlRequest(
		usb_device * device,
		uint8_t requestType,
		uint8_t request,
		uint8_t valueLow,
		uint8_t valueHigh,
		uint16_t index,
		uint16_t length,
		uint8_t * data)
{
	uint8_t direction = false; //request direction, IN or OUT
	uint8_t rcode;
	usb_setupPacket setup_pkt;

	if (requestType & 0x80)
		direction = true; //determine request direction, true for input, false for output.

	// Build setup packet.
	setup_pkt.ReqType_u.bmRequestType = requestType;
	setup_pkt.bRequest = request;
	setup_pkt.wVal_u.wValueLo = valueLow;
	setup_pkt.wVal_u.wValueHi = valueHigh;
	setup_pkt.wIndex = index;
	setup_pkt.wLength = length;

	if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
	{
		// Set device address.
		max3421e_write(MAX_REG_PERADDR, device->address);

		// Write setup packet to the FIFO and dispatch
		max3421e_writeMultiple(MAX_REG_SUDFIFO, 8, (uint8_t *) &setup_pkt);
		rcode = usb_dispatchPacket(tokSETUP, &(device->control), USB_NAK_LIMIT);

		xSemaphoreGive( xSPISemaphore );
	} else
		return -3;

	// Print error in case of failure.
	if (rcode)
	{
		xSerialPrintf_P(PSTR("Setup packet error: 0x%02x @ Tick: %u\r\n"), rcode, xTaskGetTickCount());
		return -1;
	}

	// Data stage, if present
	if (data != NULL)
	{
		rcode = usb_ctrlData(device, direction, length, data);

		// If unsuccessful, return error.
		if (rcode < 0)
		{
			xSerialPrintf_P(PSTR("Data packet error: 0x%02x @ Tick: %u\r\n"), rcode, xTaskGetTickCount());
			return -2;
		}
	}

	if( (xSemaphoreTake( xSPISemaphore, (SPI_TIMEOUT / portTICK_PERIOD_MS )) == pdTRUE ) )
	{
		// Status stage.
		if (direction)
			rcode = usb_dispatchPacket(tokOUTHS, &(device->control), USB_NAK_LIMIT);
		else
			rcode = usb_dispatchPacket(tokINHS, &(device->control), USB_NAK_LIMIT);

		xSemaphoreGive( xSPISemaphore );
	} else
		return -3;

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
int8_t usb_getDeviceDescriptor(usb_device * device, usb_deviceDescriptor * descriptor)
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
int16_t usb_getConfigurationDescriptor(usb_device * device, uint8_t conf, uint16_t length, uint8_t * data)
{
	uint16_t descriptorLength;
	int16_t rcode;

	// Read the length of the configuration descriptor.
	rcode = (usb_controlRequest(device, bmREQ_GET_DESCR, USB_REQUEST_GET_DESCRIPTOR, conf, USB_DESCRIPTOR_CONFIGURATION, 0x0000, 4, data));
	if (rcode) return -1;

	descriptorLength = (data[3] << 8) | data[2];
	if (descriptorLength<length) length = descriptorLength;

	// Read the entire the configuration descriptor.
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
int8_t usb_setAddress(usb_device * device, uint8_t address)
{
    return(usb_controlRequest(device, bmREQ_SET, USB_REQUEST_SET_ADDRESS, address, 0x00, 0x0000, 0x0000, NULL));
}

//set configuration
int8_t usb_setConfiguration(usb_device * device, uint8_t configuration)
{
    return(usb_controlRequest(device, bmREQ_SET, USB_REQUEST_SET_CONFIGURATION, configuration, 0x00, 0x0000, 0x0000, NULL));
}
