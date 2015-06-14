/*
 * This file holds USB constants and structures that are needed for
 * USB device APIs.  These are used by the USB device model, which is
 * defined in chapter 9 of the USB 2.0 specification and in the
 * Wireless USB 1.0 (spread around).
 *
 * USB 2.0 adds an additional "On The Go" (OTG) mode, which lets systems
 * act either as a USB master/host or as a USB slave/device.  That means
 * the master and slave side APIs benefit from working well together.
 *
 */

#ifndef _ch9_h_
#define _ch9_h_

/*-------------------------------------------------------------------------*/

/* CONTROL REQUEST SUPPORT */

/*
 * USB directions
 *
 * This bit flag is used in endpoint descriptors' bEndpointAddress field.
 * It's also one of three fields in control requests bRequestType.
 */
#define USB_DIR_OUT			0x00	/* to device */
#define USB_DIR_IN			0x80	/* to host */

/*
 * USB types, the second of three bRequestType fields
 */
#define USB_TYPE_MASK			(0x03 << 5)
#define USB_TYPE_STANDARD		(0x00 << 5)
#define USB_TYPE_CLASS			(0x01 << 5)
#define USB_TYPE_VENDOR			(0x02 << 5)
#define USB_TYPE_RESERVED		(0x03 << 5)

/*
 * USB recipients, the third of three bRequestType fields
 */
#define USB_RECIPIENT_MASK			0x1f

#define USB_RECIPIENT_DEVICE		0x00
#define USB_RECIPIENT_INTERFACE		0x01
#define USB_RECIPIENT_ENDPOINT		0x02
#define USB_RECIPIENT_OTHER			0x03
/* From Wireless USB 1.0 */
#define USB_RECIPIENT_PORT			0x04
#define USB_RECIPIENT_RPIPE			0x05

/*
 * Standard requests, for the bRequest field of a SETUP packet.
 *
 * These are qualified by the bRequestType field, so that for example
 * TYPE_CLASS or TYPE_VENDOR specific feature flags could be retrieved
 * by a GET_STATUS request.
 */
#define USB_REQUEST_GET_STATUS			0x00
#define USB_REQUEST_CLEAR_FEATURE		0x01
#define USB_REQUEST_SET_FEATURE			0x03
#define USB_REQUEST_SET_ADDRESS			0x05
#define USB_REQUEST_GET_DESCRIPTOR		0x06
#define USB_REQUEST_SET_DESCRIPTOR		0x07
#define USB_REQUEST_GET_CONFIGURATION	0x08
#define USB_REQUEST_SET_CONFIGURATION	0x09
#define USB_REQUEST_GET_INTERFACE		0x0A
#define USB_REQUEST_SET_INTERFACE		0x0B
#define USB_REQUEST_SYNCH_FRAME			0x0C
#define USB_REQUEST_SET_SEL				0x30
#define USB_REQUEST_SET_ISOCH_DELAY		0x31

#define USB_REQUEST_SET_ENCRYPTION		0x0D	/* Wireless USB */
#define USB_REQUEST_GET_ENCRYPTION		0x0E
#define USB_REQUEST_RPIPE_ABORT			0x0E
#define USB_REQUEST_SET_HANDSHAKE		0x0F
#define USB_REQUEST_RPIPE_RESET			0x0F
#define USB_REQUEST_GET_HANDSHAKE		0x10
#define USB_REQUEST_SET_CONNECTION		0x11
#define USB_REQUEST_SET_SECURITY_DATA	0x12
#define USB_REQUEST_GET_SECURITY_DATA	0x13
#define USB_REQUEST_SET_WUSB_DATA		0x14
#define USB_REQUEST_LOOPBACK_DATA_WRITE	0x15
#define USB_REQUEST_LOOPBACK_DATA_READ	0x16
#define USB_REQUEST_SET_INTERFACE_DS	0x17

/*
 * USB feature flags are written using USB_REQ_{CLEAR,SET}_FEATURE, and
 * are read as a bit array returned by USB_REQ_GET_STATUS.  (So there
 * are at most sixteen features of each type.)  Hubs may also support a
 * new USB_REQ_TEST_AND_SET_FEATURE to put ports into L1 suspend.
 */
#define USB_DEVICE_SELF_POWERED			0	/* (read only) */
#define USB_DEVICE_REMOTE_WAKEUP		1	/* dev may initiate wakeup */
#define USB_DEVICE_TEST_MODE			2	/* (wired high speed only) */
#define USB_DEVICE_BATTERY				2	/* (wireless) */
#define USB_DEVICE_B_HNP_ENABLE			3	/* (otg) dev may initiate HNP */
#define USB_DEVICE_WUSB_DEVICE			3	/* (wireless)*/
#define USB_DEVICE_A_HNP_SUPPORT		4	/* (otg) RH port supports HNP */
#define USB_DEVICE_A_ALT_HNP_SUPPORT	5	/* (otg) other RH port does */
#define USB_DEVICE_DEBUG_MODE			6	/* (special devices only) */

/* Standard Feature Selectors for CLEAR_FEATURE Requests    */
#define USB_FEATURE_ENDPOINT_HALT               0	// CLEAR/SET FEATURE - Endpoint Halt  IN/OUT will STALL
#define USB_FEATURE_DEVICE_REMOTE_WAKEUP        1	// CLEAR/SET FEATURE - Device remote wake-up
#define USB_FEATURE_TEST_MODE                   2	// CLEAR/SET FEATURE - Test mode


/* Setup Data Constants */
#define USB_SETUP_HOST_TO_DEVICE		0x00    // Device Request bmRequestType transfer direction - host to device transfer
#define USB_SETUP_DEVICE_TO_HOST		0x80    // Device Request bmRequestType transfer direction - device to host transfer
#define USB_SETUP_TYPE_STANDARD			0x00    // Device Request bmRequestType type - standard
#define USB_SETUP_TYPE_CLASS			0x20    // Device Request bmRequestType type - class
#define USB_SETUP_TYPE_VENDOR			0x40    // Device Request bmRequestType type - vendor
#define USB_SETUP_RECIPIENT_DEVICE		0x00    // Device Request bmRequestType recipient - device
#define USB_SETUP_RECIPIENT_INTERFACE	0x01    // Device Request bmRequestType recipient - interface
#define USB_SETUP_RECIPIENT_ENDPOINT	0x02    // Device Request bmRequestType recipient - endpoint
#define USB_SETUP_RECIPIENT_OTHER		0x03    // Device Request bmRequestType recipient - other

/*-------------------------------------------------------------------------*/

/*
 * STANDARD DESCRIPTORS ... as returned by GET_DESCRIPTOR, or
 * (rarely) accepted by SET_DESCRIPTOR.
 *
 * Note that all multi-byte values here are encoded in little endian
 * byte order "on the wire".  Within the kernel and when exposed
 * through the Linux-USB APIs, they are not converted to cpu byte
 * order; it is the responsibility of the client code to do this.
 * The single exception is when device and configuration descriptors (but
 * not other descriptors) are read from usbfs (i.e. /proc/bus/usb/BBB/DDD);
 * in this case the fields are converted to host endianness by the kernel.
 */

/*
 * Descriptor types ... USB 2.0 spec table 9.5
 */
#define USB_DESCRIPTOR_DEVICE                   0x01    // bDescriptorType for a Device Descriptor.
#define USB_DESCRIPTOR_CONFIGURATION            0x02    // bDescriptorType for a Configuration Descriptor.
#define USB_DESCRIPTOR_STRING                   0x03    // bDescriptorType for a String Descriptor.
#define USB_DESCRIPTOR_INTERFACE                0x04    // bDescriptorType for an Interface Descriptor.
#define USB_DESCRIPTOR_ENDPOINT                 0x05    // bDescriptorType for an Endpoint Descriptor.
#define USB_DESCRIPTOR_DEVICE_QUALIFIER         0x06    // bDescriptorType for a Device Qualifier.
#define USB_DESCRIPTOR_OTHER_SPEED              0x07    // bDescriptorType for a Other Speed Configuration.
#define USB_DESCRIPTOR_INTERFACE_POWER          0x08    // bDescriptorType for Interface Power.
/* these are from a minor usb 2.0 revision (ECN) */
#define USB_DESCRIPTOR_OTG						0x09    // bDescriptorType for an OTG Descriptor.
#define USB_DESCRIPTOR_DEBUG					0x0a
#define USB_DESCRIPTOR_INTERFACE_ASSOCIATION	0x0b
/* these are from the Wireless USB spec */
#define USB_DESCRIPTOR_SECURITY					0x0c
#define USB_DESCRIPTOR_KEY						0x0d
#define USB_DESCRIPTOR_ENCRYPTION_TYPE			0x0e
#define USB_DESCRIPTOR_BOS						0x0f
#define USB_DESCRIPTOR_DEVICE_CAPABILITY		0x10
#define USB_DESCRIPTOR_WIRELESS_ENDPOINT_COMP	0x11
#define USB_DESCRIPTOR_WIRE_ADAPTER				0x21
#define USB_DESCRIPTOR_RPIPE					0x22
#define USB_DESCRIPTOR_CS_RADIO_CONTROL			0x23
/* From the T10 UAS specification */
#define USB_DESCRIPTOR_PIPE_USAGE				0x24
/* From the USB 3.0 spec */
#define	USB_DESCRIPTOR_SS_ENDPOINT_COMP			0x30

/* Conventional codes for class-specific descriptors.  The convention is
 * defined in the USB "Common Class" Spec (3.11).  Individual class specs
 * are authoritative for their usage, not the "common class" writeup.
 */
#define USB_DESCRIPTOR_CS_DEVICE		(USB_TYPE_CLASS | USB_DESCRIPTOR_DEVICE)
#define USB_DESCRIPTOR_CS_CONFIG		(USB_TYPE_CLASS | USB_DESCRIPTOR_CONFIG)
#define USB_DESCRIPTOR_CS_STRING		(USB_TYPE_CLASS | USB_DESCRIPTOR_STRING)
#define USB_DESCRIPTOR_CS_INTERFACE		(USB_TYPE_CLASS | USB_DESCRIPTOR_INTERFACE)
#define USB_DESCRIPTOR_CS_ENDPOINT		(USB_TYPE_CLASS | USB_DESCRIPTOR_ENDPOINT)

/*-------------------------------------------------------------------------*/

/*
 * Endpoints
 */
#define USB_ENDPOINT_NUMBER_MASK		0x0f	/* in bEndpointAddress */
#define USB_ENDPOINT_DIR_MASK			0x80

#define USB_ENDPOINT_XFERTYPE_MASK		0x03	/* in bmAttributes */
#define USB_ENDPOINT_XFER_CONTROL		0x00    // Endpoint is a control endpoint.
#define USB_ENDPOINT_XFER_ISOC			0x01    // Endpoint is an isochronous endpoint.
#define USB_ENDPOINT_XFER_BULK			0x02    // Endpoint is a bulk endpoint.
#define USB_ENDPOINT_XFER_INT			0x03    // Endpoint is an interrupt endpoint.
#define USB_ENDPOINT_MAX_ADJUSTABLE		0x80

/* The USB 3.0 spec redefines bits 5:4 of bmAttributes as interrupt ep type. */
#define USB_ENDPOINT_INTRTYPE			0x30
#define USB_ENDPOINT_INTR_PERIODIC		(0 << 4)
#define USB_ENDPOINT_INTR_NOTIFICATION	(1 << 4)

#define USB_ENDPOINT_SYNCTYPE			0x0c
#define USB_ENDPOINT_SYNC_NONE			(0 << 2)
#define USB_ENDPOINT_SYNC_ASYNC			(1 << 2)
#define USB_ENDPOINT_SYNC_ADAPTIVE		(2 << 2)
#define USB_ENDPOINT_SYNC_SYNC			(3 << 2)

#define USB_ENDPOINT_USAGE_MASK			0x30
#define USB_ENDPOINT_USAGE_DATA			0x00
#define USB_ENDPOINT_USAGE_FEEDBACK		0x10
#define USB_ENDPOINT_USAGE_IMPLICIT_FB	0x20	/* Implicit feedback Data endpoint */


/*-------------------------------------------------------------------------*/

/* OTG SET FEATURE Constants    */
#define OTG_FEATURE_B_HNP_ENABLE		3       // SET FEATURE OTG - Enable B device to perform HNP
#define OTG_FEATURE_A_HNP_SUPPORT		4       // SET FEATURE OTG - A device supports HNP
#define OTG_FEATURE_A_ALT_HNP_SUPPORT	5       // SET FEATURE OTG - Another port on the A device supports HNP

/*-------------------------------------------------------------------------*/
#endif // _ch9_h_
