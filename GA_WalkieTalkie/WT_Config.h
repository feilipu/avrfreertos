// DAC.h

#ifndef WT_Config_h // include guard
#define WT_Config_h

#include <avr/io.h>

/*--------------------------------------------------*/
/*--------------------Globals-----------------------*/
/*--------------------------------------------------*/


#define XBEE_PACKET 64

/* Reference the handle for the serial port. */
extern xComPortHandle xSerialPort;

/* Reference the handle for the other serial port. */
extern xComPortHandle xSerial1Port;

uint16_t ch_A_out; // storage for the values to be written to MCP4822
uint16_t ch_B_out;

filter_t tx_filter; // filter for processing samples from Microphone.

uint8_t * TxDataPtr;
uint8_t * RxDataPtr;

ringBuffer_t TxBuffer;
ringBuffer_t RxBuffer;

DAC_value_t mod7_value;


// Custom structure to receive responses from AT command list below
typedef struct xb_buf_t {
  unsigned int baud;
  unsigned int payload;
} xb_buf;

// Defines sequence of AT commands to be executed (Read baud & max payload)
const xbee_atcmd_reg_t query_regs[] = {
	XBEE_ATCMD_REG( 'B', 'D', XBEE_CLT_COPY_BE, struct xb_buf_t, baud),
	XBEE_ATCMD_REG( 'N', 'P', XBEE_CLT_COPY_BE, struct xb_buf_t, payload),
   XBEE_ATCMD_REG_END       // Mark list end
};

// Define simple dispatch table for local requests only
// MUST BE A GLOBAL
const xbee_dispatch_table_entry_t xbee_frame_handlers[] =
{
	XBEE_FRAME_HANDLE_LOCAL_AT,
	XBEE_FRAME_TABLE_END
};

// Baud rate decoding table
const long xbee_baud[] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};

// Declare the XBee device structure to manage the XBee device settings
xbee_dev_t my_xbee;

/*--------------------------------------------------*/
/*-------------------Functions----------------------*/
/*--------------------------------------------------*/

static void TaskBlinkRedLED(void *pvParameters); // LED Blink

static void TaskWalkieTalkie(void *pvParameters);   // Manage WalkieTalkie initialisation & network


	// prototype for the DSP function to be implemented.
	// needs to at least provide *ch_A and *ch_B
	// within Timer0 interrupt routine - time critical I/O. Keep it short and punchy!
void audioCodec_dsp( uint16_t * ch_A, uint16_t * ch_B) __attribute__ ((hot, flatten));


// setup ADC
void AudioCodec_ADC_init(void)
{
	// turn off digital input for pin ADC7 Mic input and ADC6 Line.
	DIDR0 = _BV(ADC7D)|_BV(ADC6D)|_BV(ADC5D)|_BV(ADC4D)|_BV(ADC3D)|_BV(ADC2D)|_BV(ADC1D)|_BV(ADC0D); // turn off digital inputs
	DIDR1 = _BV(AIN1D)|_BV(AIN0D);

	ADMUX  = _BV(REFS1)|_BV(REFS0)|_BV(ADLAR)|_BV(MUX2)|_BV(MUX1)|_BV(MUX0); // 2.56V reference with external capacitor at AREF pin - left justify - start with MIC input ADC7
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // ADC enable, auto trigger, ck/128 = 192kHz
	ADCSRB =  0x00;			// free running mode
}

// adc sampling routine
static void AudioCodec_ADC(uint16_t* _modvalue) __attribute__((flatten));
static void AudioCodec_ADC(uint16_t* _modvalue)
{
	if (ADCSRA & _BV(ADIF))				// check if sample ready
	{
    	*_modvalue = ADCW;					// fetch ADCL first to freeze sample, then ADCH. It is done by the compiler.
    	ADCSRA |= _BV(ADIF);				// reset the interrupt flag
	}
}


#endif // WT_Config_h end include guard
