// DAC.h

#ifndef GASynth_h // include guard
#define GASynth_h

#include <math.h>

#include <avr/io.h>

/* debouncer include file, */
#include "buttonDebounce.h"

/*--------------------------------------------------*/
/*------------Often Configured Parameters-----------*/
/*--------------------------------------------------*/


#define LINE_SIZE 		40		// size of command line (on heap)

#define LUT_SIZE		4096	// size of the wave LUT used.
								// This size allows for 12 bit accuracy (before interpolation, which we're using anyway).

#define DELAY_BUFFER	0x1A2A	// set the size of delay buffer in samples (6698 samples).
#define DECIMATE		2		// improve the accuracy of the potentiometer sampling, by decimation.

#ifndef DECIMATE
  #define DECIMATE 				2		// number of samples for decimation accuracy.  4^DECIMATE samples are taken before reporting ADC value.
#elif (DECIMATE < 1 ) || ((DECIMATE >= 3))
  #error DECIMATE value not properly defined
#endif



//#define configTOTAL_HEAP_SIZE	( (size_t )  15699  )		// set this in freeRTOSBoardDefs.h

/*--------------Definitions-------------------*/
// Define Touch Tags
// Keyboard Touch Tags are automatically set to the ASCII value of the key.

#define KBD_TOGGLE		0x01
#define SETTINGS		0x02

#define VCO1_TOGGLE		0x12
#define VCO1_WAVE		0x13

#define VCO2_TOGGLE		0x22
#define VCO2_WAVE		0x23

#define LFO_TOGGLE		0x32
#define LFO_WAVE 		0x33

#define VCO1_PITCH		0x81
#define VCO2_PITCH		0x82
#define LFO_PITCH		0x83

#define MIXER_VCO1		0x91
#define MIXER_VCO2		0x92
#define MIXER_LFO		0x93
#define MIXER_XMOD		0x94

#define VCF_CUTOFF		0xa1
#define VCF_PEAK		0xa2

#define DELAY_TIME		0xb1
#define DELAY_FEEDBACK	0xb2

#define MASTER			0xf1

#define KBD_CONCERT		0x0000
#define KBD_VERDI		0xffff

#define WAVE_OFF		0x0000
#define WAVE_ON			0xffff

#define WAVE_SQR		0x0000
#define WAVE_SAW		0xffff
#define WAVE_SIN		0x0000
#define WAVE_TRI		0xffff

#define STOPS 			8

#define STOP_C1			0x0000
#define STOP_C2			0x2000
#define STOP_C3			0x4000
#define STOP_C4			0x6000
#define STOP_C5			0x8000
#define STOP_C6			0xa000
#define STOP_C7			0xc000
#define STOP_C8			0xe000

#define NOTES			12

#define VCO1_BUTTON		(_BV(PIND5))
#define VCO2_BUTTON		(_BV(PIND6))
#define LFO_BUTTON		(_BV(PIND7))
#define VCF_BUTTON		(_BV(PIND6)|_BV(PIND5))
#define DELAY_BUTTON	(_BV(PIND7)|_BV(PIND6))
#define CANCEL_BUTTON	(_BV(PIND7)|_BV(PIND6)|_BV(PIND5))

#define BUTTON_MASK		(_BV(PIND7)|_BV(PIND6)|_BV(PIND5))


/*--------------Type Definitions-------------------*/

typedef struct {
	uint16_t tag;
	uint16_t value;
} track_return_t;

typedef union {
	uint32_t u32;
	uint16_t u16[2];
	uint8_t  u8[4];
	track_return_t touch;
} touch_t;

typedef enum {
	off = 0,
	attack = 1,
	decay = 2,
	sustain = 3,
	release = 4
} note_state;

typedef struct {
	int16_t const * wave_table_ptr;	// Pointer to the PROGMEM LUT in use, as determined by the wave toggle.
	uint32_t phase;					// The lower 8bits are the subsample fraction and
									// The upper 24 bits contain the sample number to read from the LUT.
	uint32_t phase_increment;		// The lower 8bits are the subsample increment fraction and
									// The upper 24 bits contain the sample increment.

	uint16_t pitch;					// For VCO1 contains the keyboard register or pitch.
									// For VCO2 contains relative pitch. For LFO contains absolute pitch.
	uint16_t volume;				// VCO intensity.

	uint16_t wave;					// Just a toggle for which of two wave LUTs are selected for the VCO.
	uint16_t toggle;				// Just a toggle for VCO on or off.
} vco_t;

typedef struct {
	uint8_t note;					// The note is being played, or not.
	uint32_t const * note_table_ptr;// Pointer to the PROGMEM LUT describing the notes to be played.
	uint8_t settings_loaded;		// The settings have been recovered from EEPROM, or not.
	uint16_t kbd_toggle;			// Play the Concert Equal Temperament Tuning, or the Verdi / Just Intonation tuning.
	uint8_t potentiometer_code;		// Coding for the assignment of the potentiometers, based on button codes.
	vco_t vco1;
	vco_t vco2;
	vco_t lfo;
	uint16_t xmod;					// XMOD intensity, VCO2 modulating VCO1.
	note_state adsr;				// The note is being played, or not.
	uint16_t adsr_phase;			// attack and release increment value,
									// stepping through the attack, decay, sustain, and release table.
	uint16_t const * adsr_table_ptr;// Pointer to the PROGMEM LUT describing the adsr function to be played.
	uint16_t vcf_cutoff;
	uint16_t vcf_peak;
	uint16_t delay_time;
	uint16_t delay_feedback;
	uint16_t master;				// Master intensity.
} synth_t;


/*--------------Functions-------------------*/

int main(void) __attribute__((OS_main));

static void TaskWriteLCD(void *pvParameters); // Write to Gameduino 2 LCD
static void TaskMonitor(void *pvParameters);  // Serial monitor for GA Synth
static void TaskAnalogue(void *pvParameters); // Set up and manage the Analogue outputs

static void FT_touchTrackInit(void);// set up the system touch tracking and note establishment.
static uint8_t FT_touch(void);		// run the system touch management.
static void FT_GUI(void);      		// build the system GUI.

static void AudioCodec_ADC_init(void); // initialise the ADC for sampling the potentiometers
static void AudioCodec_ADC(uint16_t * _mod0value, uint16_t * _mod1value) __attribute__((hot, flatten));

static void shieldDButtonInit(debouncer * portDebounce, uint8_t buttons, uint8_t pulledUpButtons ); // initialise the Shield Buttons
static uint8_t shieldPhysicalIO(uint8_t button) __attribute__((hot, flatten));

void synthesizer( uint16_t * ch_A, uint16_t * ch_B) __attribute__ ((hot, flatten));
	// the DSP function for the Synthesiser, and called from the reconstruction sampling interrupt.
	// needs to at least provide *ch_A and *ch_B
	// within Timer0/1 interrupt routine - time critical I/O. Keep it short and punchy!

// get a line from the console
static void get_line (uint8_t *buff, uint8_t len);


/*--------------------------------------------------*/
/*--------------------Globals-----------------------*/
/*--------------------------------------------------*/


uint8_t _i = (uint8_t)(_BV(2 * DECIMATE) * 2);	// 4 ^ DECIMATE for two ADCS

uint16_t mod0Value;			// current value of the 0 potentiometer
uint16_t mod1Value;			// current value of the 1 potentiometer

/* Debouncing filter for the three buttons on the Sparkfun MIDI Shield which are altered to be on PORTD7, 6, and 5 */
debouncer portd;

/*--------------------------------------------------*/
/*---------------Private Functions-------------------*/
/*--------------------------------------------------*/

static void AudioCodec_ADC_init(void)
{
	DIDR0 = _BV(ADC7D)|_BV(ADC6D)|_BV(ADC5D)|_BV(ADC4D)|_BV(ADC3D)|_BV(ADC2D)|_BV(ADC1D)|_BV(ADC0D); // turn off digital inputs
	DIDR1 = _BV(AIN1D)|_BV(AIN0D);
	// setup ADCs
	ADMUX  = _BV(REFS0); // start with ADC0 - AVCC with external capacitor at AREF pin
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // ADC enable, auto trigger, ck/128 of F_CPU
	ADCSRB = 0x00; // free running mode
}

static void AudioCodec_ADC(uint16_t * _mod0Value, uint16_t * _mod1Value)
{
    if (ADCSRA & _BV(ADIF))				// check if sample ready
    {
      if (_i >= _BV(2 * DECIMATE))			// sample ADC0
      {
    	uint16_t static _mod0Temp;

        _mod0Temp += ADCW; 					// fetch ADCL first to freeze sample, is done by the compiler

        if (_i == _BV(2 * DECIMATE))		// decimate and return the ADC0 result.
        {
        	_mod0Temp >>= DECIMATE;			// Decimate the summed samples (to get better accuracy), see AVR8003.doc
            *_mod0Value = _mod0Temp;		// move temp value to return
			_mod0Temp = 0x0000;				// reset temp value
			ADMUX = 0x41;					// switch to ADC1 - AVCC with external capacitor at AREF pin
        }
        ADCSRA = 0xf7;						// reset the interrupt flag
      }

      else if (_i < (_BV(2 * DECIMATE)) )	// sample ADC1
      {
    	uint16_t static _mod1Temp;

    	_mod1Temp += ADCW;					// fetch ADCL first to freeze sample, is done by the compiler

		if (_i == 0)						// decimate and return the ADC1 result.
		{
			_mod1Temp >>= DECIMATE;			// Decimate the summed samples (to get better accuracy), see AVR8003.doc
			*_mod1Value = _mod1Temp;		// move temp value to return
			_mod1Temp = 0x0000;				// reset temp value
			ADMUX = 0x40;					// switch to ADC0 - AVCC with external capacitor at AREF pin

			_i = _BV(2 * DECIMATE) * 2;		// reset loop counter
		}
        ADCSRA = 0xf7;						// reset the interrupt flag
      }

      --_i; // decrement the sample counter
   }
}


//      port - The address of a Debouncer instantiation.
//      buttons - Specifies which portpins are attached to buttons.
//      pulledUpButtons - Specifies whether pullups or pulldowns are being used on
//          the port pins. This is the BUTTON_PIN_* 's that are being
//          pulled up. Otherwise, the debouncer assumes that the other
//          buttons are being pulled down. A 0 bit means pulldown.
//          A 1 bit means pullup. For example, 00010001 means that
//          button 0 and button 4 are both being pulled up. All other
//          buttons have pulldowns if they represent buttons.
// Returns:
//      None
//
static void shieldDButtonInit(debouncer * portDebounce, uint8_t buttons, uint8_t pulledUpButtons )
{
	// The MIDIShield buttons connected to PD5, PD6, and PD7.
	// This is moved from the original button position of PD4, PD3, and PD2, to avoid the ATmega1284p USART1 pins.
	DDRD  &= ~(buttons);			// DDRx set PD5, PD6, and PD7 as input.
	PORTD |= pulledUpButtons;	// PORTx set pull up on PD5, PD6, and PD7.

	// initialise the debouncer with D7, D6 and D5 pins pulled up.
	buttonDebounceInit( portDebounce, pulledUpButtons );
}

#endif // GASynth_h end include guard
