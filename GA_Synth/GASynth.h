// DAC.h

#ifndef GASynth_h // include guard
#define GASynth_h

#include <math.h>

#include <avr/io.h>

/*--------------------------------------------------*/
/*------------Often Configured Parameters-----------*/
/*--------------------------------------------------*/


#define LINE_SIZE 		80		// size of command line (on heap)

#ifndef SAMPLE_RATE
#define SAMPLE_RATE		16000	// set up the sampling Timer1 to 48000Hz, 44100Hz (or lower), runs at audio sampling rate in Hz.
#endif							// 384 = 3 x 2^7 divisors 2k, 3k, 4k, 6k, 8k, 12k, 16k, 24k, 48k

#define LUT_SIZE		4096	// size of the wave LUT used.
								// This size allows for 12 bit accuracy (before interpolation, which we're using anyway).

#define DELAY_BUFFER	6144	// set the size of delay buffer in samples (6144 samples).

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

typedef struct {
	const int16_t * wave_table_ptr;	// Pointer to the PROGMEM LUT in use, as determined by the wave toggle.
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
	const uint32_t * note_table_ptr;// Pointer to the PROGMEM LUT describing the notes to be played.
	uint8_t settings_loaded;		// The settings have been recovered from EEPROM, or not.
	uint16_t kbd_toggle;			// Play the Concert Equal Temperament Tuning, or the Verdi / Just Intonation tuning.
	vco_t vco1;
	vco_t vco2;
	vco_t lfo;
	uint16_t xmod;					// XMOD intensity, VCO2 modulating VCO1.
	uint16_t vcf_cutoff;
	uint16_t vcf_peak;
	uint16_t delay_time;
	uint16_t delay_feedback;
	uint16_t master;				// Master intensity.
} synth_t;


/*--------------Functions-------------------*/
static void TaskWriteLCD(void *pvParameters); // Write to Gameduino 2 LCD

static void TaskMonitor(void *pvParameters);  // Serial monitor for GA Synth

static void TaskAnalogue(void *pvParameters); // Set up and manage the Analogue outputs

static void FT_touchTrackInit(void);// set up the system touch tracking and note establishment.
static uint8_t FT_touch(void);		// run the system touch management.
static void FT_GUI(void);      		// build the system GUI.

void synthesizer( uint16_t * ch_A, uint16_t * ch_B) __attribute__ ((hot, flatten));
	// the DSP function to be implemented, and called from the sample interrupt.
	// needs to at least provide *ch_A and *ch_B
	// within Timer0 or Timer1 interrupt routine - time critical I/O. Keep it short and punchy!

// get a line from the console
static void get_line (uint8_t *buff, uint8_t len);


// Define the audio (microphone) input function that is desired by uncommenting below... xxx
//#define MIC

#if defined(MIC)

#define ADCS    1		// number of ADCs in use. 1 Microphone on ADC7.

#endif


/*--------------------------------------------------*/
/*--------------------Globals-----------------------*/
/*--------------------------------------------------*/

#if ADCS == 0
  // do nothing

#elif ADCS == 1
  DAC_value_t micValue;
#endif

/*--------------------------------------------------*/
/*---------------Public Functions-------------------*/
/*--------------------------------------------------*/



	// setup ADC
void AudioCodec_ADC_init(void)
{
#if ADCS == 0
	DIDR0  = _BV(ADC7D|_BV(ADC2D)|_BV(ADC1D)|_BV(ADC0D)); // turn off digital inputs for pins ADC0, ADC1, ADC2, ADC7

#elif (ADCS == 1)
	ADMUX  = _BV(REFS1)|_BV(REFS0)|_BV(ADLAR)|_BV(MUX2)|_BV(MUX1)|_BV(MUX0); // 2.56V reference with external capacitor at AREF pin - left justify - start with MIC input ADC7
	ADCSRA = _BV(ADEN)|_BV(ADSC)|_BV(ADATE)|_BV(ADPS2)|_BV(ADPS1)|_BV(ADPS0); // ADC enable, auto trigger, ck/128 = 192kHz
	ADCSRB =  0x00;			// free running mode
	DIDR0  = _BV(ADC7D)|_BV(ADC2D)|_BV(ADC1D)|_BV(ADC0D);	// turn off digital input for pin ADC7 Mic input.
#endif
}

// adc sampling routine
#if ADCS == 0
static void AudioCodec_ADC(void) __attribute__((unused, flatten));
static void AudioCodec_ADC()
{
    ; // do nothing
}

#elif ADCS == 1
static void AudioCodec_ADC(uint16_t* _micValue) __attribute__((flatten));
static void AudioCodec_ADC(uint16_t* _micValue)
{
	if (ADCSRA & _BV(ADIF))				// check if sample ready
	{
    	*_micValue = ADCW;					// fetch ADCL first to freeze sample, then ADCH. It is done by the compiler.
    	ADCSRA |= _BV(ADIF);				// reset the interrupt flag
	}
}

#endif

#endif // GASynth_h end include guard
