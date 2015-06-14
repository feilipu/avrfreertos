/*
  	Buzzer.c - Library for controlling the buzzer.
    This library uses a timer2  to generate the note
	frequencies and timer2 overflow interrupt to time the duration of the
	notes, so the buzzer can be playing a melody in the background while
	the rest of your code executes. This library relies on Timer2, so it will
	conflict with any other libraries that use Timer2.
*/



#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "Buzzer.h"



#define TIMER2_OFF					0x00	// timer2 disconnected

#define TIMER2_CLK_256				0x06	// 86400 Hz
#define TIMER2_CLK_1024				0x07	// 21600 Hz

#define PRE_SCALE_256				256    // for calculating the OCR2B value
#define PRE_SCALE_1024				1024

#define TIMER2_INTERRUPT_ENABLE()	TIMSK2 = _BV(OCIE2A) // enable interrupt when counter reaches CTC value.
#define TIMER2_INTERRUPT_DISABLE()	TIMSK2 = 0

//*******************************************************
//            External Variables
//*******************************************************

// Declare these in the main.c file...
extern volatile uint8_t buzzerFinished;	// flag: 0 whilst playing
extern const int8_t *buzzerSequence;
extern uint8_t buzzerInitialized;


//*******************************************************
//            Local Variables
//*******************************************************

static volatile uint16_t buzzerTimeout = 0;		// tracks buzzer time limit
static int8_t play_mode_setting = PLAY_AUTOMATIC;

static uint8_t use_program_space; 			// boolean: true if we should
											// use program space

// music settings and defaults
static uint8_t octave = 4;					// the current octave
static uint16_t whole_note_duration = 2000;	// the duration of a whole note
static uint16_t note_type = 4;              // 4 for quarter, etc
static uint16_t duration = 500;				// the duration of a note in ms

// staccato handling
static uint8_t staccato = false; 			// true if playing staccato
static uint8_t staccato_rest_duration;		// duration of a staccato
											// rest, or zero if it is time
											// to play a note
// silence handling
static uint8_t silence = false;				// true if there is a silent note


//*******************************************************
//            Local Function Definition
//*******************************************************


static int8_t currentCharacter(void);
static uint16_t getNumber(void);
static void nextNote(void);



//*******************************************************
//            External Functions
//*******************************************************


// Set up timer 2 to play the desired frequency (in Hz or .1 Hz) for the
// the desired duration (in ms). Allowed frequencies are 45 Hz to 10 kHz.
//
// Note: frequency*duration/1000 must be less than 0xFFFF (65535).  This
//  means that you can't use a max duration of 65535 ms for frequencies
//  greater than 1 kHz.  For example, the max duration you can use for a
//  frequency of 10 kHz is 6553 ms.  If you use a duration longer than this,
//  you will cause an integer overflow that produces unexpected behaviour.
//
// If 'freq' is set to 0Hz a silent note is played for 'dur' duration.

void playFrequency(uint16_t freq, uint16_t dur)
{
	buzzerFinished = 0;

	uint8_t newOCR2A;
	uint8_t newTCCR2B;
	uint8_t multiplier = 1;

	uint16_t timeout;

	if (freq & DIV_BY_10)		// if frequency's DIV_BY_10 bit is set
	{							//  then the true frequency is freq/10
		multiplier = 10;		//  (gives higher resolution for small freqs)
		freq &= ~DIV_BY_10;		// clear DIV_BY_10 bit
	}

	if (freq == 0)				// signal for silence
	{
		silence = true;			// set the flag for no output
		freq = 1000;			// and set a dummy frequency
	}
	else silence = false;

	// calculate necessary clock source and counter top value to get freq
	if (freq > 180 * ((uint16_t)multiplier))	// use clock prescaler = 256
	{
		if (freq > 10000)
			freq = 10000;			// max frequency allowed is 10kHz

		newOCR2A = (uint8_t)( F_CPU / PRE_SCALE_256 / 2 / freq  ) - 1;

		// timer2 clock select:
		newTCCR2B = TIMER2_CLK_256;	// select IO clk (prescaler = 256)
	}

	else									// use clock prescaler = 1024
	{
		uint8_t val = 45 * multiplier;
		if (freq < val)	freq = val;			// min frequency allowed is 45 Hz


		// set top (frequency):
		if (multiplier == 10)
			newOCR2A = (uint8_t)( F_CPU / PRE_SCALE_1024 / 2 * 10 / freq  ) - 1;
		else
			newOCR2A = (uint8_t)( F_CPU / PRE_SCALE_1024 / 2 / freq  ) - 1;

		// timer2 clock select
		newTCCR2B = TIMER2_CLK_1024;	// select IO clk / 1024
	}


	// set timeout (duration):
	if (multiplier == 10)
		freq = (freq + 5) / 10;

	if (silence == true)// duration for silent notes is exact
		timeout = dur;
	else
		timeout = (uint16_t)((long)dur * freq / 1000);



	// Since the Buzzer is connected to OCR2B, we have to use CTC mode. i.e Mode 2 for WGM22 WGM21 WGM20
	// OCR2B doesn't toggle for pwm modes
	// OCR2B has 'reserved' for the pwm modes, but no output toggle.

	// We're going to count from 0 to the OCR2A value, using and generate frequency (with 50% duty cycle)
	// waveform output to generate a tone on the PD3 pin.

	// In Clear Timer on Compare or CTC mode (WGM22:0 = 2), the OCR2A Register is used to
	// manipulate the counter resolution. In CTC mode the counter is cleared to zero when the counter
	// value (TCNT2) matches the OCR2A. The OCR2A defines the top value for the counter, hence
	// also its resolution. This mode allows greater control of the compare match output frequency. It
	// also simplifies the operation of counting external events.

	// FREQ = F_CPU / 2 * PRE_SCALE * (1 + OCR2A) or

	// OCR2A =  ( F_CPU / 2 / PRE_SCALE / INITIAL_FREQ ) - 1
	// 			( 22118400 / 2 / 256 / 440 ) - 1
	// 		=   97
	//			97 == 440.81 Hz

	// 			255 == 168.75 Hz at Clock Scale 256
	//			0   == 43200 Hz at Clock Scale 256

	//			256 == 42.18 Hz at Clock Scale 1024
	//			0   == 10800 Hz at Clock Scale 1024


	// In TCCR2A & TCCR2B only set WGM21, as we are looking for Mode 2 which is CTC.
	// In TCCR2A set COM2B0 so that we will toggle the output OCR2B when count reaches OCR2B.


	TIMER2_INTERRUPT_DISABLE();			// disable all timer2 interrupts while writing
										// to 16-bit registers

	TIFR2 |= 0x0F;						// clear any pending Timer overflow interrupt.

	TCCR2A = _BV(COM2B0) | _BV(WGM21);	// Set up CTC (Mode 2) with output toggle on OCR2B (buzzer pin)
	TCCR2B = newTCCR2B;					// set timer 2 clock prescaler

	OCR2A = newOCR2A;					// set the OCR2A to have the value to which we're counting,
										// since we can't count up to OCR2B.
										// Not sure why we DON'T have to set this: OCR2B = newOCR2A
										// to get the compare output toggle, but it works anyway.

	buzzerTimeout = timeout;			// set buzzer duration

	TIMER2_INTERRUPT_ENABLE();			// the Timer 2 (compare match on OCR2A) interrupt is enabled.

	if (silence == true)
		BUZZER_DDR &= ~BUZZER;			// Turn off PD3 Buzzer pin.
	else
		BUZZER_DDR |= BUZZER;			// PD3 buzzer pin set as an output
}

/*-----------------------------------------------------------*/

// Determine the frequency for the specified note, then play that note
//  for the desired duration (in ms).  This is done without using floats
//  and without having to loop.  volume controls buzzer volume, with 15 being
//  loudest and 0 being quietest.

// Note: frequency*duration/1000 must be less than 0xFFFF (65535).  This
//  means that you can't use a max duration of 65535 ms for frequencies
//  greater than 1 kHz.  For example, the max duration you can use for a
//  frequency of 10 kHz is 6553 ms.  If you use a duration longer than this,
//  you will cause an integer overflow that produces unexpected behavior.
void playNote(uint8_t note, uint16_t dur)
{
	// note = key + octave * 12, where 0 <= key < 12
	// example: A4 = A + 4 * 12, where A = 9 (so A4 = 57)
	// A note is converted to a frequency by the formula:
	//   Freq(n) = Freq(0) * a^n
	// where
	//   Freq(0) is chosen as A4, which is 440 Hz
	// and
	//   a = 2 ^ (1/12)
	// n is the number of notes you are away from A4.
	// One can see that the frequency will double every 12 notes.
	// This function exploits this property by defining the frequencies of the
	// 12 lowest notes allowed and then doubling the appropriate frequency
	// the appropriate number of times to get the frequency for the specified
	// note.

	// if note = 16, freq = 41.2 Hz (E1 - lower limit as freq must be >40 Hz)
	// if note = 57, freq = 440 Hz (A4 - central value of ET Scale)
	// if note = 111, freq = 9.96 kHz (D#9 - upper limit, freq must be <10 kHz)
	// if note = 255, freq = 1 kHz and buzzer is silent (silent note)

	// The most significant bit of freq is the "divide by 10" bit.  If set,
	// the units for frequency are .1 Hz, not Hz, and freq must be divided
	// by 10 to get the true frequency in Hz.  This allows for an extra digit
	// of resolution for low frequencies without the need for using floats.


	uint16_t freq = 0;
	uint8_t offset_note = note - 16;

	if (note == SILENT_NOTE )
	{
		freq = 0;	// silent notes => use 0kHz freq (to signal)
		playFrequency(freq, dur);
		return;
	}

	if (note <= 16)
		offset_note = 0;
	else if (offset_note > 95)
		offset_note = 95;

	uint8_t exponent = offset_note / 12;

	// frequency table for the lowest 12 allowed notes
	//   frequencies are specified in tenths of a Hertz for added resolution
	switch (offset_note - exponent * 12)	// equivalent to (offset_note % 12)
	{
		case 0:				// note E1 = 41.2 Hz
			freq = 412;
			break;
		case 1:				// note F1 = 43.7 Hz
			freq = 437;
			break;
		case 2:				// note F#1 = 46.3 Hz
			freq = 463;
			break;
		case 3:				// note G1 = 49.0 Hz
			freq = 490;
			break;
		case 4:				// note G#1 = 51.9 Hz
			freq = 519;
			break;
		case 5:				// note A1 = 55.0 Hz
			freq = 550;
			break;
		case 6:				// note A#1 = 58.3 Hz
			freq = 583;
			break;
		case 7:				// note B1 = 61.7 Hz
			freq = 617;
			break;
		case 8:				// note C2 = 65.4 Hz
			freq = 654;
			break;
		case 9:				// note C#2 = 69.3 Hz
			freq = 693;
			break;
		case 10:			// note D2 = 73.4 Hz
			freq = 734;
			break;
		case 11:			// note D#2 = 77.8 Hz
			freq = 778;
			break;
	}

	if (exponent < 7)
	{
		freq = freq << exponent;	// frequency *= 2 ^ exponent
		if (exponent > 1)			// if the frequency is greater than 160 Hz
			freq = (freq + 5) / 10;	//   we don't need the extra resolution
		else
			freq += DIV_BY_10;		// else keep the added digit of resolution
	}
	else
		freq = (freq * 64 + 2) / 5;	// == freq * 2^7 / 10 without int overflow

	playFrequency(freq, dur);	// set buzzer this freq/duration

}


/*-----------------------------------------------------------*/

// Plays the specified sequence of notes.  If the play mode is
// PLAY_AUTOMATIC, the sequence of notes will play with no further
// action required by the user.  If the play mode is PLAY_CHECK,
// the user will need to call playCheck() in the main loop to initiate
// the playing of each new note in the sequence.  The play mode can
// be changed while the sequence is playing.
// This is modelled after the PLAY commands in GW-BASIC, with just a
// few differences.
//
// The notes are specified by the characters C, D, E, F, G, A, and
// B, and they are played by default as "quarter notes" with a
// length of 500 ms.  This corresponds to a tempo of 120
// beats/min.  Other durations can be specified by putting a number
// immediately after the note.  For example, C8 specifies C played as an
// eighth note, with half the duration of a quarter note. The special
// note R plays a rest (no sound).
//
// Various control characters alter the sound:
//   '>' plays the next note one octave higher
//
//   '<' plays the next note one octave lower
//
//   '+' or '#' after a note raises any note one half-step
//
//   '-' after a note lowers any note one half-step
//
//   '.' after a note "dots" it, increasing the length by
//       50%.  Each additional dot adds half as much as the
//       previous dot, so that "A.." is 1.75 times the length of
//       "A".
//
//   'O' followed by a number sets the octave (default: O4).
//
//   'T' followed by a number sets the tempo (default: T120).
//
//   'L' followed by a number sets the default note duration to
//       the type specified by the number: 4 for quarter notes, 8
//       for eighth notes, 16 for sixteenth notes, etc.
//       (default: L4)
//
//   'MS' sets all subsequent notes to play staccato - each note is played
//       for 1/2 of its allotted time, followed by an equal period
//       of silence.
//
//   'ML' sets all subsequent notes to play legato - each note is played
//       for its full length.  This is the default setting.
//
//   '!' resets all persistent settings to their defaults.
//
// The following plays a c major scale up and back down:
//   play("L16 cdefgab>cbagfedc");
//
// Here is an example from Bach:
//   play("T240 L8 a gafaeada c+adaeafa <aa<bac#ada c#adaeaf4");
void play(const int8_t *notes)
{
	TIMER2_INTERRUPT_DISABLE();	// prevent this from being interrupted
	buzzerSequence = notes;
	use_program_space = 0;
	staccato_rest_duration = 0;
	nextNote();					// this re-enables the timer2 interrupt
}

/*-----------------------------------------------------------*/

void playFromProgramSpace(const uint8_t *notes_p )
{
	TIMER2_INTERRUPT_DISABLE();	// prevent this from being interrupted
	buzzerSequence = (const int8_t *) notes_p;
	use_program_space = 1;
	staccato_rest_duration = 0;
	nextNote();					// this re-enables the timer2 interrupt
}

/*-----------------------------------------------------------*/

// Returns 1 if the buzzer is currently playing, otherwise it returns 0
uint8_t isPlaying(void)
{
	return !buzzerFinished || buzzerSequence != 0;
}

/*-----------------------------------------------------------*/

// stop all sound playback immediately
void stopPlaying(void)
{
	TIMER2_INTERRUPT_DISABLE();		// disable interrupts
	TCCR2B = 0;						// disable Timer 2 clock
	OCR2A = 0;
	OCR2B = 0;
	BUZZER_DDR &= ~BUZZER;			// Turn off PD3 Buzzer pin.
	buzzerFinished = 1;
	buzzerSequence = 0;
}

/*-----------------------------------------------------------*/

// This puts play() into a mode where instead of advancing to the
// next note in the sequence automatically, it waits until the
// function playCheck() is called. The idea is that you can
// put playCheck() in your main loop and avoid potential
// delays due to the note sequence being checked in the middle of
// a time sensitive calculation.  It is recommended that you use
// this function if you are doing anything that can't tolerate
// being interrupted for more than a few microseconds.
// Note that the play mode can be changed while a sequence is being
// played.
//
// Usage: playMode(PLAY_AUTOMATIC) makes it automatic (the
// default), playMode(PLAY_CHECK) sets it to a mode where you have
// to call playCheck().
void playMode(uint8_t mode)
{
	play_mode_setting = mode;

	// We want to check to make sure that we didn't miss a note if we
	// are going out of play-check mode.
	if(mode == PLAY_AUTOMATIC)
		playCheck();
}

/*-----------------------------------------------------------*/

// Checks whether it is time to start another note, and starts
// it if so.  If it is not yet time to start the next note, this method
// returns without doing anything.  Call this as often as possible
// in your main loop to avoid delays between notes in the sequence.
//
// Returns true if it is still playing.
uint8_t playCheck(void)
{
	TIMER2_INTERRUPT_DISABLE();
	if(buzzerFinished && buzzerSequence != 0)
		nextNote();
	TIMER2_INTERRUPT_ENABLE();
	return buzzerSequence != 0;
}

//*******************************************************
//            Internal Functions
//*******************************************************


// Gets the current character, converting to lower-case and skipping
// spaces.  For any spaces, this automatically increments sequence!
static int8_t currentCharacter(void)
{
	int8_t c = 0;
	do
	{
		if(use_program_space)
			c = pgm_read_byte_near(buzzerSequence);
		else
			c = *buzzerSequence;

		if(c >= 'A' && c <= 'Z')
			c += 'a'-'A';
	} while(c == ' ' && (buzzerSequence ++));

	return c;
}

/*-----------------------------------------------------------*/

// Returns the numerical argument specified at buzzerSequence[0] and
// increments sequence to point to the character immediately after the
// argument.
static uint16_t getNumber(void)
{
	uint16_t arg = 0;

	// read all digits, one at a time
	int8_t c = currentCharacter();
	while(c >= '0' && c <= '9')
	{
		arg *= 10;
		arg += c-'0';
		buzzerSequence ++;
		c = currentCharacter();
	}

	return arg;
}

/*-----------------------------------------------------------*/

static void nextNote(void)
{
	uint8_t note = 0;
	uint8_t rest = 0;
	uint8_t tmp_octave = octave; // the octave for this note
	uint16_t tmp_duration; // the duration of this note
	uint16_t dot_add;

	int8_t c; // temporary variable

	// if we are playing staccato, after every note we play a rest
	if(staccato && staccato_rest_duration)
	{
		playNote(SILENT_NOTE, staccato_rest_duration);
		staccato_rest_duration = 0;
		return;
	}

 parse_character:

	// Get current character
	c = currentCharacter();
	buzzerSequence ++;

	// Interpret the character.
	switch(c)
	{
	case '>':
		// shift the octave temporarily up
		tmp_octave ++;
		goto parse_character;
	case '<':
		// shift the octave temporarily down
		tmp_octave --;
		goto parse_character;
	case 'a':
		note = A(0);
		break;
	case 'b':
		note = B(0);
		break;
	case 'c':
		note = C(0);
		break;
	case 'd':
		note = D(0);
		break;
	case 'e':
		note = E(0);
		break;
	case 'f':
		note = F(0);
		break;
	case 'g':
		note = G(0);
		break;
	case 'l':
		// set the default note duration
		note_type = getNumber();
		duration = whole_note_duration/note_type;
		goto parse_character;
	case 'm':
		// set music staccato or legato
		if(currentCharacter() == 'l')
			staccato = false;
		else
		{
			staccato = true;
			staccato_rest_duration = 0;
		}
		buzzerSequence ++;
		goto parse_character;
	case 'o':
		// set the octave permanently
		octave = getNumber();
		tmp_octave = octave;
		goto parse_character;
	case 'r':
		// Rest - the note value doesn't matter.
		rest = 1;
		break;
	case 't':
		// set the tempo
		whole_note_duration = 60*400/getNumber()*10;
		duration = whole_note_duration/note_type;
		goto parse_character;
	case '!':
		// reset to defaults
		octave = 4;
		whole_note_duration = 2000;
		note_type = 4;
		duration = 500;
		staccato = 0;
		// reset temp variables that depend on the defaults
		tmp_octave = octave;
		tmp_duration = duration;
		goto parse_character;
	default:
		buzzerSequence = 0;
		return;
	}

	note += tmp_octave*12;

	// handle sharps and flats
	c = currentCharacter();
	while(c == '+' || c == '#')
	{
		buzzerSequence ++;
		note ++;
		c = currentCharacter();
	}
	while(c == '-')
	{
		buzzerSequence ++;
		note --;
		c = currentCharacter();
	}

	// set the duration of just this note
	tmp_duration = duration;

	// If the input is 'c16', make it a 16th note, etc.
	if(c > '0' && c < '9')
		tmp_duration = whole_note_duration/getNumber();

	// Handle dotted notes - the first dot adds 50%, and each
	// additional dot adds 50% of the previous dot.
	dot_add = tmp_duration/2;
	while(currentCharacter() == '.')
	{
		buzzerSequence ++;
		tmp_duration += dot_add;
		dot_add /= 2;
	}

	if(staccato)
	{
		staccato_rest_duration = tmp_duration / 2;
		tmp_duration -= staccato_rest_duration;
	}

	// this will re-enable the timer2 overflow interrupt
	playNote(rest ? SILENT_NOTE : note, tmp_duration);
}


//*******************************************************
//            Interrupt Handler
//*******************************************************


// Timer2 overflow interrupt
ISR (TIMER2_COMPA_vect)
{
	if (buzzerTimeout-- == 0)
	{
		TIMER2_INTERRUPT_DISABLE();
		sei();						// re-enable global interrupts (because nextNote() is very slow)
		TCCR2B = 0;					// clear Timer 2 clock
		OCR2A = 0;
		OCR2B = 0;
		BUZZER_DDR &= ~BUZZER;		// Turn off PD3 Buzzer pin.
		buzzerFinished = 1;
		if (buzzerSequence && (play_mode_setting == PLAY_AUTOMATIC))
			nextNote();
	}
}


// Local Variables: **
// mode: C **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **
