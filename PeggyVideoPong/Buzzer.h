/*
 * Derived from work by Ben Schmidel et al., May 23, 2008.
 * Copyright (c) 2008 Pololu Corporation. For more information, see
 *
 *   http://www.pololu.com
 *   http://forum.pololu.com
 *
 * You may freely modify and share this code, as long as you keep this
 * notice intact (including the two links above).  Licensed under the
 * Creative Commons BY-SA 3.0 license:
 *
 *   http://creativecommons.org/licenses/by-sa/3.0/
 *
 * Disclaimer: To the extent permitted by law, Pololu provides this work
 * without any warranty.  It might be defective, in which case you agree
 * to be responsible for all resulting costs and damages.
 */

#ifndef Buzzer_h
#define Buzzer_h


#ifdef __cplusplus
extern "C" {
#endif


#define PLAY_AUTOMATIC	0		// Play without further checking
#define PLAY_CHECK		1		// Play using playCheck();

//                                             n
// Equal Tempered Scale is given by f  = f  * a
//                                   n    o
//
//  where f  is chosen as A above middle C (A4) at f  = 440 Hz
//         o                                        o
//  and a is given by the twelfth root of 2 (~1.059463094359)

// key
#define C(x)			( 0 + (x)*12)
#define C_SHARP(x)		( 1 + (x)*12)
#define D_FLAT(x)		( 1 + (x)*12)
#define D(x)			( 2 + (x)*12)
#define D_SHARP(x)		( 3 + (x)*12)
#define E_FLAT(x)		( 3 + (x)*12)
#define E(x)			( 4 + (x)*12)
#define F(x)			( 5 + (x)*12)
#define F_SHARP(x)		( 6 + (x)*12)
#define G_FLAT(x)		( 6 + (x)*12)
#define G(x)			( 7 + (x)*12)
#define G_SHARP(x)		( 8 + (x)*12)
#define A_FLAT(x)		( 8 + (x)*12)
#define A(x)			( 9 + (x)*12)
#define A_SHARP(x)		(10 + (x)*12)
#define B_FLAT(x)		(10 + (x)*12)
#define B(x)			(11 + (x)*12)

// special notes
#define A4				A(4)			// center of the Equal-Tempered Scale
#define SILENT_NOTE		0xFF			// this note will silence the buzzer

#define DIV_BY_10		(1 << 15)		// frequency bit that indicates Hz/10

#define BUZZER_DDR		DDRD
#define BUZZER			(1 << PORTD3)	// Danger Shield buzzer is on D3


//*******************************************************
//            Function Definitions
//*******************************************************

	// Sets up timer 2 to play the desired frequency (in Hz or .1 Hz) for the
	// the desired duration (in ms). Allowed frequencies are 40 Hz to 10 kHz.
	// If the most significant bit of 'freq' is set, the frequency
	// is taken to be the value of the lower 15 bits in units of .1 Hz.
	// Otherwise the units are in Hz.
	// If 'freq' is set to 0Hz a silent note is played.
	//
	// Note: frequency*duration/1000 must be less than 0xFFFF (65535).  This
	//  means that you can't use a max duration of 65535 ms for frequencies
	//  greater than 1 kHz.  For example, the max duration you can use for a
	//  frequency of 10 kHz is 6553 ms.  If you use a duration longer than this,
	//  you will cause an integer overflow that produces unexpected behaviour.
void playFrequency(uint16_t freq, uint16_t duration);

	// Sets up timer 2 to play the desired note for the desired duration
	// note = key + octave * 12, where 0 <= key < 12
	// example: A4 = A + 4 * 12, where A = 9 (so A4 = 57)
	// A note is converted to a frequency by the formula:
	//   Freq(n) = Freq(0) * a^n
	// where
	//   Freq(0) is chosen as A4, which is 440 Hz
	// and
	//   a = 2 ^ (1/12)
	// n is the number of notes you are away from A4.
	// if note = 16, freq = 41.2 Hz (E1 - lower limit as freq must be >40 Hz)
	// if note = 57, freq = 440 Hz (A4 - central value of ET Scale)
	// if note = 111, freq = 9.96 kHz (D#9 - upper limit, freq must be <10 kHz)
	// if note = 255, freq = 0 kHz and buzzer is silent (silent note)
	// If 'freq' is set to 0Hz a silent note is played.
void playNote(uint8_t note, uint16_t duration);

	// Plays the specified sequence of notes.  If the play mode is
	// PLAY_AUTOMATIC, the sequence of notes will play with no further
	// action required by the user.  If the play mode is PLAY_CHECK,
	// the user will need to call playCheck() in the main loop to initiate
	// the playing of each new note in the sequence.  The play mode can
	// be changed while the sequence is playing.
	// This is modelled after the PLAY commands in GW-BASIC, with just a
	// few differences.
	//
	// The notes are specified by the int8_tacters C, D, E, F, G, A, and
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
void play(const int8_t *sequence);

	// A version of play that takes a pointer to program space instead
	// of RAM.  This is desirable since RAM is limited and the string
	// must be in program space anyway.
void playFromProgramSpace(const uint8_t *sequence_p);

// Returns 1 if the buzzer is currently playing, otherwise it returns 0
uint8_t isPlaying(void);

// Stops all sound playback immediately.
void stopPlaying(void);


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
void playMode(uint8_t mode);

	// Checks whether it is time to start another note, and starts
	// it if so.  If it is not yet time to start the next note, this method
	// returns without doing anything.  Call this as often as possible
	// in your main loop to avoid delays between notes in the sequence.
	//
	// Returns true if it is still playing.
uint8_t playCheck(void);



#ifdef __cplusplus
}
#endif


#endif // Buzzer_h

// Local Variables: **
// mode: C++ **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **
