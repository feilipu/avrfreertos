/*
sine_generator.pde
guest openmusiclabs 7.7.11
this program creates a sinewave of variable frequency and
amplitude, presented at both left and right outputs. there
isnt any interpolation, so you only get 256 discrete frequencies
across the span of 44Hz to 10kHz.
*/

// setup codec parameters
// must be done before #includes
// see readme file in libraries folder for explanations
#define SAMPLE_RATE 44 // 44.1kHz sample rate
#define ADCS 2 // use both ADCs

// include necessary libraries
#include <Wire.h>
#include <SPI.h>
#include <AudioCodec.h>

// create data variables for audio transfer
// even though there is no input needed, the codec requires stereo data
int left_in = 0; // in from codec (LINE_IN)
int right_in = 0;
int left_out = 0; // out to codec (HP_OUT)
int right_out = 0;

// create variables for ADC results
// it only has positive values -> unsigned
unsigned int mod0_value = 0;
unsigned int mod1_value = 0;

// create sinewave lookup table
// PROGMEM stores the values in the program memory
// it is automatically included with AudioCodec.h
PROGMEM  prog_int16_t sinewave[]  = {
  // this file is stored in AudioCodec.h and is a 1024 value
  // sinewave lookup table of signed 16bit integers
  // you can replace it with your own waveform if you like
  #include <sinetable.inc>
};
unsigned int location; // lookup table value location


void setup() {
  // call this last if you are setting up other things
  AudioCodec_init(); // setup codec and microcontroller registers
}

void loop() {
  while (1); // reduces clock jitter
}

// timer1 interrupt routine - all data processed here
ISR(TIMER1_COMPA_vect, ISR_NAKED) { // dont store any registers

  // &'s are necessary on data_in variables
  AudioCodec_data(&left_in, &right_in, left_out, right_out);
  
  // create some temporary variables
  // these tend to work faster than using the main data variables
  // as they arent fetched and stored all the time
  int temp1;
  int temp2;
  
  // create a variable frequency and amplitude sinewave
  // fetch a sample from the lookup table
  temp1 = pgm_read_word_near(sinewave + location);
  // step through table at rate determined by mod1
  // use upper byte of mod1 value to set the rate
  // and have an offset of 1 so there is always an increment.
  location += 1 + (mod1_value >> 8);
  // if weve gone over the table boundary -> loop back
  // around to the other side.
  location &= 0x03ff; // fast way of doing rollover for 2^n numbers
                      // otherwise it would look like this:
                      // if (location >= 1024) {
                      // location -= 1024;
                      // }
  
  // set amplitude with mod0
  // multiply our sinewave by the mod0 value
  MultiSU16X16toH16(temp2, temp1, mod0_value);
  // our sinewave is now in temp2
  left_out = temp2; // put sinusoid out on left channel
  right_out = -temp2; // put inverted version out on right chanel

  // get ADC values
  // & is required before adc variables
  AudioCodec_ADC(&mod0_value, &mod1_value);

  reti(); // dont forget to return from the interrupt
}
