/*
tremolo.pde
guest openmusiclabs 7.11.11
this program creates a mono tremolo effect by taking input
from the left channel and multiplying it by a low frequency
sinewave.  the frequency is set with MOD1, and the depth is
set with MOD0.  the amplitude modulated signal is presented
at the right output, and a mix of the wet and dry signals
is presented at the left output.
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
// even though the function is mono, the codec requires stereo data
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
  // this file is stored in the AudioCodec library and is a
  // 1024 value sinewave lookup table of signed 16bit integers.
  // you can replace it with your own waveform if you like.
  #include <sinetable.inc>
};
 // lookup table value location
unsigned long location; // this is a 32bit number
                        // the lower 8bits are the subsample fraction
                        // and the upper 24 bits contain the sample number  


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
  unsigned int temp0;
  unsigned char frac;
  int temp1;
  int temp2;
  int temp3;
 
  // create a variable frequency and amplitude sinewave.
  // since we will be moving through the lookup table at
  // a variable frequency, we wont always land directly
  // on a single sample.  so we will average between the
  // two samples closest to us.  this is called interpolation.
  // step through the table at rate determined by mod1
  // use upper byte of mod1 value to set the rate
  // and have an offset of 1 so there is always an increment.
  location += 1 + (mod1_value >> 8);
  // if weve gone over the table boundary -> loop back
  location &= 0x0003ffff; // this is a faster way doing the table
                          // wrap around, which is possible
                          // because our table is a multiple of 2.
                          // otherwise you would do something like:
                          // if (location >= 1024*256) {
                          //   location -= 1024*256;
                          // }
  temp0 = (location >> 8);
  // get first sample and store it in temp1
  temp1 = pgm_read_word_near(sinewave + temp0);
  ++temp0; // go to next sample
  temp0 &= 0x03ff; // check if weve gone over the boundary.
                   // we can do this because its a multiple of 2,
                   // otherwise it would be:
                   // if (temp0 >= 1024) {
                   //   temp0 = 0; // reset to 0
                   // }
  // get second sample and put it in temp2
  temp2 = pgm_read_word_near(sinewave + temp0);
  
  // interpolate between samples
  // multiply each sample by the fractional distance
  // to the actual location value
  frac = (location & 0x000000ff); // fetch the lower 8b
  MultiSU16X8toH16(temp3, temp2, frac);
  // scaled sample 2 is now in temp3, and since we are done with
  // temp2, we can reuse it for the next result
  MultiSU16X8toH16(temp2, temp1, 0xff - frac);
  // temp2 now has the scaled sample 1
  temp2 += temp3; // add samples together to get an average
  // our sinewave is now in temp2
  
  // set amplitude with mod0
  // multiply our sinewave by the mod0 value
  MultiSU16X16toH16(temp1, temp2, mod0_value);
  // our sinewave is now in temp1
  
  // create a tremolo effect by multiplying input signal by sinewave
  // turn signed sinewave value into unsigned value
  temp1 += 0x8000;
  MultiSU16X16toH16(right_out, left_in, temp1);
  // put amplitude modulated data at right output
  // mix modulated and current data at left output
  // divide each by 2 for proper scaling
  left_out = (right_out >> 1) + (left_in >> 1);

  // & is required before adc variables
  AudioCodec_ADC(&mod0_value, &mod1_value);
  
  reti(); // dont forget to return from the interrupt
}
