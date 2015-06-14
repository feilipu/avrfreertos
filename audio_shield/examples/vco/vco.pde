/*
vco.pde
guest 7.11.11
this program creates a sinewave of variable frequency and
amplitude, presented at both left and right outputs. the frequency
is set by the left channel input, at a scale of (1/3)V/octave,
with 0v being C0.  it implements this with a logarithmic lookup
table.  the sinewave and lookup table are both interpolated to
give relatively good pitch accuracy from 16Hz to 10kHz.  there
is a frequency rollover at the top value of 3.3V input.
*/

// setup codec parameters
// must be done before #includes
// see readme file in libraries folder for explanations
#define SAMPLE_RATE 44 // 44.1kHz sample rate
#define ADCS 0 // dont use ADCs
#define ADCHPD 1 // high pass filter disabled -> DC coupled

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

// create sinewave lookup table
// PROGMEM stores the values in the program memory
// it is automatically included with AudioCodec.h
PROGMEM  prog_int16_t sinewave[]  = {
  // this file is stored in AudioCodec.h and is a 1024 value
  // sinewave lookup table of signed 16bit integers
  // you can replace it with your own waveform if you like
  #include <sinetable.inc>
};
// lookup table value location
unsigned long location; // this is a 32bit number
                        // the lower 8bits are the subsample fraction
                        // and the upper 24 bits contain the sample number  

// create logarithmic frequency lookup table
// PROGMEM stores the values in the program memory
// it is automatically included with AudioCodec.h
PROGMEM  prog_uint16_t logtable[]  = {
  // this file is stored in AudioCodec.h and is a 256 value
  // sinewave lookup table of unsigned 16bit integers
  // you can replace it with your own table if you like
  #include <logtable.inc>
};


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
  unsigned int temp1;
  unsigned int temp2;
  unsigned int temp3;
  
  // convert input signal to unsigned integer
  // input is inverted, so it needs to be flipped as well
  temp1 = 0x8000 - left_in;
  // fetch lookup table entry to convert CV to frequency
  temp2 = pgm_read_word_near(logtable + (temp1 >> 8));
  // fetch next value to do interpolation
  temp3 = pgm_read_word_near(logtable + ((temp1 >> 8) + 1));
  // interpolate result
  
  unsigned char frac = (temp1 & 0x00ff); // fetch the lower 8b
  // we are done with temp1, so we can reuse it here
  MultiU16X8toH16(temp1, temp3, frac);
  // scaled sample 2 is now in temp1, and since we are done with
  // temp3, we can reuse it for the next result
  MultiU16X8toH16(temp3, temp2, 0xff - frac);
  // temp3 now has the scaled sample 1
  temp1 += temp3; // add samples together to get an average
  // our frequency value is now in temp1
  
  // create a variable frequency sinewave
  // step through table at rate determined by temp1
  location += temp1;
  // if weve gone over the table boundary -> loop back
  location &= 0x0003ffff; // fast way for 2^n values
  // fetch a sample from the lookup table
  temp1 = location >> 8;
  temp2 = pgm_read_word_near(sinewave + temp1);
  // get next value for interpolation
  temp3 = pgm_read_word_near(sinewave + ((temp1 + 1) & 0x03ff));
  // interpolate result
  frac = (location & 0x000000ff); // fetch the lower 8b
  // we are done with temp1, so we can reuse it here
  MultiSU16X8toH16(temp1, temp3, frac);
  // scaled sample 2 is now in temp1, and since we are done with
  // temp3, we can reuse it for the next result
  MultiSU16X8toH16(temp3, temp2, 0xff - frac);
  // temp3 now has the scaled sample 1
  temp1 += temp3; // add samples together to get an average
  // our sinewave value is now in temp1
  
  left_out = temp1; // put sinusoid out on left channel
  right_out = -temp1; // put inverted version out on right chanel

  reti(); // dont forget to return from the interrupt
}
