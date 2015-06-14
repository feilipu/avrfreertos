/*
flanger.pde
guest openmusiclabs 7.7.11
this program implements a mono, audio-rate flanger.  it takes
input from the left channel, and delays it by a varying
amount.  the rate of this variation is set by the MOD1 knob,
and the amplitude is set by the MOD0 knob.  it then presents
this delayed data on the right channel, and a mix of the wet
and dry on the left channel.
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
unsigned int lookup; // lookup table value location

// create a delay buffer in memory
#define SIZE 800 // buffer size is limited by microcontroller SRAM size
int delaymem[SIZE]; // 800 positions x 2 bytes = 1600 bytes of SRAM
unsigned int location = 0; // buffer location to read/write from

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
  temp1 = pgm_read_word_near(sinewave + lookup);
  // step through table at rate determined by mod1
  // use upper byte of mod1 value to set the rate
  // and have an offset of 1 so there is always an increment.
  lookup += 1 + (mod1_value >> 8);
  // if weve gone over the table boundary -> loop back
  // around to the other side.
  lookup &= 0x03ff; // fast way of dealing with rollover
                    // only works for 2^n values
  // set amplitude with mod0
  // multiply our sinewave by the mod0 value
  MultiSU16X16toH16(temp2, temp1, mod0_value);
  // our sinewave is now in temp2
  
  // create a flanger effect by moving through delayed data
  // store incoming data
  delaymem[location++] = left_in; // post increment location to go to next memory location
  // check if location has gotten bigger than buffer size
  if (location >= SIZE) {
    location = 0; // reset location
  }
  // fetch delayed data with sinusoidal offset (temp2)
  unsigned int x; // create a temporary buffer index
  x = location + (SIZE/2) + (temp2 >> 8);
  if (x >= SIZE) { // check for buffer overflow
    x -= SIZE;
  }
  // fetch delayed data
  temp1 = delaymem[x];
  // fetch next delayed data for interpolation
  if (++x == SIZE) { // check for buffer overflow
    x = 0;
  }
  // we need some more temp variables
  int temp3;
  int temp4; 
  temp3 = delaymem[x];
  // interpolate between values
  MultiSU16X8toH16(temp4, temp3, temp2 & 0xff);
  MultiSU16X8toH16(temp3, temp1, 0xff - (temp2 & 0xff));
  // put delayed data at right output
  right_out = temp3 + temp4;
  // mix delayed and current data at left output
  left_out = right_out + left_in;

  // & is required before adc variables
  AudioCodec_ADC(&mod0_value, &mod1_value);
  
  reti(); // dont forget to return from the interrupt
}
