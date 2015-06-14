/*
variable_delay.pde
guest openmusiclabs 6.22.11
this program takes input from the left channel, and delays it
by a variable amount, set by the MOD0 knob.  it then presents
this delayed data on the left channel, with the non-delayed
data coming out of the right channel.
*/

// setup codec parameters
// must be done before #includes
// see readme file in libraries folder for explanations
#define SAMPLE_RATE 44 // 44.1kHz sample rate
#define ADCS 1 // only 1 ADC used here -> MOD0

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

// create variable for ADC result
// it only has positive values -> unsigned
unsigned int mod0_value =0;

// create a delay buffer in memory
#define SIZE 800 // buffer size is limited by microcontroller SRAM size
int delaymem[SIZE]; // 800 positions x 2 bytes = 1600 bytes of SRAM
unsigned int location = 0; // buffer location to read/write from
unsigned int boundary = 0; // end of buffer position

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
  
  // pass left input to right output
  right_out = left_in;

  //fetch data from buffer and put it into output register
  left_out = delaymem[location];
  // put new data in same location for maximal delay time
  delaymem[location++] = left_in; // post increment location to go to next memory location
  // check if location has gotten bigger than buffer size
  if (location >= boundary) {
    location = 0; // reset location
  }

  // & is required before adc variable
  AudioCodec_ADC(&mod0_value);

  // scale ADC value to match buffer size
  // note the use of the fancy fast math function
  // you can read about these in the readme file
  unsigned int scale = SIZE;
  MultiU16X16toH16(boundary, mod0_value, scale);

  reti(); // dont forget to return from the interrupt
}
