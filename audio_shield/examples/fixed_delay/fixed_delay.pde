/*
fixed_delay.pde
guest openmusiclabs 6.20.11
this function delays both the left and right channels by a
fixed amount.  the limit to the length of the delay is set
by the internal SRAM of the chip you are using.  for the
arduino uno (atmega328), this is 2kbytes.  each audio data
sample is 16bits, or 2bytes, so at most you can get a 1024
sample delay buffer.  but, since this is in stereo, that is
limited to 512 for each, left and right.  this is further
limited by the fact that the program itself needs to use
some of that space, so feel free to play around with the SIZE
constant.  the compiler will complain if its too big.  "SIZE
800" gives a delay of 400 samples per channel, which is a
400/44.1kHz = 9ms delay time. 
*/


// setup codec parameters
// must be done before #includes
// see readme file in libraries folder for explanations
#define SAMPLE_RATE 44 // 44.1Khz
#define ADCS 0 // no ADCs are being used

// include necessary libraries
#include <Wire.h>
#include <SPI.h>
#include <AudioCodec.h>

// create data variables for audio transfer
int left_in = 0x0000;
int left_out = 0x0000;
int right_in = 0x0000;
int right_out = 0x0000;

// create a delay buffer in memory
#define SIZE 800 // buffer size is limited by microcontroller SRAM size
int delaymem[SIZE]; // 800 positions x 2 bytes = 1600 bytes of SRAM
unsigned int location = 0; // buffer location to read/write from

void setup() {
  AudioCodec_init(); // setup codec registers
  // call this last if setting up other parts
}

void loop() {
  while (1); // reduces clock jitter
}

// timer1 interrupt routine - all data processed here
ISR(TIMER1_COMPA_vect, ISR_NAKED) { // dont store any registers

  // &'s are necessary on data_in variables
  AudioCodec_data(&left_in, &right_in, left_out, right_out);
  
  //fetch data from buffer and put it into output register
  left_out = delaymem[location];
  // put new data in same location for maximal delay time
  delaymem[location++] = left_in; // post increment location to go to next memory location
  // repeat for right channel
  right_out = delaymem[location];
  delaymem[location++] = right_in; // post increment in preperation for next transfer
  // check if location has gotten bigger than buffer size
  if (location >= SIZE) {
    location = 0; // reset location
  }
  
  // dont forget to return from interrupt
  reti();
}
