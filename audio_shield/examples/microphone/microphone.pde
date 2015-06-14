/*
microphone.pde
guest openmusiclabs 8.17.11
this program takes input from the MIC pins on the codecshield.
NOTE: you will have to do a few solder joints to connect
your microphone, a description of this is on the wiki:
http://wiki.openmusiclabs.com/wiki/CodecShieldMicMod
this can also be used with low level instruments, like guitars,
to boost the volume on the input.
*/


// setup codec parameters
// must be done before #includes
// see readme file in libraries folder for explanations
#define SAMPLE_RATE 44 // 44.1Khz
#define ADCS 0 // no ADCs are being used
#define MUTEMIC 0 // turn off the mute on the microphone
#define INSEL 1 // select the microphone input
#define MICBOOST 1 // enables the microphone +10dB amp
                   // set this to 0 if its too loud  


// include necessary libraries
#include <Wire.h>
#include <SPI.h>
#include <AudioCodec.h>

// create data variables for audio transfer
int left_in = 0x0000;
int left_out = 0x0000;
int right_in = 0x0000;
int right_out = 0x0000;


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
  
  // pass data through
  left_out = left_in;
  right_out = right_in;
  
  // dont forget to return from interrupt
  reti();
}
