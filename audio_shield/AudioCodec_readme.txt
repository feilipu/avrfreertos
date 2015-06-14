AudioCodec_readme.txt
guest openmusiclabs 7.7.11


this file describes the various functions used in the AudioCodec library.

FUNCTIONS
---------

AudioCodec_data(&left_in, &right_in, left_out, right_out) - this transfers data to and from the codec.  it must take 4 arguments, even if you are only dealing with mono data, and the first two arguments (the inputs) must have & before them.  the data is transfered as signed 16bit integers.  the total transfer time is about 4us, although a lot of this time is spent idling.  if you really need to save some time, you can copy this function directly into your sketch, and put some other functions in the while() loops.

AudioCodec_ADC(&mod0_value, &mod1_value) - this fetches the ADC values from the MOD pots.  it can take either one or two arguments, but they must have the & before them.  the ADC is only a 10bit ADC, but the values are oversampled 64 times to give a 16bit unsigned integer.  the values are further windowed by a hyterisis value HYST to eliminate random noise.  this is particularly useful when the ADC value is setting a delay time, although for some some functions, like setting of VCO frequencies, it can create a nice vibrato effect.  you can turn off the hysterisis effect by setting HYST 0 (see definitions below).


MULTIPLIES
----------

The processing time is extremely short on the Arduino, so special multiply functions are included in the AudioCodec library to speed things up.  These operate up to 4 times faster than their C counterparts.  not all multiply functions have been implemented, but a lot of the useful ones are:

MultiU16X8toH16
MultiSU16X8toL16
MultiSU16XConst8toL16
MultiSU16XConst8toH16
MultiSU16X8toH16
MultiSU16X8toH16Round
MultiU16X16toH16
MultiU16X16to32
MultiU16X16toH16Round
MultiS16X16to32
MultiS16X16toH16
MultiSU16X16to32
MultiSU16X16toH16
MultiSU16X16toH16Round
MultiSU32X16toH32Round


the function name describes what the multiply operation does.  for example, MultiSU16X8toL16(result, signed int, unsigned char) multiplies a signed 16b number by an unsigned 8bit number to give the low 16bits of the result.  the first argument is always the result, the second is the left hand part of the multiply, and the third is the right hand part of the multiply.  the results can either by the low part, the high part, or the rounded up version of the high part of the result.  take a look at some of the code examples for more demonstrations of these.


CODEC SETTINGS
--------------

the codec can be set to various bit rates and volumes, and these are all settable with the #define statement at the beginning of your sketch.  these must be placed before the #include statements, and not have an equals sign or semicolon afterwards.  for example:

#define SAMPLE_RATE 44

all of these settings take only whole number values, and have a default setting if no value is given to them.  the following is a listing of currently implemented settings you can vary.  the range of possible values is given in brackets [], with the default value in parens ().  if the setting can take a range of values, the range will be seperated with an arrow ->.

SAMPLE_RATE [2, 8, 22, (44), 88]
- this sets the codec sample rate in approximate kHz.
88 = 88.2kHz (Fck/128)
44 = 44.1kHz (Fck/256)
22 = 22.05kHz (Fck/256*2)
8 = 8.018kHz (Fck/256*5.5)
2 = 2.45kHz (Fck/256*18)

ADCS [0, 1, (2)]
- this sets the number of ADCs to be sampled by the micro controller.  you should always set this to the minimum number of ADCs you intend to use, as sampling the ADCs takes up precious processing time.

DECIMATE [1, (2), 3]
- this sets the number of ADC samples for decimation accuracy.  4 ^ DECIMATE samples are taken before reporting ADC value. See AVR8003.doc

LINVOL [0 -> 31, (23)]
- this sets the left line-in volume, in 1.5dB steps from -34.5dB to +12dB. the default setting is 0dB.

RINVOL [0 -> 31, (23)]
- this sets the right line-in volume, same as above.

LHPVOL [0, 48 -> 127, (121)]
- this sets the left headphone volume output in 1dB steps from -73dB to +6dB.  the default setting is 0dB. LHPVOL 0 mutes the output, as does any value from 0 -> 47, but for simplicity's sake, mute is given as 0 with the other values not defined.

RHPVOL [0, 48 -> 127, (121)]
- this sets the right headphone volume output, same as above.