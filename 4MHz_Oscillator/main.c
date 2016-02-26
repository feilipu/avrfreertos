////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////    main.c
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <util/delay.h>



/*--------------Functions---------------------------*/

/* Main program loop */
int main(void) __attribute__((OS_main));

int main(void)
{

  DDRD = 0xFF;
  while (1)
  {
	 PIND = 0xFF;
   }
}

