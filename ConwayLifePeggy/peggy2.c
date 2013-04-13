/*

Title:			peggy2.c 
Date Created:   3/23/08
Last Modified:  4/28/08
Target:			Atmel ATmega328p 
Environment:	AVR-GCC
Purpose:		Drive 25x25 LED array
Application:	Conway's Life

Copyright 2008 Windell H. Oskay
Distributed under the terms of the GNU General Public License, please see below. 
Adapted from a program by David Gustafik, and re-released under the GPL. :)

We'd really like to see improved versions of this software!

 
Portions Copyright David Gustafik, 2007

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include <stdint.h>
#include <stdlib.h>

#include <avr/io.h> 
#include <avr/eeprom.h>


#define CELLS_X 25		
#define CELLS_Y 25 


/////////////////////////////////////////////////////////////////////////

uint32_t current_generation[CELLS_X];
uint32_t old_generation[CELLS_X];

volatile uint32_t d[25];

uint8_t InputReg,InputRegOld;

uint8_t EditMode = 0;

uint16_t refreshNum = 10;

int8_t xCursor = 12;
int8_t yCursor = 12;
  
 
/////////////////////////////////////////////////////////////////////////

inline uint8_t get_cell(uint32_t from[],  int8_t x, int8_t y)
{
	if(x < 0) 
	   x = 24;
	   
	if(x > 24) 
	   x = 0;
	
	if(y < 0) 
	   y = 24;
	   
	if(y > 24) 
	   y = 0;
	 
	return ((from[x] & ( (uint32_t) 1 << y)) > 0);
}

/////////////////////////////////////////////////////////////////////////

inline uint16_t get_total(uint32_t from[])
{
	uint16_t total = 0;

	for(uint8_t x=0; x < CELLS_X; x++)
	{
		for(uint8_t y=0; y < CELLS_Y; y++)
		{
			if(get_cell(from,x,y)) total++;
		}
	}
	return total;
}
 
 /////////////////////////////////////////////////////////////////////////

 
inline void set_cell(uint32_t to[], int8_t x, int8_t y, uint8_t value)
{
	if(value) 
		to[x] |= (uint32_t) 1 <<  y;
	else 
		to[x] &= ~( (uint32_t) 1 << y);
	return;
}

/////////////////////////////////////////////////////////////////////////


inline void fill_cell(uint32_t to[], int8_t x, int8_t y)
{ 
	to[x] |= (uint32_t) 1 <<  y;
	return;
}

/////////////////////////////////////////////////////////////////////////


inline void clear_cell(uint32_t to[], int8_t x, int8_t y)
{
	to[x] &= ~( (uint32_t) 1 << y);
	return;
}

/////////////////////////////////////////////////////////////////////////


inline void fill_random_data(uint32_t to[])
{

	for (uint8_t temp = 0; temp < CELLS_X; temp++)
	{
	//	   to[temp] = rand() & 0xFF;

		   to[temp] = rand();
		   to[temp] <<= 15;
		   to[temp] |= rand();
	}
	return;
}

/////////////////////////////////////////////////////////////////////////


inline void clear_data(uint32_t to[])
{
	for (uint8_t temp = 0; temp < CELLS_X; temp++)
	   to[temp] = 0; 
	return;
}




/////////////////////////////////////////////////////////////////////////



inline void copy_old_new(uint32_t old_gen[], uint32_t new_gen[])
{
	for(uint8_t temp = 0; temp < CELLS_X; temp++)
		old_generation[temp] = current_generation[temp];
	return;
}

/////////////////////////////////////////////////////////////////////////



static uint8_t get_neighbours(uint32_t from[], int8_t x, int8_t y)
{
	uint8_t out = 0;
	
	if(get_cell(from,x-1,y-1))
		out++;
	
	if(get_cell(from,x-1,y))
		out++;
	
	if(get_cell(from,x-1,y+1))
		out++;

	if(get_cell(from,x,y-1))
		out++;
	
	if(get_cell(from,x,y+1))
		out++;

	if(get_cell(from,x+1,y-1))
		out++;
	
	if(get_cell(from,x+1,y))
		out++;
	
	if(get_cell(from,x+1,y+1))
		out++;

	return out;
}

/////////////////////////////////////////////////////////////////////////



static uint8_t get_difference(uint32_t a[],uint32_t b[])
{
	uint8_t diff=0;

	for(uint8_t  x=0; x < CELLS_X; x++)
	{
		for(uint8_t  y=0; y < CELLS_Y; y++)
		{
			if((get_cell(a,x,y) && !get_cell(b,x,y)) || (!get_cell(a,x,y) && get_cell(b,x,y)))
			    diff++;
		}
	}
	return diff;
}

/////////////////////////////////////////////////////////////////////////



inline void display(uint32_t from[])
{

	uint32_t longtemp;

	for(uint8_t x=0; x < CELLS_X; x++)
		{ 
	
		longtemp = 0;
		
		
		for(uint8_t y=0; y < CELLS_Y; y++)
			{  
		
				if(get_cell(from,x,y))   
					longtemp |= (uint32_t) 1 << y;
			 }
				 
		//		longtemp |=  (uint32_t) 1 << 8;   // Optional grid lines
		//		longtemp |=  (uint32_t) 1 << 16;  // Optional grid lines
		//		longtemp |=  (uint32_t) 1 << 24;  // Optional grid lines
				 
		d[x] = longtemp;
						
	}
		
	return;
}


/////////////////////////////////////////////////////////////////////////

inline void SPI_TX (uint8_t cData)
{
	//Start Transmission
	SPDR = cData;
	//Wait for transmission complete:
	while (!(SPSR & _BV(SPIF))) ;
	
}


inline void DisplayLEDs()
{

	uint8_t  j,k;
	uint8_t  out1,out2,out3,out4;
	uint32_t dtemp;
	
	k = 0;
	
	while (k < refreshNum)		// k must be at least 1

	{

		k++;

		j = 0;

		while (j < 25) 
		{
	
			if (j == 0)
			  PORTD = 160;
			else if (j < 16)
			  PORTD = j;
			else
			  PORTD = (j - 15) << 4;  
			  
			dtemp = d[j]; 
	
			out4 = dtemp & 255U;
			dtemp >>= 8;
			out3 = dtemp & 255U;
			dtemp >>= 8;
			out2 = dtemp & 255U;	 
			dtemp >>= 8;
			out1 = dtemp & 255U; 	
		
			SPI_TX(out1);
			SPI_TX(out2);
			SPI_TX(out3);
			SPI_TX(out4);	
					
			PORTD = 0;  // Turn displays off	
					
			PORTB |= _BV(1);		//Latch Pulse 
			PORTB &= ~( _BV(1));
			 
			j++;
				 
		}
	}
}



inline void delayLong(uint16_t delayLocal)
{
	uint16_t delayvar = 0;
	
	while (delayvar <=  delayLocal)		
		{ 
			asm("nop");  
			delayvar++;
		} 
}





int main(void)
{
	uint16_t generations=0;
	
	uint8_t out1,out2,out3,out4;
	uint32_t dtemp;  
	uint16_t brightness = 0;

	srand(eeprom_read_word((uint16_t *) 2));
	for(uint8_t temp = 0; temp != 255; temp++)
	{
		TCNT0 = rand();
	}
	eeprom_write_word((uint16_t *) 2,rand());
	  
PORTD = 0U;			//All B Input
DDRD = 255U;		// All D Output
	
PORTB = 1;		// Pull up on ("OFF/SELECT" button)
PORTC = 255U;	// Pull-ups on C

DDRB = 254U;  // B0 is an input ("OFF/SELECT" button)
DDRC = 0;		//All inputs

// turn OFF serial RX/TX, necessary if using arduino bootloader
UCSR0B =0;

////SET MOSI, SCK Output, all other SPI as input:
DDRB = ( 1 << 3 ) | (1 << 5) | (1 << 2) | (1 << 1);

//ENABLE SPI, MASTER, CLOCK RATE fck/4:		//TEMPORARY:: 1/128
SPCR = (1 << SPE) | ( 1 << MSTR );//| ( 1 << SPR0 );//| ( 1 << SPR1 ) | ( 1 << CPOL );

SPSR = (1 << SPI2X); // make fck/2 added by phillip

SPI_TX(0);
SPI_TX(0);
SPI_TX(0);
SPI_TX(0);

PORTB |= _BV(1);		//Latch Pulse 
PORTB &= ~( _BV(1));
 
fill_random_data(old_generation);
 
InputRegOld = (PINC & 31) | ((PINB & 1)<<5);  

 


	while(1)    //Main Loop
	{
	
	uint8_t temp=0;
	// Some routines follow to do things if the optional buttons are installed-- a simple editor is implemented.
	 
	InputReg = (PINC & 31) | ((PINB & 1)<<5);    // Input reg measures OFF ( bit 5) and b1-b5 (bits 0-4). 
	InputRegOld ^= InputReg;    // InputRegOld is now nonzero if there has been a change.
	
	if (InputRegOld) {
	
	InputRegOld &= InputReg;  // InputRegOld is now nonzero if the change was to the button-released (not-pulled-down) state.
								// i.e., the bit that was high after the XOR corresponds to a bit that is presently high.
								// The checks that follow will handle MULTIPLE buttons being pressed and unpressed at the same time.
	if (InputRegOld & 1)   //b1 "ANY" button is pressed
		{
		
		if (EditMode){
				
				temp = get_cell(old_generation,yCursor,xCursor);
				
				
			set_cell(current_generation, yCursor, xCursor, !temp ); // Invert Cell
		
		}
		}  
	if (InputRegOld & 2)   //b2 "left" button is pressed
		{
		
			if (EditMode)	{
				xCursor--;
				if(xCursor < 0) 
					xCursor = 24;
			}
		}  
	if (InputRegOld & 4)   //b3 "down" button is pressed	
		{
		
			if (EditMode)
				{ 					
					yCursor++;	// move cursor down
					if(yCursor > 24) 
						yCursor = 0;
				}
			else if (refreshNum < 25)		// if NOT in edit mode
				refreshNum++;

		}  			
	if (InputRegOld & 8)   //b4 "up" button is pressed		
		{

		if (EditMode)
		{
			yCursor--;	// move cursor up
			if(yCursor < 0) 
				yCursor = 24;
		}
		else if (refreshNum > 1)
			refreshNum--;

		}  
	if (InputRegOld & 16)	 //b5 "right" button is pressed
		{
		
			if (EditMode)	
			{
				xCursor++;
				if(xCursor > 24) 
					xCursor = 0;
			} 
		
		}
	if (InputRegOld & 32)	// s2 "Off/Select" button is pressed... 
		{
		EditMode = !EditMode;		//Toggle in/out of edit mode.
		if (EditMode)
			{
			generations = 0;								// Postpone mutations for a while.
			
			//	clear_data(current_generation);				//Clear screen when entering edit mode
															// Quite optional to enable this!
			}	
							
		}  
		
	}

	InputRegOld = InputReg;
		
	if ((InputReg & 1U) == 0) 
		if(EditMode == 0)
			fill_random_data(old_generation);	// Randomize on "Any" key only if we are NOT in edit mode.

	display(old_generation);

	DisplayLEDs();
		
	if (EditMode)
	{		// Draw an extra dot as the cursor -- Mini display mode.
			// By doing it this way, we're independent of the display matrices.

			dtemp = (uint32_t) 1 << xCursor; 
	
			out4 = dtemp & 255U;
			dtemp >>= 8;
			out3 = dtemp & 255U;
			dtemp >>= 8;
			out2 = dtemp & 255U;	 
			dtemp >>= 8;
			out1 = dtemp & 255U; 	
		
		SPI_TX(out1);
		SPI_TX(out2);
		SPI_TX(out3);
		SPI_TX(out4);


		PORTB |= _BV(1);		//Latch Pulse 
		PORTB &= ~( _BV(1));
		 
			if (yCursor < 15)
			  PORTD =  yCursor+1;
			else
			  PORTD = (yCursor - 14) << 4; 
			  
			  delayLong(brightness);

		PORTD = 0;  // Turn displays off

		SPI_TX(0);
		SPI_TX(0);
		SPI_TX(0);
		SPI_TX(0);

		PORTB |= _BV(1);		//Latch Pulse 
		PORTB &= ~( _BV(1));

		brightness += 50;
		if (brightness > 500)
		brightness = 0;

		DisplayLEDs();

	}
	else
	{		
		
			for(uint8_t x=0; x < CELLS_X; x++)
			{ 
	
				for(uint8_t y=0; y < CELLS_Y; y++)		// Inner loop; should be made quick as possible.
				{  
					temp = get_neighbours(old_generation, x, y);
				
					if(temp < 2) 
						clear_cell(current_generation, x, y);
					
					if(temp == 3) 
						fill_cell(current_generation, x, y);
					
					if(temp > 3) 
						clear_cell(current_generation, x, y);	  
	
				}
	
	
				DisplayLEDs();
			
			}
			
			DisplayLEDs();
		
	/*		if( generations++ == 200 )
			{
				current_generation[0] = rand() & 0xFF;
				current_generation[23] = rand() & 0xFF;
				current_generation[24] = rand() & 0xFF; 
				generations = 0;
			}
		
	*/		
		
			//Alternative boringness detector:
			if( generations++ == 512 || get_difference(current_generation,old_generation) < 8 || get_total(current_generation) < 6)
			{
				current_generation[7] = rand() & 0xFF;
				current_generation[8] = rand() & 0xFF;
				current_generation[9] = rand() & 0xFF; 
				generations = 0;
			}

		
	}

		copy_old_new(old_generation, current_generation); 
		DisplayLEDs ();
		 
	}
	return 0;
}

