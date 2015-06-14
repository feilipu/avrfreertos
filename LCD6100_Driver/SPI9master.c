
#include <avr/io.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "spi.h"

#include "SPI9master.h"


/* Declare a binary Semaphore flag for the SPI Bus. To ensure only single access to SPI Bus. */
extern SemaphoreHandle_t xSPISemaphore;

//************************************************************************
//Usage: spi_LCD_init(); initiate 9 bit transfers on SPI bus
//Inputs: None
//Outputs: None
//************************************************************************

void inline spi_LCD_init(void)
{
	SPI9_DDR  |= ( ( uint8_t ) ((1<<LCD_RESET)|(1<<LCD_SPI_SS)|(1<<SPI9_SCK)|(1<<SPI9_SS)|(1<<SPI9_MOSI)));
    SPI9_PORT |= ( ( uint8_t ) ((1<<LCD_RESET)|(1<<LCD_SPI_SS)) );

    if( xSPISemaphore == NULL ) 					/* Check to see if the semaphore has not been created. */
    {
    	xSPISemaphore = xSemaphoreCreateBinary(); 	/* Then create the SPI bus binary semaphore */
		if( ( xSPISemaphore ) != NULL )
			xSemaphoreGive( ( xSPISemaphore ) );	/* make the SPI bus available */
    }
}


//************************************************************************
//Usage: spi_LCD_command(RAMWR);
//Inputs: char data - character command to be sent as SPI Master (9 bit total)
//Outputs: None
//************************************************************************
void spi_LCD_command(uint8_t data)
{

	cbi(SPI9_PORT, LCD_SPI_SS);		// enable LCD chip, by setting low
	cbi(SPI9_PORT, SPI9_MOSI);		// output low on data out (9th bit low = command)

	cbi(SPI9_PORT, SPI9_SCK);		    // send clock pulse
	asm volatile ("nop");
	sbi(SPI9_PORT, SPI9_SCK);

	sbi(SPI9_PORT, SPI9_MOSI);		// output high on data out

	asm volatile ("nop");

    SPCR |= (( uint8_t ) (1<<SPE)|(1<<MSTR));	// Enable Hardware SPI,
    SPSR |= (( uint8_t ) (1<<SPI2X));         // set clock rate fck/2

    SPDR = data; 						// send data

    while(!(SPSR & (1<<SPIF)));		    // wait until send complete

	sbi(SPI9_PORT, LCD_SPI_SS);			    // disable LCD device CS


                                        // Disable Hardware SPI, this releases the SPI pins
										// for general IO use. which is used to send the 1'st
										// bit out
    SPCR &= ~(( uint8_t ) (1<<SPE)|(1<<MSTR));
    SPSR &= ~(( uint8_t ) (1<<SPI2X));

    sbi(SPI9_PORT, LCD_SPI_SS);			// disable LCD chip
}

//************************************************************************
//Usage: spi_LCD_data(RAMWR);
//Inputs: char data - character data to be sent to SPI bus as Master (9 bit total)
//Outputs: None
//************************************************************************
void spi_LCD_data(uint8_t data)
{

	cbi(SPI9_PORT, LCD_SPI_SS);			// enable chip by setting low
	sbi(SPI9_PORT, SPI9_MOSI);			// output high on data out (9th bit high = data)

	cbi(SPI9_PORT, SPI9_SCK);		        // send clock pulse
    asm volatile ("nop");
	sbi(SPI9_PORT, SPI9_SCK);

	asm volatile ("nop");

    SPCR |= (( uint8_t ) (1<<SPE)|(1<<MSTR));	// Enable Hardware SPI,
    SPSR |= (( uint8_t ) (1<<SPI2X));         // set clock rate fck/2

    SPDR = data; 						// send data

    while(!(SPSR & (1<<SPIF)));		    // wait until send complete

	sbi(SPI9_PORT, LCD_SPI_SS);			    // disable LCD device CS


                                        // Disable Hardware SPI, this releases the SPI pins
										// for general IO use. which is used to send the 1'st
										// bit out
    SPCR &= ~(( uint8_t ) (1<<SPE)|(1<<MSTR));
    SPSR &= ~(( uint8_t ) (1<<SPI2X));

    sbi(SPI9_PORT, LCD_SPI_SS);			// disable LCD chip
}



