
#include <avr/io.h>

#include "SPI9master.h"


//************************************************************************
//Usage: spi_LCD_init(); initiate 9 bit transfers on SPI bus
//Inputs: None
//Outputs: None
//************************************************************************
void inline spi_LCD_init(void)
{
	SPI_DDR  |= ( ( uint8_t ) ((1<<LCD_RESET)|(1<<LCD_SPI_SS)|(1<<SPI_SCK)|(1<<SPI_SS)|(1<<SPI_MOSI)));
    SPI_PORT |= ( ( uint8_t ) ((1<<LCD_RESET)|(1<<LCD_SPI_SS)) );
}


//************************************************************************
//Usage: spi_LCD_command(RAMWR);
//Inputs: char data - character command to be sent as SPI Master (9 bit total)
//Outputs: None
//************************************************************************
void spi_LCD_command(uint8_t data)
{

	cbi(SPI_PORT, LCD_SPI_SS);		// enable LCD chip, by setting low
	cbi(SPI_PORT, SPI_MOSI);		// output low on data out (9th bit low = command)
	
	cbi(SPI_PORT, SPI_SCK);		    // send clock pulse
	asm volatile ("nop");
	sbi(SPI_PORT, SPI_SCK);
	
	sbi(SPI_PORT, SPI_MOSI);		// output high on data out 
	
	asm volatile ("nop");

    SPCR |= (( uint8_t ) (1<<SPE)|(1<<MSTR));	// Enable Hardware SPI,
    SPSR |= (( uint8_t ) (1<<SPI2X));         // set clock rate fck/2
    
    SPDR = data; 						// send data
    
    while(!(SPSR & (1<<SPIF)));		    // wait until send complete
    
	sbi(SPI_PORT, LCD_SPI_SS);			    // disable LCD device CS


                                        // Disable Hardware SPI, this releases the SPI pins
										// for general IO use. which is used to send the 1'st 
										// bit out
    SPCR &= ~(( uint8_t ) (1<<SPE)|(1<<MSTR));
    SPSR &= ~(( uint8_t ) (1<<SPI2X));

    sbi(SPI_PORT, LCD_SPI_SS);			// disable LCD chip
}

//************************************************************************
//Usage: spi_LCD_data(RAMWR);
//Inputs: char data - character data to be sent to SPI bus as Master (9 bit total)
//Outputs: None
//************************************************************************
void spi_LCD_data(uint8_t data)
{

	cbi(SPI_PORT, LCD_SPI_SS);			// enable chip by setting low
	sbi(SPI_PORT, SPI_MOSI);			// output high on data out (9th bit high = data)
	
	cbi(SPI_PORT, SPI_SCK);		        // send clock pulse
    asm volatile ("nop");
	sbi(SPI_PORT, SPI_SCK);
	
	asm volatile ("nop");

    SPCR |= (( uint8_t ) (1<<SPE)|(1<<MSTR));	// Enable Hardware SPI,
    SPSR |= (( uint8_t ) (1<<SPI2X));         // set clock rate fck/2
    
    SPDR = data; 						// send data
    
    while(!(SPSR & (1<<SPIF)));		    // wait until send complete
    
	sbi(SPI_PORT, LCD_SPI_SS);			    // disable LCD device CS


                                        // Disable Hardware SPI, this releases the SPI pins
										// for general IO use. which is used to send the 1'st 
										// bit out
    SPCR &= ~(( uint8_t ) (1<<SPE)|(1<<MSTR));
    SPSR &= ~(( uint8_t ) (1<<SPI2X));

    sbi(SPI_PORT, LCD_SPI_SS);			// disable LCD chip
}



/*****************************************************************************
//Usage: spi_MasterInit(void); Another way to do this using 8 bit transfers
//Inputs: None
//Outputs: None
*****************************************************************************/

void spi_MasterInit(void)
{
    /* Set MOSI, SCK & SS output, all others input */
    SPI_DDR = ( uint8_t ) (1<<SPI_MOSI)|(1<<SPI_SCK)|(1<<SPI_SS);

//  Make sure the SPI SS is disabling the chip
    sbi(SPI_PORT, SPI_SS);


//  Enable SPI, Master, set clock rate fck/2 (maximum)
    SPCR = ( uint8_t ) (1<<SPE)|(1<<MSTR);
    SPSR = ( uint8_t ) (1<<SPI2X);

//   OR
    
//    Enable SPI, Master, set clock rate fck/4 */
//    SPCR = ( uint8_t ) (1<<SPE)|(1<<MSTR);
    
//  OR

//    Enable SPI, Master, set clock rate fck/16
//    SPCR = ( uint8_t ) (1<<SPE)|(1<<MSTR)|(1<<SPR0);

//  OR

//    Enable SPI, Master, set clock rate fck/32
//    SPCR = ( uint8_t ) (1<<SPE)|(1<<MSTR)|(1<<SPR1);
//    SPSR = ( uint8_t ) (1<<SPI2X);

//  OR

//    Enable SPI, Master, set clock rate fck/128 (minimum)
//    SPCR &= ~(( uint8_t ) (1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0));
}



uint8_t inline spi_WriteRead(uint8_t dataout)
{
	uint8_t datain;
    
    // enable chip, goes low
	cbi(SPI_PORT, SPI_SS);			
	
    // Start transmission (MOSI)
    SPDR = dataout;

    // Wait for transmission complete
    while(!(SPSR & (1<<SPIF)));

    // Get return Value;
    datain = SPDR;

	asm volatile ("nop"); // Hold pulse for 1 micro second      
           
    // Disable Latch & disable chip
    sbi(SPI_PORT, SPI_SS);

    // Return Serial In Value (MISO)
    return datain;
}


