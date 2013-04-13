#ifdef __cplusplus
extern "C" {
#endif

//********************************************************************
//
//				General Function Definitions
//
//********************************************************************

// initiate the LCD, assuming the SPI bus is set for 9 bit LCD transfers
void spi_LCD_init(void);

// send command using 9 bit transfers on SPI bus.
void spi_LCD_command(uint8_t data);

// send data using 9 bit transfers on SPI bus.
void spi_LCD_data(uint8_t data);


// initiate the SPI bus, using the 8 bit general case and normal CS line
void spi_MasterInit(void);

// write & read the SPI bus, using 8 bit transfers
uint8_t spi_WriteRead(uint8_t dataout);


//*	Arduino Duemilanove SPI defs

#define SPI_DDR         DDRB
#define SPI_PORT        PORTB

#define LCD_RESET       PB0
#define LCD_SPI_SS      PB1

#define SPI_SS          PB2
#define SPI_MOSI        PB3
#define SPI_MISO        PB4
#define SPI_SCK         PB5

//*******************************************************
//						Macros
//*******************************************************
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))


#ifdef __cplusplus
}
#endif
