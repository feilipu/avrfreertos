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


//*	Arduino Duemilanove SPI 9 bit defs

#define SPI9_DDR         DDRB
#define SPI9_PORT        PORTB

#define LCD_RESET        PB0
#define LCD_SPI_SS       PB1

#define SPI9_SS          PB2
#define SPI9_MOSI        PB3
#define SPI9_MISO        PB4
#define SPI9_SCK         PB5

//*******************************************************
//						Macros
//*******************************************************
#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))


#ifdef __cplusplus
}
#endif
