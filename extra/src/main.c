/**
* \file
*
* \brief Empty user application template
*
*/

/**
* \mainpage User Application template doxygen documentation
*
* \par Empty user application template
*
* Bare minimum empty user application template
*
* \par Content
*
* -# Include the ASF header files (through asf.h)
* -# Minimal main function that starts with a call to board_init()
* -# "Insert application code here" comment
*
*/

/*
* Include header files for all drivers that have been imported from
* Atmel Software Framework (ASF).
*/


/*###################################### 
The Display library is ported from Adafruits ST7735 driver. Some functions from the Adafruit GFX library where added to bring it alive.
The code for the FFT is inspired by Matthias Busse, but it's mostly based on the Wikipedia.com article.
#####################################'*/


#include <asf.h>
#include <avr32/io.h>
#include <Adafruit_ST7735.h>
#include <math.h>


uint16_t _width = 128;
uint16_t _height = 160;

float PI = 3.141592653589;

#define N_WAVE	256    // volle Länge der Sinewave[] möglich 16, 64, 256, 1024 ...
#define LOG2_N_WAVE 8  // log2(N_WAVE)
#define F_SAMPLE 5120  // 5120/128 = 40Hz per FFT "bin"
#define AMP_CARRIER 100 // amplitude of each "carrier" wave

int im[N_WAVE/2];
int re[N_WAVE/2];
Bool old[64][64];
int Sinewave[N_WAVE/2] = { // 128 Werte, die halbe positive Sinuswelle
	0, 3, 6, 9, 12, 15, 18, 21,
	24, 28, 31, 34, 37, 40, 43, 46,
	48, 51, 54, 57, 60, 63, 65, 68,
	71, 73, 76, 78, 81, 83, 85, 88,
	90, 92, 94, 96, 98, 100, 102, 104,
	106, 108, 109, 111, 112, 114, 115, 117,
	118, 119, 120, 121, 122, 123, 124, 124,
	125, 126, 126, 127, 127, 127, 127, 127,

	127, 127, 127, 127, 127, 127, 126, 126,
	125, 124, 124, 123, 122, 121, 120, 119,
	118, 117, 115, 114, 112, 111, 109, 108,
	106, 104, 102, 100, 98, 96, 94, 92,
	90, 88, 85, 83, 81, 78, 76, 73,
	71, 68, 65, 63, 60, 57, 54, 51,
	48, 46, 43, 40, 37, 34, 31, 28,
	24, 21, 18, 15, 12, 9, 6, 3,
};

double freq, phase;    // Frequenz, Phase

#define LED0 AVR32_PIN_PB00
#define RST AVR32_PIN_PB09
#define DC AVR32_PIN_PB08


#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

#define interrupt __attribute__((__interrupt__))
#define TC_CHANNEL   0
#define FALSE 0


int led_count;

int16_t ad_value;

uint16_t counter;
uint32_t tempVolume;
uint32_t volume;

#define SPI_EXAMPLE             (&AVR32_SPI)
#define SPI_SLAVECHIP_NUMBER    (0)

spi_options_t my_spi_options={
	// The SPI channel to set up : Memory is connected to CS1
	SPI_SLAVECHIP_NUMBER,
	// Preferred baudrate for the SPI.
	60000000,
	// Number of bits in each character (8 to 16).
	8,
	// Delay before first clock pulse after selecting slave (in PBA clock periods).
	0,
	// Delay between each transfer/character (in PBA clock periods).
	0,
	// Sets this chip to stay active after last transfer to it.
	1,
	// Which SPI mode to use when transmitting.
	SPI_MODE_0,
	// Disables the mode fault detection.
	// With this bit cleared, the SPI master mode will disable itself if another
	// master tries to address it.
	1
};

// GPIO pins used for SD/MMC interface
static const gpio_map_t SD_MMC_SPI_GPIO_MAP =
{
	{AVR32_SPI_SCK_0_0_PIN,  AVR32_SPI_SCK_0_0_FUNCTION },  // SPI Clock.
	{AVR32_SPI_MISO_0_0_PIN, AVR32_SPI_MISO_0_0_FUNCTION},  // MISO.
	{AVR32_SPI_MOSI_0_0_PIN, AVR32_SPI_MOSI_0_0_FUNCTION},  // MOSI.
	{AVR32_SPI_NPCS_0_0_PIN, AVR32_SPI_NPCS_0_0_FUNCTION}   // Chip Select NPCS.
};

void delay( uint16_t delay ) {
	uint32_t i = 2500 * delay; //2500
	while( i-- ) {
		asm( "nop" ); //no operation
	}
}

void spi_init_module(void)
{
	gpio_enable_module(SD_MMC_SPI_GPIO_MAP,
	sizeof(SD_MMC_SPI_GPIO_MAP) / sizeof(SD_MMC_SPI_GPIO_MAP[0]));
	//Init SPI module as master
	spi_initMaster(SPI_EXAMPLE,&my_spi_options);
	//Setup configuration for chip connected to CS1
	spi_setupChipReg(SPI_EXAMPLE,&my_spi_options,sysclk_get_pba_hz());
	//Allow the module to transfer data
	spi_enable(SPI_EXAMPLE);
}

/*
* GPIO aktivieren
*/
void enable_led( void ) {
	for( int i = 0; i < 8; ++i )
	{
		gpio_enable_gpio_pin( LED0 + i );	// Port Enable
	}
	gpio_enable_gpio_pin(RST);
	gpio_enable_gpio_pin(DC);
}

void ledBarGraph ( uint8_t leds) { // 0 - 8; 0 = all led's off
	if (leds > 7) {
		leds = 8;
	}
	for( int i = 0; i < leds; ++i )
	{
		gpio_clr_gpio_pin( LED0 + i);
	}
	for( int i = leds; i < 8; ++i )
	{
		gpio_set_gpio_pin( LED0 + i);
	}
}


/*
* ADC konfigurieren
*/
void adc_config( void ) {
	// ADC konfigurieren
	// Verstaerkung mit 0.5
	// PA09  PA10   Verstaerkung
	//    0     0     2
	//    1     0     1
	//    0     1   0.5
	//    1     1   0.1
	gpio_clr_gpio_pin( AVR32_PIN_PA10 );
	gpio_clr_gpio_pin( AVR32_PIN_PA09 ); //We use a microphone with a long output level, so we need more amplification
	
	// Wechselspannung (Anhebung um VCC/2)
	gpio_set_gpio_pin( AVR32_PIN_PA28 );
	
	// Eingaenge ADC Funktion zuweisen
	gpio_enable_module_pin( AVR32_ADC_AD_0_PIN, AVR32_ADC_AD_0_FUNCTION );
	
	// ADC Mode Register setzen (Datenblatt Seite 548)
	// Auf 8-bit umstellen
	AVR32_ADC.mr |= 1 << AVR32_ADC_MR_LOWRES_OFFSET;
	
	adc_configure( &AVR32_ADC );
	
	adc_enable( &AVR32_ADC, 0 );
	adc_enable( &AVR32_ADC, 1 );
}

/*
* Timer konfigurieren
*/
void timer_config( void ) {
	// Options for waveform generation.
	static const tc_waveform_opt_t WAVEFORM_OPT =
	{
		.channel  = TC_CHANNEL,							// Channel selection.
		.wavsel	  = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER,	// Up mode with automatic trigger(reset) on RC compare
		.tcclks   = TC_CLOCK_SOURCE_TC3					// Internal source clock 3, connected to fPBA / 8.
	};
	
	static const tc_interrupt_t TC_INTERRUPT =
	{
		.cpcs  = 1,
	};
	
	tc_init_waveform( &AVR32_TC, &WAVEFORM_OPT );		// Initialize the timer/counter waveform.
	
	// We want: (1/(12MHz/8)) * RC = 1/44kHz , hence RC = (12MHz/8) / 1000/44 = 34 to get an interrupt every 1/44 ms.
	tc_write_rc( &AVR32_TC, TC_CHANNEL, ( sysclk_get_cpu_hz() / 8 ) / 6800 ); // Set RC value.
	
	tc_configure_interrupts( &AVR32_TC, TC_CHANNEL, &TC_INTERRUPT );
	
	// Start the timer/counter.
	tc_start( &AVR32_TC, TC_CHANNEL );
}



int8_t rotation;
int16_t colstart, rowstart;
uint8_t tabcolor;
uint16_t cursor_x, cursor_y, textcolor;
uint8_t textsize;


inline void spiwrite(uint8_t c)
{
	spi_put(SPI_EXAMPLE,c);
	while(!spi_is_tx_empty(SPI_EXAMPLE)) {}
	while(!spi_is_tx_ready(SPI_EXAMPLE)) {}
}

void writecommand(uint8_t c)
{
	gpio_clr_gpio_pin(DC);
	spi_selectChip(SPI_EXAMPLE, SPI_SLAVECHIP_NUMBER);
	spiwrite(c);
	spi_unselectChip(SPI_EXAMPLE,SPI_SLAVECHIP_NUMBER);
}

void writedata(uint8_t c)
{
	gpio_set_gpio_pin(DC);
	spi_selectChip(SPI_EXAMPLE, SPI_SLAVECHIP_NUMBER);
	spiwrite(c);
	spi_unselectChip(SPI_EXAMPLE,SPI_SLAVECHIP_NUMBER);
}

void writedata16(uint16_t d)
{
	gpio_set_gpio_pin(DC);
	spi_selectChip(SPI_EXAMPLE, SPI_SLAVECHIP_NUMBER);
	spiwrite(d >> 8);
	spiwrite(d);
	spi_unselectChip(SPI_EXAMPLE,SPI_SLAVECHIP_NUMBER);
}




// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t //PROGMEM
Bcmd[] = {                  // Initialization commands for 7735B screens
	18,                       // 18 commands in list:
	ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
	50,                     //     50 ms delay
	ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
	255,                    //     255 = 500 ms delay
	ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
	0x05,                   //     16-bit color
	10,                     //     10 ms delay
	ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
	0x00,                   //     fastest refresh
	0x06,                   //     6 lines front porch
	0x03,                   //     3 lines back porch
	10,                     //     10 ms delay
	ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
	0x08,                   //     Row addr/col addr, bottom to top refresh
	ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
	0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
	//     rise, 3 cycle osc equalize
	0x02,                   //     Fix on VTL
	ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
	0x0,                    //     Line inversion
	ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
	0x02,                   //     GVDD = 4.7V
	0x70,                   //     1.0uA
	10,                     //     10 ms delay
	ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
	0x05,                   //     VGH = 14.7V, VGL = -7.35V
	ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
	0x01,                   //     Opamp current small
	0x02,                   //     Boost frequency
	ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
	0x3C,                   //     VCOMH = 4V
	0x38,                   //     VCOML = -1.1V
	10,                     //     10 ms delay
	ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
	0x11, 0x15,
	ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
	0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
	0x21, 0x1B, 0x13, 0x19, //      these config values represent)
	0x17, 0x15, 0x1E, 0x2B,
	0x04, 0x05, 0x02, 0x0E,
	ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
	0x0B, 0x14, 0x08, 0x1E, //     (ditto)
	0x22, 0x1D, 0x18, 0x1E,
	0x1B, 0x1A, 0x24, 0x2B,
	0x06, 0x06, 0x02, 0x0F,
	10,                     //     10 ms delay
	ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
	0x00, 0x02,             //     XSTART = 2
	0x00, 0x81,             //     XEND = 129
	ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
	0x00, 0x02,             //     XSTART = 1
	0x00, 0x81,             //     XEND = 160
	ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
	10,                     //     10 ms delay
	ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
255 },                  //     255 = 500 ms delay

Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
	15,                       // 15 commands in list:
	ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
	150,                    //     150 ms delay
	ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
	255,                    //     500 ms delay
	ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
	0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
	ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
	0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
	ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
	0x01, 0x2C, 0x2D,       //     Dot inversion mode
	0x01, 0x2C, 0x2D,       //     Line inversion mode
	ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
	0x07,                   //     No inversion
	ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
	0xA2,
	0x02,                   //     -4.6V
	0x84,                   //     AUTO mode
	ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
	0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
	ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
	0x0A,                   //     Opamp current small
	0x00,                   //     Boost frequency
	ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
	0x8A,                   //     BCLK/2, Opamp current small & Medium low
	0x2A,
	ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
	0x8A, 0xEE,
	ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
	0x0E,
	ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
	ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
	0xC8,                   //     row addr/col addr, bottom to top refresh
	ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
0x05 },                 //     16-bit color

Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
	2,                        //  2 commands in list:
	ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
	0x00, 0x02,             //     XSTART = 0
	0x00, 0x7F+0x02,        //     XEND = 127
	ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
	0x00, 0x01,             //     XSTART = 0
0x00, 0x9F+0x01 },      //     XEND = 159
Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
	2,                        //  2 commands in list:
	ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
	0x00, 0x00,             //     XSTART = 0
	0x00, 0x7F,             //     XEND = 127
	ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
	0x00, 0x00,             //     XSTART = 0
0x00, 0x9F },           //     XEND = 159

Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
	4,                        //  4 commands in list:
	ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
	0x02, 0x1c, 0x07, 0x12,
	0x37, 0x32, 0x29, 0x2d,
	0x29, 0x25, 0x2B, 0x39,
	0x00, 0x01, 0x03, 0x10,
	ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
	0x03, 0x1d, 0x07, 0x06,
	0x2E, 0x2C, 0x29, 0x2D,
	0x2E, 0x2E, 0x37, 0x3F,
	0x00, 0x00, 0x02, 0x10,
	ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
	10,                     //     10 ms delay
	ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void commandList(const uint8_t *addr)
{
	uint8_t  numCommands, numArgs;
	uint16_t ms;

	numCommands = pgm_read_byte(addr++);		// Number of commands to follow
	while(numCommands--) {				// For each command...
		writecommand(pgm_read_byte(addr++));	//   Read, issue command
		numArgs  = pgm_read_byte(addr++);	//   Number of args to follow
		ms       = numArgs & DELAY;		//   If hibit set, delay follows args
		numArgs &= ~DELAY;			//   Mask out delay bit
		while(numArgs--) {			//   For each argument...
			writedata(pgm_read_byte(addr++)); //   Read, issue argument
		}

		if(ms) {
			ms = pgm_read_byte(addr++);	// Read post-command delay time (ms)
			if(ms == 255) ms = 500;		// If 255, delay for 500 ms
			delay(ms);
		}
	}
}


// Initialization code common to both 'B' and 'R' type displays
void commonInit(const uint8_t *cmdList)
{
	colstart  = rowstart = 0; // May be overridden in init func


	gpio_set_gpio_pin( RST );
	delay(500);
	gpio_clr_gpio_pin( RST );
	delay(500);
	gpio_set_gpio_pin( RST );
	delay(500);
	

	if(cmdList) commandList(cmdList);
}


// Initialization for ST7735B screens
void initB(void)
{
	commonInit(Bcmd);
}


// Initialization for ST7735R screens (green or red tabs)
void initR(uint8_t options)
{
	commonInit(Rcmd1);
	if (options == INITR_GREENTAB) {
		commandList(Rcmd2green);
		colstart = 2;
		rowstart = 1;
		} else {
		// colstart, rowstart left at default '0' values
		commandList(Rcmd2red);
	}
	commandList(Rcmd3);

	// if black, change MADCTL color filter
	if (options == INITR_BLACKTAB) {
		writecommand(ST7735_MADCTL);
		writedata(0xC0);
	}

	tabcolor = options;
}


void setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	writecommand(ST7735_CASET); // Column addr set
	writedata16(x0+colstart);   // XSTART
	writedata16(x1+colstart);   // XEND
	writecommand(ST7735_RASET); // Row addr set
	writedata16(y0+rowstart);   // YSTART
	writedata16(y1+rowstart);   // YEND
	writecommand(ST7735_RAMWR); // write to RAM
}


void pushColor(uint16_t color)
{
	writedata16(color);
}

void drawPixel(int16_t x, int16_t y, uint16_t color)
{
	if ((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;
	setAddrWindow(x,y,x+1,y+1);
	writedata16(color);
}


void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((y+h-1) >= _height) h = _height-y;
	setAddrWindow(x, y, x, y+h-1);
	while (h--) {
		writedata16(color);
	}
}


void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	// Rudimentary clipping
	if ((x >= _width) || (y >= _height)) return;
	if ((x+w-1) >= _width)  w = _width-x;
	setAddrWindow(x, y, x+w-1, y);
	while (w--) {
		writedata16(color);
	}
}



// fill a rectangle
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if ((x >= _width) || (y >= _height)) return;
	if ((x + w - 1) >= _width)  w = _width  - x;
	if ((y + h - 1) >= _height) h = _height - y;
	setAddrWindow(x, y, x+w-1, y+h-1);
	for (y=h; y>0; y--) {
		for(x=w; x>0; x--) {
			writedata16(color);
		}
	}
}

void fillScreen(uint16_t color)
{
	fillRect(0, 0,  _width, _height, color);
}

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void setRotation(uint8_t m)
{
	writecommand(ST7735_MADCTL);
	rotation = m % 4; // can't be higher than 3
	if (rotation == 0) {
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
			} else {
			writedata(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
		}
		_width  = ST7735_TFTWIDTH;
		_height = ST7735_TFTHEIGHT;
	}
	if (rotation == 1) {
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
			} else {
			writedata(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		}
		_width  = ST7735_TFTHEIGHT;
		_height = ST7735_TFTWIDTH;
	}
	if (rotation == 2) {
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_RGB);
			} else {
			writedata(MADCTL_BGR);
		}
		_width  = ST7735_TFTWIDTH;
		_height = ST7735_TFTHEIGHT;
	}
	if (rotation == 3) {
		if (tabcolor == INITR_BLACKTAB) {
			writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
			} else {
			writedata(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
		}
		_width  = ST7735_TFTHEIGHT;
		_height = ST7735_TFTWIDTH;
	}
}


void invertDisplay(uint8_t i)
{
	writecommand(i ? ST7735_INVON : ST7735_INVOFF);
}

void drawCircle(int16_t x0, int16_t y0, int16_t r,
uint16_t color) {
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	drawPixel(x0  , y0+r, color);
	drawPixel(x0  , y0-r, color);
	drawPixel(x0+r, y0  , color);
	drawPixel(x0-r, y0  , color);

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		drawPixel(x0 + x, y0 + y, color);
		drawPixel(x0 - x, y0 + y, color);
		drawPixel(x0 + x, y0 - y, color);
		drawPixel(x0 - x, y0 - y, color);
		drawPixel(x0 + y, y0 + x, color);
		drawPixel(x0 - y, y0 + x, color);
		drawPixel(x0 + y, y0 - x, color);
		drawPixel(x0 - y, y0 - x, color);
	}
}

void drawCircleHelper( int16_t x0, int16_t y0,
int16_t r, uint8_t cornername, uint16_t color) {
	int16_t f     = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x     = 0;
	int16_t y     = r;

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f     += ddF_y;
		}
		x++;
		ddF_x += 2;
		f     += ddF_x;
		if (cornername & 0x4) {
			drawPixel(x0 + x, y0 + y, color);
			drawPixel(x0 + y, y0 + x, color);
		}
		if (cornername & 0x2) {
			drawPixel(x0 + x, y0 - y, color);
			drawPixel(x0 + y, y0 - x, color);
		}
		if (cornername & 0x8) {
			drawPixel(x0 - y, y0 + x, color);
			drawPixel(x0 - x, y0 + y, color);
		}
		if (cornername & 0x1) {
			drawPixel(x0 - y, y0 - x, color);
			drawPixel(x0 - x, y0 - y, color);
		}
	}
}


// Used to do circles and roundrects
void fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
uint8_t cornername, int16_t delta, uint16_t color) {

	int16_t f     = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x     = 0;
	int16_t y     = r;

	while (x<y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f     += ddF_y;
		}
		x++;
		ddF_x += 2;
		f     += ddF_x;

		if (cornername & 0x1) {
			drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
			drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
		}
		if (cornername & 0x2) {
			drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
			drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
		}
	}
}

void fillCircle(int16_t x0, int16_t y0, int16_t r,
uint16_t color) {
	drawFastVLine(x0, y0-r, 2*r+1, color);
	fillCircleHelper(x0, y0, r, 3, 0, color);
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
uint16_t color) {
	int16_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		_swap_int16_t(x0, y0);
		_swap_int16_t(x1, y1);
	}

	if (x0 > x1) {
		_swap_int16_t(x0, x1);
		_swap_int16_t(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
		} else {
		ystep = -1;
	}

	for (; x0<=x1; x0++) {
		if (steep) {
			drawPixel(y0, x0, color);
			} else {
			drawPixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}

// Draw a rectangle
void  drawRect(int16_t x, int16_t y, int16_t w, int16_t h,
uint16_t color) {
	drawFastHLine(x, y, w, color);
	drawFastHLine(x, y+h-1, w, color);
	drawFastVLine(x, y, h, color);
	drawFastVLine(x+w-1, y, h, color);
}


// Draw a rounded rectangle
void drawRoundRect(int16_t x, int16_t y, int16_t w,
int16_t h, int16_t r, uint16_t color) {
	// smarter version
	drawFastHLine(x+r  , y    , w-2*r, color); // Top
	drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
	drawFastVLine(x    , y+r  , h-2*r, color); // Left
	drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
	// draw four corners
	drawCircleHelper(x+r    , y+r    , r, 1, color);
	drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
	drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
	drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// Fill a rounded rectangle
void fillRoundRect(int16_t x, int16_t y, int16_t w,
int16_t h, int16_t r, uint16_t color) {
	// smarter version
	fillRect(x+r, y, w-2*r, h, color);

	// draw four corners
	fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
	fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}


// Draw a character
void drawChar(int16_t x, int16_t y, unsigned char c,
uint16_t color, uint16_t bg, uint8_t size) {


	if((x >= _width)            || // Clip right
	(y >= _height)           || // Clip bottom
	((x + 6 * size - 1) < 0) || // Clip left
	((y + 8 * size - 1) < 0))   // Clip top
	return;

	for(int8_t i=0; i<6; i++ ) {
		uint8_t line;
		if(i < 5) line = pgm_read_byte(font+(c*5)+i);
		else      line = 0x0;
		for(int8_t j=0; j<8; j++, line >>= 1) {
			if(line & 0x1) {
				if(size == 1) drawPixel(x+i, y+j, color);
				else          fillRect(x+(i*size), y+(j*size), size, size, color);
				} else if(bg != color) {
				if(size == 1) drawPixel(x+i, y+j, bg);
				else          fillRect(x+i*size, y+j*size, size, size, bg);
			}
		}
	}
}

void print(char characters[], uint16_t x, uint16_t y, uint16_t c, uint16_t bg, uint16_t size) {
	int i = 0;
	while (characters[i] != NULL) {
		drawChar(x+(i*5*size), y, characters[i], c, bg, size);
		i++;
	}
}


inline int FIX_MPY(int a, int b) {
	int c = ((int)a * (int)b) >> 6; /* shift right one less bit (i.e. 15-1) */
	b = c & 0x01; /* last bit shifted out = rounding-bit */
	a = (c >> 1) + b;/* last shift + rounding bit */
	return a;
}

int fix_fft(int fr[], int fi[], int m, int inverse){
	// fix_fft() - Forward / Inverse Fast Fourier Transform.
	// fr[n],fi[n] Real und Imaginär Teil Felder für Eingabe und Ergebnis
	// mit 0 <= n < 2**m;
	// inverse=0 heisst forward transform (FFT), oder =1 für iFFT.
	int mr, nn, i, j, l, k, istep, n, scale, shift;
	int qr, qi, tr, ti, wr, wi;
	int idx;
	n = 1 << m;
	if (n > N_WAVE) return -1;  /* max FFT size = N_WAVE */
	mr = 0;
	nn = n - 1;
	scale = 0;
	for (m=1; m <= nn; ++m) { /* decimation in time - re-order data */
		l = n;
		do { l >>= 1;
		} while (mr+l > nn);
		mr = (mr & (l-1)) + l;
		if (mr <= m) continue;
		tr = fr[m];
		fr[m] = fr[mr];
		fr[mr] = tr;
		ti = fi[m];
		fi[m] = fi[mr];
		fi[mr] = ti;
	}
	l = 1;
	k = LOG2_N_WAVE-1;
	while (l < n) {
		if (inverse) {
			shift = 0; /* variable scaling, depending upon data */
			for (i=0; i < n; ++i) {
				j = fr[i];
				if (j < 0) j = -j;
				m = fi[i];
				if (m < 0) m = -m;
				if (j > 16383 || m > 16383) {
					shift = 1;
					break;
				}
			}
			if (shift) ++scale;
		}
		else { // nicht invers
			shift = 1;
		}
		istep = l << 1;
		for (m=0; m < l; ++m) {
			j = m << k;
			if((idx = j+N_WAVE/4) >= 128) wr = -Sinewave[idx - 128];
			else wr = Sinewave[idx];
			if(j >= 128) wi = Sinewave[j];
			else wi = -Sinewave[j];
			if (inverse) wi = -wi;
			if (shift) {
				wr >>= 1;
				wi >>= 1;
			}
			for (i=m; i < n; i+=istep) {
				j = i + l;
				tr = FIX_MPY(wr,fr[j]) - FIX_MPY(wi,fi[j]);
				ti = FIX_MPY(wr,fi[j]) + FIX_MPY(wi,fr[j]);
				qr = fr[i];
				qi = fi[i];
				if (shift) {
					qr >>= 1;
					qi >>= 1;
				}
				fr[j] = qr - tr;
				fi[j] = qi - ti;
				fr[i] = qr + tr;
				fi[i] = qi + ti;
			}
		}
		--k;
		l = istep;
	}
	return scale;
}

void print_fft() {
	int i = 0, j = 0, largest = 0;
	/*
	largest = 0;
	for (i=0; i < 64;i++){ // Die größe Amplitude gint den maximalen Y-Achsenwert
	re[i] = sqrt(re[i] * re[i] + im[i] * im[i]);
	if(re[i] > largest) largest = re[i];
	}
	*/
	//fillRect(0, 0, 128, 80, ST7735_BLACK);
	

	for(j=40;j >= 0;j--) { // die Grafik ausgeben, mit der höchsten Amplitude anfangan
		for(i=0;i < 64;i++) {
			if((re[i] >= j) && old[i][j] == 0) {
				old[i][j] = 1;
				fillRect(16+(i*2), 110-j*2, 2, 2, ST7735_GREEN);
			} // Wenn die Amplitude dieses Wertes mindesten so gross ist wie die aktuelle, einen * ausgeben
			if (!(re[i] >= j) && old[i][j] == 1){
				old[i][j] = 0;
				fillRect(16+(i*2), 110-j*2, 2, 2, ST7735_BLACK);
			}
		}
	}
	
}

int16_t tempADC = 0;

/*
* Interruptroutine fuer Lautstaerkepegel. Wird jede 1/44 ms aufgerufen
*/
interrupt void tc_int_handler( void ) {
	tc_read_sr( &AVR32_TC, TC_CHANNEL ); // delete interrupt flag

	counter++;
	
	adc_start( &AVR32_ADC ); // Konvertierung beginnen
	ad_value = adc_get_value( &AVR32_ADC, 0 ) - (adc_get_value( &AVR32_ADC, 1 ) - 128); // retrieve ADC value, remove adjustable offset (IN2 is connected to 0-10V trimmer)
	
	/*
	srand(42);
	re[counter] = (int)rand()%(AMP_CARRIER*2)-AMP_CARRIER;//(int)(AMP_CARRIER*sin(phase*2.0*PI));
	phase += freq/F_SAMPLE; // Phase von Welle 1 updaten
	if(phase >= 1)phase -= 1;
	*/
	re[counter] = ad_value * 8;
	tempADC = ad_value;
	
	if (ad_value < 0) {
		ad_value = 0;
	}
	
	tempVolume += ad_value * ad_value;
	/*
	if (counter > 1000) {
	counter = 0;
	volume = tempVolume;
	tempVolume = 0;
	}*/
	im[counter] = 0; // Imaginärteil ist immer 0
	if (counter > N_WAVE/2) {
		
		if (tempADC > 0) {
			fillRect( tempVolume / 256 + 16, 25, 160-tempVolume / 256, 10, ST7735_BLACK);
			fillRect(15, 25, tempVolume / 256, 10, ST7735_RED);
		}
		
		volume = tempVolume;
		tempVolume = 0;
		counter = 0;
		phase = 0.0;
		int i = 0;/*
		for(i = 0; i < 128; i++) {
		re[i] = (int)(AMP_CARRIER*sin(phase*2.0*PI)); // Welle 1 berechnen
		phase += freq/F_SAMPLE; // Phase von Welle 1 updaten
		if(phase >= 1)phase -= 1;
		}*/
		fix_fft(re,im,7,0); // die FFT der Messwerte berechnen
		print_fft();
		freq = 100;
	}

}

/*
* Interrupt konfigurieren
*/
void interrupt_config( void ) {
	// Interrupts
	// Disable all interrupts.
	Disable_global_interrupt();
	// Initialize interrupt vectors.
	INTC_init_interrupts();
	// Register interrupts
	INTC_register_interrupt( tc_int_handler, AVR32_TC_IRQ0, AVR32_INTC_INT0 );
	// Enable all interrupts.
	Enable_global_interrupt();
}


/*
* Main-Funktion
*/
int main( void ) {
	
	board_init();
	
	sysclk_init();
	
	adc_config();
	
	enable_led();
	
	spi_init_module();
	
	initR(INITR_BLACKTAB);
	setRotation(3);
	
	for (int i = 0; i < 1; i++) {
		gpio_clr_gpio_pin(LED0+2);
		delay(200);
		gpio_set_gpio_pin(LED0+2);
		delay(200);
	}
	
	fillScreen(ST7735_BLACK);
	
	print("Hello!", 0, 10, ST7735_WHITE, ST7735_BLACK, 2);
	
	delay(1000);
	
	print("Welcome to the WSiualizer", 0, 40, ST7735_GREEN, ST7735_BLACK, 1);
	
	delay(2000);
	
	print("Was ist das fuer", 0, 100, ST7735_RED, ST7735_BLACK, 1);
	print("1 schlechter Wortwitz?!", 0, 110, ST7735_RED, ST7735_BLACK, 1);
	
	
	delay(1000);
	fillScreen(ST7735_BLACK);
	
	drawFastHLine(15, 112, 128, ST7735_BLUE);
	drawFastVLine(15, 40, 71, ST7735_BLUE);
	print("80Hz", 15, 114, ST7735_CYAN, ST7735_BLACK, 1);
	print("5kHz", 70, 114, ST7735_CYAN, ST7735_BLACK, 1);
	print("60kHz", 120, 114, ST7735_CYAN, ST7735_BLACK, 1);
	
	print(">THE< WSISUALIZER", 40, 10, ST7735_WHITE, ST7735_BLACK, 1);
	
	interrupt_config();
	
	timer_config();
	
	while(1) {
		ledBarGraph((volume / 256) >> 2);
	}
}

