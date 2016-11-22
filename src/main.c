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
#include <asf.h>

#define LED0 AVR32_PIN_PB00

#define interrupt __attribute__((__interrupt__))
#define TC_CHANNEL   0
#define FALSE 0

//#define DEBUG 1

// Buffer

// Buffergroesse
//uint16_t buffer_sample_size = 1000;

// Buffer
//uint32_t buffer_sample[1000];

// Bufferzaehler
//uint32_t i_sample;

// Buffersumme
//uint32_t buffer_sample_sum = 0;

int led_count;

int16_t ad_value;

uint16_t counter;
uint32_t tempVolume;
uint32_t volume;

#define USART               (&AVR32_USART1) //TX = PB2, for debugging 

const static usart_options_t usart_opt = {
	// UART config
	.baudrate    = 57600,
	.channelmode = USART_NORMAL_CHMODE,
	.charlength  = 8,
	.paritytype  = USART_NO_PARITY,
	.stopbits    = USART_1_STOPBIT,
};

/*
 * Interruptroutine fuer Lautstaerkepegel. Wird jede 1/44 ms aufgerufen
 */
interrupt void tc_int_handler( void ) {
	tc_read_sr( &AVR32_TC, TC_CHANNEL ); // delete interrupt flag

	counter++;
	adc_start( &AVR32_ADC ); // Konvertierung beginnen
	ad_value = adc_get_value( &AVR32_ADC, 0 ) - (adc_get_value( &AVR32_ADC, 1 ) - 128); // retrieve ADC value, remove adjustable offset (IN2 is connected to 0-10V trimmer)
	if (ad_value < 0) {
		ad_value = 0;
	}
	
	tempVolume += ad_value * ad_value;
	
	if (counter > 1000) {
		counter = 0;
		volume = tempVolume;
		tempVolume = 0;		
	}
}

/*
 * GPIO aktivieren
 */
void enable_led( void ) {
	for( int i = 0; i < 8; ++i )
	{
		gpio_enable_gpio_pin( LED0 + i );	// Port Enable
	}
}

void ledBarGraph ( uint8_t leds) { // 0 - 8; 0 = all led's off
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
	tc_write_rc( &AVR32_TC, TC_CHANNEL, ( sysclk_get_cpu_hz() / 8 ) / 44000 ); // Set RC value.
	
	tc_configure_interrupts( &AVR32_TC, TC_CHANNEL, &TC_INTERRUPT );
	
	// Start the timer/counter.
	tc_start( &AVR32_TC, TC_CHANNEL );
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
	
	#ifdef DEBUG
	sysclk_enable_peripheral_clock(USART);

	usart_init_rs232(USART, &usart_opt,
			sysclk_get_peripheral_bus_hz(USART));

	usart_write_line(USART, "Start\n");
	#endif
	
	interrupt_config();
	
	timer_config();
	
	while(1) {
		ledBarGraph((volume / 512) >> 2); // first and second LSB are noise anyway, they waste LED's
	}
}
