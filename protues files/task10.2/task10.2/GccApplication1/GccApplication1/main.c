#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint16_t encoder_count = 0;

void gpio_init()
{
	// Set PB0 as output (Motor control)
	DDRB |= (1 << DDB0); // Changed DDD6 to DDB0 assuming PB0 is used for motor control
	// Set PC0 as input (Potentiometer)
	DDRC &= ~(1 << DDC0);
	// Set PD0 as input (Encoder)
	DDRD &= ~(1 << DDD0); // No change, assuming PD0 is connected to the encoder
}

void adc_init()
{
	ADMUX = (1 << REFS0);                               // AVcc with external capacitor at AREF
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC and set prescaler
}

uint16_t adc_read(uint8_t channel)
{
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC))
	;
	return ADC;
}

void timer_init()
{
	TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM0A1); // Fast PWM mode and non-inverting mode on OC0A
	TCCR0B = (1 << CS00);                                // No prescaling
	OCR0A = 0;                                           // Set initial duty cycle to 0
}

void set_motor_speed(uint8_t speed)
{
	OCR0A = speed;
}

void ext_interrupt_init()
{
	EIMSK |= (1 << INT0);    // Enable external interrupt INT0
	EICRA |= (1 << ISC01);   // Trigger INT0 on falling edge (changed ISC010 to ISC01)
	sei();                   // Enable global interrupts
}

ISR(INT0_vect)
{
	encoder_count++;
}

void spi_init()
{
	DDRB |= (1 << DDB3) | (1 << DDB5); // MOSI and SCK as output
	DDRB &= ~(1 << DDB4);              // MISO as input
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0); // Enable SPI, Master mode, set clock rate fck/16
}

uint8_t spi_transfer(uint8_t data)
{
	SPDR = data;
	while (!(SPSR & (1 << SPIF)))
	;
	return SPDR;
}

int main()
{
	gpio_init();
	adc_init();
	timer_init();
	ext_interrupt_init();
	spi_init();

	sei(); // Enable global interrupts

	while (1)
	{
		uint16_t pot_value = adc_read(0);

		uint8_t motor_speed = (pot_value * 255) / 1023;
		set_motor_speed(motor_speed);

		uint8_t current_rpm = (encoder_count * 60) / 100; // Change scaling as needed
		encoder_count = 0;                                // Reset count for next calculation

		spi_transfer(current_rpm);
		_delay_ms(100);
	}
}
