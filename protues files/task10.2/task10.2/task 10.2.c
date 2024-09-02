#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile uint16_t encoder_count = 0;

void gpio_init()
{
    // Set PB0 as output (Motor control)
    DDRB |= (1 << DDD6); //?????????
    // Set PC0 as input (Potentiometer)
    DDRC &= ~(1 << DDC0);
    // Set PD0 as input (Encoder)
    DDRD &= ~(1 << DDD0); /////
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
    TCCR0A = (1 << WGM00) | (1 << WGM01) | (1 << COM01); // Fast PWM mode
    TCCR0B = (1 << CS00);                                // No prescaling
    OCR0A = 0;                                           // Set initial duty cycle to 0
}

void set_motor_speed(uint8_t speed)
{
    OCR0A = speed;
}

void ext_interrupt_init()
{
    EIMSK |= (1 << INT0);
    EICRA |= (1 << ISC010);
    sei();
}

ISR(INT0_vect)
{
    encoder_count++;
}

void spi_init()
{
    DDRB |= (1 << DDB3) | (1 << DDB5);
    DDRB &= ~(1 << DDB4);
    SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
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

    sei();

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