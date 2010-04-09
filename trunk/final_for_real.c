/*******************************************
**********	  Acoustic Modem 	************
*******************************************/

// Standard headers
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <math.h>
#include <util/delay.h>
#include <string.h>

// Project Definitions
#include "sin_vals.h"

uint8_t sample_num = 0;
uint16_t freq_sum = 0; 
void init();

ISR(TIMER1_COMPA_vect) {
	PORTD = 0;
	freq_sum = pgm_read_byte(f4000 + sample_num) + pgm_read_byte(f6000 + sample_num);
	PORTC = freq_sum >> 1;
	sample_num = (sample_num + 1) & 0x3f;
	PORTD = (1 << PB7);		
}

ISR(TIMER1_COMPB_vect, ISR_NAKED) {
	sei();
	sleep_cpu();
	reti();
}

int main() {
	uint8_t recv_byte = 0;
	init();
	while(1) {
		if(UCSR0A & _BV(RXC0)) {	// We've received a serial byte
			recv_byte = UDR0;
			if(UCSR0A & _BV(UDRE0)) {
				UDR0 = recv_byte;
			}
		}
	}
}

void init() {
	DDRC = 0xff;
	DDRD = _BV(PB7) | _BV(PB1) | _BV(PB0);

	/* Setup sampling timer */
	TCCR1B = _BV(WGM12) | _BV(CS10);	// Clear on timer compare, no prescaling
	OCR1A = 500;				// Fire every 25us = 40000Hz (our sampling frequency)
	OCR1B = 490;
	TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);		// Enable output compare A interrupt

	/* Setup UART */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);	// Enable receive and transmit
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);		// Set 8-bit frames
	UBRR0 = 129;	// Set baud rate to 9600

	sei();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
}
