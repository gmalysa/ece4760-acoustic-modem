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

#define IDLE -2
#define START_BIT -1
#define STOP_BIT 8

uint8_t sample_num = 0;
volatile uint8_t output_buffer[8];
volatile int8_t buffer_status[8] = {IDLE,IDLE,IDLE,IDLE,IDLE,IDLE,IDLE,IDLE};
uint8_t next_buffer = 0;
uint8_t output_bitpattern = 0;
uint8_t bitmasks[8] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};
void init();

ISR(TIMER1_COMPA_vect) {
	uint8_t i;
	PORTD = 0;
	PORTC = pgm_read_byte(freqBuffer[output_bitpattern] + sample_num);
	sample_num = (sample_num + 1) & 0x3f;
	PORTD = (1 << PB7);		
	if(sample_num == 0) {
		output_bitpattern = 0;
		for(i=0; i<8; ++i) {
			if(buffer_status[i] == IDLE) {continue;}
			else if(buffer_status[i] == START_BIT) {
				output_bitpattern |= bitmasks[i];
			}
			else if(buffer_status[i] == STOP_BIT) {
				buffer_status[i] = IDLE;
				continue;
			}
			else if(bitmasks[7-buffer_status[i]] & output_buffer[i]) {
				output_bitpattern |= bitmasks[i];
			}
			++buffer_status[i];
		}
	}
}

ISR(TIMER1_COMPB_vect, ISR_NAKED) {
	sei();
	sleep_cpu();
	reti();
}

int main() {
	init();
	while(1) {
		if(UCSR0A & _BV(RXC0)) {	// We've received a serial byte
			while(buffer_status[next_buffer] != IDLE) {;}
			output_buffer[next_buffer] = UDR0;
			UDR0 = output_buffer[next_buffer];
			buffer_status[next_buffer] = START_BIT;	
			next_buffer = (next_buffer + 1) & 0x7;
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
