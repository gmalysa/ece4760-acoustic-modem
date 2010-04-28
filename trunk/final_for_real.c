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

// 6:10 fixed point math defines
#define int2fix(a) (((uint16_t)(a))<<10)
#define fix2int(a) ((uint8_t)((a)>>10))
#define float2fix(a) ((uint16_t)((a)*1024.0))
#define fix2float(a) ((float)(a)/1024.0)

// Buffer status defines
#define IDLE -2
#define START_BIT -1
#define STOP_BIT 8

// Decoding defines
#define IN_BUFFER_SIZE 64
#define NOTIFY_FREQ 16
#define THRESH 5000

// Send variables
uint8_t output_sample_num = 0;
uint8_t output_buffer[8] = {0,0,0,0,0,0,0,0};
volatile int8_t output_buffer_status[8] = {IDLE,IDLE,IDLE,IDLE,IDLE,IDLE,IDLE,IDLE};
uint8_t next_buffer = 0;
uint8_t output_bitpattern = 0;
uint8_t bitmasks[8] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};
uint8_t freqCache[64];

// Debug variables
//#define DEBUG_DUMP
//#define DEBUG_THRESH
//#define DEBUG_RS_DUMP
#if defined(DEBUG_DUMP) || defined(DEBUG_RS_DUMP)
	int8_t debugCache[1024];
	volatile uint16_t debugCache_p = 0;
	volatile uint8_t debug_enabled = 1;
	uint16_t curr_debug_output;
	volatile uint16_t offset_timer = 0;
#endif

const uint16_t f4000_increment = float2fix(10.);
const uint16_t f4000_quarterphase = float2fix(2.5);

// Receive variables
#define NUM_FREQS 2
#define RESAMPLE_BUFFER_MAX_SIZE 150
float freqs[] = {4000., 6000.};
struct recv_param_t {
	int8_t *resample_buffer_sin;
	int8_t *resample_buffer_cos;
	uint8_t resample_buffer_position;
	int16_t sin_acc;
	int16_t cos_acc;
	int16_t prev_sin_acc;
	int16_t prev_cos_acc;
	int16_t last_derv_sin;
	int16_t last_derv_cos;
	uint16_t input_buffer_resample_position;
	uint8_t resample_buffer_size;
	uint16_t input_buffer_increment;
	uint16_t input_buffer_quarterphase;
	uint16_t boundary_alignment;
	uint8_t start;
	uint8_t output_char;
};
int8_t resample_buffer_sin[RESAMPLE_BUFFER_MAX_SIZE];
int8_t resample_buffer_cos[RESAMPLE_BUFFER_MAX_SIZE];
uint8_t resample_buffer_next_free = 0;

int8_t resample_buffer_sin_4000[7];
int8_t resample_buffer_cos_4000[7];
int8_t resample_buffer_sin_6000[10];
int8_t resample_buffer_cos_6000[10];
/*int8_t resample_buffer_sin_7000[12];
int8_t resample_buffer_cos_7000[12];
int8_t resample_buffer_sin_9000[15];
int8_t resample_buffer_cos_9000[15];
int8_t resample_buffer_sin_10000[16];
int8_t resample_buffer_cos_10000[16];
int8_t resample_buffer_sin_11000[18];
int8_t resample_buffer_cos_11000[18];
int8_t resample_buffer_sin_13000[21];
int8_t resample_buffer_cos_13000[21];
int8_t resample_buffer_sin_13000[24];
int8_t resample_buffer_cos_13000[24];*/
struct recv_param_t recv_params[NUM_FREQS];
volatile int8_t input_buffer[IN_BUFFER_SIZE];
//int8_t resample_buffer_sin[7] = {0,0,0,0,0,0,0};
//int8_t resample_buffer_cos[7] = {0,0,0,0,0,0,0};
uint8_t resample_buffer_position = 0;
int16_t sin_acc = 0, cos_acc = 0, prev_sin_acc = 0, prev_cos_acc = 0, last_derv_sin = 0, last_derv_cos = 0;
uint16_t input_buffer_resample_position = 0;
uint8_t input_buffer_pos = 0;
volatile uint8_t public_input_buffer_position = 0;
void init();
void find_freq(uint8_t);

ISR(TIMER1_COMPA_vect) {
	uint8_t i;
	PORTD &= ~_BV(PB7);
	PORTC = freqCache[output_sample_num];
	output_sample_num = (output_sample_num + 1) & 0x3f;
	PORTD |= _BV(PB7);	
	input_buffer[input_buffer_pos] = ADCH - 0x7F;
	ADCSRA |= _BV(ADSC);
	public_input_buffer_position = input_buffer_pos;
	#ifdef DEBUG_DUMP
		if(debug_enabled) {
			debugCache[debugCache_p] = input_buffer[input_buffer_pos];
			debugCache_p = (debugCache_p + 1) & 0x3ff;
			if(offset_timer > 0) {
				++offset_timer;
			}
		}
	#endif
	input_buffer_pos = (input_buffer_pos+1) & (IN_BUFFER_SIZE - 1);
	if(output_sample_num == 0) {
		output_bitpattern = 0;
		for(i=0; i<8; ++i) {
			if(output_buffer_status[i] == IDLE) {continue;}
			else if(output_buffer_status[i] == START_BIT) {
				output_bitpattern |= bitmasks[i];
			}
			else if(output_buffer_status[i] == STOP_BIT) {
				output_buffer_status[i] = IDLE;
				continue;
			}
			else if (output_buffer[i] & 0x80) {
				output_bitpattern |= bitmasks[i];
			}
			if (output_buffer_status[i] != START_BIT) {
				output_buffer[i] = output_buffer[i] << 1;
			}
			++output_buffer_status[i];
		}
		memcpy_P(freqCache, freqTable[output_bitpattern], sizeof(uint8_t) * 64);
	}
}

ISR(TIMER1_COMPB_vect, ISR_NAKED) {
	//PORTD ^= (1 << PB2);
	sei();
	sleep_cpu();
	reti();
}

int main() {
	uint8_t i = 0;
	init();
	while(1) {
		#if defined(DEBUG_DUMP) || defined(DEBUG_RS_DUMP)
			if(offset_timer >= 700) {
				debug_enabled = 0;
				curr_debug_output = debugCache_p+1;
				while(curr_debug_output != debugCache_p) {
					while(!(UCSR0A & _BV(UDRE0))) {;}
					UDR0 = debugCache[curr_debug_output];
					curr_debug_output = (curr_debug_output+1) & 0x3ff;
				}
				debug_enabled = 1;
				offset_timer = 0;
			}
		#endif
		if(UCSR0A & _BV(RXC0)) {	// We've received a serial byte
			while(output_buffer_status[next_buffer] != IDLE) {;}
			output_buffer[next_buffer] = UDR0;
			output_buffer_status[next_buffer] = START_BIT;	
			//next_buffer = (next_buffer + 1) & 0x7;
		}
		for(i=0; i<NUM_FREQS; ++i) {
			find_freq(i);
		}
	}
}

void find_freq(uint8_t i) {
	int8_t input_sample = 0;
	int32_t analyze_output;
	uint8_t x = fix2int(recv_params[i].input_buffer_resample_position + recv_params[i].input_buffer_quarterphase);
	
	if((x <= public_input_buffer_position && public_input_buffer_position - x <= 30) ||
	   (x > public_input_buffer_position && x - public_input_buffer_position > 30)) {
		recv_params[i].last_derv_sin = recv_params[i].prev_sin_acc - recv_params[i].sin_acc;
		recv_params[i].prev_sin_acc = recv_params[i].sin_acc;
		recv_params[i].sin_acc -= recv_params[i].resample_buffer_sin[recv_params[i].resample_buffer_position];
		input_sample = input_buffer[fix2int(recv_params[i].input_buffer_resample_position)];
		recv_params[i].sin_acc += input_sample;
		recv_params[i].resample_buffer_sin[recv_params[i].resample_buffer_position] = input_sample;

		recv_params[i].last_derv_cos = recv_params[i].prev_cos_acc - recv_params[i].cos_acc;
		recv_params[i].prev_cos_acc = recv_params[i].cos_acc;
		recv_params[i].cos_acc -= recv_params[i].resample_buffer_cos[recv_params[i].resample_buffer_position];
		input_sample = input_buffer[fix2int(recv_params[i].input_buffer_resample_position + recv_params[i].input_buffer_quarterphase)];
		recv_params[i].cos_acc += input_sample;
		recv_params[i].resample_buffer_cos[recv_params[i].resample_buffer_position] = input_sample;

		if(recv_params[i].start == 0 || (recv_params[i].start > 0 && recv_params[i].resample_buffer_position == recv_params[i].resample_buffer_size)) {
			analyze_output = (recv_params[i].sin_acc * recv_params[i].sin_acc) + (recv_params[i].cos_acc * recv_params[i].cos_acc);
		}
		else {
			analyze_output = 0;
		}	

		if(analyze_output > THRESH && recv_params[i].start == 0) {
			#if defined(DEBUG_DUMP) || defined(DEBUG_RS_DUMP)	//TODO: fix debug
				if(offset_timer == 0) {
					offset_timer = 1;
				}
			#endif
			if(abs(recv_params[i].sin_acc) > abs(recv_params[i].cos_acc)) {
				if(((recv_params[i].prev_sin_acc - recv_params[i].sin_acc) ^ recv_params[i].last_derv_sin) & 0x8000) {
					recv_params[i].start = 9;
				}
			}
			else {
				if(((recv_params[i].prev_cos_acc - recv_params[i].cos_acc) ^ recv_params[i].last_derv_cos) & 0x8000) {
					recv_params[i].start = 9;
				}
			}
			if(recv_params[i].start == 9) {
				recv_params[i].output_char = 0;
				recv_params[i].resample_buffer_position = recv_params[i].resample_buffer_size;
				recv_params[i].input_buffer_resample_position -= int2fix(recv_params[i].input_buffer_increment - recv_params[i].boundary_alignment);
			}
		}

		if(recv_params[i].resample_buffer_position++ == recv_params[i].resample_buffer_size) {
			recv_params[i].resample_buffer_position = 0;
			if (recv_params[i].start > 0) {
				#ifdef DEBUG_THRESH
					UDR0 =  analyze_output >> 8;
				#endif
				recv_params[i].output_char = recv_params[i].output_char << 1;
				if(analyze_output > THRESH) {
					recv_params[i].output_char |= 1;
				}
				recv_params[i].sin_acc = recv_params[i].cos_acc = 0;
				memset(recv_params[i].resample_buffer_sin, 0, sizeof(int8_t) * recv_params[i].resample_buffer_size);
				memset(recv_params[i].resample_buffer_cos, 0, sizeof(int8_t) * recv_params[i].resample_buffer_size);
				recv_params[i].input_buffer_resample_position -= recv_params[i].boundary_alignment;
				if(--(recv_params[i].start) == 0) {
					#if !defined(DEBUG_DUMP) && !defined(DEBUG_THRESH) && !defined(DEBUG_RS_DUMP)
						UDR0 = recv_params[i].output_char;
					#endif
					recv_params[i].prev_sin_acc = recv_params[i].prev_cos_acc = recv_params[i].last_derv_sin = recv_params[i].last_derv_cos = 0;
				}
			}
		}
		recv_params[i].input_buffer_resample_position += recv_params[i].input_buffer_increment;
	}
}

void init() {
	uint8_t i;
	float float_temp;
	DDRC = 0xff;
	DDRD = _BV(PB7) | _BV(PB2) | _BV(PB1) | _BV(PB0);

	/* Setup sampling timer */
	TCCR1B = _BV(WGM12) | _BV(CS10);	// Clear on timer compare, no prescaling
	OCR1A = 500;				// Fire every 25us = 40000Hz (our sampling frequency)
	OCR1B = 480;
	TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);		// Enable output compare A interrupt

	/* Setup AD converter */
	ADMUX = _BV(REFS0) | _BV(ADLAR);
	ADCSRA = _BV(ADEN) | _BV(ADSC) | 6;

	/* Setup UART */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);	// Enable receive and transmit
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);		// Set 8-bit frames
	#if defined(DEBUG_DUMP) || defined(DEBUG_THRESH) || defined(DEBUG_RS_DUMP)
		UCSR0A = _BV(U2X0);
		UBRR0 = 10;		// Set baud rate to 230400
	#else
		UBRR0 = 520;	// Set baud rate to 2400
	#endif

	memset(recv_params, 0, sizeof(struct recv_param_t) * NUM_FREQS);
	memset(resample_buffer_sin, 0, sizeof(int8_t) * RESAMPLE_BUFFER_MAX_SIZE);
	memset(resample_buffer_cos, 0, sizeof(int8_t) * RESAMPLE_BUFFER_MAX_SIZE);
	for(i=0; i<NUM_FREQS; ++i) {
		recv_params[i].resample_buffer_sin = resample_buffer_sin + resample_buffer_next_free;
		recv_params[i].resample_buffer_cos = resample_buffer_cos + resample_buffer_next_free;
		float_temp = 40000. / freqs[i];
		recv_params[i].input_buffer_increment = float2fix(float_temp);
		recv_params[i].input_buffer_quarterphase = float2fix(float_temp / 4.);
		recv_params[i].resample_buffer_size = ceil(64. / float_temp);
		resample_buffer_next_free += recv_params[i].resample_buffer_size;
		recv_params[i].boundary_alignment = float2fix((float_temp * recv_params[i].resample_buffer_size) - 64.);
	}

	/*recv_params[0].resample_buffer_sin = resample_buffer_sin_4000;
	recv_params[0].resample_buffer_cos = resample_buffer_cos_4000;
	recv_params[0].resample_buffer_size = 7;
	recv_params[0].input_buffer_increment = int2fix(10);
	recv_params[0].input_buffer_quarterphase = float2fix(2.5);
	recv_params[0].boundary_alignment = int2fix(6);


	recv_params[1].resample_buffer_sin = resample_buffer_sin_6000;
	recv_params[1].resample_buffer_cos = resample_buffer_cos_6000;
	recv_params[1].resample_buffer_size = 10;
	recv_params[1].input_buffer_increment = float2fix(6.66666);
	recv_params[1].input_buffer_quarterphase = float2fix(1.66666);
	recv_params[1].boundary_alignment = float2fix(2.6666);*/



	sei();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
}
