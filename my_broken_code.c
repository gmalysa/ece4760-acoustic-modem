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
#define int2lfix(a) (((uint16_t)(a))<<6)
#define lfix2int(a) ((uint16_t)((a)>>6))
#define fix2lfix(a) ((uint16_t)((a)>>4))

// Buffer status defines
#define IDLE -2
#define START_BIT -1
#define STOP_BIT 8

// Decoding defines
#define IN_BUFFER_SIZE 64

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

#ifdef DEBUG_DUMP
	#warning Compiling in Debug DUMP mode!
#endif

#ifdef DEBUG_RS_DUMP
	#warning Compiling in Debug RESAMPLE BUFFER DUMP mode!
#endif

// Receive variables
#define NUM_FREQS 2
#define RESAMPLE_BUFFER_MAX_SIZE 1024
float freqs[] = {4000., 6000., 7000.};
int32_t thresholds[] = {10000, 15000, 35000};
struct recv_param_t {
	int8_t *resample_buffer_sin;
	int8_t *resample_buffer_cos;
	int8_t *resample_buffer_sin_shift;
	int8_t *resample_buffer_cos_shift;
	uint8_t resample_buffer_position;
	uint16_t frame_position;
	uint16_t prev_frame_position;
	int16_t sin_acc;
	int16_t cos_acc;
	int16_t sin_shift_acc;
	int16_t cos_shift_acc;
//	int16_t prev_sin_acc;
//	int16_t prev_cos_acc;
//	int16_t last_derv_sin;
//	int16_t last_derv_cos;
	int32_t prev_mag;
	uint16_t input_buffer_resample_position;
	uint8_t resample_buffer_size;
	uint16_t input_buffer_increment;
	uint16_t input_buffer_1_8phase;
	uint16_t input_buffer_1_4phase;
	uint16_t input_buffer_3_8phase;
//	uint16_t boundary_alignment;
	uint8_t start;
	uint8_t output_char;
	int32_t thresh;
//	uint8_t last_pibp;	//public_input_buffer_position
};
int8_t resample_buffer_pool[RESAMPLE_BUFFER_MAX_SIZE];
uint16_t resample_buffer_next_free = 0;

struct recv_param_t recv_params[NUM_FREQS];
volatile int8_t input_buffer[IN_BUFFER_SIZE];
uint8_t input_buffer_pos = 0;
volatile uint8_t public_input_buffer_position = 0;
void init();
void find_freq(struct recv_param_t*);
void clear();

ISR(TIMER1_COMPA_vect) {
	uint8_t i;
	PORTD &= ~_BV(PB7);
	PORTC = freqCache[output_sample_num];
	output_sample_num = (output_sample_num + 1) & 0x3f;
	PORTD |= _BV(PB7);	
	input_buffer[input_buffer_pos] = ADCH - 0x7E;
	ADCSRA |= _BV(ADSC);
	public_input_buffer_position = input_buffer_pos;
	#if defined(DEBUG_DUMP)
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
				PORTD &= ~_BV(PB2);
			}
			else if(output_buffer_status[i] == STOP_BIT) {
				output_buffer_status[i] = IDLE;
				PORTD |= _BV(PB2);
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
	sei();
	sleep_cpu();
	reti();
}

int main() {
	uint8_t i = 0;
	init();
	next_buffer = 2;
	while(1) {
		#if defined(DEBUG_DUMP) || defined(DEBUG_RS_DUMP)
			#if defined(DEBUG_DUMP)
				if(offset_timer >= 700) {
			#elif defined(DEBUG_RS_DUMP)
				if(offset_timer >= 500) {
			#endif
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
			if (next_buffer == 0)
				next_buffer = 1;
//			else if (next_buffer == 1)
//				next_buffer = 2;
			else
				next_buffer = 0;
			//next_buffer = (next_buffer + 1) & 0x1;
		}
		for(i=0; i<NUM_FREQS; ++i) {
			find_freq(recv_params + i);
		}
	}
}

void find_freq(struct recv_param_t* this_param) {
	int8_t input_sample = 0;
	int32_t sin_squared=0, cos_squared=0, sum_squares = 0, shift_sum_squares = 0, analyze_output = 0;
	uint8_t x = fix2int(this_param->input_buffer_resample_position + this_param->input_buffer_3_8phase);
	
	if((x <= public_input_buffer_position && public_input_buffer_position - x <= 20) ||
	   (x > public_input_buffer_position && x - public_input_buffer_position > 20)) {
		// Sine component calculations
		this_param->sin_acc -= this_param->resample_buffer_sin[this_param->resample_buffer_position];
		input_sample = input_buffer[fix2int(this_param->input_buffer_resample_position)];
		this_param->sin_acc += input_sample;
		this_param->resample_buffer_sin[this_param->resample_buffer_position] = input_sample;

		#if defined(DEBUG_RS_DUMP)
			if(debug_enabled) {
				debugCache[debugCache_p] = input_sample;
				debugCache_p = (debugCache_p + 1) & 0x3ff;
				if(offset_timer > 0) {
					++offset_timer;
				}
				debugCache[debugCache_p] = this_param->sin_acc >> 2;
				debugCache_p = (debugCache_p + 1) & 0x3ff;
				if(offset_timer > 0) {
					++offset_timer;
				}
				/*debugCache[debugCache_p] = this_param->sin_acc;
				debugCache_p = (debugCache_p + 1) & 0x3ff;
				if(offset_timer > 0) {
					++offset_timer;
				}*/
			}
		#endif

		// Cosine component calculations
		this_param->cos_acc -= this_param->resample_buffer_cos[this_param->resample_buffer_position];
		input_sample = input_buffer[fix2int(this_param->input_buffer_resample_position + this_param->input_buffer_1_4phase)];
		this_param->cos_acc += input_sample;
		this_param->resample_buffer_cos[this_param->resample_buffer_position] = input_sample;
		
		this_param->sin_shift_acc -= this_param->resample_buffer_sin_shift[this_param->resample_buffer_position];
		input_sample = input_buffer[fix2int(this_param->input_buffer_resample_position + this_param->input_buffer_1_8phase)];
		this_param->sin_shift_acc += input_sample;
		this_param->resample_buffer_sin_shift[this_param->resample_buffer_position] = input_sample;

		this_param->cos_shift_acc -= this_param->resample_buffer_cos_shift[this_param->resample_buffer_position];
		input_sample = input_buffer[x];
		this_param->cos_shift_acc += input_sample;
		this_param->resample_buffer_cos_shift[this_param->resample_buffer_position] = input_sample;

		#if defined(DEBUG_RS_DUMP)
			/*if(debug_enabled) {
				debugCache[debugCache_p] = input_sample;
				debugCache_p = (debugCache_p + 1) & 0x3ff;
				if(offset_timer > 0) {
					++offset_timer;
				}
				debugCache[debugCache_p] = this_param->cos_acc >> 8;
				debugCache_p = (debugCache_p + 1) & 0x3ff;
				if(offset_timer > 0) {
					++offset_timer;
				}
				debugCache[debugCache_p] = this_param->cos_acc;
				debugCache_p = (debugCache_p + 1) & 0x3ff;
				if(offset_timer > 0) {
					++offset_timer;
				}
			}*/
		#endif

		// Thresholding if we're at the end of a time division or searching for a start bit
		//if(this_param->start == 0 || (this_param->resample_buffer_position >= (this_param->resample_buffer_size - 1))) {
			sin_squared = ((int32_t)this_param->sin_acc * (int32_t)this_param->sin_acc);
			cos_squared = ((int32_t)this_param->cos_acc * (int32_t)this_param->cos_acc);
			sum_squares = sin_squared + cos_squared;
			shift_sum_squares = ((int32_t)this_param->sin_shift_acc * (int32_t)this_param->sin_shift_acc) + ((int32_t)this_param->cos_shift_acc * (int32_t)this_param->cos_shift_acc);
			analyze_output = (sum_squares > shift_sum_squares ? sum_squares : shift_sum_squares);
		//}	
		
		if(analyze_output > this_param->thresh && this_param->start == 0) {
			if ((analyze_output - this_param->prev_mag) < (int32_t)-1000) {
				#if defined(DEBUG_DUMP) || defined(DEBUG_RS_DUMP)	//maybe TODO: fix debug
					if(offset_timer == 0) {
						offset_timer = 1;
					}
				#endif
				this_param->start = 1;
				this_param->output_char = 0;
				this_param->input_buffer_resample_position += (this_param->input_buffer_increment >> 1);
				this_param->frame_position = 0;
				this_param->prev_frame_position = 0;
				clear();
//				this_param->input_buffer_resample_position -= (this_param->input_buffer_increment - this_param->boundary_alignment);
				PORTD &= ~_BV(PB3);
				return;
			}
		}
		
		this_param->prev_mag = analyze_output;

		if(this_param->start) {
			if(this_param->prev_frame_position == 0) { 
				if(analyze_output > this_param->thresh) { // This is the first time we're going above the threshold
					this_param->prev_frame_position = this_param->frame_position;
				}
			}
			else {
				if(analyze_output < this_param->thresh) { // We've crossed above the threshold previously and are now going down
					if(this_param->frame_position - this_param->prev_frame_position < int2lfix(3)) { 	// False alarm
						this_param->prev_frame_position = 0;
					}
					else {	// We got a pulse
						clear();
						output_char |= output_bitmask[7-lfix2int((this_param->prev_frame_position + ((this_param->frame_position - this_param->prev_frame_position)>>1)) >> 6)];
						this_param->prev_frame_position = 0;
					}
				}
			}
		}

		
		
		// if(this_param->resample_buffer_position++ >= (this_param->resample_buffer_size - 1)) {
			// this_param->resample_buffer_position = 0;
			// if (this_param->start > 0) {
				// #ifdef DEBUG_THRESH
					// UDR0 =  analyze_output >> 8;
				// #endif
				// this_param->output_char = this_param->output_char << 1;
				// if(analyze_output > this_param->thresh) {
					// this_param->output_char |= 1;
				// }
				// this_param->sin_acc = this_param->cos_acc = this_param->sin_shift_acc = this_param->cos_shift_acc = 0;

				// memset(this_param->resample_buffer_sin, 0, sizeof(int8_t) * this_param->resample_buffer_size);
				// memset(this_param->resample_buffer_cos, 0, sizeof(int8_t) * this_param->resample_buffer_size);
				// memset(this_param->resample_buffer_sin_shift, 0, sizeof(int8_t) * this_param->resample_buffer_size);
				// memset(this_param->resample_buffer_cos_shift, 0, sizeof(int8_t) * this_param->resample_buffer_size);
				// if(this_param->start != 9) 
					// this_param->input_buffer_resample_position -= this_param->boundary_alignment;
				// if(--(this_param->start) == 0) {
					// #if !defined(DEBUG_DUMP) && !defined(DEBUG_THRESH) && !defined(DEBUG_RS_DUMP)
						// UDR0 = this_param->output_char;
					// #endif
					// PORTD |= _BV(PB3);
				// }
			// }
		// }
		
		this_param->input_buffer_resample_position += this_param->input_buffer_increment;
		this_param->frame_position += (this_param->input_buffer_increment >> 4);
		if(this_param->frame_position >= 512) {
			UDR0 = this_param->output_char;
			PORTD |= _BV(PB3);
			this_param->start = 0;
			clear();
		}
	}
	
}

void clear() {
	sin_acc = cos_acc = sin_shift_acc = cos_shift_acc = 0;
	memset(sin_acc, 0, this_param->resample_buffer_size);
	memset(cos_acc, 0, this_param->resample_buffer_size);
	memset(sin_shift_acc, 0, this_param->resample_buffer_size);
	memset(cos_shift_acc, 0, this_param->resample_buffer_size);
}

void init() {
	uint8_t i;
	float float_temp;
	DDRC = 0xff;
	DDRD = _BV(PB7) | _BV(PB3) | _BV(PB2) | _BV(PB1) | _BV(PB0);
	PORTD |= _BV(PB2) | _BV(PB3);

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

	memset(input_buffer, 0, sizeof(int8_t) * IN_BUFFER_SIZE);
	memset(recv_params, 0, sizeof(struct recv_param_t) * NUM_FREQS);
	memset(resample_buffer_pool, 0, sizeof(int8_t) * RESAMPLE_BUFFER_MAX_SIZE);
	for(i=0; i<NUM_FREQS; ++i) {
		float_temp = 40000. / freqs[i];
		recv_params[i].input_buffer_increment = float2fix(float_temp);
		recv_params[i].input_buffer_1_4phase = float2fix(float_temp / 4.);
		recv_params[i].input_buffer_1_8phase = float2fix(float_temp / 8.);
		recv_params[i].input_buffer_3_8phase = float2fix((float_temp * 3.) / 8.);
		recv_params[i].resample_buffer_size = (uint8_t)ceil(64. / float_temp);
		recv_params[i].resample_buffer_sin = resample_buffer_pool + resample_buffer_next_free;
		resample_buffer_next_free += recv_params[i].resample_buffer_size;
		recv_params[i].resample_buffer_cos = resample_buffer_pool + resample_buffer_next_free;
		resample_buffer_next_free += recv_params[i].resample_buffer_size;
		recv_params[i].resample_buffer_sin_shift = resample_buffer_pool + resample_buffer_next_free;
		resample_buffer_next_free += recv_params[i].resample_buffer_size;
		recv_params[i].resample_buffer_cos_shift = resample_buffer_pool + resample_buffer_next_free;
		resample_buffer_next_free += recv_params[i].resample_buffer_size;
		recv_params[i].boundary_alignment = float2fix((float_temp * (float)(recv_params[i].resample_buffer_size)) - 64.);
		recv_params[i].thresh = thresholds[i];
	}

	sei();
	set_sleep_mode(SLEEP_MODE_IDLE);
	sleep_enable();
}
