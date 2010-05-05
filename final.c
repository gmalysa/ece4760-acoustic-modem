/**********************************************
**********	      Acoustic Modem 	 **********
**********             by		     **********
**********    Greg Malysa (gjm76)    **********
********** Arseney Romanenko (asr96) **********
**********    ECE4760 Spring 2010    **********
***********************************************/

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
typedef uint16_t fix;
#define int2fix(a) (((uint16_t)(a))<<10)
#define fix2int(a) ((uint8_t)((a)>>10))
#define float2fix(a) ((uint16_t)((a)*1024.0))
#define fix2float(a) ((float)(a)/1024.0)

//10:6 fixed point math defines
typedef uint16_t lfix;
#define int2lfix(a) (((uint16_t)(a))<<6)
#define lfix2int(a) ((uint16_t)((a)>>6))
#define fix2lfix(a) ((uint16_t)((a)>>4))
#define float2lfix(a) ((uint16_t)((a)*64.0))

// Buffer status defines
// These numbers were chosen so that incrementing the statuses will naturally move from idle to start to data to stop
#define IDLE -2
#define START_BIT -1
#define STOP_BIT 8

// Decoding defines
#define IN_BUFFER_SIZE 64

// Send variables
uint8_t output_sample_num = 0;		// Stores which sample of the current waveform we're outputting (0-63)
uint8_t output_buffer[8] = {0,0,0,0,0,0,0,0};			// Stores the data for our 8 channels
volatile int8_t output_buffer_status[8] = {IDLE,IDLE,IDLE,IDLE,IDLE,IDLE,IDLE,IDLE};	// Stores the state for the corresponding channels
uint8_t next_buffer = 0;
uint8_t output_bitpattern = 0;		// The frequency components we are outputting are stored here, with MSB being the highest freq.
uint8_t bitmasks[8] = {0b00000001, 0b00000010, 0b00000100, 0b00001000, 0b00010000, 0b00100000, 0b01000000, 0b10000000};

// Debug variables
//#define DEBUG_DUMP
#if defined(DEBUG_DUMP)
	int8_t debugCache[1024];
	volatile uint16_t debugCache_p = 0;
	volatile uint8_t debug_enabled = 1;
	uint16_t curr_debug_output;
	volatile uint16_t offset_timer = 0;
#endif
#ifdef DEBUG_DUMP
	#warning Compiling in Debug DUMP mode!
#endif

// Receive variables
#define NUM_FREQS 3									// This number must be less than or equal to the size of the freqs[] array
#define RESAMPLE_BUFFER_MAX_SIZE 1024				// If there are many frequencies, this number may need to increase
													// or else we will have buffer overflows
float freqs[] = {7200., 6000., 4800.};				// Define which frequencies we use here
int32_t thresholds[] = {25000, 19000, 21000};		// These thresholds correspond to the frequencies above

struct recv_param_t {	// This struct stores all of the relevant parameters for a frequency
	int8_t *resample_buffer_sin;					// The phase component with 0 offset
	int8_t *resample_buffer_cos;					// The phase component with 90 offset
	int8_t *resample_buffer_sin_shift;				// The phase component with 45 offset
	int8_t *resample_buffer_cos_shift;				// The phase component with 135 offset
	uint8_t resample_buffer_position;				// The current resample buffer position indicates which resample we must delete
	lfix frame_position;							// This variable measures the offset from the start bit in 40khz samples
	lfix prev_frame_position;						// This sets is a flag variable for the start bit or the position where a 
													// non-start-bit peak crosses above the threshold
	int16_t sin_acc;
	int16_t cos_acc;
	int16_t sin_shift_acc;
	int16_t cos_shift_acc;
	fix input_buffer_resample_position;
	uint8_t resample_buffer_size;					// Stores enough resamples to cover an entire frame (64 normal samples)
	fix input_buffer_increment;						// Resampling period
	fix input_buffer_1_8phase;						// How much we must increment the resampling period to get the sin_shift component
	fix input_buffer_1_4phase;						// '' '' cos component
	fix input_buffer_3_8phase;						// '' '' cos_shift component
	fix boundary_alignment;							// The resample buffers store more than an entire frame. This variable indicates how 
													// far back we must go for each buffer to line up with the frames again
	uint8_t start;									// This flag is 1 when we have seen a start bit and have not yet examined all data bits. Otherwise, 0
	uint8_t output_char;							// This character is built up while we detect peaks.
	int32_t thresh;									// The threshold is set to the same threshold as in the thresholds[] array above
};
int8_t resample_buffer_pool[RESAMPLE_BUFFER_MAX_SIZE];		// We allocate all of the resample buffers in this array.
uint16_t resample_buffer_next_free = 0;						// This is how we fake malloc.

struct recv_param_t recv_params[NUM_FREQS];		// This array stores the parameters for the frequencies
volatile int8_t input_buffer[IN_BUFFER_SIZE];	// The global circular input buffer
uint8_t input_buffer_pos = 0;					// The ISR uses this variable to keep track of the input circular buffer position
volatile uint8_t public_input_buffer_position = 0;	// The ISR changes this variable only after it captures a new sample

// Function prototypes
void init();
void find_freq(struct recv_param_t*);
void clear(struct recv_param_t*);

ISR(TIMER1_COMPA_vect) {
	uint8_t i;
	PORTD &= ~_BV(PB7);		// Drive the DAC's WR pin low
	PORTC = pgm_read_byte(freqTable[output_bitpattern] + output_sample_num);	// Lookup the waveform and output to the DAC on PORT C
	output_sample_num = (output_sample_num + 1) & 0x3f;
	PORTD |= _BV(PB7);										// Drive the DAC's WR pin high 
	input_buffer[input_buffer_pos] = ADCH - 0x7E;			// Take an ADC measurement
	ADCSRA |= _BV(ADSC);									// Start a new ADC measurement
	public_input_buffer_position = input_buffer_pos;
	#if defined(DEBUG_DUMP)		// If we're in debug mode, continually capture input samples to the debug cache.
		if(debug_enabled) {
			debugCache[debugCache_p] = input_buffer[input_buffer_pos];
			debugCache_p = (debugCache_p + 1) & 0x3ff;
			if(offset_timer > 0) {		// We've seen a start bit, so keep capturing the samples until the main loop is satisfied
				++offset_timer;
			}
		}
	#endif
	input_buffer_pos = (input_buffer_pos+1) & (IN_BUFFER_SIZE - 1);
	if(output_sample_num == 0) {	// We only need to look at the channels every 64th iteration, when we need to figure out
									// what waveform we're outputting next
		output_bitpattern = 0;
		for(i=0; i<8; ++i) {		// This is the main state machine for the channels' statuses
			if(output_buffer_status[i] == IDLE) {continue;}
			else if(output_buffer_status[i] == START_BIT) {
				output_bitpattern |= bitmasks[i];
				PORTD &= ~_BV(PB2);		// Turn on transmit LED
			}
			else if(output_buffer_status[i] == STOP_BIT) {
				output_buffer_status[i] = IDLE;
				PORTD |= _BV(PB2);		// Turn off transmit LED
				continue;
			}
			else if (output_buffer[i] & 0x80) {		// Output data bits
				output_bitpattern |= bitmasks[i];
			}
			if (output_buffer_status[i] != START_BIT) {
				output_buffer[i] = output_buffer[i] << 1;
			}
			++output_buffer_status[i];		// Advance status
		}
	}
}

ISR(TIMER1_COMPB_vect, ISR_NAKED) {		// This ISR ensures consistent entry into the main ISR and thus consistent timing
	sei();
	sleep_cpu();
	reti();
}

int main() {
	uint8_t i = 0;
	init();
	next_buffer = 0;
	while(1) {
		#if defined(DEBUG_DUMP)
			if(offset_timer >= 700) {		// We've recorded all the samples we need to dump
				debug_enabled = 0;			// Turn off sample recording for a little bit
				curr_debug_output = debugCache_p+1;
				while(curr_debug_output != debugCache_p) {		// Blast samples out serial as fast as possible
					while(!(UCSR0A & _BV(UDRE0))) {;}	
					UDR0 = debugCache[curr_debug_output];
					curr_debug_output = (curr_debug_output+1) & 0x3ff;
				}
				debug_enabled = 1;
				offset_timer = 0;
			}
		#endif
		if(UCSR0A & _BV(RXC0)) {	// We've received a serial byte
			while(output_buffer_status[next_buffer] != IDLE) {;} 	// Stall if no channels are IDLE
			output_buffer[next_buffer] = UDR0;
			output_buffer_status[next_buffer] = START_BIT;	// Let the ISR do its thing
			if (next_buffer == 0)		// Schedule the channels in round-robin fashion
				next_buffer = 1;
			else if (next_buffer == 1)
				next_buffer = 2;
			else
				next_buffer = 0;
		}
		for(i=0; i<NUM_FREQS; ++i) {		// Cycle through all detect frequencies
			find_freq(recv_params + i);
		}
	}
}

void find_freq(struct recv_param_t* this_param) {
	int8_t input_sample = 0;
	int32_t sum_squares = 0, shift_sum_squares = 0, magnitude = 0;
	lfix pulse_width = 0, i = 0;
	uint8_t x = fix2int(this_param->input_buffer_resample_position + this_param->input_buffer_3_8phase); // This is the latest sample we need
	
	// We want the resample pointer to be slightly before the ISR's pointer or really far ahead (i.e. the ISR's pointer wrapped around)
	if((x <= public_input_buffer_position && public_input_buffer_position - x <= 20) || 
	   (x > public_input_buffer_position && x - public_input_buffer_position > 20)) {

		// Sine component calculations
		this_param->sin_acc -= this_param->resample_buffer_sin[this_param->resample_buffer_position];
		input_sample = input_buffer[fix2int(this_param->input_buffer_resample_position)];
		this_param->sin_acc += input_sample;
		this_param->resample_buffer_sin[this_param->resample_buffer_position] = input_sample;

		// Cosine (90 offset) component calculations
		this_param->cos_acc -= this_param->resample_buffer_cos[this_param->resample_buffer_position];
		input_sample = input_buffer[fix2int(this_param->input_buffer_resample_position + this_param->input_buffer_1_4phase)];
		this_param->cos_acc += input_sample;
		this_param->resample_buffer_cos[this_param->resample_buffer_position] = input_sample;
		
		// 45 offset component calculations
		this_param->sin_shift_acc -= this_param->resample_buffer_sin_shift[this_param->resample_buffer_position];
		input_sample = input_buffer[fix2int(this_param->input_buffer_resample_position + this_param->input_buffer_1_8phase)];
		this_param->sin_shift_acc += input_sample;
		this_param->resample_buffer_sin_shift[this_param->resample_buffer_position] = input_sample;

		// 135 offset component calculations
		this_param->cos_shift_acc -= this_param->resample_buffer_cos_shift[this_param->resample_buffer_position];
		input_sample = input_buffer[x];
		this_param->cos_shift_acc += input_sample;
		this_param->resample_buffer_cos_shift[this_param->resample_buffer_position] = input_sample;

		// Calculate the magnitude by averaging the sum of squares
		// We must be careful to cast to int32_t to avoid truncation
		sum_squares = ((int32_t)this_param->sin_acc * (int32_t)this_param->sin_acc) + ((int32_t)this_param->cos_acc * (int32_t)this_param->cos_acc);
		shift_sum_squares = ((int32_t)this_param->sin_shift_acc * (int32_t)this_param->sin_shift_acc) + ((int32_t)this_param->cos_shift_acc * (int32_t)this_param->cos_shift_acc);
		magnitude = (shift_sum_squares >> 1) + (sum_squares >> 1);
		
		// We're looking for a start bit
		if(this_param->start == 0) {
			if(magnitude > this_param->thresh) {	// 
				if(this_param->prev_frame_position == 0) {	// We haven't crossed above the threshold before
					this_param->prev_frame_position = 1;
					this_param->frame_position = fix2lfix(this_param->input_buffer_increment);
				}
				else {
					this_param->frame_position += fix2lfix(this_param->input_buffer_increment); // We're above the threshold, so remember how long we've been there
				}
				if(this_param->prev_frame_position) { 					// We're currently above the threshold
					if(this_param->frame_position > int2lfix(45)) { 	// We've been above the threshold for a long time, so trigger the start bit
						#if defined(DEBUG_DUMP)		
							if(offset_timer == 0) {						// Start recording samples for debug dump purposes
								offset_timer = 1;
							}
						#endif
						this_param->start = 1;
						this_param->output_char = 0;
						this_param->input_buffer_resample_position += this_param->input_buffer_increment;
						this_param->frame_position = 0;
						this_param->prev_frame_position = 0;
						clear(this_param);
						PORTD &= ~_BV(PB3);	// Turn on receive LED
						return;						
					}
				}
			}
			else {	// Below the threshold
				if(this_param->prev_frame_position) { 	// We were just above the threshold
					if(this_param->frame_position > int2lfix(40)) {	// We were above the threshold for long enough, so trigger the start bit
						#if defined(DEBUG_DUMP)
							if(offset_timer == 0) {
								offset_timer = 1;
							}
						#endif
						this_param->start = 1;
						this_param->output_char = 0;
						this_param->input_buffer_resample_position += this_param->input_buffer_increment;
						this_param->frame_position = 0;
						this_param->prev_frame_position = 0;
						clear(this_param);
						PORTD &= ~_BV(PB3);
						return;						
					}
					else {	// We didn't spend enough time above the threshold; false alarm, reset
						this_param->prev_frame_position = 0;
					}
				}
			}
		}

		// We've seen a start bit
		if(this_param->start) {
			if(this_param->prev_frame_position == 0) { 
				if(magnitude > this_param->thresh) { // This is the first time we're going above the threshold
					this_param->prev_frame_position = this_param->frame_position; 	// Remember where we were when we went above
				}
			}
			else {
				if(magnitude < this_param->thresh) { // We've crossed above the threshold previously and are now going down
					pulse_width = this_param->frame_position - this_param->prev_frame_position;
					if(pulse_width <= int2lfix(10)) { 	// False alarm
						this_param->prev_frame_position = 0;
					}
					else if(pulse_width > int2lfix(64)) { // We've detected more than one pulse
						for(i = this_param->prev_frame_position + int2lfix(32); i < this_param->frame_position; i += int2lfix(64)) {
							this_param->output_char |= (0x80 >> lfix2int(i >> 6));	// Or all the bits
						}
						this_param->prev_frame_position = 0;	// Get ready for next pulse
					}
					else {	// We got a single pulse
						this_param->output_char |= (0x80 >> lfix2int((this_param->prev_frame_position + (pulse_width>>1)) >> 6));
						this_param->prev_frame_position = 0;
					}
				}
			}
			this_param->frame_position += fix2lfix(this_param->input_buffer_increment);
			if(this_param->frame_position >= int2lfix(640)) {	// We've seen all data bits
				#if !defined(DEBUG_DUMP)
					UDR0 = this_param->output_char;		// If we're not debugging, output final character
				#endif
				PORTD |= _BV(PB3);	// Turn off receive LED
				this_param->start = 0;
				this_param->frame_position = 0;
				this_param->prev_frame_position = 0;
				clear(this_param);
			}
		}

		// Resample buffer wraparound. This happens once per frame, so we can take advantage of this and realign to the frame boundary
		if(this_param->resample_buffer_position++ >= (this_param->resample_buffer_size - 1)) {
			this_param->resample_buffer_position = 0;
			if(this_param->start) {
				// Realign everything
				this_param->frame_position = this_param->frame_position - fix2lfix(this_param->boundary_alignment) + fix2lfix(this_param->input_buffer_increment >> 1);
				if(this_param->prev_frame_position) {
					this_param->prev_frame_position = this_param->prev_frame_position - fix2lfix(this_param->boundary_alignment) + fix2lfix(this_param->input_buffer_increment >> 1);	
				}
				this_param->input_buffer_resample_position = this_param->input_buffer_resample_position - this_param->boundary_alignment + (this_param->input_buffer_increment >> 1);
			}
		}
		
		this_param->input_buffer_resample_position += this_param->input_buffer_increment;
	}
	
}

// Clears the accumulators and their respective buffers
void clear(struct recv_param_t *this_param) {
	this_param->sin_acc = this_param->cos_acc = this_param->sin_shift_acc = this_param->cos_shift_acc = 0;
	this_param->resample_buffer_position = 0;
	memset(this_param->resample_buffer_sin, 0, sizeof(int8_t) * this_param->resample_buffer_size);
	memset(this_param->resample_buffer_cos, 0, sizeof(int8_t) * this_param->resample_buffer_size);
	memset(this_param->resample_buffer_sin_shift, 0, sizeof(int8_t) * this_param->resample_buffer_size);
	memset(this_param->resample_buffer_cos_shift, 0, sizeof(int8_t) * this_param->resample_buffer_size);
}

void init() {
	uint8_t i;
	float float_temp;
	DDRC = 0xff;		// DAC data output
	DDRD = _BV(PB7) | _BV(PB3) | _BV(PB2) | _BV(PB1) | _BV(PB0);	// DAC WR | Rx LED | Tx LED | Serial Rx | Serial Tx
	PORTD |= _BV(PB2) | _BV(PB3);		// Turned LEDs off to start

	/* Setup sampling timer */
	TCCR1B = _BV(WGM12) | _BV(CS10);	// Clear on timer compare, no prescaling
	OCR1A = 500;				// Fire every 25us = 40000Hz (our sampling frequency)
	OCR1B = 480;				// Fire 20 cycles before to go into sleep mode
	TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);		// Enable output compare A and output compare B interrupts

	/* Setup AD converter */
	ADMUX = _BV(REFS0) | _BV(ADLAR);		// Use VCC as reference voltage | left align value
	ADCSRA = _BV(ADEN) | _BV(ADSC) | 6;		// Enable | start conversion | set prescalar

	/* Setup UART */
	UCSR0B = _BV(RXEN0) | _BV(TXEN0);	// Enable receive and transmit
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);		// Set 8-bit frames
	#if defined(DEBUG_DUMP)
		UCSR0A = _BV(U2X0);
		UBRR0 = 10;		// Set baud rate to 230400 for debug blast
	#else
		UBRR0 = 520;	// Set baud rate to 2400 so we have less of a chance of saturating input channels
	#endif

	memset((int8_t*)input_buffer, 0, sizeof(int8_t) * IN_BUFFER_SIZE);
	memset(recv_params, 0, sizeof(struct recv_param_t) * NUM_FREQS);
	memset(resample_buffer_pool, 0, sizeof(int8_t) * RESAMPLE_BUFFER_MAX_SIZE);
	for(i=0; i<NUM_FREQS; ++i) {
		float_temp = 40000. / freqs[i];
		recv_params[i].input_buffer_increment = float2fix(float_temp);
		recv_params[i].input_buffer_1_4phase = float2fix(float_temp / 4.);
		recv_params[i].input_buffer_1_8phase = float2fix(float_temp / 8.);
		recv_params[i].input_buffer_3_8phase = float2fix((float_temp * 3.) / 8.);
		recv_params[i].resample_buffer_size = (uint8_t)ceil(64. / float_temp);
		recv_params[i].resample_buffer_sin = resample_buffer_pool + resample_buffer_next_free;		// Malloc faking
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
	set_sleep_mode(SLEEP_MODE_IDLE);		// Sleep mode for the ISR timings
	sleep_enable();
}
