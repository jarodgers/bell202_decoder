#define F_CPU 16000000UL

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/cpufunc.h>
#include "uart.h"

#define BUFFER_SIZE 200
#define LOW_TONE 1
#define HIGH_TONE 0

volatile uint8_t signal_incoming = 0; // true when a signal is in progress, false otherwise
volatile uint8_t current_tone = HIGH_TONE; // instantaneous tone
volatile uint8_t prev_tone = HIGH_TONE; // tone from previous bit period
volatile uint8_t current_bit = 0; // the most recent bit read
uint8_t elapsed_time; // elapsed time between zero crossings
volatile uint8_t bit_period_overflow = 0; // for "flowchart" algorithm, used for seeing if it's time to write a bit or not
unsigned char buffer[BUFFER_SIZE] = {0}; // buffer for packet payload
volatile unsigned char last_8_bits = 0x00; // bit field that will hold last 8 bits received in the bit stream
volatile unsigned char byte_buffer = 0x00; // buffer to hold a single byte, gets written to the main buffer once fully allocated
uint8_t start_flag = 0x7e; // start flag constant, binary is 01111110
volatile uint8_t start_flag_found = 0; // true if start flag has been found
volatile uint8_t five_ones_found = 0; // used for bit stuffing; if five or more ones are present in data, and a zero is found right after, ignore it
volatile uint16_t crc_receive_index = 0; // keep track of how far we are into receiving crc, stop when it hits 16
volatile uint16_t bytes_read = 0; // number of bytes read in data stream
uint8_t tone_differentiator = 74;   // this will be the value that the elapsed time is compared to when determining the tone;
                                    // less than this is high tone (2200 Hz), greater than this is the low tone (1200 Hz)
uint16_t poly = 0x8408; // constant for use with CRC calculation
volatile uint16_t crc;
volatile uint8_t bit_index = 0; // bit position for incoming data to be written to buffer, starts from LSB, mod 8 when incremented
volatile uint16_t byte_index = 0; // index for putting bytes into the main

ISR(TIMER1_COMPA_vect) {
    bit_period_overflow = 1;
    TCCR1B &= 0xf8; // stop timer 1
    TCNT1 = 0; // reset timer 1 value to zero
}

ISR(INT0_vect) {
    signal_incoming = 1;
    PORTC |= 0x01; // DEBUG: data stream start detected

    elapsed_time = TCNT0; // elapsed time is the TMR0 count
    TCNT0 = 0; // reset timer 0
    TCCR0B |= (1 << CS00) | (1 << CS01); // start TMR0 with prescaler of 64

    if (elapsed_time < tone_differentiator) {
        PORTC |= 0x02; // DEBUG: tone flag high
        current_tone = HIGH_TONE;
    }
    else {
        PORTC &= ~(0x02); // DEBUG: tone flag low
        current_tone = LOW_TONE;
    }
}

ISR(TIMER0_OVF_vect) {
    // stop and reset TMR0
    TCCR0B &= 0xf8;
    TCNT0 = 0;

    // stop and reset TMR1
    TCCR1B &= 0xf8;
    TCNT1 = 0;

    // set signal_incoming to false
    signal_incoming = 0;
    PORTC &= ~(0x01); // DEBUG: data stream end detected
}

int main(void) {
    DDRB |= 0x01; // PB0 is output
    DDRC |= 0x01; // DEBUG: PC0 is output, represents data stream start and end
    DDRC |= 0x02; // DEBUG: PC1 is output, represents tone flag
    PORTC |= 0x02; // DEBUG: PC1 initially high (tone flag high)
    DDRC |= 0x04; // DEBUG: PC2 is output, indicating moment when start flag is present in last 8 bits (and going low thereafter)
    DDRC |= 0x08; // DEBUG: PC3 is output, indicating whether a zero or a one is being written
    DDRC |= 0x10; // DEBUG: PC4 is output, indicating start and end of per-bit-period processing
    DDRC |= 0x20; // DEBUG: PC5, general purpose debug

    // configure main input pin and interrupt (INT0)
    DDRD &= ~(0x04); // PD2 is an input
    EICRA |= (1 << ISC00); // any logical change on INT0 generates interrupt
    EIMSK |= (1 << INT0); // enable INT0 interrupt request

    // configure TMR0
    // this timer will measure elapsed time since last zero crossing;
    // if it overflows, that indicates that the incoming signal has ended
    TIMSK0 |= (1 << TOIE0); // timer overflow interrupt

    // configure TMR1
    // this timer will keep track of the bit period;
    // will need to have a correction factor; 16 MHz / 1.2 kHz is 13333.3333,
    // so, we can make it count to 13333 for two periods, and 13334 for one period
    TCCR1B |=  (1 << WGM12); // CTC mode (clear timer when it reaches value in OCR1A)
    OCR1A = 13333; // timer value for 1200 Hz (make it 13334 every three bit periods)
    TIMSK1 |= (1 << OCIE1A); // interrupt when the timer value matches value in OCR1A

    // set up UART for transmission to computer
    DDRD |= 0x02; // PD1 is TX/output
    uart_init(UART_BAUD_SELECT(115200,F_CPU));
    uart_puts("Starting serial!\r\n");
    
    sei(); // enable interrupts

    while (1) {
        // see if stream of data has started
        if (signal_incoming) {
            crc = 0xffff; // initialize crc
            TCCR1B |= (1 << CS10); // start timer 1, which will keep track of bit periods
            while (signal_incoming) {
                // use flowchart algorithm
                if (current_tone == prev_tone) {
                    if (!bit_period_overflow) {
                        continue;
                    }
                    else {
                        TCNT1 = 0;
                        OCR1A = 13333; // set timer 1 "overflow" value to 1 bit period
                        TCCR1B |= (1 << CS10); // start timer 1, which will keep track of bit periods
                        bit_period_overflow = 0;
                        current_bit = 1;
                    }
                }
                else { // tone has changed
                    TCNT1 = 0;
                    OCR1A = 20000; // set timer 1 "overflow" value to 1.5 bit periods
                    TCCR1B |= (1 << CS10); // start timer 1, which will keep track of bit periods
                    bit_period_overflow = 0;
                    current_bit = 0;
                }

                PORTC |= 0x10; // DEBUG: byte will be written, processing begun

                prev_tone = current_tone; // save this tone

                // check if five ones in a row have been found
                if ((last_8_bits & 0xfc) == 0xf8) {
                    five_ones_found = 1;
                }
                else {
                    five_ones_found = 0;
                }

                // place new bit into the last_8_bits field
                last_8_bits >>= 1;

                if (current_bit == 0) {
                    PORTC &= ~(0x08); // DEBUG: a zero is being written
                }
                else {
                    PORTC |= 0x08; // DEBUG: a one is being written
                    last_8_bits |= 0x80;
                }

                if (last_8_bits == start_flag) {
                    start_flag_found = 1;
                    PORTC |= 0x04; // DEBUG: start flag found
                    bit_index = 0;
                    byte_buffer = 0x00;
                    crc_receive_index = 0;
                }
                else {
                    PORTC &= ~(0x04); // DEBUG: start flag combination not present
                }
                
                if (!start_flag_found && (current_bit == 1 || (current_bit == 0 && !five_ones_found))) {
                    // write current bit to byte_buffer (no need to do anything if current bit is a zero)
                    if (current_bit == 1) {
                        byte_buffer |= (1 << bit_index);
                    }

                    // set new bit index (decrementing from 7 down to zero, cyclically)
                    // write received byte to buffer when it's fully allocated (i.e. when the bit_index becomes zero)
                    if (bit_index == 7) {
                        bit_index = 0; // reset bit_index
                        buffer[byte_index] = byte_buffer; // write this byte to the main byte buffer
                        byte_buffer = 0x00;
                        bytes_read++; // increment number of bytes read
                        byte_index++; // finished writing to this byte, move on to the next one
                    }
                    else {
                        bit_index++;
                    }
                }
                
                start_flag_found = 0;
                PORTC &= ~(0x10); // DEBUG: bit writing process complete
            } // end while(signal_incoming)

            if (bytes_read > 16) {
                // write number of bytes read over serial
                char bytes_read_string[8];
                itoa(bytes_read,bytes_read_string,10);
                uart_puts(bytes_read_string);
                uart_puts(" bytes read!\r\n");

                // do crc algorithm
                uint8_t temp_bit_index, roll_out_bit, temp_bit;
                uint16_t temp_byte_index;
                unsigned char temp_byte;
                for (temp_byte_index = 1; temp_byte_index < (bytes_read - 2); temp_byte_index++) {
                    temp_byte = buffer[temp_byte_index];
                    for (temp_bit_index = 0; temp_bit_index < 8; temp_bit_index++) {
                        temp_bit = temp_byte & 0x01;
                        temp_byte >>= 1;

                        roll_out_bit = crc & 0x0001; // going to shift right, save the bit that's being shifted out
                        crc >>= 1; // shift crc right by 1
                    
                        if ((roll_out_bit ^ temp_bit) == 0x01) {
                            crc = crc ^ poly;
                        }
                    }
                }

                // after finishing processing, complement crc
                crc = ~crc;
            
                // extract the declared crc
                uint16_t declared_crc = (uint16_t)buffer[bytes_read-1];
                declared_crc <<= 8;
                declared_crc += (uint16_t)buffer[bytes_read-2];

                // place the declared and computed crc's into strings
                char declared_crc_string[8];
                char computed_crc_string[8];
                itoa(declared_crc,declared_crc_string,16);
                itoa(crc,computed_crc_string,16);
            
                // write information over serial
                uart_puts("Declared CRC: ");
                uart_puts(declared_crc_string);
                uart_puts(", computed CRC: ");
                uart_puts(computed_crc_string);
                if (crc == declared_crc) {
                    uart_puts(" . . .CRC matches!\r\n");
                }
                else {
                    uart_puts(" . . .CRC did not match! :(\r\n");
                }

                uart_puts("Decoded message: ");
                // char hex_char_str[4]; // DEBUG
                uint16_t i = 17;
                while (i < (bytes_read-2)) {
                    uart_putc(buffer[i]);
                    // uart_putc(' '); // DEBUG
                    // itoa((uint8_t)buffer[i],hex_char_str,16); // DEBUG
                    // uart_puts(hex_char_str); // DEBUG
                    i++;
                }
                uart_puts("\r\n"); 

                // zero the buffer
                uint16_t j;
                for (j = 0; j < BUFFER_SIZE; j++) {
                    buffer[j] = 0;
                }
            }        

            // reset variables
            bytes_read = 0;
            crc_receive_index = 0;
            start_flag_found = 0;
            last_8_bits = 0x00;
            byte_buffer = 0x00;
            byte_index = 0;
            bit_index = 0;
            bit_period_overflow = 0;
            current_tone = HIGH_TONE;
            prev_tone = HIGH_TONE;
        }
    }
}
