#include "global.h"
#include <avr/io.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include "input_capture.h"

void input_capture_init(void) {
	TCCR0A = 0;                          // Initial value
	TCCR0B = _BV(CS01);                  // CS01: clk / 8 (--> 16 MHz / 8 = 2 MHz clock) (will overflow at apx. 7 kHz rate)
	TIMSK0 = 0;                          // Initial value, Overflow Interrupt Disable

	TCCR2A = 0;                          // Initial value
	TCCR2B = _BV(CS21);                  // CS21: clk / 8 (--> 16 MHz / 8 = 2 MHz clock) (will overflow at apx. 7 kHz rate)
	TIMSK2 = 0;                          // Initial value, Overflow Interrupt Disable
}

void input_capture_0_single_shot(void) {
	if (EIMSK & _BV(INT0))
		return; // Capture is already running, do nothing
	cli();
	EICRA |= _BV(ISC00) | _BV(ISC01);    // The rising edge on INT0 generates interrupt
	GPIOR0 &= ~_BV(0);                   // Clear bit in GPIOR0 -- detecting rising edge
	EIFR |= _BV(INTF0);                  // Clear interrupt flag on INT0
	EIMSK |= _BV(INT0);                  // Enable interrupts on INT0
	sei();
}

uint8_t input_capture_0_running(void) {
	return !!(EIMSK & _BV(INT0));
}

void input_capture_1_single_shot(void) {
	if (EIMSK & _BV(INT1))
		return; // Capture is already running, do nothing
	cli();
	EICRA |= _BV(ISC10) | _BV(ISC11);    // The rising edge on INT1 generates interrupt
	GPIOR0 &= ~_BV(1);                   // Clear bit in GPIOR0 -- detecting rising edge
	EIFR |= _BV(INTF1);                  // Clear interrupt flag on INT1
	EIMSK |= _BV(INT1);                  // Enable interrupts on INT1
	sei();
}

uint8_t input_capture_1_running(void) {
	return !!(EIMSK & _BV(INT1));
}

void input_capture_deinit(void) {
	TIMSK0 = 0;                          // Initial value, Overflow Interrupt Disable
	TCCR0A = 0;                          // Initial value
	TCCR0B = 0;                          // Initial value

	TIMSK2 = 0;                          // Initial value, Overflow Interrupt Disable
	TCCR2A = 0;                          // Initial value
	TCCR2B = 0;                          // Initial value
}

uint16_t convert_raw_counter_to_us(uint16_t counter) {
	// Self calibartion showed, that we sometimes report (correct value - 1).
	// We need to divide raw counter value by two, so we are going to round it up.
	return (counter + 1) >> 1;
}
