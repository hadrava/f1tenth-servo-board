#include "global.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "servo.h"
#include "uart.h"
#include "input_capture.h"
#include "hw.h"
#include "speed_controller.h"
#include "sb_states.h"

#define INPUT_CAPTURE_TIMEOUT	10
#define SERIAL_MODES_TIMEOUT	1000

// Global state of whole system
uint8_t global_state = SB_BOOT;
uint16_t debug = 0;
// Variable time gets incremented each 10 ms
uint16_t time = 0;
uint16_t substate_start_time = 0;

// Input capture
uint16_t capture_speed_us = 0;
uint16_t capture_angle_us = 0;
uint8_t capture_speed_data_age = 0xFF;
uint8_t capture_angle_data_age = 0xFF;

void manage_input_capture(void) {
	// If measurement is not running, save the result and , start new one
	if (! INPUT_CAPTURE_SPEED_RUNNING()) {
		capture_speed_us = convert_raw_counter_to_us(COUNTER_SPEED);
		capture_speed_data_age = 0;
		INPUT_CAPTURE_SPEED_SINGLE_SHOT();
	}
	if (! INPUT_CAPTURE_ANGLE_RUNNING()) {
		capture_angle_us = convert_raw_counter_to_us(COUNTER_ANGLE);
		capture_angle_data_age = 0;
		INPUT_CAPTURE_ANGLE_SINGLE_SHOT();
	}
}

// Input from serial line
uint16_t serial_speed_us = 0;
uint16_t serial_angle_us = 0;
uint8_t serial_set_mode = 0;
uint8_t serial_timeout = 0;
uint16_t serial_data_age = 0xFFFF;
unsigned char in_buffer[12];
int in_buffer_len = 0;

void uart_input_tick(void) {
	if (! UART_INPUT_READY)
		return;

	in_buffer[in_buffer_len] = UDR0;
	in_buffer_len++;

	if (in_buffer_len == 1) {
		if (in_buffer[0] != 'B') {
			// First byte incorrect, flush buffer
			in_buffer_len = 0;
		}
	}
	else if (in_buffer_len == 9) {
		// reserved, must be 0 in current protocol version
		if (in_buffer[7] != 0)
			return;
		if (in_buffer[8] != 0)
			return;

		// load data from packet
		serial_speed_us = in_buffer[1] | ((uint16_t) in_buffer[2]) << 8;
		serial_angle_us = in_buffer[3] | ((uint16_t) in_buffer[4]) << 8;
		serial_set_mode = in_buffer[5];
		serial_timeout = in_buffer[6];
		serial_data_age = 0;

		// clear buffer
		in_buffer_len = 0;

	}
}

// Output to serial line
unsigned char out_buffer [32];
int out_buffer_pos = 0;
int out_buffer_len = 0;

void uart_output_tick(void) {
	if (! UART_OUTPUT_READY)
		return;

	if (out_buffer_len > out_buffer_pos) {
		UDR0 = out_buffer[out_buffer_pos];
		out_buffer_pos++;
	}
}

// 100 Hz tasks
void check_timer_overflow() {
	if (!SERVO_OVERFLOW)
		return;

	SERVO_OVERFLOW_CLEAR();
	time++;
	// All overflow tasks
	speed_controller_simulate_state(OCR1_SPEED);

	out_buffer[0] = 'S';
	out_buffer[1] = global_state;
	out_buffer[2] = (OCR1_SPEED >> 8);
	out_buffer[3] = OCR1_SPEED;
	out_buffer[4] = (OCR1_ANGLE >> 8);
	out_buffer[5] = OCR1_ANGLE;
	out_buffer[6] = (capture_speed_us >> 8);
	out_buffer[7] = capture_speed_us;
	out_buffer[8] = (capture_angle_us >> 8);
	out_buffer[9] = capture_angle_us;
	out_buffer[10] = (time >> 8);
	out_buffer[11] = time;
	out_buffer[12] = speed_controller_current_state;
	out_buffer[13] = capture_speed_data_age;
	out_buffer[14] = capture_angle_data_age;
	out_buffer[15] = (serial_data_age >> 8);
	out_buffer[16] = serial_data_age;
	out_buffer[17] = (debug >> 8);
	out_buffer[18] = debug;
	out_buffer_len = 19;
	out_buffer_pos = 0;

	if (capture_speed_data_age < 0xFF)
		capture_speed_data_age++;
	if (capture_angle_data_age < 0xFF)
		capture_angle_data_age++;
	if (serial_data_age < 0xFFFF)
		serial_data_age++;
}

void switch_state_serial(void) {
	if ((serial_data_age < 2) && ((global_state & SB_MASK) != (serial_set_mode & SB_MASK))) {
		global_state = serial_set_mode;
		substate_start_time = time;
	}
}

void switch_to_substate(uint8_t substate) {
	global_state &= SB_MASK;
	global_state |= substate;
	substate_start_time = time;
}

void select_action(void) {
	int16_t trim = 0;
	uint16_t capture_speed_state = capture_us_to_speed_state(capture_speed_us);
	switch (global_state & SB_MASK) {
		case SB_BOOT:
			speed_controller_try_set_speed_state(1500);
			if (time - substate_start_time > 300) {// Wait first three seconds
				global_state = SB_DEFAULT_STATE;
				substate_start_time = time;
			}
			break;

		case SB_REMOTE_ONLY:
			speed_controller_try_set_angle_us(capture_angle_us);
			if (capture_speed_data_age > INPUT_CAPTURE_TIMEOUT) {
				// Safety timeout: speed input capture (disconnected wire, dead receiver)
				speed_controller_try_set_speed_state(1500);
				break;
			}
			speed_controller_try_set_speed_us(capture_speed_us);
			debug++;
			break;

		case SB_REMOTE_STATE_DEMO:
			speed_controller_try_set_angle_us(capture_angle_us);
			if (capture_speed_data_age > INPUT_CAPTURE_TIMEOUT) {
				// Safety timeout: speed input capture (disconnected wire, dead receiver)
				speed_controller_try_set_speed_state(1500);
				break;
			}
			if (capture_speed_state > 1500)
				speed_controller_try_set_speed_state(2000); // Forward
			else if (capture_speed_state < 1500)
				speed_controller_try_set_speed_state(1000 | 0x4000); // Backward
			else
				speed_controller_try_set_speed_state(1000 | 0x8000); // Brake
			break;

		case SB_SERIAL_ONLY:
			if (serial_data_age >= SERIAL_MODES_TIMEOUT) {
				// SERIAL_MODES_TIMEOUT/100 seconds without signal, switch to default state
				global_state = SB_DEFAULT_STATE;
				substate_start_time = time;
			}
			speed_controller_try_set_angle_state(serial_angle_us);
			if (serial_data_age > serial_timeout) {
				// Safety no serial signal timeout
				speed_controller_try_set_speed_state(1500);
				break;
			}
			speed_controller_try_set_speed_state(serial_speed_us);
			break;

		case SB_TAKEOVER_WITH_TRIM:
			trim = capture_angle_us - current_config.angle_trim;
		case SB_TAKEOVER:
			if (serial_data_age >= SERIAL_MODES_TIMEOUT) {
				// SERIAL_MODES_TIMEOUT/100 seconds without signal, switch to default state
				global_state = SB_DEFAULT_STATE;
				substate_start_time = time;
			}
			if ((capture_angle_data_age > INPUT_CAPTURE_TIMEOUT) || (capture_speed_data_age > INPUT_CAPTURE_TIMEOUT)) {
				// Safety timeout: speed or angle input capture (disconnected wire, dead receiver)
				speed_controller_try_set_angle_state(serial_angle_us + trim);
				speed_controller_try_set_speed_state(1500);
				break;
			}
			if ((capture_speed_state != 1500) || (serial_data_age > serial_timeout)) {
				// Takeover -- speed knob is not in neutral
				speed_controller_try_set_angle_us(capture_angle_us);
				speed_controller_try_set_speed_state(capture_speed_state | 0x4000);
				if ((global_state & (~SB_MASK)) == 0) {
					switch_to_substate(1);
				}
				else {
					if (time - substate_start_time > 5) // We were in substate 1 for more than (5 * 10) ms
						switch_to_substate(2); // We might already be in 2, in that case just reset the timer...
				}
				break;
			}
			if ((global_state & (~SB_MASK)) == 2 ) {
				// Remote released, spend some time in paused substate (2) before serial line takes the control
				speed_controller_try_set_angle_us(capture_angle_us);
				speed_controller_try_set_speed_state(capture_speed_state | 0x4000);
				if (time - substate_start_time > 200) // Released for two seconds, return to serial
					switch_to_substate(0);
				break;
			}
			else {
				// We were in substate 1 for just a few measurements, return control to serial immediatelly
				switch_to_substate(0);
			}
			speed_controller_try_set_angle_state(serial_angle_us + trim);
			speed_controller_try_set_speed_state(serial_speed_us);
			break;

		case SB_SPEED_LIMIT:
			if (serial_data_age >= SERIAL_MODES_TIMEOUT) {
				// SERIAL_MODES_TIMEOUT/100 seconds without signal, switch to default state
				global_state = SB_DEFAULT_STATE;
				substate_start_time = time;
			}
			trim = capture_angle_us - current_config.angle_trim;
			if ((capture_angle_data_age > INPUT_CAPTURE_TIMEOUT) || (capture_speed_data_age > INPUT_CAPTURE_TIMEOUT) || (serial_data_age > serial_timeout)) {
				// Safety timeout: speed or angle input capture (disconnected wire, dead receiver) or no serial data
				speed_controller_try_set_angle_state(serial_angle_us + trim);
				speed_controller_try_set_speed_state(1500);
				break;
			}
			speed_controller_try_set_angle_state(serial_angle_us + trim);
			if (capture_speed_state > 1500) {
				speed_controller_try_set_speed_state(limit_speed_state_with_speed_state(serial_speed_us, capture_speed_state));
			}
			else {
				speed_controller_try_set_speed_state(1500);
			}
			break;

		case SB_PAUSE:
			trim = capture_angle_us - current_config.angle_trim;
			if (serial_data_age >= SERIAL_MODES_TIMEOUT) {
				// SERIAL_MODES_TIMEOUT/100 seconds without signal, switch to default state
				global_state = SB_DEFAULT_STATE;
				substate_start_time = time;
			}
			if ((capture_angle_data_age > INPUT_CAPTURE_TIMEOUT) || (capture_speed_data_age > INPUT_CAPTURE_TIMEOUT) || (serial_data_age > serial_timeout)) {
				// Safety timeout: speed or angle input capture (disconnected wire, dead receiver) or no serial data
				speed_controller_try_set_angle_state(serial_angle_us + trim);
				speed_controller_try_set_speed_state(1500);
				break;
			}
			if ((global_state & (~SB_MASK)) == 0) {
				// Normal running
				if (capture_speed_state != 1500) {
					// Remote action detected
					// Brake -- speed knob is not in neutral
					speed_controller_try_set_angle_us(capture_angle_us);
					speed_controller_try_set_speed_state(1000 | 0x8000);
					switch_to_substate(1); // Start time measurement
				}
				else {
					// Use data from serial line
					speed_controller_try_set_angle_state(serial_angle_us + trim);
					speed_controller_try_set_speed_state(serial_speed_us);
				}
			}
			else if ((global_state & (~SB_MASK)) == 1) {
				// Remote action detected
				if (capture_speed_state != 1500) {
					// Remote action detected
					// Brake -- speed knob is not in neutral
					speed_controller_try_set_angle_us(capture_angle_us);
					speed_controller_try_set_speed_state(1000 | 0x8000);
					if (time - substate_start_time > 5) // We were in substate 1 for more than (5 * 10) ms
						switch_to_substate(2);
				}
				else {
					// False alarm, continue running normaly
					switch_to_substate(0);
					speed_controller_try_set_angle_state(serial_angle_us + trim);
					speed_controller_try_set_speed_state(serial_speed_us);
				}
			}
			else if ((global_state & (~SB_MASK)) == 2) {
				// Paused, button pressed, waiting for release
				if (capture_speed_state != 1500) {
					// Remote is still active
					// Brake -- speed knob is not in neutral
					speed_controller_try_set_angle_us(capture_angle_us);
					speed_controller_try_set_speed_state(1000 | 0x8000);
				}
				else {
					// Remote released
					switch_to_substate(3);
					speed_controller_try_set_angle_us(capture_angle_us);
					speed_controller_try_set_speed_state(1500);
				}
			}
			else if ((global_state & (~SB_MASK)) == 3) {
				// Remote released
				if (capture_speed_state != 1500) {
					// False signal, remote is still pressed
					// Brake -- speed knob is not in neutral
					switch_to_substate(2);
					speed_controller_try_set_angle_us(capture_angle_us);
					speed_controller_try_set_speed_state(1000 | 0x8000);
				}
				else {
					speed_controller_try_set_angle_us(capture_angle_us);
					speed_controller_try_set_speed_state(1500);
					if (time - substate_start_time > 5) // Remote was released for more than (5 * 10) ms
						switch_to_substate(4);
				}
			}
			else if ((global_state & (~SB_MASK)) == 4) {
				// Proper pause, nothing is pressed anymore
				speed_controller_try_set_angle_us(capture_angle_us);
				speed_controller_try_set_speed_state(1500);
				if (capture_speed_state != 1500) {
					// New press
					switch_to_substate(5);
				}
			}
			else if ((global_state & (~SB_MASK)) == 5) {
				// Proper pause, press detected
				speed_controller_try_set_angle_us(capture_angle_us);
				speed_controller_try_set_speed_state(1500);
				if (capture_speed_state != 1500) {
					if (time - substate_start_time > 5) // Remote was pressed for more than (5 * 10) ms
						switch_to_substate(6);
				}
				else {
					// False alarm, still nothing pressed
					switch_to_substate(4);
				}
			}
			else if ((global_state & (~SB_MASK)) == 6) {
				// Proper pause, remote pressed
				speed_controller_try_set_angle_us(capture_angle_us);
				speed_controller_try_set_speed_state(1500);
				if (capture_speed_state != 1500) {
				}
				else {
					// Release
					if (time - substate_start_time > 5) // Remote was released for more than (5 * 10) ms
						switch_to_substate(7);
				}
			}
			else if ((global_state & (~SB_MASK)) == 7) {
				// Proper pause, release detected
				speed_controller_try_set_angle_us(capture_angle_us);
				speed_controller_try_set_speed_state(1500);
				if (capture_speed_state != 1500) {
					// Nope, still pressed, get back
					switch_to_substate(6);
				}
				else {
					// Release
					if (time - substate_start_time > 5) // Remote was released for more than (5 * 10) ms
						switch_to_substate(0); // Unpause
				}
			}
			break;
	}

}

int main(void) {
	DDRB |= _BV(PB5); // LED output enable

	servo_init();
	uart_init();
	input_capture_init();
	sei();


	// XXX: uart_.*_tick() functions has to be called at least each 75 us
	// (given current uart speed). So we would be just fine with calling
	// them just once per whole loop.
	while (1) {
		PORTB |= _BV(PB5);
		uart_input_tick();
		uart_output_tick();
		check_timer_overflow();

		uart_input_tick();
		uart_output_tick();
		manage_input_capture();

		uart_input_tick();
		uart_output_tick();

		switch_state_serial();
		PORTB &= ~(_BV(PB5));
		select_action();
	}
}
