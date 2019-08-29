#include "global.h"
#include <avr/io.h>
#include <stdint.h>
#include "servo.h"
#include "hw.h"

void set_angle_servo_us(uint16_t servo_angle) {
	// Set length of servo signal in us. You can send signal that will make the servo crash to its endstop.
	OCR1_ANGLE = servo_angle;
}

void set_speed_servo_us(uint16_t servo_speed) {
	// Set length of servo signal in us. You can send signal that will make the servo crash to its endstop.
	OCR1_SPEED = servo_speed;
}


void servo_init(void) {
	set_std_servo(0x80, 0x80);                        // default servo position in the middle
	ICR1 = SERVO_ICR1;                                // Selected signal period
	TCCR1B = (1<<WGM13) | (1<<CS11);                  // WGM13, WGM11: PWM, Phase Correct, CS11: clk / 8 (--> 16 MHz / 8 = 2 MHz clock)
	TCCR1A = (1<<COM1A1) | (1<<COM1B1) | (1<<WGM11);  // Clear OC1A/B when upcounting, set when downcounting, assign output pins, WGM11: update at TOP
	DDRB |= _BV(PB1) | _BV(PB2);                      // Servo outputs enable
}

void servo_deinit(void) {
	DDRB &= ~(_BV(PB1) | _BV(PB2));      // Servo outputs disable
	TCCR1A = 0;                          // Initial value, Normal port operation, pins disconnected
	TCCR1B = 0;                          // Initial value
	ICR1 = 0;                            // Initial value
	OCR1A = 0;                           // Initial value
	OCR1B = 0;                           // Initial value
}


// Do not use following function when speed_controller is used
void set_std_servo(uint8_t servo_speed, uint8_t servo_angle) {
	// Generates signal in range 1 - 2 ms
	// Registers are double buffered in PWM modes, so any change is glitch-free
	OCR1_SPEED = STD_SERVO_OFFSET + ((uint16_t) servo_speed << 2);
	OCR1_ANGLE = STD_SERVO_OFFSET + ((uint16_t) servo_angle << 2);
}

// Do not use following function when speed_controller is used
void set_ext_servo(uint8_t servo_speed, uint8_t servo_angle) {
	// Generates signal in range 0.5 - 2.5 ms
	// Registers are double buffered in PWM modes, so any change is glitch-free
	OCR1_SPEED = EXT_SERVO_OFFSET + ((uint16_t) servo_speed << 3);
	OCR1_ANGLE = EXT_SERVO_OFFSET + ((uint16_t) servo_angle << 3);
}

// Do not use following function when speed_controller is used
void set_servo_us(uint16_t servo_speed, uint16_t servo_angle) {
	// Set length of servo signal in us. You can send signal that will make the servo crash to its endstop.
	OCR1_SPEED = servo_speed;
	OCR1_ANGLE = servo_angle;
}
