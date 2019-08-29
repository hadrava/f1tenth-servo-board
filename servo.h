#ifndef _SERVO_H_
#define _SERVO_H_

#include <stdint.h>
#include <avr/io.h>

// Offsets not used with speed_controller:
#define STD_SERVO_OFFSET 990
#define EXT_SERVO_OFFSET 480

// Counter top, (one phase is upcounting + downcounting = 2x ICR1 timer clock cycles)
// We are using 16MHz / 8 prescaler --> 2MHz timer clock
#define SERVO_100HZ8_ICR1 9921 // 9.921 ms = apx. 100.8 Hz (Measured signal period of the receiver)
#define SERVO_50HZ_ICR1 20000 // 20 ms = 50 Hz

// Use 100.8 Hz
#define SERVO_ICR1 SERVO_100HZ8_ICR1

void servo_init(void);
void servo_deinit(void);

#define SERVO_OVERFLOW (TIFR1 & (1<<ICF1))
#define SERVO_OVERFLOW_CLEAR() do {TIFR1 |= 1<<ICF1;} while (0)

// Do not use following functions when speed_controller is used
void set_std_servo(uint8_t servo_speed, uint8_t servo_angle);
void set_ext_servo(uint8_t servo_speed, uint8_t servo_angle);
void set_servo_us(uint16_t servo_speed, uint16_t servo_angle);

void set_angle_servo_us(uint16_t servo_angle);
void set_speed_servo_us(uint16_t servo_speed);


#endif
