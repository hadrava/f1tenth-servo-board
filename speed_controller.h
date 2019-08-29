#ifndef _SPEED_CONTROLLER_H_
#define _SPEED_CONTROLLER_H_

#include <stdint.h>
#include "servo.h"

#define SERVO_UPDATE_SAFE_THRESHOLD   (SERVO_ICR1 - 4)

struct speed_controller_config {
	uint16_t angle_trim;

	uint16_t max_forward;
	uint16_t min_forward_moving;
	uint16_t min_forward;
	uint16_t max_neutral;
	uint16_t min_neutral;
	uint16_t max_backward;
	uint16_t max_backward_moving;
	uint16_t min_backward;
	uint8_t transition_filter;
};

extern struct speed_controller_config current_config;
extern uint8_t speed_controller_current_state;

void speed_controller_simulate_state(uint16_t set_speed);

uint16_t capture_us_to_speed_state(uint16_t speed_us);
uint16_t limit_speed_state_with_speed_state(uint16_t speed_state, uint16_t limit);


uint8_t speed_controller_try_set_speed_state(uint16_t speed_state); // use for serial controll
// normal servo_us data, highest two bits has special meaning:
// 00xx xxxx -- direct pass-through does not matter, whtether it means brake or backward
//              (usefull for faster reduction of speed during normal driving)
// 01xx xxxx -- enforce backward (needed if we actually need to move backwards)
// 10xx xxxx -- enforce brake (usefull for long emergency brake)

uint8_t speed_controller_try_set_speed_us(uint16_t speed_us); // use for remote controll
uint8_t speed_controller_try_set_angle_state(uint16_t angle_state); // apply angle_trim
uint8_t speed_controller_try_set_angle_us(uint16_t angle_us);

#endif
