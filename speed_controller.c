#include "global.h"
#include <avr/io.h>
#include <stdint.h>
#include "servo.h"
#include "hw.h"
#include "speed_controller.h"

struct speed_controller_config current_config = {
	1510, // angle trim

	2000, // max forward
	1563, // minimal forward value which generates any movement
	1490, // lowest value that is considered as transition to forward state

	1510, // maximal value, which is still considered as neutral (note that it is larger than previous variable -- we are not sure about what is happening in between them)
	1446, // minimal value, which is still considered as neutral (note that it is smaller than next variable -- we are not sure about what is happening in between them)

	1466, // maximal value which is still counted as backward/brake value
	1400, // maximal value which moves the car backwards
	1000, // minimal value that can be send to speed controller

	4 // tansition filter: 3 are enough, better safe then sorry
};

uint8_t speed_controller_current_state = 0x77;
/*
 * Motor driver's internal state machine:
 *
 *                    |            action:
 *                    |    +          N         -
 *    previous state: *-----------------------------
 * 1:    neutral-1    | forward   neutral-1   brake
 * 1:    forward      | forward   neutral-1   brake
 * 2:    brake        | forward   neutral-2   brake
 * 4:    neutral-2    | forward   neutral-2   backward
 * 4:    backward     | forward   neutral-2   backward
 *
 *                    |         desired state:
 *                    | neutral-1   forward   brake   neutral-2   backward
 *     current state: *-----------------------------------------------------------
 * 1:    neutral-1    |     N          +        -        - N        - N -
 * 1:    forward      |     N          +        -        - N        - N -
 * 2:    brake        |    + N         +        -         N          N -
 * 4:    neutral-2    |    + N         +       + -        N           -
 * 4:    backward     |    + N         +       + -        N           -
 */

uint8_t transition_counter = 1;
void speed_controller_simulate_state(uint16_t set_speed) {
	uint8_t new_state = speed_controller_current_state & 0x0F;
	uint8_t state = (speed_controller_current_state | (speed_controller_current_state << 4)) & 0xF0;

	if (set_speed >= current_config.min_forward) {
		// Forward
		// 0  Ba Br Fo
		//     \  \ |
		//      \  \|
		//       \  |
		//        \ |
		//         \|
		//          |
		// 0  0  0  1
		new_state |= 0x10;
	}
	if ((set_speed <= current_config.max_neutral) && (set_speed >= current_config.min_neutral)) {
		// Neutral
		// 0  Ba Br Fo
		//    | /   |
		//    |/    |
		//    |     |
		// 0  ?  0  ?
		new_state |= (state & 0x50) | ((state & 0x20) << 1);
	}
	if (set_speed <= current_config.max_backward) {
		// Backward
		// 0  Ba Br Fo
		//    |  | /
		//    |  |/
		//    |  |
		// 0  ?  ?  0
		new_state |= (state & 0x60) | ((state & 0x10) << 1);
	}

	if (new_state != speed_controller_current_state) {
		new_state = (new_state & 0xF0) | (state >> 4);
		transition_counter = 1;
	}
	else {
		transition_counter++;
		if (transition_counter >= current_config.transition_filter) {
			new_state = (new_state & 0xF0) | (new_state >> 4);
			transition_counter--;
		}
	}
	speed_controller_current_state = new_state;
}

uint16_t calculate_action(uint16_t desired_speed_state) {
	uint16_t desired_speed = desired_speed_state & 0x3FFF;
	if (desired_speed == 1500) {
		// Neutral
		return (current_config.min_forward + current_config.max_backward) / 2;
	}
	else if (desired_speed > 1500) {
		// Forward
		return desired_speed - 1500 + current_config.min_forward_moving;
	}
	else if (((desired_speed_state >> 8) & 0xC0) == 0x00) {
		// Pass though, no state is enforced
		return desired_speed - 1500 + current_config.max_backward_moving;
	}
	else if (((desired_speed_state >> 8) & 0xC0) == 0x80) {
		// Brake
		if (speed_controller_current_state & 0x44) // if backward state might be active
			return current_config.max_neutral + 1; // use smallest possible forwad speed
		else
			return desired_speed - 1500 + current_config.max_backward_moving; // Apply brakes
	}
	else {
		// Backward movement
		if (speed_controller_current_state & 0x11) // if forward might be active
			return desired_speed - 1500 + current_config.max_backward_moving; // brake (or use selected backward speed)
		else if (speed_controller_current_state & 0x22) // if brake might be active
			return (current_config.min_forward + current_config.max_backward) / 2; // Neutral-2
		else
			return desired_speed - 1500 + current_config.max_backward_moving; // Finally: move backward
	}
}


uint8_t speed_controller_try_set_speed_state(uint16_t speed_state) {
	uint16_t speed_us = calculate_action(speed_state);
	return speed_controller_try_set_speed_us(speed_us);
}

uint8_t speed_controller_try_set_speed_us(uint16_t speed_us) {
	if (TCNT1 > SERVO_UPDATE_SAFE_THRESHOLD) // Do not change just before TOP
		return 0;
	if (SERVO_OVERFLOW) // Do not change until the old value gets logged
		return 0;

	OCR1_SPEED = speed_us;
	return 1;
}

uint8_t speed_controller_try_set_angle_state(uint16_t angle_state) {
	uint16_t angle_us = angle_state + current_config.angle_trim - 1500;
	return speed_controller_try_set_angle_us(angle_us);
}

uint8_t speed_controller_try_set_angle_us(uint16_t angle_us) {
	if (TCNT1 > SERVO_UPDATE_SAFE_THRESHOLD) // Do not change just before TOP
		return 0;
	if (SERVO_OVERFLOW) // Do not change until the old value gets logged
		return 0;

	OCR1_ANGLE = angle_us;
	return 1;
}

uint16_t capture_us_to_speed_state(uint16_t speed_us) {
	if (speed_us > current_config.min_forward_moving)
		return speed_us - current_config.min_forward_moving + 1500;
	else if (speed_us < current_config.max_backward_moving)
		return speed_us - current_config.max_backward_moving + 1500;
	else
		return 1500;
}

uint16_t limit_speed_state_with_speed_state(uint16_t speed_state, uint16_t limit) {
	uint16_t mode = 0xC000 & speed_state;
	speed_state &= 0x3FFF;
	if ((speed_state > 1500) && (speed_state > limit))
		speed_state = limit;
	else if ((speed_state < 1500) && ((1500 - speed_state) > (limit - 1500)))
		speed_state = 3000 - limit;

	return speed_state | mode;
}
