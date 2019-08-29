#ifndef _INPUT_CAPTURE_H_
#define _INPUT_CAPTURE_H_

#include <stdint.h>

void input_capture_init(void);
void input_capture_deinit(void);

void input_capture_0_single_shot(void);
uint8_t input_capture_0_running(void);
extern volatile uint16_t counter_0;

void input_capture_1_single_shot(void);
uint8_t input_capture_1_running(void);
extern volatile uint16_t counter_1;

uint16_t convert_raw_counter_to_us(uint16_t counter);

#endif
