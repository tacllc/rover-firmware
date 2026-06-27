#pragma once

#include <stdint.h>

void encoder_input_init(void);

int64_t encoder_get_left_ticks(void);
int64_t encoder_get_right_ticks(void);

void encoder_reset_ticks(void);
