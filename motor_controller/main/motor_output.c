#include "motor_output.h"

#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

#define MOTOR_LEFT_DIR_GPIO   GPIO_NUM_27
#define MOTOR_LEFT_PWM_GPIO   GPIO_NUM_14

#define MOTOR_RIGHT_DIR_GPIO  GPIO_NUM_12
#define MOTOR_RIGHT_PWM_GPIO  GPIO_NUM_13

#define PWM_TIMER             LEDC_TIMER_0
#define PWM_MODE              LEDC_LOW_SPEED_MODE
#define PWM_FREQ_HZ           20000
#define PWM_RESOLUTION        LEDC_TIMER_10_BIT
#define PWM_MAX_DUTY          1023

#define LEFT_PWM_CHANNEL      LEDC_CHANNEL_0
#define RIGHT_PWM_CHANNEL     LEDC_CHANNEL_1

// Temporary open-loop scaling.
// Adjust later after real motor testing.
#define MAX_COMMAND_MMPS      1500

static int clamp_int(int value, int min_value, int max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void set_motor(ledc_channel_t channel, gpio_num_t dir_gpio, int target_mmps)
{
    int command = clamp_int(target_mmps, -MAX_COMMAND_MMPS, MAX_COMMAND_MMPS);

    bool forward = command >= 0;
    int magnitude = abs(command);

    uint32_t duty = (uint32_t)((magnitude * PWM_MAX_DUTY) / MAX_COMMAND_MMPS);

    gpio_set_level(dir_gpio, forward ? 1 : 0);
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}

void motor_output_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_LEFT_DIR_GPIO) | (1ULL << MOTOR_RIGHT_DIR_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    ledc_timer_config_t timer_conf = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t left_channel = {
        .gpio_num = MOTOR_LEFT_PWM_GPIO,
        .speed_mode = PWM_MODE,
        .channel = LEFT_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&left_channel);

    ledc_channel_config_t right_channel = {
        .gpio_num = MOTOR_RIGHT_PWM_GPIO,
        .speed_mode = PWM_MODE,
        .channel = RIGHT_PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&right_channel);

    motor_output_stop();
}

void motor_output_update(int left_mmps, int right_mmps)
{
    set_motor(LEFT_PWM_CHANNEL, MOTOR_LEFT_DIR_GPIO, left_mmps);
    set_motor(RIGHT_PWM_CHANNEL, MOTOR_RIGHT_DIR_GPIO, right_mmps);
}

void motor_output_stop(void)
{
    set_motor(LEFT_PWM_CHANNEL, MOTOR_LEFT_DIR_GPIO, 0);
    set_motor(RIGHT_PWM_CHANNEL, MOTOR_RIGHT_DIR_GPIO, 0);
}