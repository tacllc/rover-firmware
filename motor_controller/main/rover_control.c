#include "rover_control.h"
#include "motor_output.h"
#include "esp_timer.h"

#define WHEEL_BASE_M 0.30f
#define CMD_VEL_TIMEOUT_MS 500

static volatile double last_linear_x = 0.0;
static volatile double last_angular_z = 0.0;
static volatile int64_t last_cmd_vel_time_ms = 0;

void rover_control_init(void)
{
    last_linear_x = 0.0;
    last_angular_z = 0.0;
    last_cmd_vel_time_ms = 0;
}

void rover_control_set_cmd_vel(double linear_x, double angular_z)
{
    last_linear_x = linear_x;
    last_angular_z = angular_z;
    last_cmd_vel_time_ms = esp_timer_get_time() / 1000;
}

void rover_control_update(int *left_mmps, int *right_mmps)
{
    float linear = (float)last_linear_x;
    float angular = (float)last_angular_z;

    int64_t now_ms = esp_timer_get_time() / 1000;

    if ((now_ms - last_cmd_vel_time_ms) > CMD_VEL_TIMEOUT_MS) {
        linear = 0.0f;
        angular = 0.0f;
    }

    float left_mps  = linear - (angular * WHEEL_BASE_M / 2.0f);
    float right_mps = linear + (angular * WHEEL_BASE_M / 2.0f);

    int left_target_mmps = (int)(left_mps * 1000.0f);
    int right_target_mmps = (int)(right_mps * 1000.0f);

    if (left_mmps != NULL) {
        *left_mmps = left_target_mmps;
    }

    if (right_mmps != NULL) {
        *right_mmps = right_target_mmps;
    }

    motor_output_update(left_target_mmps, right_target_mmps);
}