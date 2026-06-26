#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "esp_timer.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/custom_transport.h>

#include "esp32_serial_transport.h"
#include "motor_output.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define WHEEL_BASE_M 0.30f
#define CMD_VEL_TIMEOUT_MS 500
#define MOTOR_OUTPUT_TEST 0

static rcl_publisher_t left_wheel_publisher;
static rcl_publisher_t right_wheel_publisher;
static std_msgs__msg__Int32 left_wheel_msg;
static std_msgs__msg__Int32 right_wheel_msg;

static rcl_subscription_t cmd_vel_subscriber;
static geometry_msgs__msg__Twist cmd_vel_msg;

static volatile double last_linear_x = 0.0;
static volatile double last_angular_z = 0.0;
static volatile int64_t last_cmd_vel_time_ms = 0;

static volatile int left_motor_target_mmps = 0;
static volatile int right_motor_target_mmps = 0;

static size_t uart_port = UART_NUM_0;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    float linear = (float)last_linear_x;
    float angular = (float)last_angular_z;

    int64_t now_ms = esp_timer_get_time() / 1000;

    if ((now_ms - last_cmd_vel_time_ms) > CMD_VEL_TIMEOUT_MS) {
        linear = 0.0f;
        angular = 0.0f;
    }

    float left_mps  = linear - (angular * WHEEL_BASE_M / 2.0f);
    float right_mps = linear + (angular * WHEEL_BASE_M / 2.0f);

    // left_motor_target_mmps = (int)(left_mps * 1000.0f);
    // right_motor_target_mmps = (int)(right_mps * 1000.0f);

#if MOTOR_OUTPUT_TEST
    left_motor_target_mmps = 500;
    right_motor_target_mmps = -500;
#else
    left_motor_target_mmps = (int)(left_mps * 1000.0f);
    right_motor_target_mmps = (int)(right_mps * 1000.0f);
#endif

    motor_output_update(left_motor_target_mmps, right_motor_target_mmps);    

    left_wheel_msg.data = left_motor_target_mmps;
    right_wheel_msg.data = right_motor_target_mmps;

    motor_output_update(left_motor_target_mmps, right_motor_target_mmps);

    RCSOFTCHECK(rcl_publish(&left_wheel_publisher, &left_wheel_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_wheel_publisher, &right_wheel_msg, NULL));
}

void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * twist_msg =
        (const geometry_msgs__msg__Twist *)msgin;

    last_linear_x = twist_msg->linear.x;
    last_angular_z = twist_msg->angular.z;
    last_cmd_vel_time_ms = esp_timer_get_time() / 1000;
}

void micro_ros_task(void * arg)
{
    (void)arg;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_motor_controller", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &left_wheel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "left_wheel_target_mmps"));

    RCCHECK(rclc_publisher_init_default(
        &right_wheel_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "right_wheel_target_mmps"));

    RCCHECK(rclc_subscription_init_best_effort(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    rcl_timer_t timer;
    const unsigned int timer_timeout_ms = 100;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout_ms),
        timer_callback,
        true));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &cmd_vel_msg,
        &cmd_vel_callback,
        ON_NEW_DATA));

    left_wheel_msg.data = 0;
    right_wheel_msg.data = 0;

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&left_wheel_publisher, &node));
    RCCHECK(rcl_publisher_fini(&right_wheel_publisher, &node));
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_UART_TRANSPORT)
    rmw_uros_set_custom_transport(
        true,
        (void *) &uart_port,
        esp32_serial_open,
        esp32_serial_close,
        esp32_serial_write,
        esp32_serial_read
    );
#else
#error micro-ROS transports misconfigured
#endif

    motor_output_init();

    xTaskCreate(
        micro_ros_task,
        "uros_task",
        16500,
        NULL,
        5,
        NULL
    );
}

