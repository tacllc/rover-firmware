#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "micro_ros_node.h"
#include "rover_control.h"
#include "encoder_input.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Encoder publishers only for now.
// Wheel target diagnostic publishers are temporarily removed to avoid
// micro-ROS publisher/resource limits.
static rcl_publisher_t left_encoder_publisher;
static rcl_publisher_t right_encoder_publisher;

static std_msgs__msg__Int32 left_encoder_msg;
static std_msgs__msg__Int32 right_encoder_msg;

static rcl_subscription_t cmd_vel_subscriber;
static geometry_msgs__msg__Twist cmd_vel_msg;

static void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer == NULL) {
        return;
    }

    int left_mmps = 0;
    int right_mmps = 0;

    // Updates motor outputs based on latest /cmd_vel.
    rover_control_update(&left_mmps, &right_mmps);

    // Publish encoder counts.
    left_encoder_msg.data = (int32_t)encoder_get_left_ticks();
    right_encoder_msg.data = (int32_t)encoder_get_right_ticks();

    RCSOFTCHECK(rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL));
    RCSOFTCHECK(rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL));
}

static void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * twist_msg =
        (const geometry_msgs__msg__Twist *)msgin;

    rover_control_set_cmd_vel(
        twist_msg->linear.x,
        twist_msg->angular.z
    );
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
        &left_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "left_encoder_ticks"));

    RCCHECK(rclc_publisher_init_default(
        &right_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "right_encoder_ticks"));

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

    left_encoder_msg.data = 0;
    right_encoder_msg.data = 0;

    while (1) {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&left_encoder_publisher, &node));
    RCCHECK(rcl_publisher_fini(&right_encoder_publisher, &node));
    RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
    RCCHECK(rcl_node_fini(&node));

    vTaskDelete(NULL);
}