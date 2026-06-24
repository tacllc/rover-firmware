#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
//#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#include <rmw_microros/custom_transport.h>


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define WHEEL_BASE_M 0.30f

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
volatile double last_linear_x = 0.0;
volatile double last_angular_z = 0.0;
volatile int cmd_vel_count = 99999;

// void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {
// 	// RCLC_UNUSED(last_call_time);
// 	// if (timer != NULL) {
// 	// 	RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
// 	// 	msg.data++;
// 	// }
// 	// printf("cmd_vel_count=%d\n", cmd_vel_count);
//     RCLC_UNUSED(last_call_time);
//     if (timer != NULL) {
//         msg.data = (int)(last_linear_x * 1000.0);
//         RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     }
// 	printf("cmd_vel_count=%d\n", cmd_vel_count);
// }

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer != NULL) {
        float linear = (float)last_linear_x;
        float angular = (float)last_angular_z;

        float left_mps  = linear - (angular * WHEEL_BASE_M / 2.0f);
        float right_mps = linear + (angular * WHEEL_BASE_M / 2.0f);

        msg.data = (int)(left_mps * 1000.0f);
        RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

		printf(
			"linear=%.2f angular=%.2f left=%.2f right=%.2f\n",
			linear,
			angular,
			left_mps,
			right_mps);		
    }
	printf("Loop\n");
}

void cmd_vel_callback(const void * msgin)
{
	cmd_vel_count++;

    const geometry_msgs__msg__Twist * twist_msg =
        (const geometry_msgs__msg__Twist *)msgin;

    last_linear_x = twist_msg->linear.x;
    last_angular_z = twist_msg->angular.z;

    printf("cmd_vel received: linear.x=%.3f angular.z=%.3f\n",
           last_linear_x,
           last_angular_z);
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32_int32_publisher", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"freertos_int32_publisher"));

	RCCHECK(rclc_subscription_init_best_effort(
		&cmd_vel_subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"));

	// create timer,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback,
		true));

	// create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	RCCHECK(rclc_executor_add_subscription(
		&executor,
		&cmd_vel_subscriber,
		&cmd_vel_msg,
		&cmd_vel_callback,
		ON_NEW_DATA));	

	msg.data = 0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// free resources
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

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
#endif  // RMW_UXRCE_TRANSPORT_CUSTOM
    xTaskCreate(
        micro_ros_task,
        "uros_task",
        16500,
        NULL,
        5,
        NULL
    );
}
