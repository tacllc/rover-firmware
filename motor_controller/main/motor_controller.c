#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/uart.h"

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/custom_transport.h>

#include "esp32_serial_transport.h"
#include "motor_output.h"
#include "encoder_input.h"
#include "micro_ros_node.h"

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
#endif

    motor_output_init();
    encoder_input_init();

    xTaskCreate(
        micro_ros_task,
        "uros_task",
        16500,
        NULL,
        5,
        NULL
    );
}