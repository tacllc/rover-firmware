idf_component_register(
    SRCS
        "motor_controller.c"
        "micro_ros_node.c"
        "rover_control.c"
        "motor_output.c"
        "esp32_serial_transport.c"
        "encoder_input.c"
    INCLUDE_DIRS "."
    REQUIRES micro_ros_espidf_component
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_timer esp_driver_ledc
)