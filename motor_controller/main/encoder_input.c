#include "encoder_input.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include "esp_attr.h"

#define LEFT_ENC_A_GPIO   GPIO_NUM_32
#define LEFT_ENC_B_GPIO   GPIO_NUM_33
#define RIGHT_ENC_A_GPIO  GPIO_NUM_34
#define RIGHT_ENC_B_GPIO  GPIO_NUM_35

static const char *TAG = "encoder_input";

static volatile int64_t left_ticks = 0;
static volatile int64_t right_ticks = 0;

static void IRAM_ATTR left_encoder_isr(void *arg)
{
    int a = gpio_get_level(LEFT_ENC_A_GPIO);
    int b = gpio_get_level(LEFT_ENC_B_GPIO);

    if (a == b) {
        left_ticks++;
    } else {
        left_ticks--;
    }
}

static void IRAM_ATTR right_encoder_isr(void *arg)
{
    int a = gpio_get_level(RIGHT_ENC_A_GPIO);
    int b = gpio_get_level(RIGHT_ENC_B_GPIO);

    if (a == b) {
        right_ticks++;
    } else {
        right_ticks--;
    }
}

void encoder_input_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL << LEFT_ENC_A_GPIO) |
            (1ULL << LEFT_ENC_B_GPIO) |
            (1ULL << RIGHT_ENC_A_GPIO) |
            (1ULL << RIGHT_ENC_B_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);

    gpio_set_intr_type(LEFT_ENC_A_GPIO, GPIO_INTR_ANYEDGE);
    gpio_set_intr_type(RIGHT_ENC_A_GPIO, GPIO_INTR_ANYEDGE);

    gpio_install_isr_service(0);

    gpio_isr_handler_add(LEFT_ENC_A_GPIO, left_encoder_isr, NULL);
    gpio_isr_handler_add(RIGHT_ENC_A_GPIO, right_encoder_isr, NULL);

    ESP_LOGI(TAG, "Encoder input initialized");
    ESP_LOGI(TAG, "Left: A=D32/GPIO32, B=D33/GPIO33");
    ESP_LOGI(TAG, "Right: A=D34/GPIO34, B=D35/GPIO35");
}

int64_t encoder_get_left_ticks(void)
{
    return left_ticks;
}

int64_t encoder_get_right_ticks(void)
{
    return right_ticks;
}

void encoder_reset_ticks(void)
{
    left_ticks = 0;
    right_ticks = 0;
}
