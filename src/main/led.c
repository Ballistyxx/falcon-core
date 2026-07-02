#include "led.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_GPIO GPIO_NUM_48

static volatile led_mode_t s_mode = LED_CONNECTING;

static void led_task(void *arg)
{
    (void)arg;
    bool level = false;
    while (1) {
        switch (s_mode) {
        case LED_RUNNING:
            gpio_set_level(LED_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        case LED_ERROR:               /* 5 Hz: 100 ms on / 100 ms off */
            level = !level;
            gpio_set_level(LED_GPIO, level);
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        case LED_CONNECTING:          /* 1 Hz: 500 ms on / 500 ms off */
        default:
            level = !level;
            gpio_set_level(LED_GPIO, level);
            vTaskDelay(pdMS_TO_TICKS(500));
            break;
        }
    }
}

void led_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    xTaskCreate(led_task, "led_task", 2048, NULL, 2, NULL);
}

void led_set_mode(led_mode_t mode)
{
    s_mode = mode;
}
