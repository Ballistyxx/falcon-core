#include "motors.h"
#include <math.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "motors";

#define LEDC_MODE        LEDC_LOW_SPEED_MODE
#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_FREQ_HZ     20000
#define LEDC_RES         LEDC_TIMER_10_BIT
#define MAX_DUTY         ((1 << 10) - 1)   /* 1023 */

/* Safety cap on duty during bench testing: 0.0 .. 1.0 */
#define MOTOR_MAX_DUTY_FRAC 0.5f

/*
 * Physical motor layout (viewed from above, nose forward):
 *
 *     left                 right
 *   M1 (forward) ....... M3 (forward)
 *   M2 (vertical) ...... M4 (vertical)
 *
 * Single-channel bidirectional drive: each DRV8212P has IN1 on its own LEDC
 * channel (0..3 on timer 0) and IN2 as a plain GPIO used as a direction pin.
 * Four channels total, leaving channels 4..7 free — the camera XCLK (timer 1 /
 * channel 6) no longer collides, so motors and camera run at the same time.
 */
typedef struct { int in1; int in2; } motor_pins_t;

/* PCB pin map (IN1 is PWM'd, IN2 is the direction GPIO). */
static const motor_pins_t s_pins[MOTOR_COUNT] = {
    { 5,  4  },   /* M1 forward,  left  (wired to former M3 pins) */
    { 3,  17 },   /* M2 vertical, left  */
    { 47, 6  },   /* M3 forward,  right (wired to former M1 pins) */
    { 15, 16 },   /* M4 vertical, right */
};

/* Per-motor spin polarity (+1 normal, -1 reversed) to correct wiring so a
 * positive command always spins CW as commanded. M3 is reversed in hardware. */
static const float s_polarity[MOTOR_COUNT] = { 1.0f, 1.0f, -1.0f, 1.0f };

/* Motor i drives LEDC channel i on its IN1 pin. */
static inline ledc_channel_t ch(int m) { return (ledc_channel_t)m; }

static void set_duty(ledc_channel_t c, uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, c, duty);
    ledc_update_duty(LEDC_MODE, c);
}

void motors_init(void)
{
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    for (int m = 0; m < MOTOR_COUNT; m++) {
        /* IN1: PWM channel. */
        ledc_channel_config_t c1 = {
            .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER,
            .channel = ch(m), .gpio_num = s_pins[m].in1,
            .duty = 0, .hpoint = 0, .intr_type = LEDC_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c1));

        /* IN2: plain GPIO used as the direction pin (LOW = forward/CW). */
        gpio_config_t io = {
            .pin_bit_mask = 1ULL << s_pins[m].in2,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&io));
        gpio_set_level(s_pins[m].in2, 0);
    }

    ESP_LOGI(TAG, "LEDC ready: M1-M4 bidirectional @ %d Hz, duty cap %.0f%%",
             LEDC_FREQ_HZ, MOTOR_MAX_DUTY_FRAC * 100.0f);
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void motors_apply(const float cmd[MOTOR_COUNT], float out[MOTOR_COUNT])
{
    for (int m = 0; m < MOTOR_COUNT; m++) {
        float c = clampf(cmd[m] * s_polarity[m], -1.0f, 1.0f);
        out[m] = c;

        /* Average output magnitude (0 .. cap). */
        float mag = fabsf(c) * MOTOR_MAX_DUTY_FRAC;

        if (c >= 0.0f) {                 /* CW: forward, fast decay */
            gpio_set_level(s_pins[m].in2, 0);
            set_duty(ch(m), (uint32_t)(mag * MAX_DUTY));
        } else {                         /* CCW: reverse, drive on IN1-low */
            gpio_set_level(s_pins[m].in2, 1);
            set_duty(ch(m), (uint32_t)((1.0f - mag) * MAX_DUTY));
        }
    }
}

void motors_stop(void)
{
    for (int m = 0; m < MOTOR_COUNT; m++) {
        gpio_set_level(s_pins[m].in2, 0);
        set_duty(ch(m), 0);
    }
}
