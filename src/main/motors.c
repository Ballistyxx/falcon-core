#include "motors.h"
#include <math.h>
#include "driver/ledc.h"
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
 *     left                    right
 *   M3 (forward) ........ M2 (forward)
 *   M4 (altitude) ....... M1 (altitude)
 *
 * All four motors are active and fully bidirectional. The ESP32-S3 has 8 LEDC
 * channels; four bidirectional H-bridges consume all eight (channels 0..7 on
 * timer 0), so there is no channel left for the camera XCLK. The camera is
 * disabled in hardware for now; if it is brought back, the XCLK channel
 * allocation in camera.c (timer 1 / channel 6) will collide with M4 and the
 * channel budget must be reworked.
 */
#define PWM_MOTORS MOTOR_COUNT

typedef struct { int in1; int in2; } motor_pins_t;

static const motor_pins_t s_pins[MOTOR_COUNT] = {
    { 17, 6  },   /* M1 altitude, right */
    { 3,  47 },   /* M2 forward,  right */
    { 5,  4  },   /* M3 forward,  left  */
    { 15, 16 },   /* M4 altitude, left  */
};

/* LEDC channel per IN pin: motor i uses channels 2i (IN1) and 2i+1 (IN2). */
static inline ledc_channel_t ch_in1(int m) { return (ledc_channel_t)(2 * m); }
static inline ledc_channel_t ch_in2(int m) { return (ledc_channel_t)(2 * m + 1); }

static void set_duty(ledc_channel_t ch, uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
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

    for (int m = 0; m < PWM_MOTORS; m++) {
        ledc_channel_config_t c1 = {
            .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER,
            .channel = ch_in1(m), .gpio_num = s_pins[m].in1,
            .duty = 0, .hpoint = 0, .intr_type = LEDC_INTR_DISABLE,
        };
        ledc_channel_config_t c2 = {
            .speed_mode = LEDC_MODE, .timer_sel = LEDC_TIMER,
            .channel = ch_in2(m), .gpio_num = s_pins[m].in2,
            .duty = 0, .hpoint = 0, .intr_type = LEDC_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&c1));
        ESP_ERROR_CHECK(ledc_channel_config(&c2));
    }

    ESP_LOGI(TAG, "LEDC ready: M1-M4 @ %d Hz, duty cap %.0f%%",
             LEDC_FREQ_HZ, MOTOR_MAX_DUTY_FRAC * 100.0f);
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void motors_apply(const float cmd[MOTOR_COUNT], float out[MOTOR_COUNT])
{
    for (int m = 0; m < PWM_MOTORS; m++) {
        float c = clampf(cmd[m], -1.0f, 1.0f);
        out[m] = c;

        uint32_t duty = (uint32_t)(fabsf(c) * MOTOR_MAX_DUTY_FRAC * MAX_DUTY);

        if (c > 0.0f) {              /* forward */
            set_duty(ch_in1(m), duty);
            set_duty(ch_in2(m), 0);
        } else if (c < 0.0f) {       /* reverse */
            set_duty(ch_in1(m), 0);
            set_duty(ch_in2(m), duty);
        } else {                     /* coast */
            set_duty(ch_in1(m), 0);
            set_duty(ch_in2(m), 0);
        }
    }
}

void motors_stop(void)
{
    for (int m = 0; m < PWM_MOTORS; m++) {
        set_duty(ch_in1(m), 0);
        set_duty(ch_in2(m), 0);
    }
}
