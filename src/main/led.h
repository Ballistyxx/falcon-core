/*
 * Status LED (GPIO48, red, 1k to GND).
 *
 *   LED_CONNECTING : slow 1 Hz blink  – joining WiFi
 *   LED_RUNNING    : solid on         – connected and running
 *   LED_ERROR      : fast 5 Hz blink  – error state
 */

#ifndef FALCON_LED_H_
#define FALCON_LED_H_

typedef enum {
    LED_CONNECTING = 0,
    LED_RUNNING,
    LED_ERROR,
} led_mode_t;

/* Configure the GPIO and start the blink task. */
void led_init(void);

/* Switch the active pattern (safe to call from any task). */
void led_set_mode(led_mode_t mode);

#endif /* FALCON_LED_H_ */
