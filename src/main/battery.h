/*
 * Battery monitor. Reads the resistive divider on IO7 (ADC1) and converts to
 * pack voltage and a linear state-of-charge estimate (3.0 V = 0%, 4.2 V = 100%).
 */

#ifndef FALCON_BATTERY_H_
#define FALCON_BATTERY_H_

#include <stdint.h>

/* Set up the ADC oneshot unit + calibration. */
void battery_init(void);

/* Sample the divider. Outputs pack voltage (V) and SoC percent (0..100). */
void battery_read(float *voltage, uint8_t *percent);

#endif /* FALCON_BATTERY_H_ */
