/*
 * Motor control: 4x DRV8212P H-bridges driven by LEDC PWM at 20 kHz.
 *
 * Each driver uses a single PWM channel plus a direction GPIO, so it stays at
 * one LEDC channel per motor (4 total) — leaving channels free for the camera
 * XCLK so motors and camera run simultaneously — while still giving full
 * bidirectional control:
 *
 *   cmd > 0 (CW):  IN2 = LOW,  IN1 = PWM(duty)      fast-decay forward
 *   cmd < 0 (CCW): IN2 = HIGH, IN1 = PWM(1 - duty)  drive on IN1-low; reverse
 *   cmd = 0:       IN2 = LOW,  IN1 = 0              coast
 *
 * The (1 - duty) term in reverse makes the average output voltage symmetric
 * with forward for the same |cmd|. Commands are floats in [-1, 1]; magnitude is
 * scaled by a configurable safety cap (MOTOR_MAX_DUTY_FRAC).
 */

#ifndef FALCON_MOTORS_H_
#define FALCON_MOTORS_H_

#define MOTOR_COUNT 4

/* Configure the LEDC timer + one PWM channel per motor and the IN2 direction
 * GPIOs. */
void motors_init(void);

/*
 * Apply a command vector. Each value is clamped to [-1, 1]; the clamped value
 * is written back to out[] so callers can publish the actual commanded output.
 */
void motors_apply(const float cmd[MOTOR_COUNT], float out[MOTOR_COUNT]);

/* Immediately coast all motors (used on safety stop / disconnect). */
void motors_stop(void);

#endif /* FALCON_MOTORS_H_ */
