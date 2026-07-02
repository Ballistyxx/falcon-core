/*
 * Motor control: 4x DRV8212P H-bridges driven by LEDC PWM at 20 kHz.
 *
 * Each motor has two pins (IN1, IN2):
 *   cmd > 0 : IN1 = PWM(|cmd|), IN2 = LOW   (forward)
 *   cmd < 0 : IN1 = LOW,        IN2 = PWM   (reverse)
 *   cmd = 0 : IN1 = LOW,        IN2 = LOW   (coast)
 *
 * Commands are floats in [-1, 1]. The applied duty is |cmd| scaled by a
 * configurable safety cap (MOTOR_MAX_DUTY_FRAC).
 */

#ifndef FALCON_MOTORS_H_
#define FALCON_MOTORS_H_

#define MOTOR_COUNT 4

/* Configure LEDC timer + 8 channels. */
void motors_init(void);

/*
 * Apply a command vector. Each value is clamped to [-1, 1]; the clamped value
 * is written back to out[] so callers can publish the actual commanded output.
 */
void motors_apply(const float cmd[MOTOR_COUNT], float out[MOTOR_COUNT]);

/* Immediately coast all motors (used on safety stop / disconnect). */
void motors_stop(void);

#endif /* FALCON_MOTORS_H_ */
