/*
 * Attitude estimation. A lightweight complementary filter fuses the
 * accelerometer (gravity reference for roll/pitch) with integrated gyro rates.
 * Heading is the raw magnetometer atan2 for now (no tilt compensation yet);
 * yaw tracks heading. The interface is deliberately stateless to the caller so
 * a Madgwick implementation can be dropped in later without touching tasks.
 */

#ifndef FALCON_FUSION_H_
#define FALCON_FUSION_H_

void fusion_init(void);

/*
 * Advance the estimate by dt seconds.
 *   accel : m/s^2 (or g; only the direction matters for roll/pitch)
 *   gyro  : deg/s
 *   mag   : raw magnetometer units
 * Outputs are in degrees. heading is 0..360.
 */
void fusion_update(const float accel[3], const float gyro[3], const float mag[3],
                   float dt,
                   float *roll, float *pitch, float *yaw, float *heading);

#endif /* FALCON_FUSION_H_ */
