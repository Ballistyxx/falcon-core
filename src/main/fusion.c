#include "fusion.h"
#include <math.h>

#define RAD2DEG (180.0f / (float)M_PI)
#define DEG2RAD ((float)M_PI / 180.0f)

/* Complementary blend: weight on the gyro-integrated estimate per step. */
#define ALPHA 0.98f

static float s_roll;   /* deg */
static float s_pitch;  /* deg */

void fusion_init(void)
{
    s_roll = 0.0f;
    s_pitch = 0.0f;
}

void fusion_update(const float accel[3], const float gyro[3], const float mag[3],
                   float dt,
                   float *roll, float *pitch, float *yaw, float *heading)
{
    /* The board is mounted upside-down (180° roll about the forward axis, which
     * is sensor Y since roll fuses gyro[1]). That rotation negates sensor X and
     * Z; applying the same negation here brings the readings back to an
     * upright-equivalent frame. It's a proper rotation, so axis directions are
     * preserved. */
    float ax = -accel[0], ay = accel[1], az = -accel[2];
    float gx = -gyro[0],  gy = gyro[1];

    /* Roll/pitch from gravity vector. The IMU's X and Y axes are transposed
     * relative to the airframe, so roll comes from the X-gravity component
     * (gyro Y) and pitch from the Y/Z components (gyro X) — swapped from the
     * textbook mapping to match the physical mounting. */
    float acc_roll  = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
    float acc_pitch = atan2f(ay, az) * RAD2DEG;

    /* Integrate gyro (deg/s) then blend toward the accel reference. */
    s_roll  = ALPHA * (s_roll  + gy * dt) + (1.0f - ALPHA) * acc_roll;
    s_pitch = ALPHA * (s_pitch + gx * dt) + (1.0f - ALPHA) * acc_pitch;

    /* Heading: raw atan2 of the horizontal magnetometer components.
     * Tilt compensation is deferred (see instructions). */
    float h = atan2f(mag[1], mag[0]) * RAD2DEG + 45.0f;  /* mounting offset */
    h = fmodf(h, 360.0f);
    if (h < 0.0f) h += 360.0f;

    *roll = s_roll;
    *pitch = s_pitch;
    *yaw = h;
    *heading = h;
}
