/*
 * Shared flight state.
 *
 * A single struct guarded by a FreeRTOS mutex. Producers (sensor, fusion, tof,
 * battery, motor tasks and the WS receive handler) write their own slice;
 * consumers (telemetry task, motor task) take a consistent snapshot. All access
 * goes through the short, copy-in/copy-out helpers below so critical sections
 * stay tiny.
 */

#ifndef FALCON_STATE_H_
#define FALCON_STATE_H_

#include <stdint.h>
#include "sensors.h"   /* TOF_GRID_ZONES */

typedef struct {
    /* Sensor data (written by sensor_task) */
    float accel[3];
    float gyro[3];
    float mag[3];

    /* Derived data (written by fusion_task) */
    float roll, pitch, yaw;
    float heading_deg;

    /* ToF (written by tof_task): center-zone altitude plus the full 8x8 depth
     * grid (mm, row-major; 0 = no valid return in that zone). */
    uint16_t altitude_mm;
    uint16_t tof_grid[TOF_GRID_ZONES];

    /* Battery (written by battery_task) */
    float battery_voltage;
    uint8_t battery_percent;

    /* Motor commands (written by WS receive handler), -1.0 .. +1.0 */
    float motor_cmd[4];

    /* Motor actual output (written by motor_task) */
    float motor_output[4];
} falcon_state_t;

/* Create the mutex. Call once from app_main before any task starts. */
void state_init(void);

/* Take a consistent snapshot of the whole state. */
void state_snapshot(falcon_state_t *out);

/* Per-producer setters (each writes only its own fields). */
void state_set_imu(const float accel[3], const float gyro[3]);
void state_set_mag(const float mag[3]);
void state_set_attitude(float roll, float pitch, float yaw, float heading_deg);
void state_set_tof(uint16_t altitude_mm, const uint16_t grid[TOF_GRID_ZONES]);
void state_set_battery(float voltage, uint8_t percent);
void state_set_motor_output(const float output[4]);

/* Motor command channel (WS handler writes, motor_task reads). */
void state_set_motor_cmd(const float cmd[4]);
void state_get_motor_cmd(float cmd[4]);

#endif /* FALCON_STATE_H_ */
