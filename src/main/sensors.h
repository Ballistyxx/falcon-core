/*
 * Sensor subsystem: shared I2C master bus (IO8 SDA / IO18 SCL) with three
 * devices.
 *
 *   BMI270     0x68  6-axis IMU   (accel m/s^2, gyro deg/s)
 *   MMC5633NJL 0x30  magnetometer (uT, raw)
 *   VL53L5CX   0x29  multizone ToF (4x4 zone grid; center zone is altitude)
 *
 * sensors_init() brings up the bus and each device; the per-sensor read calls
 * return false if that device failed to initialize so tasks can degrade
 * gracefully.
 */

#ifndef FALCON_SENSORS_H_
#define FALCON_SENSORS_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

esp_err_t sensors_init(void);

/* Which devices came up. */
bool sensors_imu_ok(void);
bool sensors_mag_ok(void);
bool sensors_tof_ok(void);

/* accel[3] in m/s^2, gyro[3] in deg/s. Returns false on read error. */
bool sensors_read_imu(float accel[3], float gyro[3]);

/* mag[3] in microtesla (raw, uncalibrated). Returns false on read error. */
bool sensors_read_mag(float mag[3]);

/* Number of ToF zones (4x4 grid, row-major). */
#define TOF_GRID_ZONES 16

/* Reads one ToF frame. *altitude_mm gets the center-zone distance; grid[16]
 * gets every zone's distance in mm, row-major, with 0 marking a zone that had
 * no valid return. Returns false if no fresh frame is ready. */
bool sensors_read_tof(uint16_t *altitude_mm, uint16_t grid[TOF_GRID_ZONES]);

#endif /* FALCON_SENSORS_H_ */
