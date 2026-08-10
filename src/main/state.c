#include "state.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static falcon_state_t s_state;
static SemaphoreHandle_t s_mutex;

#define LOCK()   xSemaphoreTake(s_mutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(s_mutex)

void state_init(void)
{
    memset(&s_state, 0, sizeof(s_state));
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex != NULL);
}

void state_snapshot(falcon_state_t *out)
{
    LOCK();
    *out = s_state;
    UNLOCK();
}

void state_set_imu(const float accel[3], const float gyro[3])
{
    LOCK();
    memcpy(s_state.accel, accel, sizeof(s_state.accel));
    memcpy(s_state.gyro, gyro, sizeof(s_state.gyro));
    UNLOCK();
}

void state_set_mag(const float mag[3])
{
    LOCK();
    memcpy(s_state.mag, mag, sizeof(s_state.mag));
    UNLOCK();
}

void state_set_attitude(float roll, float pitch, float yaw, float heading_deg)
{
    LOCK();
    s_state.roll = roll;
    s_state.pitch = pitch;
    s_state.yaw = yaw;
    s_state.heading_deg = heading_deg;
    UNLOCK();
}

void state_set_tof(uint16_t altitude_mm, const uint16_t grid[TOF_GRID_ZONES])
{
    LOCK();
    s_state.altitude_mm = altitude_mm;
    memcpy(s_state.tof_grid, grid, sizeof(s_state.tof_grid));
    UNLOCK();
}

void state_set_battery(float voltage, uint8_t percent)
{
    LOCK();
    s_state.battery_voltage = voltage;
    s_state.battery_percent = percent;
    UNLOCK();
}

void state_set_motor_output(const float output[4])
{
    LOCK();
    memcpy(s_state.motor_output, output, sizeof(s_state.motor_output));
    UNLOCK();
}

void state_set_motor_cmd(const float cmd[4])
{
    LOCK();
    memcpy(s_state.motor_cmd, cmd, sizeof(s_state.motor_cmd));
    UNLOCK();
}

void state_get_motor_cmd(float cmd[4])
{
    LOCK();
    memcpy(cmd, s_state.motor_cmd, sizeof(s_state.motor_cmd));
    UNLOCK();
}
