/*
 * ESP-IDF platform layer for the STMicroelectronics VL53L5CX ULD driver.
 *
 * Replaces the Arduino "platform.h" shipped with the ULD. Provides the
 * VL53L5CX_Platform structure and the I2C read/write/delay primitives that the
 * driver core (vl53l5cx_api.c) calls. Backed by the ESP-IDF i2c_master driver.
 */

#ifndef VL53L5CX_PLATFORM_H_
#define VL53L5CX_PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "platform_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Platform descriptor. One per sensor instance, embedded inside
 * VL53L5CX_Configuration. The 16-bit `address` field is the 8-bit write
 * address (0x52 by default) as expected by the ULD; the underlying ESP-IDF
 * device handle is created with the 7-bit form (0x29).
 */
typedef struct {
    uint16_t              address;   /* 8-bit I2C address (0x52 default)   */
    i2c_master_dev_handle_t dev;     /* ESP-IDF device handle on the bus   */
} VL53L5CX_Platform;

/* ULD I2C primitives (implemented in vl53l5cx_platform.c). */
uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t *p_value);
uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t value);
uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t *p_values, uint32_t size);
uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t *p_values, uint32_t size);

/* Reverse the bytes of every 32-bit word in `buffer` (ULD endianness helper). */
void SwapBuffer(uint8_t *buffer, uint16_t size);

/* Hardware reset hook. No dedicated reset GPIO on Falcon Core, so this is a
 * no-op that simply succeeds. */
uint8_t Reset_Sensor(VL53L5CX_Platform *p_platform);

/* Blocking millisecond delay. */
uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t time_ms);

#ifdef __cplusplus
}
#endif

#endif /* VL53L5CX_PLATFORM_H_ */
