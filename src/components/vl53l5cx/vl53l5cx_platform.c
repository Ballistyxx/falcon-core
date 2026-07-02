/*
 * ESP-IDF platform layer for the VL53L5CX ULD driver.
 *
 * The sensor exposes a flat 16-bit, big-endian register/memory space. Every
 * access is framed as [addr_hi, addr_lo, payload...]. Large transfers (the
 * 32 KB firmware upload during init, multi-KB result reads) are split into
 * bounded chunks; the device auto-increments its internal address pointer, so
 * each chunk simply carries an advanced register address.
 */

#include "vl53l5cx_platform.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Per-chunk payload cap. Keeps the framing buffer on the stack small while
 * still moving the firmware blob in a reasonable number of transactions. */
#define VL53_CHUNK         1024
#define VL53_I2C_TIMEOUT_MS 1000

uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t reg_addr,
                uint8_t *p_values, uint32_t size)
{
    uint8_t frame[2 + VL53_CHUNK];
    uint32_t off = 0;

    while (off < size) {
        uint32_t n = size - off;
        if (n > VL53_CHUNK) {
            n = VL53_CHUNK;
        }
        uint16_t addr = (uint16_t)(reg_addr + off);
        frame[0] = (uint8_t)(addr >> 8);
        frame[1] = (uint8_t)(addr & 0xFF);
        memcpy(&frame[2], &p_values[off], n);

        if (i2c_master_transmit(p_platform->dev, frame, n + 2,
                                VL53_I2C_TIMEOUT_MS) != ESP_OK) {
            return 255;
        }
        off += n;
    }
    return 0;
}

uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t reg_addr,
                uint8_t *p_values, uint32_t size)
{
    uint32_t off = 0;

    while (off < size) {
        uint32_t n = size - off;
        if (n > VL53_CHUNK) {
            n = VL53_CHUNK;
        }
        uint16_t addr = (uint16_t)(reg_addr + off);
        uint8_t a[2] = { (uint8_t)(addr >> 8), (uint8_t)(addr & 0xFF) };

        if (i2c_master_transmit_receive(p_platform->dev, a, 2,
                                        &p_values[off], n,
                                        VL53_I2C_TIMEOUT_MS) != ESP_OK) {
            return 255;
        }
        off += n;
    }
    return 0;
}

uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t value)
{
    return WrMulti(p_platform, reg_addr, &value, 1);
}

uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t reg_addr, uint8_t *p_value)
{
    return RdMulti(p_platform, reg_addr, p_value, 1);
}

void SwapBuffer(uint8_t *buffer, uint16_t size)
{
    for (uint32_t i = 0; i < size; i += 4) {
        uint32_t w = ((uint32_t)buffer[i] << 24) |
                     ((uint32_t)buffer[i + 1] << 16) |
                     ((uint32_t)buffer[i + 2] << 8) |
                     ((uint32_t)buffer[i + 3]);
        buffer[i]     = (uint8_t)(w & 0xFF);
        buffer[i + 1] = (uint8_t)((w >> 8) & 0xFF);
        buffer[i + 2] = (uint8_t)((w >> 16) & 0xFF);
        buffer[i + 3] = (uint8_t)((w >> 24) & 0xFF);
    }
}

uint8_t Reset_Sensor(VL53L5CX_Platform *p_platform)
{
    (void)p_platform;
    /* No dedicated LPn/reset GPIO wired on Falcon Core. The sensor powers up
     * with the rail, so a software reset path is sufficient for init. */
    vTaskDelay(pdMS_TO_TICKS(100));
    return 0;
}

uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t time_ms)
{
    (void)p_platform;
    vTaskDelay(pdMS_TO_TICKS(time_ms == 0 ? 1 : time_ms));
    return 0;
}
