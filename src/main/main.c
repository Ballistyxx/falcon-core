/*
 * Falcon Core – autonomous blimp flight controller firmware.
 *
 * app_main brings up every subsystem then launches the FreeRTOS task set that
 * reads sensors, fuses attitude, drives motors, and streams telemetry/video to
 * the laptop dashboard over WiFi.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "state.h"
#include "led.h"
#include "wifi.h"
#include "motors.h"
#include "battery.h"
#include "sensors.h"
#include "fusion.h"
#include "magcal.h"
#include "camera.h"
#include "server.h"

static const char *TAG = "falcon";

/* ---- Periodic tasks ----------------------------------------------------- */

static void sensor_task(void *arg)   /* core 0, 50 Hz */
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    float accel[3], gyro[3], mag[3];
    while (1) {
        if (sensors_read_imu(accel, gyro)) {
            state_set_imu(accel, gyro);
        }
        if (sensors_read_mag(mag)) {
            state_set_mag(mag);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    }
}

static void fusion_task(void *arg)   /* core 0, 50 Hz */
{
    (void)arg;
    const float dt = 0.02f;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        falcon_state_t s;
        state_snapshot(&s);
        /* Heading uses the calibrated magnetometer; telemetry still reports the
         * raw vector so the dashboard can (re)compute calibration from it. */
        float mag_cal[3];
        magcal_apply(s.mag, mag_cal);
        float roll, pitch, yaw, heading;
        fusion_update(s.accel, s.gyro, mag_cal, dt, &roll, &pitch, &yaw, &heading);
        state_set_attitude(roll, pitch, yaw, heading);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    }
}

static void motor_task(void *arg)    /* core 0, 50 Hz */
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        float cmd[4], out[4];
        state_get_motor_cmd(cmd);
        motors_apply(cmd, out);
        state_set_motor_output(out);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(20));
    }
}

static void tof_task(void *arg)      /* core 0, 10 Hz */
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        uint16_t mm;
        uint16_t grid[16];
        if (sensors_read_tof(&mm, grid)) {
            state_set_tof(mm, grid);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    }
}

static void battery_task(void *arg)  /* core 0, 1 Hz */
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    while (1) {
        float v;
        uint8_t pct;
        battery_read(&v, &pct);
        state_set_battery(v, pct);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
    }
}

static void telemetry_task(void *arg)  /* core 0, 20 Hz */
{
    (void)arg;
    static char json[900];
    TickType_t last = xTaskGetTickCount();
    while (1) {
        falcon_state_t s;
        state_snapshot(&s);
        magcal_t mc;
        magcal_get(&mc);
        uint32_t ts = (uint32_t)(esp_timer_get_time() / 1000);

        /* Flatten the 4x4 ToF grid into a JSON array (row-major, mm). */
        char grid_str[112];
        int gp = 0;
        grid_str[gp++] = '[';
        for (int i = 0; i < 16; i++) {
            gp += snprintf(grid_str + gp, sizeof(grid_str) - gp,
                           "%s%u", i ? "," : "", s.tof_grid[i]);
        }
        grid_str[gp++] = ']';
        grid_str[gp] = '\0';

        int n = snprintf(json, sizeof(json),
            "{\"type\":\"telemetry\",\"ts\":%lu,"
            "\"imu\":{\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,"
            "\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f},"
            "\"mag\":{\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"heading\":%.1f},"
            "\"magcal\":{\"ox\":%.2f,\"oy\":%.2f,\"oz\":%.2f,"
            "\"sx\":%.4f,\"sy\":%.4f,\"sz\":%.4f},"
            "\"tof\":{\"distance_mm\":%u,\"grid\":%s},"
            "\"battery\":{\"voltage\":%.3f,\"percent\":%u},"
            "\"wifi\":{\"rssi\":%d},"
            "\"attitude\":{\"roll\":%.1f,\"pitch\":%.1f,\"yaw\":%.1f},"
            "\"motors\":{\"m1\":%.2f,\"m2\":%.2f,\"m3\":%.2f,\"m4\":%.2f}}",
            (unsigned long)ts,
            s.accel[0], s.accel[1], s.accel[2],
            s.gyro[0], s.gyro[1], s.gyro[2],
            s.mag[0], s.mag[1], s.mag[2], s.heading_deg,
            mc.offset[0], mc.offset[1], mc.offset[2],
            mc.scale[0], mc.scale[1], mc.scale[2],
            s.altitude_mm, grid_str,
            s.battery_voltage, s.battery_percent,
            wifi_rssi(),
            s.roll, s.pitch, s.yaw,
            s.motor_output[0], s.motor_output[1],
            s.motor_output[2], s.motor_output[3]);

        if (n > 0) {
            server_ws_broadcast(json, (size_t)n);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
    }
}

/* ---- app_main ----------------------------------------------------------- */

void app_main(void)
{
    ESP_LOGI(TAG, "Falcon Core booting");

    esp_err_t nvs = nvs_flash_init();
    if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    state_init();
    magcal_init();
    led_init();
    led_set_mode(LED_CONNECTING);

    /* Camera first: its 20 MHz XCLK is generated by LEDC, and on the ESP32-S3
     * all LEDC low-speed timers share one clock source. Letting the camera
     * (the stricter consumer) pick that source before the 20 kHz motor PWM
     * guarantees a source fast enough for both. */
    if (camera_init() != ESP_OK) {
        ESP_LOGW(TAG, "camera unavailable; stream disabled");
    }

    motors_init();
    battery_init();
    fusion_init();

    if (sensors_init() != ESP_OK) {
        ESP_LOGE(TAG, "sensor init error");
    }

    wifi_init_sta();
    if (wifi_wait_connected(30000)) {
        ESP_LOGI(TAG, "WiFi up. Dashboard: http://%s/", wifi_ip_str());
        led_set_mode(LED_RUNNING);
    } else {
        ESP_LOGE(TAG, "WiFi connect timeout; continuing, will keep retrying");
        led_set_mode(LED_ERROR);
    }

    ESP_ERROR_CHECK(server_start());

    /* Tasks: (name, stack, prio, core) per the firmware architecture table. */
    xTaskCreatePinnedToCore(sensor_task,    "sensor",    4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(fusion_task,    "fusion",    4096, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(motor_task,     "motor",     3072, NULL, 6, NULL, 0);
    xTaskCreatePinnedToCore(tof_task,       "tof",       4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(battery_task,   "battery",   3072, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "telemetry", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "Falcon Core running");
}
