#include "magcal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "magcal";

#define NVS_NS  "magcal"
#define NVS_KEY "cal"

static magcal_t s_cal = {
    .offset = { 0.0f, 0.0f, 0.0f },
    .scale  = { 1.0f, 1.0f, 1.0f },
};
static SemaphoreHandle_t s_lock;

void magcal_init(void)
{
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no stored calibration; using identity");
        return;
    }
    magcal_t tmp;
    size_t len = sizeof(tmp);
    if (nvs_get_blob(h, NVS_KEY, &tmp, &len) == ESP_OK && len == sizeof(tmp)) {
        s_cal = tmp;
        ESP_LOGI(TAG, "loaded cal off=[%.1f %.1f %.1f] scl=[%.3f %.3f %.3f]",
                 s_cal.offset[0], s_cal.offset[1], s_cal.offset[2],
                 s_cal.scale[0], s_cal.scale[1], s_cal.scale[2]);
    } else {
        ESP_LOGI(TAG, "no stored calibration; using identity");
    }
    nvs_close(h);
}

void magcal_get(magcal_t *out)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    *out = s_cal;
    xSemaphoreGive(s_lock);
}

void magcal_set(const magcal_t *cal)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    s_cal = *cal;
    xSemaphoreGive(s_lock);

    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGW(TAG, "nvs_open failed; calibration not persisted");
        return;
    }
    esp_err_t e = nvs_set_blob(h, NVS_KEY, cal, sizeof(*cal));
    if (e == ESP_OK) {
        e = nvs_commit(h);
    }
    nvs_close(h);
    ESP_LOGI(TAG, "calibration %s: off=[%.1f %.1f %.1f] scl=[%.3f %.3f %.3f]",
             e == ESP_OK ? "saved" : "SAVE FAILED",
             cal->offset[0], cal->offset[1], cal->offset[2],
             cal->scale[0], cal->scale[1], cal->scale[2]);
}

void magcal_apply(const float raw[3], float out[3])
{
    xSemaphoreTake(s_lock, portMAX_DELAY);
    for (int i = 0; i < 3; i++) {
        out[i] = (raw[i] - s_cal.offset[i]) * s_cal.scale[i];
    }
    xSemaphoreGive(s_lock);
}
