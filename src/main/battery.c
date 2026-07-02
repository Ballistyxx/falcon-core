#include "battery.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "battery";

/* IO7 -> ADC1 channel 6 on the ESP32-S3. */
#define BATT_ADC_UNIT     ADC_UNIT_1
#define BATT_ADC_CHANNEL  ADC_CHANNEL_6
/* 12 dB attenuation -> ~0..3.1 V usable range at the pin. */
#define BATT_ADC_ATTEN    ADC_ATTEN_DB_12

/* Resistive divider ratio (Vbatt / Vpin). Equal-resistor 2:1 divider. */
#define BATT_DIVIDER_RATIO 2.0f

#define BATT_FULL_V  4.2f
#define BATT_EMPTY_V 3.0f

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_cali;
static bool s_cali_ok;

void battery_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = BATT_ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, BATT_ADC_CHANNEL, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BATT_ADC_UNIT,
        .chan = BATT_ADC_CHANNEL,
        .atten = BATT_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    s_cali_ok = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) == ESP_OK);
    if (!s_cali_ok) {
        ESP_LOGW(TAG, "ADC calibration unavailable; using raw approximation");
    }
    ESP_LOGI(TAG, "battery ADC ready (IO7, divider %.1fx)", BATT_DIVIDER_RATIO);
}

void battery_read(float *voltage, uint8_t *percent)
{
    int raw = 0;
    if (adc_oneshot_read(s_adc, BATT_ADC_CHANNEL, &raw) != ESP_OK) {
        *voltage = 0.0f;
        *percent = 0;
        return;
    }

    int mv = 0;
    if (s_cali_ok) {
        adc_cali_raw_to_voltage(s_cali, raw, &mv);
    } else {
        /* Fallback: assume 12-bit full scale ~3.1 V at 12 dB. */
        mv = (int)((raw / 4095.0f) * 3100.0f);
    }

    float v = (mv / 1000.0f) * BATT_DIVIDER_RATIO;

    float pct = (v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    *voltage = v;
    *percent = (uint8_t)(pct + 0.5f);
}
