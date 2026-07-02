#include "sensors.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

#include "bmi270.h"
#include "vl53l5cx_api.h"

static const char *TAG = "sensors";

/* ---- Bus / pinout ------------------------------------------------------- */
#define I2C_PORT      I2C_NUM_0
#define I2C_SDA_GPIO  8
#define I2C_SCL_GPIO  18
#define I2C_FREQ_HZ   400000

#define ADDR_BMI270   0x68
#define ADDR_MMC5633  0x30
#define ADDR_VL53L5CX 0x29

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_bmi_dev;
static i2c_master_dev_handle_t s_mmc_dev;
static i2c_master_dev_handle_t s_tof_dev;

static bool s_imu_ok, s_mag_ok, s_tof_ok;

/* ---- BMI270 ------------------------------------------------------------- */
static struct bmi2_dev s_bmi;

static BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg, uint8_t *data,
                                           uint32_t len, void *intf)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf;
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, 1000) == ESP_OK
               ? BMI2_OK : BMI2_E_COM_FAIL;
}

static BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg, const uint8_t *data,
                                            uint32_t len, void *intf)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)intf;
    uint8_t buf[260];
    if (len > sizeof(buf) - 1) {
        return BMI2_E_COM_FAIL;
    }
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(dev, buf, len + 1, 1000) == ESP_OK
               ? BMI2_OK : BMI2_E_COM_FAIL;
}

static void bmi2_delay_us(uint32_t period, void *intf)
{
    (void)intf;
    esp_rom_delay_us(period);
}

static esp_err_t bmi270_setup(void)
{
    s_bmi.intf = BMI2_I2C_INTF;
    s_bmi.read = bmi2_i2c_read;
    s_bmi.write = bmi2_i2c_write;
    s_bmi.delay_us = bmi2_delay_us;
    s_bmi.intf_ptr = s_bmi_dev;
    s_bmi.read_write_len = 240;
    s_bmi.config_file_ptr = NULL;   /* use the driver's built-in config blob */

    int8_t rslt = bmi270_init(&s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "bmi270_init failed (%d)", rslt);
        return ESP_FAIL;
    }

    struct bmi2_sens_config cfg[2];
    cfg[0].type = BMI2_ACCEL;
    cfg[1].type = BMI2_GYRO;
    if (bmi2_get_sensor_config(cfg, 2, &s_bmi) != BMI2_OK) {
        return ESP_FAIL;
    }

    cfg[0].cfg.acc.odr = BMI2_ACC_ODR_100HZ;
    cfg[0].cfg.acc.range = BMI2_ACC_RANGE_2G;
    cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
    cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

    cfg[1].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;
    cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_500;
    cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
    cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
    cfg[1].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

    if (bmi2_set_sensor_config(cfg, 2, &s_bmi) != BMI2_OK) {
        return ESP_FAIL;
    }

    uint8_t sens[2] = { BMI2_ACCEL, BMI2_GYRO };
    if (bmi2_sensor_enable(sens, 2, &s_bmi) != BMI2_OK) {
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BMI270 ready (±2g, ±500dps, 100Hz)");
    return ESP_OK;
}

bool sensors_read_imu(float accel[3], float gyro[3])
{
    if (!s_imu_ok) {
        return false;
    }
    struct bmi2_sens_data data;
    if (bmi2_get_sensor_data(&data, &s_bmi) != BMI2_OK) {
        return false;
    }
    /* ±2g over int16 -> m/s^2 ; ±500dps over int16 -> deg/s */
    const float acc_scale = (2.0f / 32768.0f) * 9.80665f;
    const float gyr_scale = 500.0f / 32768.0f;
    accel[0] = data.acc.x * acc_scale;
    accel[1] = data.acc.y * acc_scale;
    accel[2] = data.acc.z * acc_scale;
    gyro[0] = data.gyr.x * gyr_scale;
    gyro[1] = data.gyr.y * gyr_scale;
    gyro[2] = data.gyr.z * gyr_scale;
    return true;
}

/* ---- MMC5633NJL --------------------------------------------------------- */
#define MMC_REG_XOUT0   0x00
#define MMC_REG_CTRL0   0x1B
#define MMC_REG_CTRL1   0x1C
#define MMC_REG_CTRL2   0x1D
#define MMC_REG_ODR     0x1A
#define MMC_REG_PRODID  0x39
#define MMC_PRODUCT_ID  0x10

#define MMC_CTRL0_TM_M       0x01
#define MMC_CTRL0_SET        0x08
#define MMC_CTRL0_RESET      0x10
#define MMC_CTRL0_AUTO_SR    0x20
#define MMC_CTRL0_CMM_FREQ   0x80
#define MMC_CTRL1_SW_RST     0x80
#define MMC_CTRL2_CMM_EN     0x10

static esp_err_t mmc_write(uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(s_mmc_dev, b, 2, 1000);
}

static esp_err_t mmc_read(uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(s_mmc_dev, &reg, 1, buf, len, 1000);
}

static esp_err_t mmc5633_setup(void)
{
    uint8_t id = 0;
    if (mmc_read(MMC_REG_PRODID, &id, 1) != ESP_OK || id != MMC_PRODUCT_ID) {
        ESP_LOGE(TAG, "MMC5633 product id mismatch (0x%02x)", id);
        return ESP_FAIL;
    }

    mmc_write(MMC_REG_CTRL1, MMC_CTRL1_SW_RST);  /* software reset */
    vTaskDelay(pdMS_TO_TICKS(20));

    mmc_write(MMC_REG_CTRL0, MMC_CTRL0_SET);     /* SET to align magnetics */
    vTaskDelay(pdMS_TO_TICKS(1));
    mmc_write(MMC_REG_CTRL0, MMC_CTRL0_RESET);   /* RESET */
    vTaskDelay(pdMS_TO_TICKS(1));

    mmc_write(MMC_REG_ODR, 50);                  /* 50 Hz output rate */
    /* Apply ODR for continuous mode + automatic SET/RESET each measurement. */
    mmc_write(MMC_REG_CTRL0, MMC_CTRL0_CMM_FREQ | MMC_CTRL0_AUTO_SR);
    mmc_write(MMC_REG_CTRL2, MMC_CTRL2_CMM_EN);  /* start continuous mode */

    ESP_LOGI(TAG, "MMC5633 ready (continuous 50Hz, auto SET/RESET)");
    return ESP_OK;
}

bool sensors_read_mag(float mag[3])
{
    if (!s_mag_ok) {
        return false;
    }
    uint8_t b[9];
    if (mmc_read(MMC_REG_XOUT0, b, 9) != ESP_OK) {
        return false;
    }
    /* 20-bit unsigned, null field at 2^19. 16384 counts/Gauss -> uT. */
    int32_t x = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[6] >> 4);
    int32_t y = ((int32_t)b[2] << 12) | ((int32_t)b[3] << 4) | (b[7] >> 4);
    int32_t z = ((int32_t)b[4] << 12) | ((int32_t)b[5] << 4) | (b[8] >> 4);
    const int32_t center = 1 << 19;
    const float scale = 100.0f / 16384.0f;   /* Gauss->uT over counts/Gauss */
    mag[0] = (x - center) * scale;
    mag[1] = (y - center) * scale;
    mag[2] = (z - center) * scale;
    return true;
}

/* ---- VL53L5CX ----------------------------------------------------------- */
static VL53L5CX_Configuration s_tof;
/* Center zone index for a 4x4 grid (zones 0..15, row-major). */
#define TOF_CENTER_ZONE 5

static esp_err_t vl53_setup(void)
{
    s_tof.platform.dev = s_tof_dev;
    s_tof.platform.address = 0x52;   /* 8-bit form; unused by the handle path */

    uint8_t alive = 0;
    if (vl53l5cx_is_alive(&s_tof, &alive) != 0 || !alive) {
        ESP_LOGE(TAG, "VL53L5CX not alive");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "VL53L5CX uploading firmware (~84KB)...");
    if (vl53l5cx_init(&s_tof) != 0) {
        ESP_LOGE(TAG, "vl53l5cx_init failed");
        return ESP_FAIL;
    }
    vl53l5cx_set_resolution(&s_tof, VL53L5CX_RESOLUTION_4X4);
    vl53l5cx_set_ranging_frequency_hz(&s_tof, 10);
    vl53l5cx_start_ranging(&s_tof);

    ESP_LOGI(TAG, "VL53L5CX ranging (4x4 @ 10Hz)");
    return ESP_OK;
}

bool sensors_read_tof(uint16_t *altitude_mm, uint16_t grid[TOF_GRID_ZONES])
{
    if (!s_tof_ok) {
        return false;
    }
    uint8_t ready = 0;
    if (vl53l5cx_check_data_ready(&s_tof, &ready) != 0 || !ready) {
        return false;
    }
    VL53L5CX_ResultsData res;
    if (vl53l5cx_get_ranging_data(&s_tof, &res) != 0) {
        return false;
    }
    for (int z = 0; z < TOF_GRID_ZONES; z++) {
        int idx = VL53L5CX_NB_TARGET_PER_ZONE * z;
        uint8_t status = res.target_status[idx];
        int16_t d = res.distance_mm[idx];
        /* Status 5 = 100% valid, 9 = 50% valid (single target); treat the rest
         * as no return so the dashboard can grey those zones out. */
        grid[z] = ((status == 5 || status == 9) && d > 0) ? (uint16_t)d : 0;
    }
    *altitude_mm = grid[TOF_CENTER_ZONE];
    return true;
}

/* ---- Init --------------------------------------------------------------- */
bool sensors_imu_ok(void) { return s_imu_ok; }
bool sensors_mag_ok(void) { return s_mag_ok; }
bool sensors_tof_ok(void) { return s_tof_ok; }

esp_err_t sensors_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    dcfg.device_address = ADDR_BMI270;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dcfg, &s_bmi_dev));
    dcfg.device_address = ADDR_MMC5633;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dcfg, &s_mmc_dev));
    dcfg.device_address = ADDR_VL53L5CX;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dcfg, &s_tof_dev));

    s_imu_ok = (bmi270_setup() == ESP_OK);
    s_mag_ok = (mmc5633_setup() == ESP_OK);
    s_tof_ok = (vl53_setup() == ESP_OK);

    ESP_LOGI(TAG, "sensors init: imu=%d mag=%d tof=%d",
             s_imu_ok, s_mag_ok, s_tof_ok);
    return ESP_OK;
}
