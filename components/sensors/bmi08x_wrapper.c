#include <bmi08x_wrapper.h>
#include <stdint.h>
#include "bmi08_defs.h"
#include "bmi088_mm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <i2c.h>
#include <esp_log.h>
#include <esp_rom_sys.h>

#define TAG "IMU"
#define GRAVITY_EARTH  (9.80665f)

static bmi08_dev dev;

/*!
 * I2C read function map to sys/i2c
 */
static BMI08_INTF_RET_TYPE bmi08_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    // ESP_LOGI("I2C_READ", "Addr 0x%X, Reg 0x%X, Len %d", device_addr, reg_addr, len);
    return i2c_sensors_read_reg(device_addr, reg_addr, reg_data, len) ? BMI08_OK : BMI08_E_COM_FAIL;
}

/*!
 * I2C write function map to sys/i2c
 */
static BMI08_INTF_RET_TYPE bmi08_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t*)intf_ptr;
    // ESP_LOGI("I2C_Write", "Addr 0x%X, Reg 0x%X, Len %d", device_addr, reg_addr, len);
    return i2c_sensors_write(device_addr, reg_addr, reg_data, len)? BMI08_OK : BMI08_E_COM_FAIL;
}

static void bmi08_delay(uint32_t period, void *intf_ptr) {
    if (period >= 1000) {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    }

    uint32_t remaining_us = period % 1000;

    if (remaining_us > 0) {
        esp_rom_delay_us(remaining_us);
    }
}

/*!
 *  @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *  range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Value in Meter Per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

static void configure_accel_gyro_data_ready_interrupts(struct bmi08_dev *bmi08dev)
{
    int8_t rslt;
    struct bmi08_accel_int_channel_cfg accel_int_config;
    struct bmi08_gyro_int_channel_cfg gyro_int_config;

    /* Configure the Interrupt configurations for accel */
    accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
    accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Set the interrupt configuration */
    rslt = bmi088_mma_set_int_config(&accel_int_config, BMI088_MM_ACCEL_DATA_RDY_INT, bmi08dev);

    if (rslt == BMI08_OK)
    {
        /* Configure the Interrupt configurations for gyro */
        gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;
        gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_HIGH;
        gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;

        rslt = bmi08g_set_int_config(&gyro_int_config, bmi08dev);
    }
}

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Value in Degree Per Second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

bool bmi08_wrapper_init(void) {
    static uint8_t acc_addr = BMI08_ACCEL_I2C_ADDR_PRIMARY;
    static uint8_t gyro_addr = BMI08_GYRO_I2C_ADDR_PRIMARY;

    dev.write = bmi08_i2c_write;
    dev.read = bmi08_i2c_read;
    dev.delay_us = bmi08_delay;

    dev.intf = BMI08_I2C_INTF;
    dev.intf_ptr_accel = &acc_addr;
    dev.intf_ptr_gyro = &gyro_addr;
    dev.read_write_len = 32;

    // Init both sensors
    int8_t res_mma = bmi088_mma_init(&dev);
    if (res_mma != BMI08_OK) {
        ESP_LOGE(TAG, "bmi088_mma_init failed with code %d", res_mma);
        return false;
    }

    int8_t res_gyro = bmi08g_init(&dev);
    if (res_gyro != BMI08_OK) {
        ESP_LOGE(TAG, "bmi08g_init failed with code %d", res_gyro);
        return false;
    }

    // Soft reset acc
    if (bmi08a_soft_reset(&dev) != BMI08_OK) {
        ESP_LOGE(TAG, "soft restart");
        return false;
    }

    dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    if (bmi08a_set_power_mode(&dev) != BMI08_OK) {
        ESP_LOGE(TAG, "set power cfg");

        return false;
    }

    dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    if (bmi08g_set_power_mode(&dev) != BMI08_OK) {
        ESP_LOGE(TAG, "set gyro power mode");
        
        return false;
    }

    // Load config file - MUST be before any power/ODR setup
    if (bmi08a_load_config_file(&dev) != BMI08_OK) {
        ESP_LOGE(TAG, "load acc config");

        return false;
    }

    // Accel config
    dev.accel_cfg.odr = BMI08_ACCEL_ODR_100_HZ;
    dev.accel_cfg.range = BMI088_MM_ACCEL_RANGE_24G;
    dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL;
    dev.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    if (bmi08a_set_power_mode(&dev) != BMI08_OK) {
        ESP_LOGE(TAG, "set power cfg");

        return false;
    }

    // Set measurement config
    if (bmi088_mma_set_meas_conf(&dev) != BMI08_OK) {
        ESP_LOGE(TAG, "set measurement config");

        return false;
    }

    // Small delay to stabilize
    vTaskDelay(pdMS_TO_TICKS(50));

    // Gyro config
    dev.gyro_cfg.odr = BMI08_GYRO_BW_32_ODR_100_HZ;
    dev.gyro_cfg.range = BMI08_GYRO_RANGE_125_DPS;
    dev.gyro_cfg.bw = BMI08_GYRO_BW_32_ODR_100_HZ;
    dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

    if (bmi08g_set_meas_conf(&dev) != BMI08_OK) { 
        ESP_LOGE(TAG, "set measurements conf gyro");

        return false;
    }

    configure_accel_gyro_data_ready_interrupts(&dev);

    return true;
}

bool bmi08_acc_data_ready(void) {
    uint8_t status;
    if (bmi08a_get_data_int_status(&status, &dev) != BMI08_OK) {
        // ESP_LOGE(TAG, "Failed to read int acc status");
        return false;
    }

    return (status & BMI08_ACCEL_DATA_READY_INT);
}

bool bmi08_gyro_data_ready(void) {
    uint8_t status;
    if (bmi08g_get_data_int_status(&status, &dev) != BMI08_OK) {
        // ESP_LOGE(TAG, "Failed to read int gyro status");
        return false;
    }

    return (status & BMI08_GYRO_DATA_READY_INT);
}

bool bmi08_get_acc_data(struct bmi08_sensor_data_f *acc) {
    struct bmi08_sensor_data data;
    int8_t res = bmi088_mma_get_data(&data, &dev);
    if (res != BMI08_OK) {
        ESP_LOGE(TAG, "Failed to read acc data: %d", res);
        return false;
    }

    acc->x = lsb_to_mps2(data.x, (float)24, 16);
    acc->y = lsb_to_mps2(data.y, (float)24, 16);
    acc->z = lsb_to_mps2(data.z, (float)24, 16);

    return true;
}

bool bmi08_get_gyro_data(struct bmi08_sensor_data_f *gyro) {
    struct bmi08_sensor_data data;
    int8_t res = bmi08g_get_data(&data, &dev);
    if (res != BMI08_OK) {
        ESP_LOGE(TAG, "Failed to read acc data: %d", res);
        return false;
    }

    gyro->x = lsb_to_dps(data.x, (float)125, 16);
    gyro->y = lsb_to_dps(data.y, (float)125, 16);
    gyro->z = lsb_to_dps(data.z, (float)125, 16);

    return true;
}