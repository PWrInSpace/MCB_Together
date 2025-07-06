#include <stdio.h>
#include <esp_log.h>
#include <i2c.h>
#include <bmi08x_wrapper.h>

void app_main(void)
{
    ESP_LOGI("TEST", ":D");
    if(i2c_sensors_init()) {
        ESP_LOGI("TEST", "i2c initialised!");
    }

    if(bmi08_wrapper_init()) {
        ESP_LOGI("TEST-bmi088", "imu initialised!");
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    struct bmi08_sensor_data_f acc;
    struct bmi08_sensor_data_f gyro;
    while(1) {
        if(bmi08_acc_data_ready()) {
            bmi08_get_acc_data(&acc);
            
        }
        else{
            continue;
        }

        if(bmi08_gyro_data_ready()) {
            bmi08_get_gyro_data(&gyro);
        }

        ESP_LOGI("IMU", "Results from acc: %f, %f, %f and from gyro: %f, %f, %f", acc.x, acc.y, acc.z, gyro.x, gyro.y, gyro.z);
    }
}
