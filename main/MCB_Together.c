#include <stdio.h>
#include <esp_log.h>
#include <i2c.h>

void app_main(void)
{
    ESP_LOGI("TEST", ":D");
    if(i2c_sensors_init()) {
        ESP_LOGI("TEST", "i2c initialised!");
    }
}
