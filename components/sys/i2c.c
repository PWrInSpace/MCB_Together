// Copyright 2025 PWrInSpace
#include <i2c.h>
#include <esp_log.h>
#define TAG "I2C"

static struct {
    i2c_t i2c_num_1;
    i2c_t i2c_num_2;
} gb;

bool I2C_master_init(i2c_t *i2c, i2c_port_t port, uint8_t sda, uint8_t scl, uint32_t freq_hz) {
    esp_err_t res;
    i2c->mode = I2C_MODE_MASTER;
    i2c->sda_io_num = sda;
    i2c->scl_io_num = scl;
    i2c->sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c->scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c->master.clk_speed = freq_hz;
    i2c->clk_flags = 0;

    res = i2c_param_config(port, i2c);
    res = i2c_driver_install(port, i2c->mode, 0, 0, 0);
    return res == ESP_OK;
}

bool I2C_master_write(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, 
                                size_t len) {
    esp_err_t res;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // think about static buffor

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    res = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));

    // try to handle timeout properly ~ Maciek

    i2c_cmd_link_delete(cmd);

    return res == ESP_OK;
}

bool I2C_master_read_reg(i2c_port_t port, uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                          size_t len) {
    esp_err_t res;
    uint8_t buffer[200] = { 0 };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create_static(buffer, sizeof(buffer));
    assert (cmd != NULL);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    res = i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "timeout");
        return false;
    }
    i2c_master_stop(cmd);

    res = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(10));

    i2c_cmd_link_delete(cmd);

    return res == ESP_OK;
}

static bool I2C_master_read_direct(i2c_port_t port, uint8_t dev_addr, uint8_t *data,
                          size_t len) {
    esp_err_t res;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);

    i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_READ, true);

    if (len > 1)
    {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    res = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(100));

    i2c_cmd_link_delete(cmd);

    return res == ESP_OK;
}

bool i2c_sensors_init(void) {
    return I2C_master_init(&gb.i2c_num_1, I2C_NUM_0, CONFIG_SDA_S, CONFIG_SCL_S, 40000);
}

bool i2c_com_init(void) {
    return I2C_master_init(&gb.i2c_num_2, I2C_NUM_1, CONFIG_SDA_COM, CONFIG_SCL_COM, 40000);
}

bool i2c_sensors_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                          size_t len) {
    if (I2C_master_read_reg(I2C_NUM_0, dev_addr, reg_addr, data, len)) {
        return true;
    }

    return false;
}

bool i2c_sensors_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data,
                          size_t len) {
    if (I2C_master_write(I2C_NUM_0, dev_addr, reg_addr, data, len)) {
        return true;
    }

    return false;
}

bool i2c_com_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                          size_t len) {
    if (I2C_master_read_reg(I2C_NUM_1, dev_addr, reg_addr, data, len)) {
        return true;
    }

    return false;
}


bool i2c_com_read_direct(uint8_t dev_addr, uint8_t *data, size_t len) {
    if (I2C_master_read_direct(I2C_NUM_1, dev_addr, data, len)) {
        return true;
    }

    return false;
}


bool i2c_com_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data,
                          size_t len) {
    if (I2C_master_write(I2C_NUM_1, dev_addr, reg_addr, data, len)) {
        return true;
    }

    return false;
}
