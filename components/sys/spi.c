#include "spi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

SemaphoreHandle_t mutex_spi;

static struct {
    spi_bus_config_t spi_bus;
    spi_host_device_t spi_host;
} spi;

bool spi_init(spi_host_device_t host, int dma_chan, uint8_t mosi, uint8_t miso, uint8_t sck) {
    esp_err_t ret;
    spi.spi_bus.mosi_io_num = mosi;
    spi.spi_bus.miso_io_num = miso;
    spi.spi_bus.sclk_io_num = sck;
    spi.spi_bus.quadwp_io_num = -1;
    spi.spi_bus.quadhd_io_num = -1;
    spi.spi_bus.max_transfer_sz = 4000;
    spi.spi_host = host;
    mutex_spi = xSemaphoreCreateMutex();
    ret = spi_bus_initialize(spi.spi_host, &spi.spi_bus, dma_chan);
    return ret == ESP_OK;
}
