#ifndef SIMPLE_SPI
#define SIMPLE_SPI

#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define SPI_MEM_MOSI CONFIG_SPI_MEM_MOSI
#define SPI_MEM_MISO CONFIG_SPI_MEM_MISO
#define SPI_MEM_SCK CONFIG_SPI_MEM_SCK

#define SPI_COM_MOSI CONFIG_SPI_COM_MOSI
#define SPI_COM_MISO CONFIG_SPI_COM_MISO
#define SPI_COM_SCK CONFIG_SPI_COM_SCK

extern SemaphoreHandle_t mutex_spi;
bool spi_init(spi_host_device_t host, int dma_chan, uint8_t mosi, uint8_t miso, uint8_t sck);
#endif
