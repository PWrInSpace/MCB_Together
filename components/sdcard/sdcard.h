// Copyright 2022 PWr in Space, Kuba
#ifndef SD_CARD_H
#define SD_CARD_H

#include <string.h>
#include <stdbool.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define SDCARD_MOUNT_POINT CONFIG_SD_MOUNT_POINT
#define SD_CREATE_FILE_PREFIX(usr_path) SDCARD_MOUNT_POINT "/" usr_path
// 8 is a placeholder for _number_.txt
#define PATH_FLIE_SIZE(usr_path) sizeof(SD_CREATE_FILE_PREFIX(usr_path)) + 8

typedef struct {
    sdmmc_card_t *card;
    spi_host_device_t spi_host;
    const char* mount_point;
    uint8_t cs_pin;
    uint8_t card_detect_pin;
    bool mounted;
} sd_card_t;

typedef struct {
    spi_host_device_t spi_host;
    const char *mount_point;
    uint8_t cs_pin;
    uint8_t cd_pin;
} sd_card_config_t;

/**
 * @brief Initialize sd card
 *
 * @param sd_card pointer to sd_card_t struct
 * @param host spi_host device
 * @param cs chip select pin
 * @param m_point mounting point
 * @return true :D
 * @return false :C
 */
bool SD_init(sd_card_t *sd_card, sd_card_config_t *cfg);

/**
 * @brief Write string to sd card
 *
 * @param sd_card pointer to sd_card_t struct
 * @param path path to file
 * @param data data
 * @param length data length
 * @return true :D
 * @return false :C
 */
bool SD_write(sd_card_t *sd_card, const char* path, const char* data, size_t length);

/**
 * @brief Check if file exists
 *
 * @param path path to file
 * @return true exists
 * @return false
 */
bool SD_file_exists(const char *path);

/**
 * @brief Mount sd card in case of unmounted
 *
 * @param sd_card sd card struct
 * @return true :D
 * @return false :C
 */
bool SD_mount(sd_card_t *sd_card);

/**
 * @brief Remount sd card
 *
 * @param sd_card pointer to sd_card_t struct
 * @return true :D
 * @return false :C
 */
bool SD_remount(sd_card_t *sd_card);

/**
 * @brief Unmount sd card
 *
 * @param sd_card pointer to sd_card_t struct
 * @return true :D
 * @return false :C
 */
bool SD_unmount(sd_card_t *sd_card);


/**
 * @brief Check what's up
 *
 * @param sd_card sd struct
 * @return true sd is fine
 * @return false sd is sad :C
 */
bool SD_is_ok(sd_card_t *sd_card);


/**
 * @brief Create a unique path to file object
 * 
 * @param file_path name of file
 * @param size size of file path buffer
 * @return true unique path created
 * @return false can not create unique, using file_path
 */
bool create_path_to_file(char *file_path, size_t size);

#endif
