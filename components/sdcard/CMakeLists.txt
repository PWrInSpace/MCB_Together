idf_component_register(SRC_DIRS "."
                    INCLUDE_DIRS "."
                    REQUIRES fatfs nvs_flash spi_flash spiffs sys)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format" "-Wall" "-Werror")