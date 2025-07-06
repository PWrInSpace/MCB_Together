idf_component_register(SRC_DIRS "." "BMI08x_SensorAPI"
                    INCLUDE_DIRS "." "BMI08x_SensorAPI"
                    REQUIRES cmock sys)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format" "-Wall" "-Werror")