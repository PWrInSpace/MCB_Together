idf_component_register(SRCS "MCB_Together.c"
                    INCLUDE_DIRS "."
                    REQUIRES sys sensors)

idf_build_set_property(COMPILE_OPTIONS "-Wall" "-Wextra" "-Wno-unused-function" "-Werror" APPEND)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format -Wall -W -Werror)
