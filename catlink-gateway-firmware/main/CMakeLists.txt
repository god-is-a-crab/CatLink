file(GLOB_RECURSE srcs "main.c" "src/*.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "./include"
                       PRIV_REQUIRES
                        bt nvs_flash
                        esp_driver_gpio esp_driver_spi esp_adc)
