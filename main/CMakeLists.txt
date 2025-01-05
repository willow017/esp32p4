set(sources_file
    "main.c"

    "drivers/ws2812/ws2812.c"
    "drivers/lvgl/lvgl_init.c"
    "drivers/audio/audio_init.c"
    "drivers/mpu6050/mpu6050.c"
    "drivers/mcpwm/mcpwm.c"
)

set(include_dir
    "."
    "drivers/ws2812"
    "drivers/lvgl"
    "drivers/audio"
    "drivers/mpu6050"
    "drivers/mcpwm"
)

idf_component_register( SRCS ${sources_file}
                        INCLUDE_DIRS ${include_dir}
                        
                        REQUIRES esp_lcd fatfs spiffs nvs_flash unity)

spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)