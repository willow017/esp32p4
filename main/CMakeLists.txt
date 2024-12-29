set(sources_file
    "main.c"

    "drivers/ws2812/ws2812.c"
    "drivers/lvgl/lvgl_init.c"
)

set(include_dir
    "."
    "drivers/ws2812"
    "drivers/lvgl"
)

idf_component_register( SRCS ${sources_file}
                        INCLUDE_DIRS ${include_dir}
                        
                        REQUIRES esp_lcd)
