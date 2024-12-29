/*
 * SPDX-FileCopyrightText: 2023-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "drivers/ws2812/ws2812.h"
#include "drivers/lvgl/lvgl_init.h"

static const char *TAG = "HK";

void bsp_init(void)
{
    bsp_ws2812_init();
}


void app_main(void)
{
    bsp_init();
    ws2812_taskinit();
    display_init();
}
