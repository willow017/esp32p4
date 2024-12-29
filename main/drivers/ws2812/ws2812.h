#ifndef _WS2812_H_
#define _WS2812_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"
#include "esp_err.h"

// GPIO assignment
#define LED_STRIP_GPIO_PIN  22

// Numbers of the LED in the strip
#define LED_STRIP_LED_COUNT 12
// 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define LED_STRIP_RMT_RES_HZ  (10 * 1000 * 1000)

void bsp_ws2812_init(void);
void ws2812_taskinit(void);

#endif
