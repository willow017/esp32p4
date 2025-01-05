#include "ws2812.h"

static const char *HK_WS2812 = "HK";
led_strip_handle_t led_strip = NULL;

TaskHandle_t ws2812_handle = NULL;
// HSV to RGB Conversion
static void hsv_to_rgb(uint16_t hue, uint8_t saturation, uint8_t value, uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint16_t h = hue % 360;
    uint8_t region = h / 60;
    uint8_t remainder = (h % 60) * 255 / 60;

    uint8_t p = (value * (255 - saturation)) / 255;
    uint8_t q = (value * (255 - (saturation * remainder) / 255)) / 255;
    uint8_t t = (value * (255 - (saturation * (255 - remainder)) / 255)) / 255;

    switch (region) {
        case 0:
            *r = value; *g = t; *b = p;
            break;
        case 1:
            *r = q; *g = value; *b = p;
            break;
        case 2:
            *r = p; *g = value; *b = t;
            break;
        case 3:
            *r = p; *g = q; *b = value;
            break;
        case 4:
            *r = t; *g = p; *b = value;
            break;
        case 5:
        default:
            *r = value; *g = p; *b = q;
            break;
    }
}

//  彩虹滚动效果
static void rainbow_effect(led_strip_handle_t led_strip)
{
    uint16_t base_hue = 0; // 基础色调

    while (1) {
        for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
            // 每个 LED 的色调相对于基础色调偏移
            uint16_t hue = (base_hue + i * (360 / LED_STRIP_LED_COUNT)) % 360;
            uint8_t red, green, blue;

            // HSV 转换为 RGB
            hsv_to_rgb(hue, 255, 255, &red, &green, &blue);

            // 设置颜色
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue));
        }

        // 刷新灯条
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

        // 更新基础色调
        base_hue = (base_hue + 3) % 360;

        // 延迟 50ms
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// 呼吸灯效果
static void breathing_effect(led_strip_handle_t led_strip, uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t brightness = 0;
    int8_t brightness_step = 1;

    while (1) {
        for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
            uint8_t r = (red * brightness) / 100;
            uint8_t g = (green * brightness) / 100;
            uint8_t b = (blue * brightness) / 100;

            // 设置颜色
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, r, g, b));
        }

        // 刷新灯条
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

        // 更新亮度
        brightness += brightness_step;
        if (brightness >= 100 || brightness <= 0) {
            brightness_step = -brightness_step; // 反转方向
        }

        // 延迟 30ms
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// 流星雨效果
static void meteor_effect(led_strip_handle_t led_strip, uint8_t red, uint8_t green, uint8_t blue, int tail_length, int delay_ms)
{
    while (1) {
        for (int head = 0; head < LED_STRIP_LED_COUNT + tail_length; head++) {
            for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
                if (i >= head - tail_length && i <= head) {
                    // 设置流星的颜色
                    uint8_t brightness = 255 - (255 * (head - i)) / tail_length;
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, (red * brightness) / 255, (green * brightness) / 255, (blue * brightness) / 255));
                } else {
                    // 清除其余部分
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                }
            }

            // 刷新灯条
            ESP_ERROR_CHECK(led_strip_refresh(led_strip));

            // 延迟
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
    }
}

// 跑马灯效果
static void running_light_effect(led_strip_handle_t led_strip, uint8_t red, uint8_t green, uint8_t blue, int delay_ms)
{
    int current_led = 0;

    while (1) {
        for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
            if (i == current_led) {
                // 点亮当前 LED
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue));
            } else {
                // 关闭其他 LED
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
            }
        }

        // 刷新灯条
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

        // 更新当前 LED
        current_led = (current_led + 1) % LED_STRIP_LED_COUNT;

        // 延迟
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

// 双色渐变
static void two_color_gradient_effect(led_strip_handle_t led_strip, uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2)
{
    while (1) {
        for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
            // 计算每个 LED 的颜色比例
            float ratio = (float)i / (LED_STRIP_LED_COUNT - 1);
            uint8_t red = r1 * (1 - ratio) + r2 * ratio;
            uint8_t green = g1 * (1 - ratio) + g2 * ratio;
            uint8_t blue = b1 * (1 - ratio) + b2 * ratio;

            // 设置颜色
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue));
        }

        // 刷新灯条
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

        // 延迟 100ms
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


led_strip_handle_t configure_led(void)
{
    // LED strip general initialization, according to your led board design
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP_GPIO_PIN, // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_LED_COUNT,      // The number of LEDs in the strip,
        .led_model = LED_MODEL_WS2812,        // LED strip model
        .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB, // The color order of the strip: GRB
        .flags = {
            .invert_out = false, // don't invert the output signal
        }
    };

    // LED strip backend configuration: RMT
    led_strip_rmt_config_t rmt_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,        // different clock source can lead to different power consumption
        .resolution_hz = LED_STRIP_RMT_RES_HZ, // RMT counter clock frequency
        .mem_block_symbols = 64,               // the memory size of each RMT channel, in words (4 bytes)
        .flags = {
            .with_dma = false, // DMA feature is available on chips like ESP32-S3/P4
        }
    };

    // LED Strip object handle
    led_strip_handle_t led_strip;
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_LOGI(HK_WS2812, "Created LED strip object with RMT backend");
    return led_strip;
}

void bsp_ws2812_init(void)
{
    led_strip = configure_led();
}

static void ws2812_handler(void *pvParameters)
{
    uint16_t base_hue = 0; // 基础色调
    const uint8_t saturation = 255; // 饱和度 (最大值 255)
    uint8_t brightness = 0; // 当前亮度
    int8_t brightness_step = 1; // 亮度变化步长（正值变亮，负值变暗）

    uint8_t red, green, blue;

    ESP_LOGI(HK_WS2812, "Start color and brightness cycling on LED strip");

    while (1) {
        for (int i = 0; i < LED_STRIP_LED_COUNT; i++) {
            // 计算每个 LED 的色调，基础色调 + 偏移
            uint16_t hue = (base_hue + i * (360 / LED_STRIP_LED_COUNT)) % 360;

            // 将 HSV 转换为 RGB
            hsv_to_rgb(hue, saturation, (brightness * 255) / 100, &red, &green, &blue);

            // 设置每个 LED 的颜色
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, red, green, blue));
        }

        // 刷新灯条以应用颜色
        ESP_ERROR_CHECK(led_strip_refresh(led_strip));

        // 更新基础色调，实现颜色渐变
        base_hue = (base_hue + 3) % 360;

        // 更新亮度，实现亮度渐变
        brightness += brightness_step;
        if (brightness >= 100 || brightness <= 0) {
            brightness_step = -brightness_step; // 反转亮度变化方向
        }

        // 延迟 50ms
        // ESP_LOGI(HK_WS2812, "Stack high watermark: %d", uxTaskGetStackHighWaterMark(NULL));

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void ws2812_taskinit(void)
{
    xTaskCreatePinnedToCore(ws2812_handler, "ws2812_handler", 2048, NULL, 3, &ws2812_handle, 1);
}