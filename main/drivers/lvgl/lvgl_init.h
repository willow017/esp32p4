#ifndef _LVGL_INIT_H_
#define _LVGL_INIT_H_

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"

#include "esp_dma_utils.h"


#include "esp_lcd_ek79007.h"


#include "lvgl.h"
#include "esp_lvgl_port.h"

#include "esp_lcd_touch_gt911.h"
#include "bsp_err_check.h"

#include "display.h"
#include "touch.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "esp_ldo_regulator.h"
#include "lv_demos.h"

#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_ldo_regulator.h"
#include "esp_dma_utils.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_ek79007.h"

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY        1
#define BSP_CAPS_TOUCH          1
#define BSP_CAPS_BUTTONS        0
#define BSP_CAPS_AUDIO          1
#define BSP_CAPS_AUDIO_SPEAKER  1
#define BSP_CAPS_AUDIO_MIC      1
#define BSP_CAPS_SDCARD         1
#define BSP_CAPS_IMU            0

/**************************************************************************************************
 *  ESP-BOX pinout
 **************************************************************************************************/
/* I2C */
#define BSP_I2C_SCL           (GPIO_NUM_8)
#define BSP_I2C_SDA           (GPIO_NUM_7)

/* Audio */
#define BSP_I2S_SCLK          (GPIO_NUM_12)
#define BSP_I2S_MCLK          (GPIO_NUM_13)
#define BSP_I2S_LCLK          (GPIO_NUM_10)
#define BSP_I2S_DOUT          (GPIO_NUM_9)
#define BSP_I2S_DSIN          (GPIO_NUM_11)
#define BSP_POWER_AMP_IO      (GPIO_NUM_53)

/* Display */
#define BSP_LCD_BACKLIGHT     (GPIO_NUM_26)
#define BSP_LCD_RST           (GPIO_NUM_27)
#define BSP_LCD_TOUCH_RST     (GPIO_NUM_NC)
#define BSP_LCD_TOUCH_INT     (GPIO_NUM_NC)

/* uSD card */
#define BSP_SD_D0             (GPIO_NUM_39)
#define BSP_SD_D1             (GPIO_NUM_40)
#define BSP_SD_D2             (GPIO_NUM_41)
#define BSP_SD_D3             (GPIO_NUM_42)
#define BSP_SD_CMD            (GPIO_NUM_44)
#define BSP_SD_CLK            (GPIO_NUM_43)

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< LVGL port configuration */
    uint32_t        buffer_size;    /*!< Size of the buffer for the screen in pixels */
    bool            double_buffer;  /*!< True, if should be allocated two buffers */
    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
        unsigned int sw_rotate: 1;   /*!< Use software rotation (slower), The feature is unavailable under avoid-tear mode */
    } flags;
} bsp_display_cfg_t;

void display_init(void);
esp_err_t bsp_i2c_init(void);
i2c_master_bus_handle_t bsp_i2c_get_handle(void);

#endif

