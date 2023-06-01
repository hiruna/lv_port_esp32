/* LVGL Example project
 *
 * Basic project to test LVGL on ESP32 based projects.
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <esp_log.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "lvgl.h"

#include "lvgl_helpers.h"

#if defined CONFIG_USE_LV_TOUCH_CALIBRATION

#include "lv_tc.h"
#include "lv_tc_screen.h"

#ifndef USE_CUSTOM_LV_TC_COEFFICIENTS

#include "esp_nvs_tc.h"

#endif
#endif

#if defined CONFIG_LV_USE_DEMO_WIDGETS
#include "lv_demos.h"
#elif defined CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER

#include "lvgl/demos/keypad_encoder/lv_demo_keypad_encoder.h"

#elif defined CONFIG_LV_USE_DEMO_BENCHMARK
#include "lvgl/demos/benchmark/lv_demo_benchmark.h"
#elif defined CONFIG_LV_USE_DEMO_STRESS
#include "lvgl/demos/stress/lv_demo_stress.h"
#elif defined CONFIG_LV_USE_DEMO_MUSIC
#include "lvgl/demos/music/lv_demo_music.h"
#endif

// GC9A01 - 1.28" 240x240 ROUND LCD DISPLAY MODULE
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_GC9A01 && LV_BUILD_EXAMPLES
#include "widgets/lv_example_widgets.h"
#endif
/*********************
 *      DEFINES
 *********************/
#define TAG "demo"
#define LV_TICK_PERIOD_MS 1

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);

static void guiTask(void *pvParameter);

static void create_demo_application(void);

/**********************
 *   APPLICATION MAIN
 **********************/
void app_main() {

    /* If you want to use a task to create the graphic, you NEED to create a Pinned task
     * Otherwise there can be problem such as memory corruption and so on.
     * NOTE: When not using Wi-Fi nor Bluetooth you can pin the guiTask to core 0 */
    xTaskCreatePinnedToCore(guiTask, "gui", 4096 * 2, NULL, 0, NULL, 1);
}

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

static void create_demo_application(void) {

    // GC9A01 - 1.28" 240x240 ROUND LCD DISPLAY MODULE
#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_GC9A01 && LV_BUILD_EXAMPLES
    lv_example_meter_3();
#elif defined CONFIG_LV_TFT_DISPLAY_MONOCHROME
    /* When using a monochrome display we only show "Hello World" centered on the
     * screen */

    /* use a pretty small demo for monochrome displays */
    /* Get the current screen  */
    lv_obj_t * scr = lv_disp_get_scr_act(NULL);

    /*Create a Label on the currently active screen*/
    lv_obj_t * label1 =  lv_label_create(scr);

    /*Modify the Label's text*/
    lv_label_set_text(label1, "Hello\nworld");

    /* Align the Label to the center
     * NULL means align on parent (which is the screen now)
     * 0, 0 at the end means an x, y offset after alignment*/
    lv_obj_align(label1, LV_ALIGN_CENTER, 0, 0);
#else
    /* Otherwise we show the selected demo */
#if defined CONFIG_LV_USE_DEMO_WIDGETS
    lv_demo_widgets();
#elif defined CONFIG_LV_USE_DEMO_KEYPAD_AND_ENCODER
    lv_demo_keypad_encoder();
#elif defined CONFIG_LV_USE_DEMO_BENCHMARK
    lv_demo_benchmark();
#elif defined CONFIG_LV_USE_DEMO_STRESS
        lv_demo_stress();
#elif defined CONFIG_LV_USE_DEMO_MUSIC
        lv_demo_music();
#else
#error "No demo application selected."
#endif
#endif
}

static void lv_tick_task(void *arg) {
    (void) arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

#if defined CONFIG_USE_LV_TOUCH_CALIBRATION

// callback function when touch calibration is complete
void lv_tc_finish_cb(lv_event_t *event) {
    lv_obj_t *originalScreen = (lv_obj_t*)(event->user_data);
    lv_obj_t *tCScreen = lv_scr_act();
    if (event->code == LV_EVENT_READY) {
        lv_disp_load_scr(originalScreen);
        create_demo_application();
        lv_obj_del(tCScreen);
    } else {
        ESP_LOGE(TAG, "unexpected lv event code '%d' (expected '%d') after touch calibration", lv_event_get_code(event),
                 LV_EVENT_READY);
    }
}

// function to create the touch calibration screen and begin the calibration
static void start_touch_calibration() {
    lv_obj_t *activeScreen = lv_scr_act();
    lv_obj_t *tCScreen = lv_tc_screen_create();
    lv_obj_add_event_cb(tCScreen, lv_tc_finish_cb, LV_EVENT_READY, activeScreen);
    lv_disp_load_scr(tCScreen);
    lv_tc_screen_start(tCScreen);
}

#endif

static void guiTask(void *pvParameter) {

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();

    /* Initialize SPI or I2C bus used by the drivers */
    lvgl_driver_init();

    lv_color_t *buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);

    /* Use double buffered when not working with monochrome displays */
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    lv_color_t *buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);
#else
    static lv_color_t *buf2 = NULL;
#endif

    static lv_disp_draw_buf_t disp_buf;

    uint32_t size_in_px = DISP_BUF_SIZE;

#if defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_IL3820         \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_JD79653A    \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_UC8151D     \
    || defined CONFIG_LV_TFT_DISPLAY_CONTROLLER_SSD1306

    /* Actual size in pixels, not bytes. */
    size_in_px *= 8;
#endif

    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = disp_driver_flush;

#if defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT || defined CONFIG_DISPLAY_ORIENTATION_PORTRAIT_INVERTED
    disp_drv.rotated = 1;
#endif

    /* When using a monochrome display we need to register the callbacks:
     * - rounder_cb
     * - set_px_cb */
#ifdef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    disp_drv.rounder_cb = disp_driver_rounder;
    disp_drv.set_px_cb = disp_driver_set_px;
    // disable antialiasing when using monochrome display
    disp_drv.antialiasing = 0;
#endif

    // need to set resolution for LVGL 8x
    disp_drv.hor_res = CONFIG_LV_HOR_RES_MAX;
    disp_drv.ver_res = CONFIG_LV_VER_RES_MAX;

    disp_drv.draw_buf = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    /* Register an input device when enabled on the menuconfig */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
    lv_indev_drv_t indev_drv;
#if CONFIG_USE_LV_TOUCH_CALIBRATION // if using LVGL Touch Calibration
    lv_tc_indev_drv_init(&indev_drv, touch_driver_read);
#ifndef CONFIG_USE_CUSTOM_LV_TC_COEFFICIENTS // if NOT using custom calibration coefficients
    lv_tc_register_coeff_save_cb(esp_nvs_tc_coeff_save_cb);
#endif
#else // if NOT using LVGL Touch Calibration
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = touch_driver_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
#endif
    lv_indev_drv_register(&indev_drv);
#endif

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
            .callback = &lv_tick_task,
            .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    /* Create the demo application */
#if CONFIG_LV_TOUCH_CONTROLLER != TOUCH_CONTROLLER_NONE
#if CONFIG_USE_CUSTOM_LV_TC_COEFFICIENTS == 0
    // esp_nvs_tc_coeff_erase(); // this can be used to erase the stored coeff data on nvs
    if (esp_nvs_tc_coeff_init()) {
        create_demo_application();
    } else {
        start_touch_calibration();
    }
#else
    lv_tc_load_coeff_from_config();
    create_demo_application();
#endif
#else
    create_demo_application();
#endif

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));

        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
        }
    }

    /* A task should NEVER return */
    free(buf1);
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}