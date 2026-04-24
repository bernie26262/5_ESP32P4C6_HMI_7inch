#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "lvgl.h"

static const char *TAG = "P4_HMI";

static lv_obj_t *touch_label = NULL;

static void touch_event_cb(lv_event_t *e)
{
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev || !touch_label) {
        return;
    }

    lv_point_t p;
    lv_indev_get_point(indev, &p);

    char buf[64];
    snprintf(buf, sizeof(buf), "Touch: x=%ld y=%ld", (long)p.x, (long)p.y);
    lv_label_set_text(touch_label, buf);
}

static void create_hmi_screen(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "Railway HMI P4");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t *status = lv_label_create(screen);
    lv_label_set_text(status, "LVGL v8 ready");
    lv_obj_set_style_text_color(status, lv_color_hex(0xA8DADC), 0);
    lv_obj_set_style_text_font(status, &lv_font_montserrat_20, 0);
    lv_obj_align(status, LV_ALIGN_TOP_MID, 0, 95);

    lv_obj_t *panel = lv_obj_create(screen);
    lv_obj_set_size(panel, 520, 220);
    lv_obj_align(panel, LV_ALIGN_CENTER, 0, 25);
    lv_obj_set_style_radius(panel, 18, 0);
    lv_obj_set_style_bg_color(panel, lv_color_hex(0x1F2A36), 0);
    lv_obj_set_style_border_color(panel, lv_color_hex(0x4A6073), 0);
    lv_obj_set_style_border_width(panel, 2, 0);
    lv_obj_add_event_cb(panel, touch_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(panel, touch_event_cb, LV_EVENT_PRESSING, NULL);

    lv_obj_t *panel_title = lv_label_create(panel);
    lv_label_set_text(panel_title, "Touch-Testfeld");
    lv_obj_set_style_text_color(panel_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(panel_title, &lv_font_montserrat_24, 0);
    lv_obj_align(panel_title, LV_ALIGN_TOP_MID, 0, 25);

    touch_label = lv_label_create(panel);
    lv_label_set_text(touch_label, "Touch: noch keine Eingabe");
    lv_obj_set_style_text_color(touch_label, lv_color_hex(0xF1FAEE), 0);
    lv_obj_set_style_text_font(touch_label, &lv_font_montserrat_20, 0);
    lv_obj_align(touch_label, LV_ALIGN_CENTER, 0, 20);

    lv_obj_t *footer = lv_label_create(screen);
    lv_label_set_text(footer, "ESP32-P4-WIFI6-Touch-LCD-7B | Minimal-HMI Bring-up");
    lv_obj_set_style_text_color(footer, lv_color_hex(0xB0B8C0), 0);
    lv_obj_set_style_text_font(footer, &lv_font_montserrat_16, 0);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -25);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Railway HMI P4 minimal bring-up");

    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
            .sw_rotate = true,
        }
    };

    lv_display_t *disp = bsp_display_start_with_config(&cfg);
    bsp_display_backlight_on();

    if (disp != NULL) {
        bsp_display_rotate(disp, LV_DISPLAY_ROTATION_180);
    }

    bsp_display_lock(0);
    create_hmi_screen();
    bsp_display_unlock();

    ESP_LOGI(TAG, "Minimal HMI screen created");
}