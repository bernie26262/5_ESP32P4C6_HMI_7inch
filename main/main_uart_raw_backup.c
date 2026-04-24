#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/uart.h"

#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "lvgl.h"

static const char *TAG = "P4_HMI";

/*
 * TODO: echte UART-Pins fuer ETH <-> P4 festlegen.
 * ETH bisher:
 *   ETH TX -> HMI RX
 *   ETH RX <- HMI TX
 *
 * Diese Werte sind Platzhalter fuer den ersten Build-/Bring-up-Test.
 */
#define HMI_UART_NUM       UART_NUM_1
#define HMI_UART_BAUD      230400
#define HMI_UART_RX_GPIO   36
#define HMI_UART_TX_GPIO   34

#define HMI_SYNC_1         0xA5
#define HMI_SYNC_2         0x5A
#define UART_FRAME_BUF_SIZE 8192

typedef enum {
    RX_WAIT_SYNC1 = 0,
    RX_WAIT_SYNC2,
    RX_WAIT_LEN1,
    RX_WAIT_LEN2,
    RX_READ_PAYLOAD
} hmi_rx_state_t;

static lv_obj_t *touch_label = NULL;
static lv_obj_t *uart_status_label = NULL;
static lv_obj_t *uart_rx_label = NULL;

static SemaphoreHandle_t g_rx_mutex;

static char g_frame_buf[UART_FRAME_BUF_SIZE];
static char g_last_json[2048];

static volatile bool g_rx_dirty = false;
static uint32_t g_rx_frames = 0;
static uint32_t g_rx_bytes = 0;
static uint32_t g_rx_len_err = 0;
static uint32_t g_rx_bad_json = 0;
static uint32_t g_ack_sent = 0;
static uint32_t g_last_seq = 0;
static uint32_t g_last_len = 0;
static int64_t g_last_rx_us = 0;

static hmi_rx_state_t g_rx_state = RX_WAIT_SYNC1;
static uint16_t g_expected_len = 0;
static uint16_t g_pos = 0;

static bool json_find_u32(const char *json, const char *key, uint32_t *out)
{
    if (!json || !key || !out) {
        return false;
    }

    const char *p = strstr(json, key);
    if (!p) {
        return false;
    }

    p += strlen(key);

    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        ++p;
    }

    if (*p != ':') {
        return false;
    }

    ++p;

    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        ++p;
    }

    uint32_t value = 0;
    bool any = false;

    while (*p >= '0' && *p <= '9') {
        any = true;
        value = value * 10u + (uint32_t)(*p - '0');
        ++p;
    }

    if (!any) {
        return false;
    }

    *out = value;
    return true;
}

static void hmi_uart_send_ack(uint32_t seq)
{
    if (seq == 0) {
        return;
    }

    char line[64];
    int len = snprintf(line, sizeof(line), "{\"type\":\"ack\",\"seq\":%lu}\n", (unsigned long)seq);

    if (len <= 0 || len >= (int)sizeof(line)) {
        return;
    }

    uart_write_bytes(HMI_UART_NUM, line, len);
    g_ack_sent++;
}

static void frame_parser_reset(void)
{
    g_rx_state = RX_WAIT_SYNC1;
    g_expected_len = 0;
    g_pos = 0;
}

static void frame_commit_payload(void)
{
    g_frame_buf[g_pos] = '\0';
    g_rx_frames++;
    g_last_len = g_pos;
    g_last_rx_us = esp_timer_get_time();

    const char *p = g_frame_buf;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') {
        ++p;
    }

    if (*p != '{') {
        g_rx_bad_json++;
        frame_parser_reset();
        return;
    }

    uint32_t seq = 0;
    if (json_find_u32(g_frame_buf, "\"seq\"", &seq)) {
        g_last_seq = seq;

        /*
         * Wichtig:
         * ACK sofort nach komplettem Frame + minimalem JSON-Sanity-Check.
         * Nicht erst nach UI-Update.
         */
        hmi_uart_send_ack(seq);
    }

    if (g_rx_mutex && xSemaphoreTake(g_rx_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        snprintf(g_last_json, sizeof(g_last_json), "%s", g_frame_buf);
        g_rx_dirty = true;
        xSemaphoreGive(g_rx_mutex);
    }

    frame_parser_reset();
}

static void frame_process_byte(uint8_t b)
{
    g_rx_bytes++;

    switch (g_rx_state) {
        case RX_WAIT_SYNC1:
            if (b == HMI_SYNC_1) {
                g_rx_state = RX_WAIT_SYNC2;
            }
            break;

        case RX_WAIT_SYNC2:
            if (b == HMI_SYNC_2) {
                g_rx_state = RX_WAIT_LEN1;
            } else if (b == HMI_SYNC_1) {
                g_rx_state = RX_WAIT_SYNC2;
            } else {
                g_rx_state = RX_WAIT_SYNC1;
            }
            break;

        case RX_WAIT_LEN1:
            g_expected_len = (uint16_t)b;
            g_rx_state = RX_WAIT_LEN2;
            break;

        case RX_WAIT_LEN2:
            g_expected_len |= ((uint16_t)b << 8);

            if (g_expected_len == 0 || g_expected_len >= UART_FRAME_BUF_SIZE) {
                g_rx_len_err++;
                frame_parser_reset();
                break;
            }

            g_pos = 0;
            g_rx_state = RX_READ_PAYLOAD;
            break;

        case RX_READ_PAYLOAD:
            if (g_pos < g_expected_len) {
                g_frame_buf[g_pos++] = (char)b;
            } else {
                g_rx_len_err++;
                frame_parser_reset();
                break;
            }

            if (g_pos == g_expected_len) {
                frame_commit_payload();
            }
            break;
    }
}

static void hmi_uart_rx_task(void *arg)
{
    uint8_t buf[256];

    ESP_LOGI(TAG, "UART RX task started: uart=%d baud=%d rx=%d tx=%d",
             HMI_UART_NUM, HMI_UART_BAUD, HMI_UART_RX_GPIO, HMI_UART_TX_GPIO);

    while (true) {
        int n = uart_read_bytes(HMI_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));

        if (n > 0) {
            for (int i = 0; i < n; ++i) {
                frame_process_byte(buf[i]);
            }
        }
    }
}

static void hmi_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate = HMI_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(HMI_UART_NUM, 8192, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(HMI_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        HMI_UART_NUM,
        HMI_UART_TX_GPIO,
        HMI_UART_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    xTaskCreate(hmi_uart_rx_task, "hmi_uart_rx", 8192, NULL, 10, NULL);
}

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

static void uart_ui_timer_cb(lv_timer_t *timer)
{
    bool dirty = false;
    char json_copy[2048];

    json_copy[0] = '\0';

    if (g_rx_mutex && xSemaphoreTake(g_rx_mutex, 0) == pdTRUE) {
        dirty = g_rx_dirty;
        if (dirty) {
            snprintf(json_copy, sizeof(json_copy), "%s", g_last_json);
            g_rx_dirty = false;
        }
        xSemaphoreGive(g_rx_mutex);
    }

    char status[160];
    snprintf(
        status,
        sizeof(status),
        "UART RX: frames=%lu bytes=%lu seq=%lu len=%lu ack=%lu lenErr=%lu badJson=%lu",
        (unsigned long)g_rx_frames,
        (unsigned long)g_rx_bytes,
        (unsigned long)g_last_seq,
        (unsigned long)g_last_len,
        (unsigned long)g_ack_sent,
        (unsigned long)g_rx_len_err,
        (unsigned long)g_rx_bad_json
    );

    if (uart_status_label) {
        lv_label_set_text(uart_status_label, status);
    }

    if (dirty && uart_rx_label) {
        lv_label_set_text(uart_rx_label, json_copy);
    }
}

static void create_hmi_screen(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x101820), 0);

    lv_obj_t *title = lv_label_create(screen);
    lv_label_set_text(title, "Railway HMI P4");
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_32, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 25);

    lv_obj_t *status = lv_label_create(screen);
    lv_label_set_text(status, "LVGL v8 ready | UART frame parser active");
    lv_obj_set_style_text_color(status, lv_color_hex(0xA8DADC), 0);
    lv_obj_set_style_text_font(status, &lv_font_montserrat_20, 0);
    lv_obj_align(status, LV_ALIGN_TOP_MID, 0, 78);

    lv_obj_t *touch_panel = lv_obj_create(screen);
    lv_obj_set_size(touch_panel, 420, 130);
    lv_obj_align(touch_panel, LV_ALIGN_TOP_LEFT, 35, 130);
    lv_obj_set_style_radius(touch_panel, 16, 0);
    lv_obj_set_style_bg_color(touch_panel, lv_color_hex(0x1F2A36), 0);
    lv_obj_set_style_border_color(touch_panel, lv_color_hex(0x4A6073), 0);
    lv_obj_set_style_border_width(touch_panel, 2, 0);
    lv_obj_add_event_cb(touch_panel, touch_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(touch_panel, touch_event_cb, LV_EVENT_PRESSING, NULL);

    lv_obj_t *touch_title = lv_label_create(touch_panel);
    lv_label_set_text(touch_title, "Touch-Test");
    lv_obj_set_style_text_color(touch_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(touch_title, &lv_font_montserrat_20, 0);
    lv_obj_align(touch_title, LV_ALIGN_TOP_MID, 0, 10);

    touch_label = lv_label_create(touch_panel);
    lv_label_set_text(touch_label, "Touch: noch keine Eingabe");
    lv_obj_set_style_text_color(touch_label, lv_color_hex(0xF1FAEE), 0);
    lv_obj_set_style_text_font(touch_label, &lv_font_montserrat_18, 0);
    lv_obj_align(touch_label, LV_ALIGN_CENTER, 0, 20);

    lv_obj_t *uart_panel = lv_obj_create(screen);
    lv_obj_set_size(uart_panel, 940, 265);
    lv_obj_align(uart_panel, LV_ALIGN_BOTTOM_MID, 0, -45);
    lv_obj_set_style_radius(uart_panel, 16, 0);
    lv_obj_set_style_bg_color(uart_panel, lv_color_hex(0x17212B), 0);
    lv_obj_set_style_border_color(uart_panel, lv_color_hex(0x4A6073), 0);
    lv_obj_set_style_border_width(uart_panel, 2, 0);

    lv_obj_t *uart_title = lv_label_create(uart_panel);
    lv_label_set_text(uart_title, "ETH -> HMI UART Test");
    lv_obj_set_style_text_color(uart_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(uart_title, &lv_font_montserrat_20, 0);
    lv_obj_align(uart_title, LV_ALIGN_TOP_LEFT, 15, 10);

    uart_status_label = lv_label_create(uart_panel);
    lv_label_set_text(uart_status_label, "UART RX: waiting for framed JSON...");
    lv_obj_set_style_text_color(uart_status_label, lv_color_hex(0xA8DADC), 0);
    lv_obj_set_style_text_font(uart_status_label, &lv_font_montserrat_16, 0);
    lv_obj_align(uart_status_label, LV_ALIGN_TOP_LEFT, 15, 45);

    uart_rx_label = lv_label_create(uart_panel);
    lv_label_set_text(uart_rx_label, "Last JSON: -");
    lv_label_set_long_mode(uart_rx_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(uart_rx_label, 900);
    lv_obj_set_style_text_color(uart_rx_label, lv_color_hex(0xF1FAEE), 0);
    lv_obj_set_style_text_font(uart_rx_label, &lv_font_montserrat_16, 0);
    lv_obj_align(uart_rx_label, LV_ALIGN_TOP_LEFT, 15, 80);

    lv_obj_t *footer = lv_label_create(screen);
    lv_label_set_text(footer, "ESP32-P4-WIFI6-Touch-LCD-7B | Minimal-HMI + UART bring-up");
    lv_obj_set_style_text_color(footer, lv_color_hex(0xB0B8C0), 0);
    lv_obj_set_style_text_font(footer, &lv_font_montserrat_16, 0);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -15);

    lv_timer_create(uart_ui_timer_cb, 250, NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Railway HMI P4 minimal bring-up");

    g_rx_mutex = xSemaphoreCreateMutex();

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

    hmi_uart_init();

    ESP_LOGI(TAG, "Minimal HMI screen created, UART parser active");
}