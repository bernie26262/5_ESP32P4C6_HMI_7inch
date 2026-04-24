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

#include "cJSON.h"

static const char *TAG = "P4_HMI";

#define HMI_UART_NUM        UART_NUM_1
#define HMI_UART_BAUD       230400
#define HMI_UART_TX_GPIO    34
#define HMI_UART_RX_GPIO    36

#define HMI_SYNC_1          0xA5
#define HMI_SYNC_2          0x5A

#define UART_FRAME_BUF_SIZE 8192
#define JSON_VIEW_SIZE      2048

typedef enum {
    RX_WAIT_SYNC1 = 0,
    RX_WAIT_SYNC2,
    RX_WAIT_LEN1,
    RX_WAIT_LEN2,
    RX_READ_PAYLOAD
} hmi_rx_state_t;

typedef struct {
    uint32_t rx_frames;
    uint32_t rx_bytes;
    uint32_t rx_len_err;
    uint32_t rx_bad_json;
    uint32_t ack_sent;
    uint32_t last_seq;
    uint32_t last_len;

    uint32_t state_frames;
    uint32_t analog_frames;
    uint32_t other_frames;

    char last_type[32];
    char eth_ip[32];

    bool power_on;
    bool auto_mode;
    bool notaus_active;
    bool ack_required;
    bool mega1_online;
    bool mega2_online;

    char last_state_json[JSON_VIEW_SIZE];
    char last_analog_json[JSON_VIEW_SIZE];
} hmi_state_t;

static SemaphoreHandle_t g_state_mutex;

static hmi_state_t g_state;

static char g_frame_buf[UART_FRAME_BUF_SIZE];

static hmi_rx_state_t g_rx_state = RX_WAIT_SYNC1;
static uint16_t g_expected_len = 0;
static uint16_t g_pos = 0;

static lv_obj_t *touch_label = NULL;
static lv_obj_t *uart_status_label = NULL;
static lv_obj_t *system_status_label = NULL;
static lv_obj_t *state_json_label = NULL;
static lv_obj_t *analog_json_label = NULL;

static volatile bool g_ui_dirty = false;

static const cJSON *json_get_path(const cJSON *root, const char *a, const char *b)
{
    const cJSON *obj = cJSON_GetObjectItemCaseSensitive(root, a);
    if (!obj || !cJSON_IsObject(obj)) {
        return NULL;
    }
    return cJSON_GetObjectItemCaseSensitive(obj, b);
}

static bool json_get_bool_path(const cJSON *root, const char *a, const char *b, bool fallback)
{
    const cJSON *v = json_get_path(root, a, b);
    if (cJSON_IsBool(v)) {
        return cJSON_IsTrue(v);
    }
    return fallback;
}

static void json_get_string_path(const cJSON *root, const char *a, const char *b, char *out, size_t out_len)
{
    const cJSON *v = json_get_path(root, a, b);
    if (cJSON_IsString(v) && v->valuestring) {
        snprintf(out, out_len, "%s", v->valuestring);
    }
}

static bool json_find_u32(const char *json, const char *key, uint32_t *out)
{
    if (!json || !key || !out) return false;

    const char *p = strstr(json, key);
    if (!p) return false;

    p += strlen(key);

    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;
    if (*p != ':') return false;
    ++p;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') ++p;

    uint32_t value = 0;
    bool any = false;

    while (*p >= '0' && *p <= '9') {
        any = true;
        value = value * 10u + (uint32_t)(*p - '0');
        ++p;
    }

    if (!any) return false;
    *out = value;
    return true;
}

static void hmi_uart_send_ack(uint32_t seq)
{
    if (seq == 0) return;

    char line[64];
    int len = snprintf(line, sizeof(line), "{\"type\":\"ack\",\"seq\":%lu}\n", (unsigned long)seq);

    if (len <= 0 || len >= (int)sizeof(line)) return;

    uart_write_bytes(HMI_UART_NUM, line, len);

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        g_state.ack_sent++;
        xSemaphoreGive(g_state_mutex);
    }
}

static void frame_parser_reset(void)
{
    g_rx_state = RX_WAIT_SYNC1;
    g_expected_len = 0;
    g_pos = 0;
}

static void hmi_state_update_from_json(const char *payload, uint32_t len)
{
    cJSON *root = cJSON_Parse(payload);

    if (!root || !cJSON_IsObject(root)) {
        if (root) cJSON_Delete(root);

        if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_state.rx_bad_json++;
            xSemaphoreGive(g_state_mutex);
        }
        return;
    }

    const cJSON *type_item = cJSON_GetObjectItemCaseSensitive(root, "type");
    const char *type = (cJSON_IsString(type_item) && type_item->valuestring) ? type_item->valuestring : "unknown";

    uint32_t seq = 0;
    if (json_find_u32(payload, "\"seq\"", &seq)) {
        hmi_uart_send_ack(seq);
    }

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        g_state.rx_frames++;
        g_state.last_len = len;

        if (seq != 0) {
            g_state.last_seq = seq;
        }

        snprintf(g_state.last_type, sizeof(g_state.last_type), "%s", type);

        if (strcmp(type, "state-lite") == 0 || strcmp(type, "state") == 0) {
            g_state.state_frames++;

            g_state.power_on = json_get_bool_path(root, "safety", "powerOn", g_state.power_on);
            g_state.notaus_active = json_get_bool_path(root, "safety", "notausActive", g_state.notaus_active);
            g_state.ack_required = json_get_bool_path(root, "safety", "ackRequired", g_state.ack_required);

            g_state.auto_mode = json_get_bool_path(root, "mega1", "modeAuto", g_state.auto_mode);
            g_state.mega1_online = json_get_bool_path(root, "mega1", "online", g_state.mega1_online);
            g_state.mega2_online = json_get_bool_path(root, "mega2", "online", g_state.mega2_online);

            json_get_string_path(root, "eth", "ip", g_state.eth_ip, sizeof(g_state.eth_ip));

            snprintf(g_state.last_state_json, sizeof(g_state.last_state_json), "%s", payload);
        } else if (strcmp(type, "analog") == 0) {
            g_state.analog_frames++;
            snprintf(g_state.last_analog_json, sizeof(g_state.last_analog_json), "%s", payload);
        } else {
            g_state.other_frames++;
        }

        g_ui_dirty = true;
        xSemaphoreGive(g_state_mutex);
    }

    cJSON_Delete(root);
}

static void frame_commit_payload(void)
{
    g_frame_buf[g_pos] = '\0';
    hmi_state_update_from_json(g_frame_buf, g_pos);
    frame_parser_reset();
}

static void frame_process_byte(uint8_t b)
{
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, 0) == pdTRUE) {
        g_state.rx_bytes++;
        xSemaphoreGive(g_state_mutex);
    }

    switch (g_rx_state) {
        case RX_WAIT_SYNC1:
            if (b == HMI_SYNC_1) g_rx_state = RX_WAIT_SYNC2;
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
                if (g_state_mutex && xSemaphoreTake(g_state_mutex, 0) == pdTRUE) {
                    g_state.rx_len_err++;
                    xSemaphoreGive(g_state_mutex);
                }
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
                if (g_state_mutex && xSemaphoreTake(g_state_mutex, 0) == pdTRUE) {
                    g_state.rx_len_err++;
                    xSemaphoreGive(g_state_mutex);
                }
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
    if (!indev || !touch_label) return;

    lv_point_t p;
    lv_indev_get_point(indev, &p);

    char buf[64];
    snprintf(buf, sizeof(buf), "Touch: x=%ld y=%ld", (long)p.x, (long)p.y);
    lv_label_set_text(touch_label, buf);
}

static const char *onoff(bool v)
{
    return v ? "EIN" : "AUS";
}

static const char *yesno(bool v)
{
    return v ? "JA" : "NEIN";
}

static void uart_ui_timer_cb(lv_timer_t *timer)
{
    hmi_state_t s;
    bool dirty = false;

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, 0) == pdTRUE) {
        s = g_state;
        dirty = g_ui_dirty;
        g_ui_dirty = false;
        xSemaphoreGive(g_state_mutex);
    } else {
        return;
    }

    char status[192];
    snprintf(
        status,
        sizeof(status),
        "RX frames=%lu bytes=%lu seq=%lu ack=%lu | state=%lu analog=%lu other=%lu | lenErr=%lu badJson=%lu | type=%s",
        (unsigned long)s.rx_frames,
        (unsigned long)s.rx_bytes,
        (unsigned long)s.last_seq,
        (unsigned long)s.ack_sent,
        (unsigned long)s.state_frames,
        (unsigned long)s.analog_frames,
        (unsigned long)s.other_frames,
        (unsigned long)s.rx_len_err,
        (unsigned long)s.rx_bad_json,
        s.last_type
    );

    lv_label_set_text(uart_status_label, status);

    char sys[256];
    snprintf(
        sys,
        sizeof(sys),
        "Power: %s | Auto: %s | Notaus: %s | ACK erforderlich: %s\nMega1: %s | Mega2: %s | ETH IP: %s",
        onoff(s.power_on),
        onoff(s.auto_mode),
        yesno(s.notaus_active),
        yesno(s.ack_required),
        s.mega1_online ? "online" : "offline",
        s.mega2_online ? "online" : "offline",
        s.eth_ip[0] ? s.eth_ip : "-"
    );

    lv_label_set_text(system_status_label, sys);

    if (dirty) {
        lv_label_set_text(state_json_label, s.last_state_json[0] ? s.last_state_json : "Noch kein state-lite empfangen.");
        lv_label_set_text(analog_json_label, s.last_analog_json[0] ? s.last_analog_json : "Noch kein analog empfangen.");
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
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *subtitle = lv_label_create(screen);
    lv_label_set_text(subtitle, "LVGL v8 | UART State Cache");
    lv_obj_set_style_text_color(subtitle, lv_color_hex(0xA8DADC), 0);
    lv_obj_set_style_text_font(subtitle, &lv_font_montserrat_20, 0);
    lv_obj_align(subtitle, LV_ALIGN_TOP_MID, 0, 65);

    lv_obj_t *touch_panel = lv_obj_create(screen);
    lv_obj_set_size(touch_panel, 300, 105);
    lv_obj_align(touch_panel, LV_ALIGN_TOP_LEFT, 25, 105);
    lv_obj_set_style_radius(touch_panel, 14, 0);
    lv_obj_set_style_bg_color(touch_panel, lv_color_hex(0x1F2A36), 0);
    lv_obj_set_style_border_color(touch_panel, lv_color_hex(0x4A6073), 0);
    lv_obj_set_style_border_width(touch_panel, 2, 0);
    lv_obj_add_event_cb(touch_panel, touch_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(touch_panel, touch_event_cb, LV_EVENT_PRESSING, NULL);

    touch_label = lv_label_create(touch_panel);
    lv_label_set_text(touch_label, "Touch: -");
    lv_obj_set_style_text_color(touch_label, lv_color_hex(0xF1FAEE), 0);
    lv_obj_set_style_text_font(touch_label, &lv_font_montserrat_18, 0);
    lv_obj_center(touch_label);

    lv_obj_t *system_panel = lv_obj_create(screen);
    lv_obj_set_size(system_panel, 650, 105);
    lv_obj_align(system_panel, LV_ALIGN_TOP_RIGHT, -25, 105);
    lv_obj_set_style_radius(system_panel, 14, 0);
    lv_obj_set_style_bg_color(system_panel, lv_color_hex(0x17212B), 0);
    lv_obj_set_style_border_color(system_panel, lv_color_hex(0x4A6073), 0);
    lv_obj_set_style_border_width(system_panel, 2, 0);

    system_status_label = lv_label_create(system_panel);
    lv_label_set_text(system_status_label, "Warte auf state-lite...");
    lv_label_set_long_mode(system_status_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(system_status_label, 610);
    lv_obj_set_style_text_color(system_status_label, lv_color_hex(0xF1FAEE), 0);
    lv_obj_set_style_text_font(system_status_label, &lv_font_montserrat_18, 0);
    lv_obj_align(system_status_label, LV_ALIGN_TOP_LEFT, 15, 18);

    lv_obj_t *uart_panel = lv_obj_create(screen);
    lv_obj_set_size(uart_panel, 970, 345);
    lv_obj_align(uart_panel, LV_ALIGN_BOTTOM_MID, 0, -38);
    lv_obj_set_style_radius(uart_panel, 16, 0);
    lv_obj_set_style_bg_color(uart_panel, lv_color_hex(0x17212B), 0);
    lv_obj_set_style_border_color(uart_panel, lv_color_hex(0x4A6073), 0);
    lv_obj_set_style_border_width(uart_panel, 2, 0);

    uart_status_label = lv_label_create(uart_panel);
    lv_label_set_text(uart_status_label, "UART RX: waiting...");
    lv_label_set_long_mode(uart_status_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(uart_status_label, 930);
    lv_obj_set_style_text_color(uart_status_label, lv_color_hex(0xA8DADC), 0);
    lv_obj_set_style_text_font(uart_status_label, &lv_font_montserrat_16, 0);
    lv_obj_align(uart_status_label, LV_ALIGN_TOP_LEFT, 15, 12);

    lv_obj_t *state_title = lv_label_create(uart_panel);
    lv_label_set_text(state_title, "Letzter state-lite:");
    lv_obj_set_style_text_color(state_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(state_title, &lv_font_montserrat_16, 0);
    lv_obj_align(state_title, LV_ALIGN_TOP_LEFT, 15, 58);

    state_json_label = lv_label_create(uart_panel);
    lv_label_set_text(state_json_label, "Noch kein state-lite empfangen.");
    lv_label_set_long_mode(state_json_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(state_json_label, 930);
    lv_obj_set_style_text_color(state_json_label, lv_color_hex(0xF1FAEE), 0);
    lv_obj_set_style_text_font(state_json_label, &lv_font_montserrat_14, 0);
    lv_obj_align(state_json_label, LV_ALIGN_TOP_LEFT, 15, 82);

    lv_obj_t *analog_title = lv_label_create(uart_panel);
    lv_label_set_text(analog_title, "Letzter analog:");
    lv_obj_set_style_text_color(analog_title, lv_color_white(), 0);
    lv_obj_set_style_text_font(analog_title, &lv_font_montserrat_16, 0);
    lv_obj_align(analog_title, LV_ALIGN_TOP_LEFT, 15, 205);

    analog_json_label = lv_label_create(uart_panel);
    lv_label_set_text(analog_json_label, "Noch kein analog empfangen.");
    lv_label_set_long_mode(analog_json_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(analog_json_label, 930);
    lv_obj_set_style_text_color(analog_json_label, lv_color_hex(0xF1FAEE), 0);
    lv_obj_set_style_text_font(analog_json_label, &lv_font_montserrat_14, 0);
    lv_obj_align(analog_json_label, LV_ALIGN_TOP_LEFT, 15, 229);

    lv_obj_t *footer = lv_label_create(screen);
    lv_label_set_text(footer, "ESP32-P4-WIFI6-Touch-LCD-7B | UART1 TX=GPIO34 RX=GPIO36");
    lv_obj_set_style_text_color(footer, lv_color_hex(0xB0B8C0), 0);
    lv_obj_set_style_text_font(footer, &lv_font_montserrat_16, 0);
    lv_obj_align(footer, LV_ALIGN_BOTTOM_MID, 0, -10);

    lv_timer_create(uart_ui_timer_cb, 250, NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Railway HMI P4 state-cache bring-up");

    memset(&g_state, 0, sizeof(g_state));
    g_state_mutex = xSemaphoreCreateMutex();

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

    ESP_LOGI(TAG, "Minimal HMI screen created, UART parser and state cache active");
}