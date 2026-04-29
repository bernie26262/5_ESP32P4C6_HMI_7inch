#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"

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
    bool safety_lock;
    bool startup_ready;
    bool startup_checklist_active;
    bool startup_m1_needs;
    bool startup_m2_needs;
    bool startup_m1_running;
    bool startup_m2_running;
    bool startup_m1_done;
    bool startup_m2_done;
    bool action_can_ack;
    bool action_can_start_m1_selftest;
    bool action_can_start_m2_selftest;
    bool action_can_startup_confirm;
    bool mega1_selftest_retry_available;
    bool mega2_selftest_retry_available;
    bool ui_startup_overlay_active;
    bool ui_m1_retry_overlay_active;
    bool ui_m2_retry_overlay_active;
    char ui_overlay_mode[16];
    char ui_retry_scope[16];
    bool mega1_online;
    bool mega2_online;
    bool warning_present;
    uint32_t mega1_warning_mask;
    bool can_write;
    bool diag_active;
    uint32_t ws_base_clients;
    uint32_t ws_diag_clients;
    uint32_t sbhf_allowed_mask;
    uint8_t sbhf_occupied_mask;
    bool sbhf_restricted;
    bool sbhf_start_pending;
    char sbhf_allowed_text[32];

    uint16_t mega1_weiche_ist_bits;
    uint16_t mega1_weiche_soll_bits;
    uint16_t mega2_turnout_ist_mask;
    uint16_t mega2_turnout_soll_mask;

    uint8_t mega1_bahnhof_mask;
    bool mega1_bahnhof_valid;
    uint16_t mega2_block_occ_mask;
    bool mega2_block_occ_valid;
    uint16_t mega2_signal_grant_mask;
    bool mega2_signal_grant_valid;
    uint16_t mega2_entry_allowed[9];
    bool mega2_entry_allowed_valid;
    uint8_t mega2_sbhf_state;
    uint8_t mega2_sbhf_current_gleis;
    bool mega2_block5_to_sbhf_active;
    bool mega2_block5_to_sbhf_valid;

    int32_t trafo_oben_v10;
    int32_t trafo_unten_v10;

    char mega1_defects[160];
    char mega2_defects[160];

    char last_state_json[JSON_VIEW_SIZE];
    char last_analog_json[JSON_VIEW_SIZE];
} hmi_state_t;

static SemaphoreHandle_t g_state_mutex;
static SemaphoreHandle_t g_uart_tx_mutex;

static hmi_state_t g_state;

static char g_frame_buf[UART_FRAME_BUF_SIZE];

static hmi_rx_state_t g_rx_state = RX_WAIT_SYNC1;
static uint16_t g_expected_len = 0;
static uint16_t g_pos = 0;

static lv_obj_t *uart_status_label = NULL;
static lv_obj_t *comm_status_label = NULL;

static lv_obj_t *settings_brightness_label = NULL;
static lv_obj_t *settings_brightness_slider = NULL;
static lv_obj_t *settings_timeout_dropdown = NULL;
static lv_obj_t *screen_off_wake_overlay = NULL;
static lv_indev_t *g_touch_indev = NULL;
static lv_timer_t *display_power_timer = NULL;
static int g_display_brightness_percent = 70;
static uint32_t g_screen_off_timeout_sec = 120;
static uint32_t g_last_user_activity_ms = 0;
static bool g_screen_off = false;

// Right panel elements (new)
static lv_obj_t *power_led = NULL;
static lv_obj_t *auto_led = NULL;
static lv_obj_t *power_on_btn = NULL;
static lv_obj_t *power_off_btn = NULL;
static lv_obj_t *auto_btn = NULL;

static lv_obj_t *trafo_label = NULL;
static lv_obj_t *write_label = NULL;
static lv_obj_t *system_label = NULL;
static lv_obj_t *fault_card_obj = NULL;
static lv_obj_t *messages_card_obj = NULL;
static lv_obj_t *fault_m1_label = NULL;
static lv_obj_t *fault_m2_label = NULL;
static lv_obj_t *m1_retry_btn = NULL;
static lv_obj_t *m2_retry_btn = NULL;
static lv_obj_t *messages_ack_label = NULL;
static lv_obj_t *messages_m1_label = NULL;
static lv_obj_t *messages_sbhf_label = NULL;
static lv_obj_t *messages_mode_label = NULL;
static lv_obj_t *messages_allowed_label = NULL;

static lv_obj_t *eth_value_label = NULL;
static lv_obj_t *mega1_value_label = NULL;
static lv_obj_t *mega2_value_label = NULL;
static lv_obj_t *safety_value_label = NULL;
static lv_obj_t *warning_value_label = NULL;
static lv_obj_t *power_value_label = NULL;
static lv_obj_t *mode_value_label = NULL;
static lv_obj_t *ws_diag_value_label = NULL;
static lv_obj_t *left_tabview = NULL;
static lv_obj_t *left_weichen_cmd_status_label = NULL;
static lv_obj_t *left_station_cmd_status_label = NULL;
static lv_obj_t *left_debug_summary_label = NULL;
static lv_obj_t *left_m1_turnout_btns[12];
static lv_obj_t *left_m1_turnout_labels[12];
static lv_obj_t *left_sbhf_turnout_cards[4];
static lv_obj_t *left_sbhf_turnout_labels[4];
static lv_obj_t *left_station_cards[4];
static lv_obj_t *left_station_labels[4];
static lv_obj_t *left_station_led_green[4];
static lv_obj_t *left_station_led_red[4];
static lv_obj_t *left_block_occ_green[9];
static lv_obj_t *left_block_occ_red[9];
static lv_obj_t *left_block_grant_green[9];
static lv_obj_t *left_block_grant_red[9];
typedef enum {
    TRACK_LEVEL_E1 = 0,
    TRACK_LEVEL_E0 = 1,
    TRACK_LEVEL_SBHF = 2,
} track_level_t;

static lv_obj_t *track_level_btns[3];
static lv_obj_t *track_content_scroll = NULL;
static lv_obj_t *track_root_panel = NULL;
static lv_obj_t *track_layer_e1 = NULL;
static lv_obj_t *track_layer_e0 = NULL;
static lv_obj_t *track_layer_sbhf = NULL;
static bool track_layer_e1_created = false;
static bool track_layer_e0_created = false;
static bool track_layer_sbhf_created = false;
static track_level_t track_active_level = TRACK_LEVEL_E1;
static int track_panel_w = 0;
static int track_panel_h = 0;
static int32_t track_ox = 0;
static int32_t track_oy = 0;
static int32_t track_scale_q10 = 1024;
static uint8_t track_line_w = 6;
static uint8_t track_occ_line_w = 3;

static lv_obj_t *track_e1_block_occ_layer[3];
static lv_obj_t *track_e1_turnout_layer[3][2]; // W9..W11, [0]=abbiegen, [1]=gerade
static lv_obj_t *track_e1_turnout_warn_layer[3]; // W9..W11
static lv_obj_t *track_e1_signal_box[4];   // Ebene 1: Grant 1->2, Grant 2->3, Bhf2, Bhf3
static lv_obj_t *track_e1_signal_red[4];   // rote LED oben
static lv_obj_t *track_e1_signal_green[4]; // gruene LED unten
static lv_obj_t *track_e1_hitbox_turnout[3]; // Ebene 1: W9, W10, W11
static lv_obj_t *track_e1_hitbox_station[2]; // Ebene 1: Bhf2, Bhf3

static lv_obj_t *track_e0_block_occ_layer[5]; // Ebene 0: B1, B3, B4, B5, B6
static lv_obj_t *track_e0_turnout_layer[9][2]; // W0..W8, [0]=abbiegen, [1]=gerade
static lv_obj_t *track_e0_turnout_warn_layer[9];
static lv_obj_t *track_e0_signal_red[6];   // 3->4, 6->4, 4->1, 4->5, Bhf0, Bhf1
static lv_obj_t *track_e0_signal_green[6];
static lv_obj_t *track_e0_hitbox_turnout[9];
static lv_obj_t *track_e0_hitbox_station[2];

static lv_obj_t *track_sbhf_block_occ_layer[6]; // Block 4,5,6, SBHF1, SBHF2, SBHF3
static lv_obj_t *track_sbhf_turnout_layer[4][2]; // W12..W15, [0]=abbiegen, [1]=gerade
static lv_obj_t *track_sbhf_turnout_warn_layer[4];
static lv_obj_t *track_sbhf_target_layer[3];
static lv_obj_t *track_sbhf_signal_box[6];   // 4->5, 5->SBHF, 6->4, SBHF1/2/3->6
static lv_obj_t *track_sbhf_signal_red[6];
static lv_obj_t *track_sbhf_signal_green[6];
static lv_obj_t *track_sbhf_info_box = NULL;
static lv_obj_t *track_sbhf_info_label = NULL;
static lv_obj_t *track_hitbox_flash_obj = NULL;
static lv_timer_t *track_hitbox_flash_timer = NULL;

static lv_obj_t *overlay = NULL;
static lv_obj_t *overlay_panel = NULL;
static lv_obj_t *overlay_title = NULL;
static lv_obj_t *overlay_text = NULL;
static lv_obj_t *overlay_status = NULL;
static lv_obj_t *overlay_m2_btn = NULL;
static lv_obj_t *overlay_m1_btn = NULL;
static lv_obj_t *overlay_ack_btn = NULL;
static lv_obj_t *overlay_ip_label = NULL;

static lv_obj_t *retry_overlay = NULL;
static lv_obj_t *retry_panel = NULL;
static lv_obj_t *retry_title = NULL;
static lv_obj_t *retry_text = NULL;
static lv_obj_t *retry_status = NULL;
static lv_obj_t *retry_close_btn = NULL;
static lv_obj_t *retry_ip_label = NULL;

static bool g_startup_session_active = false;
static bool g_retry_overlay_dismissed = false;
static bool g_pending_startup_m1 = false;
static bool g_pending_startup_m2 = false;
static bool g_pending_startup_ack = false;
static bool g_pending_m1_retry = false;
static bool g_pending_m2_retry = false;

static volatile bool g_ui_dirty = false;
static hmi_state_t g_last_rendered_state;
static bool g_last_rendered_valid = false;
static bool g_last_overlay_visible = false;
static uint32_t g_power_led_bg_cache = 0xFFFFFFFFu;
static uint32_t g_auto_led_bg_cache = 0xFFFFFFFFu;

static bool snapshot_emergency_overlay_active(const hmi_state_t *s);
static void update_left_panel_ui(const hmi_state_t *s);
static void display_settings_load(void);
static void display_settings_save(void);
static void display_apply_brightness(void);
static void display_note_user_activity(bool from_touch);

static const cJSON *json_get_path(const cJSON *root, const char *a, const char *b)
{
    const cJSON *obj = cJSON_GetObjectItemCaseSensitive(root, a);
    if (!obj || !cJSON_IsObject(obj)) {
        return NULL;
    }
    return cJSON_GetObjectItemCaseSensitive(obj, b);
}

static const cJSON *json_get_path3(const cJSON *root, const char *a, const char *b, const char *c)
{
    const cJSON *obj = json_get_path(root, a, b);
    if (!obj || !cJSON_IsObject(obj)) {
        return NULL;
    }
    return cJSON_GetObjectItemCaseSensitive(obj, c);
}

static bool json_item_bool(const cJSON *v, bool fallback)
{
    if (cJSON_IsBool(v)) {
        return cJSON_IsTrue(v);
    }
    return fallback;
}

static uint32_t json_item_u32(const cJSON *v, uint32_t fallback)
{
    if (cJSON_IsNumber(v)) {
        return (uint32_t)v->valuedouble;
    }
    return fallback;
}

static int32_t json_item_i32(const cJSON *v, int32_t fallback)
{
    if (cJSON_IsNumber(v)) {
        return (int32_t)v->valuedouble;
    }
    return fallback;
}

static void append_text(char *out, size_t out_len, const char *text)
{
    if (!out || out_len == 0 || !text || !*text) return;

    size_t used = strlen(out);
    if (used >= out_len - 1) return;

    if (used > 0) {
        snprintf(out + used, out_len - used, ", ");
        used = strlen(out);
        if (used >= out_len - 1) return;
    }

    snprintf(out + used, out_len - used, "%s", text);
}

static void build_turnout_defects(char *out, size_t out_len, uint32_t mask, uint8_t first_idx, uint8_t count, const char *suffix)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';

    for (uint8_t i = 0; i < count; ++i) {
        if (mask & (1UL << i)) {
            char item[32];
            snprintf(item, sizeof(item), "W%u%s", (unsigned)(first_idx + i), suffix ? suffix : "");
            append_text(out, out_len, item);
        }
    }
}

static void build_sbhf_allowed_text(char *out, size_t out_len, uint32_t mask)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';

    if (mask & 0x01) append_text(out, out_len, "G1");
    if (mask & 0x02) append_text(out, out_len, "G2");
    if (mask & 0x04) append_text(out, out_len, "G3");

    if (out[0] == '\0') {
        snprintf(out, out_len, "-");
    }
}

static bool json_get_bool_path(const cJSON *root, const char *a, const char *b, bool fallback)
{
    const cJSON *v = json_get_path(root, a, b);
    return json_item_bool(v, fallback);
}

static bool json_get_bool_path3(const cJSON *root, const char *a, const char *b, const char *c, bool fallback)
{
    const cJSON *v = json_get_path3(root, a, b, c);
    return json_item_bool(v, fallback);
}

static uint32_t json_get_u32_path(const cJSON *root, const char *a, const char *b, uint32_t fallback)
{
    const cJSON *v = json_get_path(root, a, b);
    return json_item_u32(v, fallback);
}

static uint32_t json_get_u32_path3(const cJSON *root, const char *a, const char *b, const char *c, uint32_t fallback)
{
    const cJSON *v = json_get_path3(root, a, b, c);
    return json_item_u32(v, fallback);
}

static uint16_t json_get_u16_path(const cJSON *root, const char *a, const char *b, uint16_t fallback, uint16_t mask)
{
    return (uint16_t)(json_get_u32_path(root, a, b, fallback) & mask);
}

static bool json_try_get_u32_path(const cJSON *root, const char *a, const char *b, uint32_t *out)
{
    const cJSON *v = json_get_path(root, a, b);
    if (!out || !cJSON_IsNumber(v)) return false;
    *out = (uint32_t)v->valuedouble;
    return true;
}

static bool json_try_get_u32_path3(const cJSON *root, const char *a, const char *b, const char *c, uint32_t *out)
{
    const cJSON *v = json_get_path3(root, a, b, c);
    if (!out || !cJSON_IsNumber(v)) return false;
    *out = (uint32_t)v->valuedouble;
    return true;
}

static bool json_try_get_bool_path3(const cJSON *root, const char *a, const char *b, const char *c, bool *out)
{
    const cJSON *v = json_get_path3(root, a, b, c);
    if (!out || !cJSON_IsBool(v)) return false;
    *out = cJSON_IsTrue(v);
    return true;
}

static int32_t json_get_i32_root(const cJSON *root, const char *key, int32_t fallback)
{
    const cJSON *v = cJSON_GetObjectItemCaseSensitive(root, key);
    return json_item_i32(v, fallback);
}

static int32_t json_get_i32_path3(const cJSON *root, const char *a, const char *b, const char *c, int32_t fallback)
{
    const cJSON *v = json_get_path3(root, a, b, c);
    return json_item_i32(v, fallback);
}

static void format_voltage_v10(char *out, size_t out_len, int32_t v10)
{
    if (!out || out_len == 0) return;

    const char *sign = (v10 < 0) ? "-" : "";
    int32_t abs_v10 = (v10 < 0) ? -v10 : v10;
    snprintf(out, out_len, "%s%ld.%01ld V",
             sign,
             (long)(abs_v10 / 10),
             (long)(abs_v10 % 10));
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

static const char *turnout_dir_text(bool gerade)
{
    return gerade ? "G" : "A";
}

static void format_turnout_label(char *out, size_t out_len, unsigned turnout_no, bool valid, bool ist_gerade, bool soll_gerade)
{
    if (!out || out_len == 0) return;

    if (!valid) {
        snprintf(out, out_len, "W%u\nIst - | Soll -", turnout_no);
        return;
    }

    snprintf(out, out_len, "W%u\nIst %s | Soll %s", turnout_no, turnout_dir_text(ist_gerade), turnout_dir_text(soll_gerade));
}

static void hmi_uart_send_ack(uint32_t seq)
{
    if (seq == 0) return;

    char line[64];
    int len = snprintf(line, sizeof(line), "{\"type\":\"ack\",\"seq\":%lu}\n", (unsigned long)seq);

    if (len <= 0 || len >= (int)sizeof(line)) return;

    if (g_uart_tx_mutex) {
        xSemaphoreTake(g_uart_tx_mutex, portMAX_DELAY);
    }
    uart_write_bytes(HMI_UART_NUM, line, len);
    if (g_uart_tx_mutex) {
        xSemaphoreGive(g_uart_tx_mutex);
    }

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        g_state.ack_sent++;
        xSemaphoreGive(g_state_mutex);
    }
}

static void hmi_uart_send_action(const char *action)
{
    if (!action || !action[0]) return;

    char line[128];
    int len = snprintf(line, sizeof(line), "{\"type\":\"action\",\"action\":\"%s\"}\n", action);

    if (len <= 0 || len >= (int)sizeof(line)) return;

    if (g_uart_tx_mutex) {
        xSemaphoreTake(g_uart_tx_mutex, portMAX_DELAY);
    }
    uart_write_bytes(HMI_UART_NUM, line, len);
    if (g_uart_tx_mutex) {
        xSemaphoreGive(g_uart_tx_mutex);
    }

    ESP_LOGI(TAG, "TX action=%s", action);
}

static void hmi_uart_send_m1_weiche_set(uint8_t idx, bool gerade)
{
    if (idx >= 12u) return;

    char line[128];
    int len = snprintf(line, sizeof(line),
                       "{\"type\":\"action\",\"action\":\"m1WeicheSet\",\"idx\":%u,\"gerade\":%s}\n",
                       (unsigned)idx, gerade ? "true" : "false");

    if (len <= 0 || len >= (int)sizeof(line)) return;

    if (g_uart_tx_mutex) {
        xSemaphoreTake(g_uart_tx_mutex, portMAX_DELAY);
    }
    uart_write_bytes(HMI_UART_NUM, line, len);
    if (g_uart_tx_mutex) {
        xSemaphoreGive(g_uart_tx_mutex);
    }

    ESP_LOGI(TAG, "TX action=m1WeicheSet idx=%u gerade=%u", (unsigned)idx, gerade ? 1u : 0u);
}

static void hmi_uart_send_m1_power_set(uint8_t bhf, bool on)
{
    if (bhf >= 4u) return;

    char line[128];
    int len = snprintf(line, sizeof(line),
                       "{\"type\":\"action\",\"action\":\"m1PowerSet\",\"bhf\":%u,\"on\":%s}\n",
                       (unsigned)bhf, on ? "true" : "false");

    if (len <= 0 || len >= (int)sizeof(line)) return;

    if (g_uart_tx_mutex) {
        xSemaphoreTake(g_uart_tx_mutex, portMAX_DELAY);
    }
    uart_write_bytes(HMI_UART_NUM, line, len);
    if (g_uart_tx_mutex) {
        xSemaphoreGive(g_uart_tx_mutex);
    }

    ESP_LOGI(TAG, "TX action=m1PowerSet bhf=%u on=%u", (unsigned)bhf, on ? 1u : 0u);
}

static bool snapshot_overlay_blocks_left_commands(const hmi_state_t *s)
{
    if (!s) return true;
    return g_startup_session_active ||
           s->ui_startup_overlay_active ||
           s->ui_m1_retry_overlay_active ||
           s->ui_m2_retry_overlay_active ||
           snapshot_emergency_overlay_active(s);
}

static bool snapshot_can_send_m1_weiche_cmd(const hmi_state_t *s)
{
    return s &&
           s->can_write &&
           s->mega1_online &&
           s->startup_ready &&
           !s->safety_lock &&
           !s->notaus_active &&
           !snapshot_overlay_blocks_left_commands(s);
}

static bool snapshot_can_send_bhf_cmd(const hmi_state_t *s)
{
    return s &&
           s->can_write &&
           s->mega1_online &&
           s->startup_ready &&
           s->mega1_bahnhof_valid &&
           !s->safety_lock &&
           !s->notaus_active &&
           !snapshot_overlay_blocks_left_commands(s);
}

static const char *snapshot_left_command_block_reason(const hmi_state_t *s, bool require_bhf_data)
{
    if (!s) return "kein Status";
    if (snapshot_overlay_blocks_left_commands(s)) return "Overlay aktiv";
    if (!s->startup_ready) return "Startup aktiv";
    if (s->notaus_active) return "Notaus aktiv";
    if (s->safety_lock) return "Safety Lock";
    if (!s->can_write) return "Diag aktiv";
    if (!s->mega1_online) return "Mega1 offline";
    if (require_bhf_data && !s->mega1_bahnhof_valid) return "keine Bahnhofsdaten";
    return "frei";
}

static void left_set_clickable_feedback(lv_obj_t *obj, bool enabled)
{
    if (!obj) return;
    if (enabled) {
        lv_obj_clear_state(obj, LV_STATE_DISABLED);
        lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    } else {
        lv_obj_add_state(obj, LV_STATE_DISABLED);
        lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
    }
    lv_obj_set_style_text_opa(obj, LV_OPA_COVER, 0);
}

static void on_left_m1_turnout_clicked(lv_event_t *e)
{
    if (!e) return;
    const uint8_t idx = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (idx >= 12u) return;

    hmi_state_t s;
    memset(&s, 0, sizeof(s));
    if (!g_state_mutex || xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    s = g_state;
    xSemaphoreGive(g_state_mutex);

    if (!snapshot_can_send_m1_weiche_cmd(&s)) return;

    // Toggle bewusst ueber IST, nicht ueber SOLL. ETH bleibt Single Source of Truth.
    const bool ist_gerade = ((s.mega1_weiche_ist_bits & (1u << idx)) != 0u);
    hmi_uart_send_m1_weiche_set(idx, !ist_gerade);
}

static void on_left_station_clicked(lv_event_t *e)
{
    if (!e) return;
    const uint8_t bhf = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
    if (bhf >= 4u) return;

    hmi_state_t s;
    memset(&s, 0, sizeof(s));
    if (!g_state_mutex || xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) != pdTRUE) return;
    s = g_state;
    xSemaphoreGive(g_state_mutex);

    if (!snapshot_can_send_bhf_cmd(&s)) return;

    const bool is_on = ((s.mega1_bahnhof_mask & (1u << bhf)) != 0u);
    hmi_uart_send_m1_power_set(bhf, !is_on);
}

static void on_power_on_clicked(lv_event_t *e)
{
    (void)e;

    bool can_send = false;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        can_send = g_state.can_write && !g_state.power_on;
        xSemaphoreGive(g_state_mutex);
    }

    if (can_send) {
        hmi_uart_send_action("powerOn");
    }
}

static void on_power_off_clicked(lv_event_t *e)
{
    (void)e;

    bool can_send = false;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        can_send = g_state.can_write && g_state.power_on;
        xSemaphoreGive(g_state_mutex);
    }

    if (can_send) {
        hmi_uart_send_action("powerOff");
    }
}

static void on_auto_clicked(lv_event_t *e)
{
    (void)e;

    bool auto_mode = false;
    bool can_send = false;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        auto_mode = g_state.auto_mode;
        can_send = g_state.can_write;
        xSemaphoreGive(g_state_mutex);
    }

    if (can_send) {
        hmi_uart_send_action(auto_mode ? "setManual" : "setAuto");
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
            g_state.safety_lock = json_get_bool_path(root, "safety", "lock", g_state.safety_lock);

            g_state.startup_ready = json_get_bool_path(root, "startup", "ready", g_state.startup_ready);
            g_state.startup_checklist_active = json_get_bool_path(root, "startup", "checklistActive", g_state.startup_checklist_active);
            g_state.startup_m1_needs = json_get_bool_path(root, "startup", "m1Needs", g_state.startup_m1_needs);
            g_state.startup_m2_needs = json_get_bool_path(root, "startup", "m2Needs", g_state.startup_m2_needs);
            g_state.startup_m1_running = json_get_bool_path(root, "startup", "m1SelftestRunning", g_state.startup_m1_running);
            g_state.startup_m2_running = json_get_bool_path(root, "startup", "m2SelftestRunning", g_state.startup_m2_running);
            g_state.startup_m1_done = json_get_bool_path(root, "startup", "m1SelftestDone", g_state.startup_m1_done);
            g_state.startup_m2_done = json_get_bool_path(root, "startup", "m2SelftestDone", g_state.startup_m2_done);

            g_state.action_can_ack = json_get_bool_path(root, "actions", "canAck", g_state.action_can_ack);
            g_state.action_can_start_m1_selftest = json_get_bool_path(root, "actions", "canStartM1Selftest", g_state.action_can_start_m1_selftest);
            g_state.action_can_start_m2_selftest = json_get_bool_path(root, "actions", "canStartM2Selftest", g_state.action_can_start_m2_selftest);
            g_state.action_can_startup_confirm = json_get_bool_path(root, "actions", "canStartupConfirm", g_state.action_can_startup_confirm);
            g_state.mega1_selftest_retry_available = json_get_bool_path(root, "mega1", "selftestRetryAvailable", g_state.mega1_selftest_retry_available);
            g_state.mega2_selftest_retry_available = json_get_bool_path(root, "mega2", "selftestRetryAvailable", g_state.mega2_selftest_retry_available);

            g_state.ui_startup_overlay_active = json_get_bool_path(root, "ui", "startupOverlayActive", g_state.ui_startup_overlay_active);
            g_state.ui_m1_retry_overlay_active = json_get_bool_path(root, "ui", "m1RetryOverlayActive", g_state.ui_m1_retry_overlay_active);
            g_state.ui_m2_retry_overlay_active = json_get_bool_path(root, "ui", "m2RetryOverlayActive", g_state.ui_m2_retry_overlay_active);
            json_get_string_path(root, "ui", "overlayMode", g_state.ui_overlay_mode, sizeof(g_state.ui_overlay_mode));
            json_get_string_path(root, "ui", "retryScope", g_state.ui_retry_scope, sizeof(g_state.ui_retry_scope));

            if (g_state.startup_m1_running || g_state.startup_m1_done || !g_state.startup_m1_needs) {
                g_pending_startup_m1 = false;
            }
            if (g_state.startup_m2_running || g_state.startup_m2_done || !g_state.startup_m2_needs) {
                g_pending_startup_m2 = false;
            }
            if (!g_state.ack_required && !g_state.startup_checklist_active) {
                g_pending_startup_ack = false;
            }
            if (g_state.ui_m1_retry_overlay_active || g_state.ui_m2_retry_overlay_active) {
                g_retry_overlay_dismissed = false;
            }

            g_state.auto_mode = json_get_bool_path(root, "mega1", "modeAuto", g_state.auto_mode);
            g_state.mega1_online = json_get_bool_path(root, "mega1", "online", g_state.mega1_online);
            g_state.mega2_online = json_get_bool_path(root, "mega2", "online", g_state.mega2_online);

            /* Weichen-Daten: state-lite nutzt je nach ETH-Stand kurze oder explizite Feldnamen.
             * Mega1-Bits bedeuten gerade. Bei SBHF/Mega2 sind die Masken historisch
             * invertiert: Bit 0 bedeutet gerade, wenn es NICHT gesetzt ist.
             */
            g_state.mega1_weiche_ist_bits = json_get_u16_path(root, "mega1", "weicheIstBits", g_state.mega1_weiche_ist_bits, 0x0FFFu);
            g_state.mega1_weiche_ist_bits = json_get_u16_path(root, "mega1", "weicheIstGeradeBits", g_state.mega1_weiche_ist_bits, 0x0FFFu);
            g_state.mega1_weiche_soll_bits = json_get_u16_path(root, "mega1", "weicheSollBits", g_state.mega1_weiche_soll_bits, 0x0FFFu);
            g_state.mega1_weiche_soll_bits = json_get_u16_path(root, "mega1", "weicheSollGeradeBits", g_state.mega1_weiche_soll_bits, 0x0FFFu);

            g_state.mega2_turnout_ist_mask = json_get_u16_path(root, "mega2", "turnoutIstMask", g_state.mega2_turnout_ist_mask, 0x000Fu);
            g_state.mega2_turnout_soll_mask = json_get_u16_path(root, "mega2", "turnoutSollMask", g_state.mega2_turnout_soll_mask, 0x000Fu);

            uint32_t u32 = 0;
            if (json_try_get_u32_path(root, "mega1", "bahnhofMask", &u32)) {
                g_state.mega1_bahnhof_mask = (uint8_t)(u32 & 0x0Fu);
                g_state.mega1_bahnhof_valid = true;
            } else if (json_find_u32(payload, "\"bahnhofMask\"", &u32)) {
                g_state.mega1_bahnhof_mask = (uint8_t)(u32 & 0x0Fu);
                g_state.mega1_bahnhof_valid = true;
            }

            if (json_try_get_u32_path(root, "mega2", "blockOccMask", &u32) ||
                json_try_get_u32_path(root, "mega2", "blockOccupiedMask", &u32) ||
                json_try_get_u32_path(root, "mega2", "belegungMask", &u32) ||
                json_find_u32(payload, "\"blockOccMask\"", &u32)) {
                g_state.mega2_block_occ_mask = (uint16_t)(u32 & 0x01FFu);
                g_state.mega2_block_occ_valid = true;
            }

            if (json_try_get_u32_path(root, "mega2", "signalGrantMask", &u32) ||
                json_try_get_u32_path(root, "mega2", "signalGrantedMask", &u32) ||
                json_try_get_u32_path(root, "mega2", "routeGrantMask", &u32) ||
                json_try_get_u32_path(root, "mega2", "routeGrantedMask", &u32) ||
                json_find_u32(payload, "\"signalGrantMask\"", &u32)) {
                g_state.mega2_signal_grant_mask = (uint16_t)(u32 & 0x0FFFu);
                g_state.mega2_signal_grant_valid = true;
            }

            const cJSON *entry_allowed = json_get_path(root, "mega2", "entryAllowed");
            if (cJSON_IsArray(entry_allowed)) {
                memset(g_state.mega2_entry_allowed, 0, sizeof(g_state.mega2_entry_allowed));
                const int n = cJSON_GetArraySize(entry_allowed);
                const int max_n = (n < 9) ? n : 9;
                for (int i = 0; i < max_n; ++i) {
                    const cJSON *item = cJSON_GetArrayItem(entry_allowed, i);
                    if (cJSON_IsNumber(item)) {
                        g_state.mega2_entry_allowed[i] = (uint16_t)(((uint32_t)item->valuedouble) & 0x01FFu);
                    }
                }
                g_state.mega2_entry_allowed_valid = true;
            }

            if (json_try_get_u32_path3(root, "mega2", "sbhf", "state", &u32)) {
                g_state.mega2_sbhf_state = (uint8_t)(u32 & 0xFFu);
            }
            if (json_try_get_u32_path3(root, "mega2", "sbhf", "currentGleis", &u32)) {
                g_state.mega2_sbhf_current_gleis = (uint8_t)(u32 & 0xFFu);
            }
            bool b = false;
            if (json_try_get_bool_path3(root, "mega2", "sbhf", "block5ToSbhfActive", &b)) {
                g_state.mega2_block5_to_sbhf_active = b;
                g_state.mega2_block5_to_sbhf_valid = true;
            }

            uint32_t turnout_bits = 0;
            if (json_find_u32(payload, "\"mega1WeicheIstBits\"", &turnout_bits) ||
                json_find_u32(payload, "\"mega1WeicheIstGeradeBits\"", &turnout_bits) ||
                json_find_u32(payload, "\"weicheIstBits\"", &turnout_bits)) {
                g_state.mega1_weiche_ist_bits = (uint16_t)(turnout_bits & 0x0FFFu);
            }
            if (json_find_u32(payload, "\"mega1WeicheSollBits\"", &turnout_bits) ||
                json_find_u32(payload, "\"mega1WeicheSollGeradeBits\"", &turnout_bits) ||
                json_find_u32(payload, "\"weicheSollBits\"", &turnout_bits)) {
                g_state.mega1_weiche_soll_bits = (uint16_t)(turnout_bits & 0x0FFFu);
            }
            uint32_t mega1_warning_mask = json_get_u32_path(root, "mega1", "warningMask", 0);
            const uint32_t mega1_selftest_fail_mask = json_get_u32_path3(root, "mega1", "diag", "selftestFailMask", 0);
            // ETH/state-lite exposes Mega1-Weichenfehler je nach Stand entweder
            // als mega1.warningMask oder als mega1.diag.selftestFailMask. Fuer
            // die Gleisbild-Warnsymbole beide Quellen zusammenfuehren.
            mega1_warning_mask |= mega1_selftest_fail_mask;
            const uint32_t mega2_warning_mask = json_get_u32_path(root, "mega2", "warningMask", 0);
            const uint32_t sbhf_warning_mask = json_get_u32_path3(root, "mega2", "sbhf", "warningMask", 0);
            g_state.mega1_warning_mask = mega1_warning_mask;
            g_state.warning_present = (mega1_warning_mask != 0) || (mega2_warning_mask != 0) || (sbhf_warning_mask != 0);

            g_state.ws_base_clients = json_get_u32_path(root, "wsClients", "base", g_state.ws_base_clients);
            g_state.ws_diag_clients = json_get_u32_path(root, "wsClients", "diag", g_state.ws_diag_clients);
            /*
             * ETH state-lite currently exposes the diag lease as:
             *   "diag": { "active": true/false, ... }
             * Full WebUI state uses:
             *   "diagCtrl": { "active": true/false, ... }
             * Accept both so P4-HMI works with state-lite and full-state payloads.
             */
            bool diag_active = json_get_bool_path(root, "diagCtrl", "active", false);
            diag_active = json_get_bool_path(root, "diag", "active", diag_active);
            g_state.diag_active = diag_active;

            const bool diag_blocks_hmi = g_state.diag_active;
            const cJSON *ui_can_write = json_get_path(root, "ui", "canWrite");
            if (cJSON_IsBool(ui_can_write)) {
                g_state.can_write = cJSON_IsTrue(ui_can_write) && !diag_blocks_hmi;
            } else {
                g_state.can_write = !diag_blocks_hmi;
            }

            g_state.sbhf_allowed_mask = json_get_u32_path3(root, "mega2", "sbhf", "allowedMask", g_state.sbhf_allowed_mask);
            if (g_state.sbhf_allowed_mask == 0) {
                g_state.sbhf_allowed_mask = json_get_u32_path(root, "mega2", "allowedMask", g_state.sbhf_allowed_mask);
            }
            build_sbhf_allowed_text(g_state.sbhf_allowed_text, sizeof(g_state.sbhf_allowed_text), g_state.sbhf_allowed_mask);
            g_state.sbhf_occupied_mask = (uint8_t)(json_get_u32_path3(root, "mega2", "sbhf", "occupiedMask", g_state.sbhf_occupied_mask) & 0x07u);
            g_state.sbhf_restricted = json_get_bool_path3(root, "mega2", "sbhf", "restricted", g_state.sbhf_restricted);
            g_state.sbhf_start_pending = json_get_bool_path3(root, "mega2", "sbhf", "startPending", g_state.sbhf_start_pending);

            json_get_string_path(root, "mega1", "defectList", g_state.mega1_defects, sizeof(g_state.mega1_defects));
            json_get_string_path(root, "mega2", "defectList", g_state.mega2_defects, sizeof(g_state.mega2_defects));

            if (g_state.mega1_defects[0] == '\0') {
                build_turnout_defects(g_state.mega1_defects, sizeof(g_state.mega1_defects), mega1_selftest_fail_mask, 0, 12, "");
            }

            if (g_state.mega2_defects[0] == '\0') {
                if (sbhf_warning_mask & 0x01) append_text(g_state.mega2_defects, sizeof(g_state.mega2_defects), "W12 defekt");
                if (sbhf_warning_mask & 0x02) append_text(g_state.mega2_defects, sizeof(g_state.mega2_defects), "W13 defekt");
                if (sbhf_warning_mask & 0x04) append_text(g_state.mega2_defects, sizeof(g_state.mega2_defects), "W14 Stoerung");
                if (sbhf_warning_mask & 0x08) append_text(g_state.mega2_defects, sizeof(g_state.mega2_defects), "W15 Stoerung");
            }

            if (g_state.startup_m1_running || !g_state.mega1_selftest_retry_available || g_state.mega1_defects[0] == '\0') {
                g_pending_m1_retry = false;
            }
            if (g_state.startup_m2_running || !g_state.mega2_selftest_retry_available || g_state.mega2_defects[0] == '\0') {
                g_pending_m2_retry = false;
            }

            json_get_string_path(root, "eth", "ip", g_state.eth_ip, sizeof(g_state.eth_ip));

            /* Full-State fallback: analog values may arrive nested in state frames.
             * Values are transmitted as volts * 10 to avoid floats.
             */
            g_state.trafo_oben_v10 = json_get_i32_path3(root, "mega2", "analog", "vA10", g_state.trafo_oben_v10);
            g_state.trafo_unten_v10 = json_get_i32_path3(root, "mega2", "analog", "vB10", g_state.trafo_unten_v10);

            snprintf(g_state.last_state_json, sizeof(g_state.last_state_json), "%s", payload);
        } else if (strcmp(type, "analog") == 0) {
            g_state.analog_frames++;

            /* Analog frames carry vA10/vB10 as volts * 10.
             * Display code converts back to one decimal place.
             */
            g_state.trafo_oben_v10 = json_get_i32_root(root, "vA10", g_state.trafo_oben_v10);
            g_state.trafo_unten_v10 = json_get_i32_root(root, "vB10", g_state.trafo_unten_v10);

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

static uint32_t timeout_dropdown_to_sec(uint16_t sel)
{
    switch (sel) {
        case 0: return 0;
        case 1: return 30;
        case 2: return 60;
        case 3: return 120;
        case 4: return 300;
        case 5: return 600;
        default: return 120;
    }
}

static uint16_t timeout_sec_to_dropdown(uint32_t sec)
{
    if (sec == 0) return 0;
    if (sec <= 30) return 1;
    if (sec <= 60) return 2;
    if (sec <= 120) return 3;
    if (sec <= 300) return 4;
    return 5;
}

static void display_settings_load(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("hmi_settings", NVS_READONLY, &h);
    if (err != ESP_OK) {
        g_display_brightness_percent = 70;
        g_screen_off_timeout_sec = 120;
        return;
    }

    int32_t brightness = g_display_brightness_percent;
    uint32_t timeout_sec = g_screen_off_timeout_sec;
    (void)nvs_get_i32(h, "brightness", &brightness);
    (void)nvs_get_u32(h, "screen_off", &timeout_sec);
    nvs_close(h);

    if (brightness < 10) brightness = 10;
    if (brightness > 100) brightness = 100;
    g_display_brightness_percent = (int)brightness;

    switch (timeout_sec) {
        case 0:
        case 30:
        case 60:
        case 120:
        case 300:
        case 600:
            g_screen_off_timeout_sec = timeout_sec;
            break;
        default:
            g_screen_off_timeout_sec = 120;
            break;
    }
}

static void display_settings_save(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open("hmi_settings", NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open hmi_settings failed: %s", esp_err_to_name(err));
        return;
    }

    (void)nvs_set_i32(h, "brightness", g_display_brightness_percent);
    (void)nvs_set_u32(h, "screen_off", g_screen_off_timeout_sec);
    (void)nvs_commit(h);
    nvs_close(h);
}

static void update_settings_labels(void)
{
    if (settings_brightness_label) {
        char buf[48];
        snprintf(buf, sizeof(buf), "Helligkeit: %d %%", g_display_brightness_percent);
        lv_label_set_text(settings_brightness_label, buf);
    }

    if (settings_timeout_dropdown) {
        lv_dropdown_set_selected(settings_timeout_dropdown, timeout_sec_to_dropdown(g_screen_off_timeout_sec));
    }
}

static void display_apply_brightness(void)
{
    int brightness = g_display_brightness_percent;
    if (brightness < 10) brightness = 10;
    if (brightness > 100) brightness = 100;
    g_display_brightness_percent = brightness;

    if (!g_screen_off) {
        esp_err_t err = bsp_display_brightness_set(brightness);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "brightness_set(%d) failed: %s", brightness, esp_err_to_name(err));
        }
    }
}

static void screen_off_overlay_set_visible(bool visible)
{
    if (!screen_off_wake_overlay) return;
    if (visible) {
        lv_obj_clear_flag(screen_off_wake_overlay, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_foreground(screen_off_wake_overlay);
    } else {
        lv_obj_add_flag(screen_off_wake_overlay, LV_OBJ_FLAG_HIDDEN);
    }
}

static void display_screen_off_enter(void)
{
    if (g_screen_off) return;
    g_screen_off = true;
    screen_off_overlay_set_visible(true);
    (void)bsp_display_brightness_set(0);
}

static void display_screen_wake(void)
{
    if (!g_screen_off) return;
    g_screen_off = false;
    display_apply_brightness();
    screen_off_overlay_set_visible(false);
}

static void display_note_user_activity(bool from_touch)
{
    g_last_user_activity_ms = lv_tick_get();
    if (from_touch && g_screen_off) {
        display_screen_wake();
    }
}

static void screen_off_wake_event_cb(lv_event_t *e)
{
    (void)e;
    display_note_user_activity(true);
}

static void display_activity_event_cb(lv_event_t *e)
{
    (void)e;
    // Jede normale Touch-Bedienung setzt den Screen-Off-Timer zurueck.
    // Wenn der Screen bereits dunkel ist, faengt das Wake-Overlay den ersten
    // Touch ab; normale Objekte sollen dann keine Anlagenaktion ausloesen.
    if (!g_screen_off) {
        display_note_user_activity(false);
    }
}

static void display_attach_activity_recursive(lv_obj_t *obj)
{
    if (!obj) return;

    if (obj != screen_off_wake_overlay) {
        lv_obj_add_event_cb(obj, display_activity_event_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(obj, display_activity_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    }

    uint32_t child_count = lv_obj_get_child_cnt(obj);
    for (uint32_t i = 0; i < child_count; ++i) {
        display_attach_activity_recursive(lv_obj_get_child(obj, i));
    }
}

static void settings_brightness_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (!settings_brightness_slider) return;

    int v = (int)lv_slider_get_value(settings_brightness_slider);
    if (v < 10) v = 10;
    if (v > 100) v = 100;
    g_display_brightness_percent = v;
    update_settings_labels();
    display_apply_brightness();
    display_note_user_activity(true);

    if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        display_settings_save();
    }
}

static void settings_timeout_event_cb(lv_event_t *e)
{
    (void)e;
    if (!settings_timeout_dropdown) return;

    uint16_t sel = lv_dropdown_get_selected(settings_timeout_dropdown);
    g_screen_off_timeout_sec = timeout_dropdown_to_sec(sel);
    display_note_user_activity(true);
    display_settings_save();
}

static void display_power_timer_cb(lv_timer_t *timer)
{
    (void)timer;

    if (g_screen_off || g_screen_off_timeout_sec == 0) {
        return;
    }

    const uint32_t now = lv_tick_get();
    const uint32_t timeout_ms = g_screen_off_timeout_sec * 1000u;
    if ((uint32_t)(now - g_last_user_activity_ms) >= timeout_ms) {
        display_screen_off_enter();
    }
}

static const char *yesno(bool v)
{
    return v ? "JA" : "NEIN";
}

static void ui_label_style(lv_obj_t *label, const lv_font_t *font, uint32_t color)
{
    if (!label) return;
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
}

static bool ui_label_set_text_if_changed(lv_obj_t *label, const char *text)
{
    if (!label) return false;
    if (!text) text = "";

    const char *old = lv_label_get_text(label);
    if (old && strcmp(old, text) == 0) {
        return false;
    }

    lv_label_set_text(label, text);
    return true;
}

static void ui_obj_set_hidden_if_changed(lv_obj_t *obj, bool hidden)
{
    if (!obj) return;

    const bool is_hidden = lv_obj_has_flag(obj, LV_OBJ_FLAG_HIDDEN);
    if (is_hidden == hidden) {
        return;
    }

    if (hidden) lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
    else lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
}

static bool ui_obj_set_bg_color_if_changed(lv_obj_t *obj, uint32_t *cache, uint32_t color)
{
    if (!obj || !cache) return false;
    if (*cache == color) return false;

    *cache = color;
    lv_obj_set_style_bg_color(obj, lv_color_hex(color), 0);
    return true;
}

static void ui_card_style(lv_obj_t *obj, uint32_t bg)
{
    if (!obj) return;
    lv_obj_set_style_bg_color(obj, lv_color_hex(bg), 0);
    lv_obj_set_style_border_color(obj, lv_color_hex(0x8EA3AD), 0);
    lv_obj_set_style_border_width(obj, 2, 0);
    lv_obj_set_style_radius(obj, 10, 0);
    lv_obj_set_style_pad_all(obj, 8, 0);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t *ui_section_title(lv_obj_t *parent, const char *text, int x, int y)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    ui_label_style(label, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y);
    return label;
}

static lv_obj_t *ui_make_button(lv_obj_t *parent, const char *text, int x, int y, int w, int h, uint32_t bg, uint32_t fg)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, h);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_bg_color(btn, lv_color_hex(bg), 0);
    lv_obj_set_style_radius(btn, 8, 0);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    ui_label_style(label, &lv_font_montserrat_14, fg);
    lv_obj_center(label);
    return btn;
}

static lv_obj_t *ui_make_led(lv_obj_t *parent, int x, int y)
{
    lv_obj_t *led = lv_obj_create(parent);
    lv_obj_set_size(led, 12, 12);
    lv_obj_align(led, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(led, lv_color_hex(0xBBBBBB), 0);
    lv_obj_set_style_border_width(led, 0, 0);
    return led;
}

static lv_obj_t *ui_make_status_value(lv_obj_t *parent, const char *name, int y)
{
    lv_obj_t *name_label = lv_label_create(parent);
    lv_label_set_text(name_label, name);
    ui_label_style(name_label, &lv_font_montserrat_12, 0xDDE7EA);
    lv_obj_align(name_label, LV_ALIGN_TOP_LEFT, 8, y + 7);

    lv_obj_t *value = lv_label_create(parent);
    lv_label_set_text(value, "---");
    lv_obj_set_size(value, 92, 23);
    lv_obj_align(value, LV_ALIGN_TOP_RIGHT, -8, y);
    lv_obj_set_style_bg_color(value, lv_color_hex(0xA0A8AD), 0);
    lv_obj_set_style_bg_opa(value, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(value, 0, 0);
    lv_obj_set_style_pad_top(value, 4, 0);
    lv_obj_set_style_text_align(value, LV_TEXT_ALIGN_CENTER, 0);
    ui_label_style(value, &lv_font_montserrat_12, 0x101820);
    return value;
}

static void ui_set_status_value(lv_obj_t *label, const char *text, uint32_t bg, uint32_t fg)
{
    if (!label) return;
    ui_label_set_text_if_changed(label, text);
    lv_obj_set_style_bg_color(label, lv_color_hex(bg), 0);
    lv_obj_set_style_text_color(label, lv_color_hex(fg), 0);
}

static void ui_set_button_enabled(lv_obj_t *btn, bool enabled, uint32_t active_bg, uint32_t inactive_bg)
{
    if (!btn) return;

    if (enabled) {
        lv_obj_clear_state(btn, LV_STATE_DISABLED);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(btn, lv_color_hex(active_bg), 0);
        lv_obj_set_style_text_opa(btn, LV_OPA_COVER, 0);
    } else {
        lv_obj_add_state(btn, LV_STATE_DISABLED);
        lv_obj_clear_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_set_style_bg_color(btn, lv_color_hex(inactive_bg), 0);
        lv_obj_set_style_text_opa(btn, LV_OPA_COVER, 0);
    }
}


static const char *selftest_text(bool done, bool running, bool pending)
{
    if (done) return "OK";
    if (running || pending) return "LAEUFT";
    return "OFFEN";
}

static bool snapshot_startup_all_done(const hmi_state_t *s)
{
    return ((!s->startup_m1_needs) || s->startup_m1_done) &&
           ((!s->startup_m2_needs) || s->startup_m2_done);
}

static bool snapshot_emergency_overlay_active(const hmi_state_t *s)
{
    const bool startup_context = s->startup_checklist_active || s->startup_m1_needs || s->startup_m2_needs;
    return s->notaus_active || (s->ack_required && !startup_context);
}

static bool snapshot_startup_overlay_active(const hmi_state_t *s)
{
    if (snapshot_emergency_overlay_active(s)) {
        return false;
    }

    if (s->ui_startup_overlay_active ||
        s->startup_checklist_active ||
        s->startup_m1_needs ||
        s->startup_m2_needs ||
        s->ack_required) {
        g_startup_session_active = true;
    }

    if (!s->ui_startup_overlay_active &&
        !s->startup_checklist_active &&
        snapshot_startup_all_done(s) &&
        !s->ack_required &&
        s->startup_ready) {
        g_startup_session_active = false;
    }

    return g_startup_session_active;
}

static bool snapshot_can_send_m1_startup_test(const hmi_state_t *s)
{
    if (!s->can_write || g_pending_startup_m1) return false;
    if (s->startup_m1_running || s->startup_m1_done) return false;
    return s->action_can_start_m1_selftest || s->startup_m1_needs;
}

static bool snapshot_can_send_m2_startup_test(const hmi_state_t *s)
{
    if (!s->can_write || g_pending_startup_m2) return false;
    if (s->startup_m2_running || s->startup_m2_done) return false;
    return s->action_can_start_m2_selftest || s->startup_m2_needs || s->notaus_active;
}

static bool snapshot_can_send_startup_confirm(const hmi_state_t *s)
{
    if (!s->can_write || g_pending_startup_ack) return false;
    if (!snapshot_startup_all_done(s)) return false;
    return s->action_can_startup_confirm || s->ack_required || s->startup_checklist_active;
}

static bool snapshot_can_send_safety_ack(const hmi_state_t *s)
{
    if (!s->can_write || !s->ack_required) return false;
    return s->action_can_ack || s->safety_lock || s->notaus_active;
}

static bool snapshot_has_m1_defect(const hmi_state_t *s)
{
    return s && s->mega1_defects[0] != '\0';
}

static bool snapshot_has_m2_defect(const hmi_state_t *s)
{
    return s && s->mega2_defects[0] != '\0';
}

static bool snapshot_can_send_m1_retry(const hmi_state_t *s)
{
    if (!s || !s->can_write || s->notaus_active || g_pending_m1_retry) return false;
    if (!s->mega1_online || !snapshot_has_m1_defect(s)) return false;
    if (!s->mega1_selftest_retry_available || s->startup_m1_running) return false;
    return true;
}

static bool snapshot_can_send_m2_retry(const hmi_state_t *s)
{
    if (!s || !s->can_write || s->notaus_active || g_pending_m2_retry) return false;
    if (!s->mega2_online || !snapshot_has_m2_defect(s)) return false;
    if (!s->mega2_selftest_retry_available || s->startup_m2_running) return false;
    return true;
}

static void send_startup_confirm_sequence(const hmi_state_t *s)
{
    if (s->mega1_online) {
        hmi_uart_send_action("setAuto");
    }
    if (s->startup_m1_needs) {
        hmi_uart_send_action("markMega1ChecklistDone");
    }
    if (s->startup_m2_needs) {
        hmi_uart_send_action("markMega2ChecklistDone");
    }
    if (s->safety_lock || s->ack_required) {
        hmi_uart_send_action("safetyAck");
    }
}

static void on_overlay_m1_clicked(lv_event_t *e)
{
    (void)e;
    hmi_state_t s;
    bool send = false;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = g_state;
        send = snapshot_can_send_m1_startup_test(&s);
        if (send) {
            g_pending_startup_m1 = true;
            g_ui_dirty = true;
        }
        xSemaphoreGive(g_state_mutex);
    }
    if (send) {
        hmi_uart_send_action("m1SelftestStart");
    }
}

static void on_overlay_m2_clicked(lv_event_t *e)
{
    (void)e;
    hmi_state_t s;
    bool send = false;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = g_state;
        send = snapshot_can_send_m2_startup_test(&s);
        if (send) {
            g_pending_startup_m2 = true;
            g_ui_dirty = true;
        }
        xSemaphoreGive(g_state_mutex);
    }
    if (send) {
        hmi_uart_send_action(snapshot_emergency_overlay_active(&s) ? "sbhfSelftestRetry" : "sbhfSelftestStartup");
    }
}

static void on_overlay_ack_clicked(lv_event_t *e)
{
    (void)e;
    hmi_state_t s;
    bool send_ack = false;
    bool send_startup = false;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = g_state;
        if (snapshot_emergency_overlay_active(&s)) {
            send_ack = snapshot_can_send_safety_ack(&s);
        } else {
            send_startup = snapshot_can_send_startup_confirm(&s);
        }
        if (send_ack || send_startup) {
            g_pending_startup_ack = true;
            g_ui_dirty = true;
        }
        xSemaphoreGive(g_state_mutex);
    }
    if (send_ack) {
        hmi_uart_send_action("safetyAck");
    } else if (send_startup) {
        send_startup_confirm_sequence(&s);
    }
}

static void on_m1_retry_clicked(lv_event_t *e)
{
    (void)e;
    hmi_state_t s;
    bool send = false;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = g_state;
        send = snapshot_can_send_m1_retry(&s);
        if (send) {
            g_pending_m1_retry = true;
            g_retry_overlay_dismissed = false;
            g_ui_dirty = true;
        }
        xSemaphoreGive(g_state_mutex);
    }
    if (send) {
        hmi_uart_send_action("powerOff");
        hmi_uart_send_action("m1SelftestStart");
    }
}

static void on_m2_retry_clicked(lv_event_t *e)
{
    (void)e;
    hmi_state_t s;
    bool send = false;
    if (g_state_mutex && xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        s = g_state;
        send = snapshot_can_send_m2_retry(&s);
        if (send) {
            g_pending_m2_retry = true;
            g_retry_overlay_dismissed = false;
            g_ui_dirty = true;
        }
        xSemaphoreGive(g_state_mutex);
    }
    if (send) {
        hmi_uart_send_action("sbhfSelftestRetry");
    }
}

static void on_retry_close_clicked(lv_event_t *e)
{
    (void)e;
    g_retry_overlay_dismissed = true;
    if (retry_overlay) {
        ui_obj_set_hidden_if_changed(retry_overlay, true);
    }
}

static lv_obj_t *ui_make_overlay_button(lv_obj_t *parent, const char *text)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_height(btn, 52);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x00BCE3), 0);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    ui_label_style(label, &lv_font_montserrat_18, 0xFFFFFF);
    lv_obj_center(label);
    return btn;
}

static void overlay_update_ip_label(lv_obj_t *label, const hmi_state_t *s)
{
    if (!label || !s) return;
    char buf[64];
    snprintf(buf, sizeof(buf), "ETH: %s", s->eth_ip[0] ? s->eth_ip : "offline");
    ui_label_set_text_if_changed(label, buf);
}

static void create_overlay_ui(lv_obj_t *screen)
{
    overlay = lv_obj_create(screen);
    lv_obj_set_size(overlay, lv_pct(100), lv_pct(100));
    lv_obj_align(overlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(overlay, LV_OPA_70, 0);
    lv_obj_set_style_bg_color(overlay, lv_color_black(), 0);
    lv_obj_set_style_border_width(overlay, 0, 0);
    lv_obj_set_style_pad_all(overlay, 0, 0);

    overlay_panel = lv_obj_create(overlay);
    lv_obj_set_width(overlay_panel, lv_pct(72));
    lv_obj_set_height(overlay_panel, LV_SIZE_CONTENT);
    lv_obj_center(overlay_panel);
    lv_obj_set_style_radius(overlay_panel, 14, 0);
    lv_obj_set_style_pad_all(overlay_panel, 18, 0);
    lv_obj_set_layout(overlay_panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(overlay_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(overlay_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(overlay_panel, 10, 0);

    overlay_title = lv_label_create(overlay_panel);
    ui_label_style(overlay_title, &lv_font_montserrat_24, 0x1B2B34);
    ui_label_set_text_if_changed(overlay_title, "Systemstart - Checkliste");

    overlay_text = lv_label_create(overlay_panel);
    lv_obj_set_width(overlay_text, lv_pct(100));
    lv_label_set_long_mode(overlay_text, LV_LABEL_LONG_WRAP);
    ui_label_style(overlay_text, &lv_font_montserrat_16, 0x1B2B34);

    overlay_status = lv_label_create(overlay_panel);
    lv_obj_set_width(overlay_status, lv_pct(100));
    lv_label_set_long_mode(overlay_status, LV_LABEL_LONG_WRAP);
    ui_label_style(overlay_status, &lv_font_montserrat_16, 0x1B2B34);

    overlay_m2_btn = ui_make_overlay_button(overlay_panel, "SBHF TEST");
    lv_obj_set_width(overlay_m2_btn, lv_pct(100));
    lv_obj_add_event_cb(overlay_m2_btn, on_overlay_m2_clicked, LV_EVENT_CLICKED, NULL);

    overlay_m1_btn = ui_make_overlay_button(overlay_panel, "MEGA1 TEST");
    lv_obj_set_width(overlay_m1_btn, lv_pct(100));
    lv_obj_add_event_cb(overlay_m1_btn, on_overlay_m1_clicked, LV_EVENT_CLICKED, NULL);

    overlay_ack_btn = ui_make_overlay_button(overlay_panel, "QUITTIEREN");
    lv_obj_set_width(overlay_ack_btn, lv_pct(100));
    lv_obj_add_event_cb(overlay_ack_btn, on_overlay_ack_clicked, LV_EVENT_CLICKED, NULL);

    overlay_ip_label = lv_label_create(overlay);
    ui_label_style(overlay_ip_label, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_set_style_text_opa(overlay_ip_label, LV_OPA_90, 0);
    lv_obj_align(overlay_ip_label, LV_ALIGN_BOTTOM_RIGHT, -18, -48);
    ui_obj_set_hidden_if_changed(overlay, true);

    retry_overlay = lv_obj_create(screen);
    lv_obj_set_size(retry_overlay, lv_pct(100), lv_pct(100));
    lv_obj_align(retry_overlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(retry_overlay, LV_OPA_70, 0);
    lv_obj_set_style_bg_color(retry_overlay, lv_color_black(), 0);
    lv_obj_set_style_border_width(retry_overlay, 0, 0);
    lv_obj_set_style_pad_all(retry_overlay, 0, 0);

    retry_panel = lv_obj_create(retry_overlay);
    lv_obj_set_width(retry_panel, lv_pct(72));
    lv_obj_set_height(retry_panel, LV_SIZE_CONTENT);
    lv_obj_center(retry_panel);
    lv_obj_set_style_radius(retry_panel, 14, 0);
    lv_obj_set_style_pad_all(retry_panel, 18, 0);
    lv_obj_set_layout(retry_panel, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(retry_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(retry_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(retry_panel, 10, 0);

    retry_title = lv_label_create(retry_panel);
    ui_label_style(retry_title, &lv_font_montserrat_24, 0x1B2B34);
    ui_label_set_text_if_changed(retry_title, "Weichentest laeuft");

    retry_text = lv_label_create(retry_panel);
    lv_obj_set_width(retry_text, lv_pct(100));
    lv_label_set_long_mode(retry_text, LV_LABEL_LONG_WRAP);
    ui_label_style(retry_text, &lv_font_montserrat_16, 0x1B2B34);
    lv_label_set_text(retry_text, "Bitte warten ...");

    retry_status = lv_label_create(retry_panel);
    lv_obj_set_width(retry_status, lv_pct(100));
    lv_label_set_long_mode(retry_status, LV_LABEL_LONG_WRAP);
    ui_label_style(retry_status, &lv_font_montserrat_16, 0x1B2B34);

    retry_close_btn = ui_make_overlay_button(retry_panel, "AUSBLENDEN");
    lv_obj_set_width(retry_close_btn, lv_pct(100));
    lv_obj_add_event_cb(retry_close_btn, on_retry_close_clicked, LV_EVENT_CLICKED, NULL);

    retry_ip_label = lv_label_create(retry_overlay);
    ui_label_style(retry_ip_label, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_set_style_text_opa(retry_ip_label, LV_OPA_90, 0);
    lv_obj_align(retry_ip_label, LV_ALIGN_BOTTOM_RIGHT, -18, -48);
    ui_obj_set_hidden_if_changed(retry_overlay, true);
}


static bool update_overlay_ui(const hmi_state_t *s)
{
    if (!s || !overlay || !retry_overlay) return false;

    const bool emergency_active = snapshot_emergency_overlay_active(s);
    const bool startup_active = snapshot_startup_overlay_active(s);
    const bool retry_active = (!emergency_active && !startup_active && !g_retry_overlay_dismissed &&
                               (s->ui_m1_retry_overlay_active || s->ui_m2_retry_overlay_active));

    if (emergency_active) {
        ui_obj_set_hidden_if_changed(overlay, false);
        lv_obj_move_foreground(overlay);
        ui_label_set_text_if_changed(overlay_title, "Sicherheitsmeldung");
        ui_label_set_text_if_changed(overlay_text, "NOTAUS oder Sicherheitsquittierung ist aktiv. Ursache pruefen und erst danach quittieren.");

        char st[192];
        snprintf(st, sizeof(st),
                 "Safety: %s\nACK erforderlich: %s\nMega1: %s   Mega2: %s",
                 s->notaus_active ? "NOTAUS" : "LOCK",
                 yesno(s->ack_required),
                 s->mega1_online ? "Online" : "Offline",
                 s->mega2_online ? "Online" : "Offline");
        ui_label_set_text_if_changed(overlay_status, st);

        ui_obj_set_hidden_if_changed(overlay_m1_btn, true);
        if (s->mega2_defects[0] != '\0' || s->startup_m2_needs) {
            ui_obj_set_hidden_if_changed(overlay_m2_btn, false);
            ui_set_button_enabled(overlay_m2_btn, snapshot_can_send_m2_startup_test(s), 0x00BCE3, 0x9DDDE8);
        } else {
            ui_obj_set_hidden_if_changed(overlay_m2_btn, true);
        }
        ui_obj_set_hidden_if_changed(overlay_ack_btn, false);
        ui_set_button_enabled(overlay_ack_btn, snapshot_can_send_safety_ack(s), 0xF9D342, 0xDDC978);
        overlay_update_ip_label(overlay_ip_label, s);
    } else if (startup_active) {
        ui_obj_set_hidden_if_changed(overlay, false);
        lv_obj_move_foreground(overlay);
        ui_label_set_text_if_changed(overlay_title, "Systemstart - Checkliste");
        ui_label_set_text_if_changed(overlay_text, "Bitte die offenen Selftests ausfuehren. Quittieren wird erst freigegeben, wenn alle notwendigen Punkte OK sind.");

        char st[256];
        snprintf(st, sizeof(st),
                 "Mega1 Selftest: %s\nSBHF Selftest:  %s\nACK erforderlich: %s\nSchreibrechte: %s",
                 selftest_text(s->startup_m1_done, s->startup_m1_running, g_pending_startup_m1),
                 selftest_text(s->startup_m2_done, s->startup_m2_running, g_pending_startup_m2),
                 yesno(s->ack_required),
                 s->can_write ? "Frei" : "Gesperrt");
        ui_label_set_text_if_changed(overlay_status, st);

        if (s->startup_m2_needs || !s->startup_m2_done) ui_obj_set_hidden_if_changed(overlay_m2_btn, false);
        else ui_obj_set_hidden_if_changed(overlay_m2_btn, true);
        if (s->startup_m1_needs || !s->startup_m1_done) ui_obj_set_hidden_if_changed(overlay_m1_btn, false);
        else ui_obj_set_hidden_if_changed(overlay_m1_btn, true);
        ui_obj_set_hidden_if_changed(overlay_ack_btn, false);

        ui_set_button_enabled(overlay_m2_btn, snapshot_can_send_m2_startup_test(s), 0x00BCE3, 0x9DDDE8);
        ui_set_button_enabled(overlay_m1_btn, snapshot_can_send_m1_startup_test(s), 0x00BCE3, 0x9DDDE8);
        ui_set_button_enabled(overlay_ack_btn, snapshot_can_send_startup_confirm(s), 0xF9D342, 0xDDC978);
        overlay_update_ip_label(overlay_ip_label, s);
    } else {
        ui_obj_set_hidden_if_changed(overlay, true);
    }

    if (retry_active) {
        ui_obj_set_hidden_if_changed(retry_overlay, false);
        lv_obj_move_foreground(retry_overlay);
        ui_label_set_text_if_changed(retry_title, "Weichentest laeuft");
        ui_label_set_text_if_changed(retry_text, "Ein Retry-Selbsttest wurde gestartet. Das HMI bleibt gesperrt, bis ETH den neuen Zustand meldet.");

        char st[160];
        snprintf(st, sizeof(st),
                 "Mega1: %s\nSBHF:  %s",
                 selftest_text(s->startup_m1_done, s->startup_m1_running, false),
                 selftest_text(s->startup_m2_done, s->startup_m2_running, false));
        ui_label_set_text_if_changed(retry_status, st);
        overlay_update_ip_label(retry_ip_label, s);
    } else {
        ui_obj_set_hidden_if_changed(retry_overlay, true);
    }

    return emergency_active || startup_active || retry_active;
}

static void uart_ui_timer_cb(lv_timer_t *timer)
{
    hmi_state_t s;
    bool dirty = true;

    if (g_state_mutex && xSemaphoreTake(g_state_mutex, 0) == pdTRUE) {
        s = g_state;
        dirty = g_ui_dirty;
        g_ui_dirty = false;
        xSemaphoreGive(g_state_mutex);
    } else {
        return;
    }

    /* Globales Dirty-Gating:
     * Ohne neue Daten oder lokale Button-Pending-Aenderung wird LVGL gar nicht
     * angefasst. Das ist wichtiger als einzelne set_text-Optimierungen, weil
     * bereits wiederholtes Layout/Style-Setzen grosse Flaechen invalidieren kann.
     */
    if (!dirty && g_last_rendered_valid) {
        return;
    }

    const bool overlay_visible = update_overlay_ui(&s);


    // Performance-Schutz: Solange ein Overlay sichtbar ist, bleibt die normale
    // Hintergrund-UI eingefroren. Dadurch werden rechte/linke Panels nicht
    // unnoetig neu gelayoutet oder neu gezeichnet, waehrend das Overlay die
    // Bedienung ohnehin sperrt. Overlay-Inhalte selbst bleiben oben aktuell.
    if (overlay_visible) {
        g_last_overlay_visible = true;
        return;
    }

    const bool overlay_just_closed = g_last_overlay_visible;
    g_last_overlay_visible = false;

    /* Nach dem Ausblenden eines Overlays einmal vollen Panel-Refresh zulassen,
     * auch wenn sich die Werte waehrend des Overlays nur gesammelt haben.
     */
    if (overlay_just_closed) {
        g_last_rendered_valid = false;
    }

    update_left_panel_ui(&s);

    if (power_led) {
        ui_obj_set_bg_color_if_changed(power_led, &g_power_led_bg_cache,
            s.power_on ? 0x2ECC71 : 0x555555);
    }

    if (auto_led) {
        ui_obj_set_bg_color_if_changed(auto_led, &g_auto_led_bg_cache,
            s.auto_mode ? 0x2ECC71 : 0x555555);
    }

    const bool can_operate = s.can_write;
    const bool power_on_enabled = can_operate && !s.power_on;
    const bool power_off_enabled = can_operate && s.power_on;
    const bool auto_enabled = can_operate;

    ui_set_button_enabled(power_on_btn, power_on_enabled, 0x19B65A, 0xA8E6B8);
    ui_set_button_enabled(power_off_btn, power_off_enabled, 0xC83A32, 0xF3A09A);
    ui_set_button_enabled(auto_btn, auto_enabled, 0x00BCE3, 0x9DDDE8);

    if (auto_btn) {
        lv_obj_t *auto_label = lv_obj_get_child(auto_btn, 0);
        if (auto_label) {
            ui_label_set_text_if_changed(auto_label, s.auto_mode ? "Manuell" : "Auto");
        }
    }

    if (trafo_label) {
        char top[24];
        char bottom[24];
        char trafo_buf[80];
        format_voltage_v10(top, sizeof(top), s.trafo_oben_v10);
        format_voltage_v10(bottom, sizeof(bottom), s.trafo_unten_v10);
        snprintf(trafo_buf, sizeof(trafo_buf),
                 "Trafo oben:  %s\nTrafo unten: %s",
                 top,
                 bottom);
        ui_label_set_text_if_changed(trafo_label, trafo_buf);
    }

    if (write_label) {
        ui_label_set_text_if_changed(write_label, s.can_write ? "Bedienung: Frei" : "Bedienung gesperrt");
        lv_obj_set_style_text_color(write_label,
            s.can_write ? lv_color_hex(0x2ECC71) : lv_color_hex(0xF9D342), 0);
    }

    if (system_label) {
        char sysbuf[384];
        snprintf(sysbuf, sizeof(sysbuf),
            "ETH: %s\n"
            "Mega1: %s\n"
            "Mega2: %s\n"
            "Safety: %s\n"
            "Warning: %s\n"
            "Power: %s\n"
            "Modus: %s\n"
            "WS/Diag: %lu/%lu %s",
            s.eth_ip[0] ? s.eth_ip : "offline",
            s.mega1_online ? "Online" : "Offline",
            s.mega2_online ? "Online" : "Offline",
            s.notaus_active ? "Notaus" : "Ok",
            s.warning_present ? "An" : "Aus",
            s.power_on ? "An" : "Aus",
            s.auto_mode ? "Auto" : "Manuell",
            (unsigned long)s.ws_base_clients,
            (unsigned long)s.ws_diag_clients,
            s.can_write ? "frei" : "gesperrt"
        );
        ui_label_set_text_if_changed(system_label, sysbuf);
    }

    if (eth_value_label) {
        ui_set_status_value(eth_value_label, s.eth_ip[0] ? s.eth_ip : "OFFLINE",
                            s.eth_ip[0] ? 0x2ECC71 : 0xF25F4C,
                            0xFFFFFF);
    }
    if (mega1_value_label) {
        ui_set_status_value(mega1_value_label, s.mega1_online ? "ONLINE" : "OFFLINE",
                            s.mega1_online ? 0x2ECC71 : 0xF25F4C, 0xFFFFFF);
    }
    if (mega2_value_label) {
        ui_set_status_value(mega2_value_label, s.mega2_online ? "ONLINE" : "OFFLINE",
                            s.mega2_online ? 0x2ECC71 : 0xF25F4C, 0xFFFFFF);
    }
    if (safety_value_label) {
        ui_set_status_value(safety_value_label, s.notaus_active ? "NOTAUS" : "OK",
                            s.notaus_active ? 0xF25F4C : 0x2ECC71, 0xFFFFFF);
    }
    if (warning_value_label) {
        ui_set_status_value(warning_value_label, s.warning_present ? "AN" : "AUS",
                            s.warning_present ? 0xF9D342 : 0x2ECC71,
                            s.warning_present ? 0x101820 : 0xFFFFFF);
    }
    if (power_value_label) {
        ui_set_status_value(power_value_label, s.power_on ? "AN" : "AUS",
                            s.power_on ? 0x2ECC71 : 0xF25F4C, 0xFFFFFF);
    }
    if (mode_value_label) {
        ui_set_status_value(mode_value_label, s.auto_mode ? "AUTO" : "MANUELL",
                            s.auto_mode ? 0x2ECC71 : 0xA0A8AD,
                            s.auto_mode ? 0xFFFFFF : 0x101820);
    }
    if (ws_diag_value_label) {
        char ws_diag_buf[32];
        snprintf(ws_diag_buf, sizeof(ws_diag_buf), "%lu/%lu",
                 (unsigned long)s.ws_base_clients, (unsigned long)s.ws_diag_clients);
        ui_set_status_value(ws_diag_value_label, ws_diag_buf,
                            s.can_write ? 0x2ECC71 : 0xF9D342,
                            s.can_write ? 0xFFFFFF : 0x101820);
    }

    if (fault_m1_label) {
        char m1_buf[192];
        snprintf(m1_buf, sizeof(m1_buf), "Mega1: %s",
                 s.mega1_defects[0] ? s.mega1_defects : "keine Defekte");
        ui_label_set_text_if_changed(fault_m1_label, m1_buf);
    }
    if (fault_m2_label) {
        char m2_buf[192];
        snprintf(m2_buf, sizeof(m2_buf), "SBHF: %s",
                 s.mega2_defects[0] ? s.mega2_defects : "keine Defekte");
        ui_label_set_text_if_changed(fault_m2_label, m2_buf);
    }

    // Defekte-Karte ist ein Flex-Container mit LV_SIZE_CONTENT.
    // Layout nur neu berechnen, wenn sich die Defekttexte wirklich geaendert haben.
    const bool faults_changed = (!g_last_rendered_valid) ||
        strcmp(s.mega1_defects, g_last_rendered_state.mega1_defects) != 0 ||
        strcmp(s.mega2_defects, g_last_rendered_state.mega2_defects) != 0;
    if (faults_changed && fault_card_obj) {
        lv_obj_update_layout(fault_card_obj);
    }
    if (faults_changed && messages_card_obj && fault_card_obj) {
        lv_obj_align_to(messages_card_obj, fault_card_obj, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 8);
    }

    ui_set_button_enabled(m1_retry_btn, snapshot_can_send_m1_retry(&s), 0x2D7FF9, 0x53616A);
    ui_set_button_enabled(m2_retry_btn, snapshot_can_send_m2_retry(&s), 0x2D7FF9, 0x53616A);

    if (messages_ack_label || messages_m1_label || messages_sbhf_label || messages_mode_label || messages_allowed_label) {
        const bool has_m1_defect = s.mega1_defects[0] != '\0';
        const bool has_m2_defect = s.mega2_defects[0] != '\0';
        
        if (messages_ack_label) {
            ui_label_set_text_if_changed(messages_ack_label, "ACK erforderlich");
            ui_obj_set_hidden_if_changed(messages_ack_label, !s.ack_required);
        }

        if (messages_m1_label) {
            ui_label_set_text_if_changed(messages_m1_label,
                "Mega1: Eine oder mehrere Weichen schalten nicht in die Sollstellung.");
            ui_obj_set_hidden_if_changed(messages_m1_label, !has_m1_defect);
        }

        if (messages_sbhf_label) {
            ui_label_set_text_if_changed(messages_sbhf_label,
                "SBHF: Eine oder mehrere Weichen melden Stoerung/Defekt.");
            ui_obj_set_hidden_if_changed(messages_sbhf_label, !has_m2_defect);
        }

        if (messages_mode_label) {
            ui_label_set_text_if_changed(messages_mode_label,
                s.sbhf_restricted ? "SBHF: eingeschraenkter Betrieb aktiv" : "SBHF: normaler Betrieb");
            lv_obj_set_style_text_color(messages_mode_label,
                s.sbhf_restricted ? lv_color_hex(0xF9D342) : lv_color_hex(0x2ECC71), 0);
            ui_obj_set_hidden_if_changed(messages_mode_label, false);
        }

        if (messages_allowed_label) {
            char allowed_buf[64];
            snprintf(allowed_buf, sizeof(allowed_buf), "SBHF erlaubte Gleise: %s",
                     s.sbhf_allowed_text[0] ? s.sbhf_allowed_text : "-");
            ui_label_set_text_if_changed(messages_allowed_label, allowed_buf);
            ui_obj_set_hidden_if_changed(messages_allowed_label, false);
        }
    }

    if (uart_status_label) {
        char status[256];
        snprintf(
            status,
            sizeof(status),
            "RX frames: %lu | bytes: %lu | seq: %lu | ack: %lu\n"
            "lenErr: %lu | badJson: %lu | letzter Typ: %s",
            (unsigned long)s.rx_frames,
            (unsigned long)s.rx_bytes,
            (unsigned long)s.last_seq,
            (unsigned long)s.ack_sent,
            (unsigned long)s.rx_len_err,
            (unsigned long)s.rx_bad_json,
            s.last_type
        );
        ui_label_set_text_if_changed(uart_status_label, status);
    }

    if (comm_status_label) {
        char comm[192];
        snprintf(
            comm,
            sizeof(comm),
            "state-lite: %lu\nanalog:     %lu\nother:      %lu",
            (unsigned long)s.state_frames,
            (unsigned long)s.analog_frames,
            (unsigned long)s.other_frames
        );
        ui_label_set_text_if_changed(comm_status_label, comm);
    }

    g_last_rendered_state = s;
    g_last_rendered_valid = true;
}

static lv_obj_t *ui_make_pill(lv_obj_t *parent, const char *text, int x, int y, int w, int h, uint32_t bg, uint32_t fg)
{
    lv_obj_t *pill = lv_obj_create(parent);
    lv_obj_set_size(pill, w, h);
    lv_obj_align(pill, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_bg_color(pill, lv_color_hex(bg), 0);
    lv_obj_set_style_bg_opa(pill, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(pill, lv_color_hex(0x78909C), 0);
    lv_obj_set_style_border_width(pill, 1, 0);
    lv_obj_set_style_radius(pill, 8, 0);
    lv_obj_set_style_pad_all(pill, 6, 0);
    lv_obj_clear_flag(pill, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label = lv_label_create(pill);
    lv_label_set_text(label, text);
    lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(label, w - 12);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    ui_label_style(label, &lv_font_montserrat_14, fg);
    lv_obj_center(label);
    return pill;
}

static void ui_led_set_color(lv_obj_t *led, uint32_t color)
{
    if (!led) return;
    lv_obj_set_style_bg_color(led, lv_color_hex(color), 0);
}

static lv_obj_t *ui_make_signal_led(lv_obj_t *parent, int x, int y, uint32_t color)
{
    lv_obj_t *led = lv_obj_create(parent);
    lv_obj_set_size(led, 15, 15);
    lv_obj_align(led, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_radius(led, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(led, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(led, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(led, lv_color_hex(0xDDE7EA), 0);
    lv_obj_set_style_border_width(led, 1, 0);
    lv_obj_clear_flag(led, LV_OBJ_FLAG_SCROLLABLE);
    return led;
}


static void left_set_dual_led(lv_obj_t *red_led, lv_obj_t *green_led, int state)
{
    // state: -1 = unknown, 0 = red, 1 = green
    if (state < 0) {
        ui_led_set_color(red_led, 0x555555);
        ui_led_set_color(green_led, 0x555555);
    } else if (state == 0) {
        ui_led_set_color(red_led, 0xF25F4C);
        ui_led_set_color(green_led, 0x555555);
    } else {
        ui_led_set_color(red_led, 0x555555);
        ui_led_set_color(green_led, 0x2ECC71);
    }
}


static lv_obj_t *track_make_line(lv_obj_t *parent, lv_point_t *pts, uint16_t count, uint32_t color, uint8_t width)
{
    lv_obj_t *line = lv_line_create(parent);
    lv_line_set_points(line, pts, count);
    lv_obj_set_style_line_color(line, lv_color_hex(color), 0);
    lv_obj_set_style_line_width(line, width, 0);
    lv_obj_set_style_line_rounded(line, false, 0);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    return line;
}

typedef struct {
    int16_t x;
    int16_t y;
} track_src_point_t;

static void track_scale_points(lv_point_t *dst, const track_src_point_t *src, uint16_t count, int32_t ox, int32_t oy, int32_t scale_q10)
{
    for (uint16_t i = 0; i < count; ++i) {
        dst[i].x = (lv_coord_t)(ox + ((int32_t)src[i].x * scale_q10) / 1024);
        dst[i].y = (lv_coord_t)(oy + ((int32_t)src[i].y * scale_q10) / 1024);
    }
}

static void track_make_joint_fills(lv_obj_t *parent, const lv_point_t *pts, uint16_t count, uint32_t color, uint8_t width)
{
    // LVGLs lv_line does not render SVG-style miter joins. With thick polylines,
    // sharp corners can show small cut-outs. We close only the internal vertices
    // with small square fills. Unlike round caps, these do not create visible
    // "Kugel" artifacts at the connection points or line ends.
    if (count < 3) return;

    int fill = (int)width - 1;
    if (fill < 4) fill = 4;

    for (uint16_t i = 1; i + 1 < count; ++i) {
        lv_obj_t *joint = lv_obj_create(parent);
        lv_obj_set_size(joint, fill, fill);
        lv_obj_set_pos(joint, pts[i].x - fill / 2, pts[i].y - fill / 2);
        lv_obj_set_style_radius(joint, 0, 0);
        lv_obj_set_style_bg_color(joint, lv_color_hex(color), 0);
        lv_obj_set_style_bg_opa(joint, LV_OPA_COVER, 0);
        lv_obj_set_style_border_width(joint, 0, 0);
        lv_obj_clear_flag(joint, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_clear_flag(joint, LV_OBJ_FLAG_SCROLLABLE);
    }
}

static lv_obj_t *track_make_scaled_line(lv_obj_t *parent, lv_point_t *dst, const track_src_point_t *src,
                                        uint16_t count, int32_t ox, int32_t oy, int32_t scale_q10,
                                        uint32_t color, uint8_t width)
{
    track_scale_points(dst, src, count, ox, oy, scale_q10);
    lv_obj_t *line = track_make_line(parent, dst, count, color, width);
    track_make_joint_fills(parent, dst, count, color, width);
    return line;
}

static void track_make_scaled_arrow_line(lv_obj_t *parent,
                                         lv_point_t *shaft_dst, const track_src_point_t *shaft_src,
                                         lv_point_t *head_dst, const track_src_point_t *head_src,
                                         int32_t ox, int32_t oy, int32_t scale_q10,
                                         uint32_t color, uint8_t width)
{
    // Zielgleis-Pfeile: Linie in Fahrtrichtung plus V-Pfeilspitze am unteren linken Ende.
    // Verwendet bewusst LVGL-Primitives statt SVG-Rendering.
    track_make_scaled_line(parent, shaft_dst, shaft_src, 2, ox, oy, scale_q10, color, width);
    track_scale_points(head_dst, head_src, 3, ox, oy, scale_q10);

    lv_obj_t *head = lv_line_create(parent);
    lv_line_set_points(head, head_dst, 3);
    lv_obj_set_style_line_color(head, lv_color_hex(color), 0);
    lv_obj_set_style_line_opa(head, LV_OPA_COVER, 0);
    lv_obj_set_style_line_width(head, width, 0);
    lv_obj_set_style_line_rounded(head, false, 0);
    lv_obj_clear_flag(head, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(head, LV_OBJ_FLAG_SCROLLABLE);
}

static lv_obj_t *track_make_scaled_warn_triangle(lv_obj_t *parent, lv_point_t *dst, const track_src_point_t *src,
                                                 int32_t ox, int32_t oy, int32_t scale_q10,
                                                 uint32_t color, uint8_t width)
{
    // Warnzeichen werden wieder aus den gezeichneten HMI_weiche_wX_warn.svg-
    // Koordinaten gerendert. Die Sichtbarkeit bleibt robust ueber warningMask
    // plus Fallback auf die Defekttexte gesteuert.
    track_scale_points(dst, src, 4, ox, oy, scale_q10);

    lv_obj_t *line = lv_line_create(parent);
    lv_line_set_points(line, dst, 4);
    lv_obj_set_style_line_color(line, lv_color_hex(color), 0);
    lv_obj_set_style_line_opa(line, LV_OPA_COVER, 0);
    lv_obj_set_style_line_width(line, width, 0);
    // Nur Warnsymbole bekommen runde Linienenden/Ecken. Die Gleislinien bleiben
    // bewusst eckig, damit dort keine Kugel-Artefakte entstehen.
    lv_obj_set_style_line_rounded(line, true, 0);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(line, LV_OBJ_FLAG_SCROLLABLE);
    return line;
}

static lv_obj_t *track_make_overlay_layer(lv_obj_t *parent, int w, int h)
{
    lv_obj_t *layer = lv_obj_create(parent);
    lv_obj_set_size(layer, w, h);
    lv_obj_set_pos(layer, 0, 0);
    lv_obj_set_style_bg_opa(layer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(layer, 0, 0);
    lv_obj_set_style_pad_all(layer, 0, 0);
    lv_obj_clear_flag(layer, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(layer, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(layer, LV_OBJ_FLAG_HIDDEN);
    return layer;
}

static void track_set_layer_color(lv_obj_t *layer, uint32_t color)
{
    if (!layer) return;

    const lv_color_t c = lv_color_hex(color);
    uint32_t child_count = lv_obj_get_child_cnt(layer);
    for (uint32_t i = 0; i < child_count; ++i) {
        lv_obj_t *child = lv_obj_get_child(layer, i);
        if (!child) continue;
        lv_obj_set_style_line_color(child, c, 0);
        lv_obj_set_style_bg_color(child, c, 0);
    }
}

static lv_obj_t *track_make_signal_dot(lv_obj_t *parent, int cx, int cy, int diameter, uint32_t color)
{
    lv_obj_t *dot = lv_obj_create(parent);
    lv_obj_set_size(dot, diameter, diameter);
    lv_obj_align(dot, LV_ALIGN_TOP_LEFT, cx - diameter / 2, cy - diameter / 2);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(dot, lv_color_hex(0x111111), 0);
    lv_obj_set_style_border_width(dot, 1, 0);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(dot, LV_OBJ_FLAG_SCROLLABLE);
    return dot;
}

static lv_obj_t *track_make_signal_box(lv_obj_t *parent, int x, int y, int w, int h)
{
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_size(box, w, h);
    lv_obj_align(box, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_radius(box, 4, 0);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_30, 0);
    lv_obj_set_style_border_color(box, lv_color_white(), 0);
    lv_obj_set_style_border_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(box, 2, 0);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    return box;
}

static void track_set_grant_signal(lv_obj_t *red_led, lv_obj_t *green_led, int state)
{
    // state: -1 = unbekannt/offline, 0 = nicht freigegeben, 1 = freigegeben
    if (state < 0) {
        ui_led_set_color(red_led, 0x555555);
        ui_led_set_color(green_led, 0x555555);
    } else if (state == 0) {
        ui_led_set_color(red_led, 0xF25F4C);
        ui_led_set_color(green_led, 0x2A3A2F);
    } else {
        ui_led_set_color(red_led, 0x3A2A2A);
        ui_led_set_color(green_led, 0x2ECC71);
    }
}

static void track_hitbox_flash_hide_cb(lv_timer_t *timer)
{
    (void)timer;
    if (track_hitbox_flash_obj) {
        lv_obj_set_style_bg_opa(track_hitbox_flash_obj, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(track_hitbox_flash_obj, 0, 0);
        track_hitbox_flash_obj = NULL;
    }
    if (track_hitbox_flash_timer) {
        lv_timer_pause(track_hitbox_flash_timer);
    }
}

static void track_hitbox_flash_pressed_cb(lv_event_t *e)
{
    lv_obj_t *box = lv_event_get_target(e);
    if (!box) return;

    if (track_hitbox_flash_obj && track_hitbox_flash_obj != box) {
        lv_obj_set_style_bg_opa(track_hitbox_flash_obj, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(track_hitbox_flash_obj, 0, 0);
    }

    track_hitbox_flash_obj = box;
    lv_obj_set_style_radius(box, 8, 0);
    lv_obj_set_style_bg_color(box, lv_color_hex(0x00BCE3), 0);
    lv_obj_set_style_bg_opa(box, LV_OPA_30, 0);
    lv_obj_set_style_border_color(box, lv_color_white(), 0);
    lv_obj_set_style_border_opa(box, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(box, 2, 0);

    if (!track_hitbox_flash_timer) {
        track_hitbox_flash_timer = lv_timer_create(track_hitbox_flash_hide_cb, 220, NULL);
    }
    lv_timer_set_period(track_hitbox_flash_timer, 220);
    lv_timer_reset(track_hitbox_flash_timer);
    lv_timer_resume(track_hitbox_flash_timer);
}

static lv_obj_t *track_make_hitbox(lv_obj_t *parent, int x, int y, int w, int h, lv_event_cb_t cb, void *user_data)
{
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_set_size(box, w, h);
    lv_obj_align(box, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_bg_opa(box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(box, 0, 0);
    lv_obj_set_style_outline_width(box, 0, 0);
    lv_obj_set_style_pad_all(box, 0, 0);
    lv_obj_clear_flag(box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(box, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(box, track_hitbox_flash_pressed_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(box, cb, LV_EVENT_CLICKED, user_data);
    return box;
}

static void track_switch_level(track_level_t level);
static void create_track_layer_sbhf(void);
static void on_track_level_button_clicked(lv_event_t *e);

static lv_obj_t *track_make_layer_button(lv_obj_t *parent, const char *text, int x, int y, int w, bool active)
{
    lv_obj_t *btn = lv_btn_create(parent);
    lv_obj_set_size(btn, w, 32);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_radius(btn, 8, 0);
    lv_obj_set_style_bg_color(btn, lv_color_hex(active ? 0x00BCE3 : 0x53616A), 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(active ? 0xBEEFFF : 0x7E8B92), 0);
    lv_obj_set_style_border_width(btn, active ? 2 : 1, 0);
    lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *label = lv_label_create(btn);
    lv_label_set_text(label, text);
    ui_label_style(label, &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_center(label);
    return btn;
}

static void track_set_layer_button_active(lv_obj_t *btn, bool active)
{
    if (!btn) return;
    lv_obj_set_style_bg_color(btn, lv_color_hex(active ? 0x00BCE3 : 0x53616A), 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(active ? 0xBEEFFF : 0x7E8B92), 0);
    lv_obj_set_style_border_width(btn, active ? 2 : 1, 0);
}

static void create_track_layer_e0(void)
{
    if (track_layer_e0_created || !track_root_panel) return;
    track_layer_e0 = track_make_overlay_layer(track_root_panel, track_panel_w, track_panel_h);
    track_layer_e0_created = true;

    lv_obj_t *panel = track_layer_e0;
    const int panel_w = track_panel_w;
    const int panel_h = track_panel_h;
    const int32_t ox = track_ox;
    const int32_t oy = track_oy;
    const int32_t scale_q10 = track_scale_q10;
    const uint8_t line_w = track_line_w;
    const uint8_t occ_line_w = track_occ_line_w;
    const uint32_t rail = 0xFFFFFF;
    const uint32_t occ_red = 0xF25F4C;
    const uint32_t turnout_ok_green = 0x2ECC71;
    const uint32_t warn_red = 0xFF0000;

    // Ebene 0: Basisgleis aus HMI_ebene0_base.svg, Referenz 1280x720.
    static const track_src_point_t e0_base_0[] = { {270, 83}, {130, 223}, {130, 501}, {267, 638}, {1012, 638}, {1152, 499}, {1152, 222}, {1012, 82}, {270, 83} };
    static lv_point_t p_e0_base_0[sizeof(e0_base_0) / sizeof(e0_base_0[0])];
    track_make_scaled_line(panel, p_e0_base_0, e0_base_0, (uint16_t)(sizeof(e0_base_0) / sizeof(e0_base_0[0])), ox, oy, scale_q10, rail, (uint8_t)line_w);
    static const track_src_point_t e0_base_1[] = { {792, 83}, {689, 186}, {689, 285}, {486, 83} };
    static lv_point_t p_e0_base_1[sizeof(e0_base_1) / sizeof(e0_base_1[0])];
    track_make_scaled_line(panel, p_e0_base_1, e0_base_1, (uint16_t)(sizeof(e0_base_1) / sizeof(e0_base_1[0])), ox, oy, scale_q10, rail, (uint8_t)line_w);
    static const track_src_point_t e0_base_2[] = { {340, 638}, {689, 289}, {689, 285} };
    static lv_point_t p_e0_base_2[sizeof(e0_base_2) / sizeof(e0_base_2[0])];
    track_make_scaled_line(panel, p_e0_base_2, e0_base_2, (uint16_t)(sizeof(e0_base_2) / sizeof(e0_base_2[0])), ox, oy, scale_q10, rail, (uint8_t)line_w);
    static const track_src_point_t e0_base_3[] = { {942, 638}, {750, 447} };
    static lv_point_t p_e0_base_3[sizeof(e0_base_3) / sizeof(e0_base_3[0])];
    track_make_scaled_line(panel, p_e0_base_3, e0_base_3, (uint16_t)(sizeof(e0_base_3) / sizeof(e0_base_3[0])), ox, oy, scale_q10, rail, (uint8_t)line_w);
    static const track_src_point_t e0_base_4[] = { {341, 352}, {437, 448}, {854, 448}, {936, 366}, {936, 297} };
    static lv_point_t p_e0_base_4[sizeof(e0_base_4) / sizeof(e0_base_4[0])];
    track_make_scaled_line(panel, p_e0_base_4, e0_base_4, (uint16_t)(sizeof(e0_base_4) / sizeof(e0_base_4[0])), ox, oy, scale_q10, rail, (uint8_t)line_w);
    static const track_src_point_t e0_base_5[] = { {1055, 303}, {1055, 426}, {939, 542}, {342, 542}, {246, 445} };
    static lv_point_t p_e0_base_5[sizeof(e0_base_5) / sizeof(e0_base_5[0])];
    track_make_scaled_line(panel, p_e0_base_5, e0_base_5, (uint16_t)(sizeof(e0_base_5) / sizeof(e0_base_5[0])), ox, oy, scale_q10, rail, (uint8_t)line_w);

    // Blockbesetzt-Overlays Ebene 0: B1, B3, B4, B5, B6.
    for (uint8_t i = 0; i < 5u; ++i) {
        track_e0_block_occ_layer[i] = track_make_overlay_layer(panel, panel_w, panel_h);
    }
    static const track_src_point_t e0_occ_b1_0[] = { {247, 447}, {331, 531} };
    static lv_point_t p_e0_occ_b1_0[sizeof(e0_occ_b1_0) / sizeof(e0_occ_b1_0[0])];
    track_make_scaled_line(track_e0_block_occ_layer[0], p_e0_occ_b1_0, e0_occ_b1_0, (uint16_t)(sizeof(e0_occ_b1_0) / sizeof(e0_occ_b1_0[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b3_0[] = { {341, 352}, {424, 436} };
    static lv_point_t p_e0_occ_b3_0[sizeof(e0_occ_b3_0) / sizeof(e0_occ_b3_0[0])];
    track_make_scaled_line(track_e0_block_occ_layer[1], p_e0_occ_b3_0, e0_occ_b3_0, (uint16_t)(sizeof(e0_occ_b3_0) / sizeof(e0_occ_b3_0[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_0[] = { {291, 640}, {267, 640}, {130, 502}, {130, 225}, {272, 84}, {436, 84} };
    static lv_point_t p_e0_occ_b4_0[sizeof(e0_occ_b4_0) / sizeof(e0_occ_b4_0[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_0, e0_occ_b4_0, (uint16_t)(sizeof(e0_occ_b4_0) / sizeof(e0_occ_b4_0[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_1[] = { {552, 83}, {728, 83} };
    static lv_point_t p_e0_occ_b4_1[sizeof(e0_occ_b4_1) / sizeof(e0_occ_b4_1[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_1, e0_occ_b4_1, (uint16_t)(sizeof(e0_occ_b4_1) / sizeof(e0_occ_b4_1[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_2[] = { {839, 82}, {1012, 82}, {1151, 221}, {1151, 499}, {1012, 638}, {997, 638}, {997, 639} };
    static lv_point_t p_e0_occ_b4_2[sizeof(e0_occ_b4_2) / sizeof(e0_occ_b4_2[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_2, e0_occ_b4_2, (uint16_t)(sizeof(e0_occ_b4_2) / sizeof(e0_occ_b4_2[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_3[] = { {900, 596}, {856, 553}, {856, 553} };
    static lv_point_t p_e0_occ_b4_3[sizeof(e0_occ_b4_3) / sizeof(e0_occ_b4_3[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_3, e0_occ_b4_3, (uint16_t)(sizeof(e0_occ_b4_3) / sizeof(e0_occ_b4_3[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_4[] = { {745, 130}, {689, 185}, {689, 226} };
    static lv_point_t p_e0_occ_b4_4[sizeof(e0_occ_b4_4) / sizeof(e0_occ_b4_4[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_4, e0_occ_b4_4, (uint16_t)(sizeof(e0_occ_b4_4) / sizeof(e0_occ_b4_4[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_5[] = { {644, 239}, {531, 127} };
    static lv_point_t p_e0_occ_b4_5[sizeof(e0_occ_b4_5) / sizeof(e0_occ_b4_5[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_5, e0_occ_b4_5, (uint16_t)(sizeof(e0_occ_b4_5) / sizeof(e0_occ_b4_5[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_6[] = { {654, 325}, {574, 404} };
    static lv_point_t p_e0_occ_b4_6[sizeof(e0_occ_b4_6) / sizeof(e0_occ_b4_6[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_6, e0_occ_b4_6, (uint16_t)(sizeof(e0_occ_b4_6) / sizeof(e0_occ_b4_6[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_7[] = { {520, 459}, {479, 500} };
    static lv_point_t p_e0_occ_b4_7[sizeof(e0_occ_b4_7) / sizeof(e0_occ_b4_7[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_7, e0_occ_b4_7, (uint16_t)(sizeof(e0_occ_b4_7) / sizeof(e0_occ_b4_7[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_8[] = { {424, 555}, {383, 596} };
    static lv_point_t p_e0_occ_b4_8[sizeof(e0_occ_b4_8) / sizeof(e0_occ_b4_8[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_8, e0_occ_b4_8, (uint16_t)(sizeof(e0_occ_b4_8) / sizeof(e0_occ_b4_8[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_9[] = { {592, 449}, {702, 449} };
    static lv_point_t p_e0_occ_b4_9[sizeof(e0_occ_b4_9) / sizeof(e0_occ_b4_9[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_9, e0_occ_b4_9, (uint16_t)(sizeof(e0_occ_b4_9) / sizeof(e0_occ_b4_9[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_10[] = { {500, 542}, {785, 542} };
    static lv_point_t p_e0_occ_b4_10[sizeof(e0_occ_b4_10) / sizeof(e0_occ_b4_10[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_10, e0_occ_b4_10, (uint16_t)(sizeof(e0_occ_b4_10) / sizeof(e0_occ_b4_10[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b4_11[] = { {881, 640}, {401, 640} };
    static lv_point_t p_e0_occ_b4_11[sizeof(e0_occ_b4_11) / sizeof(e0_occ_b4_11[0])];
    track_make_scaled_line(track_e0_block_occ_layer[2], p_e0_occ_b4_11, e0_occ_b4_11, (uint16_t)(sizeof(e0_occ_b4_11) / sizeof(e0_occ_b4_11[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b5_0[] = { {870, 434}, {936, 368}, {936, 298} };
    static lv_point_t p_e0_occ_b5_0[sizeof(e0_occ_b5_0) / sizeof(e0_occ_b5_0[0])];
    track_make_scaled_line(track_e0_block_occ_layer[3], p_e0_occ_b5_0, e0_occ_b5_0, (uint16_t)(sizeof(e0_occ_b5_0) / sizeof(e0_occ_b5_0[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    static const track_src_point_t e0_occ_b6_0[] = { {955, 528}, {1056, 427}, {1056, 304} };
    static lv_point_t p_e0_occ_b6_0[sizeof(e0_occ_b6_0) / sizeof(e0_occ_b6_0[0])];
    track_make_scaled_line(track_e0_block_occ_layer[4], p_e0_occ_b6_0, e0_occ_b6_0, (uint16_t)(sizeof(e0_occ_b6_0) / sizeof(e0_occ_b6_0[0])), ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);

    // Weichen-Overlays Ebene 0: Mega1 W0..W8.
    int turnout_line_w = occ_line_w;
    if (turnout_line_w < 3) turnout_line_w = 3;
    for (uint8_t i = 0; i < 9u; ++i) {
        track_e0_turnout_layer[i][0] = track_make_overlay_layer(panel, panel_w, panel_h);
        track_e0_turnout_layer[i][1] = track_make_overlay_layer(panel, panel_w, panel_h);
        track_e0_turnout_warn_layer[i] = track_make_overlay_layer(panel, panel_w, panel_h);
    }
    // W0 abbiegen/gerade aus HMI_weiche_w0_a.svg / HMI_weiche_w0_g.svg
    static const track_src_point_t e0_w0_a_0[] = { {295, 639}, {331, 639} };
    static lv_point_t p_e0_w0_a_0[sizeof(e0_w0_a_0) / sizeof(e0_w0_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[0][0], p_e0_w0_a_0, e0_w0_a_0, (uint16_t)(sizeof(e0_w0_a_0) / sizeof(e0_w0_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w0_a_1[] = { {352, 625}, {378, 599} };
    static lv_point_t p_e0_w0_a_1[sizeof(e0_w0_a_1) / sizeof(e0_w0_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[0][0], p_e0_w0_a_1, e0_w0_a_1, (uint16_t)(sizeof(e0_w0_a_1) / sizeof(e0_w0_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w0_g_0[] = { {295, 638}, {331, 638} };
    static lv_point_t p_e0_w0_g_0[sizeof(e0_w0_g_0) / sizeof(e0_w0_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[0][1], p_e0_w0_g_0, e0_w0_g_0, (uint16_t)(sizeof(e0_w0_g_0) / sizeof(e0_w0_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w0_g_1[] = { {359, 638}, {396, 638} };
    static lv_point_t p_e0_w0_g_1[sizeof(e0_w0_g_1) / sizeof(e0_w0_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[0][1], p_e0_w0_g_1, e0_w0_g_1, (uint16_t)(sizeof(e0_w0_g_1) / sizeof(e0_w0_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    // W1 abbiegen/gerade aus HMI_weiche_w1_a.svg / HMI_weiche_w1_g.svg
    static const track_src_point_t e0_w1_a_0[] = { {988, 638}, {952, 638} };
    static lv_point_t p_e0_w1_a_0[sizeof(e0_w1_a_0) / sizeof(e0_w1_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[1][0], p_e0_w1_a_0, e0_w1_a_0, (uint16_t)(sizeof(e0_w1_a_0) / sizeof(e0_w1_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w1_a_1[] = { {929, 625}, {903, 599} };
    static lv_point_t p_e0_w1_a_1[sizeof(e0_w1_a_1) / sizeof(e0_w1_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[1][0], p_e0_w1_a_1, e0_w1_a_1, (uint16_t)(sizeof(e0_w1_a_1) / sizeof(e0_w1_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w1_g_0[] = { {923, 639}, {887, 639} };
    static lv_point_t p_e0_w1_g_0[sizeof(e0_w1_g_0) / sizeof(e0_w1_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[1][1], p_e0_w1_g_0, e0_w1_g_0, (uint16_t)(sizeof(e0_w1_g_0) / sizeof(e0_w1_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w1_g_1[] = { {988, 639}, {952, 639} };
    static lv_point_t p_e0_w1_g_1[sizeof(e0_w1_g_1) / sizeof(e0_w1_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[1][1], p_e0_w1_g_1, e0_w1_g_1, (uint16_t)(sizeof(e0_w1_g_1) / sizeof(e0_w1_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    // W2 abbiegen/gerade aus HMI_weiche_w2_a.svg / HMI_weiche_w2_g.svg
    static const track_src_point_t e0_w2_a_0[] = { {383, 541}, {420, 541} };
    static lv_point_t p_e0_w2_a_0[sizeof(e0_w2_a_0) / sizeof(e0_w2_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[2][0], p_e0_w2_a_0, e0_w2_a_0, (uint16_t)(sizeof(e0_w2_a_0) / sizeof(e0_w2_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w2_a_1[] = { {449, 528}, {475, 502} };
    static lv_point_t p_e0_w2_a_1[sizeof(e0_w2_a_1) / sizeof(e0_w2_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[2][0], p_e0_w2_a_1, e0_w2_a_1, (uint16_t)(sizeof(e0_w2_a_1) / sizeof(e0_w2_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w2_g_0[] = { {383, 541}, {420, 541} };
    static lv_point_t p_e0_w2_g_0[sizeof(e0_w2_g_0) / sizeof(e0_w2_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[2][1], p_e0_w2_g_0, e0_w2_g_0, (uint16_t)(sizeof(e0_w2_g_0) / sizeof(e0_w2_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w2_g_1[] = { {456, 541}, {492, 541} };
    static lv_point_t p_e0_w2_g_1[sizeof(e0_w2_g_1) / sizeof(e0_w2_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[2][1], p_e0_w2_g_1, e0_w2_g_1, (uint16_t)(sizeof(e0_w2_g_1) / sizeof(e0_w2_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    // W3 abbiegen/gerade aus HMI_weiche_w3_a.svg / HMI_weiche_w3_g.svg
    static const track_src_point_t e0_w3_a_0[] = { {900, 542}, {864, 542} };
    static lv_point_t p_e0_w3_a_0[sizeof(e0_w3_a_0) / sizeof(e0_w3_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[3][0], p_e0_w3_a_0, e0_w3_a_0, (uint16_t)(sizeof(e0_w3_a_0) / sizeof(e0_w3_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w3_a_1[] = { {834, 528}, {808, 502} };
    static lv_point_t p_e0_w3_a_1[sizeof(e0_w3_a_1) / sizeof(e0_w3_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[3][0], p_e0_w3_a_1, e0_w3_a_1, (uint16_t)(sizeof(e0_w3_a_1) / sizeof(e0_w3_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w3_g_0[] = { {900, 541}, {864, 541} };
    static lv_point_t p_e0_w3_g_0[sizeof(e0_w3_g_0) / sizeof(e0_w3_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[3][1], p_e0_w3_g_0, e0_w3_g_0, (uint16_t)(sizeof(e0_w3_g_0) / sizeof(e0_w3_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w3_g_1[] = { {828, 541}, {791, 541} };
    static lv_point_t p_e0_w3_g_1[sizeof(e0_w3_g_1) / sizeof(e0_w3_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[3][1], p_e0_w3_g_1, e0_w3_g_1, (uint16_t)(sizeof(e0_w3_g_1) / sizeof(e0_w3_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    // W4 abbiegen/gerade aus HMI_weiche_w4_a.svg / HMI_weiche_w4_g.svg
    static const track_src_point_t e0_w4_a_0[] = { {478, 448}, {514, 448} };
    static lv_point_t p_e0_w4_a_0[sizeof(e0_w4_a_0) / sizeof(e0_w4_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[4][0], p_e0_w4_a_0, e0_w4_a_0, (uint16_t)(sizeof(e0_w4_a_0) / sizeof(e0_w4_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w4_a_1[] = { {543, 434}, {569, 408} };
    static lv_point_t p_e0_w4_a_1[sizeof(e0_w4_a_1) / sizeof(e0_w4_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[4][0], p_e0_w4_a_1, e0_w4_a_1, (uint16_t)(sizeof(e0_w4_a_1) / sizeof(e0_w4_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w4_g_0[] = { {478, 447}, {514, 447} };
    static lv_point_t p_e0_w4_g_0[sizeof(e0_w4_g_0) / sizeof(e0_w4_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[4][1], p_e0_w4_g_0, e0_w4_g_0, (uint16_t)(sizeof(e0_w4_g_0) / sizeof(e0_w4_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w4_g_1[] = { {550, 448}, {587, 448} };
    static lv_point_t p_e0_w4_g_1[sizeof(e0_w4_g_1) / sizeof(e0_w4_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[4][1], p_e0_w4_g_1, e0_w4_g_1, (uint16_t)(sizeof(e0_w4_g_1) / sizeof(e0_w4_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    // W5 abbiegen/gerade aus HMI_weiche_w5_a.svg / HMI_weiche_w5_g.svg
    static const track_src_point_t e0_w5_a_0[] = { {792, 488}, {766, 462} };
    static lv_point_t p_e0_w5_a_0[sizeof(e0_w5_a_0) / sizeof(e0_w5_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[5][0], p_e0_w5_a_0, e0_w5_a_0, (uint16_t)(sizeof(e0_w5_a_0) / sizeof(e0_w5_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w5_a_1[] = { {744, 448}, {708, 448} };
    static lv_point_t p_e0_w5_a_1[sizeof(e0_w5_a_1) / sizeof(e0_w5_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[5][0], p_e0_w5_a_1, e0_w5_a_1, (uint16_t)(sizeof(e0_w5_a_1) / sizeof(e0_w5_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w5_g_0[] = { {810, 448}, {773, 448} };
    static lv_point_t p_e0_w5_g_0[sizeof(e0_w5_g_0) / sizeof(e0_w5_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[5][1], p_e0_w5_g_0, e0_w5_g_0, (uint16_t)(sizeof(e0_w5_g_0) / sizeof(e0_w5_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w5_g_1[] = { {744, 448}, {708, 448} };
    static lv_point_t p_e0_w5_g_1[sizeof(e0_w5_g_1) / sizeof(e0_w5_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[5][1], p_e0_w5_g_1, e0_w5_g_1, (uint16_t)(sizeof(e0_w5_g_1) / sizeof(e0_w5_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    // W6 abbiegen/gerade aus HMI_weiche_w6_a.svg / HMI_weiche_w6_g.svg
    static const track_src_point_t e0_w6_a_0[] = { {658, 320}, {684, 294} };
    static lv_point_t p_e0_w6_a_0[sizeof(e0_w6_a_0) / sizeof(e0_w6_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[6][0], p_e0_w6_a_0, e0_w6_a_0, (uint16_t)(sizeof(e0_w6_a_0) / sizeof(e0_w6_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w6_a_1[] = { {676, 272}, {650, 246} };
    static lv_point_t p_e0_w6_a_1[sizeof(e0_w6_a_1) / sizeof(e0_w6_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[6][0], p_e0_w6_a_1, e0_w6_a_1, (uint16_t)(sizeof(e0_w6_a_1) / sizeof(e0_w6_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w6_g_0[] = { {658, 320}, {684, 294} };
    static lv_point_t p_e0_w6_g_0[sizeof(e0_w6_g_0) / sizeof(e0_w6_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[6][1], p_e0_w6_g_0, e0_w6_g_0, (uint16_t)(sizeof(e0_w6_g_0) / sizeof(e0_w6_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w6_g_1[] = { {689, 268}, {689, 231} };
    static lv_point_t p_e0_w6_g_1[sizeof(e0_w6_g_1) / sizeof(e0_w6_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[6][1], p_e0_w6_g_1, e0_w6_g_1, (uint16_t)(sizeof(e0_w6_g_1) / sizeof(e0_w6_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    // W7 abbiegen/gerade aus HMI_weiche_w7_a.svg / HMI_weiche_w7_g.svg
    static const track_src_point_t e0_w7_a_0[] = { {526, 122}, {500, 96} };
    static lv_point_t p_e0_w7_a_0[sizeof(e0_w7_a_0) / sizeof(e0_w7_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[7][0], p_e0_w7_a_0, e0_w7_a_0, (uint16_t)(sizeof(e0_w7_a_0) / sizeof(e0_w7_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w7_a_1[] = { {481, 82}, {444, 82} };
    static lv_point_t p_e0_w7_a_1[sizeof(e0_w7_a_1) / sizeof(e0_w7_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[7][0], p_e0_w7_a_1, e0_w7_a_1, (uint16_t)(sizeof(e0_w7_a_1) / sizeof(e0_w7_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w7_g_0[] = { {543, 83}, {507, 83} };
    static lv_point_t p_e0_w7_g_0[sizeof(e0_w7_g_0) / sizeof(e0_w7_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[7][1], p_e0_w7_g_0, e0_w7_g_0, (uint16_t)(sizeof(e0_w7_g_0) / sizeof(e0_w7_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w7_g_1[] = { {481, 83}, {444, 83} };
    static lv_point_t p_e0_w7_g_1[sizeof(e0_w7_g_1) / sizeof(e0_w7_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[7][1], p_e0_w7_g_1, e0_w7_g_1, (uint16_t)(sizeof(e0_w7_g_1) / sizeof(e0_w7_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    // W8 abbiegen/gerade aus HMI_weiche_w8_a.svg / HMI_weiche_w8_g.svg
    static const track_src_point_t e0_w8_a_0[] = { {833, 82}, {796, 82} };
    static lv_point_t p_e0_w8_a_0[sizeof(e0_w8_a_0) / sizeof(e0_w8_a_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[8][0], p_e0_w8_a_0, e0_w8_a_0, (uint16_t)(sizeof(e0_w8_a_0) / sizeof(e0_w8_a_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w8_a_1[] = { {778, 96}, {751, 122} };
    static lv_point_t p_e0_w8_a_1[sizeof(e0_w8_a_1) / sizeof(e0_w8_a_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[8][0], p_e0_w8_a_1, e0_w8_a_1, (uint16_t)(sizeof(e0_w8_a_1) / sizeof(e0_w8_a_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w8_g_0[] = { {833, 82}, {796, 82} };
    static lv_point_t p_e0_w8_g_0[sizeof(e0_w8_g_0) / sizeof(e0_w8_g_0[0])];
    track_make_scaled_line(track_e0_turnout_layer[8][1], p_e0_w8_g_0, e0_w8_g_0, (uint16_t)(sizeof(e0_w8_g_0) / sizeof(e0_w8_g_0[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    static const track_src_point_t e0_w8_g_1[] = { {772, 82}, {735, 82} };
    static lv_point_t p_e0_w8_g_1[sizeof(e0_w8_g_1) / sizeof(e0_w8_g_1[0])];
    track_make_scaled_line(track_e0_turnout_layer[8][1], p_e0_w8_g_1, e0_w8_g_1, (uint16_t)(sizeof(e0_w8_g_1) / sizeof(e0_w8_g_1[0])), ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);

    // Warnsymbole W0..W8 aus den HMI_weiche_wX_warn.svg-Dateien.
    int warn_line_w = (int)((10 * scale_q10) / 1024);
    if (warn_line_w < 4) warn_line_w = 4;
    if (warn_line_w > 9) warn_line_w = 9;
    static const track_src_point_t e0_w0_warn[] = { {315, 589}, {350, 589}, {332, 619}, {315, 589} };
    static lv_point_t p_e0_w0_warn[sizeof(e0_w0_warn) / sizeof(e0_w0_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[0], p_e0_w0_warn, e0_w0_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    static const track_src_point_t e0_w1_warn[] = { {942, 588}, {976, 588}, {959, 617}, {942, 588} };
    static lv_point_t p_e0_w1_warn[sizeof(e0_w1_warn) / sizeof(e0_w1_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[1], p_e0_w1_warn, e0_w1_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    static const track_src_point_t e0_w2_warn[] = { {441, 564}, {478, 564}, {460, 593}, {441, 564} };
    static lv_point_t p_e0_w2_warn[sizeof(e0_w2_warn) / sizeof(e0_w2_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[2], p_e0_w2_warn, e0_w2_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    static const track_src_point_t e0_w3_warn[] = { {842, 496}, {876, 496}, {859, 526}, {842, 496} };
    static lv_point_t p_e0_w3_warn[sizeof(e0_w3_warn) / sizeof(e0_w3_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[3], p_e0_w3_warn, e0_w3_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    static const track_src_point_t e0_w4_warn[] = { {502, 399}, {537, 399}, {520, 428}, {502, 399} };
    static lv_point_t p_e0_w4_warn[sizeof(e0_w4_warn) / sizeof(e0_w4_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[4], p_e0_w4_warn, e0_w4_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    static const track_src_point_t e0_w5_warn[] = { {736, 400}, {773, 400}, {755, 429}, {736, 400} };
    static lv_point_t p_e0_w5_warn[sizeof(e0_w5_warn) / sizeof(e0_w5_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[5], p_e0_w5_warn, e0_w5_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    static const track_src_point_t e0_w6_warn[] = { {710, 266}, {745, 266}, {728, 295}, {710, 266} };
    static lv_point_t p_e0_w6_warn[sizeof(e0_w6_warn) / sizeof(e0_w6_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[6], p_e0_w6_warn, e0_w6_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    static const track_src_point_t e0_w7_warn[] = { {444, 109}, {481, 109}, {463, 138}, {444, 109} };
    static lv_point_t p_e0_w7_warn[sizeof(e0_w7_warn) / sizeof(e0_w7_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[7], p_e0_w7_warn, e0_w7_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    static const track_src_point_t e0_w8_warn[] = { {800, 109}, {837, 109}, {819, 138}, {800, 109} };
    static lv_point_t p_e0_w8_warn[sizeof(e0_w8_warn) / sizeof(e0_w8_warn[0])];
    track_make_scaled_warn_triangle(track_e0_turnout_warn_layer[8], p_e0_w8_warn, e0_w8_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);

    // Signale Ebene 0:
    // 0=3->4, 1=6->4, 2=4->1, 3=4->5, 4=Bhf0, 5=Bhf1.
    int sig_d = (int)((15 * scale_q10) / 1024);
    if (sig_d < 8) sig_d = 8;
    if (sig_d > 15) sig_d = 15;
    const int sig_pad = 4;
    typedef struct {
        uint16_t x;
        uint16_t top_y;
        uint16_t bottom_y;
    } track_signal_pos_t;
    static const track_signal_pos_t e0_signal_pos[6] = {
        {416, 366, 383}, // Grant 3 -> 4
        {965, 456, 473}, // Grant 6 -> 4
        {318, 456, 473}, // Grant 4 -> 1
        {872, 366, 384}, // Grant 4 -> 5
        {844, 595, 612}, // Bhf0
        {407, 497, 514}, // Bhf1
    };
    for (uint8_t i = 0; i < 6u; ++i) {
        const int sig_x = ox + (e0_signal_pos[i].x * scale_q10) / 1024;
        const int sig_top_y = oy + (e0_signal_pos[i].top_y * scale_q10) / 1024;
        const int sig_bottom_y = oy + (e0_signal_pos[i].bottom_y * scale_q10) / 1024;
        const int sig_box_x = sig_x - sig_d / 2 - sig_pad;
        const int sig_box_y = sig_top_y - sig_d / 2 - sig_pad;
        const int sig_box_w = sig_d + 2 * sig_pad;
        const int sig_box_h = (sig_bottom_y - sig_top_y) + sig_d + 2 * sig_pad;
        (void)track_make_signal_box(panel, sig_box_x, sig_box_y, sig_box_w, sig_box_h);
        track_e0_signal_red[i] = track_make_signal_dot(panel, sig_x, sig_top_y, sig_d, 0x555555);
        track_e0_signal_green[i] = track_make_signal_dot(panel, sig_x, sig_bottom_y, sig_d, 0x555555);
    }

    // Grosszuegige transparente Touch-Hitboxen fuer Ebene 0.
    typedef struct {
        uint16_t x;
        uint16_t y;
        uint16_t w;
        uint16_t h;
        uint8_t idx;
    } track_hitbox_pos_t;

    static const track_hitbox_pos_t turnout_hitboxes[9] = {
        {270, 570, 150, 100, 0}, // W0
        {860, 570, 150, 100, 1}, // W1
        {405, 485, 145, 115, 2}, // W2 (Hitbox bewusst etwas nach rechts versetzt)
        {770, 485, 155, 115, 3}, // W3
        {470, 385, 145, 100, 4}, // W4
        {690, 385, 145, 100, 5}, // W5
        {625, 220, 145, 115, 6}, // W6
        {420,  60, 150, 100, 7}, // W7
        {720,  60, 150, 100, 8}, // W8
    };
    static const track_hitbox_pos_t station_hitboxes[2] = {
        {800, 560,  90,  90, 0}, // Bhf0
        {335, 470,  85,  90, 1}, // Bhf1 (Hitbox etwas nach links, weiter auf dem Signal)
    };

    for (uint8_t i = 0; i < 9u; ++i) {
        const int hx = ox + (turnout_hitboxes[i].x * scale_q10) / 1024;
        const int hy = oy + (turnout_hitboxes[i].y * scale_q10) / 1024;
        const int hw = (turnout_hitboxes[i].w * scale_q10) / 1024;
        const int hh = (turnout_hitboxes[i].h * scale_q10) / 1024;
        track_e0_hitbox_turnout[i] = track_make_hitbox(panel, hx, hy, hw, hh,
                                                       on_left_m1_turnout_clicked,
                                                       (void *)(uintptr_t)turnout_hitboxes[i].idx);
    }
    for (uint8_t i = 0; i < 2u; ++i) {
        const int hx = ox + (station_hitboxes[i].x * scale_q10) / 1024;
        const int hy = oy + (station_hitboxes[i].y * scale_q10) / 1024;
        const int hw = (station_hitboxes[i].w * scale_q10) / 1024;
        const int hh = (station_hitboxes[i].h * scale_q10) / 1024;
        track_e0_hitbox_station[i] = track_make_hitbox(panel, hx, hy, hw, hh,
                                                       on_left_station_clicked,
                                                       (void *)(uintptr_t)station_hitboxes[i].idx);
    }
}

static void create_track_layer_sbhf(void)
{
    if (track_layer_sbhf_created || !track_root_panel) return;
    track_layer_sbhf = track_make_overlay_layer(track_root_panel, track_panel_w, track_panel_h);
    track_layer_sbhf_created = true;

    const int32_t ox = track_ox;
    const int32_t oy = track_oy;
    const int32_t scale_q10 = track_scale_q10;
    const uint8_t line_w = track_line_w;
    const uint8_t occ_line_w = track_occ_line_w;
    const uint32_t rail = 0xFFFFFF;
    const uint32_t occ_red = 0xF25F4C;
    const uint32_t turnout_ok_green = 0x2ECC71;
    const uint32_t target_green = 0x2ECC71;
    const uint32_t warn_red = 0xFF0000;

    // Ebene SBHF: Track-Layout aus HMI_ebene-1_base.svg, Referenz 1280x720.
    static const track_src_point_t sbhf_outer[] = {
        {878,638}, {1012,638}, {1152,498}, {1152,222}, {1013,83}, {269,83},
        {130,222}, {130,502}, {267,639}, {517,639}, {962,194}, {1080,311},
        {1080,426}, {965,541}, {878,541}
    };
    static const track_src_point_t sbhf_gleis1[] = {
        {962,194}, {718,194}, {272,639}
    };
    static const track_src_point_t sbhf_gleis2[] = {
        {838,194}, {393,639}
    };
    static lv_point_t p_sbhf_outer[sizeof(sbhf_outer) / sizeof(sbhf_outer[0])];
    static lv_point_t p_sbhf_gleis1[sizeof(sbhf_gleis1) / sizeof(sbhf_gleis1[0])];
    static lv_point_t p_sbhf_gleis2[sizeof(sbhf_gleis2) / sizeof(sbhf_gleis2[0])];
    track_make_scaled_line(track_layer_sbhf, p_sbhf_outer, sbhf_outer,
                           (uint16_t)(sizeof(sbhf_outer) / sizeof(sbhf_outer[0])), ox, oy, scale_q10, rail, line_w);
    track_make_scaled_line(track_layer_sbhf, p_sbhf_gleis1, sbhf_gleis1,
                           (uint16_t)(sizeof(sbhf_gleis1) / sizeof(sbhf_gleis1[0])), ox, oy, scale_q10, rail, line_w);
    track_make_scaled_line(track_layer_sbhf, p_sbhf_gleis2, sbhf_gleis2,
                           (uint16_t)(sizeof(sbhf_gleis2) / sizeof(sbhf_gleis2[0])), ox, oy, scale_q10, rail, line_w);

    for (uint8_t i = 0; i < 6; ++i) {
        track_sbhf_block_occ_layer[i] = track_make_overlay_layer(track_layer_sbhf, track_panel_w, track_panel_h);
    }

    static const track_src_point_t b4_0[] = { {879,639}, {1013,639}, {1101,550} };
    static const track_src_point_t b4_1[] = { {879,542}, {965,542}, {1024,482} };
    static const track_src_point_t b5[] = { {1038,469}, {1080,427}, {1080,309}, {1004,233} };
    static const track_src_point_t b6[] = { {1114,537}, {1152,499}, {1152,221}, {1014,83}, {271,83}, {131,224}, {131,501}, {231,602} };
    static const track_src_point_t sb1[] = { {457,640}, {517,640}, {770,386} };
    static const track_src_point_t sb2[] = { {645,389}, {440,594} };
    static const track_src_point_t sb3[] = { {527,386}, {316,596} };
    static lv_point_t p_b4_0[3], p_b4_1[3], p_b5[4], p_b6[8], p_sb1[3], p_sb2[2], p_sb3[2];
    track_make_scaled_line(track_sbhf_block_occ_layer[0], p_b4_0, b4_0, 3, ox, oy, scale_q10, occ_red, occ_line_w);
    track_make_scaled_line(track_sbhf_block_occ_layer[0], p_b4_1, b4_1, 3, ox, oy, scale_q10, occ_red, occ_line_w);
    track_make_scaled_line(track_sbhf_block_occ_layer[1], p_b5, b5, 4, ox, oy, scale_q10, occ_red, occ_line_w);
    track_make_scaled_line(track_sbhf_block_occ_layer[2], p_b6, b6, 8, ox, oy, scale_q10, occ_red, occ_line_w);
    track_make_scaled_line(track_sbhf_block_occ_layer[3], p_sb1, sb1, 3, ox, oy, scale_q10, occ_red, occ_line_w);
    track_make_scaled_line(track_sbhf_block_occ_layer[4], p_sb2, sb2, 2, ox, oy, scale_q10, occ_red, occ_line_w);
    track_make_scaled_line(track_sbhf_block_occ_layer[5], p_sb3, sb3, 2, ox, oy, scale_q10, occ_red, occ_line_w);

    for (uint8_t i = 0; i < 4; ++i) {
        track_sbhf_turnout_layer[i][0] = track_make_overlay_layer(track_layer_sbhf, track_panel_w, track_panel_h);
        track_sbhf_turnout_layer[i][1] = track_make_overlay_layer(track_layer_sbhf, track_panel_w, track_panel_h);
        track_sbhf_turnout_warn_layer[i] = track_make_overlay_layer(track_layer_sbhf, track_panel_w, track_panel_h);
    }

    // W12..W15: abbiegen/gerade aus HMI_weiche_w12..w15_{a,g}.svg.
    static const track_src_point_t w12_a0[] = { {967,196}, {993,222} };
    static const track_src_point_t w12_a1[] = { {948,206}, {922,232} };
    static const track_src_point_t w12_g0[] = { {993,222}, {967,196} };
    static const track_src_point_t w12_g1[] = { {940,194}, {903,194} };
    static const track_src_point_t w13_a0[] = { {883,193}, {846,193} };
    static const track_src_point_t w13_a1[] = { {826,207}, {800,233} };
    static const track_src_point_t w13_g0[] = { {883,194}, {846,194} };
    static const track_src_point_t w13_g1[] = { {820,193}, {783,193} };
    static const track_src_point_t w14_a0[] = { {433,600}, {407,626} };
    static const track_src_point_t w14_a1[] = { {395,639}, {358,639} };
    static const track_src_point_t w14_g0[] = { {450,640}, {413,640} };
    static const track_src_point_t w14_g1[] = { {395,640}, {358,640} };
    static const track_src_point_t w15_a0[] = { {310,601}, {284,627} };
    static const track_src_point_t w15_a1[] = { {263,636}, {237,610} };
    static const track_src_point_t w15_g0[] = { {326,638}, {290,638} };
    static const track_src_point_t w15_g1[] = { {263,636}, {237,610} };
    static lv_point_t p_w12_a0[2], p_w12_a1[2], p_w12_g0[2], p_w12_g1[2];
    static lv_point_t p_w13_a0[2], p_w13_a1[2], p_w13_g0[2], p_w13_g1[2];
    static lv_point_t p_w14_a0[2], p_w14_a1[2], p_w14_g0[2], p_w14_g1[2];
    static lv_point_t p_w15_a0[2], p_w15_a1[2], p_w15_g0[2], p_w15_g1[2];
    track_make_scaled_line(track_sbhf_turnout_layer[0][0], p_w12_a0, w12_a0, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[0][0], p_w12_a1, w12_a1, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[0][1], p_w12_g0, w12_g0, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[0][1], p_w12_g1, w12_g1, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[1][0], p_w13_a0, w13_a0, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[1][0], p_w13_a1, w13_a1, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[1][1], p_w13_g0, w13_g0, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[1][1], p_w13_g1, w13_g1, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[2][0], p_w14_a0, w14_a0, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[2][0], p_w14_a1, w14_a1, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[2][1], p_w14_g0, w14_g0, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[2][1], p_w14_g1, w14_g1, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[3][0], p_w15_a0, w15_a0, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[3][0], p_w15_a1, w15_a1, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[3][1], p_w15_g0, w15_g0, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);
    track_make_scaled_line(track_sbhf_turnout_layer[3][1], p_w15_g1, w15_g1, 2, ox, oy, scale_q10, turnout_ok_green, occ_line_w);

    int warn_line_w = (int)((10 * scale_q10) / 1024);
    if (warn_line_w < 4) warn_line_w = 4;
    if (warn_line_w > 9) warn_line_w = 9;
    static const track_src_point_t w12_warn[] = { {974,143}, {955,173}, {936,143}, {974,143} };
    static const track_src_point_t w13_warn[] = { {853,143}, {834,172}, {815,143}, {853,143} };
    static const track_src_point_t w14_warn[] = { {424,661}, {405,690}, {386,661}, {424,661} };
    static const track_src_point_t w15_warn[] = { {280,661}, {261,690}, {242,661}, {280,661} };
    static lv_point_t p_w12_warn[4], p_w13_warn[4], p_w14_warn[4], p_w15_warn[4];
    track_make_scaled_warn_triangle(track_sbhf_turnout_warn_layer[0], p_w12_warn, w12_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    track_make_scaled_warn_triangle(track_sbhf_turnout_warn_layer[1], p_w13_warn, w13_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    track_make_scaled_warn_triangle(track_sbhf_turnout_warn_layer[2], p_w14_warn, w14_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    track_make_scaled_warn_triangle(track_sbhf_turnout_warn_layer[3], p_w15_warn, w15_warn, ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);

    for (uint8_t i = 0; i < 3; ++i) {
        track_sbhf_target_layer[i] = track_make_overlay_layer(track_layer_sbhf, track_panel_w, track_panel_h);
    }
    static const track_src_point_t tgt1[] = { {917,239}, {783,372} };
    static const track_src_point_t tgt2[] = { {793,241}, {661,374} };
    static const track_src_point_t tgt3[] = { {673,241}, {540,373} };
    static const track_src_point_t tgt1_head[] = { {810,361}, {783,372}, {794,345} };
    static const track_src_point_t tgt2_head[] = { {688,363}, {661,374}, {672,347} };
    static const track_src_point_t tgt3_head[] = { {567,362}, {540,373}, {551,346} };
    static lv_point_t p_tgt1[2], p_tgt2[2], p_tgt3[2];
    static lv_point_t p_tgt1_head[3], p_tgt2_head[3], p_tgt3_head[3];
    const uint8_t target_w = occ_line_w; // ca. 60 % der Gleisbild-Strichdicke
    track_make_scaled_arrow_line(track_sbhf_target_layer[0], p_tgt1, tgt1, p_tgt1_head, tgt1_head, ox, oy, scale_q10, target_green, target_w);
    track_make_scaled_arrow_line(track_sbhf_target_layer[1], p_tgt2, tgt2, p_tgt2_head, tgt2_head, ox, oy, scale_q10, target_green, target_w);
    track_make_scaled_arrow_line(track_sbhf_target_layer[2], p_tgt3, tgt3, p_tgt3_head, tgt3_head, ox, oy, scale_q10, target_green, target_w);

    int sig_d = (int)((15 * scale_q10) / 1024);
    if (sig_d < 8) sig_d = 8;
    if (sig_d > 15) sig_d = 15;
    const int sig_pad = 4;
    typedef struct { uint16_t x; uint16_t top_y; uint16_t bottom_y; } track_signal_pos_t;
    static const track_signal_pos_t sigs[6] = {
        {1026, 516, 532}, // 4->5
        {1041, 217, 233}, // 5->SBHF
        {1112, 482, 498}, // 6->4
        {544, 552, 568},  // SBHF1->6
        {426, 551, 568},  // SBHF2->6
        {298, 552, 568},  // SBHF3->6
    };
    for (uint8_t i = 0; i < 6; ++i) {
        const int sig_x = ox + (sigs[i].x * scale_q10) / 1024;
        const int sig_top_y = oy + (sigs[i].top_y * scale_q10) / 1024;
        const int sig_bottom_y = oy + (sigs[i].bottom_y * scale_q10) / 1024;
        const int sig_box_x = sig_x - sig_d / 2 - sig_pad;
        const int sig_box_y = sig_top_y - sig_d / 2 - sig_pad;
        const int sig_box_w = sig_d + 2 * sig_pad;
        const int sig_box_h = (sig_bottom_y - sig_top_y) + sig_d + 2 * sig_pad;
        track_sbhf_signal_box[i] = track_make_signal_box(track_layer_sbhf, sig_box_x, sig_box_y, sig_box_w, sig_box_h);
        track_sbhf_signal_red[i] = track_make_signal_dot(track_layer_sbhf, sig_x, sig_top_y, sig_d, 0x555555);
        track_sbhf_signal_green[i] = track_make_signal_dot(track_layer_sbhf, sig_x, sig_bottom_y, sig_d, 0x555555);
    }
}

static void track_switch_level(track_level_t level)
{
    if (!track_root_panel) return;

    if (level == TRACK_LEVEL_E0 && !track_layer_e0_created) create_track_layer_e0();
    if (level == TRACK_LEVEL_SBHF && !track_layer_sbhf_created) create_track_layer_sbhf();

    track_active_level = level;

    if (track_layer_e1) {
        if (level == TRACK_LEVEL_E1) lv_obj_clear_flag(track_layer_e1, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(track_layer_e1, LV_OBJ_FLAG_HIDDEN);
    }
    if (track_layer_e0) {
        if (level == TRACK_LEVEL_E0) lv_obj_clear_flag(track_layer_e0, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(track_layer_e0, LV_OBJ_FLAG_HIDDEN);
    }
    if (track_layer_sbhf) {
        if (level == TRACK_LEVEL_SBHF) lv_obj_clear_flag(track_layer_sbhf, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(track_layer_sbhf, LV_OBJ_FLAG_HIDDEN);
    }

    if (track_sbhf_info_box) {
        if (level == TRACK_LEVEL_SBHF) lv_obj_clear_flag(track_sbhf_info_box, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(track_sbhf_info_box, LV_OBJ_FLAG_HIDDEN);
    }

    for (uint8_t i = 0; i < 3; ++i) {
        track_set_layer_button_active(track_level_btns[i], (track_level_t)i == level);
    }
}

static void on_track_level_button_clicked(lv_event_t *e)
{
    track_level_t level = (track_level_t)(uintptr_t)lv_event_get_user_data(e);
    track_switch_level(level);
}


static void create_track_tab_ebene1_start(lv_obj_t *tab, int left_w)
{
    lv_obj_set_style_bg_color(tab, lv_color_hex(0x2B3942), 0);
    // Der Gleisbild-Tab selbst scrollt nicht mehr: die Untertabs bleiben damit
    // fix sichtbar. Nur der Inhaltsbereich unterhalb der Untertabs darf vertikal
    // scrollen, damit die SBHF-Infobox voll lesbar bleibt.
    lv_obj_clear_flag(tab, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *level_bar = lv_obj_create(tab);
    lv_obj_set_size(level_bar, left_w - 58, 40);
    lv_obj_align(level_bar, LV_ALIGN_TOP_LEFT, 14, 2);
    lv_obj_set_style_bg_color(level_bar, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_bg_opa(level_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(level_bar, 0, 0);
    lv_obj_set_style_pad_all(level_bar, 0, 0);
    lv_obj_clear_flag(level_bar, LV_OBJ_FLAG_SCROLLABLE);

    // Zweite Ebene innerhalb des Gleisbild-Tabs. Optisch wie Tabs, aber bewusst
    // als leichte LVGL-Buttons umgesetzt: weniger Objektbaum als ein weiteres
    // verschachteltes Tabview und die Umschaltlogik bleibt kontrolliert lazy.
    track_level_btns[TRACK_LEVEL_E1] = track_make_layer_button(level_bar, "Ebene 1", 0, 4, 112, true);
    track_level_btns[TRACK_LEVEL_E0] = track_make_layer_button(level_bar, "Ebene 0", 120, 4, 112, false);
    track_level_btns[TRACK_LEVEL_SBHF] = track_make_layer_button(level_bar, "SBHF", 240, 4, 112, false);
    lv_obj_add_event_cb(track_level_btns[TRACK_LEVEL_E1], on_track_level_button_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)TRACK_LEVEL_E1);
    lv_obj_add_event_cb(track_level_btns[TRACK_LEVEL_E0], on_track_level_button_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)TRACK_LEVEL_E0);
    lv_obj_add_event_cb(track_level_btns[TRACK_LEVEL_SBHF], on_track_level_button_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)TRACK_LEVEL_SBHF);

    track_content_scroll = lv_obj_create(tab);
    lv_obj_set_size(track_content_scroll, left_w - 58, 476);
    lv_obj_align(track_content_scroll, LV_ALIGN_TOP_LEFT, 14, 44);
    lv_obj_set_style_bg_opa(track_content_scroll, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(track_content_scroll, 0, 0);
    lv_obj_set_style_pad_all(track_content_scroll, 0, 0);
    lv_obj_add_flag(track_content_scroll, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(track_content_scroll, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(track_content_scroll, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_clear_flag(track_content_scroll, LV_OBJ_FLAG_SCROLL_ELASTIC);

    lv_obj_t *panel = lv_obj_create(track_content_scroll);
    const int panel_w = left_w - 58;
    const int panel_h = 438;
    lv_obj_set_size(panel, panel_w, panel_h);
    lv_obj_align(panel, LV_ALIGN_TOP_LEFT, 0, 0);
    ui_card_style(panel, 0x17313A);
    lv_obj_clear_flag(panel, LV_OBJ_FLAG_SCROLLABLE);

    track_root_panel = panel;
    track_panel_w = panel_w;
    track_panel_h = panel_h;

    track_sbhf_info_box = lv_obj_create(track_content_scroll);
    lv_obj_set_size(track_sbhf_info_box, panel_w, 170);
    lv_obj_align(track_sbhf_info_box, LV_ALIGN_TOP_LEFT, 0, 452);
    ui_card_style(track_sbhf_info_box, 0x2D3C45);
    lv_obj_clear_flag(track_sbhf_info_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(track_sbhf_info_box, LV_OBJ_FLAG_HIDDEN);

    lv_obj_t *sbhf_info_title = lv_label_create(track_sbhf_info_box);
    lv_label_set_text(sbhf_info_title, "SBHF + Weichen (Mega2)");
    ui_label_style(sbhf_info_title, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(sbhf_info_title, LV_ALIGN_TOP_LEFT, 10, 8);

    track_sbhf_info_label = lv_label_create(track_sbhf_info_box);
    lv_label_set_long_mode(track_sbhf_info_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(track_sbhf_info_label, panel_w - 24);
    lv_label_set_text(track_sbhf_info_label, "State: -\nAusfahr-Gleis: -\nBelegt: -\nErlaubt: -\nRestricted: -\nStart: -");
    ui_label_style(track_sbhf_info_label, &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_align(track_sbhf_info_label, LV_ALIGN_TOP_LEFT, 10, 34);

    // Unsichtbarer Scroll-Puffer: Ohne dieses Element berechnet LVGL die
    // untere Content-Kante auf dem P4 gelegentlich zu knapp und federt nach
    // dem Loslassen wieder nach oben. Dadurch verschwinden die letzten
    // SBHF-Infozeilen teilweise unterhalb des sichtbaren Bereichs.
    lv_obj_t *track_scroll_bottom_spacer = lv_obj_create(track_content_scroll);
    lv_obj_set_size(track_scroll_bottom_spacer, panel_w, 76);
    lv_obj_align(track_scroll_bottom_spacer, LV_ALIGN_TOP_LEFT, 0, 636);
    lv_obj_set_style_bg_opa(track_scroll_bottom_spacer, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(track_scroll_bottom_spacer, 0, 0);
    lv_obj_set_style_pad_all(track_scroll_bottom_spacer, 0, 0);
    lv_obj_clear_flag(track_scroll_bottom_spacer, LV_OBJ_FLAG_SCROLLABLE);

    track_layer_e1 = track_make_overlay_layer(track_root_panel, panel_w, panel_h);
    lv_obj_clear_flag(track_layer_e1, LV_OBJ_FLAG_HIDDEN);
    track_layer_e1_created = true;
    panel = track_layer_e1;

    const int32_t usable_w = panel_w - 22;
    const int32_t usable_h = panel_h - 22;
    int32_t sx_q10 = (usable_w * 1024) / 1280;
    int32_t sy_q10 = (usable_h * 1024) / 720;
    int32_t scale_q10 = sx_q10 < sy_q10 ? sx_q10 : sy_q10;
    if (scale_q10 <= 0) scale_q10 = 1;

    const int32_t draw_w = (1280 * scale_q10) / 1024;
    const int32_t draw_h = (720 * scale_q10) / 1024;
    const int32_t ox = 11 + (usable_w - draw_w) / 2;
    const int32_t oy = 11 + (usable_h - draw_h) / 2;
    track_ox = ox;
    track_oy = oy;
    track_scale_q10 = scale_q10;

    int line_w = (int)((15 * scale_q10) / 1024);
    if (line_w < 5) line_w = 5;
    if (line_w > 12) line_w = 12;
    track_line_w = (uint8_t)line_w;

    const uint32_t rail = 0xFFFFFF;

    // Ebene 1: Track-Layout aus HMI_ebene1_base.svg, SVG-Referenz 1280x720.
    // Die HMI-SVG ist bereits als wenige zusammenhaengende Linienzuege aufgebaut.
    // Dadurch entstehen in LVGL nur wenige lv_line-Objekte und die Knicke bleiben sauber.
    // Weichen, Warnsymbole, Blockbelegung, Blockfreigaben und Klickflaechen
    // kommen spaeter als separate Objekte darueber.
    static const track_src_point_t e1_outer[] = {
        {829, 86}, {370, 86}, {227, 229}, {227, 511}, {368, 652},
        {641, 652}, {740, 553}, {966, 553}, {1064, 455}, {1064, 286},
        {971, 193}, {427, 193}, {324, 296}
    };

    static const track_src_point_t e1_top_diag[] = {
        {494, 86}, {600, 193}
    };

    static const track_src_point_t e1_inner[] = {
        {875, 553}, {604, 282}, {498, 282}, {412, 368}
    };

    static lv_point_t p_outer[sizeof(e1_outer) / sizeof(e1_outer[0])];
    static lv_point_t p_top_diag[sizeof(e1_top_diag) / sizeof(e1_top_diag[0])];
    static lv_point_t p_inner[sizeof(e1_inner) / sizeof(e1_inner[0])];

    track_make_scaled_line(panel, p_outer, e1_outer,
                           (uint16_t)(sizeof(e1_outer) / sizeof(e1_outer[0])),
                           ox, oy, scale_q10, rail, (uint8_t)line_w);
    track_make_scaled_line(panel, p_top_diag, e1_top_diag,
                           (uint16_t)(sizeof(e1_top_diag) / sizeof(e1_top_diag[0])),
                           ox, oy, scale_q10, rail, (uint8_t)line_w);
    track_make_scaled_line(panel, p_inner, e1_inner,
                           (uint16_t)(sizeof(e1_inner) / sizeof(e1_inner[0])),
                           ox, oy, scale_q10, rail, (uint8_t)line_w);

    // Blockbesetzt-Linien fuer Ebene 1 aus den HMI_block_e1_bX_occ.svg-Dateien.
    // Diese Overlays liegen ueber dem Gleis, sind duenne rote Linien und bleiben
    // bei frei/undef unsichtbar. Sie werden einmal erzeugt und spaeter nur
    // ein-/ausgeblendet.
    int occ_line_w = (line_w * 3 + 2) / 5;
    if (occ_line_w < 3) occ_line_w = 3;
    track_occ_line_w = (uint8_t)occ_line_w;
    const uint32_t occ_red = 0xF25F4C;

    track_e1_block_occ_layer[0] = track_make_overlay_layer(panel, panel_w, panel_h);
    track_e1_block_occ_layer[1] = track_make_overlay_layer(panel, panel_w, panel_h);
    track_e1_block_occ_layer[2] = track_make_overlay_layer(panel, panel_w, panel_h);

    static const track_src_point_t e1_b1_occ[] = {
        {325, 297}, {407, 215}
    };
    static const track_src_point_t e1_b2_occ_0[] = {
        {560, 87}, {828, 87}
    };
    static const track_src_point_t e1_b2_occ_1[] = {
        {448, 194}, {535, 194}
    };
    static const track_src_point_t e1_b2_occ_2[] = {
        {828, 505}, {604, 282}, {523, 282}
    };
    static const track_src_point_t e1_b2_occ_3[] = {
        {442, 87}, {370, 87}, {226, 232}, {226, 513}, {366, 653},
        {640, 653}, {740, 553}, {811, 553}
    };
    static const track_src_point_t e1_b2_occ_4[] = {
        {925, 552}, {967, 552}, {1065, 454}, {1065, 287}, {971, 193},
        {657, 193}
    };
    static const track_src_point_t e1_b3_occ[] = {
        {412, 367}, {480, 300}
    };

    static lv_point_t p_b1_occ[sizeof(e1_b1_occ) / sizeof(e1_b1_occ[0])];
    static lv_point_t p_b2_occ_0[sizeof(e1_b2_occ_0) / sizeof(e1_b2_occ_0[0])];
    static lv_point_t p_b2_occ_1[sizeof(e1_b2_occ_1) / sizeof(e1_b2_occ_1[0])];
    static lv_point_t p_b2_occ_2[sizeof(e1_b2_occ_2) / sizeof(e1_b2_occ_2[0])];
    static lv_point_t p_b2_occ_3[sizeof(e1_b2_occ_3) / sizeof(e1_b2_occ_3[0])];
    static lv_point_t p_b2_occ_4[sizeof(e1_b2_occ_4) / sizeof(e1_b2_occ_4[0])];
    static lv_point_t p_b3_occ[sizeof(e1_b3_occ) / sizeof(e1_b3_occ[0])];

    track_make_scaled_line(track_e1_block_occ_layer[0], p_b1_occ, e1_b1_occ,
                           (uint16_t)(sizeof(e1_b1_occ) / sizeof(e1_b1_occ[0])),
                           ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    track_make_scaled_line(track_e1_block_occ_layer[1], p_b2_occ_0, e1_b2_occ_0,
                           (uint16_t)(sizeof(e1_b2_occ_0) / sizeof(e1_b2_occ_0[0])),
                           ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    track_make_scaled_line(track_e1_block_occ_layer[1], p_b2_occ_1, e1_b2_occ_1,
                           (uint16_t)(sizeof(e1_b2_occ_1) / sizeof(e1_b2_occ_1[0])),
                           ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    track_make_scaled_line(track_e1_block_occ_layer[1], p_b2_occ_2, e1_b2_occ_2,
                           (uint16_t)(sizeof(e1_b2_occ_2) / sizeof(e1_b2_occ_2[0])),
                           ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    track_make_scaled_line(track_e1_block_occ_layer[1], p_b2_occ_3, e1_b2_occ_3,
                           (uint16_t)(sizeof(e1_b2_occ_3) / sizeof(e1_b2_occ_3[0])),
                           ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    track_make_scaled_line(track_e1_block_occ_layer[1], p_b2_occ_4, e1_b2_occ_4,
                           (uint16_t)(sizeof(e1_b2_occ_4) / sizeof(e1_b2_occ_4[0])),
                           ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);
    track_make_scaled_line(track_e1_block_occ_layer[2], p_b3_occ, e1_b3_occ,
                           (uint16_t)(sizeof(e1_b3_occ) / sizeof(e1_b3_occ[0])),
                           ox, oy, scale_q10, occ_red, (uint8_t)occ_line_w);

    // Weichen-Overlays fuer Ebene 1: W9, W10, W11.
    // Jede Weiche hat genau zwei vorbereitete Layer: abbiegen und gerade.
    // Spaeter wird nur der Layer passend zur Ist-Stellung sichtbar geschaltet.
    // Gruen = Ist == Soll, Rot = Ist != Soll.
    int turnout_line_w = occ_line_w;
    if (turnout_line_w < 3) turnout_line_w = 3;
    const uint32_t turnout_ok_green = 0x2ECC71;

    for (uint8_t i = 0; i < 3; ++i) {
        track_e1_turnout_layer[i][0] = track_make_overlay_layer(panel, panel_w, panel_h);
        track_e1_turnout_layer[i][1] = track_make_overlay_layer(panel, panel_w, panel_h);
        track_e1_turnout_warn_layer[i] = track_make_overlay_layer(panel, panel_w, panel_h);
    }

    // W9 abbiegen/gerade aus HMI_weiche_w9_a.svg / HMI_weiche_w9_g.svg
    static const track_src_point_t e1_w9_a_0[] = { {836, 512}, {862, 538} };
    static const track_src_point_t e1_w9_a_1[] = { {881, 552}, {917, 552} };
    static const track_src_point_t e1_w9_g_0[] = { {819, 553}, {855, 553} };
    static const track_src_point_t e1_w9_g_1[] = { {881, 553}, {917, 553} };

    // W10 abbiegen/gerade aus HMI_weiche_w10_a.svg / HMI_weiche_w10_g.svg
    static const track_src_point_t e1_w10_a_0[] = { {562, 154}, {588, 180} };
    static const track_src_point_t e1_w10_a_1[] = { {609, 193}, {646, 193} };
    static const track_src_point_t e1_w10_g_0[] = { {545, 193}, {582, 193} };
    static const track_src_point_t e1_w10_g_1[] = { {609, 193}, {646, 193} };

    // W11 abbiegen/gerade aus HMI_weiche_w11_a.svg / HMI_weiche_w11_g.svg
    static const track_src_point_t e1_w11_a_0[] = { {452, 86}, {488, 86} };
    static const track_src_point_t e1_w11_a_1[] = { {507, 99}, {533, 125} };
    static const track_src_point_t e1_w11_g_0[] = { {452, 87}, {488, 87} };
    static const track_src_point_t e1_w11_g_1[] = { {514, 86}, {550, 86} };

    static lv_point_t p_w9_a_0[2], p_w9_a_1[2], p_w9_g_0[2], p_w9_g_1[2];
    static lv_point_t p_w10_a_0[2], p_w10_a_1[2], p_w10_g_0[2], p_w10_g_1[2];
    static lv_point_t p_w11_a_0[2], p_w11_a_1[2], p_w11_g_0[2], p_w11_g_1[2];

    track_make_scaled_line(track_e1_turnout_layer[0][0], p_w9_a_0, e1_w9_a_0, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[0][0], p_w9_a_1, e1_w9_a_1, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[0][1], p_w9_g_0, e1_w9_g_0, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[0][1], p_w9_g_1, e1_w9_g_1, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);

    track_make_scaled_line(track_e1_turnout_layer[1][0], p_w10_a_0, e1_w10_a_0, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[1][0], p_w10_a_1, e1_w10_a_1, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[1][1], p_w10_g_0, e1_w10_g_0, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[1][1], p_w10_g_1, e1_w10_g_1, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);

    track_make_scaled_line(track_e1_turnout_layer[2][0], p_w11_a_0, e1_w11_a_0, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[2][0], p_w11_a_1, e1_w11_a_1, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[2][1], p_w11_g_0, e1_w11_g_0, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);
    track_make_scaled_line(track_e1_turnout_layer[2][1], p_w11_g_1, e1_w11_g_1, 2, ox, oy, scale_q10, turnout_ok_green, (uint8_t)turnout_line_w);

    // Warnsymbole fuer W9, W10, W11 aus den HMI_weiche_wX_warn.svg-Dateien.
    // Sichtbarkeit wird spaeter nur ueber warningMask geschaltet.
    int warn_line_w = (int)((10 * scale_q10) / 1024);
    if (warn_line_w < 4) warn_line_w = 4;
    if (warn_line_w > 9) warn_line_w = 9;
    const uint32_t warn_red = 0xFF0000;

    static const track_src_point_t e1_w9_warn[] = {
        {908, 502}, {890, 533}, {871, 502}, {908, 502}
    };
    static const track_src_point_t e1_w10_warn[] = {
        {634, 143}, {615, 173}, {596, 143}, {634, 143}
    };
    static const track_src_point_t e1_w11_warn[] = {
        {492, 107}, {473, 137}, {455, 107}, {492, 107}
    };

    static lv_point_t p_w9_warn[4], p_w10_warn[4], p_w11_warn[4];
    track_make_scaled_warn_triangle(track_e1_turnout_warn_layer[0], p_w9_warn, e1_w9_warn,
                                    ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    track_make_scaled_warn_triangle(track_e1_turnout_warn_layer[1], p_w10_warn, e1_w10_warn,
                                    ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);
    track_make_scaled_warn_triangle(track_e1_turnout_warn_layer[2], p_w11_warn, e1_w11_warn,
                                    ox, oy, scale_q10, warn_red, (uint8_t)warn_line_w);

    // Signale Ebene 1: Blockfreigaben und Bahnhofs-Ausfahrten.
    // Die SVGs werden nicht als Bild geladen; verwendet werden nur die Positionen.
    // Darstellung:
    //   freigegeben       -> unten gruen
    //   nicht freigegeben -> oben rot
    //   unbekannt/offline -> beide grau
    int sig_d = (int)((15 * scale_q10) / 1024);
    if (sig_d < 8) sig_d = 8;
    if (sig_d > 15) sig_d = 15;
    const int sig_pad = 4;

    typedef struct {
        uint16_t x;
        uint16_t top_y;
        uint16_t bottom_y;
    } track_signal_pos_t;

    static const track_signal_pos_t e1_signal_pos[4] = {
        {419, 241, 258}, // Grant Block 1 -> Block 2, aus grant_1_2_green.svg
        {529, 239, 256}, // Grant Block 2 -> Block 3, aus grant_2_3_green.svg
        {254, 456, 472}, // Bhf2, aus bhf2_green.svg
        {643, 267, 283}, // Bhf3, aus bhf3_green.svg
    };

    for (uint8_t i = 0; i < 4; ++i) {
        const int sig_x = ox + (e1_signal_pos[i].x * scale_q10) / 1024;
        const int sig_top_y = oy + (e1_signal_pos[i].top_y * scale_q10) / 1024;
        const int sig_bottom_y = oy + (e1_signal_pos[i].bottom_y * scale_q10) / 1024;
        const int sig_box_x = sig_x - sig_d / 2 - sig_pad;
        const int sig_box_y = sig_top_y - sig_d / 2 - sig_pad;
        const int sig_box_w = sig_d + 2 * sig_pad;
        const int sig_box_h = (sig_bottom_y - sig_top_y) + sig_d + 2 * sig_pad;
        track_e1_signal_box[i] = track_make_signal_box(panel, sig_box_x, sig_box_y, sig_box_w, sig_box_h);
        track_e1_signal_red[i] = track_make_signal_dot(panel, sig_x, sig_top_y, sig_d, 0x555555);
        track_e1_signal_green[i] = track_make_signal_dot(panel, sig_x, sig_bottom_y, sig_d, 0x555555);
    }

    // Grosszuegige transparente Touch-Hitboxen fuer Ebene 1.
    // Die sichtbaren Gleis-/Signalobjekte bleiben rein statisch; die Hitboxen
    // senden die gleichen Commands wie die bestehenden Tabs.
    // Reihenfolge: W9, W10, W11 sowie Bhf2, Bhf3.
    typedef struct {
        uint16_t x;
        uint16_t y;
        uint16_t w;
        uint16_t h;
        uint8_t idx;
    } track_hitbox_pos_t;

    static const track_hitbox_pos_t turnout_hitboxes[3] = {
        {790, 490, 160, 100, 9},   // W9
        {525, 130, 145, 100, 10},  // W10
        {435,  62, 140, 100, 11},  // W11
    };
    static const track_hitbox_pos_t station_hitboxes[2] = {
        {220, 430,  90,  90, 2},   // Bhf2
        {610, 240,  90,  90, 3},   // Bhf3
    };

    for (uint8_t i = 0; i < 3; ++i) {
        const int hx = ox + (turnout_hitboxes[i].x * scale_q10) / 1024;
        const int hy = oy + (turnout_hitboxes[i].y * scale_q10) / 1024;
        const int hw = (turnout_hitboxes[i].w * scale_q10) / 1024;
        const int hh = (turnout_hitboxes[i].h * scale_q10) / 1024;
        track_e1_hitbox_turnout[i] = track_make_hitbox(panel, hx, hy, hw, hh,
                                                       on_left_m1_turnout_clicked,
                                                       (void *)(uintptr_t)turnout_hitboxes[i].idx);
    }
    for (uint8_t i = 0; i < 2; ++i) {
        const int hx = ox + (station_hitboxes[i].x * scale_q10) / 1024;
        const int hy = oy + (station_hitboxes[i].y * scale_q10) / 1024;
        const int hw = (station_hitboxes[i].w * scale_q10) / 1024;
        const int hh = (station_hitboxes[i].h * scale_q10) / 1024;
        track_e1_hitbox_station[i] = track_make_hitbox(panel, hx, hy, hw, hh,
                                                       on_left_station_clicked,
                                                       (void *)(uintptr_t)station_hitboxes[i].idx);
    }
}

static void create_left_panel_tabs(lv_obj_t *left_panel, int left_w)
{
    lv_obj_t *title = lv_label_create(left_panel);
    lv_label_set_text(title, "Elektrische Eisenbahn HMI");
    ui_label_style(title, &lv_font_montserrat_26, 0xFFFFFF);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 14, 8);

    left_tabview = lv_tabview_create(left_panel, LV_DIR_TOP, 45);
    lv_obj_set_size(left_tabview, left_w - 18, 525);
    lv_obj_align(left_tabview, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_set_style_bg_color(left_tabview, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_text_color(left_tabview, lv_color_hex(0xFFFFFF), 0);

    lv_obj_t *tab_weichen = lv_tabview_add_tab(left_tabview, "Weichen");
    lv_obj_t *tab_bahnhoefe = lv_tabview_add_tab(left_tabview, "Bahnhoefe");
    lv_obj_t *tab_bloecke = lv_tabview_add_tab(left_tabview, "Bloecke");
    lv_obj_t *tab_gleisbild = lv_tabview_add_tab(left_tabview, "Gleisbild");
    lv_obj_t *tab_settings = lv_tabview_add_tab(left_tabview, "Einstellungen");

    lv_obj_set_style_bg_color(tab_weichen, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_bg_color(tab_bahnhoefe, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_bg_color(tab_bloecke, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_bg_color(tab_gleisbild, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_bg_color(tab_settings, lv_color_hex(0xF3EFE3), 0);
    lv_obj_clear_flag(tab_weichen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(tab_bahnhoefe, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(tab_bloecke, LV_OBJ_FLAG_SCROLLABLE);
    // Gleisbild: Untertabs bleiben fix sichtbar; nur der Inhalt darunter scrollt.
    lv_obj_clear_flag(tab_gleisbild, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *m1_panel = lv_obj_create(tab_weichen);
    lv_obj_set_size(m1_panel, left_w - 58, 282);
    lv_obj_align(m1_panel, LV_ALIGN_TOP_LEFT, 8, 8);
    ui_card_style(m1_panel, 0x2D3C45);

    lv_obj_t *m1_title = lv_label_create(m1_panel);
    lv_label_set_text(m1_title, "Mega1-Weichen W0-W11 (Bedienung)");
    ui_label_style(m1_title, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(m1_title, LV_ALIGN_TOP_LEFT, 10, 8);

    left_weichen_cmd_status_label = lv_label_create(m1_panel);
    lv_label_set_text(left_weichen_cmd_status_label, "Bedienung gesperrt");
    ui_label_style(left_weichen_cmd_status_label, &lv_font_montserrat_14, 0xF9D342);
    lv_obj_align(left_weichen_cmd_status_label, LV_ALIGN_TOP_RIGHT, -10, 10);

    for (int i = 0; i < 12; ++i) {
        int col = i % 4;
        int row = i / 4;
        int x = 10 + col * 150;
        int y = 44 + row * 74;
        left_m1_turnout_btns[i] = ui_make_pill(m1_panel, "", x, y, 135, 60, 0x1F5361, 0xFFFFFF);
        left_m1_turnout_labels[i] = lv_obj_get_child(left_m1_turnout_btns[i], 0);
        lv_obj_add_event_cb(left_m1_turnout_btns[i], on_left_m1_turnout_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
        char buf[48];
        snprintf(buf, sizeof(buf), "W%d\nIst -- | Soll --", i);
        ui_label_set_text_if_changed(left_m1_turnout_labels[i], buf);
    }

    lv_obj_t *sbhf_panel = lv_obj_create(tab_weichen);
    lv_obj_set_size(sbhf_panel, left_w - 58, 150);
    lv_obj_align(sbhf_panel, LV_ALIGN_TOP_LEFT, 8, 304);
    ui_card_style(sbhf_panel, 0x2D3C45);

    lv_obj_t *sbhf_title = lv_label_create(sbhf_panel);
    lv_label_set_text(sbhf_title, "SBHF-Weichen W12-W15 (Anzeige)");
    ui_label_style(sbhf_title, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(sbhf_title, LV_ALIGN_TOP_LEFT, 10, 8);


    for (int i = 0; i < 4; ++i) {
        int w = 12 + i;
        int x = 10 + i * 150;
        left_sbhf_turnout_cards[i] = ui_make_pill(sbhf_panel, "", x, 52, 135, 62, 0x1F5361, 0xFFFFFF);
        lv_obj_set_style_radius(left_sbhf_turnout_cards[i], 3, 0);
        left_sbhf_turnout_labels[i] = lv_obj_get_child(left_sbhf_turnout_cards[i], 0);
        char buf[48];
        snprintf(buf, sizeof(buf), "W%d\nIst -- | Soll --", w);
        ui_label_set_text_if_changed(left_sbhf_turnout_labels[i], buf);
    }

    lv_obj_t *bh_title = lv_label_create(tab_bahnhoefe);
    lv_label_set_text(bh_title, "Bahnhoefe");
    ui_label_style(bh_title, &lv_font_montserrat_22, 0xFFFFFF);
    lv_obj_align(bh_title, LV_ALIGN_TOP_LEFT, 10, 10);

    left_station_cmd_status_label = lv_label_create(tab_bahnhoefe);
    lv_label_set_text(left_station_cmd_status_label, "Bedienung gesperrt");
    ui_label_style(left_station_cmd_status_label, &lv_font_montserrat_14, 0xF9D342);
    lv_obj_align(left_station_cmd_status_label, LV_ALIGN_TOP_RIGHT, -14, 16);

    const char *station_names[] = {"Bhf0", "Bhf1", "Bhf2", "Bhf3"};
    for (int i = 0; i < 4; ++i) {
        int col = i % 4;
        int row = i / 4;
        int x = 10 + col * 150;
        int y = 58 + row * 116;
        lv_obj_t *card = lv_obj_create(tab_bahnhoefe);
        left_station_cards[i] = card;
        lv_obj_set_size(card, 135, 104);
        lv_obj_align(card, LV_ALIGN_TOP_LEFT, x, y);
        ui_card_style(card, 0x20333C);
        lv_obj_add_flag(card, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(card, on_left_station_clicked, LV_EVENT_CLICKED, (void *)(uintptr_t)i);

        lv_obj_t *label = lv_label_create(card);
        left_station_labels[i] = label;
        lv_label_set_text(label, station_names[i]);
        lv_label_set_long_mode(label, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(label, 114);
        ui_label_style(label, &lv_font_montserrat_14, 0xFFFFFF);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, 12, 10);

        left_station_led_red[i] = ui_make_signal_led(card, 18, 64, 0x555555);
        left_station_led_green[i] = ui_make_signal_led(card, 50, 64, 0x555555);
    }

    lv_obj_t *bl_title = lv_label_create(tab_bloecke);
    lv_label_set_text(bl_title, "Bloecke");
    ui_label_style(bl_title, &lv_font_montserrat_22, 0xFFFFFF);
    lv_obj_align(bl_title, LV_ALIGN_TOP_LEFT, 10, 10);


    lv_obj_t *occ_panel = lv_obj_create(tab_bloecke);
    lv_obj_set_size(occ_panel, left_w - 58, 190);
    lv_obj_align(occ_panel, LV_ALIGN_TOP_LEFT, 8, 48);
    ui_card_style(occ_panel, 0x2D3C45);

    lv_obj_t *occ_title = lv_label_create(occ_panel);
    lv_label_set_text(occ_title, "Blockbesetzt-Status");
    ui_label_style(occ_title, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(occ_title, LV_ALIGN_TOP_LEFT, 10, 8);

    const char *occ_names[] = {"Block 1", "Block 2", "Block 3", "Block 4", "Block 5", "SBHF 1", "SBHF 2", "SBHF 3", "Block 6"};
    for (int i = 0; i < 9; ++i) {
        int col = i % 3;
        int row = i / 3;
        int x = 10 + col * 200;
        int y = 40 + row * 45;
        lv_obj_t *label = lv_label_create(occ_panel);
        lv_label_set_text(label, occ_names[i]);
        ui_label_style(label, &lv_font_montserrat_14, 0xFFFFFF);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y + 4);
        left_block_occ_red[i] = ui_make_signal_led(occ_panel, x + 94, y + 4, 0x555555);
        left_block_occ_green[i] = ui_make_signal_led(occ_panel, x + 122, y + 4, 0x555555);
    }

    lv_obj_t *grant_panel = lv_obj_create(tab_bloecke);
    lv_obj_set_size(grant_panel, left_w - 58, 190);
    lv_obj_align(grant_panel, LV_ALIGN_TOP_LEFT, 8, 252);
    ui_card_style(grant_panel, 0x2D3C45);

    lv_obj_t *grant_title = lv_label_create(grant_panel);
    lv_label_set_text(grant_title, "Blockfreigaben");
    ui_label_style(grant_title, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(grant_title, LV_ALIGN_TOP_LEFT, 10, 8);

    const char *grant_names[] = {"Block 1 -> 2", "Block 2 -> 3", "Block 3 -> 4", "Block 6 -> 4", "Block 4 -> 5", "Block 5 -> SBHF", "SBHF 1 -> Block6", "SBHF 2 -> Block6", "SBHF 3 -> Block6"};
    for (int i = 0; i < 9; ++i) {
        int col = i % 3;
        int row = i / 3;
        int x = 10 + col * 200;
        int y = 40 + row * 45;
        lv_obj_t *label = lv_label_create(grant_panel);
        lv_label_set_text(label, grant_names[i]);
        ui_label_style(label, &lv_font_montserrat_14, 0xFFFFFF);
        lv_obj_align(label, LV_ALIGN_TOP_LEFT, x, y + 4);
        left_block_grant_red[i] = ui_make_signal_led(grant_panel, x + 132, y + 4, 0x555555);
        left_block_grant_green[i] = ui_make_signal_led(grant_panel, x + 160, y + 4, 0x555555);
    }

    create_track_tab_ebene1_start(tab_gleisbild, left_w);

    // HMI startet direkt im Gleisbild auf Ebene SBHF. Ebene 0 bleibt lazy
    // und wird erst beim ersten Umschalten erzeugt.
    track_switch_level(TRACK_LEVEL_SBHF);
    lv_tabview_set_act(left_tabview, 3, LV_ANIM_OFF);

    lv_obj_t *settings_title = lv_label_create(tab_settings);
    lv_label_set_text(settings_title, "Einstellungen");
    ui_label_style(settings_title, &lv_font_montserrat_24, 0x333333);
    lv_obj_align(settings_title, LV_ALIGN_TOP_LEFT, 12, 10);

    lv_obj_t *display_panel = lv_obj_create(tab_settings);
    lv_obj_set_size(display_panel, left_w - 65, 220);
    lv_obj_align(display_panel, LV_ALIGN_TOP_LEFT, 12, 48);
    ui_card_style(display_panel, 0x17313A);

    lv_obj_t *display_title = lv_label_create(display_panel);
    lv_label_set_text(display_title, "Display");
    ui_label_style(display_title, &lv_font_montserrat_18, 0xFFFFFF);
    lv_obj_align(display_title, LV_ALIGN_TOP_LEFT, 12, 8);

    settings_brightness_label = lv_label_create(display_panel);
    lv_label_set_text(settings_brightness_label, "Helligkeit: 70 %");
    ui_label_style(settings_brightness_label, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(settings_brightness_label, LV_ALIGN_TOP_LEFT, 12, 40);

    settings_brightness_slider = lv_slider_create(display_panel);
    lv_obj_set_size(settings_brightness_slider, left_w - 115, 24);
    lv_obj_align(settings_brightness_slider, LV_ALIGN_TOP_LEFT, 12, 70);
    lv_slider_set_range(settings_brightness_slider, 10, 100);
    lv_slider_set_value(settings_brightness_slider, g_display_brightness_percent, LV_ANIM_OFF);
    lv_obj_add_event_cb(settings_brightness_slider, settings_brightness_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    lv_obj_add_event_cb(settings_brightness_slider, settings_brightness_event_cb, LV_EVENT_RELEASED, NULL);
    lv_obj_add_event_cb(settings_brightness_slider, settings_brightness_event_cb, LV_EVENT_PRESS_LOST, NULL);

    lv_obj_t *timeout_label = lv_label_create(display_panel);
    lv_label_set_text(timeout_label, "Screen-Off nach Inaktivitaet");
    ui_label_style(timeout_label, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(timeout_label, LV_ALIGN_TOP_LEFT, 12, 112);

    settings_timeout_dropdown = lv_dropdown_create(display_panel);
    lv_dropdown_set_options(settings_timeout_dropdown, "Aus\n30 s\n1 min\n2 min\n5 min\n10 min");
    lv_dropdown_set_selected(settings_timeout_dropdown, timeout_sec_to_dropdown(g_screen_off_timeout_sec));
    lv_obj_set_size(settings_timeout_dropdown, 220, 42);
    lv_obj_align(settings_timeout_dropdown, LV_ALIGN_TOP_LEFT, 12, 144);
    lv_obj_add_event_cb(settings_timeout_dropdown, settings_timeout_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *hint = lv_label_create(tab_settings);
    lv_label_set_text(hint,
                      "Screen-Off schaltet nur die Hintergrundbeleuchtung aus.\n"
                      "LVGL, UART und die Statusverarbeitung laufen weiter.\n"
                      "Der erste Touch bei dunklem Display weckt nur auf und loest keine Anlagenaktion aus.");
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(hint, left_w - 65);
    ui_label_style(hint, &lv_font_montserrat_16, 0x333333);
    lv_obj_align(hint, LV_ALIGN_TOP_LEFT, 12, 282);

    lv_obj_t *comm_panel = lv_obj_create(tab_settings);
    lv_obj_set_size(comm_panel, left_w - 65, 190);
    lv_obj_align(comm_panel, LV_ALIGN_TOP_LEFT, 12, 372);
    ui_card_style(comm_panel, 0x20333C);

    lv_obj_t *comm_title = lv_label_create(comm_panel);
    lv_label_set_text(comm_title, "Kommunikation kompakt");
    ui_label_style(comm_title, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(comm_title, LV_ALIGN_TOP_LEFT, 12, 8);

    left_debug_summary_label = lv_label_create(comm_panel);
    lv_label_set_text(left_debug_summary_label, "RX/TX, ACK und Frame-Typen kompakt.");
    lv_label_set_long_mode(left_debug_summary_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(left_debug_summary_label, left_w - 105);
    ui_label_style(left_debug_summary_label, &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_align(left_debug_summary_label, LV_ALIGN_TOP_LEFT, 12, 34);

    comm_status_label = lv_label_create(comm_panel);
    lv_label_set_text(comm_status_label, "state-lite: 0 | analog: 0 | other: 0");
    ui_label_style(comm_status_label, &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_align(comm_status_label, LV_ALIGN_TOP_LEFT, 12, 88);

    update_settings_labels();

}

static uint8_t left_block_occ_display_to_mask_bit(uint8_t display_idx)
{
    // UI-Reihenfolge: B1, B2, B3, B4, B5, SBHF1, SBHF2, SBHF3, B6.
    // Mega2-Maskenreihenfolge: B1, B2, B3, B4, B5, B6, SBHF1, SBHF2, SBHF3.
    static const uint8_t map[9] = {0, 1, 2, 3, 4, 6, 7, 8, 5};
    return (display_idx < 9u) ? map[display_idx] : 0u;
}

static int left_grant_display_state(const hmi_state_t *s, uint8_t display_idx)
{
    if (!s || !s->mega2_online) return -1;

    switch (display_idx) {
        case 0:  // Block 1 -> 2
        case 1:  // Block 2 -> 3
        case 2:  // Block 3 -> 4
        case 4:  // Block 4 -> 5
            if (!s->mega2_signal_grant_valid) return -1;
            return (s->mega2_signal_grant_mask & (1u << display_idx)) ? 1 : 0;

        case 3:  // Block 6 -> 4
            if (!s->mega2_signal_grant_valid) return -1;
            return (s->mega2_signal_grant_mask & (1u << 11)) ? 1 : 0;

        case 5:  // Block 5 -> SBHF
            if (!s->mega2_block5_to_sbhf_valid) return -1;
            return s->mega2_block5_to_sbhf_active ? 1 : 0;

        case 6:  // SBHF 1 -> Block6
        case 7:  // SBHF 2 -> Block6
        case 8:  // SBHF 3 -> Block6
            return (s->mega2_sbhf_state == 4u &&
                    s->mega2_sbhf_current_gleis == (uint8_t)(display_idx - 5u)) ? 1 : 0;

        default:
            return -1;
    }
}

static void update_track_e1_occ_ui(const hmi_state_t *s)
{
    if (!s) return;

    const bool occ_valid = s->mega2_online && s->mega2_block_occ_valid;
    for (uint8_t i = 0; i < 3; ++i) {
        lv_obj_t *layer = track_e1_block_occ_layer[i];
        if (!layer) continue;

        // Ebene 1 zeigt Block 1, Block 2 und Block 3.
        // Bei frei oder fehlender Information wird keine rote Linie dargestellt.
        const bool occupied = occ_valid && ((s->mega2_block_occ_mask & (1u << i)) != 0u);
        if (occupied) lv_obj_clear_flag(layer, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(layer, LV_OBJ_FLAG_HIDDEN);
    }
}


static bool track_mega1_defect_text_mentions_turnout(const hmi_state_t *s, uint8_t id)
{
    if (!s || s->mega1_defects[0] == '\0') return false;
    switch (id) {
        case 9:  return strstr(s->mega1_defects, "W9")  != NULL;
        case 10: return strstr(s->mega1_defects, "W10") != NULL;
        case 11: return strstr(s->mega1_defects, "W11") != NULL;
        default: return false;
    }
}

static void update_track_e1_turnouts_ui(const hmi_state_t *s)
{
    if (!s) return;

    // Ebene 1 zeigt aktuell die Mega1-Weichen W9, W10, W11.
    // Bit gesetzt bedeutet wie im bestehenden Weichen-Tab: gerade.
    // Es gibt derzeit keinen undef-Zustand; bei Mega1 offline blenden wir aus.
    static const uint8_t turnout_ids[3] = { 9, 10, 11 };
    const bool valid = s->mega1_online;

    for (uint8_t i = 0; i < 3; ++i) {
        const uint8_t id = turnout_ids[i];
        const bool ist_gerade = ((s->mega1_weiche_ist_bits & (1u << id)) != 0u);
        const bool soll_gerade = ((s->mega1_weiche_soll_bits & (1u << id)) != 0u);
        const bool mismatch = (ist_gerade != soll_gerade);
        const uint32_t color = mismatch ? 0xF25F4C : 0x2ECC71;
        const bool warn = valid && (((s->mega1_warning_mask & (1u << id)) != 0u) ||
                                    track_mega1_defect_text_mentions_turnout(s, id));

        lv_obj_t *layer_a = track_e1_turnout_layer[i][0];
        lv_obj_t *layer_g = track_e1_turnout_layer[i][1];
        lv_obj_t *layer_warn = track_e1_turnout_warn_layer[i];

        if (layer_a) {
            track_set_layer_color(layer_a, color);
            if (valid && !ist_gerade) lv_obj_clear_flag(layer_a, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_a, LV_OBJ_FLAG_HIDDEN);
        }
        if (layer_g) {
            track_set_layer_color(layer_g, color);
            if (valid && ist_gerade) lv_obj_clear_flag(layer_g, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_g, LV_OBJ_FLAG_HIDDEN);
        }
        if (layer_warn) {
            if (warn) lv_obj_clear_flag(layer_warn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_warn, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void update_track_e1_grants_ui(const hmi_state_t *s)
{
    if (!s) return;

    // Ebene 1 Signal 0/1: Blockfreigaben 1->2 und 2->3.
    // Die Bitbelegung entspricht der vorhandenen linken Blockfreigabe-Liste:
    // display_idx 0 = SignalGrantMask Bit 0, display_idx 1 = Bit 1.
    for (uint8_t i = 0; i < 2; ++i) {
        int state = -1;
        if (s->mega2_online && s->mega2_signal_grant_valid) {
            state = (s->mega2_signal_grant_mask & (1u << i)) ? 1 : 0;
        }
        track_set_grant_signal(track_e1_signal_red[i], track_e1_signal_green[i], state);
    }

    // Ebene 1 Signal 2/3: Bahnhofs-Ausfahrten Bhf2/Bhf3.
    for (uint8_t i = 0; i < 2; ++i) {
        const uint8_t bhf_id = (uint8_t)(i + 2);
        int state = -1;
        if (s->mega1_online && s->mega1_bahnhof_valid) {
            state = (s->mega1_bahnhof_mask & (1u << bhf_id)) ? 1 : 0;
        }
        track_set_grant_signal(track_e1_signal_red[i + 2], track_e1_signal_green[i + 2], state);
    }
}


static bool track_mega1_defect_text_mentions_any_turnout(const hmi_state_t *s, uint8_t id)
{
    if (!s || s->mega1_defects[0] == '\0') return false;
    char token[5];
    snprintf(token, sizeof(token), "W%u", (unsigned)id);
    return strstr(s->mega1_defects, token) != NULL;
}

static void update_track_e0_occ_ui(const hmi_state_t *s)
{
    if (!s || !track_layer_e0_created) return;

    static const uint8_t occ_bits[5] = {0, 2, 3, 4, 5}; // B1, B3, B4, B5, B6
    const bool occ_valid = s->mega2_online && s->mega2_block_occ_valid;
    for (uint8_t i = 0; i < 5u; ++i) {
        lv_obj_t *layer = track_e0_block_occ_layer[i];
        if (!layer) continue;
        const bool occupied = occ_valid && ((s->mega2_block_occ_mask & (1u << occ_bits[i])) != 0u);
        if (occupied) lv_obj_clear_flag(layer, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(layer, LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_track_e0_turnouts_ui(const hmi_state_t *s)
{
    if (!s || !track_layer_e0_created) return;

    const bool valid = s->mega1_online;
    for (uint8_t id = 0; id < 9u; ++id) {
        const bool ist_gerade = ((s->mega1_weiche_ist_bits & (1u << id)) != 0u);
        const bool soll_gerade = ((s->mega1_weiche_soll_bits & (1u << id)) != 0u);
        const bool mismatch = (ist_gerade != soll_gerade);
        const uint32_t color = mismatch ? 0xF25F4C : 0x2ECC71;
        const bool warn = valid && (((s->mega1_warning_mask & (1u << id)) != 0u) ||
                                    track_mega1_defect_text_mentions_any_turnout(s, id));

        lv_obj_t *layer_a = track_e0_turnout_layer[id][0];
        lv_obj_t *layer_g = track_e0_turnout_layer[id][1];
        lv_obj_t *layer_warn = track_e0_turnout_warn_layer[id];

        if (layer_a) {
            track_set_layer_color(layer_a, color);
            if (valid && !ist_gerade) lv_obj_clear_flag(layer_a, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_a, LV_OBJ_FLAG_HIDDEN);
        }
        if (layer_g) {
            track_set_layer_color(layer_g, color);
            if (valid && ist_gerade) lv_obj_clear_flag(layer_g, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_g, LV_OBJ_FLAG_HIDDEN);
        }
        if (layer_warn) {
            if (warn) lv_obj_clear_flag(layer_warn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_warn, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static int track_entry_allowed_state(const hmi_state_t *s, uint8_t from, uint8_t to)
{
    if (!s || !s->mega2_online) return -1;

    // ETH/WebUI entryAllowed ist 0-basiert abgelegt:
    // entryAllowed[2] Bit 3 = 3->4, entryAllowed[3] Bit 0 = 4->1 usw.
    if (s->mega2_entry_allowed_valid && from >= 1u && from <= 9u && to >= 1u && to <= 9u) {
        const uint8_t row = (uint8_t)(from - 1u);
        const uint8_t bit = (uint8_t)(to - 1u);
        return (s->mega2_entry_allowed[row] & (1u << bit)) ? 1 : 0;
    }

    // Fallback fuer HMI-state-lite, falls entryAllowed nicht mitgesendet wird.
    // Die bekannten SignalGrantMask-Bits bleiben so weiter nutzbar.
    if (!s->mega2_signal_grant_valid) return -1;
    if (from == 3u && to == 4u) return (s->mega2_signal_grant_mask & (1u << 2)) ? 1 : 0;
    if (from == 6u && to == 4u) return (s->mega2_signal_grant_mask & (1u << 11)) ? 1 : 0;
    if (from == 4u && to == 5u) return (s->mega2_signal_grant_mask & (1u << 4)) ? 1 : 0;
    if (from == 4u && to == 1u) return 0;
    return -1;
}

static void update_track_e0_grants_ui(const hmi_state_t *s)
{
    if (!s || !track_layer_e0_created) return;

    track_set_grant_signal(track_e0_signal_red[0], track_e0_signal_green[0], track_entry_allowed_state(s, 3, 4));
    track_set_grant_signal(track_e0_signal_red[1], track_e0_signal_green[1], track_entry_allowed_state(s, 6, 4));
    track_set_grant_signal(track_e0_signal_red[2], track_e0_signal_green[2], track_entry_allowed_state(s, 4, 1));
    track_set_grant_signal(track_e0_signal_red[3], track_e0_signal_green[3], track_entry_allowed_state(s, 4, 5));

    for (uint8_t i = 0; i < 2u; ++i) {
        int state = -1;
        if (s->mega1_online && s->mega1_bahnhof_valid) {
            state = (s->mega1_bahnhof_mask & (1u << i)) ? 1 : 0;
        }
        track_set_grant_signal(track_e0_signal_red[i + 4], track_e0_signal_green[i + 4], state);
    }
}

static bool track_mega2_defect_text_mentions_turnout(const hmi_state_t *s, uint8_t id)
{
    if (!s || s->mega2_defects[0] == '\0') return false;
    switch (id) {
        case 12: return strstr(s->mega2_defects, "W12") != NULL;
        case 13: return strstr(s->mega2_defects, "W13") != NULL;
        case 14: return strstr(s->mega2_defects, "W14") != NULL;
        case 15: return strstr(s->mega2_defects, "W15") != NULL;
        default: return false;
    }
}

static const char *track_sbhf_state_name(uint8_t state, uint8_t gleis)
{
    switch (state) {
        case 0: return "Leerlauf";
        case 1: return "Vorbereitung SBHF";
        case 2: return "Stellen der Weichen";
        case 3: return "Warten Ausfahrt B6";
        case 4:
            if (gleis == 1) return "Ausfahrt aus G1";
            if (gleis == 2) return "Ausfahrt aus G2";
            if (gleis == 3) return "Ausfahrt aus G3";
            return "Ausfahrt";
        case 5: return "Warten Einfahrt";
        case 6:
            if (gleis == 1) return "Einfahrt in G1";
            if (gleis == 2) return "Einfahrt in G2";
            if (gleis == 3) return "Einfahrt in G3";
            return "Einfahrt";
        case 7: return "Fehler";
        default: return "State ?";
    }
}

static void track_sbhf_occupied_text(char *out, size_t out_len, const hmi_state_t *s)
{
    if (!out || out_len == 0) return;
    out[0] = '\0';
    if (!s) {
        snprintf(out, out_len, "-");
        return;
    }
    if (s->sbhf_occupied_mask & 0x01u) append_text(out, out_len, "G1");
    if (s->sbhf_occupied_mask & 0x02u) append_text(out, out_len, "G2");
    if (s->sbhf_occupied_mask & 0x04u) append_text(out, out_len, "G3");
    if (out[0] == '\0') snprintf(out, out_len, "-");
}

static void update_track_sbhf_info_ui(const hmi_state_t *s)
{
    if (!s || !track_sbhf_info_label) return;

    char occupied[24];
    track_sbhf_occupied_text(occupied, sizeof(occupied), s);

    char ausfahr[8];
    if (s->mega2_sbhf_current_gleis >= 1u && s->mega2_sbhf_current_gleis <= 3u) {
        snprintf(ausfahr, sizeof(ausfahr), "G%u", (unsigned)s->mega2_sbhf_current_gleis);
    } else {
        snprintf(ausfahr, sizeof(ausfahr), "-");
    }

    char buf[192];
    snprintf(buf, sizeof(buf),
             "State: %s\n"
             "Ausfahr-Gleis: %s\n"
             "Belegt: %s\n"
             "Erlaubt: %s\n"
             "Restricted: %s\n"
             "Start: %s",
             s->mega2_online ? track_sbhf_state_name(s->mega2_sbhf_state, s->mega2_sbhf_current_gleis) : "-",
             s->mega2_online ? ausfahr : "-",
             s->mega2_online ? occupied : "-",
             (s->sbhf_allowed_text[0] != '\0') ? s->sbhf_allowed_text : "-",
             s->sbhf_restricted ? "ja" : "nein",
             s->mega2_online ? (s->sbhf_start_pending ? "SBHF-Start vorgemerkt" : "-") : "-");
    ui_label_set_text_if_changed(track_sbhf_info_label, buf);
}


static void update_track_sbhf_occ_ui(const hmi_state_t *s)
{
    if (!s || !track_layer_sbhf_created) return;

    static const uint8_t occ_bits[6] = {3, 4, 5, 6, 7, 8};
    const bool occ_valid = s->mega2_online && s->mega2_block_occ_valid;
    for (uint8_t i = 0; i < 6; ++i) {
        lv_obj_t *layer = track_sbhf_block_occ_layer[i];
        if (!layer) continue;
        const bool occupied = occ_valid && ((s->mega2_block_occ_mask & (1u << occ_bits[i])) != 0u);
        if (occupied) lv_obj_clear_flag(layer, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(layer, LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_track_sbhf_turnouts_ui(const hmi_state_t *s)
{
    if (!s || !track_layer_sbhf_created) return;

    const bool valid = s->mega2_online;
    for (uint8_t i = 0; i < 4; ++i) {
        const uint8_t id = (uint8_t)(12u + i);
        const bool ist_gerade = ((s->mega2_turnout_ist_mask & (1u << i)) == 0u);
        const bool soll_gerade = ((s->mega2_turnout_soll_mask & (1u << i)) == 0u);
        const bool mismatch = (ist_gerade != soll_gerade);
        const uint32_t color = mismatch ? 0xF25F4C : 0x2ECC71;
        const bool warn = valid && track_mega2_defect_text_mentions_turnout(s, id);

        lv_obj_t *layer_a = track_sbhf_turnout_layer[i][0];
        lv_obj_t *layer_g = track_sbhf_turnout_layer[i][1];
        lv_obj_t *layer_warn = track_sbhf_turnout_warn_layer[i];

        if (layer_a) {
            track_set_layer_color(layer_a, color);
            if (valid && !ist_gerade) lv_obj_clear_flag(layer_a, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_a, LV_OBJ_FLAG_HIDDEN);
        }
        if (layer_g) {
            track_set_layer_color(layer_g, color);
            if (valid && ist_gerade) lv_obj_clear_flag(layer_g, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_g, LV_OBJ_FLAG_HIDDEN);
        }
        if (layer_warn) {
            if (warn) lv_obj_clear_flag(layer_warn, LV_OBJ_FLAG_HIDDEN);
            else lv_obj_add_flag(layer_warn, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static void update_track_sbhf_target_ui(const hmi_state_t *s)
{
    if (!s || !track_layer_sbhf_created) return;

    const bool valid = s->mega2_online && (s->mega2_sbhf_current_gleis >= 1u) && (s->mega2_sbhf_current_gleis <= 3u);
    for (uint8_t i = 0; i < 3; ++i) {
        lv_obj_t *layer = track_sbhf_target_layer[i];
        if (!layer) continue;
        if (valid && s->mega2_sbhf_current_gleis == (uint8_t)(i + 1u)) lv_obj_clear_flag(layer, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(layer, LV_OBJ_FLAG_HIDDEN);
    }
}

static void update_track_sbhf_grants_ui(const hmi_state_t *s)
{
    if (!s || !track_layer_sbhf_created) return;

    static const uint8_t display_idx[6] = {4, 5, 3, 6, 7, 8};
    for (uint8_t i = 0; i < 6; ++i) {
        track_set_grant_signal(track_sbhf_signal_red[i], track_sbhf_signal_green[i],
                               left_grant_display_state(s, display_idx[i]));
    }
}

static void update_track_active_ui(const hmi_state_t *s)
{
    if (!s) return;
    if (track_active_level == TRACK_LEVEL_E1) {
        update_track_e1_occ_ui(s);
        update_track_e1_turnouts_ui(s);
        update_track_e1_grants_ui(s);
    } else if (track_active_level == TRACK_LEVEL_E0) {
        update_track_e0_occ_ui(s);
        update_track_e0_turnouts_ui(s);
        update_track_e0_grants_ui(s);
    } else if (track_active_level == TRACK_LEVEL_SBHF) {
        update_track_sbhf_occ_ui(s);
        update_track_sbhf_turnouts_ui(s);
        update_track_sbhf_target_ui(s);
        update_track_sbhf_grants_ui(s);
        update_track_sbhf_info_ui(s);
    }
}


static void update_left_panel_ui(const hmi_state_t *s)
{
    if (!s) return;

    const bool weichen_cmd_enabled = snapshot_can_send_m1_weiche_cmd(s);
    const bool bhf_cmd_enabled = snapshot_can_send_bhf_cmd(s);

    if (left_weichen_cmd_status_label) {
        char buf[80];
        if (weichen_cmd_enabled) snprintf(buf, sizeof(buf), "Bedienung frei");
        else snprintf(buf, sizeof(buf), "Bedienung gesperrt: %s", snapshot_left_command_block_reason(s, false));
        ui_label_set_text_if_changed(left_weichen_cmd_status_label, buf);
        lv_obj_set_style_text_color(left_weichen_cmd_status_label,
            lv_color_hex(weichen_cmd_enabled ? 0x2ECC71 : 0xF9D342), 0);
    }
    if (left_station_cmd_status_label) {
        char buf[80];
        if (bhf_cmd_enabled) snprintf(buf, sizeof(buf), "Bedienung frei");
        else snprintf(buf, sizeof(buf), "Bedienung gesperrt: %s", snapshot_left_command_block_reason(s, true));
        ui_label_set_text_if_changed(left_station_cmd_status_label, buf);
        lv_obj_set_style_text_color(left_station_cmd_status_label,
            lv_color_hex(bhf_cmd_enabled ? 0x2ECC71 : 0xF9D342), 0);
    }

    for (int i = 0; i < 12; ++i) {
        if (!left_m1_turnout_btns[i] || !left_m1_turnout_labels[i]) continue;
        const bool valid = s->mega1_online;
        const bool ist_gerade = ((s->mega1_weiche_ist_bits & (1u << i)) != 0u);
        const bool soll_gerade = ((s->mega1_weiche_soll_bits & (1u << i)) != 0u);
        const bool mismatch = valid && (ist_gerade != soll_gerade);
        const uint32_t bg = !valid ? 0x53616A : (mismatch ? 0xA33939 : (weichen_cmd_enabled ? 0x1F5361 : 0x53616A));
        lv_obj_set_style_bg_color(left_m1_turnout_btns[i], lv_color_hex(bg), 0);
        left_set_clickable_feedback(left_m1_turnout_btns[i], weichen_cmd_enabled);
        char label[56];
        format_turnout_label(label, sizeof(label), (unsigned)i, valid, ist_gerade, soll_gerade);
        if (!weichen_cmd_enabled) {
            size_t used = strlen(label);
            snprintf(label + used, sizeof(label) - used, "\ngesperrt");
        }
        ui_label_set_text_if_changed(left_m1_turnout_labels[i], label);
        lv_obj_set_style_text_opa(left_m1_turnout_labels[i], LV_OPA_COVER, 0);
    }


    for (int i = 0; i < 4; ++i) {
        if (!left_sbhf_turnout_cards[i] || !left_sbhf_turnout_labels[i]) continue;
        const bool valid = s->mega2_online;
        const bool ist_gerade = ((s->mega2_turnout_ist_mask & (1u << i)) == 0u);
        const bool soll_gerade = ((s->mega2_turnout_soll_mask & (1u << i)) == 0u);
        const bool mismatch = valid && (ist_gerade != soll_gerade);
        const uint32_t bg = !valid ? 0x53616A : (mismatch ? 0xA33939 : 0x1F5361);
        lv_obj_set_style_bg_color(left_sbhf_turnout_cards[i], lv_color_hex(bg), 0);
        char label[40];
        format_turnout_label(label, sizeof(label), (unsigned)(12 + i), valid, ist_gerade, soll_gerade);
        ui_label_set_text_if_changed(left_sbhf_turnout_labels[i], label);
    }

    for (int i = 0; i < 4; ++i) {
        const bool valid = s->mega1_online && s->mega1_bahnhof_valid;
        const int state = !valid ? -1 : ((s->mega1_bahnhof_mask & (1u << i)) ? 1 : 0);
        left_set_dual_led(left_station_led_red[i], left_station_led_green[i], state);

        if (left_station_cards[i]) {
            const uint32_t card_bg = !valid ? 0x53616A : (bhf_cmd_enabled ? 0x20333C : 0x53616A);
            lv_obj_set_style_bg_color(left_station_cards[i], lv_color_hex(card_bg), 0);
            left_set_clickable_feedback(left_station_cards[i], bhf_cmd_enabled);
        }
        if (left_station_labels[i]) {
            char label[56];
            snprintf(label, sizeof(label), "Bhf%d\n%s%s", i,
                     !valid ? "--" : (state == 1 ? "Ausfahrt frei" : "Ausfahrt stopp"),
                     bhf_cmd_enabled ? "" : "\ngesperrt");
            ui_label_set_text_if_changed(left_station_labels[i], label);
            lv_obj_set_style_text_opa(left_station_labels[i], LV_OPA_COVER, 0);
        }
    }

    for (int i = 0; i < 9; ++i) {
        const bool occ_valid = s->mega2_online && s->mega2_block_occ_valid;
        const uint8_t occ_bit = left_block_occ_display_to_mask_bit((uint8_t)i);
        const int occ_state = !occ_valid ? -1 : ((s->mega2_block_occ_mask & (1u << occ_bit)) ? 0 : 1);
        left_set_dual_led(left_block_occ_red[i], left_block_occ_green[i], occ_state);

        left_set_dual_led(left_block_grant_red[i], left_block_grant_green[i], left_grant_display_state(s, (uint8_t)i));
    }

    update_track_active_ui(s);

    if (left_debug_summary_label) {
        char buf[160];
        snprintf(buf, sizeof(buf), "RX %lu | ACK %lu | Typ %s | Dirty-Refresh aktiv",
                 (unsigned long)s->rx_frames,
                 (unsigned long)s->ack_sent,
                 s->last_type[0] ? s->last_type : "-");
        ui_label_set_text_if_changed(left_debug_summary_label, buf);
    }
}

static void create_hmi_screen(void)
{
    lv_obj_t *screen = lv_scr_act();
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x141D24), 0);

    const int right_w = 300;
    const int gap = 8;
    const int left_w = 1024 - right_w - (3 * gap);

    lv_obj_t *left_panel = lv_obj_create(screen);
    lv_obj_set_size(left_panel, left_w, 586);
    lv_obj_align(left_panel, LV_ALIGN_LEFT_MID, gap, 0);
    ui_card_style(left_panel, 0x24313A);

    create_left_panel_tabs(left_panel, left_w);

    lv_obj_t *right_panel = lv_obj_create(screen);
    lv_obj_set_size(right_panel, right_w, 586);
    lv_obj_align(right_panel, LV_ALIGN_RIGHT_MID, -gap, 0);
    ui_card_style(right_panel, 0x1B2B34);
    lv_obj_add_flag(right_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(right_panel, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(right_panel, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_bottom(right_panel, 56, 0);

    ui_section_title(right_panel, "Aktionen", 12, 8);
    power_led = ui_make_led(right_panel, 12, 49);
    power_on_btn = ui_make_button(right_panel, "Power On", 34, 31, 236, 42, 0x19B65A, 0xFFFFFF);
    lv_obj_add_event_cb(power_on_btn, on_power_on_clicked, LV_EVENT_CLICKED, NULL);
    power_off_btn = ui_make_button(right_panel, "Power Off", 34, 80, 236, 42, 0xC83A32, 0xFFFFFF);
    lv_obj_add_event_cb(power_off_btn, on_power_off_clicked, LV_EVENT_CLICKED, NULL);
    auto_led = ui_make_led(right_panel, 12, 147);
    auto_btn = ui_make_button(right_panel, "Auto", 34, 129, 236, 42, 0x00BCE3, 0xFFFFFF);
    lv_obj_add_event_cb(auto_btn, on_auto_clicked, LV_EVENT_CLICKED, NULL);

    lv_obj_t *trafo_card = lv_obj_create(right_panel);
    lv_obj_set_size(trafo_card, 276, 58);
    lv_obj_align(trafo_card, LV_ALIGN_TOP_LEFT, 8, 181);
    ui_card_style(trafo_card, 0x20333C);
    trafo_label = lv_label_create(trafo_card);
    lv_label_set_text(trafo_label, "Trafo oben: 0.00 V\nTrafo unten: 0.00 V");
    lv_label_set_long_mode(trafo_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(trafo_label, 248);
    ui_label_style(trafo_label, &lv_font_montserrat_14, 0xFFFFFF);
    lv_obj_align(trafo_label, LV_ALIGN_LEFT_MID, 8, 0);

    lv_obj_t *write_card = lv_obj_create(right_panel);
    lv_obj_set_size(write_card, 276, 64);
    lv_obj_align(write_card, LV_ALIGN_TOP_LEFT, 8, 247);
    ui_card_style(write_card, 0x20333C);
    ui_section_title(write_card, "Schreibrechte", 8, 4);
    write_label = lv_label_create(write_card);
    lv_label_set_text(write_label, "Bedienung frei");
    ui_label_style(write_label, &lv_font_montserrat_16, 0xF9D342);
    lv_obj_align(write_label, LV_ALIGN_TOP_LEFT, 8, 30);

    lv_obj_t *system_card = lv_obj_create(right_panel);
    lv_obj_set_size(system_card, 276, 266);
    lv_obj_align(system_card, LV_ALIGN_TOP_LEFT, 8, 317);
    ui_card_style(system_card, 0x20333C);
    ui_section_title(system_card, "Systemstatus", 8, 4);
    eth_value_label = ui_make_status_value(system_card, "ETH", 30);
    mega1_value_label = ui_make_status_value(system_card, "Mega1", 58);
    mega2_value_label = ui_make_status_value(system_card, "Mega2", 86);
    safety_value_label = ui_make_status_value(system_card, "Safety", 114);
    warning_value_label = ui_make_status_value(system_card, "Warning", 142);
    power_value_label = ui_make_status_value(system_card, "Power", 170);
    mode_value_label = ui_make_status_value(system_card, "Modus", 198);
    ws_diag_value_label = ui_make_status_value(system_card, "WS/Diag", 226);

    lv_obj_t *fault_card = lv_obj_create(right_panel);
    fault_card_obj = fault_card;
    lv_obj_set_width(fault_card, 276);
    lv_obj_set_height(fault_card, LV_SIZE_CONTENT);
    lv_obj_align(fault_card, LV_ALIGN_TOP_LEFT, 8, 591);
    ui_card_style(fault_card, 0x20333C);
    lv_obj_set_layout(fault_card, LV_LAYOUT_FLEX);
    lv_obj_set_flex_flow(fault_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(fault_card, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
    lv_obj_set_style_pad_row(fault_card, 7, 0);
    lv_obj_set_style_pad_bottom(fault_card, 10, 0);
    lv_obj_t *fault_title = ui_section_title(fault_card, "Defekte", 0, 0);
    lv_obj_set_width(fault_title, 248);

    fault_m1_label = lv_label_create(fault_card);
    lv_label_set_text(fault_m1_label, "Mega1: keine Defekte");
    lv_label_set_long_mode(fault_m1_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(fault_m1_label, 248);
    ui_label_style(fault_m1_label, &lv_font_montserrat_12, 0xFFFFFF);

    m1_retry_btn = ui_make_button(fault_card, "Mega1 Selftest Retry", 0, 0, 248, 34, 0x53616A, 0xFFFFFF);
    lv_obj_add_event_cb(m1_retry_btn, on_m1_retry_clicked, LV_EVENT_CLICKED, NULL);

    fault_m2_label = lv_label_create(fault_card);
    lv_label_set_text(fault_m2_label, "SBHF: keine Defekte");
    lv_label_set_long_mode(fault_m2_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(fault_m2_label, 248);
    ui_label_style(fault_m2_label, &lv_font_montserrat_12, 0xFFFFFF);

    m2_retry_btn = ui_make_button(fault_card, "SBHF Selftest Retry", 0, 0, 248, 34, 0x53616A, 0xFFFFFF);
    lv_obj_add_event_cb(m2_retry_btn, on_m2_retry_clicked, LV_EVENT_CLICKED, NULL);

    lv_obj_t *messages_card = lv_obj_create(right_panel);
    messages_card_obj = messages_card;
    lv_obj_set_size(messages_card, 276, 152);
    lv_obj_align_to(messages_card, fault_card, LV_ALIGN_OUT_BOTTOM_LEFT, 0, 8);
    ui_card_style(messages_card, 0x20333C);
    ui_section_title(messages_card, "Meldungen", 8, 4);
    
    lv_obj_t *messages_box = lv_obj_create(messages_card);
    lv_obj_set_size(messages_box, 248, 112);
    lv_obj_align(messages_box, LV_ALIGN_TOP_LEFT, 8, 30);
    lv_obj_set_style_bg_opa(messages_box, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(messages_box, 0, 0);
    lv_obj_set_style_pad_all(messages_box, 0, 0);
    lv_obj_set_style_pad_row(messages_box, 4, 0);
    lv_obj_clear_flag(messages_box, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(messages_box, LV_FLEX_FLOW_COLUMN);

    messages_ack_label = lv_label_create(messages_box);
    lv_label_set_text(messages_ack_label, "ACK erforderlich");
    lv_label_set_long_mode(messages_ack_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(messages_ack_label, 248);
    ui_label_style(messages_ack_label, &lv_font_montserrat_12, 0xF9D342);
    lv_obj_add_flag(messages_ack_label, LV_OBJ_FLAG_HIDDEN);

    messages_m1_label = lv_label_create(messages_box);
    lv_label_set_text(messages_m1_label, "Mega1: Eine oder mehrere Weichen schalten nicht in die Sollstellung.");
    lv_label_set_long_mode(messages_m1_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(messages_m1_label, 248);
    ui_label_style(messages_m1_label, &lv_font_montserrat_12, 0xFF4D4D);
    lv_obj_add_flag(messages_m1_label, LV_OBJ_FLAG_HIDDEN);

    messages_sbhf_label = lv_label_create(messages_box);
    lv_label_set_text(messages_sbhf_label, "SBHF: Eine oder mehrere Weichen melden Stoerung/Defekt.");
    lv_label_set_long_mode(messages_sbhf_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(messages_sbhf_label, 248);
    ui_label_style(messages_sbhf_label, &lv_font_montserrat_12, 0xFF4D4D);
    lv_obj_add_flag(messages_sbhf_label, LV_OBJ_FLAG_HIDDEN);

    messages_mode_label = lv_label_create(messages_box);
    lv_label_set_text(messages_mode_label, "SBHF: normaler Betrieb");
    lv_label_set_long_mode(messages_mode_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(messages_mode_label, 248);
    ui_label_style(messages_mode_label, &lv_font_montserrat_12, 0x2ECC71);

    messages_allowed_label = lv_label_create(messages_box);
    lv_label_set_text(messages_allowed_label, "SBHF erlaubte Gleise: -");
    lv_label_set_long_mode(messages_allowed_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(messages_allowed_label, 248);
    ui_label_style(messages_allowed_label, &lv_font_montserrat_12, 0xDDDDDD);

    screen_off_wake_overlay = lv_obj_create(screen);
    lv_obj_set_size(screen_off_wake_overlay, 1024, 600);
    lv_obj_align(screen_off_wake_overlay, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(screen_off_wake_overlay, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(screen_off_wake_overlay, 0, 0);
    lv_obj_clear_flag(screen_off_wake_overlay, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(screen_off_wake_overlay, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(screen_off_wake_overlay, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(screen_off_wake_overlay, screen_off_wake_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(screen_off_wake_overlay, screen_off_wake_event_cb, LV_EVENT_CLICKED, NULL);

    // Nach dem Aufbau aller sichtbaren UI-Objekte Aktivitaetserkennung
    // registrieren. Damit setzt jede Beruehrung den Screen-Off-Timer zurueck.
    display_attach_activity_recursive(screen);

    create_overlay_ui(screen);

    g_last_user_activity_ms = lv_tick_get();
    lv_timer_create(uart_ui_timer_cb, 250, NULL);
    display_power_timer = lv_timer_create(display_power_timer_cb, 500, NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Railway HMI P4 state-cache bring-up");

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);
    display_settings_load();

    memset(&g_state, 0, sizeof(g_state));
    g_state_mutex = xSemaphoreCreateMutex();
    g_uart_tx_mutex = xSemaphoreCreateMutex();

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
    (void)bsp_display_brightness_init();
    display_apply_brightness();
    g_touch_indev = bsp_display_get_input_dev();

    if (disp != NULL) {
        bsp_display_rotate(disp, LV_DISPLAY_ROTATION_180);
    }

    bsp_display_lock(0);
    create_hmi_screen();
    bsp_display_unlock();

    hmi_uart_init();

    ESP_LOGI(TAG, "Minimal HMI screen created, UART parser and state cache active");
}
