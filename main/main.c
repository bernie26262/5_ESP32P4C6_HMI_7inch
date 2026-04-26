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
    bool can_write;
    bool diag_active;
    uint32_t ws_base_clients;
    uint32_t ws_diag_clients;
    uint32_t sbhf_allowed_mask;
    bool sbhf_restricted;
    char sbhf_allowed_text[32];

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

static lv_obj_t *touch_label = NULL;
static lv_obj_t *uart_status_label = NULL;
static lv_obj_t *comm_status_label = NULL;
static lv_obj_t *frame_status_label = NULL;

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
static lv_obj_t *messages_label = NULL;

static lv_obj_t *eth_value_label = NULL;
static lv_obj_t *mega1_value_label = NULL;
static lv_obj_t *mega2_value_label = NULL;
static lv_obj_t *safety_value_label = NULL;
static lv_obj_t *warning_value_label = NULL;
static lv_obj_t *power_value_label = NULL;
static lv_obj_t *mode_value_label = NULL;
static lv_obj_t *ws_diag_value_label = NULL;

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
            const uint32_t mega1_warning_mask = json_get_u32_path(root, "mega1", "warningMask", 0);
            const uint32_t mega2_warning_mask = json_get_u32_path(root, "mega2", "warningMask", 0);
            const uint32_t sbhf_warning_mask = json_get_u32_path3(root, "mega2", "sbhf", "warningMask", 0);
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
            g_state.sbhf_restricted = json_get_bool_path3(root, "mega2", "sbhf", "restricted", g_state.sbhf_restricted);

            json_get_string_path(root, "mega1", "defectList", g_state.mega1_defects, sizeof(g_state.mega1_defects));
            json_get_string_path(root, "mega2", "defectList", g_state.mega2_defects, sizeof(g_state.mega2_defects));

            if (g_state.mega1_defects[0] == '\0') {
                const uint32_t m1_selftest_fail_mask = json_get_u32_path3(root, "mega1", "diag", "selftestFailMask", 0);
                build_turnout_defects(g_state.mega1_defects, sizeof(g_state.mega1_defects), m1_selftest_fail_mask, 0, 12, "");
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

    if (messages_label) {
        const bool has_m1_defect = s.mega1_defects[0] != '\0';
        const bool has_m2_defect = s.mega2_defects[0] != '\0';
        const bool has_message = s.ack_required || s.warning_present || has_m1_defect || has_m2_defect || s.sbhf_restricted;

        char msg_buf[320];
        snprintf(msg_buf, sizeof(msg_buf),
            "%s%s%s%s%sSBHF erlaubte Gleise: %s",
            s.ack_required ? "ACK erforderlich\n" : "",
            s.sbhf_restricted ? "SBHF: Restricted Mode aktiv\n" : "",
            has_m1_defect ? "Mega1: Eine oder mehrere Weichen schalten nicht in die Sollstellung.\n" : "",
            has_m2_defect ? "SBHF: Eine oder mehrere Weichen melden Stoerung/Defekt.\n" : "",
            has_message ? "" : "Keine Meldungen\n",
            s.sbhf_allowed_text[0] ? s.sbhf_allowed_text : "-"
        );
        ui_label_set_text_if_changed(messages_label, msg_buf);
        lv_obj_set_style_text_color(messages_label,
            has_message ? lv_color_hex(0xF9D342) : lv_color_hex(0x2ECC71), 0);
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

    lv_obj_t *title = lv_label_create(left_panel);
    lv_label_set_text(title, "Elektrische Eisenbahn HMI");
    ui_label_style(title, &lv_font_montserrat_26, 0xFFFFFF);
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 14, 8);

    lv_obj_t *tabview = lv_tabview_create(left_panel, LV_DIR_TOP, 45);
    lv_obj_set_size(tabview, left_w - 18, 525);
    lv_obj_align(tabview, LV_ALIGN_BOTTOM_MID, 0, -2);
    lv_obj_set_style_bg_color(tabview, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_text_color(tabview, lv_color_hex(0xFFFFFF), 0);

    lv_obj_t *tab_weichen = lv_tabview_add_tab(tabview, "Weichen");
    lv_obj_t *tab_bahnhoefe = lv_tabview_add_tab(tabview, "Bahnhoefe");
    lv_obj_t *tab_bloecke = lv_tabview_add_tab(tabview, "Bloecke");
    lv_obj_t *tab_debug = lv_tabview_add_tab(tabview, "Debug");

    lv_obj_set_style_bg_color(tab_weichen, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_bg_color(tab_bahnhoefe, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_bg_color(tab_bloecke, lv_color_hex(0x2B3942), 0);
    lv_obj_set_style_bg_color(tab_debug, lv_color_hex(0xF3EFE3), 0);

    lv_obj_t *m1_title = lv_label_create(tab_weichen);
    lv_label_set_text(m1_title, "Mega1-Weichen");
    ui_label_style(m1_title, &lv_font_montserrat_18, 0xFFFFFF);
    lv_obj_align(m1_title, LV_ALIGN_TOP_LEFT, 8, 8);

    lv_obj_t *m1_state = lv_label_create(tab_weichen);
    lv_label_set_text(m1_state,
                      "W0: -/-   W1: -/-   W2: -/-   W3: -/-\n"
                      "W4: -/-   W5: -/-   W6: -/-   W7: -/-\n"
                      "W8: -/-   W9: -/-   W10: -/-  W11: -/-");
    ui_label_style(m1_state, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(m1_state, LV_ALIGN_TOP_LEFT, 8, 38);

    const char *btn_names[] = {"W0", "W1", "W2", "W3", "W4", "W5", "W6", "W7", "W8", "W9", "W10", "W11"};
    for (int i = 0; i < 12; ++i) {
        int col = i % 4;
        int row = i / 4;
        ui_make_button(tab_weichen, btn_names[i], 70 + col * 110, 120 + row * 62, 90, 48, 0x00BCE3, 0xFFFFFF);
    }

    lv_obj_t *sbhf_panel = lv_obj_create(tab_weichen);
    lv_obj_set_size(sbhf_panel, left_w - 64, 135);
    lv_obj_align(sbhf_panel, LV_ALIGN_BOTTOM_LEFT, 8, -12);
    ui_card_style(sbhf_panel, 0x2D3C45);

    lv_obj_t *sbhf_title = lv_label_create(sbhf_panel);
    lv_label_set_text(sbhf_title, "SBHF-Weichen");
    ui_label_style(sbhf_title, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(sbhf_title, LV_ALIGN_TOP_LEFT, 10, 8);

    lv_obj_t *sbhf_state = lv_label_create(sbhf_panel);
    lv_label_set_text(sbhf_state, "W12: -/-   W13: -/-\nW14: -/-   W15: -/-");
    ui_label_style(sbhf_state, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(sbhf_state, LV_ALIGN_TOP_LEFT, 10, 38);

    lv_obj_t *bahnhoefe_label = lv_label_create(tab_bahnhoefe);
    lv_label_set_text(bahnhoefe_label, "Bahnhoefe-Ansicht wird vorbereitet.");
    ui_label_style(bahnhoefe_label, &lv_font_montserrat_22, 0xFFFFFF);
    lv_obj_align(bahnhoefe_label, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t *bloecke_label = lv_label_create(tab_bloecke);
    lv_label_set_text(bloecke_label, "Bloecke-Ansicht wird vorbereitet.");
    ui_label_style(bloecke_label, &lv_font_montserrat_22, 0xFFFFFF);
    lv_obj_align(bloecke_label, LV_ALIGN_TOP_LEFT, 10, 10);

    lv_obj_t *debug_title = lv_label_create(tab_debug);
    lv_label_set_text(debug_title, "Kommunikation");
    ui_label_style(debug_title, &lv_font_montserrat_24, 0x333333);
    lv_obj_align(debug_title, LV_ALIGN_TOP_LEFT, 12, 10);

    uart_status_label = lv_label_create(tab_debug);
    lv_label_set_text(uart_status_label, "UART RX: waiting...");
    lv_label_set_long_mode(uart_status_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(uart_status_label, left_w - 65);
    ui_label_style(uart_status_label, &lv_font_montserrat_16, 0x333333);
    lv_obj_align(uart_status_label, LV_ALIGN_TOP_LEFT, 12, 52);

    lv_obj_t *frame_panel = lv_obj_create(tab_debug);
    lv_obj_set_size(frame_panel, 255, 142);
    lv_obj_align(frame_panel, LV_ALIGN_TOP_LEFT, 12, 122);
    ui_card_style(frame_panel, 0x17313A);

    lv_obj_t *frame_title = lv_label_create(frame_panel);
    lv_label_set_text(frame_title, "Frame-Typen");
    ui_label_style(frame_title, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(frame_title, LV_ALIGN_TOP_LEFT, 12, 8);

    comm_status_label = lv_label_create(frame_panel);
    lv_label_set_text(comm_status_label, "state-lite: 0\nanalog:     0\nother:      0");
    ui_label_style(comm_status_label, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_align(comm_status_label, LV_ALIGN_TOP_LEFT, 12, 42);

    lv_obj_t *touch_panel = lv_obj_create(tab_debug);
    lv_obj_set_size(touch_panel, 255, 95);
    lv_obj_align(touch_panel, LV_ALIGN_TOP_LEFT, 285, 122);
    ui_card_style(touch_panel, 0x17313A);
    lv_obj_add_event_cb(touch_panel, touch_event_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_add_event_cb(touch_panel, touch_event_cb, LV_EVENT_PRESSING, NULL);

    touch_label = lv_label_create(touch_panel);
    lv_label_set_text(touch_label, "Touch: -");
    ui_label_style(touch_label, &lv_font_montserrat_16, 0xFFFFFF);
    lv_obj_center(touch_label);

    frame_status_label = lv_label_create(tab_debug);
    lv_label_set_text(frame_status_label,
                      "UART-Framing, JSON-Parser, State-Cache und ACK-Pfad laufen.\n"
                      "Roh-JSON wird nicht mehr angezeigt.");
    lv_label_set_long_mode(frame_status_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(frame_status_label, left_w - 65);
    ui_label_style(frame_status_label, &lv_font_montserrat_18, 0x333333);
    lv_obj_align(frame_status_label, LV_ALIGN_TOP_LEFT, 12, 300);

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
    messages_label = lv_label_create(messages_card);
    lv_label_set_text(messages_label, "Keine Meldungen");
    lv_label_set_long_mode(messages_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(messages_label, 248);
    ui_label_style(messages_label, &lv_font_montserrat_12, 0x2ECC71);
    lv_obj_align(messages_label, LV_ALIGN_TOP_LEFT, 8, 30);

    create_overlay_ui(screen);

    lv_timer_create(uart_ui_timer_cb, 250, NULL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting Railway HMI P4 state-cache bring-up");

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