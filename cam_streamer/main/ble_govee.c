#include "ble_govee.h"
#include "app_config.h"
#include "net_services.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_coexist.h"
#include "cJSON.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/util/util.h"

static const char *TAG = "ble_govee";

static govee_env_state_t s_govee = {
    .valid = false,
    .mac_str = "--:--:--:--:--:--",
    .model_str = "Đang dò BLE...",
    .ble_name = "",
    .temp_c = 0.0f,
    .humidity_pct = 0.0f,
    .battery_pct = 0,
    .rssi = -100,
    .last_seen_uptime_sec = 0,
    .rx_packet_cnt = 0,
    .ble_scanning = false,
    .cycle_sec = 0,
    .post_ok_cnt = 0,
    .post_err_cnt = 0,
    .last_http_code = 0,
    .last_post_uptime_sec = 0,
    .last_posted_temp_c = -999.0f,
    .last_auto_cmd = "",
    .last_post_status = "Waiting for Govee BLE"
};

static SemaphoreHandle_t s_govee_mutex = NULL;
static bool s_nimble_synced = false;
static bool s_nimble_started = false;
static volatile bool s_urgent_delta_trigger = false;

static inline bool is_valid_govee_reading(float temp_c, float hum_pct, uint8_t bat_pct) {
    return (temp_c >= 10.0f && temp_c <= 45.0f &&
            hum_pct >= 10.0f && hum_pct <= 99.0f &&
            bat_pct <= 100);
}

static bool decode_ec88_3byte(const uint8_t *b, float *out_t, float *out_h, uint8_t *out_bat) {
    uint32_t raw = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | (uint32_t)b[2];
    if (raw == 0 || raw == 0xFFFFFFUL) return false;
    bool is_neg = (raw & 0x800000UL) != 0;
    uint32_t val = raw & 0x7FFFFFUL;
    float t = (float)(val / 1000UL) / 10.0f;
    if (is_neg) t = -t;
    float h = (float)(val % 1000UL) / 10.0f;
    uint8_t bat = b[3] & 0x7F; // Mask charging bit if present
    if (!is_valid_govee_reading(t, h, bat)) return false;
    *out_t = t;
    *out_h = h;
    *out_bat = bat;
    return true;
}

static bool decode_h5074_le(const uint8_t *b, float *out_t, float *out_h, uint8_t *out_bat) {
    int16_t raw_t = (int16_t)((uint16_t)b[1] | ((uint16_t)b[2] << 8));
    uint16_t raw_h = (uint16_t)b[3] | ((uint16_t)b[4] << 8);
    float t = (float)raw_t / 100.0f;
    float h = (float)raw_h / 100.0f;
    uint8_t bat = b[5] & 0x7F;
    if (!is_valid_govee_reading(t, h, bat)) return false;
    *out_t = t;
    *out_h = h;
    *out_bat = bat;
    return true;
}

static bool decode_h5179_le(const uint8_t *b, float *out_t, float *out_h, uint8_t *out_bat) {
    int16_t raw_t = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
    uint16_t raw_h = (uint16_t)b[6] | ((uint16_t)b[7] << 8);
    float t = (float)raw_t / 100.0f;
    float h = (float)raw_h / 100.0f;
    uint8_t bat = b[8] & 0x7F;
    if (!is_valid_govee_reading(t, h, bat)) return false;
    *out_t = t;
    *out_h = h;
    *out_bat = bat;
    return true;
}

static bool parse_govee_mfg_payload(const uint8_t *mfg, size_t mfg_len,
                                    float *out_t, float *out_h, uint8_t *out_bat,
                                    char *out_model, size_t model_len) {
    if (!mfg || mfg_len < 6) return false;

    uint16_t company_id = (uint16_t)mfg[0] | ((uint16_t)mfg[1] << 8);

    // 1. Company ID 0xEC88: Govee H5075 / H5101 / H5102 / H5100 / H5074
    if (company_id == 0xEC88) {
        // Case 1A: Govee H5074 (7-byte Little-Endian payload after 2-byte Company ID -> mfg_len == 9)
        if (mfg_len == 9 && decode_h5074_le(mfg + 2, out_t, out_h, out_bat)) {
            snprintf(out_model, model_len, "Govee H5074");
            return true;
        }
        // Case 1B: Govee H5075 / H5101 / H5102 / H5100 (3-byte combined value at mfg[3..5], battery at mfg[6])
        if (mfg_len >= 7 && decode_ec88_3byte(mfg + 3, out_t, out_h, out_bat)) {
            snprintf(out_model, model_len, "Govee H5075/H510x");
            return true;
        }
        // Case 1C: Compact 6-byte 0xEC88 frame (3-byte combined value at mfg[2..4], battery at mfg[5])
        if (mfg_len == 6 && decode_ec88_3byte(mfg + 2, out_t, out_h, out_bat)) {
            snprintf(out_model, model_len, "Govee H5100");
            return true;
        }
        // Case 1D: Fallback H5074 if 7-byte buffer passed directly or 8-byte truncated
        if (mfg_len >= 8 && decode_h5074_le(mfg + 2, out_t, out_h, out_bat)) {
            snprintf(out_model, model_len, "Govee H5074");
            return true;
        }
        if (mfg_len == 7 && decode_h5074_le(mfg, out_t, out_h, out_bat)) {
            snprintf(out_model, model_len, "Govee H5074");
            return true;
        }
    }

    // 2. Company ID 0x8801: Govee H5179 (length >= 9 bytes)
    if (company_id == 0x8801) {
        // Standard 11-byte frame (2-byte Company ID 0x8801 + 9-byte payload)
        if (mfg_len >= 11 && decode_h5179_le(mfg + 2, out_t, out_h, out_bat)) {
            snprintf(out_model, model_len, "Govee H5179");
            return true;
        }
        // Direct 9-byte frame (offset [4..5] temp, [6..7] hum, [8] bat)
        if (mfg_len >= 9 && decode_h5179_le(mfg, out_t, out_h, out_bat)) {
            snprintf(out_model, model_len, "Govee H5179");
            return true;
        }
    }

    return false;
}

static bool match_target_mac_filter(const char *mac_str) {
    if (g_cfg.baby_target_mac[0] == '\0') return true;
    // Case-insensitive substring or full MAC match
    char filter_up[20] = {0};
    for (size_t i = 0; i < sizeof(filter_up) - 1 && g_cfg.baby_target_mac[i]; i++) {
        char c = g_cfg.baby_target_mac[i];
        if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
        filter_up[i] = c;
    }
    return (strstr(mac_str, filter_up) != NULL);
}

static int govee_ble_gap_event(struct ble_gap_event *event, void *arg) {
    if (event->type != BLE_GAP_EVENT_DISC) {
        return 0;
    }

    const uint8_t *addr = event->disc.addr.val;
    char mac_str[20];
    snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

    if (!match_target_mac_filter(mac_str)) {
        return 0;
    }

    const uint8_t *data = event->disc.data;
    uint8_t data_len = event->disc.length_data;
    if (!data || data_len < 4) return 0;

    char adv_name[32] = {0};
    float temp_c = 0.0f, hum_pct = 0.0f;
    uint8_t bat_pct = 0;
    char model_str[24] = {0};
    bool decoded = false;

    size_t pos = 0;
    while (pos < data_len) {
        uint8_t ad_len = data[pos];
        if (ad_len == 0 || (pos + 1 + ad_len) > data_len) break;
        uint8_t ad_type = data[pos + 1];
        const uint8_t *ad_payload = &data[pos + 2];
        size_t payload_len = ad_len - 1;

        if ((ad_type == 0x08 || ad_type == 0x09) && payload_len > 0) {
            size_t copy_len = (payload_len < sizeof(adv_name) - 1) ? payload_len : (sizeof(adv_name) - 1);
            memcpy(adv_name, ad_payload, copy_len);
            adv_name[copy_len] = '\0';
        } else if (ad_type == 0xFF || ad_type == 0x16) {
            if (parse_govee_mfg_payload(ad_payload, payload_len, &temp_c, &hum_pct, &bat_pct, model_str, sizeof(model_str))) {
                decoded = true;
            }
        }
        pos += (1 + ad_len);
    }

    // Update BLE name if advertised in Scan Response from previously matched MAC
    if (!decoded && adv_name[0] != '\0' && s_govee_mutex) {
        if (xSemaphoreTake(s_govee_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            if (s_govee.valid && strcmp(s_govee.mac_str, mac_str) == 0) {
                strncpy(s_govee.ble_name, adv_name, sizeof(s_govee.ble_name) - 1);
            }
            xSemaphoreGive(s_govee_mutex);
        }
        return 0;
    }

    if (!decoded) return 0;

    uint32_t now_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    if (s_govee_mutex && xSemaphoreTake(s_govee_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        bool first_reading = !s_govee.valid;
        float prev_posted = s_govee.last_posted_temp_c;

        s_govee.valid = true;
        strncpy(s_govee.mac_str, mac_str, sizeof(s_govee.mac_str) - 1);
        strncpy(s_govee.model_str, model_str, sizeof(s_govee.model_str) - 1);
        if (adv_name[0] != '\0') {
            strncpy(s_govee.ble_name, adv_name, sizeof(s_govee.ble_name) - 1);
        }
        s_govee.temp_c = temp_c;
        s_govee.humidity_pct = hum_pct;
        s_govee.battery_pct = bat_pct;
        s_govee.rssi = event->disc.rssi;
        s_govee.last_seen_uptime_sec = now_sec;
        s_govee.rx_packet_cnt++;

        // Immediate push trigger if first valid reading after boot OR temp changed >= 0.3 C
        if (first_reading || prev_posted < -100.0f || fabsf(temp_c - prev_posted) >= 0.3f) {
            s_urgent_delta_trigger = true;
        }
        xSemaphoreGive(s_govee_mutex);
    }

    ESP_LOGI(TAG, "Decoded %s [%s]: Temp=%.1f C, Hum=%.1f%%, Bat=%u%%, RSSI=%d dBm",
             model_str, mac_str, temp_c, hum_pct, (unsigned)bat_pct, (int)event->disc.rssi);
    return 0;
}

static void start_ble_scan_window(void) {
    if (!s_nimble_synced || !g_cfg.baby_mon_enable) return;
    if (ble_gap_disc_active()) return;

    struct ble_gap_disc_params disc_params;
    memset(&disc_params, 0, sizeof(disc_params));
    disc_params.passive = 0;           // Active scan to also capture Govee Scan Response name
    disc_params.itvl = 0x0050;         // 50ms scan interval (gentle on 2.4GHz Wi-Fi coexistence)
    disc_params.window = 0x0020;       // 20ms scan window (~40% radio duty during 10s scan phase)
    disc_params.filter_duplicates = 0; // Receive every advertisement update in the 10s window
    disc_params.limited = 0;

    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &disc_params, govee_ble_gap_event, NULL);
    if (rc == 0 || rc == BLE_HS_EALREADY) {
        if (s_govee_mutex && xSemaphoreTake(s_govee_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_govee.ble_scanning = true;
            xSemaphoreGive(s_govee_mutex);
        }
    } else {
        ESP_LOGW(TAG, "ble_gap_disc start returned rc=%d", rc);
    }
}

static void stop_ble_scan_window(void) {
    if (!s_nimble_synced) return;
    if (ble_gap_disc_active()) {
        ble_gap_disc_cancel();
    }
    if (s_govee_mutex && xSemaphoreTake(s_govee_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_govee.ble_scanning = false;
        xSemaphoreGive(s_govee_mutex);
    }
}

typedef struct {
    char *buf;
    size_t len;
    size_t cap;
} http_resp_accum_t;

static esp_err_t room_env_http_event_handler(esp_http_client_event_t *evt) {
    http_resp_accum_t *acc = (http_resp_accum_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && acc && acc->buf && evt->data_len > 0) {
        size_t avail = (acc->cap - 1 > acc->len) ? (acc->cap - 1 - acc->len) : 0;
        size_t to_copy = ((size_t)evt->data_len < avail) ? (size_t)evt->data_len : avail;
        if (to_copy > 0) {
            memcpy(acc->buf + acc->len, evt->data, to_copy);
            acc->len += to_copy;
            acc->buf[acc->len] = '\0';
        }
    }
    return ESP_OK;
}

esp_err_t app_ble_govee_force_post_now(void) {
    // 1. Ensure BLE scan is stopped before Wi-Fi HTTP POST (2.4GHz antenna anti-collision)
    stop_ble_scan_window();
    vTaskDelay(pdMS_TO_TICKS(120));

    govee_env_state_t snap;
    app_ble_govee_get_state(&snap);
    if (!snap.valid) {
        return ESP_ERR_INVALID_STATE;
    }

    const char *url = g_cfg.baby_server_url[0] ? g_cfg.baby_server_url : "http://10.25.7.116:8686/api/room-env";
    const char *sensor_name = g_cfg.baby_sensor_name[0] ? g_cfg.baby_sensor_name : "Govee Phòng Bé Bon";
    const char *source_dev = g_cfg.baby_source_device[0] ? g_cfg.baby_source_device : "ESP32-CAM";

    char *post_body = (char *)calloc(1, 320);
    char *resp_buf = (char *)calloc(1, 512);
    if (!post_body || !resp_buf) {
        if (post_body) free(post_body);
        if (resp_buf) free(resp_buf);
        return ESP_ERR_NO_MEM;
    }

    // Round to 1 decimal place as required by target API schema
    float temp_1d = roundf(snap.temp_c * 10.0f) / 10.0f;
    float hum_1d = roundf(snap.humidity_pct * 10.0f) / 10.0f;

    snprintf(post_body, 320,
             "{\"sensor_name\":\"%s\",\"source_device\":\"%s\",\"temp_c\":%.1f,\"humidity_pct\":%.1f,\"battery_pct\":%u}",
             sensor_name, source_dev, temp_1d, hum_1d, (unsigned)snap.battery_pct);

    http_resp_accum_t acc = {
        .buf = resp_buf,
        .len = 0,
        .cap = 512
    };

    esp_http_client_config_t cfg = {
        .url = url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
        .event_handler = room_env_http_event_handler,
        .user_data = &acc
    };

    esp_err_t err = ESP_FAIL;
    int status_code = 0;
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (client) {
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, post_body, strlen(post_body));
        err = esp_http_client_perform(client);
        status_code = esp_http_client_get_status_code(client);
        esp_http_client_cleanup(client);
    }

    uint32_t now_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
    char parsed_auto_cmd[32] = {0};
    bool server_ok = false;

    if (err == ESP_OK && status_code >= 200 && status_code < 300) {
        server_ok = true;
        if (resp_buf[0] == '{') {
            cJSON *root = cJSON_Parse(resp_buf);
            if (root) {
                cJSON *cmd_it = cJSON_GetObjectItem(root, "auto_cmd");
                if (cJSON_IsString(cmd_it) && cmd_it->valuestring) {
                    strncpy(parsed_auto_cmd, cmd_it->valuestring, sizeof(parsed_auto_cmd) - 1);
                }
                cJSON_Delete(root);
            }
        }
    }

    if (s_govee_mutex && xSemaphoreTake(s_govee_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_govee.last_http_code = status_code;
        if (server_ok) {
            s_govee.post_ok_cnt++;
            s_govee.last_post_uptime_sec = now_sec;
            s_govee.last_posted_temp_c = temp_1d;
            strncpy(s_govee.last_auto_cmd, parsed_auto_cmd, sizeof(s_govee.last_auto_cmd) - 1);
            if (parsed_auto_cmd[0]) {
                snprintf(s_govee.last_post_status, sizeof(s_govee.last_post_status),
                         "OK (%d) -> %s", status_code, parsed_auto_cmd);
            } else {
                snprintf(s_govee.last_post_status, sizeof(s_govee.last_post_status),
                         "OK (HTTP %d)", status_code);
            }
        } else {
            s_govee.post_err_cnt++;
            snprintf(s_govee.last_post_status, sizeof(s_govee.last_post_status),
                     "ERR (%s / HTTP %d)", esp_err_to_name(err), status_code);
        }
        xSemaphoreGive(s_govee_mutex);
    }

    // Also mirror room environment event to MQTT if connected
    if (g_cfg.mqtt_enable && net_mqtt_is_connected()) {
        char *mqtt_msg = (char *)calloc(1, 384);
        if (mqtt_msg) {
            snprintf(mqtt_msg, 384,
                     "{\"sensor_name\":\"%s\",\"mac\":\"%s\",\"model\":\"%s\",\"temp_c\":%.1f,"
                     "\"humidity_pct\":%.1f,\"battery_pct\":%u,\"rssi\":%d,\"auto_cmd\":\"%s\",\"http_ok\":%s}",
                     sensor_name, snap.mac_str, snap.model_str, temp_1d, hum_1d,
                     (unsigned)snap.battery_pct, (int)snap.rssi, parsed_auto_cmd,
                     server_ok ? "true" : "false");
            net_mqtt_publish_event("baby_env", mqtt_msg);
            free(mqtt_msg);
        }
    }

    ESP_LOGI(TAG, "POST %s -> HTTP %d | Payload: %s | Resp: %s",
             url, status_code, post_body, resp_buf[0] ? resp_buf : "(empty)");

    free(post_body);
    free(resp_buf);
    return server_ok ? ESP_OK : ESP_FAIL;
}

static void govee_coexist_worker_task(void *arg) {
    uint32_t cycle_sec = 0;

    // Wait 4 seconds after boot so Wi-Fi association completes cleanly first
    vTaskDelay(pdMS_TO_TICKS(4000));

    while (1) {
        if (!g_cfg.baby_mon_enable) {
            stop_ble_scan_window();
            vTaskDelay(pdMS_TO_TICKS(2000));
            cycle_sec = 0;
            continue;
        }

        // Coexistence Schedule:
        // 0..9s   (10s): Scan BLE Govee advertisements
        // 10s          : Stop BLE Scan -> HTTP POST via Wi-Fi
        // 10..59s (50s): BLE Radio OFF (100% bandwidth for Wi-Fi Camera Stream & HTTP)
        if (cycle_sec < 10) {
            start_ble_scan_window();
        } else {
            stop_ble_scan_window();
        }

        if (s_govee_mutex && xSemaphoreTake(s_govee_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_govee.cycle_sec = cycle_sec;
            xSemaphoreGive(s_govee_mutex);
        }

        // Check if we need to push immediately due to >= 0.3C delta OR end of 10s BLE scan window
        bool should_post = false;
        if (s_urgent_delta_trigger && cycle_sec >= 2) {
            s_urgent_delta_trigger = false;
            should_post = true;
            ESP_LOGI(TAG, "Urgent temperature delta (>= 0.3 C or initial read) -> stopping BLE scan & pushing HTTP POST immediately!");
            cycle_sec = 10; // Jump directly into the 50s Wi-Fi rest window after pushing
        } else if (cycle_sec == 10) {
            govee_env_state_t st;
            app_ble_govee_get_state(&st);
            uint32_t now_sec = (uint32_t)(esp_timer_get_time() / 1000000ULL);
            // Only post if we have a valid reading received within the last 5 minutes
            if (st.valid && (now_sec - st.last_seen_uptime_sec <= 300)) {
                should_post = true;
            }
        }

        if (should_post) {
            stop_ble_scan_window();
            app_ble_govee_force_post_now();
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        cycle_sec = (cycle_sec + 1) % 60;
    }
}

static void govee_ble_on_sync(void) {
    uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
    ble_hs_id_infer_auto(0, &own_addr_type);
    s_nimble_synced = true;
    ESP_LOGI(TAG, "NimBLE Host synced (Observer Coexistence Mode ready)");
}

static void govee_ble_on_reset(int reason) {
    s_nimble_synced = false;
    ESP_LOGW(TAG, "NimBLE Host reset, reason=%d", reason);
}

static void govee_nimble_host_task(void *param) {
    ESP_LOGI(TAG, "NimBLE Host Task started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_ble_govee_init(bool rescue_mode) {
    if (rescue_mode) {
        ESP_LOGW(TAG, "Rescue Safe Mode active: Skipping NimBLE initialization");
        return;
    }
    if (s_nimble_started) return;

    s_govee_mutex = xSemaphoreCreateMutex();

    // Favor balanced Wi-Fi + BLE coexistence on shared 2.4GHz antenna
    esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);

    esp_err_t ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init NimBLE port: %s", esp_err_to_name(ret));
        snprintf(s_govee.last_post_status, sizeof(s_govee.last_post_status), "NimBLE Init Err");
        return;
    }

    ble_hs_cfg.sync_cb = govee_ble_on_sync;
    ble_hs_cfg.reset_cb = govee_ble_on_reset;

    nimble_port_freertos_init(govee_nimble_host_task);
    s_nimble_started = true;

    xTaskCreatePinnedToCore(govee_coexist_worker_task, "govee_ble", 4096, NULL, 3, NULL, 0);
    ESP_LOGI(TAG, "Govee BLE Baby Monitor initialized (10s Scan / 50s Rest Coexistence active)");
}

void app_ble_govee_get_state(govee_env_state_t *out) {
    if (!out) return;
    if (s_govee_mutex && xSemaphoreTake(s_govee_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        *out = s_govee;
        xSemaphoreGive(s_govee_mutex);
    } else {
        *out = s_govee;
    }
}
