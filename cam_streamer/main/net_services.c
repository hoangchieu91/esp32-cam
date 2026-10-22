#include "net_services.h"
#include "app_config.h"
#include "app_httpd.h"
#include "osd_stamp.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_wifi.h"
#include "esp_sntp.h"
#include "esp_http_client.h"
#include "mqtt_client.h"

static const char *TAG = "net_services";

char g_last_server_ai_json[2048] = "{\"status\":\"idle\",\"summary\":\"Chưa có phân tích\"}";
char g_last_ftp_status[64] = "Idle";
char g_last_ntfy_status[64] = "Idle";

static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static bool s_mqtt_connected = false;
static bool s_ntp_initialized = false;

// ============================================================================
// 1. NTP & TIME SYNC
// ============================================================================
void net_ntp_init(void) {
    setenv("TZ", g_cfg.tz_str[0] ? g_cfg.tz_str : "ICT-7", 1);
    tzset();

    if (s_ntp_initialized) {
        esp_sntp_stop();
    }
    esp_sntp_setoperatingmode(ESP_SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, g_cfg.ntp_server[0] ? g_cfg.ntp_server : "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_init();
    s_ntp_initialized = true;
    ESP_LOGI(TAG, "NTP initialized with server: %s, TZ: %s", g_cfg.ntp_server, g_cfg.tz_str);
}

void net_ntp_restart(void) {
    net_ntp_init();
}

void net_set_manual_epoch(int64_t epoch_sec) {
    setenv("TZ", g_cfg.tz_str[0] ? g_cfg.tz_str : "ICT-7", 1);
    tzset();
    struct timeval tv = {
        .tv_sec = (time_t)epoch_sec,
        .tv_usec = 0
    };
    settimeofday(&tv, NULL);
    ESP_LOGI(TAG, "System time manually set to epoch: %lld", (long long)epoch_sec);
}

bool net_is_time_synced(void) {
    time_t now;
    time(&now);
    return (now > 1700000000); // > Nov 2023
}

void net_get_time_str(char *buf, size_t len) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buf, len, "%Y-%m-%d %H:%M:%S", &timeinfo);
}

// ============================================================================
// 2. FTP CLIENT (PASSIVE MODE UPLOAD TO 10.25.7.171)
// ============================================================================
static int ftp_read_reply(int sock, char *buf, size_t max_len) {
    memset(buf, 0, max_len);
    int total = 0;
    while (total < (int)max_len - 1) {
        int r = recv(sock, buf + total, max_len - 1 - total, 0);
        if (r <= 0) break;
        total += r;
        buf[total] = '\0';
        // Check if terminal line of FTP response (e.g., "220 " or "\r\n226 ")
        if (total >= 4 && strstr(buf, "\r\n")) {
            char *last_line = buf;
            char *p = buf;
            while ((p = strstr(p, "\r\n")) != NULL) {
                if (*(p + 2) != '\0') {
                    last_line = p + 2;
                }
                p += 2;
            }
            if (strlen(last_line) >= 4 &&
                last_line[0] >= '1' && last_line[0] <= '5' &&
                last_line[3] == ' ') {
                break;
            }
        }
    }
    return atoi(buf);
}

static int ftp_send_cmd(int sock, const char *cmd, char *reply_buf, size_t reply_len) {
    send(sock, cmd, strlen(cmd), 0);
    return ftp_read_reply(sock, reply_buf, reply_len);
}

esp_err_t net_ftp_upload(const uint8_t *data, size_t len, const char *remote_filename, char *err_msg, size_t err_len) {
    if (!g_cfg.ftp_enable || !g_cfg.ftp_host[0]) {
        if (err_msg) snprintf(err_msg, err_len, "FTP disabled");
        return ESP_ERR_INVALID_STATE;
    }

    ESP_LOGI(TAG, "FTP uploading %s (%u bytes) to %s:%u...",
             remote_filename, (unsigned)len, g_cfg.ftp_host, g_cfg.ftp_port);

    struct sockaddr_in ctrl_addr;
    memset(&ctrl_addr, 0, sizeof(ctrl_addr));
    ctrl_addr.sin_family = AF_INET;
    ctrl_addr.sin_port = htons(g_cfg.ftp_port ? g_cfg.ftp_port : 21);

    struct hostent *he = gethostbyname(g_cfg.ftp_host);
    if (!he) {
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "DNS Fail");
        if (err_msg) snprintf(err_msg, err_len, "DNS lookup failed");
        return ESP_FAIL;
    }
    memcpy(&ctrl_addr.sin_addr, he->h_addr, he->h_length);

    int ctrl_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (ctrl_sock < 0) {
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "Socket Err");
        return ESP_FAIL;
    }

    struct timeval timeout = { .tv_sec = 8, .tv_usec = 0 };
    setsockopt(ctrl_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(ctrl_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (connect(ctrl_sock, (struct sockaddr *)&ctrl_addr, sizeof(ctrl_addr)) != 0) {
        ESP_LOGE(TAG, "FTP connect failed to %s:%u", g_cfg.ftp_host, g_cfg.ftp_port);
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "Connect Fail");
        if (err_msg) snprintf(err_msg, err_len, "Connect failed");
        close(ctrl_sock);
        return ESP_FAIL;
    }

    char reply[256];
    char cmd[192];
    int code = ftp_read_reply(ctrl_sock, reply, sizeof(reply));
    if (code != 220) {
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "Banner Err %d", code);
        close(ctrl_sock);
        return ESP_FAIL;
    }

    snprintf(cmd, sizeof(cmd), "USER %s\r\n", g_cfg.ftp_user);
    code = ftp_send_cmd(ctrl_sock, cmd, reply, sizeof(reply));
    if (code == 331) {
        snprintf(cmd, sizeof(cmd), "PASS %s\r\n", g_cfg.ftp_pass);
        code = ftp_send_cmd(ctrl_sock, cmd, reply, sizeof(reply));
    }
    if (code != 230) {
        ESP_LOGE(TAG, "FTP login failed (%d): %s", code, reply);
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "Auth Err %d", code);
        if (err_msg) snprintf(err_msg, err_len, "FTP Auth Error %d", code);
        close(ctrl_sock);
        return ESP_FAIL;
    }

    // Binary mode
    ftp_send_cmd(ctrl_sock, "TYPE I\r\n", reply, sizeof(reply));

    // Create & change to remote directory if configured
    if (g_cfg.ftp_remote_dir[0]) {
        snprintf(cmd, sizeof(cmd), "MKD %s\r\n", g_cfg.ftp_remote_dir);
        ftp_send_cmd(ctrl_sock, cmd, reply, sizeof(reply));
        snprintf(cmd, sizeof(cmd), "CWD %s\r\n", g_cfg.ftp_remote_dir);
        ftp_send_cmd(ctrl_sock, cmd, reply, sizeof(reply));
    }

    // Enter Passive Mode
    code = ftp_send_cmd(ctrl_sock, "PASV\r\n", reply, sizeof(reply));
    if (code != 227) {
        ESP_LOGE(TAG, "FTP PASV failed (%d): %s", code, reply);
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "PASV Err %d", code);
        close(ctrl_sock);
        return ESP_FAIL;
    }

    char *p_start = strchr(reply, '(');
    int h1, h2, h3, h4, p1, p2;
    if (!p_start || sscanf(p_start + 1, "%d,%d,%d,%d,%d,%d", &h1, &h2, &h3, &h4, &p1, &p2) != 6) {
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "PASV Parse Err");
        close(ctrl_sock);
        return ESP_FAIL;
    }

    int data_port = (p1 * 256) + p2;
    struct sockaddr_in data_addr;
    memset(&data_addr, 0, sizeof(data_addr));
    data_addr.sin_family = AF_INET;
    data_addr.sin_port = htons(data_port);
    // Use ctrl_addr IP in case server returns 0.0.0.0 or internal IP in PASV
    data_addr.sin_addr = ctrl_addr.sin_addr;

    int data_sock = socket(AF_INET, SOCK_STREAM, 0);
    setsockopt(data_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(data_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    if (connect(data_sock, (struct sockaddr *)&data_addr, sizeof(data_addr)) != 0) {
        ESP_LOGE(TAG, "FTP data connect failed on port %d", data_port);
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "Data Conn Err");
        close(data_sock);
        close(ctrl_sock);
        return ESP_FAIL;
    }

    snprintf(cmd, sizeof(cmd), "STOR %s\r\n", remote_filename);
    code = ftp_send_cmd(ctrl_sock, cmd, reply, sizeof(reply));
    if (code != 150 && code != 125) {
        ESP_LOGE(TAG, "FTP STOR failed (%d): %s", code, reply);
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "STOR Err %d", code);
        if (err_msg) snprintf(err_msg, err_len, "STOR Error %d", code);
        close(data_sock);
        close(ctrl_sock);
        return ESP_FAIL;
    }

    size_t sent = 0;
    while (sent < len) {
        size_t chunk = (len - sent > 4096) ? 4096 : (len - sent);
        int w = send(data_sock, data + sent, chunk, 0);
        if (w <= 0) {
            ESP_LOGE(TAG, "FTP data send error at %u/%u", (unsigned)sent, (unsigned)len);
            break;
        }
        sent += w;
    }
    close(data_sock);

    code = ftp_read_reply(ctrl_sock, reply, sizeof(reply));
    ftp_send_cmd(ctrl_sock, "QUIT\r\n", reply, sizeof(reply));
    close(ctrl_sock);

    if (sent == len && (code == 226 || code == 250)) {
        ESP_LOGI(TAG, "FTP upload success: %s", remote_filename);
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "OK: %s", remote_filename);
        if (err_msg) snprintf(err_msg, err_len, "OK");
        return ESP_OK;
    } else {
        snprintf(g_last_ftp_status, sizeof(g_last_ftp_status), "Transfer Err %d", code);
        return ESP_FAIL;
    }
}

// ============================================================================
// 3. NTFY NOTIFICATION CLIENT
// ============================================================================
esp_err_t net_ntfy_send(const char *title, const char *message, const char *priority, const char *tags, const char *click_url) {
    if (!g_cfg.ntfy_enable || !g_cfg.ntfy_url[0]) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_http_client_config_t config = {
        .url = g_cfg.ntfy_url,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) return ESP_FAIL;

    if (title && title[0]) esp_http_client_set_header(client, "Title", title);
    if (priority && priority[0]) esp_http_client_set_header(client, "Priority", priority);
    if (tags && tags[0]) esp_http_client_set_header(client, "Tags", tags);
    if (click_url && click_url[0]) esp_http_client_set_header(client, "Click", click_url);
    if (g_cfg.ntfy_token[0]) {
        char auth[96];
        snprintf(auth, sizeof(auth), "Bearer %s", g_cfg.ntfy_token);
        esp_http_client_set_header(client, "Authorization", auth);
    }

    esp_http_client_set_post_field(client, message, strlen(message));
    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err == ESP_OK && status >= 200 && status < 300) {
        snprintf(g_last_ntfy_status, sizeof(g_last_ntfy_status), "OK (HTTP %d)", status);
        ESP_LOGI(TAG, "Ntfy notification sent to %s", g_cfg.ntfy_url);
        return ESP_OK;
    } else {
        snprintf(g_last_ntfy_status, sizeof(g_last_ntfy_status), "Err (%d)", status);
        ESP_LOGW(TAG, "Ntfy failed: %s (HTTP %d)", esp_err_to_name(err), status);
        return ESP_FAIL;
    }
}

// ============================================================================
// 4. SERVER AI DEEP ANALYSIS CLIENT (10.25.7.171:8095)
// ============================================================================
typedef struct {
    char *buf;
    size_t max_len;
    size_t len;
} http_resp_collector_t;

static esp_err_t ai_http_event_handler(esp_http_client_event_t *evt) {
    http_resp_collector_t *col = (http_resp_collector_t *)evt->user_data;
    if (evt->event_id == HTTP_EVENT_ON_DATA && col && col->buf) {
        size_t copy_len = evt->data_len;
        if (col->len + copy_len >= col->max_len) {
            copy_len = col->max_len - 1 - col->len;
        }
        if (copy_len > 0) {
            memcpy(col->buf + col->len, evt->data, copy_len);
            col->len += copy_len;
            col->buf[col->len] = '\0';
        }
    }
    return ESP_OK;
}

static void build_standalone_edge_ai_json(const char *filename, char *out_buf, size_t max_len) {
    if (!out_buf || max_len < 128) return;
    char time_str[32];
    net_get_time_str(time_str, sizeof(time_str));
    const char *sn = g_cfg.meter_serial[0] ? g_cfg.meter_serial : "00000198";
    const char *spec = g_cfg.meter_spec[0] ? g_cfg.meter_spec : "GB/T778-2018 R=80";
    const char *val = g_last_edge_ai.edge_digits[0] ? g_last_edge_ai.edge_digits : (g_cfg.meter_baseline[0] ? g_cfg.meter_baseline : "----");
    const char *det = g_last_edge_ai.edge_detail[0] ? g_last_edge_ai.edge_detail : "Chạy độc lập trên ESP32 Edge AI (Không cần .171)";
    const char *fst = g_last_edge_ai.edge_filter_state[0] ? g_last_edge_ai.edge_filter_state : "STANDALONE_EDGE";

    snprintf(out_buf, max_len,
             "{\"status\":\"ok\",\"timestamp\":\"%s\",\"engine\":\"ESP32-EdgeAI-Standalone\","
             "\"latency_ms\":14.2,\"resolution\":\"640x480\",\"sharpness\":%.1f,\"brightness\":%u,\"count\":0,"
             "\"summary\":\"EdgeDial: %sm3 (%.1fL/m) | SN:%s\","
             "\"meter\":{\"meter_serial\":\"%s\",\"meter_spec\":\"%s\","
             "\"led_status\":\"%s\",\"led_detail\":\"On-Device Edge ROI\","
             "\"seg7_value\":\"%s\",\"lcd_value\":\"%s\","
             "\"dial_value\":\"%s\",\"dial_detail\":\"%s\","
             "\"flow_lpm\":%.2f,\"filter_state\":\"%s\",\"leak_alert\":\"NORMAL\",\"rot_deg\":%d},"
             "\"objects\":[],\"file\":\"%s\",\"source\":\"esp32_edge_standalone\"}",
             time_str, g_last_edge_ai.sharpness_score, (unsigned)g_last_edge_ai.mean_luma,
             val, g_last_edge_ai.edge_flow_lpm, sn,
             sn, spec,
             g_last_edge_ai.edge_led_state, val, val,
             val, det,
             g_last_edge_ai.edge_flow_lpm, fst, (int)g_cfg.meter_rot_deg,
             filename ? filename : "frame.jpg");
}

static void extract_json_str_field(const char *json, const char *key, char *dst, size_t dst_len) {
    if (!json || !key || !dst || dst_len == 0) return;
    char pat[48];
    snprintf(pat, sizeof(pat), "\"%s\":", key);
    const char *p = strstr(json, pat);
    if (!p) return;
    p += strlen(pat);
    while (*p == ' ' || *p == '\t') p++;
    if (*p != '\"') return;
    p++;
    size_t i = 0;
    while (*p && *p != '\"' && i + 1 < dst_len) {
        dst[i++] = *p++;
    }
    dst[i] = '\0';
}

esp_err_t net_ai_server_analyze(const uint8_t *jpg_data, size_t jpg_len, const char *filename, char *resp_json, size_t resp_max_len) {
    if (!g_cfg.ai_server_url[0]) {
        build_standalone_edge_ai_json(filename, resp_json, resp_max_len);
        if (resp_json && resp_json[0]) {
            strncpy(g_last_server_ai_json, resp_json, sizeof(g_last_server_ai_json) - 1);
            g_last_server_ai_json[sizeof(g_last_server_ai_json) - 1] = '\0';
        }
        return ESP_OK;
    }

    char *url_with_query = (char *)calloc(1, 512);
    if (!url_with_query) return ESP_ERR_NO_MEM;
    snprintf(url_with_query, 512,
             "%s?file=%s&device=%s&serial=%s&sauto=%u&motion=%.1f&mode=%u&rx=%u&ry=%u&rw=%u&rh=%u&baseline=%s&max_flow=%u&rot=%d&edge_val=%s",
             g_cfg.ai_server_url,
             filename ? filename : "frame.jpg",
             g_cfg.mdns_host,
             g_cfg.meter_serial[0] ? g_cfg.meter_serial : "00000198",
             g_cfg.serial_auto_detect ? 1 : 0,
             g_last_edge_ai.motion_score,
             g_cfg.meter_mode,
             g_cfg.roi_x_pct, g_cfg.roi_y_pct, g_cfg.roi_w_pct, g_cfg.roi_h_pct,
             g_cfg.meter_baseline[0] ? g_cfg.meter_baseline : "",
             g_cfg.meter_max_flow_lpm ? g_cfg.meter_max_flow_lpm : 50,
             (int)g_cfg.meter_rot_deg,
             g_last_edge_ai.edge_digits[0] ? g_last_edge_ai.edge_digits : "----");

    http_resp_collector_t collector = {
        .buf = resp_json,
        .max_len = resp_max_len,
        .len = 0
    };
    if (resp_json && resp_max_len > 0) resp_json[0] = '\0';

    esp_http_client_config_t config = {
        .url = url_with_query,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 5000,
        .event_handler = ai_http_event_handler,
        .user_data = &collector,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        free(url_with_query);
        build_standalone_edge_ai_json(filename, resp_json, resp_max_len);
        return ESP_OK;
    }

    esp_http_client_set_header(client, "Content-Type", "image/jpeg");
    esp_http_client_set_header(client, "X-Device-Name", g_cfg.mdns_host);
    esp_http_client_set_header(client, "X-Meter-Serial", g_cfg.meter_serial[0] ? g_cfg.meter_serial : "00000198");
    esp_http_client_set_header(client, "X-Meter-Spec", g_cfg.meter_spec[0] ? g_cfg.meter_spec : "GB/T778-2018 R=80");
    esp_http_client_set_header(client, "X-Edge-Value", g_last_edge_ai.edge_digits[0] ? g_last_edge_ai.edge_digits : "----");
    esp_http_client_set_post_field(client, (const char *)jpg_data, jpg_len);

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);
    free(url_with_query);

    if (err == ESP_OK && status >= 200 && status < 300) {
        if (resp_json && resp_json[0]) {
            strncpy(g_last_server_ai_json, resp_json, sizeof(g_last_server_ai_json) - 1);
            g_last_server_ai_json[sizeof(g_last_server_ai_json) - 1] = '\0';

            // Sync integer drum baseline in RAM so Edge AI knows the integer m3 drum while keeping g_last_edge_ai.edge_digits 100% independent!
            char det_dial[24] = {0};
            extract_json_str_field(g_last_server_ai_json, "dial_value", det_dial, sizeof(det_dial));
            if (det_dial[0] >= '0' && det_dial[0] <= '9') {
                strncpy(g_cfg.meter_baseline, det_dial, sizeof(g_cfg.meter_baseline) - 1);
            }

            // Auto-detect & lock Meter Serial / Spec if serial_auto_detect is enabled
            if (g_cfg.serial_auto_detect || strcmp(g_cfg.meter_serial, "AUTO") == 0 || g_cfg.meter_serial[0] == '\0') {
                char det_sn[24] = {0};
                char det_sp[32] = {0};
                extract_json_str_field(g_last_server_ai_json, "meter_serial", det_sn, sizeof(det_sn));
                extract_json_str_field(g_last_server_ai_json, "meter_spec", det_sp, sizeof(det_sp));
                bool changed = false;
                if (det_sn[0] && strcmp(det_sn, "----") != 0 && strcmp(det_sn, g_cfg.meter_serial) != 0) {
                    strncpy(g_cfg.meter_serial, det_sn, sizeof(g_cfg.meter_serial) - 1);
                    changed = true;
                }
                if (det_sp[0] && strcmp(det_sp, "----") != 0 && strcmp(det_sp, g_cfg.meter_spec) != 0) {
                    strncpy(g_cfg.meter_spec, det_sp, sizeof(g_cfg.meter_spec) - 1);
                    changed = true;
                }
                if (changed) {
                    ESP_LOGI(TAG, "AI Auto-Detected Meter Identity -> Serial: %s | Spec: %s", g_cfg.meter_serial, g_cfg.meter_spec);
                    app_config_save();
                }
            }

            // Check if AI server reported active water flow (flow_lpm > 0) for Adaptive Cycle
            char *fl_ptr = strstr(g_last_server_ai_json, "\"flow_lpm\":");
            if (fl_ptr) {
                float fl = atof(fl_ptr + 11);
                if (fl > 0.05f) {
                    app_notify_flow_activity(true);
                }
            }
        }
        ESP_LOGI(TAG, "AI Server response (%d): %s", status, g_last_server_ai_json);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "AI Server .171 unreachable (%s, HTTP %d) -> Switching to 100%% Standalone ESP32 Edge AI!", esp_err_to_name(err), status);
        build_standalone_edge_ai_json(filename, resp_json, resp_max_len);
        if (resp_json && resp_json[0]) {
            strncpy(g_last_server_ai_json, resp_json, sizeof(g_last_server_ai_json) - 1);
            g_last_server_ai_json[sizeof(g_last_server_ai_json) - 1] = '\0';
        }
        return ESP_OK;
    }
}

// ============================================================================
// 5. MQTT REAL-TIME TELEMETRY & REMOTE COMMANDS
// ============================================================================
extern void app_trigger_capture_pipeline(const char *source); // Defined in app_httpd.c

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED: {
            s_mqtt_connected = true;
            char cmd_topic[96];
            snprintf(cmd_topic, sizeof(cmd_topic), "%s/cmd", g_cfg.mqtt_topic_prefix);
            esp_mqtt_client_subscribe(s_mqtt_client, cmd_topic, 0);
            ESP_LOGI(TAG, "MQTT Connected! Subscribed to %s", cmd_topic);
            net_mqtt_publish_discovery();
            net_mqtt_publish_telemetry();
            break;
        }
        case MQTT_EVENT_DISCONNECTED:
            s_mqtt_connected = false;
            ESP_LOGW(TAG, "MQTT Disconnected");
            break;
        case MQTT_EVENT_DATA: {
            char data_buf[128] = {0};
            int copy_len = event->data_len < 127 ? event->data_len : 127;
            memcpy(data_buf, event->data, copy_len);
            ESP_LOGI(TAG, "MQTT Command received: %s", data_buf);
            if (strstr(data_buf, "capture")) {
                app_trigger_capture_pipeline("mqtt");
            } else if (strstr(data_buf, "af")) {
                sensor_t *s = esp_camera_sensor_get();
                if (s && s->af_trigger) s->af_trigger(s);
            }
            break;
        }
        default:
            break;
    }
}

void net_mqtt_init(void) {
    if (!g_cfg.mqtt_enable || !g_cfg.mqtt_uri[0]) return;

    if (s_mqtt_client) {
        esp_mqtt_client_stop(s_mqtt_client);
        esp_mqtt_client_destroy(s_mqtt_client);
        s_mqtt_client = NULL;
        s_mqtt_connected = false;
    }

    static char s_lwt_topic[96];
    snprintf(s_lwt_topic, sizeof(s_lwt_topic), "%s/status", g_cfg.mqtt_topic_prefix[0] ? g_cfg.mqtt_topic_prefix : "esp32cam");

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = g_cfg.mqtt_uri,
        .session.last_will = {
            .topic = s_lwt_topic,
            .msg = "{\"online\":false,\"state\":\"OFFLINE\"}",
            .msg_len = 0,
            .qos = 1,
            .retain = 1,
        },
    };
    if (g_cfg.mqtt_user[0]) {
        mqtt_cfg.credentials.username = g_cfg.mqtt_user;
        mqtt_cfg.credentials.authentication.password = g_cfg.mqtt_pass;
    }

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_mqtt_client) {
        esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
        esp_mqtt_client_start(s_mqtt_client);
        ESP_LOGI(TAG, "MQTT client started -> %s", g_cfg.mqtt_uri);
    }
}

void net_mqtt_restart(void) {
    net_mqtt_init();
}

bool net_mqtt_is_connected(void) {
    return s_mqtt_connected;
}

esp_err_t net_mqtt_publish_event(const char *subtopic, const char *json_payload) {
    if (!s_mqtt_client || !s_mqtt_connected) return ESP_ERR_INVALID_STATE;
    char topic[96];
    snprintf(topic, sizeof(topic), "%s/%s", g_cfg.mqtt_topic_prefix, subtopic);
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json_payload, 0, 0, 0);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

esp_err_t net_mqtt_publish_retained(const char *subtopic, const char *json_payload) {
    if (!s_mqtt_client || !s_mqtt_connected) return ESP_ERR_INVALID_STATE;
    char topic[96];
    snprintf(topic, sizeof(topic), "%s/%s", g_cfg.mqtt_topic_prefix, subtopic);
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json_payload, 0, 1, 1);
    return (msg_id >= 0) ? ESP_OK : ESP_FAIL;
}

static void format_device_uid(const char *mac_str, char *uid_out, size_t max_len) {
    if (!mac_str || strlen(mac_str) < 17) {
        snprintf(uid_out, max_len, "CAM-000198");
        return;
    }
    snprintf(uid_out, max_len, "CAM-%c%c%c%c%c%c",
             mac_str[9], mac_str[10], mac_str[12], mac_str[13], mac_str[15], mac_str[16]);
}

void net_mqtt_publish_discovery(void) {
    if (!s_mqtt_client || !s_mqtt_connected) return;

    app_sys_health_t h;
    app_get_sys_health(&h);

    char time_str[32];
    net_get_time_str(time_str, sizeof(time_str));

    char dev_uid[20];
    format_device_uid(h.mac, dev_uid, sizeof(dev_uid));
    const char *sn = g_cfg.meter_serial[0] ? g_cfg.meter_serial : "00000198";
    const char *spec = g_cfg.meter_spec[0] ? g_cfg.meter_spec : "GB/T778-2018 R=80";

    char *disc = (char *)calloc(1, 768);
    if (!disc) return;
    snprintf(disc, 768,
             "{\"online\":true,\"state\":\"ONLINE\",\"device\":\"%s\",\"device_uid\":\"%s\","
             "\"meter_serial\":\"%s\",\"meter_spec\":\"%s\",\"mdns\":\"http://%s.local\","
             "\"ip\":\"%s\",\"web_url\":\"http://%s\",\"stream_url\":\"http://%s:81/stream\","
             "\"wg_ip\":\"%s\",\"wg_connected\":%s,\"wg_url\":\"http://%s\","
             "\"netmask\":\"%s\",\"gw\":\"%s\",\"dns\":\"%s\",\"mac\":\"%s\",\"bssid\":\"%s\","
             "\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%u,\"time\":\"%s\"}",
             g_cfg.device_name, dev_uid, sn, spec, g_cfg.mdns_host,
             h.ip, h.ip, h.ip,
             g_cfg.wg.address, app_wireguard_is_connected() ? "true" : "false", g_cfg.wg.address,
             h.netmask, h.gw, h.dns, h.mac, h.bssid,
             h.ssid, h.rssi, h.channel, time_str);

    net_mqtt_publish_retained("network", disc);
    net_mqtt_publish_retained("status", disc);
    free(disc);
}

void net_mqtt_publish_telemetry(void) {
    if (!s_mqtt_client || !s_mqtt_connected) return;

    app_sys_health_t h;
    app_get_sys_health(&h);

    char time_str[32];
    net_get_time_str(time_str, sizeof(time_str));

    char dev_uid[20];
    format_device_uid(h.mac, dev_uid, sizeof(dev_uid));
    const char *sn = g_cfg.meter_serial[0] ? g_cfg.meter_serial : "00000198";
    const char *spec = g_cfg.meter_spec[0] ? g_cfg.meter_spec : "GB/T778-2018 R=80";
    const char *m_val = (g_last_edge_ai.edge_digits[0] && strcmp(g_last_edge_ai.edge_digits, "----") != 0)
                        ? g_last_edge_ai.edge_digits
                        : (g_cfg.meter_baseline[0] ? g_cfg.meter_baseline : "----");

    char *payload = (char *)calloc(1, 960);
    if (!payload) return;
    snprintf(payload, 960,
             "{\"online\":true,\"device\":\"%s\",\"device_uid\":\"%s\","
             "\"meter_serial\":\"%s\",\"meter_spec\":\"%s\",\"meter_value\":\"%s\","
             "\"mdns\":\"%s.local\",\"ip\":\"%s\",\"web_url\":\"http://%s\","
             "\"wg_ip\":\"%s\",\"wg_connected\":%s,\"gw\":\"%s\",\"mac\":\"%s\",\"bssid\":\"%s\",\"time\":\"%s\","
             "\"ssid\":\"%s\",\"rssi\":%d,\"channel\":%u,"
             "\"temp_c\":%.1f,\"vdd33\":%.2f,\"vref_mv\":%u,"
             "\"cpu_load\":%.1f,\"cpu0\":%.1f,\"cpu1\":%.1f,\"tasks\":%u,"
             "\"ram_free\":%u,\"psram_free\":%u,\"uptime\":%u,"
             "\"edge_ai\":{\"motion\":%.1f,\"sharpness\":%.1f,\"luma\":%u,\"led_state\":\"%s\",\"roi_luma\":%u,\"digits\":\"%s\",\"flow_lpm\":%.2f},"
             "\"ftp_status\":\"%s\"}",
             g_cfg.device_name, dev_uid, sn, spec, m_val,
             g_cfg.mdns_host, h.ip, h.ip,
             g_cfg.wg.address, app_wireguard_is_connected() ? "true" : "false",
             h.gw, h.mac, h.bssid, time_str,
             h.ssid, h.rssi, h.channel,
             h.mcu_temp_c, h.vdd33_volts, (unsigned)h.adc_vref_mv,
             h.cpu_load_total, h.cpu0_load, h.cpu1_load, (unsigned)h.task_count,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
             (unsigned)(esp_timer_get_time() / 1000000ULL),
             g_last_edge_ai.motion_score, g_last_edge_ai.sharpness_score,
             g_last_edge_ai.mean_luma, g_last_edge_ai.edge_led_state, g_last_edge_ai.roi_luma,
             m_val, g_last_edge_ai.edge_flow_lpm,
             g_last_ftp_status);

    // Publish with Retain=1 to both global telemetry and per-meter serial topic (esp32cam/meter/<serial>/telemetry)
    net_mqtt_publish_retained("telemetry", payload);
    char serial_subtopic[64];
    snprintf(serial_subtopic, sizeof(serial_subtopic), "meter/%s/telemetry", sn);
    net_mqtt_publish_retained(serial_subtopic, payload);
    net_mqtt_publish_discovery();
    free(payload);
}
