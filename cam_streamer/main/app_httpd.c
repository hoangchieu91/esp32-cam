#include "app_httpd.h"
#include "httpd_ota_sys.h"
#include "httpd_ai_mcp.h"
#include "app_config.h"
#include "osd_stamp.h"
#include "net_services.h"
#include "web_ui.h"

#include "esp_http_server.h"
#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "esp_camera_af.h"
#include "img_converters.h"
#include "esp_log.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_wifi.h"
#include "cJSON.h"

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "camera_httpd";

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static httpd_handle_t s_web_httpd = NULL;
static httpd_handle_t s_stream_httpd = NULL;
static SemaphoreHandle_t s_cam_mutex = NULL;

static bool s_sd_mounted = false;
static uint64_t s_sd_total_mb = 0;
static uint64_t s_sd_fat_total_mb = 0;
static uint64_t s_sd_free_mb = 0;
static char s_sd_name[32] = "SU16G";

// Industrial Pipeline SLA Counters
static uint32_t s_capture_ok_cnt = 0;
static uint32_t s_capture_err_cnt = 0;
static uint32_t s_last_capture_ms = 0;
static uint32_t s_sd_write_ok_cnt = 0;
static uint32_t s_sd_write_err_cnt = 0;
static uint32_t s_ftp_ok_cnt = 0;
static uint32_t s_ftp_err_cnt = 0;
static uint32_t s_fb_fail_streak = 0;

// PSRAM Cache of Last Illuminated & Focused JPEG Frame (prevents black screen at night & keeps camera cool)
static uint8_t *s_last_lit_jpg_buf = NULL;
static size_t s_last_lit_jpg_len = 0;

static void cache_lit_jpeg_in_psram(const uint8_t *buf, size_t len) {
    if (!buf || len == 0 || len > 256 * 1024) return;
    uint8_t *nb = (uint8_t *)heap_caps_malloc(len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!nb) return;
    memcpy(nb, buf, len);
    uint8_t *old = s_last_lit_jpg_buf;
    s_last_lit_jpg_buf = nb;
    s_last_lit_jpg_len = len;
    if (old) free(old);
}

void app_set_sd_mounted(bool mounted, uint64_t total_mb, uint64_t fat_total_mb, uint64_t free_mb, const char *card_name) {
    s_sd_mounted = mounted;
    s_sd_total_mb = total_mb;
    s_sd_fat_total_mb = fat_total_mb;
    s_sd_free_mb = free_mb;
    if (card_name && card_name[0]) {
        strncpy(s_sd_name, card_name, sizeof(s_sd_name) - 1);
    }
}

// ============================================================================
// CORE CAPTURE + OSD + SD + FTP + SERVER AI + MQTT + NTFY PIPELINE
// ============================================================================
static esp_err_t execute_capture_pipeline(const char *mode, char *out_filename, size_t fn_len, char *out_ftp_st, size_t ftp_len);

esp_err_t app_execute_capture_pipeline(const char *mode, char *out_filename, size_t fn_len, char *out_ftp_st, size_t ftp_len) {
    return execute_capture_pipeline(mode, out_filename, fn_len, out_ftp_st, ftp_len);
}

static esp_err_t execute_capture_pipeline(const char *mode, char *out_filename, size_t fn_len, char *out_ftp_st, size_t ftp_len) {
    int64_t t_start_us = esp_timer_get_time();
    if (s_cam_mutex) xSemaphoreTake(s_cam_mutex, pdMS_TO_TICKS(3000));
    app_cam_wake_if_sleeping();
    bool flash_assisted = app_flash_pre_capture_assist();
    sensor_t *s_cap = esp_camera_sensor_get();
    if (s_cap) {
        if (g_cfg.af_mode == 0) {
            app_cam_trigger_af_and_lock();
        } else if (s_cap->af_set_manual_position && g_cfg.af_manual_pos > 0) {
            s_cap->af_set_manual_position(s_cap, g_cfg.af_manual_pos);
            vTaskDelay(pdMS_TO_TICKS(110));
        }
    }
    // Flush 2 stale DMA frames so the captured image is guaranteed post-focus & post-exposure
    for (int f = 0; f < 2; f++) {
        camera_fb_t *stale_fb = esp_camera_fb_get();
        if (stale_fb) esp_camera_fb_return(stale_fb);
    }
    camera_fb_t *fb = esp_camera_fb_get();
    // Thermal Protection: Immediately de-energize VCM coil (0mA) and Flash LED right after shutter closes!
    if (app_cam_get_stream_clients() == 0) {
        app_cam_release_vcm_coil();
    }
    if (!fb) {
        app_flash_post_capture_restore(flash_assisted);
        s_capture_err_cnt++;
        s_fb_fail_streak++;
        if (s_fb_fail_streak >= 3) {
            s_fb_fail_streak = 0;
            app_recover_camera_hardware();
        }
        if (s_cam_mutex) xSemaphoreGive(s_cam_mutex);
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_FAIL;
    }
    app_flash_post_capture_restore(flash_assisted);
    s_fb_fail_streak = 0;
    s_capture_ok_cnt++;

    // Generate timestamp filename
    time_t now;
    struct tm ti;
    time(&now);
    localtime_r(&now, &ti);
    char base_name[64];
    if (now > 1700000000) {
        strftime(base_name, sizeof(base_name), "cam_%Y%m%d_%H%M%S.jpg", &ti);
    } else {
        snprintf(base_name, sizeof(base_name), "cam_%lld.jpg", (long long)(esp_timer_get_time() / 1000ULL));
    }
    if (out_filename) snprintf(out_filename, fn_len, "%s", base_name);

    // Keep clean un-drawn JPEG copy in PSRAM for AI Server analysis & Night Viewport Cache
    size_t raw_len = fb->len;
    uint8_t *raw_buf = (uint8_t *)heap_caps_malloc(raw_len, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (raw_buf) {
        memcpy(raw_buf, fb->buf, raw_len);
    }
    cache_lit_jpeg_in_psram(fb->buf, fb->len);

    // Burn OSD Timestamp & run Edge AI
    uint8_t *jpg_buf = NULL;
    size_t jpg_len = 0;
    bool jpg_allocated = false;
    process_frame_osd_and_ai(fb->buf, fb->len, fb->width, fb->height, 88, mode, &jpg_buf, &jpg_len, &jpg_allocated);

    // Return camera frame buffer as soon as we have our OSD JPEG (or copy if not allocated)
    uint8_t *work_buf = NULL;
    size_t work_len = jpg_len;
    if (jpg_allocated) {
        work_buf = jpg_buf;
    } else if (raw_buf) {
        work_buf = raw_buf;
        work_len = raw_len;
    }
    esp_camera_fb_return(fb);
    if (s_cam_mutex) xSemaphoreGive(s_cam_mutex);

    if (!work_buf) {
        if (raw_buf) free(raw_buf);
        return ESP_FAIL;
    }

    bool ai_only = (mode && (strcmp(mode, "ai_only") == 0 || strcmp(mode, "debug_board") == 0));

    // 1. Save to SD Card (with Industrial Auto-Remount on failure)
    if (!ai_only && g_cfg.sd_save_enable && s_sd_mounted) {
        char sd_path[96];
        snprintf(sd_path, sizeof(sd_path), "/sdcard/%s", base_name);
        FILE *f = fopen(sd_path, "wb");
        if (!f) {
            ESP_LOGW(TAG, "SD write failed, triggering Industrial SD Auto-Remount...");
            app_remount_sd_card();
            f = fopen(sd_path, "wb");
        }
        if (f) {
            fwrite(work_buf, 1, work_len, f);
            fclose(f);
            s_sd_write_ok_cnt++;
            ESP_LOGI(TAG, "Saved to SD Card: %s (%u bytes)", sd_path, (unsigned)work_len);
        } else {
            s_sd_write_err_cnt++;
            ESP_LOGE(TAG, "Failed to write SD file after remount: %s", sd_path);
        }
    }

    // 2. Upload via FTP to Server 10.25.7.171
    char ftp_err[64] = "Skipped";
    if (!ai_only && g_cfg.ftp_enable) {
        if (net_ftp_upload(work_buf, work_len, base_name, ftp_err, sizeof(ftp_err)) == ESP_OK) {
            s_ftp_ok_cnt++;
        } else {
            s_ftp_err_cnt++;
        }
    }
    if (out_ftp_st) snprintf(out_ftp_st, ftp_len, "%s", ftp_err);
    s_last_capture_ms = (uint32_t)((esp_timer_get_time() - t_start_us) / 1000LL);

    // 3. Push clean un-drawn frame to Deep AI Server (10.25.7.171:8095)
    char *ai_resp = (char *)calloc(1, 2048);
    esp_err_t ai_ret = ESP_FAIL;
    if (ai_resp) {
        const uint8_t *ai_img = raw_buf ? raw_buf : work_buf;
        size_t ai_len = raw_buf ? raw_len : work_len;
        ai_ret = net_ai_server_analyze(ai_img, ai_len, base_name, ai_resp, 2048);
    }
    if (raw_buf && raw_buf != work_buf) {
        free(raw_buf);
    }

    // 4. Publish Real-Time MQTT Event (Global + Per-Meter Serial Topic)
    char *mqtt_event = (char *)calloc(1, 2560);
    if (mqtt_event) {
        const char *sn = g_cfg.meter_serial[0] ? g_cfg.meter_serial : "00000198";
        const char *spec = g_cfg.meter_spec[0] ? g_cfg.meter_spec : "GB/T778-2018 R=80";
        snprintf(mqtt_event, 2560,
                 "{\"event\":\"capture\",\"source\":\"%s\",\"meter_serial\":\"%s\",\"meter_spec\":\"%s\","
                 "\"file\":\"%s\",\"size\":%u,\"ftp\":\"%s\",\"edge_digits\":\"%s\",\"edge_motion\":%.1f,"
                 "\"edge_led\":\"%s\",\"ai\":%s}",
                 mode ? mode : "manual", sn, spec, base_name, (unsigned)work_len,
                 ftp_err, g_last_edge_ai.edge_digits, g_last_edge_ai.motion_score, g_last_edge_ai.edge_led_state,
                 (ai_ret == ESP_OK && ai_resp && ai_resp[0] == '{') ? ai_resp : "\"pending\"");
        net_mqtt_publish_event("capture", mqtt_event);
        char serial_cap_topic[64];
        snprintf(serial_cap_topic, sizeof(serial_cap_topic), "meter/%s/capture", sn);
        net_mqtt_publish_event(serial_cap_topic, mqtt_event);
        free(mqtt_event);
    }

    // 5. Send Ntfy Notification if enabled
    if (g_cfg.ntfy_enable && !ai_only) {
        char *ntfy_msg = (char *)calloc(1, 512);
        if (ntfy_msg) {
            snprintf(ntfy_msg, 512,
                     "[%s] Đã chụp: %s (%u KB) | FTP: %s | Motion: %.1f%% | AI: %.240s",
                     g_cfg.device_name, base_name, (unsigned)(work_len / 1024),
                     ftp_err, g_last_edge_ai.motion_score,
                     (ai_ret == ESP_OK && ai_resp && ai_resp[0]) ? ai_resp : "Hoàn tất");
            net_ntfy_send("ESP32-CAM AI Alert", ntfy_msg, "default", "camera,robot", "http://10.25.7.171:8095");
            free(ntfy_msg);
        }
    }

    if (ai_resp) free(ai_resp);

    free(work_buf);
    return ESP_OK;
}

typedef struct {
    char source[24];
} async_cap_arg_t;

static void async_capture_task(void *arg) {
    async_cap_arg_t *a = (async_cap_arg_t *)arg;
    char fn[64], ftp_st[64];
    execute_capture_pipeline(a ? a->source : "auto", fn, sizeof(fn), ftp_st, sizeof(ftp_st));
    if (a) free(a);
    vTaskDelete(NULL);
}

void app_trigger_capture_pipeline(const char *source) {
    async_cap_arg_t *a = (async_cap_arg_t *)malloc(sizeof(async_cap_arg_t));
    if (a) {
        snprintf(a->source, sizeof(a->source), "%s", source ? source : "trigger");
        xTaskCreatePinnedToCore(async_capture_task, "cap_pipe", 8192, a, 5, NULL, 1);
    }
}

// ============================================================================
// HTTP HANDLERS
// ============================================================================
static esp_err_t index_handler(httpd_req_t *req) {
    char etag[64];
    snprintf(etag, sizeof(etag), "\"ui-%u-" __TIME__ "\"", (unsigned)index_html_gz_len);
    char inm[64] = {0};
    if (httpd_req_get_hdr_value_str(req, "If-None-Match", inm, sizeof(inm)) == ESP_OK) {
        if (strcmp(inm, etag) == 0) {
            httpd_resp_set_status(req, "304 Not Modified");
            httpd_resp_set_hdr(req, "ETag", etag);
            return httpd_resp_send(req, NULL, 0);
        }
    }
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "ETag", etag);
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, (const char *)index_html_gz, index_html_gz_len);
}

static esp_err_t status_handler(httpd_req_t *req) {
    size_t ram_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    size_t ram_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t ram_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    size_t ram_largest = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);

    size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t psram_min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);

    uint32_t uptime = (uint32_t)(esp_timer_get_time() / 1000000ULL);

    app_sys_health_t *hp = (app_sys_health_t *)calloc(1, sizeof(app_sys_health_t));
    if (!hp) return ESP_ERR_NO_MEM;
    app_get_sys_health(hp);
    #define h (*hp)

    sensor_t *s = esp_camera_sensor_get();
    bool af_focused = true, af_busy = false;

    char time_str[32];
    net_get_time_str(time_str, sizeof(time_str));

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device", g_cfg.device_name);
    cJSON_AddStringToObject(root, "mdns", g_cfg.mdns_host);
    cJSON_AddStringToObject(root, "ip", h.ip);
    cJSON_AddStringToObject(root, "netmask", h.netmask);
    cJSON_AddStringToObject(root, "gw", h.gw);
    cJSON_AddStringToObject(root, "dns", h.dns);
    cJSON_AddStringToObject(root, "mac", h.mac);
    cJSON_AddStringToObject(root, "bssid", h.bssid);
    cJSON_AddNumberToObject(root, "channel", h.channel);
    cJSON_AddStringToObject(root, "time", time_str);
    cJSON_AddBoolToObject(root, "time_synced", net_is_time_synced());
    cJSON_AddStringToObject(root, "ssid", h.ssid);
    cJSON_AddNumberToObject(root, "rssi", h.rssi);
    cJSON_AddNumberToObject(root, "active_slot", app_wifi_get_active_slot());
    cJSON_AddBoolToObject(root, "mqtt", net_mqtt_is_connected());
    cJSON_AddBoolToObject(root, "sd_mounted", s_sd_mounted);
    cJSON_AddNumberToObject(root, "sd_total_mb", (double)s_sd_total_mb);
    cJSON_AddNumberToObject(root, "sd_fat_total_mb", (double)s_sd_fat_total_mb);
    cJSON_AddNumberToObject(root, "sd_free_mb", (double)s_sd_free_mb);
    cJSON_AddStringToObject(root, "sd_name", s_sd_name);

    // System Health: Temperature, Voltage, CPU, Memory
    cJSON_AddNumberToObject(root, "mcu_temp_c", ((int)(h.mcu_temp_c * 10.0f)) / 10.0);
    cJSON_AddNumberToObject(root, "vdd33_volts", ((int)(h.vdd33_volts * 100.0f)) / 100.0);
    cJSON_AddNumberToObject(root, "adc_vref_mv", h.adc_vref_mv);
    cJSON_AddNumberToObject(root, "cpu_load", ((int)(h.cpu_load_total * 10.0f)) / 10.0);
    cJSON_AddNumberToObject(root, "cpu0_load", ((int)(h.cpu0_load * 10.0f)) / 10.0);
    cJSON_AddNumberToObject(root, "cpu1_load", ((int)(h.cpu1_load * 10.0f)) / 10.0);
    cJSON_AddNumberToObject(root, "task_count", h.task_count);
    cJSON_AddNumberToObject(root, "cpu_freq_mhz", h.cpu_freq_mhz);
    cJSON_AddStringToObject(root, "reset_reason", h.reset_reason);

    cJSON_AddNumberToObject(root, "ram_total", ram_total);
    cJSON_AddNumberToObject(root, "ram_free", ram_free);
    cJSON_AddNumberToObject(root, "ram_min_free", ram_min_free);
    cJSON_AddNumberToObject(root, "ram_largest", ram_largest);
    cJSON_AddNumberToObject(root, "psram_total", psram_total);
    cJSON_AddNumberToObject(root, "psram_free", psram_free);
    cJSON_AddNumberToObject(root, "psram_min_free", psram_min_free);
    cJSON_AddNumberToObject(root, "uptime", uptime);

    cJSON_AddBoolToObject(root, "af_focused", af_focused);
    cJSON_AddBoolToObject(root, "af_busy", af_busy);
    cJSON_AddNumberToObject(root, "af_pos", g_cfg.af_manual_pos);
    cJSON_AddNumberToObject(root, "edge_motion", g_last_edge_ai.motion_score);
    cJSON_AddNumberToObject(root, "edge_sharp", g_last_edge_ai.sharpness_score);
    cJSON_AddNumberToObject(root, "edge_luma", g_last_edge_ai.mean_luma);
    cJSON_AddStringToObject(root, "edge_led_state", g_last_edge_ai.edge_led_state);
    cJSON_AddStringToObject(root, "edge_digits", g_last_edge_ai.edge_digits);
    cJSON_AddStringToObject(root, "edge_detail", g_last_edge_ai.edge_detail);
    cJSON_AddNumberToObject(root, "edge_flow_lpm", g_last_edge_ai.edge_flow_lpm);
    cJSON_AddStringToObject(root, "edge_filter_state", g_last_edge_ai.edge_filter_state);
    cJSON_AddNumberToObject(root, "roi_luma", g_last_edge_ai.roi_luma);
    cJSON_AddNumberToObject(root, "roi_max_luma", g_last_edge_ai.roi_max_luma);
    cJSON_AddNumberToObject(root, "meter_mode", g_cfg.meter_mode);
    cJSON_AddNumberToObject(root, "roi_x_pct", g_cfg.roi_x_pct);
    cJSON_AddNumberToObject(root, "roi_y_pct", g_cfg.roi_y_pct);
    cJSON_AddNumberToObject(root, "roi_w_pct", g_cfg.roi_w_pct);
    cJSON_AddNumberToObject(root, "roi_h_pct", g_cfg.roi_h_pct);
    cJSON_AddStringToObject(root, "meter_baseline", g_cfg.meter_baseline);
    cJSON_AddNumberToObject(root, "meter_max_flow_lpm", g_cfg.meter_max_flow_lpm);
    cJSON_AddNumberToObject(root, "meter_rot_deg", g_cfg.meter_rot_deg);
    cJSON_AddStringToObject(root, "meter_serial", g_cfg.meter_serial[0] ? g_cfg.meter_serial : "00000198");
    cJSON_AddStringToObject(root, "meter_spec", g_cfg.meter_spec[0] ? g_cfg.meter_spec : "GB/T778-2018 R=80");
    cJSON_AddBoolToObject(root, "serial_auto_detect", g_cfg.serial_auto_detect);
    cJSON_AddNumberToObject(root, "flash_duty", app_flash_get_duty());
    cJSON_AddNumberToObject(root, "flash_mode", g_cfg.flash_mode);
    cJSON_AddNumberToObject(root, "flash_cap_duty", g_cfg.flash_cap_duty);
    cJSON_AddNumberToObject(root, "flash_warmup_ms", g_cfg.flash_warmup_ms);
    cJSON_AddNumberToObject(root, "auto_capture_sec", g_cfg.auto_capture_sec);
    cJSON_AddBoolToObject(root, "adaptive_cycle_enable", g_cfg.adaptive_cycle_enable);
    cJSON_AddNumberToObject(root, "fast_capture_sec", g_cfg.fast_capture_sec);
    cJSON_AddNumberToObject(root, "next_cap_sec", app_get_next_capture_countdown_sec());
    cJSON_AddBoolToObject(root, "adaptive_fast_active", app_is_adaptive_fast_active());
    cJSON_AddBoolToObject(root, "cam_sleeping", app_cam_is_sleeping());
    cJSON_AddBoolToObject(root, "eco_mode", g_cfg.eco_mode);
    cJSON_AddStringToObject(root, "ftp_status", g_last_ftp_status);
    cJSON_AddStringToObject(root, "ntfy_status", g_last_ntfy_status);
    cJSON_AddStringToObject(root, "last_ai", g_last_server_ai_json);

    // Industrial OTA, Partition & Rescue Mode Status
    cJSON_AddStringToObject(root, "running_part", h.running_part);
    cJSON_AddStringToObject(root, "next_ota_part", h.next_ota_part);
    cJSON_AddStringToObject(root, "ota_state", h.ota_state);
    cJSON_AddNumberToObject(root, "part_size_kb", h.part_size_kb);
    cJSON_AddNumberToObject(root, "crash_count", h.crash_count);
    cJSON_AddBoolToObject(root, "rescue_mode", h.rescue_mode);
    cJSON_AddBoolToObject(root, "wdt_active", h.wdt_active);

    // Industrial Camera Bus, Stack Watermark, CoreDump & Pipeline SLA
    cJSON_AddNumberToObject(root, "min_stack_bytes", h.min_stack_bytes);
    cJSON_AddStringToObject(root, "min_stack_task", h.min_stack_task);
    extern char g_cam_diag_str[160];
    cJSON_AddBoolToObject(root, "cam_i2c_ok", h.cam_i2c_ok);
    cJSON_AddNumberToObject(root, "cam_pid_hex", h.cam_pid_hex);
    cJSON_AddStringToObject(root, "cam_diag", g_cam_diag_str);
    cJSON_AddNumberToObject(root, "cam_recovery_cnt", h.cam_recovery_cnt);
    cJSON_AddBoolToObject(root, "coredump_present", h.coredump_present);
    cJSON_AddNumberToObject(root, "cam_isp_profile", g_cfg.cam_isp_profile);
    cJSON_AddNumberToObject(root, "cam_wb_mode", g_cfg.cam_wb_mode);
    cJSON_AddNumberToObject(root, "cam_sharpness", g_cfg.cam_sharpness);
    cJSON_AddNumberToObject(root, "cam_denoise", g_cfg.cam_denoise);
    cJSON_AddNumberToObject(root, "cam_gainceiling", g_cfg.cam_gainceiling);
    cJSON_AddBoolToObject(root, "cam_aec2", g_cfg.cam_aec2);
    cJSON_AddBoolToObject(root, "sd_auto_prune", g_cfg.sd_auto_prune);
    cJSON_AddNumberToObject(root, "sd_min_free_mb", g_cfg.sd_min_free_mb);
    cJSON_AddNumberToObject(root, "capture_ok_cnt", s_capture_ok_cnt);
    cJSON_AddNumberToObject(root, "capture_err_cnt", s_capture_err_cnt);
    cJSON_AddNumberToObject(root, "last_capture_ms", s_last_capture_ms);
    cJSON_AddNumberToObject(root, "sd_write_ok_cnt", s_sd_write_ok_cnt);
    cJSON_AddNumberToObject(root, "sd_write_err_cnt", s_sd_write_err_cnt);
    cJSON_AddNumberToObject(root, "ftp_ok_cnt", s_ftp_ok_cnt);
    cJSON_AddNumberToObject(root, "ftp_err_cnt", s_ftp_err_cnt);

    // Industrial WireGuard VPN Tunnel Status
    cJSON_AddBoolToObject(root, "wg_enabled", g_cfg.wg.enabled);
    cJSON_AddBoolToObject(root, "wg_connected", app_wireguard_is_connected());
    cJSON_AddStringToObject(root, "wg_ip", g_cfg.wg.address);
    cJSON_AddStringToObject(root, "wg_endpoint", g_cfg.wg.endpoint_host);
    cJSON_AddNumberToObject(root, "wg_port", g_cfg.wg.endpoint_port);
    cJSON_AddNumberToObject(root, "wg_mtu", app_wireguard_get_mtu());
    cJSON_AddNumberToObject(root, "wg_reconnect_cnt", app_wireguard_get_reconnect_cnt());
    cJSON_AddStringToObject(root, "wg_vault", app_vault_get_status_str());

    // Silicon Hardware Profile (SoC, Flash, PSRAM, Camera Sensor)
    esp_chip_info_t cinfo;
    esp_chip_info(&cinfo);
    uint32_t flash_bytes = 0;
    esp_flash_get_size(NULL, &flash_bytes);
    size_t psram_phys_bytes = esp_psram_get_size();
    cJSON_AddStringToObject(root, "hw_soc", (cinfo.revision >= 300) ? "ESP32-D0WD-V3 (Xtensa Dual-Core LX6)" : "ESP32-D0WDQ6 (Xtensa Dual-Core LX6)");
    cJSON_AddNumberToObject(root, "hw_cores", cinfo.cores);
    cJSON_AddNumberToObject(root, "hw_rev", cinfo.revision);
    cJSON_AddNumberToObject(root, "hw_flash_mb", (double)(flash_bytes / (1024 * 1024)));
    cJSON_AddNumberToObject(root, "hw_psram_phys_mb", (double)(psram_phys_bytes / (1024 * 1024)));
    cJSON_AddStringToObject(root, "hw_idf_ver", esp_get_idf_version());
    cJSON_AddStringToObject(root, "hw_cam_name", (h.cam_pid_hex == 0x5640) ? "OmniVision OV5640 5MP (VCM Auto-Focus)" : ((h.cam_pid_hex == 0x2640) ? "OmniVision OV2640 2MP" : "Generic DVP Sensor"));
    cJSON_AddNumberToObject(root, "hw_cam_xclk_mhz", (s && s->xclk_freq_hz > 0) ? (s->xclk_freq_hz / 1000000) : 10);
    cJSON_AddNumberToObject(root, "hw_cam_ver", s ? s->id.VER : 0);
    cJSON_AddNumberToObject(root, "rear_led_mode", app_get_rear_led_mode());
    cJSON_AddBoolToObject(root, "gpio12_relay", app_get_gpio12_relay());
    cJSON_AddNumberToObject(root, "gpio13_level", app_get_gpio13_sensor_level());
    cJSON_AddNumberToObject(root, "gpio13_pulses", app_get_gpio13_pulse_count());

    #undef h
    free(hp);
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, out, strlen(out));
    free(out);
    return ret;
}

static esp_err_t config_get_handler(httpd_req_t *req) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "mdns_host", g_cfg.mdns_host);
    cJSON_AddStringToObject(root, "device_name", g_cfg.device_name);
    cJSON_AddBoolToObject(root, "ap_always_on", g_cfg.ap_always_on);
    cJSON_AddNumberToObject(root, "active_wifi_slot", app_wifi_get_active_slot());

    cJSON *wlist = cJSON_CreateArray();
    for (int i = 0; i < MAX_WIFI_PROFILES; i++) {
        cJSON *w = cJSON_CreateObject();
        cJSON_AddStringToObject(w, "ssid", g_cfg.wifi_list[i].ssid);
        cJSON_AddStringToObject(w, "password", g_cfg.wifi_list[i].password);
        cJSON_AddNumberToObject(w, "enabled", g_cfg.wifi_list[i].enabled);
        cJSON_AddItemToArray(wlist, w);
    }
    cJSON_AddItemToObject(root, "wifi_list", wlist);

    cJSON_AddStringToObject(root, "ntp_server", g_cfg.ntp_server);
    cJSON_AddStringToObject(root, "tz_str", g_cfg.tz_str);
    cJSON_AddBoolToObject(root, "osd_enable", g_cfg.osd_enable);

    cJSON_AddBoolToObject(root, "ftp_enable", g_cfg.ftp_enable);
    cJSON_AddStringToObject(root, "ftp_host", g_cfg.ftp_host);
    cJSON_AddNumberToObject(root, "ftp_port", g_cfg.ftp_port);
    cJSON_AddStringToObject(root, "ftp_user", g_cfg.ftp_user);
    cJSON_AddStringToObject(root, "ftp_pass", g_cfg.ftp_pass);
    cJSON_AddStringToObject(root, "ftp_remote_dir", g_cfg.ftp_remote_dir);

    cJSON_AddBoolToObject(root, "mqtt_enable", g_cfg.mqtt_enable);
    cJSON_AddStringToObject(root, "mqtt_uri", g_cfg.mqtt_uri);
    cJSON_AddStringToObject(root, "mqtt_topic_prefix", g_cfg.mqtt_topic_prefix);
    cJSON_AddNumberToObject(root, "mqtt_interval_sec", g_cfg.mqtt_interval_sec);

    cJSON_AddBoolToObject(root, "ntfy_enable", g_cfg.ntfy_enable);
    cJSON_AddStringToObject(root, "ntfy_url", g_cfg.ntfy_url);

    cJSON_AddBoolToObject(root, "sd_save_enable", g_cfg.sd_save_enable);
    cJSON_AddNumberToObject(root, "auto_capture_sec", g_cfg.auto_capture_sec);
    cJSON_AddBoolToObject(root, "adaptive_cycle_enable", g_cfg.adaptive_cycle_enable);
    cJSON_AddNumberToObject(root, "fast_capture_sec", g_cfg.fast_capture_sec);

    cJSON_AddBoolToObject(root, "edge_ai_enable", g_cfg.edge_ai_enable);
    cJSON_AddNumberToObject(root, "edge_motion_thresh", g_cfg.edge_motion_thresh);
    cJSON_AddBoolToObject(root, "edge_trigger_capture", g_cfg.edge_trigger_capture);
    cJSON_AddStringToObject(root, "ai_server_url", g_cfg.ai_server_url);

    cJSON_AddNumberToObject(root, "meter_mode", g_cfg.meter_mode);
    cJSON_AddNumberToObject(root, "roi_x_pct", g_cfg.roi_x_pct);
    cJSON_AddNumberToObject(root, "roi_y_pct", g_cfg.roi_y_pct);
    cJSON_AddNumberToObject(root, "roi_w_pct", g_cfg.roi_w_pct);
    cJSON_AddNumberToObject(root, "roi_h_pct", g_cfg.roi_h_pct);
    cJSON_AddBoolToObject(root, "roi_draw_osd", g_cfg.roi_draw_osd);
    cJSON_AddStringToObject(root, "meter_baseline", g_cfg.meter_baseline);
    cJSON_AddNumberToObject(root, "meter_max_flow_lpm", g_cfg.meter_max_flow_lpm);
    cJSON_AddNumberToObject(root, "meter_rot_deg", g_cfg.meter_rot_deg);
    cJSON_AddStringToObject(root, "meter_serial", g_cfg.meter_serial[0] ? g_cfg.meter_serial : "00000198");
    cJSON_AddStringToObject(root, "meter_spec", g_cfg.meter_spec[0] ? g_cfg.meter_spec : "GB/T778-2018 R=80");
    cJSON_AddBoolToObject(root, "serial_auto_detect", g_cfg.serial_auto_detect);

    cJSON_AddNumberToObject(root, "cam_framesize", g_cfg.cam_framesize);
    cJSON_AddNumberToObject(root, "cam_quality", g_cfg.cam_quality);
    cJSON_AddNumberToObject(root, "cam_brightness", g_cfg.cam_brightness);
    cJSON_AddNumberToObject(root, "cam_contrast", g_cfg.cam_contrast);
    cJSON_AddNumberToObject(root, "cam_hmirror", g_cfg.cam_hmirror);
    cJSON_AddNumberToObject(root, "cam_vflip", g_cfg.cam_vflip);
    cJSON_AddNumberToObject(root, "af_manual_pos", g_cfg.af_manual_pos);
    cJSON_AddNumberToObject(root, "flash_duty", g_cfg.flash_duty);
    cJSON_AddNumberToObject(root, "flash_mode", g_cfg.flash_mode);
    cJSON_AddNumberToObject(root, "flash_cap_duty", g_cfg.flash_cap_duty);
    cJSON_AddNumberToObject(root, "flash_warmup_ms", g_cfg.flash_warmup_ms);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, out, strlen(out));
    free(out);
    return ret;
}

#define JSON_STR_COPY(obj, key, dst) do { \
    cJSON *it = cJSON_GetObjectItem(obj, key); \
    if (cJSON_IsString(it) && it->valuestring) { \
        strncpy(dst, it->valuestring, sizeof(dst) - 1); \
        dst[sizeof(dst) - 1] = '\0'; \
    } \
} while(0)

#define JSON_INT_SET(obj, key, dst) do { \
    cJSON *it = cJSON_GetObjectItem(obj, key); \
    if (cJSON_IsNumber(it)) dst = it->valueint; \
    else if (cJSON_IsBool(it)) dst = cJSON_IsTrue(it) ? 1 : 0; \
} while(0)

static esp_err_t config_post_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 4096) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char *buf = (char *)malloc(total_len + 1);
    if (!buf) return ESP_FAIL;
    int received = 0;
    while (received < total_len) {
        int r = httpd_req_recv(req, buf + received, total_len - received);
        if (r <= 0) { free(buf); return ESP_FAIL; }
        received += r;
    }
    buf[total_len] = '\0';

    cJSON *root = cJSON_Parse(buf);
    free(buf);
    if (!root) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char old_mqtt_uri[96];
    strncpy(old_mqtt_uri, g_cfg.mqtt_uri, sizeof(old_mqtt_uri));

    JSON_STR_COPY(root, "mdns_host", g_cfg.mdns_host);
    JSON_STR_COPY(root, "device_name", g_cfg.device_name);
    JSON_INT_SET(root, "ap_always_on", g_cfg.ap_always_on);

    JSON_STR_COPY(root, "ntp_server", g_cfg.ntp_server);
    JSON_STR_COPY(root, "tz_str", g_cfg.tz_str);
    JSON_INT_SET(root, "osd_enable", g_cfg.osd_enable);

    JSON_INT_SET(root, "ftp_enable", g_cfg.ftp_enable);
    JSON_STR_COPY(root, "ftp_host", g_cfg.ftp_host);
    JSON_INT_SET(root, "ftp_port", g_cfg.ftp_port);
    JSON_STR_COPY(root, "ftp_user", g_cfg.ftp_user);
    JSON_STR_COPY(root, "ftp_pass", g_cfg.ftp_pass);
    JSON_STR_COPY(root, "ftp_remote_dir", g_cfg.ftp_remote_dir);

    JSON_INT_SET(root, "mqtt_enable", g_cfg.mqtt_enable);
    JSON_STR_COPY(root, "mqtt_uri", g_cfg.mqtt_uri);
    JSON_STR_COPY(root, "mqtt_topic_prefix", g_cfg.mqtt_topic_prefix);
    JSON_INT_SET(root, "mqtt_interval_sec", g_cfg.mqtt_interval_sec);

    JSON_INT_SET(root, "ntfy_enable", g_cfg.ntfy_enable);
    JSON_STR_COPY(root, "ntfy_url", g_cfg.ntfy_url);

    JSON_INT_SET(root, "sd_save_enable", g_cfg.sd_save_enable);
    JSON_INT_SET(root, "auto_capture_sec", g_cfg.auto_capture_sec);
    JSON_INT_SET(root, "adaptive_cycle_enable", g_cfg.adaptive_cycle_enable);
    JSON_INT_SET(root, "fast_capture_sec", g_cfg.fast_capture_sec);

    JSON_INT_SET(root, "edge_ai_enable", g_cfg.edge_ai_enable);
    JSON_INT_SET(root, "edge_motion_thresh", g_cfg.edge_motion_thresh);
    JSON_INT_SET(root, "edge_trigger_capture", g_cfg.edge_trigger_capture);
    JSON_STR_COPY(root, "ai_server_url", g_cfg.ai_server_url);

    JSON_INT_SET(root, "meter_mode", g_cfg.meter_mode);
    JSON_INT_SET(root, "roi_x_pct", g_cfg.roi_x_pct);
    JSON_INT_SET(root, "roi_y_pct", g_cfg.roi_y_pct);
    JSON_INT_SET(root, "roi_w_pct", g_cfg.roi_w_pct);
    JSON_INT_SET(root, "roi_h_pct", g_cfg.roi_h_pct);
    JSON_INT_SET(root, "roi_draw_osd", g_cfg.roi_draw_osd);
    JSON_STR_COPY(root, "meter_baseline", g_cfg.meter_baseline);
    JSON_INT_SET(root, "meter_max_flow_lpm", g_cfg.meter_max_flow_lpm);
    JSON_INT_SET(root, "meter_rot_deg", g_cfg.meter_rot_deg);
    JSON_STR_COPY(root, "meter_serial", g_cfg.meter_serial);
    JSON_STR_COPY(root, "meter_spec", g_cfg.meter_spec);
    JSON_INT_SET(root, "serial_auto_detect", g_cfg.serial_auto_detect);
    JSON_INT_SET(root, "flash_mode", g_cfg.flash_mode);
    JSON_INT_SET(root, "flash_cap_duty", g_cfg.flash_cap_duty);
    JSON_INT_SET(root, "flash_warmup_ms", g_cfg.flash_warmup_ms);

    cJSON *wlist = cJSON_GetObjectItem(root, "wifi_list");
    if (cJSON_IsArray(wlist)) {
        int cnt = cJSON_GetArraySize(wlist);
        for (int i = 0; i < cnt && i < MAX_WIFI_PROFILES; i++) {
            cJSON *w = cJSON_GetArrayItem(wlist, i);
            JSON_STR_COPY(w, "ssid", g_cfg.wifi_list[i].ssid);
            JSON_STR_COPY(w, "password", g_cfg.wifi_list[i].password);
            JSON_INT_SET(w, "enabled", g_cfg.wifi_list[i].enabled);
        }
    }

    cJSON_Delete(root);
    app_config_save();

    if (strcmp(old_mqtt_uri, g_cfg.mqtt_uri) != 0) {
        net_mqtt_restart();
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"status\":\"ok\"}", -1);
}

static esp_err_t time_post_handler(httpd_req_t *req) {
    char query[96];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[32];
        if (httpd_query_key_value(query, "epoch", val, sizeof(val)) == ESP_OK) {
            int64_t epoch = atoll(val);
            if (epoch > 1700000000LL) {
                net_set_manual_epoch(epoch);
            }
        } else if (httpd_query_key_value(query, "ntp", val, sizeof(val)) == ESP_OK) {
            net_ntp_restart();
        }
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"status\":\"ok\"}", -1);
}

static esp_err_t wifi_scan_handler(httpd_req_t *req) {
    wifi_scan_config_t scan_cfg = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = false
    };
    esp_wifi_scan_start(&scan_cfg, true);
    uint16_t ap_num = 15;
    wifi_ap_record_t ap_records[15];
    esp_wifi_scan_get_ap_records(&ap_num, ap_records);

    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < ap_num; i++) {
        if (ap_records[i].ssid[0] == '\0') continue;
        cJSON *it = cJSON_CreateObject();
        cJSON_AddStringToObject(it, "ssid", (char *)ap_records[i].ssid);
        cJSON_AddNumberToObject(it, "rssi", ap_records[i].rssi);
        cJSON_AddNumberToObject(it, "channel", ap_records[i].primary);
        cJSON_AddItemToArray(arr, it);
    }
    char *out = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, out, strlen(out));
    free(out);
    return ret;
}

static esp_err_t capture_handler(httpd_req_t *req) {
    char mode[32] = "all";
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "mode", mode, sizeof(mode));
    }

    char filename[64] = {0};
    char ftp_st[64] = {0};
    esp_err_t err = execute_capture_pipeline(mode, filename, sizeof(filename), ftp_st, sizeof(ftp_st));

    char out_json[256];
    if (err == ESP_OK) {
        snprintf(out_json, sizeof(out_json),
                 "{\"status\":\"ok\",\"file\":\"%s\",\"ftp\":\"%s\"}", filename, ftp_st);
    } else {
        snprintf(out_json, sizeof(out_json), "{\"status\":\"error\"}");
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, out_json, strlen(out_json));
}

// Instant Hardware-JPEG Snapshot (~35ms) for Web UI / 4G VPN / AI Server polling
// Pass ?fresh=1 to pulse Flash Assist + VCM Focus for an on-demand night snapshot
// Pass ?osd=1 if burned-in OSD is explicitly needed
static esp_err_t snapshot_handler(httpd_req_t *req) {
    bool want_osd = false;
    bool want_fresh = false;
    char query[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        char val[16];
        if (httpd_query_key_value(query, "osd", val, sizeof(val)) == ESP_OK && atoi(val) == 1) {
            want_osd = true;
        }
        if (httpd_query_key_value(query, "fresh", val, sizeof(val)) == ESP_OK && atoi(val) == 1) {
            want_fresh = true;
        }
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=snapshot.jpg");

    // Serve PSRAM-cached illuminated & focused frame when idle/dark so night view never turns black and camera stays cool
    if (!want_fresh && !want_osd && app_flash_get_duty() == 0 &&
        s_last_lit_jpg_buf != NULL && s_last_lit_jpg_len > 0) {
        return httpd_resp_send(req, (const char *)s_last_lit_jpg_buf, s_last_lit_jpg_len);
    }

    if (s_cam_mutex) xSemaphoreTake(s_cam_mutex, pdMS_TO_TICKS(2500));
    app_cam_wake_if_sleeping();
    app_cam_touch_activity();

    bool do_assist = (want_fresh || s_last_lit_jpg_buf == NULL);
    bool flash_assisted = false;
    if (do_assist) {
        flash_assisted = app_flash_pre_capture_assist();
    }
    sensor_t *s_snap = esp_camera_sensor_get();
    if (s_snap && s_snap->af_set_manual_position && g_cfg.af_manual_pos > 0) {
        if (do_assist || app_flash_get_duty() > 0) {
            s_snap->af_set_manual_position(s_snap, g_cfg.af_manual_pos);
            if (do_assist) {
                vTaskDelay(pdMS_TO_TICKS(110));
                for (int f = 0; f < 2; f++) {
                    camera_fb_t *stale = esp_camera_fb_get();
                    if (stale) esp_camera_fb_return(stale);
                }
            }
        }
    }

    camera_fb_t *fb = esp_camera_fb_get();
    if (app_cam_get_stream_clients() == 0 && app_flash_get_duty() == 0) {
        app_cam_release_vcm_coil();
    }
    if (do_assist) {
        app_flash_post_capture_restore(flash_assisted);
    }
    if (!fb) {
        if (s_cam_mutex) xSemaphoreGive(s_cam_mutex);
        if (s_last_lit_jpg_buf && s_last_lit_jpg_len > 0) {
            return httpd_resp_send(req, (const char *)s_last_lit_jpg_buf, s_last_lit_jpg_len);
        }
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    cache_lit_jpeg_in_psram(fb->buf, fb->len);

    esp_err_t res = ESP_OK;
    if (want_osd) {
        uint8_t *jpg_buf = NULL;
        size_t jpg_len = 0;
        bool allocated = false;
        process_frame_osd_and_ai(fb->buf, fb->len, fb->width, fb->height, 85, NULL, &jpg_buf, &jpg_len, &allocated);
        res = httpd_resp_send(req, (const char *)jpg_buf, jpg_len);
        if (allocated && jpg_buf) free(jpg_buf);
    } else {
        res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }

    esp_camera_fb_return(fb);
    if (s_cam_mutex) xSemaphoreGive(s_cam_mutex);
    return res;
}

static esp_err_t list_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, "[", 1);
    DIR *dir = opendir("/sdcard");
    bool first = true;
    if (dir) {
        struct dirent *de;
        while ((de = readdir(dir)) != NULL) {
            if (de->d_name[0] == '.' ||
                strcmp(de->d_name, "wg_vault.conf") == 0 ||
                strcmp(de->d_name, "System Volume Information") == 0) continue;
            if (de->d_type == DT_REG) {
                struct stat st;
                char path[300];
                snprintf(path, sizeof(path), "/sdcard/%s", de->d_name);
                if (stat(path, &st) == 0) {
                    char item[384];
                    snprintf(item, sizeof(item), "%s{\"name\":\"%s\",\"size\":%ld,\"type\":\"file\"}",
                             first ? "" : ",", de->d_name, (long)st.st_size);
                    httpd_resp_send_chunk(req, item, strlen(item));
                    first = false;
                }
            } else if (de->d_type == DT_DIR && strcmp(de->d_name, ".") != 0 && strcmp(de->d_name, "..") != 0) {
                char item[384];
                snprintf(item, sizeof(item), "%s{\"name\":\"%s/\",\"size\":0,\"type\":\"dir\"}",
                         first ? "" : ",", de->d_name);
                httpd_resp_send_chunk(req, item, strlen(item));
                first = false;
            }
        }
        closedir(dir);
    }
    httpd_resp_send_chunk(req, "]", 1);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t sdcard_file_handler(httpd_req_t *req) {
    // URI is /sdcard/filename.jpg
    const char *uri = req->uri;
    if (strstr(uri, "..")) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    FILE *f = fopen(uri, "rb");
    if (!f) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    char *chunk = (char *)malloc(4096);
    if (!chunk) { fclose(f); return ESP_FAIL; }
    size_t read_bytes;
    while ((read_bytes = fread(chunk, 1, 4096, f)) > 0) {
        if (httpd_resp_send_chunk(req, chunk, read_bytes) != ESP_OK) break;
    }
    free(chunk);
    fclose(f);
    return httpd_resp_send_chunk(req, NULL, 0);
}

static esp_err_t delete_handler(httpd_req_t *req) {
    char query[128], file[64];
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK &&
        httpd_query_key_value(query, "file", file, sizeof(file)) == ESP_OK) {
        if (!strstr(file, "..") && !strchr(file, '/')) {
            char path[96];
            snprintf(path, sizeof(path), "/sdcard/%s", file);
            unlink(path);
            ESP_LOGI(TAG, "Deleted SD file: %s", path);
        }
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"status\":\"ok\"}", -1);
}

static esp_err_t cmd_handler(httpd_req_t *req) {
    char var[32], val[32], src[96];
    if (httpd_req_get_url_query_str(req, src, sizeof(src)) == ESP_OK) {
        if (httpd_query_key_value(src, "var", var, sizeof(var)) == ESP_OK &&
            httpd_query_key_value(src, "val", val, sizeof(val)) == ESP_OK) {
            int v = atoi(val);
            app_cam_wake_if_sleeping();
            sensor_t *s = esp_camera_sensor_get();
            if (s) {
                if (!strcmp(var, "focus")) {
                    g_cfg.af_manual_pos = (uint16_t)v;
                    g_cfg.af_mode = 1;
                    if (s->af_set_manual_position) s->af_set_manual_position(s, (uint16_t)v);
                    app_config_save();
                } else if (!strcmp(var, "af_trigger")) {
                    uint16_t locked_vcm = app_cam_trigger_af_and_lock();
                    char rbuf[96];
                    snprintf(rbuf, sizeof(rbuf), "{\"status\":\"ok\",\"locked_vcm\":%u}", (unsigned)locked_vcm);
                    httpd_resp_set_type(req, "application/json");
                    return httpd_resp_send(req, rbuf, strlen(rbuf));
                } else if (!strcmp(var, "af_roi_sweep")) {
                    uint16_t best_vcm = app_cam_roi_focus_sweep();
                    char rbuf[96];
                    snprintf(rbuf, sizeof(rbuf), "{\"status\":\"ok\",\"best_vcm\":%u,\"sharpness\":%.1f}",
                             best_vcm, g_last_edge_ai.sharpness_score);
                    httpd_resp_set_type(req, "application/json");
                    return httpd_resp_send(req, rbuf, strlen(rbuf));
                } else if (!strcmp(var, "eco_mode")) {
                    g_cfg.eco_mode = (v != 0);
                    app_config_save();
                } else if (!strcmp(var, "af_mode")) {
                    g_cfg.af_mode = (uint8_t)v;
                    if (s->af_set_mode) s->af_set_mode(s, v);
                    app_config_save();
                } else if (!strcmp(var, "framesize")) {
                    g_cfg.cam_framesize = (uint8_t)v;
                    s->set_framesize(s, (framesize_t)v);
                    app_config_save();
                } else if (!strcmp(var, "quality")) {
                    g_cfg.cam_quality = (uint8_t)v;
                    s->set_quality(s, v);
                    app_config_save();
                } else if (!strcmp(var, "brightness")) {
                    g_cfg.cam_brightness = (int8_t)v;
                    s->set_brightness(s, v);
                    app_config_save();
                } else if (!strcmp(var, "contrast")) {
                    g_cfg.cam_contrast = (int8_t)v;
                    s->set_contrast(s, v);
                    app_config_save();
                } else if (!strcmp(var, "saturation")) {
                    g_cfg.cam_saturation = (int8_t)v;
                    s->set_saturation(s, v);
                    app_config_save();
                } else if (!strcmp(var, "hmirror")) {
                    g_cfg.cam_hmirror = (uint8_t)v;
                    s->set_hmirror(s, v);
                    app_config_save();
                } else if (!strcmp(var, "vflip")) {
                    g_cfg.cam_vflip = (uint8_t)v;
                    s->set_vflip(s, v);
                    app_config_save();
                } else if (!strcmp(var, "isp_profile")) {
                    app_apply_cam_isp_profile((uint8_t)v);
                    app_config_save();
                } else if (!strcmp(var, "wb_mode")) {
                    g_cfg.cam_wb_mode = (uint8_t)v;
                    if (s->set_wb_mode) s->set_wb_mode(s, v);
                    app_config_save();
                } else if (!strcmp(var, "sharpness")) {
                    g_cfg.cam_sharpness = (int8_t)v;
                    if (s->set_sharpness) s->set_sharpness(s, v);
                    app_config_save();
                } else if (!strcmp(var, "denoise")) {
                    g_cfg.cam_denoise = (uint8_t)v;
                    if (s->set_denoise) s->set_denoise(s, v);
                    app_config_save();
                } else if (!strcmp(var, "gainceiling")) {
                    g_cfg.cam_gainceiling = (uint8_t)v;
                    if (s->set_gainceiling) s->set_gainceiling(s, (gainceiling_t)v);
                    app_config_save();
                } else if (!strcmp(var, "aec2")) {
                    g_cfg.cam_aec2 = (v != 0);
                    if (s->set_aec2) s->set_aec2(s, g_cfg.cam_aec2 ? 1 : 0);
                    app_config_save();
                }
            }
            if (!strcmp(var, "wifi_slot")) {
                app_wifi_connect_slot(v);
            } else if (!strcmp(var, "target_preset")) {
                app_apply_target_preset(val);
            } else if (!strcmp(var, "test_ntfy")) {
                net_ntfy_send("ESP32-CAM Test", "Kết nối thông báo đẩy Ntfy từ ESP32-CAM thành công!", "default", "white_check_mark,camera", "http://10.25.7.171:8095");
            } else if (!strcmp(var, "meter_mode")) {
                g_cfg.meter_mode = (uint8_t)v;
                if (v == 4) {
                    app_apply_cam_isp_profile(4); // Dial Water Meter Anti-Glare + Red Pointer Boost
                }
                app_config_save();
            } else if (!strcmp(var, "flash_duty")) {
                if (v < 0) v = 0;
                if (v > 255) v = 255;
                g_cfg.flash_duty = (uint8_t)v;
                app_flash_set_duty(g_cfg.flash_duty);
            } else if (!strcmp(var, "flash_mode")) {
                g_cfg.flash_mode = (uint8_t)v;
                app_config_save();
            } else if (!strcmp(var, "flash_cap_duty")) {
                if (v < 10) v = 10;
                if (v > 255) v = 255;
                g_cfg.flash_cap_duty = (uint8_t)v;
                app_config_save();
            } else if (!strcmp(var, "auto_capture_sec")) {
                g_cfg.auto_capture_sec = (uint32_t)(v >= 0 ? v : 0);
                app_config_save();
            } else if (!strcmp(var, "adaptive_cycle")) {
                g_cfg.adaptive_cycle_enable = (v != 0);
                app_config_save();
            } else if (!strcmp(var, "fast_capture_sec")) {
                g_cfg.fast_capture_sec = (uint16_t)(v >= 5 ? v : 30);
                app_config_save();
            } else if (!strcmp(var, "meter_baseline")) {
                strncpy(g_cfg.meter_baseline, val, sizeof(g_cfg.meter_baseline) - 1);
                g_cfg.meter_baseline[sizeof(g_cfg.meter_baseline) - 1] = '\0';
                app_config_save();
            } else if (!strcmp(var, "meter_serial")) {
                strncpy(g_cfg.meter_serial, val, sizeof(g_cfg.meter_serial) - 1);
                g_cfg.meter_serial[sizeof(g_cfg.meter_serial) - 1] = '\0';
                app_config_save();
                net_mqtt_publish_telemetry();
            } else if (!strcmp(var, "meter_spec")) {
                // URL decode %20 to space
                size_t j = 0;
                for (size_t i = 0; val[i] && j + 1 < sizeof(g_cfg.meter_spec); i++) {
                    if (val[i] == '%' && val[i+1] == '2' && val[i+2] == '0') {
                        g_cfg.meter_spec[j++] = ' ';
                        i += 2;
                    } else if (val[i] == '+') {
                        g_cfg.meter_spec[j++] = ' ';
                    } else {
                        g_cfg.meter_spec[j++] = val[i];
                    }
                }
                g_cfg.meter_spec[j] = '\0';
                app_config_save();
                net_mqtt_publish_telemetry();
            } else if (!strcmp(var, "serial_auto")) {
                g_cfg.serial_auto_detect = (v != 0);
                app_config_save();
            } else if (!strcmp(var, "meter_rot_deg")) {
                if (v < -180) v = -180;
                if (v > 180) v = 180;
                g_cfg.meter_rot_deg = (int16_t)v;
                app_config_save();
            } else if (!strcmp(var, "roi_x")) {
                g_cfg.roi_x_pct = (uint8_t)v;
            } else if (!strcmp(var, "roi_y")) {
                g_cfg.roi_y_pct = (uint8_t)v;
            } else if (!strcmp(var, "roi_w")) {
                g_cfg.roi_w_pct = (uint8_t)v;
            } else if (!strcmp(var, "roi_h")) {
                g_cfg.roi_h_pct = (uint8_t)v;
            } else if (!strcmp(var, "roi_save")) {
                app_config_save();
            } else if (!strcmp(var, "rescue_mode")) {
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, "{\"status\":\"ok\",\"msg\":\"rebooting_rescue\"}", -1);
                app_set_rescue_mode_reboot(v != 0);
                return ESP_OK;
            } else if (!strcmp(var, "reboot")) {
                httpd_resp_set_type(req, "application/json");
                httpd_resp_send(req, "{\"status\":\"ok\",\"msg\":\"rebooting\"}", -1);
                vTaskDelay(pdMS_TO_TICKS(300));
                esp_restart();
                return ESP_OK;
            } else if (!strcmp(var, "factory_reset")) {
                app_config_factory_reset_preserve_vpn();
            } else if (!strcmp(var, "sd_format")) {
                app_format_sd_card();
            } else if (!strcmp(var, "sd_prune")) {
                app_prune_oldest_sd_files(v > 0 ? v : 5);
            } else if (!strcmp(var, "sd_auto_prune")) {
                g_cfg.sd_auto_prune = (v != 0);
                app_config_save();
            } else if (!strcmp(var, "cam_recover")) {
                app_recover_camera_hardware();
            } else if (!strcmp(var, "rear_led_mode")) {
                app_set_rear_led_mode((uint8_t)v);
            } else if (!strcmp(var, "gpio12_relay")) {
                app_set_gpio12_relay(v != 0);
            } else if (!strcmp(var, "gpio13_reset")) {
                app_reset_gpio13_pulses();
            }
        }
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"status\":\"ok\"}", -1);
}


static esp_err_t stream_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    char part_buf[64];
    uint32_t frame_idx = 0;

    httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    app_cam_set_stream_clients(1);
    app_cam_wake_if_sleeping();
    sensor_t *s_st = esp_camera_sensor_get();
    if (s_st && s_st->af_set_manual_position && g_cfg.af_manual_pos > 0) {
        s_st->af_set_manual_position(s_st, g_cfg.af_manual_pos);
    }

    // Smart Night Fill-Light during Live MJPEG stream (auto turns OFF when stream closes)
    bool stream_auto_lit = false;
    if (g_cfg.flash_mode >= 1 && app_flash_get_duty() == 0 && g_last_edge_ai.mean_luma < 32) {
        uint8_t cap_duty = g_cfg.flash_cap_duty ? g_cfg.flash_cap_duty : 28;
        app_flash_set_duty(cap_duty);
        stream_auto_lit = true;
    }

    while (res == ESP_OK) {
        if (s_cam_mutex) xSemaphoreTake(s_cam_mutex, pdMS_TO_TICKS(1000));
        app_cam_touch_activity();
        fb = esp_camera_fb_get();
        if (!fb) {
            if (s_cam_mutex) xSemaphoreGive(s_cam_mutex);
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }

        // Run lightweight Edge AI motion check every 20 frames (~2.5 sec)
        frame_idx++;
        if (frame_idx == 2 || (frame_idx % 20 == 0)) {
            cache_lit_jpeg_in_psram(fb->buf, fb->len);
            if (g_cfg.edge_ai_enable && fb->width <= 800) {
                uint8_t *dummy_out = NULL;
                size_t dummy_len = 0;
                bool dummy_alloc = false;
                bool old_osd = g_cfg.osd_enable;
                g_cfg.osd_enable = false; // Only compute Edge AI metrics during stream
                process_frame_osd_and_ai(fb->buf, fb->len, fb->width, fb->height, 80, NULL, &dummy_out, &dummy_len, &dummy_alloc);
                g_cfg.osd_enable = old_osd;
                if (dummy_alloc && dummy_out) free(dummy_out);
                // If scene turned dark while streaming in Smart Auto mode, enable soft fill-light
                if (!stream_auto_lit && g_cfg.flash_mode >= 1 && app_flash_get_duty() == 0 && g_last_edge_ai.mean_luma < 22) {
                    uint8_t cap_duty = g_cfg.flash_cap_duty ? g_cfg.flash_cap_duty : 28;
                    app_flash_set_duty(cap_duty);
                    stream_auto_lit = true;
                }
            }
        }

        size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, (unsigned)fb->len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }

        esp_camera_fb_return(fb);
        if (s_cam_mutex) xSemaphoreGive(s_cam_mutex);
        fb = NULL;
        // Thermal Eco pacing: 85ms (~10-12 FPS) in Eco Mode vs 20ms in Turbo Mode
        vTaskDelay(pdMS_TO_TICKS(g_cfg.eco_mode ? 85 : 20));
    }
    if (stream_auto_lit && g_cfg.flash_duty == 0) {
        app_flash_set_duty(0);
    }
    app_cam_set_stream_clients(-1);
    return res;
}

void start_camera_server(void) {
    if (s_web_httpd != NULL) return; // Already started
    if (!s_cam_mutex) s_cam_mutex = xSemaphoreCreateMutex();

    // 1. Main Control & API Server on Port 80
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.stack_size = 12288;
    config.max_uri_handlers = 30;
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    config.uri_match_fn = httpd_uri_match_wildcard;

    httpd_uri_t uris[] = {
        {"/", HTTP_GET, index_handler, NULL},
        {"/status", HTTP_GET, status_handler, NULL},
        {"/api/config", HTTP_GET, config_get_handler, NULL},
        {"/api/config", HTTP_POST, config_post_handler, NULL},
        {"/api/time", HTTP_POST, time_post_handler, NULL},
        {"/api/wifi/scan", HTTP_GET, wifi_scan_handler, NULL},
        {"/api/wireguard", HTTP_GET, wg_get_handler, NULL},
        {"/api/wireguard", HTTP_POST, wg_post_handler, NULL},
        {"/api/ota/upload", HTTP_POST, ota_upload_handler, NULL},
        {"/api/ota/pull", HTTP_POST, ota_pull_handler, NULL},
        {"/api/ota/rollback", HTTP_POST, ota_rollback_handler, NULL},
        {"/api/tasks", HTTP_GET, tasks_handler, NULL},
        {"/api/ai/manifest", HTTP_GET, ai_manifest_handler, NULL},
        {"/api/ai/observe", HTTP_GET, ai_observe_handler, NULL},
        {"/mcp", HTTP_GET, mcp_handler, NULL},
        {"/mcp", HTTP_POST, mcp_handler, NULL},
        {"/capture", HTTP_GET, capture_handler, NULL},
        {"/snapshot", HTTP_GET, snapshot_handler, NULL},
        {"/list", HTTP_GET, list_handler, NULL},
        {"/delete", HTTP_GET, delete_handler, NULL},
        {"/control", HTTP_GET, cmd_handler, NULL},
        {"/sdcard/*", HTTP_GET, sdcard_file_handler, NULL},
    };

    if (httpd_start(&s_web_httpd, &config) == ESP_OK) {
        for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
            httpd_register_uri_handler(s_web_httpd, &uris[i]);
        }
        ESP_LOGI(TAG, "Web Control Server started on port 80 (LRU purge enabled)");
    }

    // 2. Dedicated MJPEG Stream Server ONLY on Port 81 (never block Port 80 API thread!)
    httpd_uri_t stream_uri = {"/stream", HTTP_GET, stream_handler, NULL};

    httpd_config_t stream_cfg = HTTPD_DEFAULT_CONFIG();
    stream_cfg.server_port = 81;
    stream_cfg.ctrl_port = 32769;
    stream_cfg.stack_size = 8192;
    stream_cfg.max_open_sockets = 3;
    stream_cfg.lru_purge_enable = true;
    stream_cfg.recv_wait_timeout = 4;
    stream_cfg.send_wait_timeout = 4;
    if (httpd_start(&s_stream_httpd, &stream_cfg) == ESP_OK) {
        httpd_register_uri_handler(s_stream_httpd, &stream_uri);
        ESP_LOGI(TAG, "Dedicated Stream Server started on port 81");
    }
}
