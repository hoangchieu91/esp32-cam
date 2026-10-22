#include "httpd_ota_sys.h"
#include "app_httpd.h"
#include "app_config.h"
#include "app_wireguard.h"

#include "esp_http_client.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "esp_log.h"
#include "cJSON.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "httpd_ota_sys";

// ============================================================================
// INDUSTRIAL DUAL-OTA HANDLERS (WEB UPLOAD, SERVER PULL, ROLLBACK RESCUE)
// ============================================================================
static void delayed_reboot_task(void *arg) {
    vTaskDelay(pdMS_TO_TICKS(800));
    esp_restart();
    vTaskDelete(NULL);
}

esp_err_t ota_upload_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    if (total_len < 100000 || total_len > 2020000) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid firmware size (must be 100KB..1.98MB)");
        return ESP_FAIL;
    }

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No passive OTA partition found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting Web OTA Upload (%d bytes) to partition '%s' @ 0x%lx...",
             total_len, update_part->label, (unsigned long)update_part->address);

    esp_ota_handle_t ota_handle = 0;
    esp_err_t err = esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
        return ESP_FAIL;
    }

    char *buf = (char *)malloc(4096);
    if (!buf) {
        esp_ota_abort(ota_handle);
        return ESP_FAIL;
    }

    int received = 0;
    bool header_checked = false;
    while (received < total_len) {
        int to_read = (total_len - received > 4096) ? 4096 : (total_len - received);
        int r = httpd_req_recv(req, buf, to_read);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            ESP_LOGE(TAG, "OTA recv error at %d/%d", received, total_len);
            free(buf);
            esp_ota_abort(ota_handle);
            return ESP_FAIL;
        }
        if (!header_checked) {
            if ((uint8_t)buf[0] != ESP_IMAGE_HEADER_MAGIC) {
                ESP_LOGE(TAG, "Invalid ESP32 Firmware Magic Byte: 0x%02x (expected 0xE9)", (uint8_t)buf[0]);
                free(buf);
                esp_ota_abort(ota_handle);
                httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid .bin magic header (not 0xE9)");
                return ESP_FAIL;
            }
            header_checked = true;
        }
        err = esp_ota_write(ota_handle, buf, r);
        if (err != ESP_OK) {
            free(buf);
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_write failed");
            return ESP_FAIL;
        }
        received += r;
    }
    free(buf);

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end verification failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Firmware checksum/image verification failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_set_boot_partition failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA Upload succeeded! Switching boot partition to '%s' and rebooting...", update_part->label);
    char resp[128];
    snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"next_part\":\"%s\",\"bytes\":%d}", update_part->label, received);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, resp, strlen(resp));

    xTaskCreate(delayed_reboot_task, "ota_rst", 2048, NULL, 5, NULL);
    return ESP_OK;
}

esp_err_t ota_pull_handler(httpd_req_t *req) {
    char query[192] = {0}, url[160] = "http://10.25.7.171:8095/firmware/cam_streamer.bin";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "url", url, sizeof(url));
    }

    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
    if (!update_part) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Pulling OTA Firmware from %s -> partition '%s'...", url, update_part->label);
    esp_http_client_config_t cfg = {
        .url = url,
        .timeout_ms = 20000,
    };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);
    if (!client || esp_http_client_open(client, 0) != ESP_OK) {
        if (client) esp_http_client_cleanup(client);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"msg\":\"Cannot connect to firmware URL\"}", -1);
    }

    int content_len = esp_http_client_fetch_headers(client);
    int status_code = esp_http_client_get_status_code(client);
    if (status_code != 200 || content_len < 100000) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"msg\":\"Firmware file not found on server (.171)\"}", -1);
    }

    esp_ota_handle_t ota_handle = 0;
    if (esp_ota_begin(update_part, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle) != ESP_OK) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    char *buf = (char *)malloc(4096);
    int total_read = 0;
    while (1) {
        int r = esp_http_client_read(client, buf, 4096);
        if (r <= 0) break;
        if (esp_ota_write(ota_handle, buf, r) != ESP_OK) {
            total_read = -1;
            break;
        }
        total_read += r;
    }
    free(buf);
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (total_read > 100000 && esp_ota_end(ota_handle) == ESP_OK && esp_ota_set_boot_partition(update_part) == ESP_OK) {
        char resp[128];
        snprintf(resp, sizeof(resp), "{\"status\":\"ok\",\"next_part\":\"%s\",\"bytes\":%d}", update_part->label, total_read);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, resp, strlen(resp));
        xTaskCreate(delayed_reboot_task, "ota_rst", 2048, NULL, 5, NULL);
        return ESP_OK;
    } else {
        esp_ota_abort(ota_handle);
        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"status\":\"error\",\"msg\":\"OTA Pull write/verify failed\"}", -1);
    }
}

esp_err_t ota_rollback_handler(httpd_req_t *req) {
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    esp_app_desc_t other_desc;
    if (other && esp_ota_get_partition_description(other, &other_desc) == ESP_OK) {
        esp_ota_set_boot_partition(other);
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, "{\"status\":\"ok\",\"msg\":\"Rolling back to alternate OTA partition\"}", -1);
        xTaskCreate(delayed_reboot_task, "ota_rst", 2048, NULL, 5, NULL);
        return ESP_OK;
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"status\":\"error\",\"msg\":\"Phân vùng dự phòng chưa có bản firmware hợp lệ để Rollback\"}", -1);
}

// ============================================================================
// FREERTOS TASK & STACK HIGH-WATER MARK PROFILER
// ============================================================================
esp_err_t tasks_handler(httpd_req_t *req) {
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *t_arr = (TaskStatus_t *)malloc(num_tasks * sizeof(TaskStatus_t));
    if (!t_arr) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    uint32_t total_rt = 0;
    UBaseType_t fetched = uxTaskGetSystemState(t_arr, num_tasks, &total_rt);
    if (total_rt == 0) total_rt = 1;

    cJSON *arr = cJSON_CreateArray();
    for (UBaseType_t i = 0; i < fetched; i++) {
        cJSON *it = cJSON_CreateObject();
        cJSON_AddStringToObject(it, "name", t_arr[i].pcTaskName);
        cJSON_AddNumberToObject(it, "prio", (int)t_arr[i].uxCurrentPriority);
        cJSON_AddNumberToObject(it, "stack_hwm", (int)t_arr[i].usStackHighWaterMark);
        const char *st = "Run";
        switch (t_arr[i].eCurrentState) {
            case eRunning:   st = "Running"; break;
            case eReady:     st = "Ready"; break;
            case eBlocked:   st = "Blocked"; break;
            case eSuspended: st = "Suspended"; break;
            case eDeleted:   st = "Deleted"; break;
            default:         st = "Idle"; break;
        }
        cJSON_AddStringToObject(it, "state", st);
#ifdef CONFIG_FREERTOS_VTASKLIST_INCLUDE_COREID
        int core = (int)t_arr[i].xCoreID;
        if (core == 0) cJSON_AddStringToObject(it, "core", "Core 0");
        else if (core == 1) cJSON_AddStringToObject(it, "core", "Core 1");
        else cJSON_AddStringToObject(it, "core", "Any");
#else
        cJSON_AddStringToObject(it, "core", "Dual");
#endif
        double pct = ((double)t_arr[i].ulRunTimeCounter * 100.0) / (double)total_rt;
        cJSON_AddNumberToObject(it, "cpu_pct", ((int)(pct * 10.0)) / 10.0);
        cJSON_AddItemToArray(arr, it);
    }
    free(t_arr);

    char *out = cJSON_PrintUnformatted(arr);
    cJSON_Delete(arr);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, out, strlen(out));
    free(out);
    return ret;
}

// ============================================================================
// WIREGUARD VPN CONFIGURATION ENDPOINTS
// ============================================================================
esp_err_t wg_get_handler(httpd_req_t *req) {
    bool connected = app_wireguard_is_connected();
    char *conf_buf = (char *)calloc(1, 768);
    if (conf_buf && strlen(g_cfg.wg.private_key) > 0 && strlen(g_cfg.wg.public_key) > 0) {
        snprintf(conf_buf, 768,
            "[Interface]\n"
            "PrivateKey = %s\n"
            "Address = %s/24\n\n"
            "[Peer]\n"
            "PublicKey = %s\n"
            "%s%s%s"
            "AllowedIPs = 10.212.156.0/24\n"
            "Endpoint = %s:%u\n"
            "PersistentKeepalive = %u\n",
            g_cfg.wg.private_key,
            g_cfg.wg.address,
            g_cfg.wg.public_key,
            strlen(g_cfg.wg.preshared_key) > 0 ? "PresharedKey = " : "",
            strlen(g_cfg.wg.preshared_key) > 0 ? g_cfg.wg.preshared_key : "",
            strlen(g_cfg.wg.preshared_key) > 0 ? "\n" : "",
            g_cfg.wg.endpoint_host,
            g_cfg.wg.endpoint_port > 0 ? g_cfg.wg.endpoint_port : 51820,
            g_cfg.wg.persistent_keepalive > 0 ? g_cfg.wg.persistent_keepalive : 25
        );
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "ok");
    cJSON_AddBoolToObject(root, "enabled", g_cfg.wg.enabled);
    cJSON_AddBoolToObject(root, "connected", connected);
    cJSON_AddStringToObject(root, "address", g_cfg.wg.address);
    cJSON_AddStringToObject(root, "netmask", g_cfg.wg.netmask);
    cJSON_AddStringToObject(root, "endpoint", g_cfg.wg.endpoint_host);
    cJSON_AddNumberToObject(root, "port", g_cfg.wg.endpoint_port);
    cJSON_AddStringToObject(root, "conf", conf_buf ? conf_buf : "");
    if (conf_buf) free(conf_buf);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    esp_err_t ret = httpd_resp_send(req, out, strlen(out));
    free(out);
    return ret;
}

esp_err_t wg_post_handler(httpd_req_t *req) {
    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 2048) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid body");
        return ESP_FAIL;
    }
    char *buf = (char *)calloc(1, total_len + 1);
    if (!buf) return ESP_FAIL;
    int ret = httpd_req_recv(req, buf, total_len);
    if (ret <= 0) {
        free(buf);
        return ESP_FAIL;
    }
    cJSON *json = cJSON_Parse(buf);
    free(buf);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
        return ESP_FAIL;
    }

    cJSON *conf_item = cJSON_GetObjectItem(json, "conf");
    cJSON *en_item = cJSON_GetObjectItem(json, "enabled");

    if (conf_item && cJSON_IsString(conf_item) && strlen(conf_item->valuestring) > 10) {
        app_wg_config_t parsed = {0};
        if (app_wireguard_parse_conf(conf_item->valuestring, &parsed) == ESP_OK) {
            g_cfg.wg = parsed;
        } else {
            cJSON_Delete(json);
            httpd_resp_set_type(req, "application/json");
            return httpd_resp_send(req, "{\"status\":\"error\",\"msg\":\"File .conf thiếu PrivateKey, PublicKey hoặc Address\"}", -1);
        }
    }
    if (en_item) {
        if (cJSON_IsBool(en_item)) g_cfg.wg.enabled = cJSON_IsTrue(en_item);
        else if (cJSON_IsNumber(en_item)) g_cfg.wg.enabled = (en_item->valueint != 0);
    }
    cJSON_Delete(json);

    app_config_save();
    app_wireguard_init(&g_cfg.wg);
    if (g_cfg.wg.enabled) {
        app_wireguard_start();
    } else {
        app_wireguard_stop();
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, "{\"status\":\"ok\"}", -1);
}
