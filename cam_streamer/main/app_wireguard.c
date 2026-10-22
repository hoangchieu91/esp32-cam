#include "app_wireguard.h"
#include "esp_wireguard.h"
#include "lwip/netif.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <time.h>
#include <sys/time.h>

static const char *TAG = "app_wireguard";

// Camera-Optimized WireGuard Tunnel MTU (1360 bytes -> TCP MSS 1320 bytes)
// Prevents UDP packet fragmentation & MJPEG/JPEG stall over 4G/PPPoE/NAT links
#define WG_CAMERA_OPTIMAL_MTU 1360

static app_wg_config_t s_wg_config = {0};
static wireguard_config_t s_esp_wg_config = {0};
static wireguard_ctx_t s_wg_ctx = {0};
static bool s_is_inited = false;
static bool s_is_started = false;
static bool s_force_reconnect = false;
static uint32_t s_wg_reconnect_cnt = 0;
static TaskHandle_t s_wg_task_handle = NULL;

static void trim_str(char *str) {
    if (!str) return;
    char *p = str;
    while (isspace((unsigned char)*p)) p++;
    if (p != str) memmove(str, p, strlen(p) + 1);

    size_t len = strlen(str);
    while (len > 0 && isspace((unsigned char)str[len - 1])) {
        str[len - 1] = '\0';
        len--;
    }
}

static void cidr_to_netmask(int prefix, char *out_mask, size_t mask_size) {
    if (prefix < 0) prefix = 0;
    if (prefix > 32) prefix = 32;
    uint32_t mask = (prefix == 0) ? 0 : (~0U << (32 - prefix));
    snprintf(out_mask, mask_size, "%u.%u.%u.%u",
             (unsigned int)((mask >> 24) & 0xFF),
             (unsigned int)((mask >> 16) & 0xFF),
             (unsigned int)((mask >> 8) & 0xFF),
             (unsigned int)(mask & 0xFF));
}

esp_err_t app_wireguard_parse_conf(const char *conf_str, app_wg_config_t *out_cfg) {
    if (!conf_str || !out_cfg) return ESP_ERR_INVALID_ARG;
    memset(out_cfg, 0, sizeof(app_wg_config_t));
    out_cfg->enabled = true;
    out_cfg->persistent_keepalive = 25;
    out_cfg->endpoint_port = 51820;

    char line[256];
    const char *p = conf_str;

    while (*p) {
        size_t idx = 0;
        while (*p && *p != '\r' && *p != '\n' && idx < (sizeof(line) - 1)) {
            line[idx++] = *p++;
        }
        line[idx] = '\0';
        while (*p == '\r' || *p == '\n') p++;

        trim_str(line);
        if (line[0] == '\0' || line[0] == '#' || line[0] == ';') continue;

        char *eq = strchr(line, '=');
        if (!eq) continue;

        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        trim_str(key);
        trim_str(val);

        if (strcasecmp(key, "PrivateKey") == 0) {
            snprintf(out_cfg->private_key, sizeof(out_cfg->private_key), "%s", val);
        } else if (strcasecmp(key, "PublicKey") == 0) {
            snprintf(out_cfg->public_key, sizeof(out_cfg->public_key), "%s", val);
        } else if (strcasecmp(key, "PresharedKey") == 0) {
            snprintf(out_cfg->preshared_key, sizeof(out_cfg->preshared_key), "%s", val);
        } else if (strcasecmp(key, "Address") == 0) {
            // Support comma-separated addresses (take first IPv4)
            char *comma = strchr(val, ',');
            if (comma) *comma = '\0';
            trim_str(val);
            char *slash = strchr(val, '/');
            if (slash) {
                *slash = '\0';
                int prefix = atoi(slash + 1);
                snprintf(out_cfg->address, sizeof(out_cfg->address), "%s", val);
                cidr_to_netmask(prefix, out_cfg->netmask, sizeof(out_cfg->netmask));
            } else {
                snprintf(out_cfg->address, sizeof(out_cfg->address), "%s", val);
                snprintf(out_cfg->netmask, sizeof(out_cfg->netmask), "255.255.255.0");
            }
        } else if (strcasecmp(key, "Endpoint") == 0) {
            char *colon = strrchr(val, ':');
            if (colon) {
                *colon = '\0';
                out_cfg->endpoint_port = (uint16_t)atoi(colon + 1);
                snprintf(out_cfg->endpoint_host, sizeof(out_cfg->endpoint_host), "%s", val);
            } else {
                snprintf(out_cfg->endpoint_host, sizeof(out_cfg->endpoint_host), "%s", val);
            }
        } else if (strcasecmp(key, "PersistentKeepalive") == 0) {
            out_cfg->persistent_keepalive = (uint16_t)atoi(val);
        }
    }

    if (strlen(out_cfg->netmask) == 0) {
        snprintf(out_cfg->netmask, sizeof(out_cfg->netmask), "255.255.255.0");
    }

    return (strlen(out_cfg->private_key) > 0 && strlen(out_cfg->public_key) > 0 && strlen(out_cfg->address) > 0)
           ? ESP_OK : ESP_FAIL;
}

esp_err_t app_wireguard_init(const app_wg_config_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    memcpy(&s_wg_config, cfg, sizeof(app_wg_config_t));

    if (!s_wg_config.enabled || strlen(s_wg_config.private_key) == 0 ||
        strlen(s_wg_config.public_key) == 0 || strlen(s_wg_config.address) == 0) {
        ESP_LOGI(TAG, "WireGuard VPN disabled or key incomplete.");
        s_is_inited = false;
        return ESP_OK;
    }

    memset(&s_esp_wg_config, 0, sizeof(wireguard_config_t));
    s_esp_wg_config.private_key = s_wg_config.private_key;
    s_esp_wg_config.public_key = s_wg_config.public_key;
    if (strlen(s_wg_config.preshared_key) > 0) {
        s_esp_wg_config.preshared_key = s_wg_config.preshared_key;
    }
    s_esp_wg_config.allowed_ip = s_wg_config.address;
    s_esp_wg_config.allowed_ip_mask = s_wg_config.netmask[0] ? s_wg_config.netmask : "255.255.255.0";
    s_esp_wg_config.endpoint = s_wg_config.endpoint_host;
    s_esp_wg_config.port = s_wg_config.endpoint_port > 0 ? s_wg_config.endpoint_port : 51820;
    s_esp_wg_config.persistent_keepalive = s_wg_config.persistent_keepalive > 0 ? s_wg_config.persistent_keepalive : 25;

    ESP_LOGI(TAG, "WireGuard Configured: Tunnel IP=%s/%s -> Peer=%s:%d (MTU=%d)",
             s_wg_config.address, s_esp_wg_config.allowed_ip_mask,
             s_wg_config.endpoint_host, s_esp_wg_config.port, WG_CAMERA_OPTIMAL_MTU);

    s_is_inited = true;
    return ESP_OK;
}

void app_wireguard_notify_wifi_roamed(void) {
    if (s_is_started) {
        s_force_reconnect = true;
    }
}

static void wireguard_connect_task(void *arg) {
    ESP_LOGI(TAG, "WireGuard VPN Supervisor started on Core %d...", xPortGetCoreID());
    vTaskDelay(pdMS_TO_TICKS(1500));

    // Wait up to 12 seconds for SNTP / Web UI time sync (WireGuard TAI64N anti-replay requirement)
    time_t initial_time = time(NULL);
    if (initial_time < 1704067200) {
        ESP_LOGI(TAG, "Waiting up to 12s for NTP/Web time sync before TAI64N handshake...");
        for (int i = 0; i < 24 && s_is_started; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
            initial_time = time(NULL);
            if (initial_time >= 1704067200) {
                ESP_LOGI(TAG, "Time synced (epoch=%lld) -> Ready for WireGuard handshake!", (long long)initial_time);
                break;
            }
        }
    }

    uint32_t down_sec = 0;

    while (true) {
        if (!s_is_started || !s_is_inited) {
            if (s_wg_ctx.netif != NULL) {
                ESP_LOGI(TAG, "WireGuard tunnel disabled -> disconnecting netif...");
                esp_wireguard_disconnect(&s_wg_ctx);
                s_wg_ctx.netif = NULL;
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (s_force_reconnect && s_wg_ctx.netif != NULL) {
            ESP_LOGW(TAG, "WiFi roamed or config updated -> refreshing WireGuard UDP socket & DNS...");
            s_force_reconnect = false;
            esp_wireguard_disconnect(&s_wg_ctx);
            s_wg_ctx.netif = NULL;
            s_wg_reconnect_cnt++;
            down_sec = 0;
            vTaskDelay(pdMS_TO_TICKS(800));
        }

        if (s_wg_ctx.netif == NULL) {
            s_force_reconnect = false;
            esp_err_t init_err = esp_wireguard_init(&s_esp_wg_config, &s_wg_ctx);
            if (init_err == ESP_OK) {
                esp_err_t conn_err = esp_wireguard_connect(&s_wg_ctx);
                if (conn_err == ESP_OK && s_wg_ctx.netif != NULL) {
                    // CRITICAL CAMERA OPTIMIZATION: Clamp WireGuard netif MTU to 1360 bytes
                    // so lwIP advertises TCP MSS = 1320 bytes on MJPEG/HTTP/FTP/OTA streams
                    struct netif *lwip_wg = (struct netif *)s_wg_ctx.netif;
                    lwip_wg->mtu = WG_CAMERA_OPTIMAL_MTU;
                    ESP_LOGI(TAG, "WireGuard interface up (IP=%s, MTU=%u). Handshake with %s:%d in progress...",
                             s_wg_config.address, (unsigned)lwip_wg->mtu,
                             s_wg_config.endpoint_host, s_esp_wg_config.port);
                    down_sec = 0;
                } else {
                    ESP_LOGW(TAG, "esp_wireguard_connect failed (%s), retrying in 10s...", esp_err_to_name(conn_err));
                    esp_wireguard_disconnect(&s_wg_ctx);
                    s_wg_ctx.netif = NULL;
                    for (int i = 0; i < 20 && s_is_started && !s_force_reconnect; i++) vTaskDelay(pdMS_TO_TICKS(500));
                    continue;
                }
            } else {
                ESP_LOGW(TAG, "esp_wireguard_init failed (%s), retrying in 10s...", esp_err_to_name(init_err));
                for (int i = 0; i < 20 && s_is_started && !s_force_reconnect; i++) vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }
        } else {
            // Monitor Peer Handshake State
            if (esp_wireguardif_peer_is_up(&s_wg_ctx) != ESP_OK) {
                down_sec += 5;
                time_t cur_time = time(NULL);
                // 1. If clock was 1970 during initial handshake and is now synced, reconnect immediately
                if (cur_time >= 1704067200 && initial_time < 1704067200) {
                    ESP_LOGI(TAG, "System clock synced (epoch=%lld) -> Re-initiating WireGuard TAI64N handshake...", (long long)cur_time);
                    esp_wireguard_disconnect(&s_wg_ctx);
                    s_wg_ctx.netif = NULL;
                    initial_time = cur_time;
                    s_wg_reconnect_cnt++;
                    down_sec = 0;
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    continue;
                }
                // 2. Industrial Self-Healing: If peer handshake fails for > 60s, re-resolve DDNS endpoint & re-bind UDP
                if (down_sec >= 60) {
                    ESP_LOGW(TAG, "WireGuard peer handshake down for %us -> Re-resolving DDNS '%s' & reconnecting...",
                             (unsigned)down_sec, s_wg_config.endpoint_host);
                    esp_wireguard_disconnect(&s_wg_ctx);
                    s_wg_ctx.netif = NULL;
                    s_wg_reconnect_cnt++;
                    down_sec = 0;
                    vTaskDelay(pdMS_TO_TICKS(1500));
                    continue;
                }
            } else {
                down_sec = 0;
            }
        }
        for (int i = 0; i < 10 && s_is_started && !s_force_reconnect; i++) {
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

esp_err_t app_wireguard_start(void) {
    if (!s_is_inited) {
        return ESP_ERR_INVALID_STATE;
    }
    s_is_started = true;
    s_force_reconnect = true;
    if (s_wg_task_handle == NULL) {
        // Pin to Core 0 (Network/WiFi core) with 6144B stack so Core 1 remains 100% dedicated to Camera & Edge AI
        BaseType_t ret = xTaskCreatePinnedToCore(wireguard_connect_task, "wg_vpn", 6144, NULL, 4, &s_wg_task_handle, 0);
        if (ret != pdPASS) {
            s_is_started = false;
            s_wg_task_handle = NULL;
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

esp_err_t app_wireguard_stop(void) {
    if (!s_is_started) return ESP_OK;
    ESP_LOGI(TAG, "Stopping WireGuard VPN tunnel...");
    s_is_started = false;
    s_force_reconnect = true;
    return ESP_OK;
}

bool app_wireguard_is_connected(void) {
    if (!s_is_started || s_wg_ctx.netif == NULL) return false;
    return (esp_wireguardif_peer_is_up(&s_wg_ctx) == ESP_OK);
}

uint32_t app_wireguard_get_reconnect_cnt(void) {
    return s_wg_reconnect_cnt;
}

uint16_t app_wireguard_get_mtu(void) {
    if (s_wg_ctx.netif != NULL) {
        struct netif *lwip_wg = (struct netif *)s_wg_ctx.netif;
        return lwip_wg->mtu;
    }
    return WG_CAMERA_OPTIMAL_MTU;
}

esp_err_t app_wireguard_get_config(app_wg_config_t *cfg) {
    if (!cfg) return ESP_ERR_INVALID_ARG;
    memcpy(cfg, &s_wg_config, sizeof(app_wg_config_t));
    return ESP_OK;
}
