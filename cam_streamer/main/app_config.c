#include "app_config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_partition.h"
#include "esp_log.h"
#include <string.h>
#include <stdio.h>
#include <sys/stat.h>

static const char *TAG = "app_config";
#define NVS_NAMESPACE     "cam_cfg"
#define NVS_BLOB_KEY      "cfg_v9"
#define NVS_WG_VAULT_NS   "wg_vault"
#define SD_WG_VAULT_PATH  "/sdcard/wg_vault.conf"

// Dedicated Raw Flash Partition @ 0x3FC000 (16KB, SubType 0x99)
// Dual-Sector A/B (Sector 0 @ 0x0000 = Primary, Sector 1 @ 0x1000 = Mirror)
#define VPN_VAULT_SUBTYPE 0x99
#define VPN_VAULT_MAGIC   0x57475632UL // "WGV2"
#define VPN_VAULT_VERSION 1UL

typedef struct {
    uint32_t magic;
    uint32_t version;
    app_wg_config_t wg;
    wifi_profile_t wifi_list[MAX_WIFI_PROFILES];
    char mdns_host[32];
    char device_name[32];
    uint32_t checksum;
} vpn_vault_flash_t;

app_config_t g_cfg;
static char s_vault_status_str[64] = "Flash vpn_vault (0x3FC000)";
static bool s_loaded_from_flash_vault = false;

static uint32_t calc_vault_checksum(const vpn_vault_flash_t *v) {
    const uint8_t *p = (const uint8_t *)v;
    size_t len = offsetof(vpn_vault_flash_t, checksum);
    uint32_t hash = 2166136261UL; // FNV-1a 32-bit
    for (size_t i = 0; i < len; i++) {
        hash ^= p[i];
        hash *= 16777619UL;
    }
    return hash;
}

static const esp_partition_t *get_vpn_vault_partition(void) {
    const esp_partition_t *part = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)VPN_VAULT_SUBTYPE, "vpn_vault");
    return part;
}

static void write_sd_wg_vault_file(const app_wg_config_t *wg) {
    struct stat st;
    if (stat("/sdcard", &st) != 0) return; // SD not mounted yet
    if (strlen(wg->private_key) == 0 || strlen(wg->public_key) == 0) return;

    FILE *f = fopen(SD_WG_VAULT_PATH, "w");
    if (!f) return;
    fprintf(f,
        "# ESP32-CAM Indestructible WireGuard Vault Backup\n"
        "# Enabled = %d\n"
        "[Interface]\n"
        "PrivateKey = %s\n"
        "Address = %s/24\n\n"
        "[Peer]\n"
        "PublicKey = %s\n",
        wg->enabled ? 1 : 0,
        wg->private_key,
        wg->address,
        wg->public_key);
    if (strlen(wg->preshared_key) > 0) {
        fprintf(f, "PresharedKey = %s\n", wg->preshared_key);
    }
    fprintf(f,
        "AllowedIPs = 10.212.156.0/24\n"
        "Endpoint = %s:%u\n"
        "PersistentKeepalive = %u\n",
        wg->endpoint_host,
        wg->endpoint_port > 0 ? wg->endpoint_port : 51820,
        wg->persistent_keepalive > 0 ? wg->persistent_keepalive : 25);
    fclose(f);
    ESP_LOGI(TAG, "Synced WireGuard Vault backup to %s", SD_WG_VAULT_PATH);
}

static bool read_sd_wg_vault_file(app_wg_config_t *out_wg) {
    FILE *f = fopen(SD_WG_VAULT_PATH, "r");
    if (!f) return false;
    char *buf = (char *)calloc(1, 1024);
    if (!buf) {
        fclose(f);
        return false;
    }
    size_t n = fread(buf, 1, 1023, f);
    fclose(f);
    if (n < 20) {
        free(buf);
        return false;
    }

    app_wg_config_t *parsed = (app_wg_config_t *)calloc(1, sizeof(app_wg_config_t));
    if (!parsed) {
        free(buf);
        return false;
    }
    bool ok = false;
    if (app_wireguard_parse_conf(buf, parsed) == ESP_OK) {
        if (strstr(buf, "# Enabled = 0") != NULL) {
            parsed->enabled = false;
        } else {
            parsed->enabled = true;
        }
        *out_wg = *parsed;
        ok = true;
    }
    free(parsed);
    free(buf);
    return ok;
}

static void save_nvs_wg_vault(const app_wg_config_t *wg) {
    nvs_handle_t h;
    if (nvs_open(NVS_WG_VAULT_NS, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u32(h, "magic", VPN_VAULT_MAGIC);
    nvs_set_u8(h, "en", wg->enabled ? 1 : 0);
    nvs_set_str(h, "priv", wg->private_key);
    nvs_set_str(h, "pub", wg->public_key);
    nvs_set_str(h, "psk", wg->preshared_key);
    nvs_set_str(h, "addr", wg->address);
    nvs_set_str(h, "mask", wg->netmask);
    nvs_set_str(h, "host", wg->endpoint_host);
    nvs_set_u16(h, "port", wg->endpoint_port);
    nvs_set_u16(h, "ka", wg->persistent_keepalive);
    nvs_commit(h);
    nvs_close(h);
}

static bool load_nvs_wg_vault(app_wg_config_t *wg) {
    nvs_handle_t h;
    if (nvs_open(NVS_WG_VAULT_NS, NVS_READONLY, &h) != ESP_OK) return false;
    uint32_t magic = 0;
    if (nvs_get_u32(h, "magic", &magic) != ESP_OK || magic != VPN_VAULT_MAGIC) {
        nvs_close(h);
        return false;
    }
    uint8_t en = 1;
    nvs_get_u8(h, "en", &en);
    wg->enabled = (en != 0);
    size_t sz;
    sz = sizeof(wg->private_key); nvs_get_str(h, "priv", wg->private_key, &sz);
    sz = sizeof(wg->public_key);  nvs_get_str(h, "pub",  wg->public_key,  &sz);
    sz = sizeof(wg->preshared_key); nvs_get_str(h, "psk", wg->preshared_key, &sz);
    sz = sizeof(wg->address);     nvs_get_str(h, "addr", wg->address,     &sz);
    sz = sizeof(wg->netmask);     nvs_get_str(h, "mask", wg->netmask,     &sz);
    sz = sizeof(wg->endpoint_host); nvs_get_str(h, "host", wg->endpoint_host, &sz);
    nvs_get_u16(h, "port", &wg->endpoint_port);
    nvs_get_u16(h, "ka",   &wg->persistent_keepalive);
    nvs_close(h);
    return (strlen(wg->private_key) > 0 && strlen(wg->public_key) > 0);
}

esp_err_t app_vault_save_network(void) {
    // 1. Save to Dedicated Key-Value NVS Namespace ("wg_vault")
    save_nvs_wg_vault(&g_cfg.wg);

    // 2. Save to Dedicated Raw Flash Partition "vpn_vault" @ 0x3FC000 (Dual-Sector A/B)
    const esp_partition_t *part = get_vpn_vault_partition();
    if (part && part->size >= 0x2000) {
        vpn_vault_flash_t *v = (vpn_vault_flash_t *)calloc(1, sizeof(vpn_vault_flash_t));
        if (v) {
            v->magic = VPN_VAULT_MAGIC;
            v->version = VPN_VAULT_VERSION;
            v->wg = g_cfg.wg;
            memcpy(v->wifi_list, g_cfg.wifi_list, sizeof(v->wifi_list));
            strncpy(v->mdns_host, g_cfg.mdns_host, sizeof(v->mdns_host) - 1);
            strncpy(v->device_name, g_cfg.device_name, sizeof(v->device_name) - 1);
            v->checksum = calc_vault_checksum(v);

            // Write Sector 1 (Mirror @ 0x1000) first, then Sector 0 (Primary @ 0x0000)
            if (esp_partition_erase_range(part, 0x1000, 0x1000) == ESP_OK) {
                esp_partition_write(part, 0x1000, v, sizeof(vpn_vault_flash_t));
            }
            if (esp_partition_erase_range(part, 0x0000, 0x1000) == ESP_OK) {
                esp_partition_write(part, 0x0000, v, sizeof(vpn_vault_flash_t));
            }
            free(v);
            s_loaded_from_flash_vault = true;
            snprintf(s_vault_status_str, sizeof(s_vault_status_str), "vpn_vault @ 0x3FC000 (A/B) + NVS + SD");
            ESP_LOGI(TAG, "Saved WireGuard & 5-WiFi to dedicated partition vpn_vault (0x3FC000 A/B)");
        }
    }

    // 3. Mirror to MicroSD Card (/sdcard/wg_vault.conf) if mounted
    write_sd_wg_vault_file(&g_cfg.wg);
    return ESP_OK;
}

static bool load_flash_vpn_vault(void) {
    const esp_partition_t *part = get_vpn_vault_partition();
    if (!part || part->size < 0x2000) return false;

    vpn_vault_flash_t *v = (vpn_vault_flash_t *)calloc(1, sizeof(vpn_vault_flash_t));
    if (!v) return false;

    // Try Sector 0 (Primary @ 0x0000)
    if (esp_partition_read(part, 0x0000, v, sizeof(vpn_vault_flash_t)) == ESP_OK) {
        if (v->magic == VPN_VAULT_MAGIC && v->checksum == calc_vault_checksum(v) &&
            strlen(v->wg.private_key) > 0 && strlen(v->wg.public_key) > 0) {
            g_cfg.wg = v->wg;
            memcpy(g_cfg.wifi_list, v->wifi_list, sizeof(g_cfg.wifi_list));
            if (v->mdns_host[0]) strncpy(g_cfg.mdns_host, v->mdns_host, sizeof(g_cfg.mdns_host) - 1);
            if (v->device_name[0]) strncpy(g_cfg.device_name, v->device_name, sizeof(g_cfg.device_name) - 1);
            free(v);
            snprintf(s_vault_status_str, sizeof(s_vault_status_str), "vpn_vault @ 0x3FC000 (Primary OK)");
            ESP_LOGI(TAG, "Restored WireGuard (%s) & 5-WiFi from dedicated partition vpn_vault @ 0x3FC000!",
                     g_cfg.wg.address);
            return true;
        }
    }

    // Fallback to Sector 1 (Mirror @ 0x1000) if power was lost during Primary sector write
    if (esp_partition_read(part, 0x1000, v, sizeof(vpn_vault_flash_t)) == ESP_OK) {
        if (v->magic == VPN_VAULT_MAGIC && v->checksum == calc_vault_checksum(v) &&
            strlen(v->wg.private_key) > 0 && strlen(v->wg.public_key) > 0) {
            g_cfg.wg = v->wg;
            memcpy(g_cfg.wifi_list, v->wifi_list, sizeof(g_cfg.wifi_list));
            if (v->mdns_host[0]) strncpy(g_cfg.mdns_host, v->mdns_host, sizeof(g_cfg.mdns_host) - 1);
            if (v->device_name[0]) strncpy(g_cfg.device_name, v->device_name, sizeof(g_cfg.device_name) - 1);
            // Heal Sector 0
            if (esp_partition_erase_range(part, 0x0000, 0x1000) == ESP_OK) {
                esp_partition_write(part, 0x0000, v, sizeof(vpn_vault_flash_t));
            }
            free(v);
            snprintf(s_vault_status_str, sizeof(s_vault_status_str), "vpn_vault @ 0x3FC000 (Mirror Healed)");
            ESP_LOGW(TAG, "Recovered WireGuard & 5-WiFi from vpn_vault Mirror Sector 1 and healed Sector 0!");
            return true;
        }
    }
    free(v);
    return false;
}

esp_err_t app_vault_sync_sdcard(void) {
    if (!s_loaded_from_flash_vault) {
        // If Flash vpn_vault was empty (e.g. after full chip erase_flash), check SD card backup!
        app_wg_config_t *sd_wg = (app_wg_config_t *)calloc(1, sizeof(app_wg_config_t));
        if (sd_wg) {
            if (read_sd_wg_vault_file(sd_wg)) {
                g_cfg.wg = *sd_wg;
                free(sd_wg);
                app_vault_save_network();
                snprintf(s_vault_status_str, sizeof(s_vault_status_str), "Recovered from SD wg_vault.conf");
                ESP_LOGW(TAG, "Recovered WireGuard config (%s) from MicroSD %s after Flash erase!",
                         g_cfg.wg.address, SD_WG_VAULT_PATH);
                return ESP_OK;
            }
            free(sd_wg);
        }
    }
    // Ensure SD card always has the latest backup copy
    write_sd_wg_vault_file(&g_cfg.wg);
    return ESP_OK;
}

const char *app_vault_get_status_str(void) {
    return s_vault_status_str;
}

void app_config_reset_defaults(void) {
    memset(&g_cfg, 0, sizeof(app_config_t));

    strncpy(g_cfg.mdns_host, "esp32-cam", sizeof(g_cfg.mdns_host) - 1);
    strncpy(g_cfg.device_name, "ESP32-CAM Smart AI", sizeof(g_cfg.device_name) - 1);

    // Default Slot 0: HA / 123123123
    strncpy(g_cfg.wifi_list[0].ssid, "HA", 32);
    strncpy(g_cfg.wifi_list[0].password, "123123123", 64);
    g_cfg.wifi_list[0].enabled = 1;

    for (int i = 1; i < MAX_WIFI_PROFILES; i++) {
        g_cfg.wifi_list[i].ssid[0] = '\0';
        g_cfg.wifi_list[i].password[0] = '\0';
        g_cfg.wifi_list[i].enabled = 0;
    }

    strncpy(g_cfg.ap_ssid, "ESP32-CAM-Config", sizeof(g_cfg.ap_ssid) - 1);
    strncpy(g_cfg.ap_pass, "12345678", sizeof(g_cfg.ap_pass) - 1);
    g_cfg.ap_always_on = false;

    strncpy(g_cfg.ntp_server, "pool.ntp.org", sizeof(g_cfg.ntp_server) - 1);
    strncpy(g_cfg.tz_str, "ICT-7", sizeof(g_cfg.tz_str) - 1);
    g_cfg.osd_enable = true;

    // Default FTP -> 10.25.7.171
    g_cfg.ftp_enable = true;
    strncpy(g_cfg.ftp_host, "10.25.7.171", sizeof(g_cfg.ftp_host) - 1);
    g_cfg.ftp_port = 21;
    strncpy(g_cfg.ftp_user, "nxchieu", sizeof(g_cfg.ftp_user) - 1);
    strncpy(g_cfg.ftp_pass, "123", sizeof(g_cfg.ftp_pass) - 1);
    strncpy(g_cfg.ftp_remote_dir, "/var/ftp/esp32cam", sizeof(g_cfg.ftp_remote_dir) - 1);

    // Default MQTT -> 10.25.7.171
    g_cfg.mqtt_enable = true;
    strncpy(g_cfg.mqtt_uri, "mqtt://10.25.7.171:1883", sizeof(g_cfg.mqtt_uri) - 1);
    g_cfg.mqtt_user[0] = '\0';
    g_cfg.mqtt_pass[0] = '\0';
    strncpy(g_cfg.mqtt_topic_prefix, "esp32cam", sizeof(g_cfg.mqtt_topic_prefix) - 1);
    g_cfg.mqtt_interval_sec = 5;

    // Default Ntfy -> OFF by default to protect ntfy.sh 250/day upstream quota on 10.25.7.116:8092
    g_cfg.ntfy_enable = false;
    strncpy(g_cfg.ntfy_url, "http://10.25.7.116:8092/esp32cam", sizeof(g_cfg.ntfy_url) - 1);
    g_cfg.ntfy_token[0] = '\0';

    // Storage & Flexible/Adaptive Capture Cycle
    g_cfg.sd_save_enable = true;
    g_cfg.auto_capture_sec = 300;
    g_cfg.adaptive_cycle_enable = true;
    g_cfg.fast_capture_sec = 30;

    // Edge AI & Server AI
    g_cfg.edge_ai_enable = true;
    g_cfg.edge_motion_thresh = 20;
    g_cfg.edge_trigger_capture = false;
    strncpy(g_cfg.ai_server_url, "http://10.25.7.171:8095/api/analyze", sizeof(g_cfg.ai_server_url) - 1);

    // Smart Meter & Indicator ROI defaults (Calibrated for Ningbo GB/T 778-2018 Rotary Water Meter)
    g_cfg.meter_mode = 4;
    g_cfg.roi_x_pct = 8;
    g_cfg.roi_y_pct = 3;
    g_cfg.roi_w_pct = 84;
    g_cfg.roi_h_pct = 94;
    g_cfg.roi_draw_osd = true;
    strncpy(g_cfg.meter_baseline, "00304.954", sizeof(g_cfg.meter_baseline) - 1);
    g_cfg.meter_max_flow_lpm = 50; // 50 L/min = 3.0 m3/h max residential water meter flow
    g_cfg.meter_rot_deg = -90;     // -90 deg CCW upright deskew
    strncpy(g_cfg.meter_serial, "00000198", sizeof(g_cfg.meter_serial) - 1);
    strncpy(g_cfg.meter_spec, "GB/T778-2018 R=80", sizeof(g_cfg.meter_spec) - 1);
    g_cfg.serial_auto_detect = true;

    // Camera & Auxiliary Flash LED (GPIO 4 PWM) defaults
    g_cfg.cam_framesize = 10;
    g_cfg.cam_quality = 10;
    g_cfg.cam_brightness = -1;
    g_cfg.cam_contrast = 2;
    g_cfg.cam_saturation = 2;
    g_cfg.cam_hmirror = 0;
    g_cfg.cam_vflip = 1;
    g_cfg.af_mode = 1;
    g_cfg.af_manual_pos = 620;
    g_cfg.flash_duty = 0;
    g_cfg.ntfy_enable = false; // Force disable continuous Ntfy spam on boot to protect 250/day ntfy.sh quota        // Always OFF when idle
    g_cfg.flash_mode = 1;        // 1 = Smart Auto Fill-Light ONLY when ambient scene is dark
    g_cfg.flash_cap_duty = 28;   // ~11% PWM anti-glare soft illumination for water meter glass
    g_cfg.flash_warmup_ms = 350; // 350ms AE/AWB settle time before snapping
    g_cfg.eco_mode = true;

    // Industrial Camera ISP & Storage Ring-Buffer defaults
    g_cfg.cam_isp_profile = 4;
    g_cfg.cam_wb_mode = 1;
    g_cfg.cam_sharpness = 2;
    g_cfg.cam_denoise = 1;
    g_cfg.cam_gainceiling = 1;
    g_cfg.cam_aec2 = false;
    g_cfg.sd_auto_prune = true;
    g_cfg.sd_min_free_mb = 200;

    // Tier-4 Firmware Built-in WireGuard Default Profile (10.212.156.7 -> nxchieu.duckdns.org:51820)
    g_cfg.wg.enabled = true;
    strncpy(g_cfg.wg.private_key, "ACdNoeuQNB3j1zLwx89w5FL2J49F9asnpS1KrgJw3lk=", sizeof(g_cfg.wg.private_key) - 1);
    strncpy(g_cfg.wg.address, "10.212.156.7", sizeof(g_cfg.wg.address) - 1);
    strncpy(g_cfg.wg.netmask, "255.255.255.0", sizeof(g_cfg.wg.netmask) - 1);
    strncpy(g_cfg.wg.public_key, "3+AQYQEPXumCI97lO5hVWT80Vj++e6GBaMw05H09GEQ=", sizeof(g_cfg.wg.public_key) - 1);
    strncpy(g_cfg.wg.preshared_key, "lIEp7kq2nXgT6nCqiM+3EslDj6TM7eL6dTebTwlo8bA=", sizeof(g_cfg.wg.preshared_key) - 1);
    strncpy(g_cfg.wg.endpoint_host, "nxchieu.duckdns.org", sizeof(g_cfg.wg.endpoint_host) - 1);
    g_cfg.wg.endpoint_port = 51820;
    g_cfg.wg.persistent_keepalive = 25;

    // Baby Monitor & Govee BLE Room Environment Gateway defaults
    g_cfg.baby_mon_enable = true;
    g_cfg.baby_lock_flash = false; // Default false on Water Meter camera so Smart Auto Flash (11% PWM) works at night
    strncpy(g_cfg.baby_server_url, "http://10.25.7.116:8686/api/room-env", sizeof(g_cfg.baby_server_url) - 1);
    strncpy(g_cfg.baby_sensor_name, "Govee Phòng Bé Bon", sizeof(g_cfg.baby_sensor_name) - 1);
    strncpy(g_cfg.baby_source_device, "ESP32-CAM", sizeof(g_cfg.baby_source_device) - 1);
    g_cfg.baby_target_mac[0] = '\0';
}

void app_config_factory_reset_preserve_vpn(void) {
    // Preserve current WireGuard & 5-WiFi profiles while resetting Camera/AI/Cloud settings
    app_config_reset_defaults();
    load_flash_vpn_vault();
    app_config_save();
}

void app_config_init(void) {
    app_config_reset_defaults();

    // 1. Load general app settings from standard NVS namespace ("cam_cfg")
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t required_size = sizeof(app_config_t);
        err = nvs_get_blob(handle, NVS_BLOB_KEY, &g_cfg, &required_size);
        if (err != ESP_OK || required_size != sizeof(app_config_t)) {
            ESP_LOGW(TAG, "App config struct version changed -> resetting general settings while preserving VPN Vault & Meter Identity...");
            app_config_reset_defaults();
            size_t sz;
            sz = sizeof(g_cfg.meter_serial); nvs_get_str(handle, "m_serial", g_cfg.meter_serial, &sz);
            sz = sizeof(g_cfg.meter_spec);   nvs_get_str(handle, "m_spec",   g_cfg.meter_spec,   &sz);
            sz = sizeof(g_cfg.meter_baseline); nvs_get_str(handle, "m_base", g_cfg.meter_baseline, &sz);
            uint8_t sauto = 1;
            if (nvs_get_u8(handle, "m_sauto", &sauto) == ESP_OK) g_cfg.serial_auto_detect = (sauto != 0);
        }
        nvs_close(handle);
    }
    if (g_cfg.meter_serial[0] == '\0') {
        strncpy(g_cfg.meter_serial, "00000198", sizeof(g_cfg.meter_serial) - 1);
    }
    if (g_cfg.meter_spec[0] == '\0') {
        strncpy(g_cfg.meter_spec, "GB/T778-2018 R=80", sizeof(g_cfg.meter_spec) - 1);
    }

    // Never persist continuous manual torch across reboots (prevents 24/7 LED glare/heating)
    g_cfg.flash_duty = 0;
    g_cfg.ntfy_enable = false; // Force disable continuous Ntfy spam on boot to protect 250/day ntfy.sh quota

    // 2. Always overlay WireGuard & 5-WiFi from Dedicated Flash Partition "vpn_vault" @ 0x3FC000
    if (load_flash_vpn_vault()) {
        s_loaded_from_flash_vault = true;
        save_nvs_wg_vault(&g_cfg.wg);
    } else if (load_nvs_wg_vault(&g_cfg.wg)) {
        // Migrate from NVS wg_vault into dedicated partition vpn_vault @ 0x3FC000
        ESP_LOGI(TAG, "Migrating WireGuard config from NVS wg_vault to partition vpn_vault @ 0x3FC000...");
        app_vault_save_network();
    } else {
        // Initialize dedicated partition vpn_vault @ 0x3FC000 with built-in profile
        ESP_LOGI(TAG, "Initializing dedicated partition vpn_vault @ 0x3FC000 with default VPN profile...");
        app_vault_save_network();
        s_loaded_from_flash_vault = false; // Allow SD card wg_vault.conf override if present on first boot
    }
}

esp_err_t app_config_save(void) {
    // Always persist WireGuard & 5-WiFi to the indestructible vpn_vault partition (0x3FC000) + NVS + SD
    app_vault_save_network();

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for write: %s", esp_err_to_name(err));
        return err;
    }
    err = nvs_set_blob(handle, NVS_BLOB_KEY, &g_cfg, sizeof(app_config_t));
    if (err == ESP_ERR_NVS_NOT_ENOUGH_SPACE) {
        ESP_LOGW(TAG, "NVS namespace full from old versions -> erasing stale keys and retrying...");
        nvs_erase_all(handle);
        nvs_commit(handle);
        err = nvs_set_blob(handle, NVS_BLOB_KEY, &g_cfg, sizeof(app_config_t));
    }
    nvs_set_str(handle, "m_serial", g_cfg.meter_serial);
    nvs_set_str(handle, "m_spec",   g_cfg.meter_spec);
    nvs_set_str(handle, "m_base",   g_cfg.meter_baseline);
    nvs_set_u8(handle,  "m_sauto",  g_cfg.serial_auto_detect ? 1 : 0);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    ESP_LOGI(TAG, "Saved configuration to NVS & vpn_vault (%s)", esp_err_to_name(err));
    return err;
}
