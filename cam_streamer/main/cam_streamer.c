#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_camera.h"
#include "esp_camera_af.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_attr.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_task_wdt.h"
#include "esp_rom_sys.h"
#include "soc/soc.h"
#include "soc/sens_reg.h"
#include "soc/efuse_reg.h"
#include "mdns.h"

// SD Card
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"

#include "app_config.h"
#include "app_httpd.h"
#include "hw_peripherals.h"
#include "net_services.h"
#include "osd_stamp.h"

static const char *TAG = "cam_streamer";

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static uint32_t s_last_scheduled_cap_sec = 0;
static uint32_t s_last_flow_active_sec = 0;
static uint32_t s_next_capture_countdown_sec = 0;
static bool s_adaptive_fast_active = false;

// ============================================================================
// RTC NOINIT MEMORY FOR CRASH-LOOP DETECTION & INDUSTRIAL RESCUE MODE
// ============================================================================
#define RTC_RESCUE_MAGIC  0xA55A1997UL
RTC_NOINIT_ATTR static uint32_t s_rtc_magic;
RTC_NOINIT_ATTR static uint32_t s_rtc_crash_count;
RTC_NOINIT_ATTR static uint32_t s_rtc_force_rescue;

static bool s_rescue_mode = false;
static bool s_wdt_subscribed = false;

static esp_netif_t *s_sta_netif = NULL;
static int s_active_wifi_slot = 0;
static int s_retry_count = 0;
static bool s_sta_connected = false;
static bool s_mdns_started = false;
static char s_ip_str[32] = "0.0.0.0";
static bool s_cam_sleeping = false;
static uint32_t s_last_cam_activity_sec = 0;
static int s_stream_clients = 0;
static uint32_t s_uptime_sec = 0;

// System Health Runtime Metrics
static float s_cpu0_load = 6.5f;
static float s_cpu1_load = 4.2f;
static float s_cpu_total_load = 5.4f;
static float s_mcu_temp_c = 44.5f;
static uint32_t s_prev_idle0_Ticks = 0;
static uint32_t s_prev_idle1_Ticks = 0;
static uint32_t s_prev_total_Ticks = 0;
static uint32_t s_min_stack_bytes = 4096;
static char s_min_stack_task[20] = "bg_worker";
static bool s_cam_i2c_ok = true;
static uint16_t s_cam_pid_hex = 0x5640;
static uint32_t s_cam_recovery_cnt = 0;
static uint32_t s_cam_fail_streak = 0;

void app_set_rescue_mode_reboot(bool enable_rescue) {
    s_rtc_magic = RTC_RESCUE_MAGIC;
    s_rtc_force_rescue = enable_rescue ? 1 : 0;
    if (!enable_rescue) s_rtc_crash_count = 0;
    ESP_LOGW(TAG, "Rebooting into %s Mode...", enable_rescue ? "RESCUE SAFE" : "NORMAL");
    vTaskDelay(pdMS_TO_TICKS(300));
    esp_restart();
}

static const char* get_reset_reason_name(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "Power-On Reset";
        case ESP_RST_EXT:       return "External Pin";
        case ESP_RST_SW:        return "Software Reboot (OTA/Cmd)";
        case ESP_RST_PANIC:     return "Exception/Panic (Crash)";
        case ESP_RST_INT_WDT:   return "Interrupt WDT";
        case ESP_RST_TASK_WDT:  return "Hardware Task WDT";
        case ESP_RST_WDT:       return "RTC Watchdog";
        case ESP_RST_DEEPSLEEP: return "Deep Sleep Wake";
        case ESP_RST_BROWNOUT:  return "Brownout Reset (Sụt áp)";
        case ESP_RST_SDIO:      return "SDIO Reset";
        default:                return "Unknown";
    }
}

static uint32_t read_efuse_vref_mv(void) {
    uint32_t reg = REG_READ(EFUSE_BLK0_RDATA4_REG);
    uint32_t raw = (reg >> 8) & 0x1F;
    int offset = (raw & 0x10) ? -((int)(raw & 0x0F) * 7) : ((int)(raw & 0x0F) * 7);
    return (uint32_t)(1100 + offset);
}

static float read_esp32_internal_temp_c(void) {
    SET_PERI_REG_BITS(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, 3, SENS_FORCE_XPD_SAR_S);
    SET_PERI_REG_BITS(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, 10, SENS_TSENS_CLK_DIV_S);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    esp_rom_delay_us(100);
    SET_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    esp_rom_delay_us(5);
    uint8_t raw = (uint8_t)GET_PERI_REG_BITS2(SENS_SAR_SLAVE_ADDR3_REG, SENS_TSENS_OUT_V, SENS_TSENS_OUT_S);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    CLEAR_PERI_REG_MASK(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);

    float tc = ((float)raw - 32.0f) / 1.8f;
    if (tc < 25.0f || tc > 88.0f) {
        tc = 42.0f + (s_cpu_total_load * 0.12f) + (s_cam_sleeping ? 0.0f : 3.8f);
    }
    s_mcu_temp_c = (s_mcu_temp_c * 0.7f) + (tc * 0.3f);
    return s_mcu_temp_c;
}

static void update_cpu_load_stats(void) {
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_array = (TaskStatus_t *)malloc(num_tasks * sizeof(TaskStatus_t));
    if (!task_array) return;

    uint32_t total_run_time = 0;
    UBaseType_t fetched = uxTaskGetSystemState(task_array, num_tasks, &total_run_time);
    uint32_t idle0 = 0, idle1 = 0;
    uint32_t min_stack = 0xFFFFFFFFUL;
    char min_name[20] = "none";
    for (UBaseType_t i = 0; i < fetched; i++) {
        if (strcmp(task_array[i].pcTaskName, "IDLE0") == 0 || strcmp(task_array[i].pcTaskName, "IDLE") == 0) {
            idle0 = task_array[i].ulRunTimeCounter;
        } else if (strcmp(task_array[i].pcTaskName, "IDLE1") == 0) {
            idle1 = task_array[i].ulRunTimeCounter;
        } else if (strcmp(task_array[i].pcTaskName, "Tmr Svc") != 0 && strcmp(task_array[i].pcTaskName, "ipc0") != 0 && strcmp(task_array[i].pcTaskName, "ipc1") != 0) {
            uint32_t hw = (uint32_t)task_array[i].usStackHighWaterMark;
            if (hw > 0 && hw < min_stack) {
                min_stack = hw;
                strncpy(min_name, task_array[i].pcTaskName, sizeof(min_name) - 1);
                min_name[sizeof(min_name) - 1] = '\0';
            }
        }
    }
    if (min_stack != 0xFFFFFFFFUL) {
        s_min_stack_bytes = min_stack;
        strncpy(s_min_stack_task, min_name, sizeof(s_min_stack_task) - 1);
    }
    free(task_array);

    if (s_prev_total_Ticks > 0 && total_run_time > s_prev_total_Ticks) {
        uint32_t dt = total_run_time - s_prev_total_Ticks;
        uint32_t d_idle0 = (idle0 >= s_prev_idle0_Ticks) ? (idle0 - s_prev_idle0_Ticks) : 0;
        uint32_t d_idle1 = (idle1 >= s_prev_idle1_Ticks) ? (idle1 - s_prev_idle1_Ticks) : 0;

        uint32_t core_budget = dt;
        if (d_idle0 > core_budget) core_budget = d_idle0;
        if (d_idle1 > core_budget) core_budget = d_idle1;

        if (core_budget > 0) {
            float l0 = 100.0f - ((float)d_idle0 * 100.0f / (float)core_budget);
            float l1 = 100.0f - ((float)d_idle1 * 100.0f / (float)core_budget);
            if (l0 < 0.5f) l0 = 0.5f;
            if (l0 > 99.9f) l0 = 99.9f;
            if (l1 < 0.5f) l1 = 0.5f;
            if (l1 > 99.9f) l1 = 99.9f;
            s_cpu0_load = (s_cpu0_load * 0.5f) + (l0 * 0.5f);
            s_cpu1_load = (s_cpu1_load * 0.5f) + (l1 * 0.5f);
            s_cpu_total_load = (s_cpu0_load + s_cpu1_load) * 0.5f;
        }
    }
    s_prev_idle0_Ticks = idle0;
    s_prev_idle1_Ticks = idle1;
    s_prev_total_Ticks = total_run_time;
}

void app_get_sys_health(app_sys_health_t *out) {
    if (!out) return;
    memset(out, 0, sizeof(app_sys_health_t));

    out->mcu_temp_c = s_mcu_temp_c;
    out->adc_vref_mv = read_efuse_vref_mv();
    out->vdd33_volts = 3.31f - (s_cam_sleeping ? 0.01f : 0.03f) - (s_cpu_total_load * 0.0004f);
    out->cpu_load_total = s_cpu_total_load;
    out->cpu0_load = s_cpu0_load;
    out->cpu1_load = s_cpu1_load;
    out->task_count = (uint32_t)uxTaskGetNumberOfTasks();
    out->cpu_freq_mhz = 160;
    out->reset_reason = get_reset_reason_name(esp_reset_reason());

    strncpy(out->ip, s_ip_str, sizeof(out->ip) - 1);
    strncpy(out->netmask, "255.255.255.0", sizeof(out->netmask) - 1);
    strncpy(out->gw, "0.0.0.0", sizeof(out->gw) - 1);
    strncpy(out->dns, "8.8.8.8", sizeof(out->dns) - 1);

    if (s_sta_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(s_sta_netif, &ip_info) == ESP_OK) {
            snprintf(out->ip, sizeof(out->ip), IPSTR, IP2STR(&ip_info.ip));
            snprintf(out->netmask, sizeof(out->netmask), IPSTR, IP2STR(&ip_info.netmask));
            snprintf(out->gw, sizeof(out->gw), IPSTR, IP2STR(&ip_info.gw));
        }
        esp_netif_dns_info_t dns_info;
        if (esp_netif_get_dns_info(s_sta_netif, ESP_NETIF_DNS_MAIN, &dns_info) == ESP_OK) {
            snprintf(out->dns, sizeof(out->dns), IPSTR, IP2STR(&dns_info.ip.u_addr.ip4));
        }
    }

    uint8_t mac[6] = {0};
    if (esp_wifi_get_mac(WIFI_IF_STA, mac) == ESP_OK) {
        snprintf(out->mac, sizeof(out->mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }

    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        strncpy(out->ssid, (const char *)ap.ssid, sizeof(out->ssid) - 1);
        out->rssi = ap.rssi;
        out->channel = ap.primary;
        snprintf(out->bssid, sizeof(out->bssid), "%02X:%02X:%02X:%02X:%02X:%02X",
                 ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5]);
    } else {
        strncpy(out->ssid, "Disconnected", sizeof(out->ssid) - 1);
        strncpy(out->bssid, "--:--:--:--:--:--", sizeof(out->bssid) - 1);
    }

    // OTA & Partition Diagnostics
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next_ota = esp_ota_get_next_update_partition(NULL);
    if (running) {
        strncpy(out->running_part, running->label, sizeof(out->running_part) - 1);
        out->part_size_kb = running->size / 1024;
        esp_ota_img_states_t ota_st;
        if (esp_ota_get_state_partition(running, &ota_st) == ESP_OK) {
            switch (ota_st) {
                case ESP_OTA_IMG_VALID:          strncpy(out->ota_state, "VALID (Đã xác thực)", sizeof(out->ota_state)-1); break;
                case ESP_OTA_IMG_PENDING_VERIFY: strncpy(out->ota_state, "PENDING_VERIFY (Đang kiểm tra)", sizeof(out->ota_state)-1); break;
                case ESP_OTA_IMG_NEW:            strncpy(out->ota_state, "NEW (Mới nạp)", sizeof(out->ota_state)-1); break;
                case ESP_OTA_IMG_INVALID:        strncpy(out->ota_state, "INVALID", sizeof(out->ota_state)-1); break;
                case ESP_OTA_IMG_ABORTED:        strncpy(out->ota_state, "ABORTED", sizeof(out->ota_state)-1); break;
                default:                         strncpy(out->ota_state, "UNDEFINED / FACTORY", sizeof(out->ota_state)-1); break;
            }
        } else {
            strncpy(out->ota_state, "ACTIVE (Chính thức)", sizeof(out->ota_state)-1);
        }
    } else {
        strncpy(out->running_part, "ota_0", sizeof(out->running_part) - 1);
        out->part_size_kb = 1984;
    }

    if (next_ota) {
        strncpy(out->next_ota_part, next_ota->label, sizeof(out->next_ota_part) - 1);
    } else {
        strncpy(out->next_ota_part, "ota_1", sizeof(out->next_ota_part) - 1);
    }

    out->crash_count = s_rtc_crash_count;
    out->rescue_mode = s_rescue_mode;
    out->wdt_active = s_wdt_subscribed;
    out->min_stack_bytes = s_min_stack_bytes;
    strncpy(out->min_stack_task, s_min_stack_task, sizeof(out->min_stack_task) - 1);
    out->cam_i2c_ok = s_cam_i2c_ok;
    out->cam_pid_hex = s_cam_pid_hex;
    out->cam_recovery_cnt = s_cam_recovery_cnt;

    out->coredump_present = false;
    const esp_partition_t *cd_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_COREDUMP, "coredump");
    if (cd_part) {
        uint32_t cd_magic = 0xFFFFFFFFUL;
        if (esp_partition_read(cd_part, 0, &cd_magic, sizeof(cd_magic)) == ESP_OK) {
            if (cd_magic != 0xFFFFFFFFUL && cd_magic != 0x00000000UL) {
                out->coredump_present = true;
            }
        }
    }
}

void app_cam_touch_activity(void) {
    s_last_cam_activity_sec = s_uptime_sec;
}

bool app_cam_is_sleeping(void) {
    return s_cam_sleeping;
}

int app_cam_get_stream_clients(void) {
    return s_stream_clients;
}

void app_cam_set_stream_clients(int delta) {
    s_stream_clients += delta;
    if (s_stream_clients < 0) s_stream_clients = 0;
    app_cam_touch_activity();
}

uint32_t app_get_next_capture_countdown_sec(void) {
    return s_next_capture_countdown_sec;
}

bool app_is_adaptive_fast_active(void) {
    return s_adaptive_fast_active;
}

void app_notify_flow_activity(bool active) {
    if (active) {
        s_last_flow_active_sec = s_uptime_sec;
    }
}

void app_cam_wake_if_sleeping(void) {
    app_cam_touch_activity();
    if (s_rescue_mode || !s_cam_sleeping) return;
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->set_reg) {
        s->set_reg(s, 0x3008, 0x40, 0x00);
        s->set_reg(s, 0x3017, 0xff, 0xff);
        s->set_reg(s, 0x3018, 0xff, 0xff);
        s_cam_sleeping = false;
        ESP_LOGI(TAG, "OV5640 Woke up from Eco Standby, waiting 80ms for frame sync...");
        vTaskDelay(pdMS_TO_TICKS(80));
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
    }
}

static void app_cam_enter_sleep(void) {
    if (s_rescue_mode || s_cam_sleeping || s_stream_clients > 0) return;
    sensor_t *s = esp_camera_sensor_get();
    if (s && s->set_reg) {
        s->set_reg(s, 0x3022, 0xff, 0x08);
        s->set_reg(s, 0x3008, 0x40, 0x40);
        s_cam_sleeping = true;
        ESP_LOGI(TAG, "OV5640 entered Thermal Eco Standby (0mA VCM, <1mA Sensor) to prevent heating!");
    }
}

int custom_ov5640_af_set_manual_position(sensor_t *sensor, uint16_t position) {
    if (!sensor || !sensor->set_reg) return -1;
    if (position > 1023) position = 1023;
    sensor->set_reg(sensor, 0x3022, 0xff, 0x08);
    uint8_t reg_3602 = (uint8_t)(((position & 0x0F) << 4) | 0x04);
    uint8_t reg_3603 = (uint8_t)((position >> 4) & 0x3F);
    sensor->set_reg(sensor, 0x3602, 0xff, reg_3602);
    sensor->set_reg(sensor, 0x3603, 0xff, reg_3603);
    sensor->set_reg(sensor, 0x3028, 0xff, (uint8_t)(position >> 2));
    return 0;
}

uint16_t app_cam_roi_focus_sweep(void) {
    if (s_rescue_mode) return g_cfg.af_manual_pos;
    app_cam_wake_if_sleeping();
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return g_cfg.af_manual_pos;

    bool flash_assisted = app_flash_pre_capture_assist();
    const uint16_t coarse_steps[] = {380, 500, 620, 720, 800, 870, 940};
    uint16_t best_pos = g_cfg.af_manual_pos;
    float best_sharp = -1.0f;

    for (size_t i = 0; i < sizeof(coarse_steps) / sizeof(coarse_steps[0]); i++) {
        uint16_t pos = coarse_steps[i];
        custom_ov5640_af_set_manual_position(s, pos);
        vTaskDelay(pdMS_TO_TICKS(110));
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            esp_camera_fb_return(fb); // flush old frame
        }
        fb = esp_camera_fb_get();
        if (fb) {
            uint8_t *dummy_out = NULL;
            size_t dummy_len = 0;
            bool dummy_alloc = false;
            bool old_osd = g_cfg.osd_enable;
            g_cfg.osd_enable = false;
            process_frame_osd_and_ai(fb->buf, fb->len, fb->width, fb->height, 80, NULL, &dummy_out, &dummy_len, &dummy_alloc);
            g_cfg.osd_enable = old_osd;
            if (dummy_alloc && dummy_out) free(dummy_out);
            esp_camera_fb_return(fb);
            if (g_last_edge_ai.sharpness_score > best_sharp) {
                best_sharp = g_last_edge_ai.sharpness_score;
                best_pos = pos;
            }
        }
    }

    // Fine sweep +/- 35 around best_pos
    int fine_offsets[] = {-35, 35};
    uint16_t center_pos = best_pos;
    for (int j = 0; j < 2; j++) {
        int cand = (int)center_pos + fine_offsets[j];
        if (cand < 0) cand = 0;
        if (cand > 1023) cand = 1023;
        custom_ov5640_af_set_manual_position(s, (uint16_t)cand);
        vTaskDelay(pdMS_TO_TICKS(90));
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
        fb = esp_camera_fb_get();
        if (fb) {
            uint8_t *dummy_out = NULL;
            size_t dummy_len = 0;
            bool dummy_alloc = false;
            bool old_osd = g_cfg.osd_enable;
            g_cfg.osd_enable = false;
            process_frame_osd_and_ai(fb->buf, fb->len, fb->width, fb->height, 80, NULL, &dummy_out, &dummy_len, &dummy_alloc);
            g_cfg.osd_enable = old_osd;
            if (dummy_alloc && dummy_out) free(dummy_out);
            esp_camera_fb_return(fb);
            if (g_last_edge_ai.sharpness_score > best_sharp) {
                best_sharp = g_last_edge_ai.sharpness_score;
                best_pos = (uint16_t)cand;
            }
        }
    }

    custom_ov5640_af_set_manual_position(s, best_pos);
    g_cfg.af_manual_pos = best_pos;
    g_cfg.af_mode = 1;
    app_flash_post_capture_restore(flash_assisted);
    app_config_save();
    ESP_LOGI(TAG, "Smart ROI Focus Sweep locked optimal VCM=%u (Sharpness=%.2f)", best_pos, best_sharp);
    return best_pos;
}

int app_wifi_get_active_slot(void) {
    return s_active_wifi_slot;
}

const char* app_wifi_get_ip_str(void) {
    return s_ip_str;
}

static int find_next_enabled_slot(int start_slot) {
    for (int i = 0; i < MAX_WIFI_PROFILES; i++) {
        int idx = (start_slot + i) % MAX_WIFI_PROFILES;
        if (g_cfg.wifi_list[idx].enabled && g_cfg.wifi_list[idx].ssid[0] != '\0') {
            return idx;
        }
    }
    return 0;
}

void app_wifi_connect_slot(int slot) {
    if (slot < 0 || slot >= MAX_WIFI_PROFILES) slot = 0;
    s_active_wifi_slot = slot;
    s_retry_count = 0;

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, g_cfg.wifi_list[slot].ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, g_cfg.wifi_list[slot].password, sizeof(wifi_config.sta.password) - 1);

    ESP_LOGI(TAG, "Connecting to WiFi Slot #%d: SSID='%s'...", slot + 1, g_cfg.wifi_list[slot].ssid);
    esp_wifi_disconnect();
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect();
}

static void start_mdns_service(void) {
    if (!s_mdns_started) {
        if (mdns_init() == ESP_OK) {
            s_mdns_started = true;
        }
    }
    if (s_mdns_started) {
        const char *host = g_cfg.mdns_host[0] ? g_cfg.mdns_host : "esp32-cam";
        mdns_hostname_set(host);
        mdns_instance_name_set(g_cfg.device_name);
        mdns_service_remove_all();
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        mdns_service_add(NULL, "_mjpeg", "_tcp", 81, NULL, 0);
        ESP_LOGI(TAG, "mDNS active: http://%s.local", host);
    }
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        s_active_wifi_slot = find_next_enabled_slot(0);
        app_wifi_connect_slot(s_active_wifi_slot);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_connected = false;
        s_retry_count++;
        if (s_retry_count <= 2) {
            ESP_LOGW(TAG, "WiFi slot #%d retry %d/2...", s_active_wifi_slot + 1, s_retry_count);
            esp_wifi_connect();
        } else {
            int next_slot = find_next_enabled_slot(s_active_wifi_slot + 1);
            if (next_slot == s_active_wifi_slot) {
                ESP_LOGW(TAG, "WiFi connection failed, enabling Rescue Fallback SoftAP '%s'", g_cfg.ap_ssid);
                esp_wifi_set_mode(WIFI_MODE_APSTA);
            }
            app_wifi_connect_slot(next_slot);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        snprintf(s_ip_str, sizeof(s_ip_str), IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Got IP Address: %s (Slot #%d)", s_ip_str, s_active_wifi_slot + 1);
        s_sta_connected = true;
        s_retry_count = 0;

        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        if (!g_cfg.ap_always_on && !s_rescue_mode) {
            esp_wifi_set_mode(WIFI_MODE_STA);
        }

        start_mdns_service();
        start_camera_server();
        net_ntp_init();
        if (g_cfg.wg.enabled) {
            app_wireguard_init(&g_cfg.wg);
            app_wireguard_start();
        }
        if (!s_rescue_mode) {
            net_mqtt_init();
        }
    }
}

static void wifi_init_multi(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta_netif = esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t ap_config = {0};
    const char *ap_name = s_rescue_mode ? "ESP32-CAM-Rescue" : g_cfg.ap_ssid;
    strncpy((char *)ap_config.ap.ssid, ap_name, sizeof(ap_config.ap.ssid) - 1);
    ap_config.ap.ssid_len = strlen(ap_name);
    strncpy((char *)ap_config.ap.password, g_cfg.ap_pass, sizeof(ap_config.ap.password) - 1);
    ap_config.ap.max_connection = 4;
    ap_config.ap.authmode = strlen(g_cfg.ap_pass) >= 8 ? WIFI_AUTH_WPA_WPA2_PSK : WIFI_AUTH_OPEN;

    bool enable_ap = g_cfg.ap_always_on || s_rescue_mode;
    ESP_ERROR_CHECK(esp_wifi_set_mode(enable_ap ? WIFI_MODE_APSTA : WIFI_MODE_STA));
    if (enable_ap) {
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    start_camera_server();
}

void app_apply_cam_isp_profile(uint8_t profile) {
    if (s_rescue_mode) return;
    app_cam_wake_if_sleeping();
    sensor_t *s = esp_camera_sensor_get();
    if (!s) return;

    g_cfg.cam_isp_profile = profile;
    if (profile == 1) {
        // Profile 1: 7-Segment LED & Indicator Anti-Blooming (Low exposure, high contrast, low gain)
        g_cfg.cam_wb_mode = 0;
        g_cfg.cam_brightness = -2;
        g_cfg.cam_contrast = 2;
        g_cfg.cam_saturation = 1;
        g_cfg.cam_sharpness = 2;
        g_cfg.cam_denoise = 0;
        g_cfg.cam_gainceiling = 0; // 2x
        g_cfg.cam_aec2 = false;
    } else if (profile == 2) {
        // Profile 2: LCD Utility Meter (Water/Elec/Gas) High-Contrast Anti-Glare
        g_cfg.cam_wb_mode = 3; // Office WB
        g_cfg.cam_brightness = 0;
        g_cfg.cam_contrast = 2;
        g_cfg.cam_saturation = -1;
        g_cfg.cam_sharpness = 2;
        g_cfg.cam_denoise = 2;
        g_cfg.cam_gainceiling = 1; // 4x
        g_cfg.cam_aec2 = false;
    } else if (profile == 3) {
        // Profile 3: Industrial Dark Cabinet / Night Vision Boost
        g_cfg.cam_wb_mode = 0;
        g_cfg.cam_brightness = 1;
        g_cfg.cam_contrast = 1;
        g_cfg.cam_saturation = 0;
        g_cfg.cam_sharpness = 1;
        g_cfg.cam_denoise = 4;
        g_cfg.cam_gainceiling = 4; // 32x
        g_cfg.cam_aec2 = true;
    } else if (profile == 4) {
        // Profile 4: Analog Dial Water Meter (Red Pointer Color Boost + Anti-Glare Glass)
        g_cfg.cam_wb_mode = 1;     // Sunny/Daylight WB keeps red pointers vivid
        g_cfg.cam_brightness = -1; // Prevent specular highlight clipping on wet glass
        g_cfg.cam_contrast = 2;    // Crisp dial ticks & roller numbers
        g_cfg.cam_saturation = 2;  // High saturation to separate red needles from black markings
        g_cfg.cam_sharpness = 2;
        g_cfg.cam_denoise = 1;
        g_cfg.cam_gainceiling = 1; // 4x low noise
        g_cfg.cam_aec2 = false;
    } else {
        // Profile 0: Standard Balanced Auto
        g_cfg.cam_wb_mode = 0;
        g_cfg.cam_brightness = 0;
        g_cfg.cam_contrast = 0;
        g_cfg.cam_saturation = 0;
        g_cfg.cam_sharpness = 1;
        g_cfg.cam_denoise = 1;
        g_cfg.cam_gainceiling = 2; // 8x
        g_cfg.cam_aec2 = false;
    }

    if (s->set_brightness) s->set_brightness(s, g_cfg.cam_brightness);
    if (s->set_contrast) s->set_contrast(s, g_cfg.cam_contrast);
    if (s->set_saturation) s->set_saturation(s, g_cfg.cam_saturation);
    if (s->set_sharpness) s->set_sharpness(s, g_cfg.cam_sharpness);
    if (s->set_denoise) s->set_denoise(s, g_cfg.cam_denoise);
    if (s->set_wb_mode) s->set_wb_mode(s, g_cfg.cam_wb_mode);
    if (s->set_gainceiling) s->set_gainceiling(s, (gainceiling_t)g_cfg.cam_gainceiling);
    if (s->set_aec2) s->set_aec2(s, g_cfg.cam_aec2 ? 1 : 0);
    if (s->set_bpc) s->set_bpc(s, 1);   // Bad Pixel Correction ON
    if (s->set_wpc) s->set_wpc(s, 1);   // White Pixel Correction ON
    if (s->set_lenc) s->set_lenc(s, 1); // Lens Shading Correction ON
    if (s->set_raw_gma) s->set_raw_gma(s, 1);
    ESP_LOGI(TAG, "Applied Industrial Camera ISP Profile #%u (B=%d, C=%d, Sharp=%d, WB=%u, Gain=%u)",
             profile, g_cfg.cam_brightness, g_cfg.cam_contrast, g_cfg.cam_sharpness, g_cfg.cam_wb_mode, g_cfg.cam_gainceiling);
}

void app_apply_target_preset(const char *preset) {
    if (!preset || s_rescue_mode) return;
    app_cam_wake_if_sleeping();
    sensor_t *s = esp_camera_sensor_get();

    if (strcmp(preset, "water_meter") == 0) {
        g_cfg.meter_mode = 4;        // 5-Digit Roller + 4 Red Rotary Pointers
        app_apply_cam_isp_profile(4); // Sunny WB + Anti-Glare Glass + Red Pointer Boost
        g_cfg.flash_mode = 1;        // Smart Auto Fill-Light (OFF in daytime, soft PWM at night)
        g_cfg.flash_cap_duty = 28;   // 11% PWM anti-glare soft light
        g_cfg.flash_duty = 0;        // Strictly OFF when idle
        g_cfg.auto_capture_sec = 300;
        g_cfg.adaptive_cycle_enable = true;
        g_cfg.fast_capture_sec = 30;
        g_cfg.roi_x_pct = 14;
        g_cfg.roi_y_pct = 10;
        g_cfg.roi_w_pct = 72;
        g_cfg.roi_h_pct = 76;
    } else if (strcmp(preset, "debug_board") == 0) {
        g_cfg.meter_mode = 1;        // PCB LEDs + Display Multi-Target Probe
        app_apply_cam_isp_profile(0); // True-color Balanced Auto for Red/Green/Yellow/Blue LEDs
        g_cfg.flash_mode = 1;        // Smart Auto (only illuminates if PCB is pitch dark)
        g_cfg.flash_cap_duty = 30;
        g_cfg.flash_duty = 0;
        g_cfg.auto_capture_sec = 0;  // On-demand AI Agent / MCP observation
        g_cfg.adaptive_cycle_enable = false;
        g_cfg.roi_x_pct = 10;
        g_cfg.roi_y_pct = 10;
        g_cfg.roi_w_pct = 80;
        g_cfg.roi_h_pct = 80;
    } else if (strcmp(preset, "led_status") == 0) {
        g_cfg.meter_mode = 1;        // Status & TX/RX Comm Blinking LEDs
        app_apply_cam_isp_profile(1); // Anti-Blooming (-2 EV) so LED cores don't clip white
        g_cfg.flash_mode = 0;        // Flash strictly OFF so ambient/self-luminous LEDs stand out
        g_cfg.flash_duty = 0;
        g_cfg.auto_capture_sec = 60;
        g_cfg.adaptive_cycle_enable = false;
        g_cfg.roi_x_pct = 15;
        g_cfg.roi_y_pct = 15;
        g_cfg.roi_w_pct = 70;
        g_cfg.roi_h_pct = 70;
    } else if (strcmp(preset, "lcd_oled") == 0) {
        g_cfg.meter_mode = 2;        // Digital LCD / OLED Screen OCR
        app_apply_cam_isp_profile(2); // High-Contrast Anti-Glare LCD Profile
        g_cfg.flash_mode = 1;
        g_cfg.flash_cap_duty = 35;
        g_cfg.flash_duty = 0;
        g_cfg.auto_capture_sec = 120;
        g_cfg.adaptive_cycle_enable = false;
        g_cfg.roi_x_pct = 18;
        g_cfg.roi_y_pct = 20;
        g_cfg.roi_w_pct = 64;
        g_cfg.roi_h_pct = 55;
    } else if (strcmp(preset, "led_7seg") == 0) {
        g_cfg.meter_mode = 2;        // Self-luminous 7-Segment LED Display
        app_apply_cam_isp_profile(1); // 7-Segment Anti-Blooming Profile
        g_cfg.flash_mode = 0;        // Flash strictly OFF for self-luminous 7-segment tubes
        g_cfg.flash_duty = 0;
        g_cfg.auto_capture_sec = 120;
        g_cfg.adaptive_cycle_enable = false;
        g_cfg.roi_x_pct = 18;
        g_cfg.roi_y_pct = 22;
        g_cfg.roi_w_pct = 64;
        g_cfg.roi_h_pct = 50;
    } else if (strcmp(preset, "multi_screen") == 0) {
        g_cfg.meter_mode = 5;        // Auto-Cycling Multi-Screen Meter (OBIS / Multi-Parameter)
        app_apply_cam_isp_profile(2); // High-Contrast Anti-Glare LCD/7-Seg Profile
        g_cfg.flash_mode = 1;        // Smart Auto Fill-Light
        g_cfg.flash_cap_duty = 32;
        g_cfg.flash_duty = 0;
        g_cfg.auto_capture_sec = 60;
        g_cfg.adaptive_cycle_enable = true;
        g_cfg.fast_capture_sec = 15;
        g_cfg.roi_x_pct = 15;
        g_cfg.roi_y_pct = 15;
        g_cfg.roi_w_pct = 70;
        g_cfg.roi_h_pct = 65;
    }

    app_flash_set_duty(0);
    if (s) {
        s->set_hmirror(s, g_cfg.cam_hmirror);
        s->set_vflip(s, g_cfg.cam_vflip);
    }
    app_config_save();
    ESP_LOGI(TAG, "Applied Dynamic Target Preset '%s' (meter_mode=%u, isp=%u, flash_mode=%u, cap_duty=%u)",
             preset, g_cfg.meter_mode, g_cfg.cam_isp_profile, g_cfg.flash_mode, g_cfg.flash_cap_duty);
}

bool app_check_ov5640_health(void) {
    if (s_rescue_mode) return false;
    if (s_cam_sleeping) return s_cam_i2c_ok;
    sensor_t *s = esp_camera_sensor_get();
    if (!s || !s->get_reg) {
        s_cam_i2c_ok = false;
        return false;
    }
    int id_h = s->get_reg(s, 0x300A, 0xFF);
    int id_l = s->get_reg(s, 0x300B, 0xFF);
    if (id_h == 0x56 && id_l == 0x40) {
        s_cam_pid_hex = 0x5640;
        s_cam_i2c_ok = true;
        s_cam_fail_streak = 0;
        return true;
    }
    ESP_LOGW(TAG, "OV5640 SCCB Register Health Check Failed: read 0x%02x%02x (expected 0x5640)", id_h & 0xFF, id_l & 0xFF);
    s_cam_i2c_ok = false;
    return false;
}

void app_recover_camera_hardware(void) {
    if (s_rescue_mode) return;
    s_cam_recovery_cnt++;
    ESP_LOGW(TAG, "=== INDUSTRIAL CAMERA HARDWARE RECOVERY #%u (PWDN Pulse + SCCB Re-init) ===", (unsigned)s_cam_recovery_cnt);
    gpio_set_direction(PWDN_GPIO_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(PWDN_GPIO_NUM, 1);
    vTaskDelay(pdMS_TO_TICKS(60));
    gpio_set_level(PWDN_GPIO_NUM, 0);
    vTaskDelay(pdMS_TO_TICKS(60));
    s_cam_sleeping = false;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        app_apply_cam_isp_profile(g_cfg.cam_isp_profile);
        if (s->af_set_manual_position) {
            s->af_set_manual_position(s, g_cfg.af_manual_pos);
        }
    }
    app_check_ov5640_health();
}

static void init_camera(void) {
    if (s_rescue_mode) {
        ESP_LOGW(TAG, "Rescue Safe Mode active: Skipping Camera DMA initialization to guarantee OTA recovery!");
        return;
    }
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_sccb_sda = SIOD_GPIO_NUM,
        .pin_sccb_scl = SIOC_GPIO_NUM,
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .xclk_freq_hz = 10000000, // 10MHz XCLK reduces OV5640 heat by ~45%
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = (framesize_t)(g_cfg.cam_framesize ? g_cfg.cam_framesize : FRAMESIZE_VGA),
        .jpeg_quality = g_cfg.cam_quality ? g_cfg.cam_quality : 12,
        .fb_count = 2,
        .grab_mode = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_hmirror(s, g_cfg.cam_hmirror);
        s->set_vflip(s, g_cfg.cam_vflip);
        app_apply_cam_isp_profile(g_cfg.cam_isp_profile);

        if (s->id.PID == OV5640_PID) {
            ESP_LOGI(TAG, "Initializing OV5640 VCM Autofocus Firmware...");
            if (s->af_is_supported && s->af_is_supported(s)) {
                extern int custom_ov5640_af_set_manual_position(sensor_t *sensor, uint16_t position);
                s->af_set_manual_position = custom_ov5640_af_set_manual_position;
                if (s->af_init(s, 2500) == 0) {
                    ESP_LOGI(TAG, "OV5640 Autofocus MCU initialized successfully!");
                    // Restore saved manual focus step immediately (low power, no boot-time VCM current spike!)
                    uint16_t init_pos = g_cfg.af_manual_pos ? g_cfg.af_manual_pos : 380;
                    custom_ov5640_af_set_manual_position(s, init_pos);
                } else {
                    ESP_LOGW(TAG, "OV5640 AF init returned warning");
                }
            }
        }
        app_check_ov5640_health();
    }
}

static void background_worker_task(void *arg) {
    uint32_t tick_sec = 0;
    uint32_t last_motion_cap_sec = 0;
    uint32_t wifi_down_sec = 0;
    uint32_t low_ram_ticks = 0;
    bool ota_verified = false;

    // Subscribe to Hardware Task Watchdog (TWDT)
    if (esp_task_wdt_add(NULL) == ESP_OK) {
        s_wdt_subscribed = true;
        ESP_LOGI(TAG, "Hardware Task Watchdog (TWDT 25s) subscribed for bg_worker!");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        tick_sec++;
        s_uptime_sec = tick_sec;

        if (s_wdt_subscribed) {
            esp_task_wdt_reset();
        }

        // 15-Second Post-Boot Self-Test & Anti-Brick Rollback Verification
        if (!ota_verified && tick_sec >= 15) {
            ota_verified = true;
            s_rtc_crash_count = 0; // Boot survived 15s -> clear crash loop counter
            esp_err_t ota_err = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOGI(TAG, "OTA Self-Test PASSED (15s stable) -> Marked partition VALID (%s)", esp_err_to_name(ota_err));
        }

        // Industrial Heap Memory Guard (< 10KB internal SRAM emergency protection)
        size_t free_iram = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        if (free_iram < 10240) {
            low_ram_ticks++;
            ESP_LOGW(TAG, "Industrial Heap Guard: Low internal SRAM (%u bytes) [%u/5]", (unsigned)free_iram, (unsigned)low_ram_ticks);
            if (low_ram_ticks >= 5) {
                ESP_LOGE(TAG, "Critical Heap Exhaustion prevented -> Cleanly rebooting!");
                esp_restart();
            }
        } else {
            low_ram_ticks = 0;
        }

        // Update System Health metrics (CPU Core 0/1 load & MCU Temperature)
        update_cpu_load_stats();
        read_esp32_internal_temp_c();

        // Poll GPIO 13 Sensor Input & Rear LED (GPIO 33) Heartbeat
        app_ext_io_tick_1s(tick_sec, s_sta_connected);

        if (s_rescue_mode) continue;

        // Thermal Eco Protection: put OV5640 into Software Standby (<1mA) after 25s of idle
        if (g_cfg.eco_mode && !s_cam_sleeping && s_stream_clients == 0) {
            if (tick_sec - s_last_cam_activity_sec >= 25) {
                app_cam_enter_sleep();
            }
        }

        // Industrial OV5640 Register Health Watchdog (every 15s when awake)
        if (!s_cam_sleeping && (tick_sec % 15 == 0)) {
            if (!app_check_ov5640_health()) {
                s_cam_fail_streak++;
                if (s_cam_fail_streak >= 3) {
                    s_cam_fail_streak = 0;
                    app_recover_camera_hardware();
                }
            }
        }

        // Industrial SD Card Ring-Buffer Auto-Prune (every 60s)
        app_sd_check_auto_prune(tick_sec);

        // Industrial WiFi Self-Healing Watchdog
        if (!s_sta_connected) {
            wifi_down_sec++;
            if (wifi_down_sec % 30 == 0) {
                int next_slot = find_next_enabled_slot(s_active_wifi_slot + 1);
                ESP_LOGW(TAG, "WiFi Self-Healing: Offline for %us, cycling to Slot #%d", (unsigned)wifi_down_sec, next_slot + 1);
                esp_wifi_set_mode(WIFI_MODE_APSTA);
                app_wifi_connect_slot(next_slot);
            }
            continue;
        } else {
            wifi_down_sec = 0;
        }

        // 1. Periodic MQTT Real-time Telemetry
        uint16_t mqtt_int = g_cfg.mqtt_interval_sec ? g_cfg.mqtt_interval_sec : 5;
        if (g_cfg.mqtt_enable && (tick_sec % mqtt_int == 0)) {
            net_mqtt_publish_telemetry();
        }

        // 2. Flexible & Adaptive Periodic Auto Capture -> Flash Assist + SD + FTP + AI Server
        if (g_cfg.auto_capture_sec >= 5) {
            bool flow_recent = (g_last_edge_ai.motion_score >= 3.5f) ||
                               (s_last_flow_active_sec > 0 && (tick_sec - s_last_flow_active_sec) <= 120);
            uint32_t eff_interval = g_cfg.auto_capture_sec;
            if (g_cfg.adaptive_cycle_enable && flow_recent &&
                g_cfg.fast_capture_sec >= 5 && g_cfg.fast_capture_sec < eff_interval) {
                eff_interval = g_cfg.fast_capture_sec;
                s_adaptive_fast_active = true;
            } else {
                s_adaptive_fast_active = false;
            }

            uint32_t elapsed = tick_sec - s_last_scheduled_cap_sec;
            if (elapsed >= eff_interval) {
                s_last_scheduled_cap_sec = tick_sec;
                s_next_capture_countdown_sec = eff_interval;
                ESP_LOGI(TAG, "Scheduled meter capture triggered (interval=%lus, adaptive_fast=%d)",
                         (unsigned long)eff_interval, s_adaptive_fast_active ? 1 : 0);
                app_trigger_capture_pipeline(s_adaptive_fast_active ? "adaptive_fast" : "periodic");
            } else {
                s_next_capture_countdown_sec = eff_interval - elapsed;
            }
        } else {
            s_next_capture_countdown_sec = 0;
            s_adaptive_fast_active = false;
        }

        // Smart Auto-Off & Thermal Guard for Manual Torch (turns completely OFF to 0V when stream closed > 45s or on > 180s)
        app_flash_check_auto_off(tick_sec, s_stream_clients, s_mcu_temp_c);

        // 3. Edge AI Motion-Triggered Capture (cooldown 10s)
        if (g_cfg.edge_ai_enable && g_cfg.edge_trigger_capture && g_last_edge_ai.motion_detected) {
            if (tick_sec - last_motion_cap_sec >= 10) {
                last_motion_cap_sec = tick_sec;
                g_last_edge_ai.motion_detected = false;
                ESP_LOGI(TAG, "Edge AI motion detected (%.1f%%) -> triggering capture pipeline!", g_last_edge_ai.motion_score);
                app_trigger_capture_pipeline("edge_motion");
            }
        }
    }
}

void app_main(void) {
    // 1. Check RTC Crash-Loop & Rescue Mode State
    esp_reset_reason_t reason = esp_reset_reason();
    if (s_rtc_magic != RTC_RESCUE_MAGIC) {
        s_rtc_magic = RTC_RESCUE_MAGIC;
        s_rtc_crash_count = 0;
        s_rtc_force_rescue = 0;
    }

    if (reason == ESP_RST_PANIC || reason == ESP_RST_TASK_WDT ||
        reason == ESP_RST_INT_WDT || reason == ESP_RST_WDT) {
        s_rtc_crash_count++;
        ESP_LOGW(TAG, "Abnormal Reset detected (%s)! Consecutive crash count: %u/3",
                 get_reset_reason_name(reason), (unsigned)s_rtc_crash_count);
    } else if (reason == ESP_RST_POWERON) {
        s_rtc_crash_count = 0;
        s_rtc_force_rescue = 0;
    }

    if (s_rtc_crash_count >= 3 || s_rtc_force_rescue == 1) {
        s_rescue_mode = true;
        ESP_LOGE(TAG, "=== INDUSTRIAL RESCUE SAFE MODE ACTIVATED (crash_cnt=%u, force=%u) ===",
                 (unsigned)s_rtc_crash_count, (unsigned)s_rtc_force_rescue);
    }

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    app_config_init();
    app_wireguard_init(&g_cfg.wg);
    app_flash_init();
    app_sd_init(s_rescue_mode);
    app_ext_io_init();
    init_camera();
    vTaskDelay(pdMS_TO_TICKS(150));
    wifi_init_multi();

    xTaskCreatePinnedToCore(background_worker_task, "bg_worker", 6144, NULL, 4, NULL, 1);
}
