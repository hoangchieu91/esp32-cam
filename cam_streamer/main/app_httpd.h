#ifndef _APP_HTTPD_H_
#define _APP_HTTPD_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "hw_peripherals.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float mcu_temp_c;
    float vdd33_volts;
    uint32_t adc_vref_mv;
    float cpu_load_total;
    float cpu0_load;
    float cpu1_load;
    uint32_t task_count;
    uint32_t cpu_freq_mhz;
    const char *reset_reason;
    char ip[20];
    char netmask[20];
    char gw[20];
    char dns[20];
    char mac[20];
    char bssid[20];
    uint8_t channel;
    int8_t rssi;
    char ssid[33];

    // Industrial OTA, Partition & Rescue Mode Info
    char running_part[16];
    char next_ota_part[16];
    char ota_state[28];
    uint32_t part_size_kb;
    uint32_t crash_count;
    bool rescue_mode;
    bool wdt_active;

    // Industrial Camera Bus, Stack Watermark & CoreDump Diagnostics
    uint32_t min_stack_bytes;
    char min_stack_task[20];
    bool cam_i2c_ok;
    uint16_t cam_pid_hex;
    uint32_t cam_recovery_cnt;
    bool coredump_present;
} app_sys_health_t;

void start_camera_server(void);
void app_trigger_capture_pipeline(const char *source);
esp_err_t app_execute_capture_pipeline(const char *mode, char *out_filename, size_t fn_len, char *out_ftp_st, size_t ftp_len);
void app_set_sd_mounted(bool mounted, uint64_t total_mb, uint64_t fat_total_mb, uint64_t free_mb, const char *card_name);

// Camera ISP, Presets & Autofocus Sweep
void app_apply_cam_isp_profile(uint8_t profile);
void app_apply_target_preset(const char *preset);
bool app_check_ov5640_health(void);
void app_recover_camera_hardware(void);
void app_cam_release_vcm_coil(void);
uint16_t app_cam_trigger_af_and_lock(void);
uint16_t app_cam_roi_focus_sweep(void);

// WiFi, OTA & System Health Helper callbacks defined in cam_streamer.c
int app_wifi_get_active_slot(void);
const char* app_wifi_get_ip_str(void);
void app_wifi_connect_slot(int slot);
void app_get_sys_health(app_sys_health_t *out);
void app_set_rescue_mode_reboot(bool enable_rescue);

// Thermal / Eco Mode control & Adaptive Scheduler
void app_cam_touch_activity(void);
void app_cam_wake_if_sleeping(void);
bool app_cam_is_sleeping(void);
int  app_cam_get_stream_clients(void);
void app_cam_set_stream_clients(int delta);
uint32_t app_get_next_capture_countdown_sec(void);
bool app_is_adaptive_fast_active(void);
void app_notify_flow_activity(bool active);

#ifdef __cplusplus
}
#endif

#endif /* _APP_HTTPD_H_ */
