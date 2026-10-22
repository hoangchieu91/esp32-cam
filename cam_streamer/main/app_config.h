#ifndef _APP_CONFIG_H_
#define _APP_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "app_wireguard.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_WIFI_PROFILES 5

typedef struct {
    char ssid[33];
    char password[65];
    uint8_t enabled;
} wifi_profile_t;

typedef struct {
    // Device & mDNS
    char mdns_host[32];
    char device_name[32];

    // 5 WiFi Profiles + AP Fallback
    wifi_profile_t wifi_list[MAX_WIFI_PROFILES];
    char ap_ssid[32];
    char ap_pass[32];
    bool ap_always_on;

    // NTP & Time
    char ntp_server[64];
    char tz_str[32];
    bool osd_enable;

    // FTP Config
    bool ftp_enable;
    char ftp_host[64];
    uint16_t ftp_port;
    char ftp_user[32];
    char ftp_pass[32];
    char ftp_remote_dir[64];

    // MQTT Config
    bool mqtt_enable;
    char mqtt_uri[96];
    char mqtt_user[32];
    char mqtt_pass[32];
    char mqtt_topic_prefix[48];
    uint16_t mqtt_interval_sec;

    // Ntfy Config
    bool ntfy_enable;
    char ntfy_url[96];
    char ntfy_token[64];

    // Storage & Flexible/Adaptive Capture Cycle
    bool sd_save_enable;
    uint32_t auto_capture_sec;     // 0 = disabled, 5..86400s (normal/idle interval)
    bool adaptive_cycle_enable;    // Auto switch to fast_capture_sec when meter dial is rotating
    uint16_t fast_capture_sec;     // Fast interval when flow/motion detected in ROI (default 30s)

    // Edge AI & Server AI (10.25.7.171)
    bool edge_ai_enable;
    uint8_t edge_motion_thresh; // 1 - 100
    bool edge_trigger_capture;
    char ai_server_url[96];

    // Smart Meter & Indicator ROI (LED Status, 7-Segment, LCD, Analog Dial Water Meter)
    uint8_t meter_mode;    // 0=All/Auto, 1=LED, 2=7-Seg, 3=LCD, 4=Analog Dial Water Meter (Kim quay)
    uint8_t roi_x_pct;     // 0 - 90 (%)
    uint8_t roi_y_pct;     // 0 - 90 (%)
    uint8_t roi_w_pct;     // 10 - 100 (%)
    uint8_t roi_h_pct;     // 10 - 100 (%)
    bool roi_draw_osd;     // Draw ROI box on captured image
    char meter_baseline[24];       // Prior calibrated reading e.g. "00304.954"
    uint16_t meter_max_flow_lpm;   // Max physical flow rate (L/min, default 50 L/min = 3.0 m3/h)
    int16_t meter_rot_deg;         // Fine rotation angle (-180 .. +180 deg) to deskew tilted meters
    char meter_serial[24];         // Meter Serial Number e.g. "00000198" (or "AUTO" for AI auto-detect)
    char meter_spec[32];           // Meter Model/Standard Spec e.g. "GB/T778-2018 R=80"
    bool serial_auto_detect;       // true = Auto-detect/verify serial & spec from meter face

    // Camera & Auxiliary Flash LED (GPIO 4 PWM) Settings
    uint8_t cam_framesize; // FRAMESIZE_VGA = 10, SVGA = 11, XGA = 12, HD = 13, UXGA = 15
    uint8_t cam_quality;   // 4 - 63 (lower is higher quality)
    int8_t cam_brightness; // -2 to 2
    int8_t cam_contrast;   // -2 to 2
    int8_t cam_saturation; // -2 to 2
    uint8_t cam_hmirror;
    uint8_t cam_vflip;
    uint8_t af_mode;       // 0 = Auto Continuous, 1 = Manual / Single
    uint16_t af_manual_pos; // 0 - 1023
    uint8_t flash_duty;        // 0 - 255 manual continuous torch PWM
    uint8_t flash_mode;        // 0 = Off, 1 = Auto Low-Light (roi_luma < 85), 2 = Always Pre-Capture Fill
    uint8_t flash_cap_duty;    // 0 - 255 anti-glare PWM duty during capture (default 55 ~ 22%)
    uint16_t flash_warmup_ms;  // Pre-capture AE/AWB settle delay (default 350ms)
    bool eco_mode;         // Thermal Eco Mode (Auto sensor sleep + 10MHz XCLK + 10FPS stream cap)

    // Industrial Camera ISP & Storage Ring-Buffer
    uint8_t cam_isp_profile; // 0=Standard Auto, 1=LED 7-Seg Anti-Blooming, 2=LCD Utility Meter High-Contrast, 3=Low-Light Night
    uint8_t cam_wb_mode;     // 0=Auto, 1=Sunny, 2=Cloudy, 3=Office, 4=Home
    int8_t  cam_sharpness;   // -2 to 2
    uint8_t cam_denoise;     // 0 to 8
    uint8_t cam_gainceiling; // 0=2x .. 6=128x
    bool    cam_aec2;        // DSP Night Mode AEC2
    bool    sd_auto_prune;   // Ring-buffer auto-delete oldest cam_*.jpg when SD low
    uint16_t sd_min_free_mb; // Target minimum free MB on SD

    // Industrial WireGuard VPN Tunnel
    app_wg_config_t wg;
} app_config_t;

extern app_config_t g_cfg;

void app_config_init(void);
esp_err_t app_config_save(void);
void app_config_reset_defaults(void);
void app_config_factory_reset_preserve_vpn(void);
esp_err_t app_vault_save_network(void);
esp_err_t app_vault_sync_sdcard(void);
const char *app_vault_get_status_str(void);

#ifdef __cplusplus
}
#endif

#endif /* _APP_CONFIG_H_ */
