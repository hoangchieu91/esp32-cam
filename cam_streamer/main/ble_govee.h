#ifndef _BLE_GOVEE_H_
#define _BLE_GOVEE_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    char mac_str[20];
    char model_str[24];
    char ble_name[32];
    float temp_c;
    float humidity_pct;
    uint8_t battery_pct;
    int8_t rssi;
    uint32_t last_seen_uptime_sec;
    uint32_t rx_packet_cnt;

    // Coexistence & HTTP Sync State
    bool ble_scanning;
    uint32_t cycle_sec;          // 0..59s (0..9s = BLE Scan, 10..59s = Wi-Fi Push & Radio Rest)
    uint32_t post_ok_cnt;
    uint32_t post_err_cnt;
    int last_http_code;
    uint32_t last_post_uptime_sec;
    float last_posted_temp_c;
    char last_auto_cmd[32];
    char last_post_status[64];
} govee_env_state_t;

void app_ble_govee_init(bool rescue_mode);
void app_ble_govee_get_state(govee_env_state_t *out);
esp_err_t app_ble_govee_force_post_now(void);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_GOVEE_H_ */
