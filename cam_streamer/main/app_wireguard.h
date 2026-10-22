#ifndef _APP_WIREGUARD_H_
#define _APP_WIREGUARD_H_

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool enabled;
    char private_key[64];
    char address[32];              // e.g. "10.212.156.18"
    char netmask[32];              // e.g. "255.255.255.0"
    char public_key[64];
    char preshared_key[64];
    char endpoint_host[96];        // e.g. "nxchieu.duckdns.org"
    uint16_t endpoint_port;        // e.g. 51820
    uint16_t persistent_keepalive; // e.g. 25
} app_wg_config_t;

esp_err_t app_wireguard_init(const app_wg_config_t *cfg);
esp_err_t app_wireguard_start(void);
esp_err_t app_wireguard_stop(void);
void      app_wireguard_notify_wifi_roamed(void);
bool      app_wireguard_is_connected(void);
uint32_t  app_wireguard_get_reconnect_cnt(void);
uint16_t  app_wireguard_get_mtu(void);
esp_err_t app_wireguard_get_config(app_wg_config_t *cfg);
esp_err_t app_wireguard_parse_conf(const char *conf_str, app_wg_config_t *out_cfg);

#ifdef __cplusplus
}
#endif

#endif /* _APP_WIREGUARD_H_ */
