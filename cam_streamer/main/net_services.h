#ifndef _NET_SERVICES_H_
#define _NET_SERVICES_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Sync NTP & System Time
void net_ntp_init(void);
void net_ntp_restart(void);
void net_set_manual_epoch(int64_t epoch_sec);
void net_get_time_str(char *buf, size_t len);
bool net_is_time_synced(void);

// FTP Client
esp_err_t net_ftp_upload(const uint8_t *data, size_t len, const char *remote_filename, char *err_msg, size_t err_len);

// Ntfy Notification Client
esp_err_t net_ntfy_send(const char *title, const char *message, const char *priority, const char *tags, const char *click_url);

// MQTT Client
void net_mqtt_init(void);
void net_mqtt_restart(void);
bool net_mqtt_is_connected(void);
esp_err_t net_mqtt_publish_event(const char *subtopic, const char *json_payload);
esp_err_t net_mqtt_publish_retained(const char *subtopic, const char *json_payload);
void net_mqtt_publish_discovery(void);
void net_mqtt_publish_telemetry(void);

// Server AI (10.25.7.171) HTTP POST Image & Get Deep Analysis JSON
esp_err_t net_ai_server_analyze(const uint8_t *jpg_data, size_t jpg_len, const char *filename, char *resp_json, size_t resp_max_len);

// Latest AI result cached on ESP32 for Web UI
extern char g_last_server_ai_json[2048];
extern char g_last_ftp_status[64];
extern char g_last_ntfy_status[64];

#ifdef __cplusplus
}
#endif

#endif /* _NET_SERVICES_H_ */
