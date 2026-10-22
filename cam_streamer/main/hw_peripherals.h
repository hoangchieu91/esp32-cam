#ifndef _HW_PERIPHERALS_H_
#define _HW_PERIPHERALS_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Auxiliary Flash LED (GPIO 4 LEDC PWM + Hard-0V NPN Base Clamp)
void app_flash_init(void);
void app_flash_set_duty(uint8_t duty);
uint8_t app_flash_get_duty(void);
bool app_flash_pre_capture_assist(void);
void app_flash_post_capture_restore(bool was_assisted);
void app_flash_check_auto_off(uint32_t tick_sec, int stream_clients, float mcu_temp_c);

// Auxiliary Expansion I/O (GPIO 33 Rear Red LED, GPIO 12 Relay Output, GPIO 13 Pulse/Leak Sensor Input)
void app_ext_io_init(void);
void app_ext_io_tick_1s(uint32_t tick_sec, bool wifi_connected);
void app_set_rear_led_mode(uint8_t mode); // 0=OFF, 1=ON, 2=Heartbeat
uint8_t app_get_rear_led_mode(void);
void app_set_gpio12_relay(bool on);
bool app_get_gpio12_relay(void);
int app_get_gpio13_sensor_level(void);
uint32_t app_get_gpio13_pulse_count(void);
void app_reset_gpio13_pulses(void);

// MicroSD Card 1-Bit SDMMC & Ring-Buffer Auto-Prune
void app_sd_init(bool rescue_mode);
esp_err_t app_format_sd_card(void);
void app_remount_sd_card(void);
int app_prune_oldest_sd_files(int max_delete);
void app_sd_check_auto_prune(uint32_t tick_sec);

#ifdef __cplusplus
}
#endif

#endif /* _HW_PERIPHERALS_H_ */
