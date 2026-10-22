#include "hw_peripherals.h"
#include "app_httpd.h"
#include "app_config.h"
#include "app_wireguard.h"
#include "osd_stamp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_rom_sys.h"
#include "soc/rtc_io_reg.h"
#include "soc/soc.h"

#include <string.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

static const char *TAG = "hw_periph";

#define FLASH_GPIO_NUM     4
#define REAR_LED_GPIO_NUM 33
#define RELAY_GPIO12_NUM  12
#define SENSOR_GPIO13_NUM 13

static uint8_t s_current_flash_duty = 0;
static bool s_flash_initialized = false;
static uint32_t s_flash_on_since_sec = 0;
static uint32_t s_latest_tick_sec = 0;
static bool s_rescue_flag = false;

static uint8_t s_rear_led_mode = 2; // 0=OFF, 1=ON, 2=Heartbeat
static bool s_gpio12_relay_on = false;
static uint32_t s_gpio13_pulse_cnt = 0;
static int s_prev_gpio13_level = 1;

static sdmmc_card_t *s_sd_card_ptr = NULL;

// ============================================================================
// 1. AUXILIARY FLASH LED (GPIO 4 LEDC PWM + HARD 0.00V IDLE CLAMP)
// ============================================================================
void app_flash_init(void) {
    if (s_rescue_flag) return;
    if (!s_flash_initialized) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_8_BIT,
            .timer_num        = LEDC_TIMER_1,
            .freq_hz          = 5000,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        if (ledc_timer_config(&ledc_timer) == ESP_OK) {
            s_flash_initialized = true;
        }
    }
    app_flash_set_duty(0);
    ESP_LOGI(TAG, "Auxiliary Flash LED PWM initialized on GPIO %d (Hard-OFF 0V idle protection active)", FLASH_GPIO_NUM);
}

void app_flash_set_duty(uint8_t duty) {
    if (s_rescue_flag) return;
    if (!s_flash_initialized) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_8_BIT,
            .timer_num        = LEDC_TIMER_1,
            .freq_hz          = 5000,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        if (ledc_timer_config(&ledc_timer) == ESP_OK) {
            s_flash_initialized = true;
        }
    }
    s_current_flash_duty = duty;
    if (duty == 0) {
        s_flash_on_since_sec = 0;
        if (s_flash_initialized) {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
            ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        }
        // Explicitly disable any SDMMC internal pull-up on GPIO 4 and clamp NPN transistor base to 0.00V
        gpio_reset_pin(FLASH_GPIO_NUM);
        gpio_pullup_dis(FLASH_GPIO_NUM);
        gpio_pulldown_en(FLASH_GPIO_NUM);
        gpio_set_direction(FLASH_GPIO_NUM, GPIO_MODE_OUTPUT);
        gpio_set_level(FLASH_GPIO_NUM, 0);
        return;
    }

    if (s_flash_on_since_sec == 0) {
        s_flash_on_since_sec = s_latest_tick_sec ? s_latest_tick_sec : 1;
    }
    gpio_pullup_dis(FLASH_GPIO_NUM);
    gpio_pulldown_en(FLASH_GPIO_NUM);
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = FLASH_GPIO_NUM,
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_1,
        .duty           = duty,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

uint8_t app_flash_get_duty(void) {
    return s_current_flash_duty;
}

bool app_flash_pre_capture_assist(void) {
    if (s_rescue_flag) return false;
    if (g_cfg.flash_duty >= 10) return false;

    if (g_cfg.flash_mode == 0) {
        if (s_current_flash_duty != 0) app_flash_set_duty(0);
        return false;
    }

    bool need_flash = false;
    if (g_cfg.flash_mode == 2) {
        need_flash = true;
    } else if (g_cfg.flash_mode == 1) {
        int ov_avg = -1;
        sensor_t *s = esp_camera_sensor_get();
        if (s && s->get_reg) {
            ov_avg = s->get_reg(s, 0x56A1, 0xFF);
        }
        uint8_t dark_thresh_roi = (g_cfg.meter_mode == 1 || g_cfg.meter_mode == 2) ? 42 : 78;
        int dark_thresh_ov = (g_cfg.meter_mode == 1 || g_cfg.meter_mode == 2) ? 28 : 48;

        if (ov_avg >= 0 && ov_avg < dark_thresh_ov) {
            need_flash = true;
        } else if (g_last_edge_ai.roi_luma > 0 && g_last_edge_ai.roi_luma < dark_thresh_roi) {
            need_flash = true;
        } else if (g_last_edge_ai.mean_luma > 0 && g_last_edge_ai.mean_luma < dark_thresh_roi) {
            need_flash = true;
        }
    }
    if (!need_flash) {
        if (s_current_flash_duty != 0) app_flash_set_duty(0);
        return false;
    }

    uint8_t target_duty = g_cfg.flash_cap_duty ? g_cfg.flash_cap_duty : 28;
    app_flash_set_duty(target_duty);

    uint16_t warmup = g_cfg.flash_warmup_ms ? g_cfg.flash_warmup_ms : 350;
    if (warmup > 1500) warmup = 1500;
    vTaskDelay(pdMS_TO_TICKS(warmup));

    for (int k = 0; k < 2; k++) {
        camera_fb_t *flush_fb = esp_camera_fb_get();
        if (flush_fb) {
            esp_camera_fb_return(flush_fb);
            vTaskDelay(pdMS_TO_TICKS(35));
        }
    }
    ESP_LOGI(TAG, "Smart Pre-Capture Flash Assist active: PWM=%u/255 (%u%%), warmup=%ums",
             target_duty, (unsigned)(target_duty * 100 / 255), warmup);
    return true;
}

void app_flash_post_capture_restore(bool was_assisted) {
    if (!was_assisted && g_cfg.flash_duty == 0 && s_current_flash_duty == 0) return;
    app_flash_set_duty(g_cfg.flash_duty);
}

void app_flash_check_auto_off(uint32_t tick_sec, int stream_clients, float mcu_temp_c) {
    s_latest_tick_sec = tick_sec;
    if (s_current_flash_duty > 0 && s_flash_on_since_sec > 0) {
        uint32_t on_dur = (tick_sec >= s_flash_on_since_sec) ? (tick_sec - s_flash_on_since_sec) : 0;
        bool no_stream_timeout = (stream_clients == 0 && on_dur >= 45);
        if (no_stream_timeout || on_dur >= 180 || mcu_temp_c > 72.0f) {
            ESP_LOGI(TAG, "Smart Flash Auto-Off Guard: Turning manual torch OFF (0V) after %lus (stream_clients=%d)",
                     (unsigned long)on_dur, stream_clients);
            g_cfg.flash_duty = 0;
            app_flash_set_duty(0);
        }
    }
}

// ============================================================================
// 2. AUXILIARY EXPANSION I/O (GPIO 33 LED, GPIO 12 RELAY, GPIO 13 SENSOR)
// ============================================================================
void app_ext_io_init(void) {
    // 1. GPIO 33: Built-in Rear Red Status LED (Active-LOW: 0=ON, 1=OFF) — zero camera glare!
    gpio_reset_pin(REAR_LED_GPIO_NUM);
    gpio_set_direction(REAR_LED_GPIO_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(REAR_LED_GPIO_NUM, (s_rear_led_mode == 1) ? 0 : 1);

    // 2. GPIO 12 (SD_DATA2): Keep pulled HIGH as input unless Relay Output is explicitly activated,
    // so SD card DAT2/DAT3 bus lines stay cleanly at 3.3V high-impedance!
    gpio_reset_pin(RELAY_GPIO12_NUM);
    if (s_gpio12_relay_on) {
        gpio_set_direction(RELAY_GPIO12_NUM, GPIO_MODE_OUTPUT);
        gpio_set_level(RELAY_GPIO12_NUM, 1);
    } else {
        gpio_set_direction(RELAY_GPIO12_NUM, GPIO_MODE_INPUT);
        gpio_pulldown_dis(RELAY_GPIO12_NUM);
        gpio_pullup_en(RELAY_GPIO12_NUM);
    }

    // 3. GPIO 13 (SD_DATA3 / CS): Input with Pull-Up enabled (keeps SD Card CS HIGH in SDMMC mode)
    gpio_reset_pin(SENSOR_GPIO13_NUM);
    gpio_set_direction(SENSOR_GPIO13_NUM, GPIO_MODE_INPUT);
    gpio_pulldown_dis(SENSOR_GPIO13_NUM);
    gpio_pullup_en(SENSOR_GPIO13_NUM);
    s_prev_gpio13_level = gpio_get_level(SENSOR_GPIO13_NUM);
    ESP_LOGI(TAG, "Expansion I/O initialized: GPIO33=RearLED(Heartbeat), GPIO12=RelayOut(LOW), GPIO13=SensorIn(%d)", s_prev_gpio13_level);
}

void app_ext_io_tick_1s(uint32_t tick_sec, bool wifi_connected) {
    int cur_g13 = gpio_get_level(SENSOR_GPIO13_NUM);
    if (cur_g13 != s_prev_gpio13_level) {
        if (cur_g13 == 0) s_gpio13_pulse_cnt++;
        s_prev_gpio13_level = cur_g13;
    }
    if (s_rear_led_mode == 2) {
        if (wifi_connected && (tick_sec % 5 == 0)) {
            gpio_set_level(REAR_LED_GPIO_NUM, 0); // Brief 35ms heartbeat flash on back of PCB
            vTaskDelay(pdMS_TO_TICKS(35));
            gpio_set_level(REAR_LED_GPIO_NUM, 1);
        } else if (!wifi_connected) {
            gpio_set_level(REAR_LED_GPIO_NUM, (tick_sec % 2 == 0) ? 0 : 1);
        } else {
            gpio_set_level(REAR_LED_GPIO_NUM, 1);
        }
    }
}

void app_set_rear_led_mode(uint8_t mode) {
    s_rear_led_mode = (mode > 2) ? 2 : mode;
    if (s_rear_led_mode == 0) {
        gpio_set_level(REAR_LED_GPIO_NUM, 1);
    } else if (s_rear_led_mode == 1) {
        gpio_set_level(REAR_LED_GPIO_NUM, 0);
    }
}

uint8_t app_get_rear_led_mode(void) {
    return s_rear_led_mode;
}

void app_set_gpio12_relay(bool on) {
    s_gpio12_relay_on = on;
    gpio_set_direction(RELAY_GPIO12_NUM, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO12_NUM, on ? 1 : 0);
    ESP_LOGI(TAG, "Auxiliary Relay/Valve Output GPIO 12 set to %s", on ? "HIGH (ON)" : "LOW (OFF)");
}

bool app_get_gpio12_relay(void) {
    return s_gpio12_relay_on;
}

int app_get_gpio13_sensor_level(void) {
    return gpio_get_level(SENSOR_GPIO13_NUM);
}

uint32_t app_get_gpio13_pulse_count(void) {
    return s_gpio13_pulse_cnt;
}

void app_reset_gpio13_pulses(void) {
    s_gpio13_pulse_cnt = 0;
}

// ============================================================================
// 3. MICROSD CARD 1-BIT SDMMC & RING-BUFFER AUTO-PRUNE
// ============================================================================
static void prepare_sdmmc_pins_before_mount(void) {
    REG_WRITE(RTC_IO_DIG_PAD_HOLD_REG, 0);
    CLEAR_PERI_REG_MASK(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_HOLD | RTC_IO_TOUCH_PAD0_MUX_SEL);
    CLEAR_PERI_REG_MASK(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD | RTC_IO_TOUCH_PAD2_MUX_SEL);
    CLEAR_PERI_REG_MASK(RTC_IO_TOUCH_PAD3_REG, RTC_IO_TOUCH_PAD3_HOLD | RTC_IO_TOUCH_PAD3_MUX_SEL);
    CLEAR_PERI_REG_MASK(RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_HOLD | RTC_IO_TOUCH_PAD4_MUX_SEL);
    CLEAR_PERI_REG_MASK(RTC_IO_TOUCH_PAD5_REG, RTC_IO_TOUCH_PAD5_HOLD | RTC_IO_TOUCH_PAD5_MUX_SEL);
    CLEAR_PERI_REG_MASK(RTC_IO_TOUCH_PAD6_REG, RTC_IO_TOUCH_PAD6_HOLD | RTC_IO_TOUCH_PAD6_MUX_SEL);

    const int sd_pins[] = {2, 4, 12, 13, 14, 15};
    for (int i = 0; i < 6; i++) {
        gpio_hold_dis((gpio_num_t)sd_pins[i]);
    }

    // Standard SD Physical Layer Power-Up Sequence:
    // Hold CS/DAT3 (GPIO 13) and CMD (GPIO 15) HIGH while sending 80 dummy clock pulses on CLK (GPIO 14)
    const int pullup_pins[] = {2, 12, 13, 14, 15};
    for (int i = 0; i < 5; i++) {
        gpio_num_t pin = (gpio_num_t)pullup_pins[i];
        gpio_reset_pin(pin);
        gpio_pulldown_dis(pin);
        gpio_pullup_en(pin);
    }
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, 1);
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_15, 1);
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
    for (int clk = 0; clk < 80; clk++) {
        gpio_set_level(GPIO_NUM_14, 0);
        esp_rom_delay_us(4);
        gpio_set_level(GPIO_NUM_14, 1);
        esp_rom_delay_us(4);
    }
    for (int i = 0; i < 5; i++) {
        gpio_num_t pin = (gpio_num_t)pullup_pins[i];
        gpio_set_direction(pin, GPIO_MODE_INPUT);
        gpio_pulldown_dis(pin);
        gpio_pullup_en(pin);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
}

void app_sd_init(bool rescue_mode) {
    s_rescue_flag = rescue_mode;
    if (rescue_mode) {
        app_set_sd_mounted(false, 0, 0, 0, "Rescue-Skipped");
        return;
    }
    if (s_sd_card_ptr != NULL) {
        return; // Already mounted
    }
    ESP_LOGI(TAG, "Initializing SD Card (1-bit Mode)...");
    prepare_sdmmc_pins_before_mount();

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 8,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.slot = SDMMC_HOST_SLOT_1;
    host.flags = SDMMC_HOST_FLAG_1BIT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &s_sd_card_ptr);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD mount at 20MHz returned 0x%x (%s), retrying at 10MHz...", ret, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(40));
        prepare_sdmmc_pins_before_mount();
        host.max_freq_khz = 10000;
        ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &s_sd_card_ptr);
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SD mount at 10MHz returned 0x%x (%s), retrying at 4MHz...", ret, esp_err_to_name(ret));
        vTaskDelay(pdMS_TO_TICKS(40));
        prepare_sdmmc_pins_before_mount();
        host.max_freq_khz = 4000;
        ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &s_sd_card_ptr);
    }
    if (ret != ESP_OK) {
        char err_tag[32];
        snprintf(err_tag, sizeof(err_tag), "Lỏng khe (0x%x)", (unsigned)ret);
        ESP_LOGE(TAG, "Failed to mount SD card: %s (%s)", esp_err_to_name(ret), err_tag);
        app_set_sd_mounted(false, 0, 0, 0, err_tag);
    } else {
        ESP_LOGI(TAG, "SD Card mounted successfully!");
        uint64_t hw_total_mb = ((uint64_t)s_sd_card_ptr->csd.capacity * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
        FATFS *fs;
        DWORD fre_clust;
        uint64_t free_mb = hw_total_mb;
        uint64_t fat_total_mb = hw_total_mb;
        if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
            uint64_t tot_sectors = (uint64_t)(fs->n_fatent - 2) * (uint64_t)fs->csize;
            uint64_t free_sectors = (uint64_t)fre_clust * (uint64_t)fs->csize;
            fat_total_mb = (tot_sectors * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
            free_mb = (free_sectors * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
            ESP_LOGI(TAG, "SD Card HW=%lluMB, FAT_Volume=%lluMB, Free=%lluMB",
                     (unsigned long long)hw_total_mb, (unsigned long long)fat_total_mb, (unsigned long long)free_mb);
        }
        app_set_sd_mounted(true, hw_total_mb, fat_total_mb, free_mb, s_sd_card_ptr->cid.name);
        app_vault_sync_sdcard();
        app_wireguard_init(&g_cfg.wg);
    }
    // Always reclaim GPIO 4 after SDMMC probing so SD_DATA1 pull-up never turns on the Flash LED
    app_flash_set_duty(s_current_flash_duty);
}

esp_err_t app_format_sd_card(void) {
    if (s_rescue_flag || !s_sd_card_ptr) return ESP_FAIL;
    ESP_LOGW(TAG, "Formatting SD Card FAT32 filesystem...");
    esp_err_t err = esp_vfs_fat_sdcard_format("/sdcard", s_sd_card_ptr);
    if (err == ESP_OK) {
        FATFS *fs;
        DWORD fre_clust;
        uint64_t hw_total_mb = ((uint64_t)s_sd_card_ptr->csd.capacity * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
        uint64_t fat_total_mb = hw_total_mb, free_mb = hw_total_mb;
        if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
            fat_total_mb = ((uint64_t)(fs->n_fatent - 2) * fs->csize * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
            free_mb = ((uint64_t)fre_clust * fs->csize * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
        }
        app_set_sd_mounted(true, hw_total_mb, fat_total_mb, free_mb, s_sd_card_ptr->cid.name);
        app_vault_sync_sdcard();
    }
    return err;
}

void app_remount_sd_card(void) {
    if (s_rescue_flag) return;
    ESP_LOGW(TAG, "Industrial SD Self-Healing: Unmounting & remounting SDMMC...");
    if (s_sd_card_ptr) {
        esp_vfs_fat_sdcard_unmount("/sdcard", s_sd_card_ptr);
        s_sd_card_ptr = NULL;
    }
    vTaskDelay(pdMS_TO_TICKS(80));
    app_sd_init(s_rescue_flag);
}

int app_prune_oldest_sd_files(int max_delete) {
    if (s_rescue_flag || !s_sd_card_ptr || max_delete <= 0) return 0;
    int deleted = 0;
    for (int iter = 0; iter < max_delete; iter++) {
        DIR *dir = opendir("/sdcard");
        if (!dir) break;
        char oldest_rel[160] = {0};
        char oldest_key[64] = {0};
        char oldest_dir[64] = {0};
        struct dirent *de;
        while ((de = readdir(dir)) != NULL) {
            if (de->d_type == DT_REG && strncmp(de->d_name, "cam_", 4) == 0 && strstr(de->d_name, ".jpg")) {
                if (oldest_key[0] == '\0' || strcmp(de->d_name, oldest_key) < 0) {
                    strncpy(oldest_key, de->d_name, sizeof(oldest_key) - 1);
                    strncpy(oldest_rel, de->d_name, sizeof(oldest_rel) - 1);
                    oldest_dir[0] = '\0';
                }
            } else if (de->d_type == DT_DIR &&
                       ((strlen(de->d_name) == 10 && de->d_name[0] == '2' && de->d_name[4] == '-' && de->d_name[7] == '-') ||
                        strcmp(de->d_name, "unsynced") == 0)) {
                char sub_path[64];
                snprintf(sub_path, sizeof(sub_path), "/sdcard/%.16s", de->d_name);
                DIR *sdir = opendir(sub_path);
                if (sdir) {
                    struct dirent *sde;
                    int cnt = 0;
                    while ((sde = readdir(sdir)) != NULL) {
                        if (sde->d_type == DT_REG && strncmp(sde->d_name, "cam_", 4) == 0 && strstr(sde->d_name, ".jpg")) {
                            cnt++;
                            if (oldest_key[0] == '\0' || strcmp(sde->d_name, oldest_key) < 0) {
                                strncpy(oldest_key, sde->d_name, sizeof(oldest_key) - 1);
                                snprintf(oldest_rel, sizeof(oldest_rel), "%.16s/%.64s", de->d_name, sde->d_name);
                                strncpy(oldest_dir, sub_path, sizeof(oldest_dir) - 1);
                            }
                        }
                    }
                    closedir(sdir);
                    if (cnt == 0) rmdir(sub_path);
                }
            }
        }
        closedir(dir);
        if (oldest_rel[0] != '\0') {
            char path[200];
            snprintf(path, sizeof(path), "/sdcard/%s", oldest_rel);
            if (unlink(path) == 0) {
                deleted++;
                ESP_LOGI(TAG, "SD Ring-Buffer Auto-Prune deleted oldest file: %s", path);
                if (oldest_dir[0] != '\0') rmdir(oldest_dir);
            } else {
                break;
            }
        } else {
            break;
        }
    }
    if (deleted > 0 && s_sd_card_ptr) {
        FATFS *fs;
        DWORD fre_clust;
        uint64_t hw_total_mb = ((uint64_t)s_sd_card_ptr->csd.capacity * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
        uint64_t fat_total_mb = hw_total_mb, free_mb = hw_total_mb;
        if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
            fat_total_mb = ((uint64_t)(fs->n_fatent - 2) * fs->csize * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
            free_mb = ((uint64_t)fre_clust * fs->csize * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
        }
        app_set_sd_mounted(true, hw_total_mb, fat_total_mb, free_mb, s_sd_card_ptr->cid.name);
    }
    return deleted;
}

void app_sd_check_auto_prune(uint32_t tick_sec) {
    if (!s_rescue_flag && !s_sd_card_ptr && tick_sec == 15) {
        app_remount_sd_card();
    }
    if (g_cfg.sd_auto_prune && s_sd_card_ptr && (tick_sec % 60 == 0)) {
        FATFS *fs;
        DWORD fre_clust;
        if (f_getfree("0:", &fre_clust, &fs) == FR_OK) {
            uint64_t free_mb = ((uint64_t)fre_clust * fs->csize * s_sd_card_ptr->csd.sector_size) / (1024 * 1024);
            uint16_t min_mb = g_cfg.sd_min_free_mb ? g_cfg.sd_min_free_mb : 200;
            if (free_mb < min_mb) {
                app_prune_oldest_sd_files(5);
            }
        }
    }
}
