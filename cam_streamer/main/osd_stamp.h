#ifndef _OSD_STAMP_H_
#define _OSD_STAMP_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

// Lightweight Edge AI motion, sharpness & ROI Indicator LED metrics computed on RGB565 frame
typedef struct {
    float motion_score;     // 0.0 - 100.0 (% of blocks changed)
    float sharpness_score;  // Edge gradient metric
    uint8_t mean_luma;      // 0 - 255 average brightness
    bool motion_detected;

    // Edge ROI metrics for Status Indicator LED / Meter window
    uint8_t roi_luma;
    uint8_t roi_max_luma;
    uint8_t roi_r;
    uint8_t roi_g;
    uint8_t roi_b;
    char edge_led_state[24]; // e.g., "ON (DO)", "ON (XANH LA)", "OFF"
    char edge_digits[16];    // On-device Edge AI 7-Segment / LCD / Dial Water Meter readout e.g. "0304.954"
    char edge_detail[64];    // Standalone Edge AI detail string
    float edge_flow_lpm;     // On-device estimated flow rate (L/min)
    char edge_filter_state[24]; // "VALID_MONOTONIC", "CARRY_ROLLOVER", "CLAMPED_JITTER"
} edge_ai_result_t;

extern edge_ai_result_t g_last_edge_ai;

/**
 * @brief Run lightweight Edge AI analysis + ROI indicator check + burn timestamp & ROI box onto JPEG image.
 */
bool process_frame_osd_and_ai(
    const uint8_t *src_jpg, size_t src_len,
    uint16_t width, uint16_t height,
    uint8_t quality, const char *extra_tag,
    uint8_t **out_jpg, size_t *out_len, bool *out_allocated
);

#ifdef __cplusplus
}
#endif

#endif /* _OSD_STAMP_H_ */
