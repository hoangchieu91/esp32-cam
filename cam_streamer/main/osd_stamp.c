#include "osd_stamp.h"
#include "app_config.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "osd_stamp";

edge_ai_result_t g_last_edge_ai = {
    .motion_score = 0.0f,
    .sharpness_score = 0.0f,
    .mean_luma = 0,
    .motion_detected = false,
    .roi_luma = 0,
    .roi_max_luma = 0,
    .roi_r = 0,
    .roi_g = 0,
    .roi_b = 0,
    .edge_led_state = "OFF",
    .edge_digits = "----"
};

// 7-Segment bitmask table: bit6=a, bit5=b, bit4=c, bit3=d, bit2=e, bit1=f, bit0=g
static const struct { uint8_t mask; char ch; } s_seg7_table[] = {
    {0b1111110, '0'},
    {0b0110000, '1'},
    {0b0000110, '1'},
    {0b1101101, '2'},
    {0b1111001, '3'},
    {0b0110011, '4'},
    {0b1011011, '5'},
    {0b1011111, '6'},
    {0b0011111, '6'},
    {0b1110000, '7'},
    {0b1110010, '7'},
    {0b1111111, '8'},
    {0b1111011, '9'},
    {0b1110011, '9'},
};

static char decode_edge_7seg_box(const uint8_t *rgb565, uint16_t width, int x0, int y0, int bw, int bh, uint8_t thresh, bool invert) {
    if (bh < 10 || bw < 3) return 0;
    if (bw * 3 < bh) return '1'; // Narrow vertical bar is '1'

    // Sample 7 segments (a,b,c,d,e,f,g) inside [x0..x0+bw, y0..y0+bh]
    const int seg_boxes[7][4] = {
        {25, 75,  2, 18}, // a: top
        {70, 98, 12, 46}, // b: top-right
        {70, 98, 54, 88}, // c: bottom-right
        {25, 75, 82, 98}, // d: bottom
        { 2, 30, 54, 88}, // e: bottom-left
        { 2, 30, 12, 46}, // f: top-left
        {25, 75, 42, 58}, // g: middle
    };

    uint8_t mask = 0;
    for (int s = 0; s < 7; s++) {
        int sx0 = x0 + (bw * seg_boxes[s][0]) / 100;
        int sx1 = x0 + (bw * seg_boxes[s][1]) / 100;
        int sy0 = y0 + (bh * seg_boxes[s][2]) / 100;
        int sy1 = y0 + (bh * seg_boxes[s][3]) / 100;
        if (sx1 <= sx0) sx1 = sx0 + 1;
        if (sy1 <= sy0) sy1 = sy0 + 1;

        int active = 0, total = 0;
        for (int y = sy0; y < sy1; y += 2) {
            for (int x = sx0; x < sx1; x += 2) {
                const uint8_t *p = rgb565 + (y * width + x) * 2;
                uint8_t r = p[0] & 0xF8;
                uint8_t g = ((p[0] & 0x07) << 5) | ((p[1] & 0xE0) >> 3);
                uint8_t b = (p[1] & 0x1F) << 3;
                uint8_t v = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
                bool on = invert ? (v < thresh) : (v > thresh);
                if (on) active++;
                total++;
            }
        }
        if (total > 0 && (active * 100 / total) >= 38) {
            mask |= (1 << (6 - s));
        }
    }

    for (size_t i = 0; i < sizeof(s_seg7_table)/sizeof(s_seg7_table[0]); i++) {
        if (s_seg7_table[i].mask == mask) return s_seg7_table[i].ch;
    }
    return 0;
}

// Previous 32x24 downsampled luma grid for fast Edge AI motion detection
#define GRID_W 32
#define GRID_H 24
static uint8_t s_prev_grid[GRID_W * GRID_H];
static bool s_grid_initialized = false;

// Standard 8x8 bitmap font for ASCII 32..95 (Space to '_')
static const uint8_t font8x8_basic[64][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // 32 ' '
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // 33 '!'
    {0x6C,0x6C,0x48,0x00,0x00,0x00,0x00,0x00}, // 34 '"'
    {0x6C,0x6C,0xFE,0x6C,0xFE,0x6C,0x6C,0x00}, // 35 '#'
    {0x18,0x3E,0x60,0x3C,0x06,0x7C,0x18,0x00}, // 36 '$'
    {0x00,0xC6,0xCC,0x18,0x30,0x66,0xC6,0x00}, // 37 '%'
    {0x38,0x6C,0x38,0x76,0xDC,0xCC,0x76,0x00}, // 38 '&'
    {0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00}, // 39 '''
    {0x0C,0x18,0x30,0x30,0x30,0x18,0x0C,0x00}, // 40 '('
    {0x30,0x18,0x0C,0x0C,0x0C,0x18,0x30,0x00}, // 41 ')'
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // 42 '*'
    {0x00,0x18,0x18,0x7E,0x18,0x18,0x00,0x00}, // 43 '+'
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30}, // 44 ','
    {0x00,0x00,0x00,0x7E,0x00,0x00,0x00,0x00}, // 45 '-'
    {0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00}, // 46 '.'
    {0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00}, // 47 '/'
    {0x7C,0xC6,0xCE,0xDE,0xF6,0xE6,0x7C,0x00}, // 48 '0'
    {0x18,0x38,0x18,0x18,0x18,0x18,0x7E,0x00}, // 49 '1'
    {0x7C,0xC6,0x06,0x1C,0x30,0x66,0xFE,0x00}, // 50 '2'
    {0x7C,0xC6,0x06,0x3C,0x06,0xC6,0x7C,0x00}, // 51 '3'
    {0x1C,0x3C,0x6C,0xCC,0xFE,0x0C,0x1E,0x00}, // 52 '4'
    {0xFE,0xC0,0xFC,0x06,0x06,0xC6,0x7C,0x00}, // 53 '5'
    {0x38,0x60,0xC0,0xFC,0xC6,0xC6,0x7C,0x00}, // 54 '6'
    {0xFE,0xC6,0x0C,0x18,0x30,0x30,0x30,0x00}, // 55 '7'
    {0x7C,0xC6,0xC6,0x7C,0xC6,0xC6,0x7C,0x00}, // 56 '8'
    {0x7C,0xC6,0xC6,0x7E,0x06,0x0C,0x78,0x00}, // 57 '9'
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00}, // 58 ':'
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30}, // 59 ';'
    {0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x00}, // 60 '<'
    {0x00,0x00,0x7E,0x00,0x00,0x7E,0x00,0x00}, // 61 '='
    {0x60,0x30,0x18,0x0C,0x18,0x30,0x60,0x00}, // 62 '>'
    {0x7C,0xC6,0x0C,0x18,0x18,0x00,0x18,0x00}, // 63 '?'
    {0x7C,0xC6,0xDE,0xDE,0xDE,0xC0,0x78,0x00}, // 64 '@'
    {0x38,0x6C,0xC6,0xFE,0xC6,0xC6,0xC6,0x00}, // 65 'A'
    {0xFC,0x66,0x66,0x7C,0x66,0x66,0xFC,0x00}, // 66 'B'
    {0x3C,0x66,0xC0,0xC0,0xC0,0x66,0x3C,0x00}, // 67 'C'
    {0xF8,0x6C,0x66,0x66,0x66,0x6C,0xF8,0x00}, // 68 'D'
    {0xFE,0x62,0x68,0x78,0x68,0x62,0xFE,0x00}, // 69 'E'
    {0xFE,0x62,0x68,0x78,0x68,0x60,0xF0,0x00}, // 70 'F'
    {0x3C,0x66,0xC0,0xC0,0xCE,0x66,0x3E,0x00}, // 71 'G'
    {0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0x00}, // 72 'H'
    {0x3C,0x18,0x18,0x18,0x18,0x18,0x3C,0x00}, // 73 'I'
    {0x1E,0x0C,0x0C,0x0C,0xCC,0xCC,0x78,0x00}, // 74 'J'
    {0xE6,0x66,0x6C,0x78,0x6C,0x66,0xE6,0x00}, // 75 'K'
    {0xF0,0x60,0x60,0x60,0x62,0x66,0xFE,0x00}, // 76 'L'
    {0xC6,0xEE,0xFE,0xFE,0xD6,0xC6,0xC6,0x00}, // 77 'M'
    {0xC6,0xE6,0xF6,0xDE,0xCE,0xC6,0xC6,0x00}, // 78 'N'
    {0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00}, // 79 'O'
    {0xFC,0x66,0x66,0x7C,0x60,0x60,0xF0,0x00}, // 80 'P'
    {0x7C,0xC6,0xC6,0xC6,0xD6,0xDE,0x7C,0x06}, // 81 'Q'
    {0xFC,0x66,0x66,0x7C,0x6C,0x66,0xE6,0x00}, // 82 'R'
    {0x7C,0xC6,0x60,0x38,0x0C,0xC6,0x7C,0x00}, // 83 'S'
    {0x7E,0x5A,0x18,0x18,0x18,0x18,0x3C,0x00}, // 84 'T'
    {0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00}, // 85 'U'
    {0xC6,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x00}, // 86 'V'
    {0xC6,0xC6,0xD6,0xFE,0xFE,0xEE,0xC6,0x00}, // 87 'W'
    {0xC6,0xC6,0x6C,0x38,0x6C,0xC6,0xC6,0x00}, // 88 'X'
    {0x66,0x66,0x66,0x3C,0x18,0x18,0x3C,0x00}, // 89 'Y'
    {0xFE,0xC6,0x8C,0x18,0x32,0x66,0xFE,0x00}, // 90 'Z'
    {0x3C,0x30,0x30,0x30,0x30,0x30,0x3C,0x00}, // 91 '['
    {0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00}, // 92 '\'
    {0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00}, // 93 ']'
    {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00}, // 94 '^'
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF}, // 95 '_'
};

static inline void rgb565_be_to_rgb(const uint8_t *p, uint8_t *r, uint8_t *g, uint8_t *b) {
    *r = p[0] & 0xF8;
    *g = ((p[0] & 0x07) << 5) | ((p[1] & 0xE0) >> 3);
    *b = (p[1] & 0x1F) << 3;
}

static inline uint8_t rgb565_be_to_luma(const uint8_t *p) {
    uint8_t r, g, b;
    rgb565_be_to_rgb(p, &r, &g, &b);
    return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

static void run_edge_ai_on_rgb565(const uint8_t *rgb565, uint16_t width, uint16_t height) {
    uint8_t curr_grid[GRID_W * GRID_H];
    uint32_t total_luma = 0;
    uint32_t diff_blocks = 0;
    uint32_t grad_sum = 0;

    int step_x = width / GRID_W;
    int step_y = height / GRID_H;
    if (step_x < 1) step_x = 1;
    if (step_y < 1) step_y = 1;

    for (int gy = 0; gy < GRID_H; gy++) {
        int py = gy * step_y + (step_y / 2);
        if (py >= height) py = height - 1;
        for (int gx = 0; gx < GRID_W; gx++) {
            int px = gx * step_x + (step_x / 2);
            if (px >= width) px = width - 1;
            const uint8_t *p = rgb565 + (py * width + px) * 2;
            uint8_t luma = rgb565_be_to_luma(p);
            curr_grid[gy * GRID_W + gx] = luma;
            total_luma += luma;

            if (gx > 0) {
                int dl = (int)luma - (int)curr_grid[gy * GRID_W + (gx - 1)];
                grad_sum += (dl < 0) ? -dl : dl;
            }
        }
    }

    if (s_grid_initialized) {
        for (int i = 0; i < GRID_W * GRID_H; i++) {
            int diff = (int)curr_grid[i] - (int)s_prev_grid[i];
            if (diff < 0) diff = -diff;
            if (diff > 18) {
                diff_blocks++;
            }
        }
    }
    memcpy(s_prev_grid, curr_grid, sizeof(curr_grid));
    s_grid_initialized = true;

    g_last_edge_ai.mean_luma = (uint8_t)(total_luma / (GRID_W * GRID_H));
    g_last_edge_ai.motion_score = (float)diff_blocks * 100.0f / (float)(GRID_W * GRID_H);
    g_last_edge_ai.sharpness_score = (float)grad_sum / (float)(GRID_W * GRID_H);
    g_last_edge_ai.motion_detected = (g_last_edge_ai.motion_score >= (float)g_cfg.edge_motion_thresh);

    // Compute Edge ROI metrics for Status Indicator LED / Meter window
    int rx0 = (width * g_cfg.roi_x_pct) / 100;
    int ry0 = (height * g_cfg.roi_y_pct) / 100;
    int rw  = (width * g_cfg.roi_w_pct) / 100;
    int rh  = (height * g_cfg.roi_h_pct) / 100;
    if (rx0 + rw > width) rw = width - rx0;
    if (ry0 + rh > height) rh = height - ry0;
    if (rw < 8) rw = 8;
    if (rh < 8) rh = 8;

    uint32_t roi_luma_sum = 0;
    uint32_t roi_samples = 0;
    uint8_t max_luma = 0;
    uint32_t bright_r = 0, bright_g = 0, bright_b = 0, bright_cnt = 0;

    int r_step_x = (rw > 48) ? (rw / 24) : 2;
    int r_step_y = (rh > 48) ? (rh / 24) : 2;

    for (int y = ry0; y < ry0 + rh; y += r_step_y) {
        for (int x = rx0; x < rx0 + rw; x += r_step_x) {
            const uint8_t *p = rgb565 + (y * width + x) * 2;
            uint8_t r, g, b;
            rgb565_be_to_rgb(p, &r, &g, &b);
            uint8_t l = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
            roi_luma_sum += l;
            roi_samples++;
            if (l > max_luma) max_luma = l;
            if (l >= 145 || r >= 170 || g >= 170 || b >= 170) {
                bright_r += r;
                bright_g += g;
                bright_b += b;
                bright_cnt++;
            }
        }
    }

    g_last_edge_ai.roi_luma = roi_samples ? (uint8_t)(roi_luma_sum / roi_samples) : 0;
    g_last_edge_ai.roi_max_luma = max_luma;

    if (bright_cnt >= 2 && max_luma >= 150) {
        uint8_t mr = (uint8_t)(bright_r / bright_cnt);
        uint8_t mg = (uint8_t)(bright_g / bright_cnt);
        uint8_t mb = (uint8_t)(bright_b / bright_cnt);
        g_last_edge_ai.roi_r = mr;
        g_last_edge_ai.roi_g = mg;
        g_last_edge_ai.roi_b = mb;

        if (mr > mg + 40 && mr > mb + 40) {
            snprintf(g_last_edge_ai.edge_led_state, sizeof(g_last_edge_ai.edge_led_state), "ON (DO)");
        } else if (mg > mr + 30 && mg > mb + 30) {
            snprintf(g_last_edge_ai.edge_led_state, sizeof(g_last_edge_ai.edge_led_state), "ON (XANH LA)");
        } else if (mr > 160 && mg > 140 && mb < 110) {
            snprintf(g_last_edge_ai.edge_led_state, sizeof(g_last_edge_ai.edge_led_state), "ON (VANG)");
        } else if (mb > mr + 35 && mb > mg + 20) {
            snprintf(g_last_edge_ai.edge_led_state, sizeof(g_last_edge_ai.edge_led_state), "ON (XANH DUONG)");
        } else {
            snprintf(g_last_edge_ai.edge_led_state, sizeof(g_last_edge_ai.edge_led_state), "ON (SANG)");
        }
    } else {
        g_last_edge_ai.roi_r = 0;
        g_last_edge_ai.roi_g = 0;
        g_last_edge_ai.roi_b = 0;
        snprintf(g_last_edge_ai.edge_led_state, sizeof(g_last_edge_ai.edge_led_state), "OFF");
    }

    // On-Device Edge AI 7-Segment & LCD Digit Segmentation inside ROI
    uint8_t mid_thresh = (uint8_t)((g_last_edge_ai.roi_luma + max_luma) / 2);
    if (mid_thresh < 60) mid_thresh = 60;
    bool invert_lcd = (g_cfg.meter_mode == 3); // Mode 3 = dark LCD digits on lighter background
    if (invert_lcd) mid_thresh = (g_last_edge_ai.roi_luma > 30) ? (g_last_edge_ai.roi_luma - 18) : 90;

    char decoded_str[12] = {0};
    int d_idx = 0;
    int in_char = 0, cx0 = rx0, cy_min = ry0 + rh, cy_max = ry0;

    for (int x = rx0; x < rx0 + rw && d_idx < 8; x += 2) {
        int col_hits = 0;
        for (int y = ry0 + 4; y < ry0 + rh - 4; y += 2) {
            const uint8_t *p = rgb565 + (y * width + x) * 2;
            uint8_t l = rgb565_be_to_luma(p);
            bool fg = invert_lcd ? (l < mid_thresh) : (l > mid_thresh);
            if (fg) {
                col_hits++;
                if (y < cy_min) cy_min = y;
                if (y > cy_max) cy_max = y;
            }
        }
        if (col_hits >= 3) {
            if (!in_char) {
                in_char = 1;
                cx0 = x;
                cy_min = ry0 + rh;
                cy_max = ry0;
            }
        } else if (in_char) {
            in_char = 0;
            int cw = x - cx0;
            int ch = cy_max - cy_min;
            if (cw >= 4 && ch >= (rh / 4) && cw <= ch + 6) {
                char c = decode_edge_7seg_box(rgb565, width, cx0, cy_min, cw, ch, mid_thresh, invert_lcd);
                if (c) decoded_str[d_idx++] = c;
            }
        }
    }
    if (d_idx > 0 && g_cfg.meter_mode != 4) {
        decoded_str[d_idx] = '\0';
        snprintf(g_last_edge_ai.edge_digits, sizeof(g_last_edge_ai.edge_digits), "%s", decoded_str);
    } else if (g_cfg.meter_mode == 4 || g_cfg.meter_mode == 0) {
        // On-Device Analog Water Meter Red-Pointer Sub-Dial Angle Decoder (4 Sub-Dials x0.1 .. x0.0001)
        char dial_digits[8] = {0};
        int found_dials = 0;
        int sub_w = rw / 4;
        if (sub_w >= 12) {
            for (int k = 0; k < 4; k++) {
                int sx0 = rx0 + k * sub_w;
                int sx1 = sx0 + sub_w;
                int cx = (sx0 + sx1) / 2;
                int cy = ry0 + (rh / 2);
                int max_dist2 = 0;
                int tip_dx = 0, tip_dy = 0;
                int red_pts = 0;
                for (int y = ry0 + 2; y < ry0 + rh - 2; y += 2) {
                    for (int x = sx0 + 2; x < sx1 - 2; x += 2) {
                        const uint8_t *p = rgb565 + (y * width + x) * 2;
                        uint8_t r, g, b;
                        rgb565_be_to_rgb(p, &r, &g, &b);
                        if (r > 95 && r > g + 32 && r > b + 32) {
                            red_pts++;
                            int dx = x - cx;
                            int dy = y - cy;
                            int d2 = dx * dx + dy * dy;
                            if (d2 > max_dist2) {
                                max_dist2 = d2;
                                tip_dx = dx;
                                tip_dy = dy;
                            }
                        }
                    }
                }
                if (red_pts >= 3 && max_dist2 >= 9) {
                    float ang = atan2f((float)tip_dx, (float)(-tip_dy)) * (180.0f / 3.14159265f);
                    ang -= (float)g_cfg.meter_rot_deg;
                    while (ang < 0.0f) ang += 360.0f;
                    while (ang >= 360.0f) ang -= 360.0f;
                    int dig = ((int)(ang / 36.0f)) % 10;
                    dial_digits[k] = (char)('0' + dig);
                    found_dials++;
                } else {
                    dial_digits[k] = '-';
                }
            }
        }
        if (found_dials >= 2) {
            dial_digits[4] = '\0';
            if (g_cfg.meter_baseline[0] != '\0') {
                int int_part = atoi(g_cfg.meter_baseline);
                snprintf(g_last_edge_ai.edge_digits, sizeof(g_last_edge_ai.edge_digits), "%05d.%s", int_part, dial_digits);
            } else {
                snprintf(g_last_edge_ai.edge_digits, sizeof(g_last_edge_ai.edge_digits), "0.%s", dial_digits);
            }
        } else if (d_idx > 0) {
            decoded_str[d_idx] = '\0';
            snprintf(g_last_edge_ai.edge_digits, sizeof(g_last_edge_ai.edge_digits), "%s", decoded_str);
        } else if (g_cfg.meter_baseline[0] != '\0' && g_cfg.meter_mode == 4) {
            snprintf(g_last_edge_ai.edge_digits, sizeof(g_last_edge_ai.edge_digits), "%.15s", g_cfg.meter_baseline);
        } else {
            snprintf(g_last_edge_ai.edge_digits, sizeof(g_last_edge_ai.edge_digits), "----");
        }
    } else {
        snprintf(g_last_edge_ai.edge_digits, sizeof(g_last_edge_ai.edge_digits), "----");
    }
}

static inline void put_pixel_rgb565_be(uint8_t *rgb565, uint16_t width, uint16_t height, int x, int y, uint16_t color_be) {
    if (x < 0 || x >= width || y < 0 || y >= height) return;
    uint8_t *p = rgb565 + (y * width + x) * 2;
    p[0] = (uint8_t)(color_be >> 8);
    p[1] = (uint8_t)(color_be & 0xFF);
}

static void draw_roi_target_box(uint8_t *rgb565, uint16_t width, uint16_t height) {
    int rx0 = (width * g_cfg.roi_x_pct) / 100;
    int ry0 = (height * g_cfg.roi_y_pct) / 100;
    int rw  = (width * g_cfg.roi_w_pct) / 100;
    int rh  = (height * g_cfg.roi_h_pct) / 100;
    int rx1 = rx0 + rw - 1;
    int ry1 = ry0 + rh - 1;
    if (rx1 >= width) rx1 = width - 1;
    if (ry1 >= height - 28) ry1 = height - 28;
    if (rx1 <= rx0 + 10 || ry1 <= ry0 + 10) return;

    // 0x07E0 = Bright Green in RGB565
    uint16_t col = 0x07E0;
    // Dashed rectangle outline
    for (int x = rx0; x <= rx1; x++) {
        if ((x % 8) < 5) {
            put_pixel_rgb565_be(rgb565, width, height, x, ry0, col);
            put_pixel_rgb565_be(rgb565, width, height, x, ry1, col);
            put_pixel_rgb565_be(rgb565, width, height, x, ry0 + 1, col);
            put_pixel_rgb565_be(rgb565, width, height, x, ry1 - 1, col);
        }
    }
    for (int y = ry0; y <= ry1; y++) {
        if ((y % 8) < 5) {
            put_pixel_rgb565_be(rgb565, width, height, rx0, y, col);
            put_pixel_rgb565_be(rgb565, width, height, rx1, y, col);
            put_pixel_rgb565_be(rgb565, width, height, rx0 + 1, y, col);
            put_pixel_rgb565_be(rgb565, width, height, rx1 - 1, y, col);
        }
    }
}

static void draw_char_2x(uint8_t *rgb565, uint16_t width, uint16_t height, int x0, int y0, char c, uint16_t color_be) {
    if (c >= 'a' && c <= 'z') {
        c = c - 'a' + 'A';
    }
    if (c < 32 || c > 95) {
        c = ' ';
    }
    const uint8_t *glyph = font8x8_basic[c - 32];
    uint8_t hi = (uint8_t)(color_be >> 8);
    uint8_t lo = (uint8_t)(color_be & 0xFF);

    for (int row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        for (int col = 0; col < 8; col++) {
            if (bits & (0x80 >> col)) {
                for (int dy = 0; dy < 2; dy++) {
                    int py = y0 + row * 2 + dy;
                    if (py < 0 || py >= height) continue;
                    for (int dx = 0; dx < 2; dx++) {
                        int px = x0 + col * 2 + dx;
                        if (px < 0 || px >= width) continue;
                        uint8_t *p = rgb565 + (py * width + px) * 2;
                        p[0] = hi;
                        p[1] = lo;
                    }
                }
            }
        }
    }
}

static void draw_osd_banner(uint8_t *rgb565, uint16_t width, uint16_t height, const char *text) {
    int banner_h = 26;
    if (height < banner_h) return;
    int y_start = height - banner_h;

    // Darken bottom strip for contrast (divide RGB by 4)
    for (int y = y_start; y < height; y++) {
        uint8_t *row = rgb565 + (y * width) * 2;
        for (int x = 0; x < width; x++) {
            uint16_t v = ((uint16_t)row[x * 2] << 8) | row[x * 2 + 1];
            uint16_t r = ((v >> 11) & 0x1F) >> 2;
            uint16_t g = ((v >> 5) & 0x3F) >> 2;
            uint16_t b = (v & 0x1F) >> 2;
            uint16_t dark = (r << 11) | (g << 5) | b;
            row[x * 2] = (uint8_t)(dark >> 8);
            row[x * 2 + 1] = (uint8_t)(dark & 0xFF);
        }
    }

    // Draw yellow/cyan/white text (0xFFE0 = yellow in RGB565, 0xFFFF = white)
    int x = 10;
    int y = y_start + 5;
    for (size_t i = 0; text[i] != '\0'; i++) {
        if (x + 16 > width - 4) break;
        uint16_t col = (i < 19) ? 0xFFE0 : 0xFFFF;
        draw_char_2x(rgb565, width, height, x, y, text[i], col);
        x += 16;
    }
}

bool process_frame_osd_and_ai(
    const uint8_t *src_jpg, size_t src_len,
    uint16_t width, uint16_t height,
    uint8_t quality, const char *extra_tag,
    uint8_t **out_jpg, size_t *out_len, bool *out_allocated
) {
    *out_jpg = (uint8_t *)src_jpg;
    *out_len = src_len;
    *out_allocated = false;

    if (!g_cfg.osd_enable && !g_cfg.edge_ai_enable) {
        return true;
    }

    size_t rgb_size = (size_t)width * (size_t)height * 2;
    uint8_t *rgb_buf = (uint8_t *)heap_caps_malloc(rgb_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!rgb_buf) {
        ESP_LOGW(TAG, "Failed to allocate %u bytes in PSRAM for OSD/AI", (unsigned)rgb_size);
        return false;
    }

    if (!jpg2rgb565(src_jpg, src_len, rgb_buf, JPG_SCALE_NONE)) {
        ESP_LOGW(TAG, "jpg2rgb565 decode failed");
        free(rgb_buf);
        return false;
    }

    // jpg2rgb565 outputs Little-Endian RGB565 (uint16_t on Xtensa), whereas
    // fmt2jpg(PIXFORMAT_RGB565) and OSD/EdgeAI helpers expect Big-Endian Camera RGB565.
    // Fast 32-bit word byte-swap (~2ms for 640x480) restores 100% accurate colors!
    uint32_t *w32 = (uint32_t *)rgb_buf;
    size_t num_words = rgb_size / 4;
    for (size_t i = 0; i < num_words; i++) {
        uint32_t v = w32[i];
        w32[i] = ((v & 0x00FF00FFUL) << 8) | ((v & 0xFF00FF00UL) >> 8);
    }

    if (g_cfg.edge_ai_enable) {
        run_edge_ai_on_rgb565(rgb_buf, width, height);
    }

    if (g_cfg.osd_enable) {
        if (g_cfg.roi_draw_osd) {
            draw_roi_target_box(rgb_buf, width, height);
        }

        time_t now;
        struct tm timeinfo;
        time(&now);
        localtime_r(&now, &timeinfo);

        char time_str[32];
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo);

        char banner_text[96];
        if (extra_tag && extra_tag[0]) {
            snprintf(banner_text, sizeof(banner_text), "%s  %s  %s",
                     time_str, g_cfg.mdns_host, extra_tag);
        } else if (g_cfg.edge_ai_enable) {
            snprintf(banner_text, sizeof(banner_text), "%s  %s  LED:%s",
                     time_str, g_cfg.mdns_host, g_last_edge_ai.edge_led_state);
        } else {
            snprintf(banner_text, sizeof(banner_text), "%s  %s",
                     time_str, g_cfg.mdns_host);
        }

        draw_osd_banner(rgb_buf, width, height, banner_text);

        uint8_t *encoded_jpg = NULL;
        size_t encoded_len = 0;
        uint8_t q = (quality > 78) ? 78 : quality;
        if (fmt2jpg(rgb_buf, rgb_size, width, height, PIXFORMAT_RGB565, q, &encoded_jpg, &encoded_len) &&
            encoded_len >= 4 && encoded_len < 130000 &&
            encoded_jpg[encoded_len - 2] == 0xFF && encoded_jpg[encoded_len - 1] == 0xD9) {
            *out_jpg = encoded_jpg;
            *out_len = encoded_len;
            *out_allocated = true;
        } else {
            if (encoded_jpg) free(encoded_jpg);
            encoded_jpg = NULL;
            encoded_len = 0;
            if (fmt2jpg(rgb_buf, rgb_size, width, height, PIXFORMAT_RGB565, 65, &encoded_jpg, &encoded_len) &&
                encoded_len >= 4 && encoded_jpg[encoded_len - 2] == 0xFF && encoded_jpg[encoded_len - 1] == 0xD9) {
                *out_jpg = encoded_jpg;
                *out_len = encoded_len;
                *out_allocated = true;
            } else {
                if (encoded_jpg) free(encoded_jpg);
                ESP_LOGW(TAG, "fmt2jpg re-encode failed or overflowed, falling back to raw JPEG");
            }
        }
    }

    free(rgb_buf);
    return true;
}
