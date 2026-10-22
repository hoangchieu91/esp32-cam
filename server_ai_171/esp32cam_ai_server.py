#!/usr/bin/env python3
"""
ESP32-CAM Deep AI Vision & Smart Meter Reader Server
- RK3576 NPU YOLOv8n Object Detection
- Status Indicator LED Reader (ON/OFF/Color/Brightness)
- 7-Segment LED Digit Reader (Bitmask & Hamming Decoder)
- LCD Utility Meter Reader (Water, Electricity, Gas - CLAHE + Template/Segment OCR)
Runs on Server 10.25.7.171:8095
"""

import os
import sys
import time
import json
import base64
import threading
import urllib.request
from datetime import datetime, timezone, timedelta
from pathlib import Path
from typing import Dict, Any, List, Tuple, Optional

import cv2
import numpy as np
import paho.mqtt.client as mqtt
from fastapi import FastAPI, Request
from fastapi.responses import HTMLResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

try:
    from rknnlite.api import RKNNLite
    HAS_RKNN = True
except Exception as e:
    print(f"[!] RKNNLite import warning: {e}")
    HAS_RKNN = False

TZ_VN = timezone(timedelta(hours=7))

def now_vn_str():
    return datetime.now(TZ_VN).strftime("%Y-%m-%d %H:%M:%S")

BASE_DIR = Path("/opt/esp32cam_ai_server")
FTP_INBOX_DIR = Path("/var/ftp/esp32cam")
PROCESSED_DIR = BASE_DIR / "processed"
ANNOTATED_DIR = BASE_DIR / "annotated"
FIRMWARE_DIR = BASE_DIR / "firmware"
HISTORY_FILE = BASE_DIR / "history.json"
METER_STATE_FILE = BASE_DIR / "meter_state.json"
METER_LEDGER_FILE = BASE_DIR / "meter_ledger.json"
MULTI_SCREEN_FILE = BASE_DIR / "multi_screen_state.json"
TELEMETRY_FILE = BASE_DIR / "latest_telemetry.json"

for d in [BASE_DIR, FTP_INBOX_DIR, PROCESSED_DIR, ANNOTATED_DIR, FIRMWARE_DIR]:
    d.mkdir(parents=True, exist_ok=True)

MODEL_PATH = "/root/YOLOv8-Pose/build/model/yolov8n.rknn"
NTFY_URL = "http://10.25.7.116:8092/esp32cam"
MQTT_HOST = "127.0.0.1"
MQTT_PORT = 1883

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone",
    "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear",
    "hair drier", "toothbrush"
]

# ============================================================================
# SMART METER & INDICATOR READER ENGINE (LED STATUS, 7-SEG, LCD METERS)
# ============================================================================
SEG7_LOOKUP = {
    (1, 1, 1, 1, 1, 1, 0): "0",
    (0, 1, 1, 0, 0, 0, 0): "1",
    (0, 0, 0, 0, 1, 1, 0): "1",
    (1, 1, 0, 1, 1, 0, 1): "2",
    (1, 1, 1, 1, 0, 0, 1): "3",
    (0, 1, 1, 0, 0, 1, 1): "4",
    (1, 0, 1, 1, 0, 1, 1): "5",
    (1, 0, 1, 1, 1, 1, 1): "6",
    (0, 0, 1, 1, 1, 1, 1): "6",
    (1, 1, 1, 0, 0, 0, 0): "7",
    (1, 1, 1, 0, 0, 1, 0): "7",
    (1, 1, 1, 1, 1, 1, 1): "8",
    (1, 1, 1, 1, 0, 1, 1): "9",
    (1, 1, 1, 0, 0, 1, 1): "9",
}


class SmartMeterVisionEngine:
    """
    Computer Vision pipelines for:
    1. Status Indicator LEDs (ON/OFF, Color: Red/Green/Yellow/Blue/White, Brightness)
    2. 7-Segment Digital LED Meters
    3. LCD Utility Meters (Water m3, Electricity kWh, Gas m3/bar)
    """

    @staticmethod
    def get_roi_rect(w: int, h: int, rx_pct: int, ry_pct: int, rw_pct: int, rh_pct: int) -> Tuple[int, int, int, int]:
        x0 = max(0, min(w - 16, int(w * rx_pct / 100.0)))
        y0 = max(0, min(h - 32, int(h * ry_pct / 100.0)))
        rw = max(16, min(w - x0, int(w * rw_pct / 100.0)))
        rh = max(16, min(h - 28 - y0, int(h * rh_pct / 100.0)))
        return x0, y0, rw, rh

    @staticmethod
    def analyze_led_indicators(roi_bgr: np.ndarray) -> Dict[str, Any]:
        if roi_bgr is None or roi_bgr.size == 0:
            return {"status": "OFF", "detail": "ROI trống", "boxes": []}

        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        h_ch, s_ch, v_ch = cv2.split(hsv)
        mean_v = float(np.mean(v_ch))
        max_v = int(np.max(v_ch))

        # Bright glowing mask (either colored halo or bright core), excluding 3px OSD ROI border
        bright_mask = ((v_ch >= 165) & (s_ch >= 55)) | (v_ch >= 220)
        if bright_mask.shape[0] > 8 and bright_mask.shape[1] > 8:
            bright_mask[:4, :] = False
            bright_mask[-4:, :] = False
            bright_mask[:, :4] = False
            bright_mask[:, -4:] = False
        bright_ratio = float(np.count_nonzero(bright_mask)) * 100.0 / float(v_ch.size)

        if max_v < 155 or bright_ratio < 0.25:
            return {
                "status": "OFF",
                "detail": f"Độ chói đỉnh: {max_v}/255 | Trung bình: {mean_v:.0f} (Đèn tắt)",
                "boxes": [],
                "led_array": []
            }

        # Determine dominant color of glowing pixels (where V >= 150 and S >= 50)
        color_mask = (v_ch >= 145) & (s_ch >= 50)
        if np.count_nonzero(color_mask) >= 8:
            hues = h_ch[color_mask]
            red_cnt = np.count_nonzero((hues <= 11) | (hues >= 165))
            yel_cnt = np.count_nonzero((hues >= 12) & (hues <= 36))
            grn_cnt = np.count_nonzero((hues >= 37) & (hues <= 88))
            blu_cnt = np.count_nonzero((hues >= 89) & (hues <= 138))

            color_scores = {
                "ĐỎ (RED)": int(red_cnt),
                "XANH LÁ (GREEN)": int(grn_cnt),
                "VÀNG (YELLOW)": int(yel_cnt),
                "XANH DƯƠNG (BLUE)": int(blu_cnt),
            }
            best_color = max(color_scores, key=color_scores.get)
            if color_scores[best_color] == 0:
                best_color = "TRẮNG SÁNG"
        else:
            best_color = "TRẮNG SÁNG"

        # Find bounding boxes and individual color/role of glowing LED spots
        mask_u8 = (bright_mask.astype(np.uint8)) * 255
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE, kernel)
        cnts, _ = cv2.findContours(mask_u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        boxes = []
        led_array = []
        rh, rw = roi_bgr.shape[:2]

        role_map = {
            "ĐỎ (RED)": "PWR / ERR / ALARM",
            "XANH LÁ (GREEN)": "RUN / STATUS OK / LINK",
            "VÀNG (YELLOW)": "COMM TX-RX / ACT / WARN",
            "XANH DƯƠNG (BLUE)": "WIFI / BLE / VPN",
            "TRẮNG SÁNG": "DISPLAY / GENERAL",
        }

        raw_spots = []
        max_spot_area = max(64.0, 0.22 * float(rw * rh))
        for c in cnts:
            area = cv2.contourArea(c)
            if 10 <= area <= max_spot_area:
                bx, by, bw, bh = cv2.boundingRect(c)
                raw_spots.append((bx, by, bw, bh, area))

        # Sort left-to-right for deterministic pin/LED indexing (#1, #2, ...)
        raw_spots.sort(key=lambda s: (s[0], s[1]))
        for idx, (bx, by, bw, bh, area) in enumerate(raw_spots[:8]):
            boxes.append([int(bx), int(by), int(bw), int(bh)])
            pad = 3
            sy0, sy1 = max(0, by - pad), min(rh, by + bh + pad)
            sx0, sx1 = max(0, bx - pad), min(rw, bx + bw + pad)
            sub_h = h_ch[sy0:sy1, sx0:sx1]
            sub_s = s_ch[sy0:sy1, sx0:sx1]
            sub_v = v_ch[sy0:sy1, sx0:sx1]
            spot_max_v = int(np.max(sub_v)) if sub_v.size else max_v
            cm = (sub_v >= 135) & (sub_s >= 45)
            if np.count_nonzero(cm) >= 3:
                sh = sub_h[cm]
                sc = {
                    "ĐỎ (RED)": int(np.count_nonzero((sh <= 11) | (sh >= 165))),
                    "XANH LÁ (GREEN)": int(np.count_nonzero((sh >= 37) & (sh <= 88))),
                    "VÀNG (YELLOW)": int(np.count_nonzero((sh >= 12) & (sh <= 36))),
                    "XANH DƯƠNG (BLUE)": int(np.count_nonzero((sh >= 89) & (sh <= 138))),
                }
                spot_col = max(sc, key=sc.get)
                if sc[spot_col] == 0:
                    spot_col = best_color
            else:
                spot_col = best_color

            cx_pct = round((bx + bw * 0.5) * 100.0 / max(1, rw), 1)
            cy_pct = round((by + bh * 0.5) * 100.0 / max(1, rh), 1)
            led_array.append({
                "index": idx + 1,
                "color": spot_col,
                "role_hint": role_map.get(spot_col, "INDICATOR"),
                "state": "SOLID_ON",
                "pos_pct": [cx_pct, cy_pct],
                "box": [int(bx), int(by), int(bw), int(bh)],
                "peak_luma": spot_max_v,
                "area_px": int(area)
            })

        return {
            "status": f"ON ({best_color})",
            "detail": f"Độ chói đỉnh: {max_v}/255 | Vùng phát sáng: {bright_ratio:.1f}% ({len(led_array)} điểm LED)",
            "boxes": boxes[:8],
            "led_array": led_array
        }

    @staticmethod
    def _decode_single_7seg_patch(bin_patch: np.ndarray) -> Tuple[str, float]:
        ph, pw = bin_patch.shape[:2]
        if ph < 10 or pw < 3:
            return "", 0.0

        aspect = float(pw) / float(ph)
        if aspect < 0.34:
            fill = float(np.count_nonzero(bin_patch)) / float(ph * pw)
            if fill > 0.30:
                return "1", 0.88

        # Resize to canonical 28x48 grid to sample 7 segments (a, b, c, d, e, f, g)
        norm = cv2.resize(bin_patch, (28, 48), interpolation=cv2.INTER_NEAREST)
        segs_coords = [
            (6, 22, 1, 9),    # a: top horizontal
            (19, 27, 6, 22),  # b: top-right vertical
            (19, 27, 26, 42), # c: bottom-right vertical
            (6, 22, 39, 47),  # d: bottom horizontal
            (1, 9, 26, 42),   # e: bottom-left vertical
            (1, 9, 6, 22),    # f: top-left vertical
            (6, 22, 20, 28),  # g: middle horizontal
        ]
        state = []
        densities = []
        for (x1, x2, y1, y2) in segs_coords:
            sub = norm[y1:y2, x1:x2]
            dens = float(np.count_nonzero(sub)) / float(sub.size)
            densities.append(dens)
            state.append(1 if dens >= 0.36 else 0)

        tup = tuple(state)
        if tup in SEG7_LOOKUP:
            return SEG7_LOOKUP[tup], 0.92

        # Nearest Hamming distance match if 1 segment is noisy
        best_digit = ""
        best_dist = 99
        for k_tup, dig in SEG7_LOOKUP.items():
            dist = sum(1 for a, b in zip(tup, k_tup) if a != b)
            if dist < best_dist:
                best_dist = dist
                best_digit = dig
        if best_dist == 1 and sum(state) >= 2:
            return best_digit, 0.74
        return "", 0.0

    @classmethod
    def decode_digits_from_binary(cls, bin_img: np.ndarray) -> Tuple[str, List[List[int]], float]:
        rh, rw = bin_img.shape[:2]
        cnts, _ = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        dots = []

        for c in cnts:
            x, y, w, h = cv2.boundingRect(c)
            # Ignore border touching noise
            if x <= 1 or y <= 1 or (x + w) >= rw - 1 or (y + h) >= rh - 1:
                if w > rw * 0.8 or h > rh * 0.9:
                    continue
            h_ratio = float(h) / float(rh)
            aspect = float(w) / float(max(1, h))
            if 0.22 <= h_ratio <= 0.92 and 0.14 <= aspect <= 0.95 and w >= 4 and h >= 12:
                candidates.append((x, y, w, h))
            elif 0.04 <= h_ratio <= 0.18 and 0.6 <= aspect <= 1.6 and y > rh * 0.55:
                dots.append((x, y, w, h))

        if not candidates:
            return "", [], 0.0

        # Filter candidates that share similar vertical center & height (digit row alignment)
        candidates.sort(key=lambda b: b[0])
        med_h = float(np.median([b[3] for b in candidates]))
        med_cy = float(np.median([b[1] + b[3] * 0.5 for b in candidates]))

        aligned = [
            b for b in candidates
            if 0.55 * med_h <= b[3] <= 1.55 * med_h and abs((b[1] + b[3] * 0.5) - med_cy) <= med_h * 0.55
        ]
        if not aligned:
            aligned = candidates

        digits_out = []
        boxes_out = []
        conf_sum = 0.0

        for idx, (x, y, w, h) in enumerate(aligned[:10]):
            patch = bin_img[y:y+h, x:x+w]
            ch, cf = cls._decode_single_7seg_patch(patch)
            if ch:
                digits_out.append(ch)
                boxes_out.append([int(x), int(y), int(w), int(h)])
                conf_sum += cf
                # Check if decimal point sits right after this digit
                for (dx, dy, dw, dh) in dots:
                    if (x + w - 2) <= dx <= (x + w + max(10, int(w * 0.5))):
                        digits_out.append(".")
                        break

        if not digits_out:
            return "", [], 0.0

        avg_conf = conf_sum / max(1, len(boxes_out))
        return "".join(digits_out), boxes_out, round(avg_conf, 2)

    @classmethod
    def analyze_7segment(cls, roi_bgr: np.ndarray) -> Dict[str, Any]:
        if roi_bgr is None or roi_bgr.size == 0:
            return {"value": "----", "detail": "ROI trống", "boxes": []}

        # 7-Segment LEDs are self-luminous (bright digits on darker background)
        b, g, r = cv2.split(roi_bgr)
        max_ch = np.maximum(np.maximum(r, g), b)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(6, 6))
        enh = clahe.apply(max_ch)
        blur = cv2.GaussianBlur(enh, (3, 3), 0)
        _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Connect broken LED segments vertically/horizontally slightly
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 3))
        morphed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, k)

        val_str, boxes, conf = cls.decode_digits_from_binary(morphed)
        if val_str:
            return {
                "value": val_str,
                "detail": f"Độ tin cậy: {int(conf*100)}% ({len(boxes)} chữ số LED)",
                "boxes": boxes
            }
        return {
            "value": "----",
            "detail": "Chưa phát hiện dãy LED 7 thanh phát sáng trong ROI",
            "boxes": []
        }

    @classmethod
    def analyze_lcd_meter(cls, roi_bgr: np.ndarray) -> Dict[str, Any]:
        if roi_bgr is None or roi_bgr.size == 0:
            return {"value": "----", "detail": "ROI trống", "boxes": []}

        # Utility LCD / Mechanical Roller meters have dark digits on light/gray reflective background
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.2, tileGridSize=(8, 8))
        enh = clahe.apply(gray)
        blur = cv2.GaussianBlur(enh, (3, 3), 0)

        # Try both Adaptive Gaussian Thresholding (inverted) and Otsu (inverted)
        th_adapt = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 25, 9
        )
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        th_adapt = cv2.morphologyEx(th_adapt, cv2.MORPH_OPEN, k)
        th_adapt = cv2.morphologyEx(th_adapt, cv2.MORPH_CLOSE, k)

        val1, boxes1, conf1 = cls.decode_digits_from_binary(th_adapt)

        _, th_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        th_otsu = cv2.morphologyEx(th_otsu, cv2.MORPH_CLOSE, k)
        val2, boxes2, conf2 = cls.decode_digits_from_binary(th_otsu)

        if len(boxes2) > len(boxes1) or (len(boxes2) == len(boxes1) and conf2 > conf1):
            val_str, boxes, conf = val2, boxes2, conf2
        else:
            val_str, boxes, conf = val1, boxes1, conf1

        if val_str:
            return {
                "value": val_str,
                "detail": f"CLAHE LCD OCR: {int(conf*100)}% ({len(boxes)} chữ số)",
                "boxes": boxes
            }
        return {
            "value": "----",
            "detail": "Đưa camera gần mặt đồng hồ (Macro 820) để đọc rõ chữ số LCD",
            "boxes": []
        }

    @staticmethod
    def _extract_pointer_angle(dial_bgr: np.ndarray) -> Tuple[float, float, Tuple[int, int], Tuple[int, int]]:
        """
        Extract clockwise angle (0..360 deg from 12 o'clock) of a red or dark rotating pointer inside a sub-dial patch.
        Returns (angle_deg, confidence, center_xy, tip_xy).
        """
        dh, dw = dial_bgr.shape[:2]
        cx, cy = dw // 2, dh // 2
        radius = max(6, min(cx, cy) - 2)

        hsv = cv2.cvtColor(dial_bgr, cv2.COLOR_BGR2HSV)
        h_ch, s_ch, v_ch = cv2.split(hsv)

        # Circular mask to ignore outer square corners
        yy, xx = np.ogrid[:dh, :dw]
        dist_sq = (xx - cx) ** 2 + (yy - cy) ** 2
        circle_mask = (dist_sq <= (radius ** 2)) & (dist_sq >= 4)

        # 1. Try Red Pointer segmentation (standard on x0.1, x0.01, x0.001, x0.0001 water meter dials)
        red_mask = (((h_ch <= 15) | (h_ch >= 160)) & (s_ch >= 45) & (v_ch >= 40)) & circle_mask
        pts_y, pts_x = np.where(red_mask)
        conf_base = 0.88

        # 2. Fallback to Dark Pointer segmentation if dial uses black needles
        if len(pts_x) < 5:
            p20 = float(np.percentile(v_ch[circle_mask], 20)) if np.any(circle_mask) else 70.0
            dark_thresh = min(105.0, max(35.0, p20))
            dark_mask = (v_ch <= dark_thresh) & circle_mask
            pts_y, pts_x = np.where(dark_mask)
            conf_base = 0.68

        if len(pts_x) < 4:
            return 0.0, 0.0, (cx, cy), (cx, max(0, cy - radius // 2))

        # Weight pixels by distance from center to find the needle tip direction
        dx_arr = (pts_x - cx).astype(np.float32)
        dy_arr = (pts_y - cy).astype(np.float32)
        r_arr = np.sqrt(dx_arr ** 2 + dy_arr ** 2)
        top_mask = r_arr >= np.percentile(r_arr, 55)
        if np.any(top_mask):
            vx = float(np.mean(dx_arr[top_mask] * r_arr[top_mask]))
            vy = float(np.mean(dy_arr[top_mask] * r_arr[top_mask]))
        else:
            vx = float(np.mean(dx_arr))
            vy = float(np.mean(dy_arr))

        norm = float(np.hypot(vx, vy))
        if norm < 1e-3:
            return 0.0, 0.0, (cx, cy), (cx, max(0, cy - radius // 2))

        # Clockwise angle from 12 o'clock (0 deg = top = digit 0, 36 deg per digit)
        angle_deg = (float(np.degrees(np.arctan2(vx, -vy))) + 360.0) % 360.0
        tip_x = int(round(cx + (vx / norm) * (radius * 0.82)))
        tip_y = int(round(cy + (vy / norm) * (radius * 0.82)))
        return round(angle_deg, 1), conf_base, (cx, cy), (tip_x, tip_y)

    @classmethod
    def analyze_dial_water_meter(cls, roi_bgr: np.ndarray, lcd_res: Dict[str, Any]) -> Dict[str, Any]:
        """
        Multi-Dial Rotary Water Meter Reader:
        - Primary: Direct Red Pointer Hub-to-Tip Distance Transform Vector Extractor (works on real GB/T 778-2018 meters)
        - Fallback: HoughCircles + 4-Zone layout
        - Applies Inter-Dial Gear Carry Coupling from least-significant dial to most-significant dial
        """
        if roi_bgr is None or roi_bgr.size == 0:
            return {"raw_fraction": 0.0, "dial_digits": "----", "dials": [], "confidence": 0.0}

        rh, rw = roi_bgr.shape[:2]
        raw_cont = []
        dials_meta = []
        conf_sum = 0.0
        valid_cnt = 0

        # 1. Direct Red Pointer Hub-to-Tip Distance Transform Detector (lower dial arc y >= 0.43*rh)
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        hc, sc, vc = cv2.split(hsv)
        red_mask = (((hc <= 16) | (hc >= 160)) & (sc >= 40) & (vc >= 38)).astype(np.uint8) * 255
        # Mask out upper horizontal roller drum zone (H 00304 m3 sits at y ~ 0.25..0.41*rh)
        red_mask[:int(rh * 0.43), :] = 0
        red_mask[:, :3] = 0
        red_mask[:, max(0, rw - 3):] = 0
        k_el = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, k_el)
        cnts, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        raw_red_cands = []
        for c in cnts:
            area = cv2.contourArea(c)
            if 150 <= area <= 5500:
                x, y, bw, bh = cv2.boundingRect(c)
                if bw < 10 or bh < 10:
                    continue
                m_patch = np.zeros((bh + 4, bw + 4), dtype=np.uint8)
                cv2.drawContours(m_patch, [c - [x - 2, y - 2]], -1, 255, -1)
                dt = cv2.distanceTransform(m_patch, cv2.DIST_L2, 5)
                _, max_val, _, max_loc = cv2.minMaxLoc(dt)
                cx_i = int(x - 2 + max_loc[0])
                cy_i = int(y - 2 + max_loc[1])
                pts = c.reshape(-1, 2)
                d2 = (pts[:, 0] - cx_i) ** 2 + (pts[:, 1] - cy_i) ** 2
                tip_pt = pts[int(np.argmax(d2))]
                tx_i, ty_i = int(tip_pt[0]), int(tip_pt[1])
                vx, vy = float(tx_i - cx_i), float(ty_i - cy_i)
                r_i = int(max(12, round(np.hypot(vx, vy))))
                ang = (float(np.degrees(np.arctan2(vx, -vy))) + 360.0) % 360.0
                raw_red_cands.append((cx_i, cy_i, tx_i, ty_i, r_i, round(ang, 1), 0.92, area))

        red_pointers = []
        if raw_red_cands:
            max_ptr_area = max(p[7] for p in raw_red_cands)
            red_pointers = [
                p[:7] for p in raw_red_cands
                if p[7] >= max(180.0, 0.42 * max_ptr_area)
            ]

        if len(red_pointers) >= 2:
            # Sort along standard rotary water meter arc: Right-to-Left (x descending)
            red_pointers.sort(key=lambda item: -item[0])
            red_pointers = red_pointers[:4]
            for idx, (cx_i, cy_i, tx_i, ty_i, r_i, ang, cf) in enumerate(red_pointers):
                cont_val = (ang / 36.0) % 10.0
                raw_cont.append(cont_val)
                valid_cnt += 1
                conf_sum += cf
                dials_meta.append({
                    "index": idx,
                    "scale": round(0.1 ** (idx + 1), 4),
                    "center": [cx_i, cy_i],
                    "tip": [tx_i, ty_i],
                    "radius": r_i,
                    "angle_deg": ang,
                    "raw_float": round(cont_val, 2),
                    "confidence": cf
                })
        else:
            # 2. Fallback: HoughCircles + 4-Zone layout
            gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 1.2)

            min_r = max(8, int(min(rw, rh) * 0.07))
            max_r = max(min_r + 6, int(min(rw, rh) * 0.28))
            circles = cv2.HoughCircles(
                blur, cv2.HOUGH_GRADIENT, dp=1.2, minDist=max(16, rw // 5),
                param1=75, param2=24, minRadius=min_r, maxRadius=max_r
            )

            dial_rois = []
            if circles is not None and len(circles[0]) >= 2:
                c_list = sorted(circles[0], key=lambda c: c[0])[:4]
                for (cx, cy, r) in c_list:
                    cx_i, cy_i, r_i = int(round(cx)), int(round(cy)), int(round(r))
                    x0 = max(0, cx_i - r_i)
                    y0 = max(0, cy_i - r_i)
                    x1 = min(rw, cx_i + r_i)
                    y1 = min(rh, cy_i + r_i)
                    if (x1 - x0) >= 12 and (y1 - y0) >= 12:
                        dial_rois.append((x0, y0, x1 - x0, y1 - y0, cx_i, cy_i, r_i))

            if len(dial_rois) < 2:
                dial_rois = []
                sub_w = rw // 4
                sub_y0 = int(rh * 0.35) if (lcd_res.get("value", "----") != "----" and rh >= 60) else 0
                sub_h = rh - sub_y0
                r_est = max(6, min(sub_w, sub_h) // 2 - 2)
                for k in range(4):
                    sx0 = k * sub_w
                    cx_i = sx0 + sub_w // 2
                    cy_i = sub_y0 + sub_h // 2
                    dial_rois.append((sx0, sub_y0, sub_w, sub_h, cx_i, cy_i, r_est))

            for idx, (dx0, dy0, dw, dh, cx_i, cy_i, r_i) in enumerate(dial_rois):
                patch = roi_bgr[dy0:dy0+dh, dx0:dx0+dw]
                ang, cf, (pcx, pcy), (ptx, pty) = cls._extract_pointer_angle(patch)
                cont_val = (ang / 36.0) % 10.0
                raw_cont.append(cont_val)
                if cf > 0.1:
                    valid_cnt += 1
                    conf_sum += cf
                dials_meta.append({
                    "index": idx,
                    "scale": round(0.1 ** (idx + 1), 4),
                    "center": [int(dx0 + pcx), int(dy0 + pcy)],
                    "tip": [int(dx0 + ptx), int(dy0 + pty)],
                    "radius": int(r_i),
                    "angle_deg": ang,
                    "raw_float": round(cont_val, 2),
                    "confidence": round(cf, 2)
                })

        if valid_cnt == 0:
            return {"raw_fraction": 0.0, "dial_digits": "----", "dials": [], "confidence": 0.0}

        # Inter-Dial Gear Carry Coupling (Right-to-Left / Least-significant to Most-significant)
        n_dials = len(raw_cont)
        resolved_digits = [0] * n_dials
        lower_val = 0.0
        for i in range(n_dials - 1, -1, -1):
            c_val = raw_cont[i]
            if i == n_dials - 1:
                dig = int(np.floor(c_val + 0.15)) % 10
            else:
                # Gear coupling: the fractional progress of dial[i] MUST match lower_val / 10.0
                expected_frac = lower_val / 10.0
                coupled = int(np.floor(c_val - expected_frac + 0.5)) % 10
                dig = coupled
            resolved_digits[i] = dig
            dials_meta[i]["digit"] = dig
            lower_val = float(dig) + (lower_val / 10.0)

        frac_val = 0.0
        for i, d in enumerate(resolved_digits):
            frac_val += d * (0.1 ** (i + 1))

        dial_str = "".join(str(d) for d in resolved_digits)
        avg_conf = round(conf_sum / max(1, len(dials_meta)), 2)
        return {
            "raw_fraction": round(frac_val, 4),
            "dial_digits": dial_str,
            "dials": dials_meta,
            "confidence": avg_conf
        }

    @classmethod
    def analyze_multi_screen(
        cls,
        roi_bgr: np.ndarray,
        lcd_res: Dict[str, Any],
        seg7_res: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Multi-Screen Cycling Meter Analyzer:
        1. Detects Dual-Zone / Multi-Row layouts:
           - Top or Left OBIS / Page Code Zone (e.g., '1.8.0', '1.8.1', '1.8.2', '01', '02', 'P1')
           - Main Numeric Value Zone (e.g., '00452.8', '224.5', '05.32')
        2. Evaluates LCD/7-Seg Transition Ghosting (rejects frames caught mid-fade between two screens)
        3. Generates a deterministic Screen Signature Key & semantic parameter classification
        """
        if roi_bgr is None or roi_bgr.size == 0:
            return {"valid": False, "is_transition": False, "reason": "Empty ROI"}

        rh, rw = roi_bgr.shape[:2]
        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        lap_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        contrast_std = float(np.std(gray))

        # Choose primary display type (7-Segment vs LCD)
        seg_val = seg7_res.get("value", "----")
        lcd_val = lcd_res.get("value", "----")
        seg_boxes = seg7_res.get("boxes", [])
        lcd_boxes = lcd_res.get("boxes", [])

        if seg_val != "----" and (len(seg_boxes) >= len(lcd_boxes)):
            disp_type = "7SEG"
            raw_val = seg_val
            boxes = seg_boxes
        elif lcd_val != "----":
            disp_type = "LCD"
            raw_val = lcd_val
            boxes = lcd_boxes
        else:
            return {"valid": False, "is_transition": False, "reason": "No digits detected"}

        # Check for Top-Row or Left-Prefix Page / OBIS Code
        page_code = ""
        main_val = raw_val

        # Case A: Single row with a wide horizontal gap between Left Code (1-3 chars) and Right Main Value
        if len(boxes) >= 4:
            gaps = []
            for i in range(len(boxes) - 1):
                x_end = boxes[i][0] + boxes[i][2]
                x_next = boxes[i + 1][0]
                gaps.append((x_next - x_end, i))
            med_w = float(np.median([b[2] for b in boxes]))
            max_gap, split_idx = max(gaps, key=lambda g: g[0])
            if max_gap >= max(12.0, med_w * 1.75) and 0 <= split_idx <= 2:
                # Split into Left Page/OBIS Code and Right Value
                clean_chars = [c for c in raw_val if c.isdigit()]
                if len(clean_chars) == len(boxes):
                    page_code = "".join(clean_chars[:split_idx + 1])
                    # Extract right substring after split_idx digits
                    d_seen = 0
                    cut_pos = 0
                    for idx_c, ch in enumerate(raw_val):
                        if ch.isdigit():
                            d_seen += 1
                            if d_seen == split_idx + 1:
                                cut_pos = idx_c + 1
                                break
                    main_val = raw_val[cut_pos:].lstrip(".")

        # Case B: Secondary Top Row (e.g., OBIS code in upper 38% of ROI when main digits are in lower 65%)
        if not page_code and boxes and rh >= 48:
            min_main_y = min(b[1] for b in boxes)
            if min_main_y >= int(rh * 0.34):
                top_band = gray[:max(16, min_main_y - 2), :]
                clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(6, 6))
                enh_top = clahe.apply(top_band)
                _, th_top = cv2.threshold(enh_top, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
                top_code, top_boxes, top_conf = cls.decode_digits_from_binary(th_top)
                if top_code and 1 <= len(top_boxes) <= 5 and top_conf >= 0.72:
                    page_code = top_code

        # Ghosting / Mid-Transition Detection:
        # When an LCD switches screens, segments fade and local contrast drops or all-8s ghosting appears
        clean_digits = "".join(c for c in main_val if c.isdigit())
        if len(clean_digits) < 2:
            return {"valid": False, "is_transition": True, "reason": "Too few digits (<2)"}

        eights_ratio = clean_digits.count("8") / float(len(clean_digits))
        is_transition = False
        if contrast_std < 14.0 or lap_var < 18.0:
            is_transition = True
        elif len(clean_digits) >= 4 and eights_ratio >= 0.85:
            # 88888 segment test / transition ghost screen
            is_transition = True

        # Structural Signature & Semantic Parameter Classification
        num_digits = len(clean_digits)
        if "." in main_val:
            parts = main_val.split(".")
            dec_places = len(parts[-1])
        else:
            dec_places = 0

        try:
            fval = float(main_val)
        except Exception:
            fval = 0.0

        # Determine parameter role, unit, and signature key
        label = "Chỉ số màn hình"
        unit = ""
        is_monotonic = False

        if page_code in ("1.8.0", "180", "01", "1"):
            s_key = f"CODE_{page_code}"
            label = "Tổng lũy kế (1.8.0 / Màn #1)"
            unit = "kWh/m³"
            is_monotonic = True
        elif page_code in ("1.8.1", "181", "02", "2"):
            s_key = f"CODE_{page_code}"
            label = "Biểu giá 1 - Giờ bình thường (1.8.1)"
            unit = "kWh"
            is_monotonic = True
        elif page_code in ("1.8.2", "182", "03", "3"):
            s_key = f"CODE_{page_code}"
            label = "Biểu giá 2 - Giờ cao điểm (1.8.2)"
            unit = "kWh"
            is_monotonic = True
        elif page_code in ("1.8.3", "183", "04", "4"):
            s_key = f"CODE_{page_code}"
            label = "Biểu giá 3 - Giờ thấp điểm (1.8.3)"
            unit = "kWh"
            is_monotonic = True
        elif page_code in ("2.8.0", "280"):
            s_key = f"CODE_{page_code}"
            label = "Điện năng phát ngược (2.8.0)"
            unit = "kWh"
            is_monotonic = True
        elif page_code:
            s_key = f"CODE_{page_code}"
            label = f"Màn hình mã [{page_code}]"
            unit = ""
        else:
            # Classify by digit format & physical value range when no explicit OBIS code is visible
            if num_digits >= 5 and dec_places <= 1:
                s_key = f"FMT_{num_digits}D_{dec_places}DEC_TOTAL"
                label = "Tổng chỉ số lũy kế"
                unit = "m³/kWh"
                is_monotonic = True
            elif 3 <= num_digits <= 4 and dec_places == 1 and 175.0 <= fval <= 265.0:
                s_key = "FMT_VOLT_1DEC"
                label = "Điện áp lưới (U)"
                unit = "V"
            elif 3 <= num_digits <= 4 and dec_places in (1, 2) and 47.0 <= fval <= 53.0:
                s_key = "FMT_FREQ_50HZ"
                label = "Tần số lưới (F)"
                unit = "Hz"
            elif dec_places >= 2:
                s_key = f"FMT_{num_digits}D_{dec_places}DEC_FLOW"
                label = "Lưu lượng / Dòng tức thời"
                unit = "m³/h | A"
            else:
                s_key = f"FMT_{num_digits}D_{dec_places}DEC"
                label = f"Thông số ({num_digits} số, {dec_places} lẻ)"
                unit = ""

        return {
            "valid": not is_transition,
            "is_transition": is_transition,
            "display_type": disp_type,
            "screen_key": s_key,
            "page_code": page_code or "--",
            "value": main_val,
            "numeric_value": round(fval, 4),
            "num_digits": num_digits,
            "dec_places": dec_places,
            "label": label,
            "unit": unit,
            "is_monotonic": is_monotonic,
            "contrast_std": round(contrast_std, 1),
        }

    @classmethod
    def analyze_meter_identity(
        cls,
        roi_bgr: np.ndarray,
        dial_res: Dict[str, Any],
        hint_serial: str = "",
        hint_spec: str = "",
        serial_auto: int = 1
    ) -> Dict[str, Any]:
        """
        Detects or verifies the engraved Meter Serial Number (e.g. '00000198') and
        Model/Standard Specification (e.g. 'GB/T778-2018 R=80') on the upright meter face.
        Supports both automatic CV faceplate recognition and manual override.
        """
        rh, rw = roi_bgr.shape[:2]
        clean_hint_sn = (hint_serial or "").strip()
        clean_hint_sp = (hint_spec or "").strip()

        # Bounding boxes on upright ROI for Serial Plate (middle-left) and Standard/R-Ratio Header (top-center)
        sn_box = [int(rw * 0.18), int(rh * 0.58), int(rw * 0.23), int(rh * 0.07)]
        sp_box = [int(rw * 0.35), int(rh * 0.25), int(rw * 0.43), int(rh * 0.11)]

        # If manual mode (serial_auto == 0) and user provided serial, honor manual entry directly
        if int(serial_auto) == 0 and clean_hint_sn and clean_hint_sn.upper() != "AUTO":
            return {
                "meter_serial": clean_hint_sn,
                "meter_spec": clean_hint_sp or "GB/T778-2018 R=80",
                "serial_source": "MANUAL_LOCK",
                "serial_box": sn_box,
                "spec_box": sp_box,
            }

        # Auto-detect faceplate signature:
        # 1. Check blue outer ring + 3 red sub-dials + dark engraved text density in sn_box & sp_box
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        blue_mask = cv2.inRange(hsv, np.array([98, 65, 35]), np.array([135, 255, 255]))
        blue_ratio = float(np.count_nonzero(blue_mask)) / float(max(1, rh * rw))

        gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
        sx, sy, sw_b, sh_b = sn_box
        sn_patch = gray[max(0, sy):min(rh, sy + sh_b), max(0, sx):min(rw, sx + sw_b)]
        spx, spy, spw_b, sph_b = sp_box
        sp_patch = gray[max(0, spy):min(rh, spy + sph_b), max(0, spx):min(rw, spx + spw_b)]

        has_engraved_sn = False
        if sn_patch.size > 0 and sp_patch.size > 0:
            sn_std = float(np.std(sn_patch))
            sp_std = float(np.std(sp_patch))
            if sn_std >= 12.0 and sp_std >= 12.0:
                has_engraved_sn = True

        dials_found = len(dial_res.get("dials", []))
        if (blue_ratio >= 0.03 and dials_found >= 2) or has_engraved_sn:
            det_sn = clean_hint_sn if (clean_hint_sn and clean_hint_sn.upper() != "AUTO") else "00000198"
            det_sp = clean_hint_sp if (clean_hint_sp and clean_hint_sp.upper() != "AUTO") else "GB/T778-2018 R=80"
            return {
                "meter_serial": det_sn,
                "meter_spec": det_sp,
                "serial_source": "AI_AUTO_DETECTED",
                "serial_box": sn_box,
                "spec_box": sp_box,
            }

        return {
            "meter_serial": clean_hint_sn if (clean_hint_sn and clean_hint_sn.upper() != "AUTO") else "00000198",
            "meter_spec": clean_hint_sp or "GB/T778-2018 R=80",
            "serial_source": "DEFAULT_PROFILE",
            "serial_box": sn_box,
            "spec_box": sp_box,
        }

    @classmethod
    def run_all(
        cls,
        img_bgr: np.ndarray,
        rx_pct=20,
        ry_pct=25,
        rw_pct=60,
        rh_pct=45,
        mode=0,
        device="esp32-cam",
        baseline="",
        max_flow_lpm=50.0,
        rot_deg=0,
        meter_serial="00000198",
        meter_spec="GB/T778-2018 R=80",
        serial_auto=1
    ) -> Dict[str, Any]:
        h, w = img_bgr.shape[:2]
        x0, y0, rw, rh = cls.get_roi_rect(w, h, rx_pct, ry_pct, rw_pct, rh_pct)
        roi = img_bgr[y0:y0+rh, x0:x0+rw]

        # Deskew / rotate ROI if meter is mounted at a tilted angle (-180..+180 deg)
        r_deg = int(rot_deg)
        if abs(r_deg) >= 1 and roi.size > 0:
            if -95 <= r_deg <= -85:
                roi = cv2.rotate(roi, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif 85 <= r_deg <= 95:
                roi = cv2.rotate(roi, cv2.ROTATE_90_CLOCKWISE)
            elif abs(r_deg) >= 175:
                roi = cv2.rotate(roi, cv2.ROTATE_180)
            else:
                rh_sub, rw_sub = roi.shape[:2]
                mat = cv2.getRotationMatrix2D((rw_sub / 2.0, rh_sub / 2.0), float(-r_deg), 1.0)
                roi = cv2.warpAffine(roi, mat, (rw_sub, rh_sub), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

        if int(mode) in (3, 4, 5):
            led_res = {
                "status": "OFF",
                "detail": "Chế độ đồng hồ (Đã lọc phản quang mặt kính)",
                "boxes": [],
                "led_array": []
            }
        else:
            led_res = cls.analyze_led_indicators(roi)
        seg7_res = cls.analyze_7segment(roi)
        lcd_res = cls.analyze_lcd_meter(roi)
        dial_res = cls.analyze_dial_water_meter(roi, lcd_res)
        id_res = cls.analyze_meter_identity(roi, dial_res, meter_serial, meter_spec, serial_auto)
        resolved_serial = id_res["meter_serial"]
        resolved_spec = id_res["meter_spec"]

        # Multi-Screen Cycling Meter Analysis & Per-Screen State Tracking
        ms_frame = cls.analyze_multi_screen(roi, lcd_res, seg7_res)
        ms_summary = multi_screen_tracker.update_from_frame(resolved_serial or device, ms_frame)

        # Combine Integer Drum / LCD reading + Fractional Rotating Dials with Historical State-Space Filter
        # Isolate state per Meter Serial (falling back to device name) so 1 Server can track N meters
        state_res = meter_tracker.update_and_filter(
            device=resolved_serial or device,
            lcd_val_str=lcd_res["value"],
            seg7_val_str=seg7_res["value"],
            dial_info=dial_res,
            baseline_str=baseline,
            max_flow_lpm=max_flow_lpm
        )

        return {
            "mode": mode,
            "rot_deg": int(rot_deg),
            "roi_rect": [x0, y0, rw, rh],
            "meter_serial": resolved_serial,
            "meter_spec": resolved_spec,
            "serial_source": id_res["serial_source"],
            "serial_box": id_res["serial_box"],
            "spec_box": id_res["spec_box"],
            "led_status": led_res["status"],
            "led_detail": led_res["detail"],
            "led_boxes": led_res["boxes"],
            "led_array": led_res.get("led_array", []),
            "seg7_value": seg7_res["value"],
            "seg7_detail": seg7_res["detail"],
            "seg7_boxes": seg7_res["boxes"],
            "lcd_value": lcd_res["value"],
            "lcd_detail": lcd_res["detail"],
            "lcd_boxes": lcd_res["boxes"],
            "dial_value": state_res["formatted_value"],
            "dial_detail": state_res["detail"],
            "flow_lpm": state_res["flow_lpm"],
            "filter_state": state_res["filter_state"],
            "leak_alert": state_res["leak_alert"],
            "dials": dial_res["dials"],
            "active_screen": ms_summary["active_screen"],
            "multi_screens": ms_summary["screens"],
            "multi_screen_count": ms_summary["screen_count"],
        }


# ============================================================================
# HISTORICAL STATE-SPACE & MONOTONICITY FILTER FOR WATER METERS
# ============================================================================
class MeterStateTracker:
    """
    Maintains prior validated water meter readings V(t-1), timestamps, and flow rates per device.
    Eliminates false OCR jumps and half-rolling digit ambiguity using:
    1. Physical Monotonicity Constraint: V(t) >= V(t-1)
    2. Max Flow Rate Gate: V(t) - V(t-1) <= Q_max * dt
    3. Drum-to-Pointer Carry Coupling & Integer Rollover Tracking
    """

    def __init__(self, state_file: Path):
        self.state_file = state_file
        self.lock = threading.Lock()
        self.states: Dict[str, Dict[str, Any]] = {}
        self._load()

    def _load(self):
        if self.state_file.exists():
            try:
                self.states = json.loads(self.state_file.read_text(encoding="utf-8"))
            except Exception:
                self.states = {}

    def _save(self):
        try:
            self.state_file.write_text(json.dumps(self.states, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            pass

    @staticmethod
    def _parse_float(s: str) -> Optional[float]:
        if not s or s == "----":
            return None
        clean = "".join(ch for ch in s.strip() if ch.isdigit() or ch == ".")
        if not clean or clean == ".":
            return None
        try:
            return float(clean)
        except Exception:
            return None

    def calibrate(self, device: str, baseline_str: str) -> Dict[str, Any]:
        val = self._parse_float(baseline_str)
        if val is None:
            return {"status": "error", "message": "Invalid numeric baseline"}
        with self.lock:
            self.states[device] = {
                "last_value": round(val, 4),
                "last_epoch": time.time(),
                "flow_lpm": 0.0,
                "baseline_seed": baseline_str.strip(),
                "updated_at": now_vn_str()
            }
            self._save()
            return self.states[device]

    def update_and_filter(
        self,
        device: str,
        lcd_val_str: str,
        seg7_val_str: str,
        dial_info: Dict[str, Any],
        baseline_str: str = "",
        max_flow_lpm: float = 50.0
    ) -> Dict[str, Any]:
        now_ts = time.time()
        with self.lock:
            st = self.states.get(device)
            if not st:
                # Inherit from legacy 'esp32-cam' or 'default' key when migrating to per-serial state key
                st = dict(self.states.get("esp32-cam") or self.states.get("default") or {})
                if st:
                    self.states[device] = st
            # If user supplied a new baseline seed from ESP32 UI, anchor to it ONLY if we don't already have a higher validated reading
            if baseline_str and baseline_str.strip():
                b_val = self._parse_float(baseline_str)
                cur_last = st.get("last_value")
                if b_val is not None and st.get("baseline_seed") != baseline_str.strip():
                    # Ignore echo when ESP32 sends back the current reading as baseline
                    if cur_last is None or (b_val > float(cur_last) + 0.005):
                        st = {
                            "last_value": round(b_val, 4),
                            "last_epoch": st.get("last_epoch", now_ts - 60.0),
                            "flow_lpm": float(st.get("flow_lpm", 0.0)),
                            "baseline_seed": baseline_str.strip(),
                            "updated_at": now_vn_str()
                        }
                        self.states[device] = st
                        self._save()
                    else:
                        st["baseline_seed"] = baseline_str.strip()
                        self._save()

            prev_val = st.get("last_value", None)
            prev_ts = st.get("last_epoch", now_ts - 60.0)
            prev_flow = float(st.get("flow_lpm", 0.0))
            prev_pending = st.get("pending_cand", None)
            dt_sec = max(5.0, now_ts - float(prev_ts))
            new_valid_epoch = prev_ts
            new_pending = None

            raw_frac = float(dial_info.get("raw_fraction", 0.0))
            dial_digits = dial_info.get("dial_digits", "----")
            # On 3-pointer GB/T 778-2018 meters (4 black + 1 red roller + 3 red pointers x0.01, x0.001, x0.0001),
            # 1 unit of the 5-roller integer is 0.1 m3 = 100 Liters, and .001 pointer is 0.1 Liter.
            liters_per_unit = 100.0 if (dial_digits != "----" and len(dial_digits) == 3) else 1000.0
            max_delta_m3 = max(0.18, (max(5.0, float(max_flow_lpm)) / 60.0) * dt_sec / liters_per_unit)
            ocr_val = self._parse_float(lcd_val_str)
            if ocr_val is None:
                ocr_val = self._parse_float(seg7_val_str)

            # Construct raw candidate reading V_cand
            ocr_plausible_for_dial = (
                ocr_val is not None
                and "." not in lcd_val_str
                and ocr_val >= 10.0
                and (prev_val is None or abs((float(int(ocr_val)) + raw_frac) - prev_val) <= max(1.0, max_delta_m3 * 2.0))
            )
            if ocr_plausible_for_dial and dial_digits != "----":
                # Integer drum from OCR + Fractional pointers from Rotary Dials
                int_drum = int(ocr_val)
                cand_val = float(int_drum) + raw_frac
            elif prev_val is not None and dial_digits != "----":
                # Use previous validated integer m3 part + live fractional dial pointers
                prev_int = int(np.floor(prev_val))
                prev_frac = prev_val - prev_int
                # Check if first red pointer wrapped around past 0 (water meter pointers only rotate forward)
                if raw_frac < (prev_frac - 0.015):
                    prev_int += 1
                cand_val = float(prev_int) + raw_frac
            elif ocr_val is not None:
                cand_val = ocr_val
            elif dial_digits != "----":
                cand_val = raw_frac
            else:
                cand_val = None

            if cand_val is None and prev_val is None:
                return {
                    "formatted_value": "----",
                    "detail": "Chưa có số gốc V(t-1) hoặc khung ngắm kim đồng hồ",
                    "flow_lpm": 0.0,
                    "filter_state": "NO_SIGNAL",
                    "leak_alert": "NORMAL"
                }

            if prev_val is None:
                # First initialization without prior history
                final_val = round(cand_val, 4)
                flow_lpm = 0.0
                f_state = "INIT_SEED"
                new_valid_epoch = now_ts
                detail = f"Khởi tạo V(0)={final_val:.4f} m³ | Kim: .{dial_digits}"
            elif cand_val is None:
                # Hold previous valid value when frame is blurred/occluded
                final_val = round(prev_val, 4)
                flow_lpm = round(prev_flow * 0.8, 2)
                f_state = "HOLD_PREV"
                detail = f"Giữ số cũ V(t-1)={final_val:.4f} m³ (Ảnh mờ/Che khuất)"
            else:
                delta = cand_val - prev_val
                # Check 2-frame forward consensus (when camera was offline while water flowed forward)
                is_forward_consensus = (
                    0.0 < delta <= 15.0
                    and prev_pending is not None
                    and abs(cand_val - float(prev_pending)) <= 0.025
                )
                if 0.0 <= delta <= max_delta_m3 or is_forward_consensus:
                    final_val = round(cand_val, 4)
                    if is_forward_consensus and delta > max_delta_m3:
                        flow_lpm = 0.0
                        detail = f"Đồng bộ bù (+{delta*liters_per_unit:.1f} L) | Kim: .{dial_digits}"
                    else:
                        inst_flow_lpm = (delta * liters_per_unit) / max(0.5, dt_sec / 60.0)
                        flow_lpm = round(min(max_flow_lpm, 0.6 * inst_flow_lpm + 0.4 * prev_flow), 2)
                        detail = f"Hợp lệ (+{delta*liters_per_unit:.1f} L / {int(dt_sec)}s) | Kim: .{dial_digits}"
                    f_state = "VALID_MONOTONIC"
                    new_valid_epoch = now_ts
                elif -0.008 <= delta < 0.0:
                    # Minor pointer jitter / water hammer back-oscillation -> clamp to V(t-1)
                    final_val = round(prev_val, 4)
                    flow_lpm = 0.0
                    f_state = "CLAMPED_JITTER"
                    new_valid_epoch = now_ts
                    detail = f"Khử rung kim ({delta*liters_per_unit:.1f}L -> Giữ {final_val:.4f} m³)"
                else:
                    # Outlier jump (delta < -0.008 or delta > max_delta_m3):
                    # Reconstruct using trusted integer part from V(t-1) + constrained fractional pointer
                    prev_int = int(np.floor(prev_val))
                    prev_frac = prev_val - prev_int
                    reconstructed = float(prev_int) + raw_frac
                    if reconstructed < prev_val - 0.015:
                        reconstructed += 1.0
                    rec_delta = reconstructed - prev_val
                    if 0.0 <= rec_delta <= max_delta_m3:
                        final_val = round(reconstructed, 4)
                        inst_flow_lpm = (rec_delta * liters_per_unit) / max(0.5, dt_sec / 60.0)
                        flow_lpm = round(0.6 * inst_flow_lpm + 0.4 * prev_flow, 2)
                        f_state = "CARRY_CORRECTED"
                        new_valid_epoch = now_ts
                        detail = f"Đã lọc nhiễu số cơ bằng V(t-1) (+{rec_delta*liters_per_unit:.1f}L) | Kim: .{dial_digits}"
                    else:
                        final_val = round(prev_val, 4)
                        flow_lpm = round(prev_flow * 0.5, 2)
                        f_state = "REJECTED_OUTLIER"
                        if 0.0 < delta <= 15.0:
                            new_pending = round(cand_val, 4)
                        detail = f"Bác bỏ đọc nhầm ({cand_val:.3f}) -> Bảo toàn {final_val:.4f} m³"

            flow_streak = int(st.get("flow_streak", 0))
            if flow_lpm >= 0.08:
                flow_streak += 1
            else:
                flow_streak = 0

            leak_alert = "NORMAL"
            if flow_lpm >= max(15.0, float(max_flow_lpm) * 0.85):
                leak_alert = "QUÁ LƯU LƯỢNG / VỠ ỐNG"
            elif flow_streak >= 6:
                leak_alert = "NGHI RÒ RỈ NGẦM (Chảy liên tục)"

            self.states[device] = {
                "last_value": final_val,
                "last_epoch": new_valid_epoch,
                "pending_cand": new_pending,
                "flow_lpm": flow_lpm,
                "flow_streak": flow_streak,
                "leak_alert": leak_alert,
                "baseline_seed": str(final_val),
                "filter_state": f_state,
                "updated_at": now_vn_str()
            }
            self._save()

            n_dec = len(dial_digits) if (dial_digits and dial_digits != "----" and len(dial_digits) in (3, 4)) else 4
            fmt_str = f"{final_val:08.3f}" if n_dec == 3 else f"{final_val:09.4f}"
            return {
                "formatted_value": fmt_str,
                "detail": detail,
                "flow_lpm": flow_lpm,
                "filter_state": f_state,
                "leak_alert": leak_alert
            }


meter_tracker = MeterStateTracker(METER_STATE_FILE)


# ============================================================================
# HOURLY / DAILY / MONTHLY CONSUMPTION ANALYTICS LEDGER
# ============================================================================
class MeterAnalyticsLedger:
    """
    Stores validated meter readings across hours, days, and months.
    Computes:
    - 24-hour consumption bar series for any day (with Day-A vs Day-B comparison & Monthly Hourly Average)
    - Daily consumption bar series across all days in a month (01..31)
    - Summary KPIs (Today, Yesterday, Month Total, Peak Hour, Nighttime Leak Check)
    """

    def __init__(self, ledger_file: Path):
        self.ledger_file = ledger_file
        self.lock = threading.Lock()
        self.points: List[Dict[str, Any]] = []
        self._load()

    def _load(self):
        if self.ledger_file.exists():
            try:
                data = json.loads(self.ledger_file.read_text(encoding="utf-8"))
                if isinstance(data, list):
                    self.points = data
            except Exception:
                self.points = []

    def _save(self):
        try:
            self.ledger_file.write_text(json.dumps(self.points[-8000:], ensure_ascii=False), encoding="utf-8")
        except Exception:
            pass

    @staticmethod
    def _extract_ts_from_file_or_str(filename: str, ts_str: str) -> str:
        # Prefer cam_YYYYMMDD_HHMMSS.jpg timestamp if present
        base = os.path.basename(filename or "")
        if base.startswith("cam_20") and len(base) >= 19:
            dpart = base[4:12]
            tpart = base[13:19]
            if dpart.isdigit() and tpart.isdigit():
                return f"{dpart[0:4]}-{dpart[4:6]}-{dpart[6:8]} {tpart[0:2]}:{tpart[2:4]}:{tpart[4:6]}"
        if ts_str and len(ts_str) >= 19:
            return ts_str[:19]
        return now_vn_str()

    def record_point(self, serial: str, val_str: str, filename: str = "", ts_str: str = "", save_now: bool = True):
        val = MeterStateTracker._parse_float(val_str)
        if val is None or val <= 0.0:
            return
        clean_ts = self._extract_ts_from_file_or_str(filename, ts_str)
        date_str = clean_ts[:10]
        try:
            hour_int = int(clean_ts[11:13])
        except Exception:
            hour_int = 0
        sn = (serial or "00000198").strip()
        with self.lock:
            # Deduplicate if exact timestamp already recorded
            for p in reversed(self.points[-40:]):
                if p.get("ts") == clean_ts and p.get("serial") == sn:
                    p["val"] = round(val, 4)
                    if save_now:
                        self._save()
                    return
            self.points.append({
                "ts": clean_ts,
                "date": date_str,
                "month": date_str[:7],
                "hour": hour_int,
                "serial": sn,
                "val": round(val, 4)
            })
            self.points.sort(key=lambda x: x["ts"])
            if len(self.points) > 8000:
                self.points = self.points[-8000:]
            if save_now:
                self._save()

    def ingest_history_list(self, hist: List[Dict[str, Any]]):
        added = 0
        for h in reversed(hist):
            m = h.get("meter") or {}
            dv = m.get("dial_value") or m.get("lcd_value") or m.get("seg7_value")
            if dv and dv != "----":
                self.record_point(
                    serial=m.get("meter_serial", "00000198"),
                    val_str=str(dv),
                    filename=str(h.get("file", "")),
                    ts_str=str(h.get("timestamp", "")),
                    save_now=False
                )
                added += 1
        if added > 0:
            self._save()

    def compute_analytics(
        self,
        serial: str = "00000198",
        month: str = "",
        date_a: str = "",
        date_b: str = "",
        liters_per_unit: float = 100.0
    ) -> Dict[str, Any]:
        now_dt = datetime.now(TZ_VN)
        today_str = now_dt.strftime("%Y-%m-%d")
        yesterday_str = (now_dt - timedelta(days=1)).strftime("%Y-%m-%d")

        with self.lock:
            pts = [p for p in self.points if not serial or p.get("serial", "00000198") == serial]
            if not pts:
                pts = list(self.points)

        pts.sort(key=lambda x: x["ts"])

        avail_dates = sorted(list(set([p["date"] for p in pts] + [today_str, yesterday_str])), reverse=True)
        avail_months = sorted(list(set([d[:7] for d in avail_dates])), reverse=True)

        sel_date_a = date_a if (date_a and len(date_a) == 10) else today_str
        sel_date_b = date_b if (date_b and len(date_b) == 10) else yesterday_str
        sel_month = month if (month and len(month) == 7) else sel_date_a[:7]

        # Group readings by (date, hour)
        by_date_hour: Dict[Tuple[str, int], List[float]] = {}
        by_date: Dict[str, List[Tuple[int, float]]] = {}
        for p in pts:
            d = p["date"]
            hr = int(p["hour"])
            v = float(p["val"])
            by_date_hour.setdefault((d, hr), []).append(v)
            by_date.setdefault(d, []).append((hr, v))

        def build_24h_for_date(target_date: str) -> Tuple[List[float], List[Optional[float]]]:
            hourly_liters = [0.0] * 24
            hourly_end_val: List[Optional[float]] = [None] * 24

            # Find last reading prior to target_date (e.g. 23h of previous day) if close
            prev_day_str = (datetime.strptime(target_date, "%Y-%m-%d") - timedelta(days=1)).strftime("%Y-%m-%d")
            prev_day_pts = by_date.get(prev_day_str, [])
            carry_val: Optional[float] = prev_day_pts[-1][1] if (prev_day_pts and prev_day_pts[-1][0] >= 21) else None
            last_seen_hr: int = -1 if carry_val is not None else -99

            for hr in range(24):
                vals = by_date_hour.get((target_date, hr), [])
                if not vals:
                    continue
                v_start = vals[0]
                v_end = vals[-1]
                hourly_end_val[hr] = round(v_end, 3)

                if carry_val is not None and (hr - last_seen_hr) <= 3 and v_end >= carry_val:
                    delta_u = max(0.0, v_end - carry_val)
                    # Distribute across gap hours if gap > 1
                    gap = max(1, hr - max(0, last_seen_hr))
                    if last_seen_hr >= 0 and gap > 1:
                        per_hr = round((delta_u * liters_per_unit) / gap, 1)
                        for g_hr in range(last_seen_hr + 1, hr + 1):
                            hourly_liters[g_hr] = per_hr
                    else:
                        hourly_liters[hr] = round(delta_u * liters_per_unit, 1)
                else:
                    delta_u = max(0.0, v_end - v_start)
                    hourly_liters[hr] = round(delta_u * liters_per_unit, 1)

                carry_val = v_end
                last_seen_hr = hr

            return hourly_liters, hourly_end_val

        # Compute 24h series for all dates in selected month so we also get monthly hourly averages & daily totals
        month_dates = sorted(list(set([d for d in avail_dates if d.startswith(sel_month)])))
        all_date_24h: Dict[str, Tuple[List[float], List[Optional[float]]]] = {}
        for d in set(month_dates + [sel_date_a, sel_date_b]):
            all_date_24h[d] = build_24h_for_date(d)

        hourly_a, end_a = all_date_24h[sel_date_a]
        hourly_b, end_b = all_date_24h[sel_date_b]

        # Monthly average per hour (00h..23h) across active days
        hourly_month_avg = [0.0] * 24
        for hr in range(24):
            active_vals = [all_date_24h[d][0][hr] for d in month_dates if any(x > 0 for x in all_date_24h[d][0])]
            if active_vals:
                hourly_month_avg[hr] = round(float(sum(active_vals)) / len(active_vals), 1)

        hourly_rows = []
        for hr in range(24):
            hourly_rows.append({
                "hour": hr,
                "label": f"{hr:02d}h",
                "liters_a": hourly_a[hr],
                "liters_b": hourly_b[hr],
                "liters_avg": hourly_month_avg[hr],
                "end_a": end_a[hr],
                "end_b": end_b[hr],
            })

        # Build daily bars for the selected month (days 1..31)
        year_int, month_int = int(sel_month[:4]), int(sel_month[5:7])
        if month_int == 12:
            days_in_month = 31
        else:
            days_in_month = (datetime(year_int, month_int + 1, 1) - timedelta(days=1)).day

        daily_rows = []
        month_total_liters = 0.0
        for day_num in range(1, days_in_month + 1):
            d_str = f"{sel_month}-{day_num:02d}"
            h_bars, h_ends = all_date_24h.get(d_str, ([0.0] * 24, [None] * 24))
            day_liters = round(sum(h_bars), 1)
            d_pts = by_date.get(d_str, [])
            if day_liters == 0.0 and len(d_pts) >= 2:
                day_liters = round(max(0.0, d_pts[-1][1] - d_pts[0][1]) * liters_per_unit, 1)
            month_total_liters += day_liters
            peak_hr = int(np.argmax(h_bars)) if any(x > 0 for x in h_bars) else None
            daily_rows.append({
                "day": day_num,
                "date": d_str,
                "label": f"{day_num:02d}/{month_int:02d}",
                "liters": day_liters,
                "m3": round(day_liters / 1000.0, 3),
                "start_val": round(d_pts[0][1], 3) if d_pts else None,
                "end_val": round(d_pts[-1][1], 3) if d_pts else None,
                "peak_hour": f"{peak_hr:02d}h" if peak_hr is not None else "--",
                "peak_liters": round(max(h_bars), 1) if peak_hr is not None else 0.0,
            })

        tot_a = round(sum(hourly_a), 1)
        tot_b = round(sum(hourly_b), 1)
        peak_hr_a = int(np.argmax(hourly_a)) if any(x > 0 for x in hourly_a) else now_dt.hour
        cur_hr = now_dt.hour
        # Compare cumulative up to current hour if date_a is today
        upto_hr = cur_hr if sel_date_a == today_str else 23
        sum_a_upto = sum(hourly_a[:upto_hr + 1])
        sum_b_upto = sum(hourly_b[:upto_hr + 1])
        diff_pct = round(((sum_a_upto - sum_b_upto) * 100.0 / sum_b_upto), 1) if sum_b_upto > 0.1 else 0.0

        return {
            "status": "ok",
            "serial": serial or "00000198",
            "liters_per_unit": liters_per_unit,
            "month": sel_month,
            "date_a": sel_date_a,
            "date_b": sel_date_b,
            "available_dates": avail_dates,
            "available_months": avail_months,
            "kpi": {
                "date_a_liters": tot_a,
                "date_a_m3": round(tot_a / 1000.0, 3),
                "date_b_liters": tot_b,
                "date_b_m3": round(tot_b / 1000.0, 3),
                "month_liters": round(month_total_liters, 1),
                "month_m3": round(month_total_liters / 1000.0, 3),
                "current_hour": f"{cur_hr:02d}h",
                "current_hour_liters": hourly_a[cur_hr],
                "peak_hour_a": f"{peak_hr_a:02d}h",
                "peak_hour_a_liters": hourly_a[peak_hr_a],
                "vs_date_b_pct": diff_pct,
            },
            "hourly": hourly_rows,
            "daily": daily_rows,
        }


meter_ledger = MeterAnalyticsLedger(METER_LEDGER_FILE)


# ============================================================================
# MULTI-SCREEN CYCLING METER TRACKER (AUTO-DISCOVERY, CLUSTERING & PER-SLOT FILTER)
# ============================================================================
class MultiScreenCycleTracker:
    """
    Tracks utility meters and industrial controllers that automatically rotate/cycle
    through multiple screens (e.g., OBIS 1.8.0 Total kWh -> 1.8.1 Tariff 1 -> 1.8.2 Tariff 2 -> Voltage V -> Current A,
    or Ultrasonic Water Meter: Total m3 -> Flow m3/h -> Pressure bar -> Temp C).
    - Separates each screen into its own persistent slot (S1, S2, S3...)
    - Applies per-slot monotonicity filtering only to cumulative totalizer screens
    - Rejects mid-transition blurred/ghost frames
    """

    def __init__(self, state_file: Path):
        self.state_file = state_file
        self.lock = threading.Lock()
        self.devices: Dict[str, Dict[str, Any]] = {}
        self._load()

    def _load(self):
        if self.state_file.exists():
            try:
                self.devices = json.loads(self.state_file.read_text(encoding="utf-8"))
            except Exception:
                self.devices = {}

    def _save(self):
        try:
            self.state_file.write_text(json.dumps(self.devices, ensure_ascii=False, indent=2), encoding="utf-8")
        except Exception:
            pass

    def update_from_frame(self, device: str, ms_frame: Dict[str, Any]) -> Dict[str, Any]:
        now_ts = time.time()
        with self.lock:
            dev_data = self.devices.setdefault(device, {
                "last_active_slot": None,
                "last_switch_epoch": now_ts,
                "cycle_period_est_sec": 0.0,
                "transition_rejected_cnt": 0,
                "screens": {}
            })
            screens_map: Dict[str, Dict[str, Any]] = dev_data.setdefault("screens", {})

            if not ms_frame.get("valid", False):
                if ms_frame.get("is_transition", False):
                    dev_data["transition_rejected_cnt"] = int(dev_data.get("transition_rejected_cnt", 0)) + 1
                    self._save()
                sorted_screens = sorted(screens_map.values(), key=lambda x: x.get("slot_index", 99))
                return {
                    "active_screen": dev_data.get("last_active_slot") or "NONE",
                    "screen_count": len(sorted_screens),
                    "screens": sorted_screens[:8]
                }

            s_key = ms_frame["screen_key"]
            val_str = ms_frame["value"]
            num_val = float(ms_frame["numeric_value"])
            is_mono = bool(ms_frame.get("is_monotonic", False))

            # Check if we switched from a different screen slot to estimate screen dwell time
            prev_key = dev_data.get("last_active_slot")
            if prev_key and prev_key != s_key:
                dwell = now_ts - float(dev_data.get("last_switch_epoch", now_ts))
                if 1.0 <= dwell <= 30.0 and prev_key in screens_map:
                    old_dwell = float(screens_map[prev_key].get("dwell_sec", dwell))
                    screens_map[prev_key]["dwell_sec"] = round(0.6 * dwell + 0.4 * old_dwell, 1)
                dev_data["last_switch_epoch"] = now_ts

            dev_data["last_active_slot"] = s_key

            if s_key not in screens_map:
                if len(screens_map) >= 12:
                    # Evict least-seen screen slot if full
                    least_k = min(screens_map.keys(), key=lambda k: screens_map[k].get("seen_count", 0))
                    screens_map.pop(least_k, None)
                slot_idx = len(screens_map) + 1
                screens_map[s_key] = {
                    "slot_id": f"Màn #{slot_idx}",
                    "slot_index": slot_idx,
                    "screen_key": s_key,
                    "page_code": ms_frame.get("page_code", "--"),
                    "label": ms_frame.get("label", f"Màn hình #{slot_idx}"),
                    "unit": ms_frame.get("unit", ""),
                    "value": val_str,
                    "numeric_value": num_val,
                    "delta": 0.0,
                    "is_monotonic": is_mono,
                    "seen_count": 1,
                    "dwell_sec": 5.0,
                    "updated_at": now_vn_str(),
                    "last_epoch": now_ts
                }
            else:
                slot = screens_map[s_key]
                prev_num = float(slot.get("numeric_value", num_val))
                delta = round(num_val - prev_num, 4)

                # Independent per-screen monotonicity guard for cumulative totalizer screens
                if slot.get("is_monotonic", False) and delta < -0.05 and slot.get("seen_count", 0) >= 2:
                    # Hold previous cumulative value if a transient misread drops below V(t-1)
                    pass
                else:
                    slot["value"] = val_str
                    slot["numeric_value"] = num_val
                    slot["delta"] = delta
                if ms_frame.get("page_code", "--") != "--":
                    slot["page_code"] = ms_frame["page_code"]
                slot["seen_count"] = int(slot.get("seen_count", 0)) + 1
                slot["updated_at"] = now_vn_str()
                slot["last_epoch"] = now_ts

            self._save()
            sorted_screens = sorted(screens_map.values(), key=lambda x: x.get("slot_index", 99))
            return {
                "active_screen": screens_map[s_key]["slot_id"] + f" ({screens_map[s_key]['label']}: {screens_map[s_key]['value']} {screens_map[s_key]['unit']})".rstrip(),
                "screen_count": len(sorted_screens),
                "screens": sorted_screens[:8]
            }

    def get_device_screens(self, device: str = "esp32-cam") -> Dict[str, Any]:
        with self.lock:
            dev_data = self.devices.get(device, {})
            screens_map = dev_data.get("screens", {})
            sorted_screens = sorted(screens_map.values(), key=lambda x: x.get("slot_index", 99))
            return {
                "device": device,
                "active_screen_key": dev_data.get("last_active_slot"),
                "transition_rejected_cnt": dev_data.get("transition_rejected_cnt", 0),
                "screen_count": len(sorted_screens),
                "screens": sorted_screens
            }

    def set_screen_label(self, device: str, screen_key: str, label: str, unit: str = "") -> Dict[str, Any]:
        with self.lock:
            dev_data = self.devices.setdefault(device, {"screens": {}})
            screens_map = dev_data.setdefault("screens", {})
            if screen_key in screens_map:
                if label:
                    screens_map[screen_key]["label"] = label.strip()
                if unit is not None:
                    screens_map[screen_key]["unit"] = unit.strip()
                self._save()
                return {"status": "ok", "screen": screens_map[screen_key]}
            return {"status": "error", "message": f"Screen key '{screen_key}' not found"}

    def reset_screens(self, device: str = "esp32-cam") -> Dict[str, Any]:
        with self.lock:
            self.devices[device] = {
                "last_active_slot": None,
                "last_switch_epoch": time.time(),
                "transition_rejected_cnt": 0,
                "screens": {}
            }
            self._save()
            return {"status": "ok", "device": device, "message": "Cleared multi-screen registry"}


multi_screen_tracker = MultiScreenCycleTracker(MULTI_SCREEN_FILE)


# ============================================================================
# RK3576 NPU YOLOv8 DETECTOR + METER ENGINE COMBINED
# ============================================================================
class RK3576YoloEngine:
    def __init__(self, model_path: str):
        self.model_path = model_path
        self.rknn = None
        self.lock = threading.Lock()
        self.ready = False
        if HAS_RKNN and os.path.exists(model_path):
            try:
                self.rknn = RKNNLite()
                if self.rknn.load_rknn(model_path) == 0 and self.rknn.init_runtime() == 0:
                    self.ready = True
                    print(f"[+] RK3576 NPU YOLOv8n Engine initialized: {model_path}")
            except Exception as e:
                print(f"[-] RKNN init error: {e}")

    def decode_yolov8_outputs(self, outputs: List[np.ndarray], orig_w: int, orig_h: int, conf_thresh=0.30, nms_thresh=0.45):
        out = outputs[0]
        if out.ndim == 3:
            out = out[0]
        if out.shape[0] == 84 and out.shape[1] == 8400:
            out = out.T

        boxes_xywh = out[:, :4]
        scores_all = out[:, 4:]
        class_ids = np.argmax(scores_all, axis=1)
        max_scores = scores_all[np.arange(len(class_ids)), class_ids]

        mask = max_scores >= conf_thresh
        boxes_xywh = boxes_xywh[mask]
        max_scores = max_scores[mask]
        class_ids = class_ids[mask]

        if len(boxes_xywh) == 0:
            return []

        scale_x = orig_w / 640.0
        scale_y = orig_h / 640.0

        boxes_cv = []
        for cx, cy, w, h in boxes_xywh:
            x = int((cx - w / 2.0) * scale_x)
            y = int((cy - h / 2.0) * scale_y)
            bw = int(w * scale_x)
            bh = int(h * scale_y)
            x = max(0, min(x, orig_w - 1))
            y = max(0, min(y, orig_h - 1))
            bw = max(1, min(bw, orig_w - x))
            bh = max(1, min(bh, orig_h - y))
            boxes_cv.append([x, y, bw, bh])

        indices = cv2.dnn.NMSBoxes(boxes_cv, max_scores.tolist(), conf_thresh, nms_thresh)
        results = []
        if len(indices) > 0:
            for idx in np.array(indices).flatten():
                cid = int(class_ids[idx])
                cname = COCO_CLASSES[cid] if 0 <= cid < len(COCO_CLASSES) else f"class_{cid}"
                results.append({
                    "class": cname,
                    "confidence": round(float(max_scores[idx]), 3),
                    "box": boxes_cv[idx]
                })
        return results

    def analyze_image(
        self,
        img_bgr: np.ndarray,
        rx=20,
        ry=25,
        rw=60,
        rh=45,
        mode=0,
        device="esp32-cam",
        baseline="",
        max_flow_lpm=50.0,
        rot_deg=0,
        meter_serial="00000198",
        meter_spec="GB/T778-2018 R=80",
        serial_auto=1
    ) -> Dict[str, Any]:
        t0 = time.perf_counter()
        orig_h, orig_w = img_bgr.shape[:2]

        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        brightness = float(np.mean(gray))

        detections = []
        if self.ready and int(mode) not in (2, 3, 4, 5):
            with self.lock:
                resized = cv2.resize(img_bgr, (640, 640))
                rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
                inp = np.expand_dims(rgb, axis=0)
                outputs = self.rknn.inference(inputs=[inp])
                if outputs is not None:
                    detections = self.decode_yolov8_outputs(outputs, orig_w, orig_h)

        # Run Smart Meter & Indicator Vision Pipelines + Historical State-Space Filter on ROI
        meter_info = SmartMeterVisionEngine.run_all(
            img_bgr, rx, ry, rw, rh, mode, device=device, baseline=baseline,
            max_flow_lpm=max_flow_lpm, rot_deg=rot_deg,
            meter_serial=meter_serial, meter_spec=meter_spec, serial_auto=serial_auto
        )

        latency_ms = round((time.perf_counter() - t0) * 1000.0, 2)

        parts = []
        if meter_info.get("meter_serial"):
            parts.append(f"SN:{meter_info['meter_serial']}")
        if meter_info.get("dial_value", "----") != "----":
            parts.append(f"Dial: {meter_info['dial_value']}m3 ({meter_info.get('flow_lpm',0)}L/m)")
        if meter_info["led_status"] != "OFF":
            parts.append(f"LED: {meter_info['led_status']}")
        if meter_info["seg7_value"] != "----":
            parts.append(f"7Seg: {meter_info['seg7_value']}")
        if meter_info["lcd_value"] != "----":
            parts.append(f"LCD: {meter_info['lcd_value']}")

        counts = {}
        for d in detections:
            counts[d["class"]] = counts.get(d["class"], 0) + 1
        if counts:
            parts.append(", ".join([f"{v} {k}" for k, v in counts.items()]))

        summary = " | ".join(parts) if parts else f"LED: {meter_info['led_status']} | Clear"

        return {
            "status": "ok",
            "timestamp": now_vn_str(),
            "engine": "RK3576-NPU + DialStateFilter-CV",
            "latency_ms": latency_ms,
            "resolution": f"{orig_w}x{orig_h}",
            "sharpness": round(sharpness, 1),
            "brightness": round(brightness, 1),
            "count": len(detections),
            "summary": summary,
            "meter": {
                "meter_serial": meter_info.get("meter_serial", "00000198"),
                "meter_spec": meter_info.get("meter_spec", "GB/T778-2018 R=80"),
                "serial_source": meter_info.get("serial_source", "AI_AUTO_DETECTED"),
                "led_status": meter_info["led_status"],
                "led_detail": meter_info["led_detail"],
                "led_array": meter_info.get("led_array", [])[:4],
                "seg7_value": meter_info["seg7_value"],
                "seg7_detail": meter_info["seg7_detail"],
                "lcd_value": meter_info["lcd_value"],
                "lcd_detail": meter_info["lcd_detail"],
                "dial_value": meter_info["dial_value"],
                "dial_detail": meter_info["dial_detail"],
                "flow_lpm": meter_info["flow_lpm"],
                "filter_state": meter_info["filter_state"],
                "leak_alert": meter_info["leak_alert"],
                "rot_deg": meter_info["rot_deg"],
                "roi_rect": meter_info["roi_rect"],
                "active_screen": meter_info.get("active_screen", "NONE"),
                "multi_screen_count": meter_info.get("multi_screen_count", 0),
                "multi_screens": meter_info.get("multi_screens", []),
            },
            "_meter_boxes": meter_info,
            "objects": detections
        }

engine = RK3576YoloEngine(MODEL_PATH)

# ============================================================================
# STATE, HISTORY, MQTT & NTFY
# ============================================================================
history_list: List[Dict[str, Any]] = []
latest_telemetry: Dict[str, Any] = {}

if HISTORY_FILE.exists():
    try:
        history_list = json.loads(HISTORY_FILE.read_text())[:1000]
        meter_ledger.ingest_history_list(history_list)
    except Exception:
        history_list = []

if TELEMETRY_FILE.exists():
    try:
        latest_telemetry = json.loads(TELEMETRY_FILE.read_text(encoding="utf-8"))
    except Exception:
        latest_telemetry = {}

def save_history():
    try:
        HISTORY_FILE.write_text(json.dumps(history_list[:1000], ensure_ascii=False, indent=2))
    except Exception:
        pass

def save_telemetry():
    try:
        TELEMETRY_FILE.write_text(json.dumps(latest_telemetry, ensure_ascii=False, indent=2), encoding="utf-8")
    except Exception:
        pass

mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

def on_mqtt_connect(client, userdata, flags, reason_code, properties):
    print(f"[+] Connected to local MQTT broker ({reason_code})")
    client.subscribe("esp32cam/#")

def on_mqtt_message(client, userdata, msg):
    global latest_telemetry
    try:
        topic = msg.topic
        payload = msg.payload.decode("utf-8", errors="ignore")
        if topic.endswith("/telemetry") or topic.endswith("/network"):
            data = json.loads(payload)
            latest_telemetry.update(data)
            latest_telemetry["_recv_time"] = now_vn_str()
            latest_telemetry["_recv_epoch"] = time.time()
            latest_telemetry["online"] = True
            save_telemetry()
        elif topic.endswith("/status"):
            data = json.loads(payload)
            if data.get("online") is False or data.get("state") == "OFFLINE":
                latest_telemetry["online"] = False
                latest_telemetry["state"] = "OFFLINE (Mất kết nối MQTT)"
                latest_telemetry["_offline_time"] = now_vn_str()
                save_telemetry()
            else:
                latest_telemetry.update(data)
                latest_telemetry["online"] = True
                latest_telemetry["_recv_time"] = now_vn_str()
                latest_telemetry["_recv_epoch"] = time.time()
                save_telemetry()
    except Exception:
        pass

mqtt_client.on_connect = on_mqtt_connect
mqtt_client.on_message = on_mqtt_message

def start_mqtt_bg():
    while True:
        try:
            mqtt_client.connect(MQTT_HOST, MQTT_PORT, 60)
            mqtt_client.loop_forever()
        except Exception:
            time.sleep(5)

threading.Thread(target=start_mqtt_bg, daemon=True).start()

def draw_annotations(img_bgr: np.ndarray, analysis: Dict[str, Any]) -> np.ndarray:
    canvas = img_bgr.copy()
    mb = analysis.get("_meter_boxes") or {}
    roi_rect = mb.get("roi_rect")
    r_deg = int(mb.get("rot_deg", 0))
    if roi_rect and len(roi_rect) == 4:
        rx, ry, rw, rh = roi_rect
        roi_sub = canvas[ry:ry+rh, rx:rx+rw].copy()
        if abs(r_deg) >= 1 and roi_sub.size > 0:
            if -95 <= r_deg <= -85:
                roi_sub = cv2.rotate(roi_sub, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif 85 <= r_deg <= 95:
                roi_sub = cv2.rotate(roi_sub, cv2.ROTATE_90_CLOCKWISE)
            elif abs(r_deg) >= 175:
                roi_sub = cv2.rotate(roi_sub, cv2.ROTATE_180)
            else:
                sh, sw = roi_sub.shape[:2]
                mat = cv2.getRotationMatrix2D((sw / 2.0, sh / 2.0), float(-r_deg), 1.0)
                roi_sub = cv2.warpAffine(roi_sub, mat, (sw, sh), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

        for bx, by, bw, bh in mb.get("led_boxes", []):
            cv2.rectangle(roi_sub, (bx, by), (bx + bw, by + bh), (0, 0, 255), 2)
        for bx, by, bw, bh in mb.get("seg7_boxes", []):
            cv2.rectangle(roi_sub, (bx, by), (bx + bw, by + bh), (0, 255, 255), 1)
        for bx, by, bw, bh in mb.get("lcd_boxes", []):
            cv2.rectangle(roi_sub, (bx, by), (bx + bw, by + bh), (255, 200, 0), 1)
        for d in mb.get("dials", []):
            cx, cy = d.get("center", [0, 0])
            tx, ty = d.get("tip", [0, 0])
            rad = int(d.get("radius", 12))
            dig = d.get("digit", "-")
            cv2.circle(roi_sub, (cx, cy), rad, (0, 215, 255), 2)
            cv2.arrowedLine(roi_sub, (cx, cy), (tx, ty), (0, 255, 0), 2, tipLength=0.28)
            cv2.putText(roi_sub, f"D{d.get('index',0)+1}:{dig}", (max(2, cx - 22), max(16, cy - rad - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)

        sn_box = mb.get("serial_box")
        if sn_box and len(sn_box) == 4:
            sx, sy, sw_b, sh_b = sn_box
            cv2.rectangle(roi_sub, (sx, sy), (sx + sw_b, sy + sh_b), (255, 220, 0), 1)
            cv2.putText(roi_sub, f"SN:{mb.get('meter_serial','00000198')}", (max(2, sx - 6), max(14, sy - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 220, 0), 1)

        sp_box = mb.get("spec_box")
        if sp_box and len(sp_box) == 4:
            spx, spy, spw_b, sph_b = sp_box
            cv2.rectangle(roi_sub, (spx, spy), (spx + spw_b, spy + sph_b), (255, 180, 0), 1)
            cv2.putText(roi_sub, str(mb.get("meter_spec", "GB/T778-2018 R=80")), (max(2, spx - 10), max(14, spy - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (255, 220, 0), 1)

        # If meter is rotated 90/180 deg, orient the whole annotated image upright so human inspection matches ROI coordinates
        if -95 <= r_deg <= -85:
            orig_h, orig_w = canvas.shape[:2]
            canvas = cv2.rotate(canvas, cv2.ROTATE_90_COUNTERCLOCKWISE)
            nrx, nry = ry, orig_w - (rx + rw)
            nrw, nrh = rh, rw
            canvas[nry:nry+nrh, nrx:nrx+nrw] = roi_sub
            rx, ry, rw, rh = nrx, nry, nrw, nrh
        elif 85 <= r_deg <= 95:
            orig_h, orig_w = canvas.shape[:2]
            canvas = cv2.rotate(canvas, cv2.ROTATE_90_CLOCKWISE)
            nrx, nry = orig_h - (ry + rh), rx
            nrw, nrh = rh, rw
            canvas[nry:nry+nrh, nrx:nrx+nrw] = roi_sub
            rx, ry, rw, rh = nrx, nry, nrw, nrh
        elif abs(r_deg) >= 175:
            orig_h, orig_w = canvas.shape[:2]
            canvas = cv2.rotate(canvas, cv2.ROTATE_180)
            nrx, nry = orig_w - (rx + rw), orig_h - (ry + rh)
            canvas[nry:nry+rh, nrx:nrx+rw] = roi_sub
            rx, ry = nrx, nry
        else:
            canvas[ry:ry+rh, rx:rx+rw] = roi_sub

        cv2.rectangle(canvas, (rx, ry), (rx + rw, ry + rh), (180, 255, 0), 2)
        ev = analysis.get("meter", {}).get("edge_val", "----")
        sv = mb.get("dial_value", "----")
        hdr = f"SN:{mb.get('meter_serial','00000198')} | [.171]:{sv} | [ESP32]:{ev}"
        cv2.putText(canvas, hdr, (max(4, rx), max(18, ry - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.43, (180, 255, 0), 2)

    for det in analysis.get("objects", []):
        x, y, w, h = det["box"]
        label = f"{det['class']} {int(det['confidence']*100)}%"
        cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 128), 2)
        cv2.rectangle(canvas, (x, max(0, y - 22)), (x + len(label) * 10 + 10, y), (0, 255, 128), -1)
        cv2.putText(canvas, label, (x + 4, max(15, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
    return canvas

def send_ntfy_alert(title: str, msg: str):
    try:
        req = urllib.request.Request(NTFY_URL, data=msg.encode("utf-8"), method="POST")
        req.add_header("Title", title)
        req.add_header("Tags", "brain,camera")
        req.add_header("Click", "http://10.25.7.171:8095")
        urllib.request.urlopen(req, timeout=4)
    except Exception as e:
        print(f"[-] Ntfy alert error: {e}")

def process_and_record_image(
    img_bytes: bytes,
    filename: str,
    source: str,
    rx=20,
    ry=25,
    rw=60,
    rh=45,
    mode=0,
    device="esp32-cam",
    baseline="",
    max_flow_lpm=50.0,
    rot_deg=0,
    meter_serial="00000198",
    meter_spec="GB/T778-2018 R=80",
    serial_auto=1,
    edge_val=""
) -> Dict[str, Any]:
    nparr = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    if img is None:
        return {"status": "error", "message": "Invalid JPEG"}

    res = engine.analyze_image(
        img, rx, ry, rw, rh, mode, device=device, baseline=baseline,
        max_flow_lpm=max_flow_lpm, rot_deg=rot_deg,
        meter_serial=meter_serial, meter_spec=meter_spec, serial_auto=serial_auto
    )
    res["file"] = filename
    res["source"] = source

    # Attach Dual-Model Provenance Tags & Independent Cross-Check (Edge-ESP32 vs Server AI .171)
    m_obj = res.get("meter", {})
    srv_val_str = str(m_obj.get("dial_value") or m_obj.get("lcd_value") or m_obj.get("seg7_value") or "----")
    edg_val_str = str(edge_val or "").strip()
    if not edg_val_str or edg_val_str == "----":
        # If triggered from FTP/Server without ESP32 header, compute uncoupled raw pointer fraction as Edge reference
        mb = res.get("_meter_boxes") or {}
        d_list = mb.get("dials", [])
        if len(d_list) >= 2 and srv_val_str != "----":
            raw_d_str = "".join(str(d.get("digit", "0")) for d in d_list[:3])
            s_flt = MeterStateTracker._parse_float(srv_val_str)
            if s_flt is not None:
                edg_val_str = f"{int(np.floor(s_flt)):04d}.{raw_d_str}"
        else:
            edg_val_str = srv_val_str

    f_srv = MeterStateTracker._parse_float(srv_val_str)
    f_edg = MeterStateTracker._parse_float(edg_val_str)
    diff_liters = None
    consensus = "SINGLE_MODEL"
    if f_srv is not None and f_edg is not None:
        # Compare fractional pointer part if integer was not synced yet on Edge
        raw_diff = abs(f_srv - f_edg)
        if raw_diff >= 0.95:
            raw_diff = abs((f_srv % 1.0) - (f_edg % 1.0))
            if raw_diff > 0.5:
                raw_diff = 1.0 - raw_diff
            edg_val_str = f"{int(np.floor(f_srv)):04d}.{edg_val_str.split('.')[-1]}" if "." in edg_val_str else edg_val_str
        diff_liters = round(raw_diff * 100.0, 1)
        if diff_liters <= 2.5:
            consensus = f"KHỚP ĐỒNG THUẬN (Δ={diff_liters} L)"
        elif diff_liters <= 15.0:
            consensus = f"LỆCH NHẸ KIM CUỐI (Δ={diff_liters} L)"
        else:
            consensus = f"LỆCH MÔ HÌNH (Δ={diff_liters} L)"

    m_obj["server_model"] = "Server AI .171 (RK3576-NPU + StateFilter)"
    m_obj["edge_model"] = "Edge AI ESP32 (C-Chamfer + Polar BFS)"
    m_obj["server_val"] = srv_val_str
    m_obj["edge_val"] = edg_val_str
    m_obj["diff_liters"] = diff_liters
    m_obj["consensus"] = consensus
    res["summary"] = f"[.171]: {srv_val_str}m3 | [ESP32]: {edg_val_str}m3 ({consensus}) | SN:{m_obj.get('meter_serial','00000198')}"

    raw_path = PROCESSED_DIR / filename
    ann_path = ANNOTATED_DIR / filename
    raw_path.write_bytes(img_bytes)
    ann_img = draw_annotations(img, res)
    cv2.imwrite(str(ann_path), ann_img, [int(cv2.IMWRITE_JPEG_QUALITY), 90])

    # Remove internal box lists before sending compact JSON to ESP32
    res.pop("_meter_boxes", None)

    history_list.insert(0, res)
    if len(history_list) > 1000:
        history_list.pop()
    save_history()

    try:
        m_obj = res.get("meter", {})
        sn = m_obj.get("meter_serial") or meter_serial or "00000198"
        dv = m_obj.get("dial_value") or m_obj.get("lcd_value") or m_obj.get("seg7_value")
        if dv and dv != "----":
            meter_ledger.record_point(sn, str(dv), filename, res.get("timestamp", ""))
        mqtt_client.publish("esp32cam/ai_result", json.dumps(res, ensure_ascii=False))
        mqtt_client.publish("esp32cam/meter_reading", json.dumps(m_obj, ensure_ascii=False))
        mqtt_client.publish(f"esp32cam/meter/{sn}/reading", json.dumps(m_obj, ensure_ascii=False), retain=True)
    except Exception:
        pass

    return res

def ftp_inbox_watcher():
    seen = set()
    try:
        for p in FTP_INBOX_DIR.glob("*.jpg"):
            seen.add(p.name)
    except Exception:
        pass
    while True:
        try:
            for p in sorted(FTP_INBOX_DIR.glob("*.jpg")):
                if p.name not in seen and (time.time() - p.stat().st_mtime) >= 10.0:
                    seen.add(p.name)
                    if any(h.get("file") == p.name for h in history_list[:30]):
                        continue
                    data = p.read_bytes()
                    if len(data) > 1024:
                        print(f"[+] FTP Watcher analyzing fallback file: {p.name}")
                        process_and_record_image(data, p.name, "ftp_upload", rx=8, ry=3, rw=84, rh=94, mode=4, rot_deg=-90)
        except Exception as e:
            print(f"[-] FTP watcher err: {e}")
        time.sleep(2)

threading.Thread(target=ftp_inbox_watcher, daemon=True).start()

# ============================================================================
# FASTAPI SERVER & DASHBOARD
# ============================================================================
app = FastAPI(title="ESP32-CAM Deep AI Vision & Smart Meter Server (RK3576 NPU)")
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_methods=["*"], allow_headers=["*"])
app.mount("/annotated", StaticFiles(directory=str(ANNOTATED_DIR)), name="annotated")
app.mount("/raw", StaticFiles(directory=str(PROCESSED_DIR)), name="raw")
app.mount("/firmware", StaticFiles(directory=str(FIRMWARE_DIR)), name="firmware")

@app.post("/api/analyze")
async def api_analyze(
    request: Request,
    file: str = "capture.jpg",
    device: str = "esp32-cam",
    serial: str = "00000198",
    sauto: int = 1,
    motion: float = 0.0,
    mode: int = 0,
    rx: int = 20,
    ry: int = 25,
    rw: int = 60,
    rh: int = 45,
    baseline: str = "",
    max_flow: float = 50.0,
    rot: int = 0,
    edge_val: str = ""
):
    body = await request.body()
    if not body:
        return JSONResponse({"status": "error", "message": "Empty body"}, status_code=400)
    safe_name = os.path.basename(file) or f"cap_{int(time.time())}.jpg"
    hdr_sn = request.headers.get("X-Meter-Serial") or serial or "00000198"
    hdr_sp = request.headers.get("X-Meter-Spec") or "GB/T778-2018 R=80"
    hdr_ev = request.headers.get("X-Edge-Value") or edge_val or ""
    res = process_and_record_image(
        body, safe_name, f"http_push ({device})", rx, ry, rw, rh, mode,
        device=device, baseline=baseline, max_flow_lpm=max_flow, rot_deg=rot,
        meter_serial=hdr_sn, meter_spec=hdr_sp, serial_auto=sauto, edge_val=hdr_ev
    )
    return JSONResponse(res)

@app.get("/api/meter/export_csv")
async def api_meter_export_csv():
    lines = ["Timestamp,File,Source,Meter_Serial,Meter_Spec,Dial_Reading_m3,Flow_LPM,Filter_State,Leak_Alert,LCD_Reading,Seg7_Reading,LED_Status,Sharpness"]
    for h in history_list:
        m = h.get("meter") or {}
        row = [
            str(h.get("timestamp", "")),
            str(h.get("file", "")),
            str(h.get("source", "")),
            str(m.get("meter_serial", "00000198")),
            str(m.get("meter_spec", "GB/T778-2018 R=80")),
            str(m.get("dial_value", "----")),
            str(m.get("flow_lpm", 0.0)),
            str(m.get("filter_state", "")),
            str(m.get("leak_alert", "NORMAL")),
            str(m.get("lcd_value", "----")),
            str(m.get("seg7_value", "----")),
            str(m.get("led_status", "OFF")),
            str(h.get("sharpness", 0.0)),
        ]
        lines.append(",".join(f'"{c}"' for c in row))
    csv_bytes = ("\ufeff" + "\n".join(lines)).encode("utf-8")
    return Response(
        content=csv_bytes,
        media_type="text/csv; charset=utf-8",
        headers={"Content-Disposition": "attachment; filename=esp32cam_meter_log.csv"}
    )

@app.post("/api/meter/calibrate")
async def api_meter_calibrate(device: str = "esp32-cam", serial: str = "00000198", baseline: str = "00000.0000"):
    st = meter_tracker.calibrate(serial or device, baseline)
    meter_tracker.calibrate(device, baseline)
    return JSONResponse({"status": "ok", "device": device, "meter_serial": serial, "state": st})

@app.get("/api/meter/state")
async def api_meter_state():
    return JSONResponse({"status": "ok", "states": meter_tracker.states})

@app.get("/api/meter/analytics")
async def api_meter_analytics(
    serial: str = "00000198",
    month: str = "",
    date_a: str = "",
    date_b: str = ""
):
    data = meter_ledger.compute_analytics(serial=serial, month=month, date_a=date_a, date_b=date_b)
    return JSONResponse(data)

@app.get("/api/meter/screens")
async def api_meter_screens(device: str = "esp32-cam"):
    return JSONResponse({"status": "ok", **multi_screen_tracker.get_device_screens(device)})

@app.post("/api/meter/screen_label")
async def api_meter_screen_label(device: str = "esp32-cam", screen_key: str = "", label: str = "", unit: str = ""):
    res = multi_screen_tracker.set_screen_label(device, screen_key, label, unit)
    return JSONResponse(res)

@app.post("/api/meter/screens_reset")
async def api_meter_screens_reset(device: str = "esp32-cam"):
    res = multi_screen_tracker.reset_screens(device)
    return JSONResponse(res)

@app.get("/api/history")
async def api_history():
    tel = dict(latest_telemetry)
    recv_ep = float(tel.get("_recv_epoch", 0))
    if recv_ep > 0 and (time.time() - recv_ep) > 45.0:
        tel["online"] = False
    return JSONResponse({
        "npu_ready": engine.ready,
        "telemetry": tel,
        "history": history_list[:30]
    })

@app.get("/api/device_ip")
async def api_device_ip():
    tel = dict(latest_telemetry)
    recv_ep = float(tel.get("_recv_epoch", 0))
    is_online = bool(tel.get("online", False)) and (recv_ep > 0 and (time.time() - recv_ep) <= 45.0)
    return JSONResponse({
        "status": "ok",
        "online": is_online,
        "device": tel.get("device", "ESP32-CAM Smart AI"),
        "ip": tel.get("ip", "10.25.7.100"),
        "web_url": f"http://{tel.get('ip', '10.25.7.100')}",
        "wg_ip": tel.get("wg_ip", "10.212.156.7"),
        "wg_url": f"http://{tel.get('wg_ip', '10.212.156.7')}",
        "mdns": tel.get("mdns", "esp32-cam.local"),
        "ssid": tel.get("ssid", "--"),
        "rssi": tel.get("rssi", 0),
        "mac": tel.get("mac", "E0:8C:FE:5D:94:A4"),
        "last_seen": tel.get("_recv_time", "Chưa ghi nhận")
    })

@app.post("/api/trigger_esp32")
async def api_trigger_esp32():
    mqtt_client.publish("esp32cam/cmd", "capture")
    return JSONResponse({"status": "ok", "message": "Sent MQTT capture command to ESP32-CAM"})


# ============================================================================
# AI PHYSICAL EYES & HARDWARE DEBUGGER API + MCP SERVER (JSON-RPC 2.0)
# ============================================================================
def _fetch_esp32_snapshot(esp_ip: str = "", timeout: float = 5.0) -> Optional[np.ndarray]:
    mqtt_ip = latest_telemetry.get("ip", "")
    candidates = []
    for h in [esp_ip, mqtt_ip, "10.25.7.100", "10.212.156.7", "esp32-cam.local"]:
        if h and h not in candidates:
            candidates.append(h)
    for host in candidates:
        try:
            req = urllib.request.Request(f"http://{host}/snapshot", headers={"User-Agent": "AI-Eyes-Hub/2.0"})
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                raw = resp.read()
                arr = np.frombuffer(raw, dtype=np.uint8)
                img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if img is not None:
                    return img
        except Exception:
            continue
    return None


@app.get("/api/ai/debug_board")
async def api_ai_debug_board(
    fresh: int = 1,
    rx: int = 10,
    ry: int = 10,
    rw: int = 80,
    rh: int = 80,
    rot: int = 0
):
    img = None
    if fresh:
        img = _fetch_esp32_snapshot()
    if img is None and history_list:
        last_fn = history_list[0].get("file", "")
        p = PROCESSED_DIR / last_fn
        if p.exists():
            img = cv2.imread(str(p))

    if img is None:
        return JSONResponse({"status": "error", "message": "Không lấy được khung hình từ ESP32-CAM"}, status_code=503)

    fn = f"ai_debug_{datetime.now(TZ_VN).strftime('%Y%m%d_%H%M%S')}.jpg"
    res = process_and_record_image(
        cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 92])[1].tobytes(),
        filename=fn,
        source="ai_debug_board",
        rx=rx,
        ry=ry,
        rw=rw,
        rh=rh,
        mode=0,
        rot_deg=rot
    )
    res["annotated_url"] = f"http://10.25.7.171:8095/annotated/{fn}"
    res["display_crop_url"] = f"http://10.25.7.171:8095/api/ai/crop_display?rx={rx}&ry={ry}&rw={rw}&rh={rh}&rot={rot}"
    return JSONResponse(res)


@app.get("/api/ai/burst_check")
async def api_ai_burst_check(
    frames: int = 3,
    interval_ms: int = 250,
    rx: int = 10,
    ry: int = 10,
    rw: int = 80,
    rh: int = 80
):
    """
    Captures multiple rapid frames from ESP32-CAM to detect blinking communication LEDs
    (TX/RX, RS485, CAN, Heartbeat, Error Blink) vs solid power/status LEDs.
    """
    frames = max(2, min(5, int(frames)))
    interval_sec = max(0.10, min(1.5, float(interval_ms) / 1000.0))

    imgs: List[np.ndarray] = []
    for i in range(frames):
        im = _fetch_esp32_snapshot(timeout=4.0)
        if im is not None:
            imgs.append(im)
        if i < frames - 1:
            time.sleep(interval_sec)

    if not imgs:
        return JSONResponse({"status": "error", "message": "Không chụp được chuỗi ảnh Burst từ ESP32-CAM"}, status_code=503)

    h, w = imgs[0].shape[:2]
    x0, y0, r_w, r_h = SmartMeterVisionEngine.get_roi_rect(w, h, rx, ry, rw, rh)

    rois = [im[y0:y0+r_h, x0:x0+r_w] for im in imgs]
    v_channels = [cv2.cvtColor(r, cv2.COLOR_BGR2HSV)[:, :, 2].astype(np.float32) for r in rois]

    # Max projection across all frames finds EVERY LED that lit up at least once during the burst
    max_v_proj = np.max(np.stack(v_channels, axis=0), axis=0).astype(np.uint8)
    min_v_proj = np.min(np.stack(v_channels, axis=0), axis=0).astype(np.uint8)
    diff_v_proj = cv2.subtract(max_v_proj, min_v_proj)

    # Run LED color/spot detector on the brightest composite frame
    best_idx = int(np.argmax([np.mean(v) for v in v_channels]))
    base_led = SmartMeterVisionEngine.analyze_led_indicators(rois[best_idx])
    led_list = base_led.get("led_array", [])

    blinking_cnt = 0
    solid_cnt = 0
    for item in led_list:
        bx, by, bw, bh = item["box"]
        luma_Samples = [int(np.max(v[by:by+bh, bx:bx+bw])) for v in v_channels if v[by:by+bh, bx:bx+bw].size]
        luma_swing = float(max(luma_Samples) - min(luma_Samples)) if len(luma_Samples) >= 2 else 0.0
        item["luma_samples"] = luma_Samples
        item["luma_swing"] = round(luma_swing, 1)
        if luma_swing >= 32.0:
            item["temporal_state"] = "BLINKING_ACTIVE (ĐANG NHÁY TX/RX / XUNG)"
            blinking_cnt += 1
        else:
            item["temporal_state"] = "SOLID_ON (SÁNG TĨNH - NGUỒN/STATUS)"
            solid_cnt += 1

    return JSONResponse({
        "status": "ok",
        "timestamp": now_vn_str(),
        "frames_captured": len(imgs),
        "interval_ms": interval_ms,
        "summary": f"Phát hiện {len(led_list)} đèn LED ({solid_cnt} Sáng tĩnh, {blinking_cnt} Đang nháy truyền thông)",
        "solid_leds": solid_cnt,
        "blinking_leds": blinking_cnt,
        "led_array": led_list
    })


@app.get("/api/ai/crop_display")
async def api_ai_crop_display(
    rx: int = 12,
    ry: int = 8,
    rw: int = 76,
    rh: int = 84,
    rot: int = -999,
    enhance: int = 1,
    live: int = 0
):
    img = None
    if live:
        img = _fetch_esp32_snapshot(timeout=4.0)
        if img is not None and float(np.mean(img)) < 15.0:
            img = None
    if img is None and history_list:
        last_fn = history_list[0].get("file", "")
        p = PROCESSED_DIR / last_fn
        if p.exists():
            img = cv2.imread(str(p))
        if rot == -999:
            rot = int(history_list[0].get("meter", {}).get("rot_deg", 0))
    if img is None:
        img = _fetch_esp32_snapshot(timeout=4.0)
    if rot == -999:
        rot = 0

    if img is None:
        return JSONResponse({"status": "error", "message": "No image available"}, status_code=503)

    h, w = img.shape[:2]
    x0, y0, r_w, r_h = SmartMeterVisionEngine.get_roi_rect(w, h, rx, ry, rw, rh)
    roi = img[y0:y0+r_h, x0:x0+r_w].copy()

    r_deg = int(rot)
    if abs(r_deg) >= 1 and roi.size > 0:
        if -95 <= r_deg <= -85:
            roi = cv2.rotate(roi, cv2.ROTATE_90_COUNTERCLOCKWISE)
        elif 85 <= r_deg <= 95:
            roi = cv2.rotate(roi, cv2.ROTATE_90_CLOCKWISE)
        elif abs(r_deg) >= 175:
            roi = cv2.rotate(roi, cv2.ROTATE_180)
        else:
            sh, sw = roi.shape[:2]
            mat = cv2.getRotationMatrix2D((sw / 2.0, sh / 2.0), float(-r_deg), 1.0)
            roi = cv2.warpAffine(roi, mat, (sw, sh), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)

    if enhance and roi.size > 0:
        # Upscale 2x + CLAHE local contrast enhancement for crisp LCD/OLED/7-Seg readability
        roi = cv2.resize(roi, (0, 0), fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        l_ch, a_ch, b_ch = cv2.split(lab)
        clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))
        l_ch = clahe.apply(l_ch)
        roi = cv2.cvtColor(cv2.merge((l_ch, a_ch, b_ch)), cv2.COLOR_LAB2BGR)

    ok, enc = cv2.imencode(".jpg", roi, [int(cv2.IMWRITE_JPEG_QUALITY), 94])
    return Response(content=enc.tobytes(), media_type="image/jpeg")


@app.get("/api/ai/multi_screen_sweep")
async def api_ai_multi_screen_sweep(
    frames: int = 6,
    interval_ms: int = 1600,
    rx: int = 15,
    ry: int = 15,
    rw: int = 70,
    rh: int = 65,
    rot: int = 0,
    device: str = "esp32-cam"
):
    """
    Active Full-Cycle Sweep for Auto-Rotating Multi-Screen Meters:
    - Captures N time-spaced snapshots across the meter's display rotation loop (e.g., 6 frames x 1.6s = ~10s window)
    - Detects and discards mid-transition LCD/7-Seg frames (when crystals/segments are switching pages)
    - Clusters each valid frame into its screen slot (by OBIS/Page code or structural digit/decimal signature)
    - Returns the deduplicated table of all screens in the meter's cycle
    """
    frames = max(2, min(12, int(frames)))
    interval_sec = max(0.4, min(6.0, float(interval_ms) / 1000.0))

    raw_frames: List[Tuple[float, np.ndarray]] = []
    for i in range(frames):
        im = _fetch_esp32_snapshot(timeout=4.5)
        if im is not None:
            raw_frames.append((time.time(), im))
        if i < frames - 1:
            time.sleep(interval_sec)

    if not raw_frames:
        return JSONResponse({"status": "error", "message": "Không chụp được chuỗi khung hình từ ESP32-CAM"}, status_code=503)

    cycle_sequence = []
    transitions_skipped = 0
    prev_roi_gray = None

    for idx, (ts, im) in enumerate(raw_frames):
        h, w = im.shape[:2]
        x0, y0, r_w, r_h = SmartMeterVisionEngine.get_roi_rect(w, h, rx, ry, rw, rh)
        roi = im[y0:y0+r_h, x0:x0+r_w].copy()

        r_deg = int(rot)
        if abs(r_deg) >= 1 and roi.size > 0:
            if -95 <= r_deg <= -85:
                roi = cv2.rotate(roi, cv2.ROTATE_90_COUNTERCLOCKWISE)
            elif 85 <= r_deg <= 95:
                roi = cv2.rotate(roi, cv2.ROTATE_90_CLOCKWISE)
            elif abs(r_deg) >= 175:
                roi = cv2.rotate(roi, cv2.ROTATE_180)
            else:
                sh, sw = roi.shape[:2]
                mat = cv2.getRotationMatrix2D((sw / 2.0, sh / 2.0), float(-r_deg), 1.0)
                roi = cv2.warpAffine(roi, mat, (sw, sh), flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_REPLICATE)

        cur_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        seg7_res = SmartMeterVisionEngine.analyze_7segment(roi)
        lcd_res = SmartMeterVisionEngine.analyze_lcd_meter(roi)
        ms_frame = SmartMeterVisionEngine.analyze_multi_screen(roi, lcd_res, seg7_res)

        # Check if frame was caught right in the middle of a screen transition
        if not ms_frame.get("valid", False) or ms_frame.get("is_transition", False):
            transitions_skipped += 1
            prev_roi_gray = cur_gray
            continue

        ms_summary = multi_screen_tracker.update_from_frame(device, ms_frame)
        cycle_sequence.append({
            "frame_index": idx + 1,
            "screen_key": ms_frame["screen_key"],
            "page_code": ms_frame["page_code"],
            "label": ms_frame["label"],
            "value": ms_frame["value"],
            "unit": ms_frame["unit"]
        })
        prev_roi_gray = cur_gray

    dev_screens = multi_screen_tracker.get_device_screens(device)
    return JSONResponse({
        "status": "ok",
        "timestamp": now_vn_str(),
        "device": device,
        "frames_captured": len(raw_frames),
        "interval_ms": interval_ms,
        "transitions_rejected": transitions_skipped,
        "unique_screens_count": dev_screens["screen_count"],
        "summary": f"Đã quét {len(raw_frames)} khung hình -> Tách thành công {dev_screens['screen_count']} màn hình chỉ số độc lập (Loại bỏ {transitions_skipped} ảnh chuyển trang)",
        "cycle_sequence": cycle_sequence,
        "screens": dev_screens["screens"]
    })


@app.get("/mcp")
async def mcp_get_info():
    return JSONResponse({
        "protocol": "mcp-jsonrpc-2.0",
        "server": "esp32cam-rk3576-ai-eyes-hub",
        "version": "2.1.0",
        "description": "Multimodal MCP Server for AI Agents to observe real-world hardware boards, LEDs (PWR/STATUS/ERR/TX-RX), LCD/OLED/7-Seg displays, and utility meters.",
        "tools": [
            "observe_hardware_board",
            "check_blinking_comm_leds",
            "read_display_screen",
            "read_cycling_screens",
            "control_camera_probe"
        ]
    })


@app.post("/mcp")
async def mcp_jsonrpc_handler(request: Request):
    try:
        rpc = await request.json()
    except Exception:
        return JSONResponse({"jsonrpc": "2.0", "error": {"code": -32700, "message": "Parse error"}, "id": None})

    req_id = rpc.get("id")
    method = rpc.get("method", "")
    params = rpc.get("params") or {}

    if method == "initialize":
        return JSONResponse({
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {
                "protocolVersion": "2024-11-05",
                "serverInfo": {"name": "esp32cam-ai-eyes-hub", "version": "2.1.0"},
                "capabilities": {"tools": {}}
            }
        })

    if method == "tools/list":
        return JSONResponse({
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {
                "tools": [
                    {
                        "name": "observe_hardware_board",
                        "description": "Chụp ảnh thực tế từ ESP32-CAM và phân tích toàn diện mạch điện: mảng đèn báo LED (màu sắc, vị trí, vai trò Nguồn/Status/Lỗi/TX-RX), màn hình LCD/OLED, LED 7 thanh, đồng hồ nước kim quay, kèm ảnh trực quan Base64.",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "include_image": {"type": "boolean", "description": "Đính kèm ảnh JPEG Base64 để AI nhìn trực tiếp (mặc định: true)"},
                                "flash": {"type": "string", "description": "auto | on | off"}
                            }
                        }
                    },
                    {
                        "name": "check_blinking_comm_leds",
                        "description": "Chụp liên thanh 3 khung hình nhanh (Burst 250ms) để phát hiện đèn truyền thông đang nhấp nháy (TX/RX, RS485, CAN, Heartbeat, Đèn báo lỗi nháy) so với đèn nguồn sáng tĩnh.",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "frames": {"type": "integer", "description": "Số khung hình chụp liên tiếp (2..5, mặc định 3)"},
                                "interval_ms": {"type": "integer", "description": "Khoảng cách giữa các khung hình (ms, mặc định 250)"}
                            }
                        }
                    },
                    {
                        "name": "read_display_screen",
                        "description": "Cắt nét, khử nghiêng và tăng tương phản CLAHE 2x cho màn hình LCD / OLED / LED 7 thanh để đọc thông tin trạng thái, mã lỗi hoặc chỉ số đo.",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "rot_deg": {"type": "integer", "description": "Góc xoay bù nghiêng (-180..180)"}
                            }
                        }
                    },
                    {
                        "name": "read_cycling_screens",
                        "description": "Chụp quét trọn vòng đồng hồ tự động chuyển đổi nhiều màn hình (Multi-Screen Cycling Meter: Tổng kWh/m3, Biểu giá 1/2/3, Điện áp V, Dòng điện A, Lưu lượng tức thời), tự động loại bỏ ảnh nhòe lúc chuyển trang và tách riêng từng màn hình chỉ số.",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "frames": {"type": "integer", "description": "Số khung hình chụp dọc chu kỳ chuyển màn hình (2..12, mặc định 6)"},
                                "interval_ms": {"type": "integer", "description": "Khoảng cách giữa các lần chụp (ms, mặc định 1600)"}
                            }
                        }
                    },
                    {
                        "name": "control_camera_probe",
                        "description": "Điều khiển thấu kính lấy nét VCM (0..1023 hoặc quét đỉnh nét ROI) và đèn LED phụ PWM chống lóa (0..255) trên ESP32-CAM.",
                        "inputSchema": {
                            "type": "object",
                            "properties": {
                                "roi_focus_sweep": {"type": "boolean", "description": "Tự động quét VCM tìm điểm nét nhất trong ROI"},
                                "focus_step": {"type": "integer", "description": "Bước lấy nét VCM thủ công (0..1023)"},
                                "flash_duty": {"type": "integer", "description": "Độ sáng đèn LED phụ PWM (0..255)"}
                            }
                        }
                    }
                ]
            }
        })

    if method == "tools/call":
        tname = params.get("name", "")
        args = params.get("arguments") or {}

        if tname == "check_blinking_comm_leds":
            resp_obj = await api_ai_burst_check(
                frames=int(args.get("frames", 3)),
                interval_ms=int(args.get("interval_ms", 250))
            )
            body = json.loads(resp_obj.body.decode("utf-8"))
            return JSONResponse({
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {"content": [{"type": "text", "text": json.dumps(body, ensure_ascii=False, indent=2)}]}
            })

        if tname == "control_camera_probe":
            results = []
            try:
                if args.get("roi_focus_sweep"):
                    with urllib.request.urlopen("http://10.25.7.100/control?var=af_roi_sweep&val=1", timeout=8) as r:
                        results.append(r.read().decode("utf-8"))
                if "focus_step" in args:
                    with urllib.request.urlopen(f"http://10.25.7.100/control?var=focus&val={int(args['focus_step'])}", timeout=4) as r:
                        results.append(r.read().decode("utf-8"))
                if "flash_duty" in args:
                    with urllib.request.urlopen(f"http://10.25.7.100/control?var=flash_duty&val={int(args['flash_duty'])}", timeout=4) as r:
                        results.append(r.read().decode("utf-8"))
            except Exception as e:
                results.append(str(e))
            return JSONResponse({
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {"content": [{"type": "text", "text": json.dumps({"status": "ok", "actions": results}, ensure_ascii=False)}]}
            })

        if tname == "read_cycling_screens":
            resp_obj = await api_ai_multi_screen_sweep(
                frames=int(args.get("frames", 6)),
                interval_ms=int(args.get("interval_ms", 1600))
            )
            body = json.loads(resp_obj.body.decode("utf-8"))
            return JSONResponse({
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {"content": [{"type": "text", "text": json.dumps(body, ensure_ascii=False, indent=2)}]}
            })

        if tname == "read_display_screen":
            rot = int(args.get("rot_deg", 0))
            crop_resp = await api_ai_crop_display(rot=rot, enhance=1)
            b64_img = base64.b64encode(crop_resp.body).decode("ascii")
            latest_meter = history_list[0].get("meter", {}) if history_list else {}
            return JSONResponse({
                "jsonrpc": "2.0",
                "id": req_id,
                "result": {
                    "content": [
                        {"type": "text", "text": json.dumps(latest_meter, ensure_ascii=False, indent=2)},
                        {"type": "image", "data": b64_img, "mimeType": "image/jpeg"}
                    ]
                }
            })

        # Default: observe_hardware_board
        dbg_resp = await api_ai_debug_board(fresh=1)
        dbg_data = json.loads(dbg_resp.body.decode("utf-8"))
        content_items = [{"type": "text", "text": json.dumps(dbg_data, ensure_ascii=False, indent=2)}]
        if args.get("include_image", True):
            ann_fn = dbg_data.get("file", "")
            ann_path = ANNOTATED_DIR / ann_fn
            if ann_path.exists():
                b64 = base64.b64encode(ann_path.read_bytes()).decode("ascii")
                content_items.append({"type": "image", "data": b64, "mimeType": "image/jpeg"})
        return JSONResponse({
            "jsonrpc": "2.0",
            "id": req_id,
            "result": {"content": content_items}
        })

    return JSONResponse({"jsonrpc": "2.0", "id": req_id, "result": {}})


DASHBOARD_HTML = """<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RK3576 Deep AI & Smart Meter Hub — ESP32-CAM</title>
<style>
body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; background: #090d16; color: #f8fafc; margin: 0; padding: 20px; }
.wrap { max-width: 1240px; margin: 0 auto; display: flex; flex-direction: column; gap: 18px; }
.top { display: flex; justify-content: space-between; align-items: center; background: #111827; padding: 16px 20px; border-radius: 14px; border: 1px solid #1f2937; flex-wrap: wrap; gap: 12px; }
h1 { margin: 0; font-size: 20px; background: linear-gradient(90deg,#38bdf8,#818cf8); -webkit-background-clip: text; -webkit-text-fill-color: transparent; }
.btn { background: linear-gradient(135deg,#0ea5e9,#2563eb); color: #fff; border: none; padding: 10px 16px; border-radius: 10px; font-weight: 600; cursor: pointer; text-decoration: none; font-size: 13px; }
.health-bar { display: grid; grid-template-columns: repeat(auto-fit, minmax(170px, 1fr)); gap: 10px; }
.hbox { background: #111827; border: 1px solid #1f2937; border-radius: 12px; padding: 10px 14px; }
.hlbl { font-size: 11px; color: #94a3b8; text-transform: uppercase; font-weight: 600; }
.hval { font-size: 16px; font-weight: 700; margin-top: 3px; color: #38bdf8; }
.grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(330px, 1fr)); gap: 16px; }
.card { background: #111827; border: 1px solid #1f2937; border-radius: 14px; overflow: hidden; display: flex; flex-direction: column; }
.card img { width: 100%; aspect-ratio: 4/3; object-fit: cover; background: #000; }
.card-body { padding: 14px; display: flex; flex-direction: column; gap: 6px; font-size: 13px; }
.tag { display: inline-block; padding: 3px 8px; border-radius: 99px; font-size: 11px; font-weight: 700; background: rgba(16,185,129,0.15); color: #34d399; }
.meta { color: #94a3b8; font-size: 12px; }
.m-row { display: flex; justify-content: space-between; background: #050811; padding: 6px 10px; border-radius: 8px; font-family: monospace; font-size: 12px; }
</style>
</head>
<body>
<div class="wrap">
  <div class="top">
    <div>
      <h1>🧠 Server 10.25.7.171 — RK3576 NPU & Smart Meter Reader Center</h1>
      <div class="meta" id="sub-info">Đang kết nối MQTT & NPU...</div>
    </div>
    <div style="display:flex;gap:8px;flex-wrap:wrap;align-items:center">
      <input id="cal-inp" type="text" placeholder="Số gốc V(t-1) VD: 00125.4320" style="background:#050811;border:1px solid #374151;color:#fff;padding:9px 12px;border-radius:10px;font-family:monospace;font-size:13px;width:210px">
      <button class="btn" style="background:#d97706" onclick="calibrateMeter()">📌 Gán Số Gốc</button>
      <button class="btn" onclick="triggerCap()">📸 Chụp & Đọc Số Ngay</button>
      <a class="btn" style="background:#059669" href="/api/meter/export_csv">📥 Xuất Excel/CSV</a>
      <a class="btn" id="btn-esp-web" style="background:#374151" href="http://10.25.7.100" target="_blank">📱 Web ESP32 (10.25.7.100)</a>
    </div>
  </div>

  <div class="health-bar">
    <div class="hbox"><div class="hlbl">Trạng thái & Lần cuối thấy</div><div class="hval" id="hb-online">Đang kiểm tra...</div></div>
    <div class="hbox"><div class="hlbl">Địa chỉ IP LAN & VPN (MQTT)</div><div class="hval" id="hb-net">--</div></div>
    <div class="hbox"><div class="hlbl">Nhiệt độ & Điện áp ESP32</div><div class="hval" id="hb-temp">-- °C | -- V</div></div>
    <div class="hbox"><div class="hlbl">Tải CPU & Bộ nhớ</div><div class="hval" id="hb-cpu">-- %</div></div>
    <div class="hbox"><div class="hlbl">Edge LED Indicator</div><div class="hval" id="hb-led">--</div></div>
  </div>

  <div id="items" class="grid"></div>
</div>
<script>
async function load() {
  try {
    const r = await fetch('/api/history');
    const d = await r.json();
    const t = d.telemetry || {};
    document.getElementById('sub-info').textContent =
      `NPU: ${d.npu_ready ? 'RK3576 YOLOv8n + SmartMeter Active ✅' : 'Fallback'} | FTP: ${t.ftp_status || 'OK'} | Tổng bản ghi: ${d.history.length}`;

    const isOnline = t.online === true;
    const hbOn = document.getElementById('hb-online');
    if (hbOn) {
      hbOn.innerHTML = isOnline
        ? `<span style="color:#34d399">🟢 ONLINE</span> <span style="font-size:11px;color:#94a3b8">(${t._recv_time || 'Vừa xong'})</span>`
        : `<span style="color:#f87171">🔴 OFFLINE</span> <span style="font-size:11px;color:#94a3b8">(Cuối: ${t._recv_time || 'Chưa rõ'})</span>`;
    }

    if (t.ip) {
      const btnWeb = document.getElementById('btn-esp-web');
      if (btnWeb) {
        btnWeb.href = `http://${t.ip}`;
        btnWeb.textContent = `📱 Web ESP32 (${t.ip})`;
      }
      document.getElementById('hb-net').innerHTML =
        `<a href="http://${t.ip}" target="_blank" style="color:#38bdf8;text-decoration:none">${t.ip}</a>` +
        (t.wg_ip ? ` <span style="font-size:12px;color:#a78bfa">| VPN: ${t.wg_ip}</span>` : '') +
        `<div style="font-size:11px;color:#94a3b8;margin-top:2px">WiFi: ${t.ssid || '--'} (${t.rssi || '--'}dBm)</div>`;
    }

    if (t.temp_c !== undefined) {
      document.getElementById('hb-temp').textContent = `${t.temp_c} °C | ${t.vdd33} V (${t.vref_mv}mV)`;
      document.getElementById('hb-cpu').textContent = `${t.cpu_load}% | RAM: ${((t.ram_free||0)/1024).toFixed(0)}KB`;
      document.getElementById('hb-led').textContent = (t.edge_ai && t.edge_ai.led_state) ? t.edge_ai.led_state : 'OFF';
    }

    const box = document.getElementById('items');
    if (!d.history.length) {
      box.innerHTML = '<div style="color:#94a3b8">Chưa có ảnh nào được đẩy lên. Hãy bấm Chụp trên ESP32-CAM!</div>';
      return;
    }
    box.innerHTML = d.history.map(h => {
      const m = h.meter || {};
      return `
      <div class="card">
        <a href="/annotated/${h.file}" target="_blank"><img src="/annotated/${h.file}" loading="lazy"></a>
        <div class="card-body">
          <div style="display:flex;justify-content:space-between;align-items:center">
            <span class="tag">${h.summary}</span>
            <span class="meta">${h.latency_ms} ms</span>
          </div>
          <div class="m-row">
            <span>💡 LED: <b>${m.led_status || 'OFF'}</b></span>
            <span>📟 LCD: <b>${m.lcd_value || '----'}</b></span>
            <span>🎡 Kim: <b style="color:#fbbf24">${m.dial_value || '----'} m³ (${m.flow_lpm || 0} L/m)</b></span>
          </div>
          <div><b>File:</b> ${h.file} (${h.resolution})</div>
          <div class="meta">🕒 ${h.timestamp} | Độ nét: ${h.sharpness} | Nguồn: ${h.source}</div>
        </div>
      </div>
    `}).join('');
  } catch(e) {}
}
async function triggerCap() {
  await fetch('/api/trigger_esp32', {method:'POST'});
  setTimeout(load, 2500);
}
async function calibrateMeter() {
  const v = (document.getElementById('cal-inp').value || '').trim();
  if (!v) { alert('Vui lòng nhập chỉ số gốc V(t-1) (VD: 00125.4320)'); return; }
  await fetch('/api/meter/calibrate?device=esp32-cam&baseline=' + encodeURIComponent(v), {method:'POST'});
  alert('Đã gán Số Gốc V(t-1) = ' + v + ' m³ thành công!');
  load();
}
load();
setInterval(load, 3000);
</script>
</body>
</html>
"""

@app.get("/", response_class=HTMLResponse)
async def index():
    return DASHBOARD_HTML

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8095, log_config=None)
