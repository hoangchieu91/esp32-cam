# AGENTS.md — Bối cảnh Dự án ESP32-CAM Smart Meter & Edge-to-Server AI Vision

> **Tài liệu Kỹ thuật Đầy đủ cho AI**: Đọc [`docs/PROJECT_DOCS_AI.md`](./docs/PROJECT_DOCS_AI.md)  
> **Cổng Tài liệu Trực quan cho Người dùng**: Mở [`docs/index.html`](./docs/index.html)

## 1. Thông tin Hạ tầng & Kết nối Thực tế
- **Mạch ESP32-CAM**:
  - IP LAN: `http://10.25.7.100` (`http://esp32-cam.local`)
  - IP WireGuard VPN: `http://10.212.156.7` (Peer: `nxchieu.duckdns.org:51820`, `MTU 1360`)
  - Phần cứng: `ESP32-D0WD-V3` + `4MB Flash` + `8MB PSRAM` + Camera `OV5640 5MP (VCM Autofocus)` + `GPIO 4` LEDC PWM Flash + Thẻ nhớ `16GB MicroSD (1-bit SDMMC)`.
- **Biên dịch & Nạp Firmware Trực tiếp từ Máy Chính Windows (Không cần máy trung gian T490s)**:
  - Môi trường ESP-IDF cục bộ trên Windows: `C:\Users\nguye\esp\v5.5.1\esp-idf\export.ps1` (`ESP-IDF v5.5.1`).
  - Lệnh chuẩn 1 bước (Đóng gói Web UI + Build Local + Đẩy HTTP OTA trực tiếp qua LAN/VPN + Đồng bộ Server `.171`):
    ```powershell
    python tools/deploy.py
    ```
  - Nạp OTA trực tiếp qua HTTP API: `POST http://10.25.7.100/api/ota/upload` (hoặc qua VPN `http://10.212.156.7/api/ota/upload`) và Kéo OTA từ Server `.171`: `POST http://10.25.7.100/api/ota/pull`.
  - *(Dự phòng vật lý)* Máy trạm Linux `100.126.145.106` (`nxchieu` / `DungCliff@1997`, `/dev/ttyUSB0`) chỉ dùng khi cần nạp cứu hộ qua cáp UART cứng.
- **Máy chủ Deep AI, FTP & MQTT (`10.25.7.171`)**:
  - SSH: `root` / `123`
  - Dịch vụ AI Hub (RK3576 NPU): `http://10.25.7.171:8095` (`/opt/esp32cam_ai_server/esp32cam_ai_server.py`, systemd `esp32cam-ai.service`).
  - FTP Server: `10.25.7.171:21` (`nxchieu` / `123`, thư mục `/var/ftp/esp32cam`).
  - MQTT Broker: `mqtt://10.25.7.171:1883` (`esp32cam/#`).
- **Máy chủ Thông báo Ntfy (`10.25.7.116`)**:
  - Topic URL: `http://10.25.7.116:8092/esp32cam`

## 2. Nguyên tắc Bắt buộc khi Sửa đổi Firmware (`cam_streamer`)
1. **Bảo toàn Phân vùng Bất tử `vpn_vault @ 0x3FC000`**:
   - Tuyệt đối không xóa hoặc ghi đè vùng nhớ `0x3FC000` (`16 KB`, SubType `0x99`) trong `partitions.csv`. Đây là nơi lưu khóa WireGuard VPN và 5 WiFi theo cơ chế Dual-Sector A/B.
2. **Bảo vệ Stack & Internal SRAM**:
   - Không cấp phát mảng hoặc struct lớn (`> 512 bytes`) trên Stack của `main` hoặc `httpd` task. Luôn dùng `calloc`/`malloc` và `free` (hoặc `MALLOC_CAP_SPIRAM` cho bộ đệm hình ảnh).
3. **Quy trình Sửa Giao diện Web (`ui_src/style.css`, `body.html`, `app.js`)**:
   - Giao diện Frontend được chia thành 3 file mô-đun tại `cam_streamer/main/ui_src/`: `style.css`, `body.html`, và `app.js` (tự động đồng bộ hai chiều với `index.html`). Sau khi sửa, chạy `python cam_streamer/main/ui_src/pack_ui.py` (hoặc `python tools/deploy.py`) để sinh lại `cam_streamer/main/web_ui.h` (nén Gzip). Không sửa trực tiếp `web_ui.h`.
4. **Đèn phụ `GPIO 4`, Thẻ nhớ MicroSD & Chân I/O Mở rộng (`GPIO 33, 12, 13`)**:
   - Thẻ nhớ bắt buộc chạy ở chế độ **1-bit SDMMC** (`GPIO 14, 15, 2`) để giải phóng chân `GPIO 4` cho bộ băm xung PWM `LEDC_CHANNEL_1` (chống lóa mặt kính đồng hồ nước) và giải phóng `GPIO 12` (Relay Output), `GPIO 13` (Pulse/Tamper Interrupt Input), `GPIO 33` (Đèn LED đỏ báo nhịp tim mặt sau).

## 3. Tai Mắt Vật Lý Cho AI & Gỡ Lỗi Phần Cứng Tự Động (Zero-Prompt AI Eyes & MCP)
Khi cần quan sát mạch điện thực tế, kiểm tra đèn báo (`PWR`, `RUN`, `ERR`, đèn nháy truyền thông `TX/RX / RS485`), đọc màn hình `LCD / OLED / LED 7 thanh`, hoặc đọc **đồng hồ đa màn hình chuyển vòng tự động (Multi-Screen Cycling Meter)**, mọi AI Agent sử dụng trực tiếp các cổng sau mà không cần người dùng chụp ảnh thủ công:
- **Zero-Prompt Self-Describing Manifest**: `GET http://10.25.7.100/api/ai/manifest` (hoặc qua VPN `http://10.212.156.7/api/ai/manifest`)
- **Quan Sát & Phân Tích Mạch 1-Lệnh (`1-Call Observe`)**: `GET http://10.25.7.100/api/ai/observe?flash=auto&focus=keep&mode=debug_board`
- **Quét Trọn Vòng Lặp Đồng Hồ Đa Màn Hình (`Multi-Screen Sweep`)**: `GET http://10.25.7.171:8095/api/ai/multi_screen_sweep?frames=6&interval_ms=1600` (và xem các khe đã tách tại `GET http://10.25.7.171:8095/api/meter/screens`)
- **Bắt Nhịp Nháy Đèn Truyền Thông (`3-Frame Burst TX/RX Detector`)**: `GET http://10.25.7.171:8095/api/ai/burst_check?frames=3&interval_ms=250`
- **Ảnh Cắt Nét Màn Hình LCD/OLED (`2x CLAHE Contrast`)**: `GET http://10.25.7.171:8095/api/ai/crop_display`
- **MCP Server Đa Phương Thức (`JSON-RPC 2.0` trả cả JSON + Ảnh Base64)**:
  - Trên Server AI (Đầy đủ Vision + Ảnh Base64, gồm 5 tools: `observe_hardware_board`, `check_blinking_comm_leds`, `read_display_screen`, `read_cycling_screens`, `control_camera_probe`): `POST http://10.25.7.171:8095/mcp`
  - Trực tiếp trên mạch ESP32-CAM: `POST http://10.25.7.100/mcp` (hoặc `http://10.212.156.7/mcp`)
