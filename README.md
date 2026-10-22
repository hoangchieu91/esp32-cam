# ESP32-CAM Industrial Smart Meter & Edge-to-Server AI Vision System

Hệ thống Camera AI Công nghiệp chuyên dụng đọc **Đồng hồ Nước Kim quay (Multi-Dial Rotary Water Meter)**, **Màn hình LCD (Điện/Nước/Khí)**, **Đồng hồ Số LED 7 Thanh** và **Đèn báo Trạng thái**, kết hợp xử lý tại biên (**Edge AI trên ESP32-CAM**) và phân tích chuyên sâu kèm lọc sai số lịch sử $V(t-1)$ trên máy chủ **NPU RK3576 (`10.25.7.171:8095`)**.

---

## 📚 Tài liệu Dự án (`docs/`)

Toàn bộ tài liệu kiến trúc, nghiệp vụ thuật toán và quy trình kết nối được đóng gói trong thư mục [`docs/`](./docs/):

1. 🤖 **Dành cho AI Agent & Kỹ sư Phát triển (Markdown)**:  
   👉 [**`docs/PROJECT_DOCS_AI.md`**](./docs/PROJECT_DOCS_AI.md)  
   *(Chi tiết sơ đồ phân vùng Flash 4MB, két sắt `vpn_vault @ 0x3FC000`, thuật toán khớp nối bánh răng kim quay, bộ lọc trạng thái lịch sử `MeterStateTracker`, điều khiển PWM chống lóa `GPIO 4` và danh mục REST API / MQTT).*
2. 👨‍💻 **Dành cho Người đọc & Vận hành (Interactive HTML Handbook)**:  
   👉 [**`docs/index.html`**](./docs/index.html)  
   *(Cẩm nang kỹ thuật trực quan Tiếng Việt có sơ đồ kiến trúc, bảng tra cứu chế độ đọc đồng hồ, quy trình căn chuẩn số gốc $V(t-1)$ và hướng dẫn vận hành).*
3. 🧭 **Bối cảnh Nhanh cho AI (`AGENTS.md`)**:  
   👉 [**`AGENTS.md`**](./AGENTS.md)

---

## 🚀 Tóm tắt Năng lực Cốt lõi

- **Quang học OV5640 AF & Đèn phụ PWM Chống lóa (`GPIO 4`)**:
  - Điều khiển tiêu cự VCM (`0..1023`) lấy nét Macro mặt đồng hồ, tự động ngắt dòng cuộn dây VCM và đưa cảm biến vào chế độ ngủ **Thermal Eco Standby (`<1mA`)** khi nhàn rỗi.
  - Đèn LED phụ `GPIO 4` điều khiển bằng **PWM 5kHz 8-bit** (mặc định `22%` công suất chống lóa kính đồng hồ), tự động bật trước `350ms` + xả 1 frame DMA để cân bằng sáng AEC/AWB trước khi chụp.
- **Đọc Đồng hồ Nước Kim quay & Lọc Sai số Lịch sử $V(t-1)$**:
  - Giải mã góc quay $\theta \in [0^\circ, 360^\circ)$ của 4 kim đỏ ($\times 0.1, \times 0.01, \times 0.001, \times 0.0001\text{ m}^3$) kết hợp **Khớp nối Bánh răng Liên cấp (Inter-Dial Gear Carry Coupling)** khử lỗi kim nằm lửng.
  - Bộ lọc trạng thái **`MeterStateTracker`** kiểm tra tính đơn điệu vật lý ($V_t \ge V_{t-1}$) và giới hạn lưu lượng tối đa ($\Delta V \le Q_{\max} \cdot \Delta t$) để loại bỏ hoàn toàn các lần đọc nhầm.
- **Nhận diện Đồng hồ Đa Màn Hình Tự Động Chuyển Vòng (`Mode 5` & `MultiScreenCycleTracker`)**:
  - Tách riêng **Mã trang / OBIS (`1.8.0`, `1.8.1`, `01`, `02`...)** và **Giá trị đo chính** bằng Dual-Zone OCR, kết hợp bộ lọc khử nhòe/bóng ma (`is_transition`) khi màn hình tinh thể lỏng đang lật trang.
  - Phân nhóm tự động tối đa **12 khe màn hình độc lập (`Màn #1..#12`)**, chỉ áp dụng khóa đơn điệu $V(t) \ge V(t-1)$ cho các khe lũy kế (kWh, $m^3$) và cho phép biến thiên tự do với các khe tức thời (`V`, `A`, `Hz`, `m³/h`, `bar`).
- **Mở rộng Ngoại vi Phần cứng (`GPIO 33, 12, 13`) & Giao diện Di động Chuẩn iPhone 12**:
  - Tận dụng các chân giải phóng từ chế độ 1-bit SDMMC: `GPIO 33` (Đèn LED đỏ báo trạng thái/nhịp tim mặt sau), `GPIO 12` (Ngõ ra điều khiển Rơ-le/Van khóa nước), `GPIO 13` (Ngõ vào đếm xung đồng hồ hoặc công tắc từ chống cạy vỏ).
- **Chu kỳ Đọc Linh hoạt & Thích ứng Lưu lượng (Adaptive Cadence)**:
  - Cấu hình mềm từ `15 giây` đến `24 giờ`; tự động rút ngắn chu kỳ đọc xuống `30 giây/lần` khi phát hiện kim đồng hồ đang quay và trở về chu kỳ tiết kiệm khi khóa nước.
- **Phân vùng Bất tử `vpn_vault @ 0x3FC000` (4-Tier Persistence)**:
  - Lưu trữ cấu hình **WireGuard VPN (`10.212.156.7`)** và **5 mạng WiFi** trên phân vùng Flash độc lập Dual-Sector A/B + NVS `wg_vault` + sao lưu `/sdcard/wg_vault.conf`. Miễn nhiễm hoàn toàn với OTA, `idf.py flash` và Factory Reset.
- **Trạm Tai Mắt Thực Tế & Gỡ Lỗi Phần Cứng Cho AI (`Zero-Prompt AI Eyes & MCP`)**:
  - Cung cấp Manifest tự mô tả (`/api/ai/manifest`), API quan sát mạch 1 lệnh (`/api/ai/observe`), quét trọn vòng lặp đa màn hình (`/api/ai/multi_screen_sweep`), bắt nhịp nháy đèn truyền thông TX/RX (`/api/ai/burst_check`), cắt nét màn hình LCD/OLED (`/api/ai/crop_display`) và MCP Server JSON-RPC 2.0 (`/mcp`).

---

## 🛠️ Quy trình Phát triển & Nạp OTA Không Dây (Không cần máy trung gian T490s)

Toàn bộ quá trình đóng gói giao diện Web, biên dịch `ESP-IDF v5.5.1` và nạp Firmware qua mạng (Direct HTTP OTA + Server Pull `.171`) được thực hiện trực tiếp ngay trên máy tính Windows hiện tại bằng 1 lệnh duy nhất:

```powershell
# Đóng gói Web UI + Biên dịch ESP-IDF cục bộ + Đẩy OTA lên ESP32-CAM + Đồng bộ AI Server (.171)
python tools/deploy.py

# Các tùy chọn nhanh:
python tools/deploy.py --ui-only       # Chỉ đóng gói index.html -> web_ui.h (Gzip)
python tools/deploy.py --build-only    # Chỉ biên dịch kiểm tra lỗi C/ESP-IDF cục bộ
python tools/deploy.py --ai-only       # Chỉ cập nhật mã nguồn AI Server lên 10.25.7.171
```
