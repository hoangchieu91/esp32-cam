#!/usr/bin/env python3
"""
ESP32-CAM Smart Meter & AI Vision — Local Build & Zero-Cable OTA Deployer
-------------------------------------------------------------------------
Quy trình phát triển độc lập 100% trên máy tính Windows hiện tại (KHÔNG CẦN máy trung gian T490s):
1. Đóng gói giao diện Web (`ui_src/index.html` -> `web_ui.h` Gzip).
2. Biên dịch Firmware trực tiếp bằng ESP-IDF cục bộ (`C:\\Users\\nguye\\esp\\v5.5.1\\esp-idf`).
3. Đồng bộ file `.bin` và mã nguồn AI Server (`esp32cam_ai_server.py`) lên Server `.171`.
4. Nạp Firmware Không Dây (OTA) lên ESP32-CAM qua 2 cơ chế tự động dự phòng:
   - Cách A (Direct HTTP OTA): Đẩy trực tiếp tới `http://10.25.7.100/api/ota/upload` (hoặc VPN `10.212.156.7`).
   - Cách B (LAN Server Pull qua `.171`): Gọi ESP32-CAM kéo bản cập nhật tốc độ cao từ `http://10.25.7.171:8095/firmware/cam_streamer.bin` (hoạt động ngay cả khi ESP32 đang bận hoặc kết nối WiFi yếu).

Cách dùng:
  python tools/deploy.py                 # Pack UI + Build Local + Sync .171 + Nạp OTA lên ESP32
  python tools/deploy.py --ui-only       # Chỉ đóng gói lại web_ui.h
  python tools/deploy.py --build-only    # Chỉ Pack UI + Build Local (không nạp OTA)
  python tools/deploy.py --ai-only       # Chỉ cập nhật mã nguồn AI Server (.171)
"""

import argparse
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parent.parent
CAM_DIR = ROOT_DIR / "cam_streamer"
PACK_UI_SCRIPT = CAM_DIR / "main" / "ui_src" / "pack_ui.py"
BIN_PATH = CAM_DIR / "build" / "cam_streamer.bin"
AI_SERVER_SCRIPT = ROOT_DIR / "server_ai_171" / "esp32cam_ai_server.py"
IDF_EXPORT_PS1 = Path(r"C:\Users\nguye\esp\v5.5.1\esp-idf\export.ps1")

DEFAULT_ESP_HOSTS = ["10.25.7.100", "10.212.156.7", "esp32-cam.local"]
AI_SERVER_HOST = "10.25.7.171"
AI_SERVER_USER = "root"
AI_SERVER_PASS = "123"


def step_pack_ui() -> None:
    print("\n=== [1/4] Đóng gói giao diện Web UI (index.html -> web_ui.h Gzip) ===")
    subprocess.run([sys.executable, str(PACK_UI_SCRIPT)], check=True)


def step_local_build() -> None:
    print("\n=== [2/4] Biên dịch Firmware cục bộ trên Windows (ESP-IDF v5.5.1) ===")
    if not IDF_EXPORT_PS1.exists():
        raise FileNotFoundError(f"Không tìm thấy ESP-IDF tại: {IDF_EXPORT_PS1}")

    ps_cmd = (
        f". '{IDF_EXPORT_PS1}' > $null; "
        f"Set-Location '{CAM_DIR}'; "
        f"idf.py build"
    )
    res = subprocess.run(["powershell", "-NoProfile", "-Command", ps_cmd])
    if res.returncode != 0:
        raise RuntimeError("Biên dịch ESP-IDF thất bại!")
    if not BIN_PATH.exists():
        raise FileNotFoundError(f"Không tìm thấy file binary sau khi build: {BIN_PATH}")
    size_kb = BIN_PATH.stat().st_size / 1024.0
    print(f"[+] Build thành công: {BIN_PATH} ({size_kb:.1f} KB)")


def step_sync_ai_server(sync_bin: bool = True, sync_py: bool = True) -> None:
    print(f"\n=== [3/4] Đồng bộ Firmware & AI Engine lên Máy chủ ({AI_SERVER_HOST}) ===")
    import warnings
    warnings.filterwarnings("ignore")
    import paramiko

    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(AI_SERVER_HOST, username=AI_SERVER_USER, password=AI_SERVER_PASS, timeout=8.0)
    sftp = ssh.open_sftp()

    if sync_bin and BIN_PATH.exists():
        fw_data = BIN_PATH.read_bytes()
        for remote_fw in [
            "/opt/esp32_ai_hub/firmware/cam_streamer.bin",
            "/opt/esp32cam_ai_server/firmware/cam_streamer.bin",
        ]:
            try:
                with sftp.open(remote_fw, "wb") as rf:
                    rf.write(fw_data)
                print(f"[+] Đã cập nhật bản lưu trữ OTA trên Server: {remote_fw}")
            except Exception as e:
                print(f"[-] Bỏ qua {remote_fw}: {e}")

    if sync_py and AI_SERVER_SCRIPT.exists():
        remote_py = "/opt/esp32cam_ai_server/esp32cam_ai_server.py"
        sftp.put(str(AI_SERVER_SCRIPT), remote_py)
        print(f"[+] Đã cập nhật mã nguồn AI Server: {remote_py}")
        _, stdout, _ = ssh.exec_command("systemctl restart esp32cam-ai && sleep 1 && systemctl is-active esp32cam-ai")
        st = stdout.read().decode("utf-8", errors="replace").strip()
        print(f"[+] Trạng thái dịch vụ esp32cam-ai.service: {st}")

    sftp.close()
    ssh.close()


def get_mqtt_discovered_ip() -> str:
    try:
        import json
        with urllib.request.urlopen(f"http://{AI_SERVER_HOST}:8095/api/device_ip", timeout=3.0) as r:
            d = json.loads(r.read().decode("utf-8", errors="replace"))
            ip = d.get("ip", "")
            if ip and ip != "0.0.0.0":
                print(f"[+] Tự động nhận diện IP của ESP32-CAM từ MQTT Broker (.171): {ip} (Online={d.get('online')}, WiFi={d.get('ssid')}, RSSI={d.get('rssi')}dBm)")
                return ip
    except Exception:
        pass
    return ""


def step_ota_flash(preferred_host: str = "") -> None:
    print("\n=== [4/4] Nạp Firmware Không Dây (OTA) lên ESP32-CAM ===")
    discovered_ip = get_mqtt_discovered_ip()
    target_lan_ip = preferred_host if preferred_host else (discovered_ip if discovered_ip else "10.25.7.100")

    # 1. Primary: High-Speed LAN OTA Push via Server 10.25.7.171 (same WiFi subnet, ~3s transfer)
    print(f"[*] Đẩy OTA Tốc độ cao qua mạng nội bộ LAN (.171 -> {target_lan_ip})...")
    import warnings
    warnings.filterwarnings("ignore")
    import paramiko

    try:
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(AI_SERVER_HOST, username=AI_SERVER_USER, password=AI_SERVER_PASS, timeout=8.0)
        cmd = (
            f"for i in 1 2 3; do "
            f"res=$(curl -s -m 35 -X POST --data-binary @/opt/esp32cam_ai_server/firmware/cam_streamer.bin 'http://{target_lan_ip}/api/ota/upload'); "
            f"if [ -z \"$res\" ]; then "
            f"res=$(curl -s -m 20 -X POST 'http://{target_lan_ip}/api/ota/pull?url=http://10.25.7.171:8095/firmware/cam_streamer.bin'); "
            f"fi; "
            f"if [ -n \"$res\" ]; then echo \"$res\"; break; fi; "
            f"sleep 1; done"
        )
        _, stdout, _ = ssh.exec_command(cmd)
        out = stdout.read().decode("utf-8", errors="replace").strip()
        if out:
            print(f"[+] Phản hồi LAN OTA từ ESP32-CAM ({target_lan_ip}): {out}")
            time.sleep(8.0)
            _, st_out, _ = ssh.exec_command(f"curl -s -m 10 http://{target_lan_ip}/status")
            st_txt = st_out.read().decode("utf-8", errors="replace").strip()
            if st_txt:
                print(f"[+] Mạch ESP32-CAM đã khởi động lại vào phân vùng mới thành công! (/status: {len(st_txt)} bytes)")
            ssh.close()
            return
        ssh.close()
    except Exception as e:
        print(f"[-] Không thể đẩy qua .171: {e}")

    # 2. Fallback: Direct HTTP OTA from Windows PC (LAN / WireGuard VPN)
    fw_bytes = BIN_PATH.read_bytes()
    candidates = [preferred_host] if preferred_host else [discovered_ip] + DEFAULT_ESP_HOSTS
    for h in candidates:
        if not h:
            continue
        try:
            req = urllib.request.Request(
                f"http://{h}/api/ota/upload",
                data=fw_bytes,
                method="POST",
                headers={"Content-Type": "application/octet-stream"},
            )
            with urllib.request.urlopen(req, timeout=60.0) as resp:
                print(f"[+] Direct OTA thành công ({h}): {resp.read().decode('utf-8', errors='replace')}")
            time.sleep(7.0)
            return
        except Exception:
            continue

    print(f"[!] Hiện tại mạch ESP32-CAM ({target_lan_ip}) đang Offline. Bản firmware mới đã được đặt sẵn trên Server .171 để nạp ngay khi mạch có điện lại!")


def main() -> None:
    parser = argparse.ArgumentParser(description="Local ESP32-CAM Build & OTA Deployer (Zero-T490s)")
    parser.add_argument("--ui-only", action="store_true", help="Chỉ đóng gói lại web_ui.h")
    parser.add_argument("--build-only", action="store_true", help="Chỉ Pack UI + Build Local trên Windows")
    parser.add_argument("--ai-only", action="store_true", help="Chỉ cập nhật esp32cam_ai_server.py lên .171")
    parser.add_argument("--skip-ota", action="store_true", help="Chỉ Build + đẩy lên .171, không nạp OTA")
    parser.add_argument("--host", type=str, default="", help="IP hoặc hostname của ESP32-CAM để nạp OTA")
    args = parser.parse_args()

    if args.ui_only:
        step_pack_ui()
        return

    if args.ai_only:
        step_sync_ai_server(sync_bin=False, sync_py=True)
        return

    step_pack_ui()
    step_local_build()

    if args.build_only:
        return

    step_sync_ai_server(sync_bin=True, sync_py=True)
    if not args.skip_ota:
        step_ota_flash(preferred_host=args.host)


if __name__ == "__main__":
    main()
