let gConfig = null;
let hmirrorVal = 0, vflipVal = 0;
let currentTab = 'live';
let streamMode = 'mjpeg'; // 'mjpeg' | 'snap' | 'pause'
let snapTimer = null;
let statusInFlight = false;
let interactiveRoi = false;
let roiDragStart = null;
let sdCacheFiles = [];
let sdFilterMode = 'all';

function showToast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.style.display = 'block';
  clearTimeout(window._toastTimer);
  window._toastTimer = setTimeout(() => t.style.display = 'none', 2600);
}

function formatUptime(sec) {
  const d = Math.floor(sec / 86400);
  const h = Math.floor((sec % 86400) / 3600);
  const m = Math.floor((sec % 3600) / 60);
  const s = sec % 60;
  const pad = n => String(n).padStart(2, '0');
  return (d > 0 ? `${d}d ` : '') + `${pad(h)}:${pad(m)}:${pad(s)}`;
}

// ============================================================================
// STREAM VIEWPORT & BANDWIDTH / THERMAL MANAGEMENT
// ============================================================================
let streamWatchdog = null;

function applyStreamSource() {
  const sImg = document.getElementById('stream-img');
  if (snapTimer) { clearInterval(snapTimer); snapTimer = null; }
  if (streamWatchdog) { clearTimeout(streamWatchdog); streamWatchdog = null; }
  if (document.hidden || currentTab !== 'live' || streamMode === 'pause') {
    sImg.onload = null;
    sImg.onerror = null;
    sImg.src = '';
    return;
  }
  if (streamMode === 'mjpeg') {
    const fallbackToSnap = () => {
      if (streamWatchdog) { clearTimeout(streamWatchdog); streamWatchdog = null; }
      if (streamMode === 'mjpeg') {
        streamMode = 'snap';
        ['mjpeg', 'snap', 'pause'].forEach(m => {
          const b = document.getElementById('mode-btn-' + m);
          if (b) b.classList.toggle('active', m === 'snap');
        });
        applyStreamSource();
        showToast('📸 Tự chuyển sang Ảnh Tĩnh nhanh (Tối ưu 4G/VPN)');
      }
    };
    sImg.onload = function() {
      if (streamWatchdog) { clearTimeout(streamWatchdog); streamWatchdog = null; }
    };
    sImg.onerror = fallbackToSnap;
    streamWatchdog = setTimeout(fallbackToSnap, 2200);
    sImg.src = 'http://' + window.location.hostname + ':81/stream?t=' + Date.now();
  } else if (streamMode === 'snap') {
    sImg.onload = null;
    sImg.onerror = null;
    let snapBusy = false;
    const loadNextSnap = () => {
      if (document.hidden || currentTab !== 'live' || streamMode !== 'snap' || snapBusy) return;
      snapBusy = true;
      sImg.onload = () => { snapBusy = false; };
      sImg.onerror = () => { snapBusy = false; };
      sImg.src = '/snapshot?t=' + Date.now();
    };
    loadNextSnap();
    snapTimer = setInterval(loadNextSnap, 1500);
  }
}

function setStreamMode(mode) {
  streamMode = mode;
  ['mjpeg', 'snap', 'pause'].forEach(m => {
    const b = document.getElementById('mode-btn-' + m);
    if (b) b.classList.toggle('active', m === mode);
  });
  applyStreamSource();
  const labels = {
    mjpeg: '🎥 Đã bật luồng Live MJPEG thời gian thực',
    snap: '📸 Chế độ Ảnh Tĩnh (2s/lần) — Siêu tiết kiệm 4G/VPN & Mát máy',
    pause: '⏸️ Đã tạm ngắt luồng hình ảnh (Eco Standby)'
  };
  showToast(labels[mode] || '');
}

function refreshSnapshotOnce() {
  const sImg = document.getElementById('stream-img');
  sImg.src = '/snapshot?t=' + Date.now();
  showToast('🔄 Đã làm mới khung hình!');
}

function toggleFullscreenStream() {
  const wrap = document.getElementById('stream-wrap');
  if (!document.fullscreenElement) {
    if (wrap.requestFullscreen) wrap.requestFullscreen();
  } else {
    if (document.exitFullscreen) document.exitFullscreen();
  }
}

// Automatically pause camera stream when phone screen is off or tab is hidden
document.addEventListener('visibilitychange', () => {
  applyStreamSource();
  if (!document.hidden) updateStatus();
});

// ============================================================================
// NAVIGATION TABS & SUB-TABS
// ============================================================================
function switchTab(id, btn) {
  currentTab = id;
  document.querySelectorAll('.tab-content').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(el => el.classList.remove('active'));
  document.getElementById('tab-' + id).classList.add('active');
  if (btn) {
    btn.classList.add('active');
    if (btn.scrollIntoView) {
      btn.scrollIntoView({ behavior: 'smooth', inline: 'center', block: 'nearest' });
    }
  }
  applyStreamSource();
  if (id === 'sd') loadSdFiles();
  if (id === 'health') loadFreeRtosTasks();
  if (id === 'net') loadWgConfig();
  if (id === 'aieyes') {
    loadMultiScreens();
    showAiDisplayCrop();
  }
}

function switchSubTab(id, btn) {
  document.querySelectorAll('.subtab-pane').forEach(el => el.classList.remove('active'));
  document.querySelectorAll('.subtab-btn').forEach(el => el.classList.remove('active'));
  document.getElementById('sub-' + id).classList.add('active');
  if (btn) btn.classList.add('active');
}

// ============================================================================
// INTERACTIVE TOUCH / MOUSE DRAG ROI ON VIDEO VIEWPORT
// ============================================================================
function toggleInteractiveRoi() {
  interactiveRoi = !interactiveRoi;
  const btn = document.getElementById('btn-draw-roi');
  const wrap = document.getElementById('stream-wrap');
  btn.classList.toggle('draw-on', interactiveRoi);
  wrap.classList.toggle('drawing-roi', interactiveRoi);
  btn.textContent = interactiveRoi ? '✅ Đang Vẽ ROI...' : '✋ Kéo Vẽ ROI';
  if (interactiveRoi) {
    showToast('✋ Hãy chạm và kéo trên khung hình Camera để khoanh vùng đồng hồ / đèn báo!');
  }
}

(function initRoiTouchAndMouse() {
  const wrap = document.getElementById('stream-wrap');
  if (!wrap) return;

  function getRelPct(e) {
    const rect = wrap.getBoundingClientRect();
    const touch = (e.touches && e.touches[0]) || (e.changedTouches && e.changedTouches[0]) || e;
    const x = Math.max(0, Math.min(100, ((touch.clientX - rect.left) / rect.width) * 100));
    const y = Math.max(0, Math.min(100, ((touch.clientY - rect.top) / rect.height) * 100));
    return { x: Math.round(x), y: Math.round(y) };
  }

  function onStart(e) {
    if (!interactiveRoi) return;
    e.preventDefault();
    roiDragStart = getRelPct(e);
  }

  function onMove(e) {
    if (!interactiveRoi || !roiDragStart) return;
    e.preventDefault();
    const cur = getRelPct(e);
    const rx = Math.min(85, Math.min(roiDragStart.x, cur.x));
    const ry = Math.min(85, Math.min(roiDragStart.y, cur.y));
    const rw = Math.max(10, Math.min(100 - rx, Math.abs(cur.x - roiDragStart.x)));
    const rh = Math.max(10, Math.min(100 - ry, Math.abs(cur.y - roiDragStart.y)));
    document.getElementById('rng-rx').value = rx;
    document.getElementById('rng-ry').value = ry;
    document.getElementById('rng-rw').value = rw;
    document.getElementById('rng-rh').value = rh;
    updateRoiBox();
  }

  function onEnd(e) {
    if (!interactiveRoi || !roiDragStart) return;
    roiDragStart = null;
    const rx = document.getElementById('rng-rx').value;
    const ry = document.getElementById('rng-ry').value;
    const rw = document.getElementById('rng-rw').value;
    const rh = document.getElementById('rng-rh').value;
    setRoiPreset(rx, ry, rw, rh);
    toggleInteractiveRoi();
  }

  wrap.addEventListener('mousedown', onStart);
  window.addEventListener('mousemove', onMove);
  window.addEventListener('mouseup', onEnd);
  wrap.addEventListener('touchstart', onStart, { passive: false });
  wrap.addEventListener('touchmove', onMove, { passive: false });
  wrap.addEventListener('touchend', onEnd);
})();

function updateRoiBox() {
  const rx = parseInt(document.getElementById('rng-rx').value) || 0;
  const ry = parseInt(document.getElementById('rng-ry').value) || 0;
  const rw = parseInt(document.getElementById('rng-rw').value) || 50;
  const rh = parseInt(document.getElementById('rng-rh').value) || 40;
  document.getElementById('lbl-rx').textContent = rx + '%';
  document.getElementById('lbl-ry').textContent = ry + '%';
  document.getElementById('lbl-rw').textContent = rw + '%';
  document.getElementById('lbl-rh').textContent = rh + '%';
  const box = document.getElementById('roi-overlay');
  box.style.left = rx + '%';
  box.style.top = ry + '%';
  box.style.width = Math.min(100 - rx, rw) + '%';
  box.style.height = Math.min(100 - ry, rh) + '%';
}

function setRoiPreset(x, y, w, h) {
  document.getElementById('rng-rx').value = x;
  document.getElementById('rng-ry').value = y;
  document.getElementById('rng-rw').value = w;
  document.getElementById('rng-rh').value = h;
  updateRoiBox();
  fetch(`/control?var=roi_x&val=${x}`);
  fetch(`/control?var=roi_y&val=${y}`);
  fetch(`/control?var=roi_w&val=${w}`);
  fetch(`/control?var=roi_h&val=${h}`);
  showToast(`🎯 Đã áp dụng vùng ngắm ROI (${x}%, ${y}%, ${w}%x${h}%)!`);
}

async function saveRoiConfig() {
  await fetch('/control?var=roi_save&val=1');
  showToast('💾 Đã lưu tọa độ khung ngắm ROI vào bộ nhớ NVS!');
}

function onMeterModeChange(val) {
  const names = [
    'ROI: TỔNG HỢP',
    'ROI: ĐÈN BÁO LED',
    'ROI: SỐ LED 7 THANH',
    'ROI: MÀN HÌNH LCD',
    'ROI: ĐỒNG HỒ NƯỚC KIM QUAY',
    'ROI: ĐỒNG HỒ ĐA MÀN HÌNH CHUYỂN VÒNG'
  ];
  document.getElementById('roi-mode-lbl').textContent = names[parseInt(val)] || names[0];
  setCamCtrl('meter_mode', val);
  if (parseInt(val) === 4) {
    document.getElementById('sel-isp-profile').value = '4';
  } else if (parseInt(val) === 5) {
    document.getElementById('sel-isp-profile').value = '2';
  }
}

async function applyTargetPreset(preset) {
  const labels = {
    water_meter: '💧 Đồng Hồ Nước (5 Số + Kim Đỏ, Chống lóa kính, Chu kỳ 5p/30s)',
    multi_screen: '🔄 Đồng Hồ Đa Màn Hình Chuyển Vòng (Tự tách từng trang chỉ số OBIS/Đơn vị, Chu kỳ 60s/15s)',
    debug_board: '🔌 Debug Bo Mạch (Đèn PWR/RUN/ERR/TX-RX + Màn hình, Chụp theo lệnh AI)',
    led_status: '🚨 Giám Sát Đèn Báo & RS485 TX/RX (Tắt Flash chống lóa, Anti-Blooming)',
    lcd_oled: '📟 Màn Hình LCD / OLED (Tương phản cao CLAHE, Chu kỳ 2 phút)',
    led_7seg: '🔢 Đọc LED 7 Thanh Tự Phát Sáng (Tắt Flash, Giảm phơi sáng -2 EV)'
  };
  await fetch(`/control?var=target_preset&val=${encodeURIComponent(preset)}`);
  setTimeout(loadConfig, 300);
  showToast(`⚡ Đã chuyển mềm sang hồ sơ: ${labels[preset] || preset}`);
}

async function saveMeterBaseline() {
  const v = (document.getElementById('inp-meter-base').value || '').trim();
  if (!v) {
    showToast('⚠️ Vui lòng nhập chỉ số đồng hồ nước hiện tại (VD: 00304.954)');
    return;
  }
  await fetch('/control?var=meter_baseline&val=' + encodeURIComponent(v));
  // Also push calibration seed to Deep AI Server (10.25.7.171:8095)
  try {
    const dev = document.getElementById('cfg-mdns').value || 'esp32-cam';
    const sn = (document.getElementById('inp-meter-serial')?.value || '00000198').trim();
    fetch(`http://10.25.7.171:8095/api/meter/calibrate?device=${encodeURIComponent(dev)}&serial=${encodeURIComponent(sn)}&baseline=${encodeURIComponent(v)}`, { method: 'POST' }).catch(()=>{});
  } catch(e) {}
  showToast(`📌 Đã gán Số Gốc V(t-1) = ${v} m³! Các lần đọc tới sẽ bám sát số này.`);
}

async function saveMeterIdentity() {
  const sn = (document.getElementById('inp-meter-serial').value || '00000198').trim();
  const sp = (document.getElementById('inp-meter-spec').value || 'GB/T778-2018 R=80').trim();
  const autoEn = document.getElementById('chk-serial-auto').checked ? 1 : 0;
  await fetch('/control?var=meter_serial&val=' + encodeURIComponent(sn));
  await fetch('/control?var=meter_spec&val=' + encodeURIComponent(sp));
  await fetch('/control?var=serial_auto&val=' + autoEn);
  const idBadge = document.getElementById('lbl-meter-identity');
  if (idBadge) idBadge.textContent = `🏷️ SN: ${sn} | ${sp}`;
  const mqLbl = document.getElementById('lbl-mqtt-serial-topic');
  const pfx = document.getElementById('cfg-mqtt-prefix')?.value || 'esp32cam';
  if (mqLbl) mqLbl.textContent = `MQTT: ${pfx}/meter/${sn}`;
  showToast(`🏷️ Đã lưu định danh đồng hồ: SN=${sn} (${sp})`);
}

function onCyclePresetChange(sec) {
  document.getElementById('cfg-auto-sec').value = sec;
  setCamCtrl('auto_capture_sec', sec);
  showToast(parseInt(sec) > 0 ? `⏱️ Đã bật chu kỳ đọc tự động mỗi ${sec} giây!` : '⏱️ Đã tắt chu kỳ đọc tự động!');
}

function setFlashTorch(duty) {
  document.getElementById('rng-flash-torch').value = duty;
  document.getElementById('lbl-flash-torch').textContent = Math.round(duty * 100 / 255) + '%';
  setCamCtrl('flash_duty', duty);
}

let flowHistoryPts = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

function previewRotationDeg(deg) {
  const d = parseInt(deg) || 0;
  document.getElementById('lbl-rot-deg').textContent = d + '°';
  const sImg = document.getElementById('stream-img');
  if (sImg) {
    if (d === 0) {
      sImg.style.transform = '';
    } else {
      const rad = Math.abs(d) * Math.PI / 180;
      const sinV = Math.abs(Math.sin(rad));
      const scale = (1 - 0.25 * sinV).toFixed(3);
      sImg.style.transform = `rotate(${d}deg) scale(${scale})`;
    }
  }
}

function setRotPreset(deg) {
  document.getElementById('rng-rot-deg').value = deg;
  previewRotationDeg(deg);
  setCamCtrl('meter_rot_deg', deg);
  showToast(`🔄 Đã bù góc xoay mặt đồng hồ: ${deg}°`);
}

async function runRoiFocusSweep() {
  showToast('🔬 Đang quét tự động VCM (380..940) để tìm đỉnh nét cao nhất trong vùng ROI...');
  try {
    const r = await fetch('/control?var=af_roi_sweep&val=1');
    const d = await r.json();
    if (d.best_vcm !== undefined) {
      document.getElementById('rng-focus').value = d.best_vcm;
      document.getElementById('lbl-focus').textContent = d.best_vcm;
      showToast(`✅ Đã khóa đỉnh nét ROI tại VCM = ${d.best_vcm} (Độ nét: ${d.sharpness})`);
    }
  } catch (e) {
    showToast('⚠️ Lỗi khi quét đỉnh nét ROI');
  }
}

function updateFlowSparkline(flowLpm, leakAlert) {
  flowHistoryPts.push(Number(flowLpm) || 0);
  if (flowHistoryPts.length > 24) flowHistoryPts.shift();
  const maxVal = Math.max(5.0, ...flowHistoryPts);
  const pts = flowHistoryPts.map((v, i) => {
    const x = Math.round((i / (flowHistoryPts.length - 1)) * 360);
    const y = Math.round(38 - (Math.min(v, maxVal) / maxVal) * 32);
    return `${x},${y}`;
  }).join(' ');
  const poly = document.getElementById('flow-spark-poly');
  if (poly) poly.setAttribute('points', pts);

  const badge = document.getElementById('leak-status-badge');
  if (badge) {
    if (leakAlert && leakAlert !== 'NORMAL') {
      badge.className = 'badge warn';
      badge.textContent = `🚨 ${leakAlert} (${flowLpm} L/p)`;
    } else if (flowLpm > 0.05) {
      badge.className = 'badge info';
      badge.textContent = `💧 Đang chảy (${flowLpm} L/phút)`;
    } else {
      badge.className = 'badge ok';
      badge.textContent = '✅ Đường ống kín (0.0 L/p)';
    }
  }
}

// ============================================================================
// WIFI PROFILES & FULL CONFIGURATION
// ============================================================================
function renderWifiSlots(list, activeSlot) {
  const box = document.getElementById('wifi-slots-container');
  box.innerHTML = '';
  for (let i = 0; i < 5; i++) {
    const w = (list && list[i]) ? list[i] : {ssid:'', password:'', enabled:0};
    const isAct = (i === activeSlot);
    const div = document.createElement('div');
    div.className = 'wifi-slot' + (isAct ? ' active-slot' : '');
    div.innerHTML = `
      <div class="slot-head">
        <span>ƯU TIÊN #${i+1} ${isAct ? '✅ (ĐANG KẾT NỐI)' : ''}</span>
        <div style="display:flex;gap:8px;align-items:center">
          <label style="font-weight:400;cursor:pointer"><input type="checkbox" id="wf-en-${i}" ${w.enabled ? 'checked' : ''}> Bật</label>
          <button class="btn btn-sm btn-primary" onclick="connectWifiSlot(${i})">Kết nối</button>
        </div>
      </div>
      <div class="form-grid">
        <input type="text" id="wf-ssid-${i}" placeholder="Tên WiFi (SSID #${i+1})" value="${w.ssid || ''}">
        <input type="password" id="wf-pass-${i}" placeholder="Mật khẩu WiFi #${i+1}" value="${w.password || ''}">
      </div>
    `;
    box.appendChild(div);
  }
}

async function loadConfig() {
  try {
    const r = await fetch('/api/config');
    const c = await r.json();
    gConfig = c;
    document.getElementById('dev-title').textContent = c.device_name || 'ESP32-CAM AI';
    document.getElementById('cfg-mdns').value = c.mdns_host || 'esp32-cam';
    document.getElementById('cfg-devname').value = c.device_name || '';
    document.getElementById('cfg-ap-on').checked = !!c.ap_always_on;

    document.getElementById('cfg-ntp-srv').value = c.ntp_server || 'pool.ntp.org';
    document.getElementById('cfg-tz').value = c.tz_str || 'ICT-7';
    document.getElementById('cfg-osd-en').checked = !!c.osd_enable;

    document.getElementById('cfg-ftp-en').checked = !!c.ftp_enable;
    document.getElementById('cfg-ftp-host').value = c.ftp_host || '';
    document.getElementById('cfg-ftp-port').value = c.ftp_port || 21;
    document.getElementById('cfg-ftp-user').value = c.ftp_user || '';
    document.getElementById('cfg-ftp-pass').value = c.ftp_pass || '';
    document.getElementById('cfg-ftp-dir').value = c.ftp_remote_dir || '';

    document.getElementById('cfg-mqtt-en').checked = !!c.mqtt_enable;
    document.getElementById('cfg-mqtt-uri').value = c.mqtt_uri || '';
    document.getElementById('cfg-mqtt-prefix').value = c.mqtt_topic_prefix || 'esp32cam';
    document.getElementById('cfg-mqtt-int').value = c.mqtt_interval_sec || 5;

    document.getElementById('cfg-ntfy-en').checked = !!c.ntfy_enable;
    document.getElementById('cfg-ntfy-url').value = c.ntfy_url || '';

    document.getElementById('cfg-sd-en').checked = !!c.sd_save_enable;
    document.getElementById('cfg-auto-sec').value = c.auto_capture_sec || 0;
    const cpSel = document.getElementById('sel-cycle-preset');
    if (cpSel) cpSel.value = String(c.auto_capture_sec || 0);
    const adChk = document.getElementById('chk-adaptive-cycle');
    if (adChk) adChk.checked = c.adaptive_cycle_enable !== undefined ? !!c.adaptive_cycle_enable : true;

    document.getElementById('cfg-edge-en').checked = !!c.edge_ai_enable;
    document.getElementById('cfg-edge-thresh').value = c.edge_motion_thresh || 20;
    document.getElementById('cfg-edge-trig').checked = !!c.edge_trigger_capture;
    document.getElementById('cfg-ai-url').value = c.ai_server_url || '';
    document.getElementById('cfg-roi-osd').checked = c.roi_draw_osd !== undefined ? !!c.roi_draw_osd : true;

    document.getElementById('sel-meter-mode').value = c.meter_mode || 0;
    const baseInp = document.getElementById('inp-meter-base');
    if (baseInp && c.meter_baseline) baseInp.value = c.meter_baseline;
    const snInp = document.getElementById('inp-meter-serial');
    if (snInp) snInp.value = c.meter_serial || '00000198';
    const spInp = document.getElementById('inp-meter-spec');
    if (spInp) spInp.value = c.meter_spec || 'GB/T778-2018 R=80';
    const saChk = document.getElementById('chk-serial-auto');
    if (saChk) saChk.checked = c.serial_auto_detect !== undefined ? !!c.serial_auto_detect : true;
    const idBadge = document.getElementById('lbl-meter-identity');
    if (idBadge) idBadge.textContent = `🏷️ SN: ${c.meter_serial || '00000198'} | ${c.meter_spec || 'GB/T778-2018 R=80'}`;
    const mqLbl = document.getElementById('lbl-mqtt-serial-topic');
    if (mqLbl) mqLbl.textContent = `MQTT: ${c.mqtt_topic_prefix || 'esp32cam'}/meter/${c.meter_serial || '00000198'}`;
    if (c.meter_rot_deg !== undefined) {
      const rEl = document.getElementById('rng-rot-deg');
      if (rEl) rEl.value = c.meter_rot_deg;
      previewRotationDeg(c.meter_rot_deg);
    }

    const flMode = document.getElementById('sel-flash-mode');
    if (flMode && c.flash_mode !== undefined) flMode.value = c.flash_mode;
    const flCap = document.getElementById('rng-flash-cap');
    if (flCap && c.flash_cap_duty !== undefined) {
      flCap.value = c.flash_cap_duty;
      document.getElementById('lbl-flash-cap').textContent = Math.round(c.flash_cap_duty * 100 / 255) + '% (' + c.flash_cap_duty + ')';
    }
    const flTorch = document.getElementById('rng-flash-torch');
    if (flTorch && c.flash_duty !== undefined) {
      flTorch.value = c.flash_duty;
      document.getElementById('lbl-flash-torch').textContent = Math.round(c.flash_duty * 100 / 255) + '%';
    }
    document.getElementById('rng-rx').value = c.roi_x_pct !== undefined ? c.roi_x_pct : 20;
    document.getElementById('rng-ry').value = c.roi_y_pct !== undefined ? c.roi_y_pct : 25;
    document.getElementById('rng-rw').value = c.roi_w_pct !== undefined ? c.roi_w_pct : 60;
    document.getElementById('rng-rh').value = c.roi_h_pct !== undefined ? c.roi_h_pct : 45;
    updateRoiBox();

    document.getElementById('sel-framesize').value = c.cam_framesize || 10;
    document.getElementById('rng-qual').value = c.cam_quality || 12;
    document.getElementById('lbl-qual').textContent = c.cam_quality || 12;
    document.getElementById('rng-bright').value = c.cam_brightness || 0;
    document.getElementById('lbl-bright').textContent = c.cam_brightness || 0;
    document.getElementById('rng-contrast').value = c.cam_contrast || 0;
    document.getElementById('lbl-contrast').textContent = c.cam_contrast || 0;
    document.getElementById('rng-focus').value = c.af_manual_pos || 512;
    document.getElementById('lbl-focus').textContent = c.af_manual_pos || 512;
    hmirrorVal = c.cam_hmirror || 0;
    vflipVal = c.cam_vflip || 0;

    renderWifiSlots(c.wifi_list, c.active_wifi_slot);
  } catch (e) { console.error(e); }
}

async function saveFullConfig() {
  const wifi_list = [];
  for (let i = 0; i < 5; i++) {
    const ssidEl = document.getElementById(`wf-ssid-${i}`);
    if (ssidEl) {
      wifi_list.push({
        ssid: ssidEl.value.trim(),
        password: document.getElementById(`wf-pass-${i}`).value,
        enabled: document.getElementById(`wf-en-${i}`).checked ? 1 : 0
      });
    }
  }
  const payload = {
    mdns_host: document.getElementById('cfg-mdns').value.trim(),
    device_name: document.getElementById('cfg-devname').value.trim(),
    ap_always_on: document.getElementById('cfg-ap-on').checked ? 1 : 0,
    ntp_server: document.getElementById('cfg-ntp-srv').value.trim(),
    tz_str: document.getElementById('cfg-tz').value.trim(),
    osd_enable: document.getElementById('cfg-osd-en').checked ? 1 : 0,
    ftp_enable: document.getElementById('cfg-ftp-en').checked ? 1 : 0,
    ftp_host: document.getElementById('cfg-ftp-host').value.trim(),
    ftp_port: parseInt(document.getElementById('cfg-ftp-port').value) || 21,
    ftp_user: document.getElementById('cfg-ftp-user').value.trim(),
    ftp_pass: document.getElementById('cfg-ftp-pass').value,
    ftp_remote_dir: document.getElementById('cfg-ftp-dir').value.trim(),
    mqtt_enable: document.getElementById('cfg-mqtt-en').checked ? 1 : 0,
    mqtt_uri: document.getElementById('cfg-mqtt-uri').value.trim(),
    mqtt_topic_prefix: document.getElementById('cfg-mqtt-prefix').value.trim(),
    mqtt_interval_sec: parseInt(document.getElementById('cfg-mqtt-int').value) || 5,
    ntfy_enable: document.getElementById('cfg-ntfy-en').checked ? 1 : 0,
    ntfy_url: document.getElementById('cfg-ntfy-url').value.trim(),
    sd_save_enable: document.getElementById('cfg-sd-en').checked ? 1 : 0,
    auto_capture_sec: parseInt(document.getElementById('cfg-auto-sec').value) || 0,
    adaptive_cycle_enable: document.getElementById('chk-adaptive-cycle').checked ? 1 : 0,
    edge_ai_enable: document.getElementById('cfg-edge-en').checked ? 1 : 0,
    edge_motion_thresh: parseInt(document.getElementById('cfg-edge-thresh').value) || 20,
    edge_trigger_capture: document.getElementById('cfg-edge-trig').checked ? 1 : 0,
    ai_server_url: document.getElementById('cfg-ai-url').value.trim(),
    meter_mode: parseInt(document.getElementById('sel-meter-mode').value) || 0,
    meter_baseline: (document.getElementById('inp-meter-base').value || '').trim(),
    meter_serial: (document.getElementById('inp-meter-serial')?.value || '00000198').trim(),
    meter_spec: (document.getElementById('inp-meter-spec')?.value || 'GB/T778-2018 R=80').trim(),
    serial_auto_detect: document.getElementById('chk-serial-auto')?.checked ? 1 : 0,
    meter_rot_deg: parseInt(document.getElementById('rng-rot-deg').value) || 0,
    flash_mode: isNaN(parseInt(document.getElementById('sel-flash-mode').value)) ? 1 : parseInt(document.getElementById('sel-flash-mode').value),
    flash_cap_duty: isNaN(parseInt(document.getElementById('rng-flash-cap').value)) ? 55 : parseInt(document.getElementById('rng-flash-cap').value),
    roi_x_pct: parseInt(document.getElementById('rng-rx').value) || 20,
    roi_y_pct: parseInt(document.getElementById('rng-ry').value) || 25,
    roi_w_pct: parseInt(document.getElementById('rng-rw').value) || 60,
    roi_h_pct: parseInt(document.getElementById('rng-rh').value) || 45,
    roi_draw_osd: document.getElementById('cfg-roi-osd').checked ? 1 : 0,
    wifi_list: wifi_list
  };
  const r = await fetch('/api/config', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload)
  });
  if (r.ok) {
    showToast('✅ Đã lưu toàn bộ cấu hình vào bộ nhớ NVS!');
    loadConfig();
  } else {
    showToast('❌ Lỗi lưu cấu hình');
  }
}

function updateLedDotStyle(stateStr) {
  const dot = document.getElementById('rd-led-dot');
  const valEl = document.getElementById('rd-led-val');
  if (!stateStr || stateStr === 'OFF') {
    dot.style.background = '#475569';
    dot.style.boxShadow = 'none';
    valEl.style.color = '#94a3b8';
    return;
  }
  const s = stateStr.toUpperCase();
  if (s.includes('DO') || s.includes('RED') || s.includes('ĐỎ')) {
    dot.style.background = '#ef4444';
    dot.style.boxShadow = '0 0 12px #ef4444';
    valEl.style.color = '#f87171';
  } else if (s.includes('XANH LA') || s.includes('GREEN') || s.includes('XANH LÁ')) {
    dot.style.background = '#10b981';
    dot.style.boxShadow = '0 0 12px #10b981';
    valEl.style.color = '#34d399';
  } else if (s.includes('VANG') || s.includes('YELLOW') || s.includes('VÀNG')) {
    dot.style.background = '#f59e0b';
    dot.style.boxShadow = '0 0 12px #f59e0b';
    valEl.style.color = '#fbbf24';
  } else if (s.includes('DUONG') || s.includes('BLUE') || s.includes('DƯƠNG')) {
    dot.style.background = '#0ea5e9';
    dot.style.boxShadow = '0 0 12px #0ea5e9';
    valEl.style.color = '#38bdf8';
  } else {
    dot.style.background = '#f8fafc';
    dot.style.boxShadow = '0 0 12px #f8fafc';
    valEl.style.color = '#f8fafc';
  }
}

// ============================================================================
// NON-OVERLAPPING TELEMETRY POLLING WITH ABORTCONTROLLER TIMEOUT
// ============================================================================
async function updateStatus() {
  if (document.hidden || statusInFlight) return;
  statusInFlight = true;
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), 3500);
  const t0 = Date.now();
  try {
    const r = await fetch('/status', { signal: ctrl.signal });
    const d = await r.json();
    const rtt = Date.now() - t0;

    document.getElementById('online-dot').className = 'brand-dot';
    document.getElementById('clock-str').textContent = `${d.time} | ${d.mdns}.local (${d.ip})`;
    document.getElementById('time-now-big').textContent = d.time;

    const tb = document.getElementById('temp-badge');
    tb.textContent = `🌡️ ${(d.mcu_temp_c || 44).toFixed(1)}°C`;
    tb.className = 'badge ' + (d.mcu_temp_c > 65 ? 'warn' : 'info');

    document.getElementById('volt-badge').textContent = `⚡ ${(d.vdd33_volts || 3.30).toFixed(2)}V`;
    document.getElementById('wifi-badge').textContent = `📶 ${d.ssid} (${d.rssi}dBm)`;

    const wgb = document.getElementById('wg-badge');
    if (wgb) {
      wgb.textContent = d.wg_connected ? `🔒 WG: ${d.wg_ip}` : (d.wg_enabled ? '🔒 WG: Handshake...' : '🔒 WG: Off');
      wgb.className = 'badge ' + (d.wg_connected ? 'ok' : (d.wg_enabled ? 'info' : ''));
    }

    const mb = document.getElementById('mqtt-badge');
    mb.textContent = d.mqtt ? 'MQTT: OK' : 'MQTT: Off';
    mb.className = 'badge ' + (d.mqtt ? 'ok' : 'warn');

    const sb = document.getElementById('sd-badge');
    sb.textContent = d.sd_mounted ? `💾 SD: ${(d.sd_free_mb/1024).toFixed(1)}GB` : 'SD: Lỗi';
    sb.className = 'badge ' + (d.sd_mounted ? 'ok' : 'warn');

    // Live Stream HUD & Edge LED Indicator + Flash & Countdown
    document.getElementById('hud-ai').textContent = `Motion: ${(d.edge_motion || 0).toFixed(1)}%`;
    document.getElementById('hud-af').textContent = `AF: ${d.af_focused ? 'Đã nét ✅' : (d.af_busy ? 'Đang lấy nét...' : 'Sẵn sàng')} (VCM: ${d.af_pos})`;
    document.getElementById('ftp-last-st').textContent = `FTP: ${d.ftp_status}`;
    document.getElementById('roi-led-live').textContent = `LED: ${d.edge_led_state || 'OFF'}`;
    document.getElementById('rd-led-luma').textContent = `Luma: ${d.roi_luma || 0} (Max ${d.roi_max_luma || 0})`;

    const flSt = document.getElementById('lbl-flash-st');
    if (flSt) {
      const fPct = Math.round(((d.flash_duty || 0) * 100) / 255);
      flSt.textContent = d.flash_duty > 0 ? `🔦 Đang sáng: ${fPct}%` : (d.flash_mode === 2 ? '⚡ Nháy khi chụp' : (d.flash_mode === 1 ? '🌙 Auto khi tối' : 'Tắt'));
    }
    const nextCapLbl = document.getElementById('lbl-next-cap');
    if (nextCapLbl) {
      if (d.auto_capture_sec >= 5) {
        nextCapLbl.textContent = `⏳ Đọc sau: ${d.next_cap_sec || 0}s ${d.adaptive_fast_active ? '⚡(Nhanh)' : ''}`;
      } else {
        nextCapLbl.textContent = 'Chu kỳ: Tắt';
      }
    }
    if (d.meter_serial || d.meter_spec) {
      const idBadge = document.getElementById('lbl-meter-identity');
      if (idBadge) idBadge.textContent = `🏷️ SN: ${d.meter_serial || '00000198'} | ${d.meter_spec || 'GB/T778-2018 R=80'}`;
    }
    if (d.edge_digits && d.edge_digits !== '----' && (!d.last_ai || d.last_ai.includes('idle'))) {
      document.getElementById('rd-dial-val').textContent = d.edge_digits;
      if (d.edge_detail) document.getElementById('rd-dial-sub').textContent = d.edge_detail;
    }

    // Pipeline Status Bar (Tab 2)
    document.getElementById('pipe-cam-st').textContent = `OV5640 (${d.last_capture_ms || 0}ms | OK:${d.capture_ok_cnt || 0})`;
    document.getElementById('pipe-edge-st').textContent = `Motion ${(d.edge_motion||0).toFixed(0)}% | LED: ${d.edge_led_state||'OFF'}`;
    document.getElementById('pipe-ftp-st').textContent = `${d.ftp_status || 'Idle'} (OK:${d.ftp_ok_cnt||0})`;
    document.getElementById('pipe-mq-st').textContent = `MQTT: ${d.mqtt ? 'OK' : 'Off'} | Ntfy: ${d.ntfy_status || 'Idle'}`;

    // Parse Server AI JSON for Smart Meter & Indicator results
    if (d.last_ai) {
      try {
        const ai = typeof d.last_ai === 'string' ? JSON.parse(d.last_ai) : d.last_ai;
        document.getElementById('ai-json-box').textContent = JSON.stringify(ai, null, 2);

        const ledSt = (ai.meter && ai.meter.led_status) ? ai.meter.led_status : (d.edge_led_state || 'OFF');
        document.getElementById('rd-led-val').textContent = ledSt;
        updateLedDotStyle(ledSt);
        if (ai.meter && ai.meter.led_detail) {
          document.getElementById('rd-led-sub').textContent = ai.meter.led_detail;
        }
        if (ai.meter && ai.meter.seg7_value !== undefined) {
          document.getElementById('rd-7seg-val').textContent = ai.meter.seg7_value || '----';
          document.getElementById('rd-7seg-sub').textContent = ai.meter.seg7_detail || 'Đã phân tích vùng ROI 7-Segment';
        }
        if (ai.meter && ai.meter.lcd_value !== undefined) {
          document.getElementById('rd-lcd-val').textContent = ai.meter.lcd_value || '----';
          document.getElementById('rd-lcd-sub').textContent = ai.meter.lcd_detail || `Độ nét: ${ai.sharpness || '--'} | NPU: ${ai.latency_ms || '--'}ms`;
        }
        if (ai.meter && ai.meter.dial_value !== undefined) {
          document.getElementById('rd-dial-val').textContent = ai.meter.dial_value || '----';
          document.getElementById('rd-dial-sub').textContent = ai.meter.dial_detail || 'Góc kim quay + Khớp số cũ V(t-1)';
          if (ai.meter.flow_lpm !== undefined) {
            document.getElementById('rd-dial-flow').textContent = `${ai.meter.flow_lpm} L/phút`;
            updateFlowSparkline(ai.meter.flow_lpm, ai.meter.leak_alert);
          }
        }
        if (ai.meter) {
          const elLcd = document.getElementById('aieyes-lcd-txt');
          if (elLcd) elLcd.textContent = ai.meter.lcd_value || '----';
          const elSeg = document.getElementById('aieyes-seg7-txt');
          if (elSeg) elSeg.textContent = ai.meter.seg7_value || '----';
          const elDial = document.getElementById('aieyes-dial-txt');
          if (elDial) elDial.textContent = (ai.meter.dial_value || '----') + ' m³';
          const elLed = document.getElementById('aieyes-led-txt');
          if (elLed) elLed.textContent = ai.meter.led_status || 'OFF';
          if (Array.isArray(ai.meter.led_array) && !window._burstLock) {
            renderAiLedArrayTable(ai.meter.led_array, ai.meter.led_detail);
          }
          if (ai.meter.multi_screens && !window._msSweepLock) {
            renderMultiScreensTable(ai.meter.multi_screens, ai.meter.multi_screen);
          }
        }
      } catch(e) {
        document.getElementById('ai-json-box').textContent = d.last_ai;
      }
    } else {
      document.getElementById('rd-led-val').textContent = d.edge_led_state || 'OFF';
      updateLedDotStyle(d.edge_led_state);
    }

    // System Health Updates
    const tempC = d.mcu_temp_c || 44.5;
    document.getElementById('h-temp-val').textContent = `${tempC.toFixed(1)} °C`;
    const tempPct = Math.min(100, Math.max(10, ((tempC - 25) / 60) * 100));
    const tBar = document.getElementById('h-temp-bar');
    tBar.style.width = `${tempPct}%`;
    tBar.className = 'pbar-fill' + (tempC > 62 ? ' warn' : '');
    document.getElementById('h-cam-thermal').textContent =
      d.cam_sleeping ? 'Camera OV5640: ❄️ Đang Ngủ Standby (<1mA, VCM 0mA)' :
      (d.eco_mode ? 'Camera OV5640: ❄️ Chế độ Mát Eco (10MHz XCLK)' : 'Camera OV5640: 🔥 Turbo Active');

    document.getElementById('h-volt-val').textContent = `${(d.vdd33_volts || 3.30).toFixed(2)} V`;
    document.getElementById('h-vref-sub').textContent = `eFuse ADC Vref: ${d.adc_vref_mv || 1100} mV | Brownout: Ổn định`;

    const cpuLoad = d.cpu_load || 7.5;
    document.getElementById('h-cpu-val').textContent = `${cpuLoad.toFixed(1)} %`;
    document.getElementById('h-cpu-bar').style.width = `${Math.min(100, cpuLoad)}%`;
    document.getElementById('h-cpu-cores').textContent =
      `Core 0 (Net/VPN): ${(d.cpu0_load||0).toFixed(1)}% | Core 1 (Cam/AI): ${(d.cpu1_load||0).toFixed(1)}% | ${d.task_count||0} Tasks`;

    document.getElementById('h-uptime-val').textContent = formatUptime(d.uptime || 0);
    document.getElementById('h-reset-reason').textContent = `Reset: ${d.reset_reason || 'Power-On'}`;

    // Memory Bars
    const ramTot = d.ram_total || 300000;
    const ramFree = d.ram_free || 150000;
    const ramUsed = Math.max(0, ramTot - ramFree);
    const ramPct = Math.min(100, (ramUsed * 100) / ramTot);
    document.getElementById('h-ram-txt').textContent = `${(ramFree/1024).toFixed(0)} KB trống / ${(ramTot/1024).toFixed(0)} KB`;
    document.getElementById('h-ram-bar').style.width = `${ramPct.toFixed(1)}%`;
    document.getElementById('h-ram-min').textContent =
      `Đã dùng: ${ramPct.toFixed(1)}% | Min Free: ${((d.ram_min_free||0)/1024).toFixed(0)} KB | Block lớn nhất: ${((d.ram_largest||0)/1024).toFixed(0)} KB`;

    const psTot = d.psram_total || 4194304;
    const psFree = d.psram_free || 4000000;
    const psUsed = Math.max(0, psTot - psFree);
    const psPct = Math.min(100, (psUsed * 100) / psTot);
    document.getElementById('h-psram-txt').textContent = `${(psFree/1048576).toFixed(2)} MB trống / ${(psTot/1048576).toFixed(2)} MB`;
    document.getElementById('h-psram-bar').style.width = `${psPct.toFixed(1)}%`;
    document.getElementById('h-psram-min').textContent =
      `Đã dùng: ${psPct.toFixed(1)}% | Min Free lịch sử: ${((d.psram_min_free||0)/1048576).toFixed(2)} MB`;

    const sdHwTot = d.sd_total_mb || 15193;
    const sdFatTot = d.sd_fat_total_mb || sdHwTot;
    const sdFree = d.sd_free_mb || 15190;
    const sdUsed = Math.max(0, sdFatTot - sdFree);
    const sdPct = sdFatTot > 0 ? Math.min(100, (sdUsed * 100) / sdFatTot) : 0;
    document.getElementById('h-sd-txt').textContent = `${(sdFree/1024).toFixed(2)} GB trống / ${(sdFatTot/1024).toFixed(2)} GB`;
    document.getElementById('h-sd-bar').style.width = `${Math.max(2, sdPct).toFixed(1)}%`;
    document.getElementById('h-sd-sub').textContent = `Vật lý: ${(sdHwTot/1024).toFixed(2)}GB | FAT: ${(sdFatTot/1024).toFixed(2)}GB | Đã dùng: ${sdUsed}MB (${sdPct.toFixed(1)}%)`;

    // Sync SD Card bar in Tab 4
    document.getElementById('sd-cap-summary').textContent = `${(sdFree/1024).toFixed(2)} GB trống / ${(sdFatTot/1024).toFixed(2)} GB (Đã dùng ${sdPct.toFixed(1)}%)`;
    document.getElementById('sd-cap-bar').style.width = `${Math.max(2, sdPct).toFixed(1)}%`;

    // Network & WireGuard Table (Tab 3)
    document.getElementById('h-ping-badge').textContent = `Ping RTT: ${rtt} ms`;
    document.getElementById('h-net-ip').textContent = d.ip || '0.0.0.0';
    document.getElementById('h-net-gw').textContent = `${d.netmask || '255.255.255.0'} / GW: ${d.gw || '0.0.0.0'}`;
    document.getElementById('h-net-dns').textContent = d.dns || '8.8.8.8';
    document.getElementById('h-net-mac').textContent = d.mac || '--';
    document.getElementById('h-net-mdns').textContent = `http://${d.mdns || 'esp32-cam'}.local`;
    document.getElementById('h-net-wg').textContent = d.wg_connected
      ? `✅ Đã thông hầm (${d.wg_ip} ➔ ${d.wg_endpoint}:${d.wg_port} | MTU ${d.wg_mtu})`
      : (d.wg_enabled ? `⏳ Đang Handshake (${d.wg_ip} ➔ ${d.wg_endpoint}:${d.wg_port})` : 'Tắt (Chưa kích hoạt)');

    const wgPill = document.getElementById('wg-status-pill');
    if (wgPill) {
      wgPill.textContent = d.wg_connected ? `✅ ĐÃ KẾT NỐI (${d.wg_ip})` : (d.wg_enabled ? '⏳ ĐANG HANDSHAKE...' : 'ĐANG TẮT');
      wgPill.className = 'badge ' + (d.wg_connected ? 'ok' : (d.wg_enabled ? 'info' : ''));
    }
    const wgIpTxt = document.getElementById('wg-ip-txt');
    if (wgIpTxt) wgIpTxt.textContent = `${d.wg_ip || '10.212.156.7'} (MTU ${d.wg_mtu || 1360})`;
    const wgPeerTxt = document.getElementById('wg-peer-txt');
    if (wgPeerTxt) wgPeerTxt.textContent = `Peer: ${d.wg_endpoint || 'nxchieu.duckdns.org'}:${d.wg_port || 51820} | Reconnect: ${d.wg_reconnect_cnt || 0}`;
    const wgVaultTxt = document.getElementById('wg-vault-txt');
    if (wgVaultTxt && d.wg_vault) wgVaultTxt.textContent = `🛡️ Lưu trữ: ${d.wg_vault}`;

    document.getElementById('h-net-ssid').textContent = `${d.ssid} (Ưu tiên #${(d.active_slot||0)+1})`;
    document.getElementById('h-net-bssid').textContent = d.bssid || '--';
    const sigQual = Math.min(100, Math.max(0, 2 * ((d.rssi || -70) + 100)));
    document.getElementById('h-net-rssi').textContent = `${d.rssi} dBm (${sigQual}% - Kênh ${d.channel || 1})`;
    document.getElementById('h-svc-mqtt').textContent = d.mqtt ? '✅ Đã kết nối (10.25.7.171:1883)' : '❌ Ngắt kết nối';
    document.getElementById('h-svc-ftp').textContent = `FTP: ${d.ftp_status || 'Idle'} | Ntfy: ${d.ntfy_status || 'Idle'}`;

    // Rescue Banner & OTA Partition Status
    document.getElementById('rescue-banner').style.display = d.rescue_mode ? 'flex' : 'none';
    const runP = d.running_part || 'ota_0';
    const nxtP = d.next_ota_part || 'ota_1';
    const pSize = d.part_size_kb || 1984;
    document.getElementById('h-part-summary').textContent = `Đang chạy: ${runP} ➔ Chờ nạp: ${nxtP}`;
    document.getElementById('h-ota-running').textContent = `${runP} (${pSize}KB)`;
    document.getElementById('h-ota-next').textContent = `${nxtP} (${pSize}KB)`;
    document.getElementById('h-ota-state').textContent = d.ota_state || 'VALID';
    document.getElementById('h-wdt-st').textContent = d.wdt_active ? '✅ TWDT 25s' : 'Init';
    document.getElementById('h-crash-cnt').textContent = `${d.crash_count || 0}/3`;
    document.getElementById('h-op-mode').textContent = d.rescue_mode ? '🚨 RESCUE' : '✅ NORMAL';

    // Industrial Camera Bus, Pipeline SLA & Stack Watermark
    const pidHex = '0x' + (d.cam_pid_hex || 0x5640).toString(16).toUpperCase();
    document.getElementById('h-cam-i2c').textContent = d.cam_i2c_ok ? `${pidHex} (OV5640) OK ✅` : `⚠️ Lỗi Bus SCCB (${pidHex})`;
    document.getElementById('h-cam-rec').textContent = `${d.cam_recovery_cnt || 0} lần tự phục hồi`;
    const ispNames = ['0: Chuẩn Tự Động', '1: LED 7 Thanh (Chống lóa)', '2: Đồng Hồ LCD (Tương phản cao)', '3: Tủ Tối / Nhìn Đêm', '4: Đồng Hồ Nước Kim Quay (Chống lóa + Nổi Kim Đỏ)'];
    document.getElementById('h-isp-prof').textContent = ispNames[d.cam_isp_profile || 0] || ispNames[0];
    document.getElementById('h-cap-lat').textContent = `${d.last_capture_ms || 0} ms (OK: ${d.capture_ok_cnt || 0} | Lỗi: ${d.capture_err_cnt || 0})`;
    document.getElementById('h-io-sla').textContent = `SD OK: ${d.sd_write_ok_cnt || 0} (Err: ${d.sd_write_err_cnt || 0}) | FTP OK: ${d.ftp_ok_cnt || 0} (Err: ${d.ftp_err_cnt || 0})`;
    const minStk = d.min_stack_bytes || 2048;
    document.getElementById('h-min-stack').textContent = `${minStk} B (${d.min_stack_task || 'sys_evt'}) ${minStk > 512 ? '✅' : '⚠️'}`;
    document.getElementById('h-coredump-st').textContent = d.coredump_present ? '⚠️ Có bản ghi Crash CoreDump' : '✅ Sạch (0 lỗi Panic)';
    document.getElementById('h-sd-ring').textContent = d.sd_auto_prune ? `✅ Bật (Giữ trống ≥ ${d.sd_min_free_mb || 200} MB)` : 'Tắt';

    // Tab 6: Silicon Hardware Profile Live Telemetry
    const revMaj = Math.floor((d.hw_rev || 300) / 100);
    const revMin = (d.hw_rev || 300) % 100;
    const hwIdf = document.getElementById('hw-idf-badge');
    if (hwIdf && d.hw_idf_ver) hwIdf.textContent = `ESP-IDF ${d.hw_idf_ver}`;
    const hwSocSub = document.getElementById('hw-soc-sub');
    if (hwSocSub) hwSocSub.textContent = `${d.hw_cores || 2} Nhân Xtensa® LX6 | Rev v${revMaj}.${revMin} @ ${d.cpu_freq_mhz || 160}MHz`;
    const hwTblSoc = document.getElementById('hw-tbl-soc');
    if (hwTblSoc) hwTblSoc.textContent = `${d.hw_soc || 'ESP32-D0WD-V3'} — Silicon Rev v${revMaj}.${revMin}`;
    const hwTblFreq = document.getElementById('hw-tbl-freq');
    if (hwTblFreq) hwTblFreq.textContent = `${d.hw_cores || 2} Nhân @ ${d.cpu_freq_mhz || 160} MHz (Tải hiện tại: ${(d.cpu_load || 0).toFixed(1)}% | Nhiệt: ${(d.mcu_temp_c || 45).toFixed(1)}°C)`;
    const hwTblSram = document.getElementById('hw-tbl-sram');
    if (hwTblSram) hwTblSram.textContent = `520 KB vật lý (Heap khả dụng: ${Math.round((d.ram_total||303139)/1024)} KB | Trống: ${Math.round((d.ram_free||114000)/1024)} KB)`;
    const hwTblPsram = document.getElementById('hw-tbl-psram');
    if (hwTblPsram) hwTblPsram.textContent = `${d.hw_psram_phys_mb || 8} MB vật lý (${((d.psram_total||4178936)/1048576).toFixed(2)} MB ánh xạ MMU | Trống: ${((d.psram_free||3939000)/1048576).toFixed(2)} MB)`;
    const hwTblFlash = document.getElementById('hw-tbl-flash');
    if (hwTblFlash) hwTblFlash.textContent = `${d.hw_flash_mb || 4} MB SPI NOR Flash (Đang chạy: ${runP} | Dự phòng: ${nxtP} | Vault 0x3FC000)`;
    const hwTblVolt = document.getElementById('hw-tbl-volt');
    if (hwTblVolt) hwTblVolt.textContent = `3.0 V – 3.6 V (Đang chạy: ${(d.vdd33_volts||3.30).toFixed(2)} V | eFuse ADC Vref: ${d.adc_vref_mv||1093} mV)`;
    const hwCamBadge = document.getElementById('hw-cam-badge');
    if (hwCamBadge) {
      hwCamBadge.textContent = d.cam_i2c_ok ? `SCCB ${pidHex} OK ✅` : `⚠️ Lỗi ${pidHex}`;
      hwCamBadge.className = 'badge ' + (d.cam_i2c_ok ? 'ok' : 'warn');
    }
    const hwTblCam = document.getElementById('hw-tbl-cam');
    if (hwTblCam && d.hw_cam_name) hwTblCam.textContent = `${d.hw_cam_name} (PID ${pidHex}, VER 0x${(d.hw_cam_ver||0).toString(16).toUpperCase()})`;
    const hwTblAf = document.getElementById('hw-tbl-af');
    if (hwTblAf) hwTblAf.textContent = `Mô-tơ VCM 1024 bước (Vị trí hiện tại: ${d.af_pos || 512}/1023 | Độ nét ROI: ${(d.edge_sharp||0).toFixed(1)})`;
    const hwTblXclk = document.getElementById('hw-tbl-xclk');
    if (hwTblXclk) hwTblXclk.textContent = `XCLK = ${d.hw_cam_xclk_mhz || 10} MHz (LEDC Timer 0) + 2× Framebuffer PSRAM`;
    const hwTblSd = document.getElementById('hw-tbl-sd');
    if (hwTblSd) hwTblSd.textContent = d.sd_mounted ? `${d.sd_name || 'SU16G'} (${((d.sd_total_mb||15193)/1024).toFixed(1)} GB | Trống ${((d.sd_free_mb||1172)/1024).toFixed(2)} GB) — 1-Bit SDMMC` : 'Chưa gắn thẻ nhớ';

    // Expansion I/O Status (GPIO 33, GPIO 12, GPIO 13)
    const g33El = document.getElementById('io-gpio33-txt');
    if (g33El && d.rear_led_mode !== undefined) {
      const mNames = ['🌑 Đang Tắt (OFF)', '🔴 Đang Bật Sáng (ON)', '💓 Nhịp tim Heartbeat (5s)'];
      g33El.textContent = mNames[d.rear_led_mode] || mNames[2];
      g33El.style.color = d.rear_led_mode === 0 ? '#94a3b8' : (d.rear_led_mode === 1 ? '#f87171' : '#34d399');
    }
    const g12El = document.getElementById('io-gpio12-txt');
    if (g12El && d.gpio12_relay !== undefined) {
      g12El.textContent = d.gpio12_relay ? '🟢 ĐANG BẬT (HIGH 3.3V)' : '⚪ ĐANG TẮT (LOW 0V)';
      g12El.style.color = d.gpio12_relay ? '#34d399' : '#94a3b8';
    }
    const g13El = document.getElementById('io-gpio13-txt');
    if (g13El && d.gpio13_level !== undefined) {
      const lvlTxt = d.gpio13_level ? 'HIGH (1 - Bình thường)' : '🚨 LOW (0 - Kích hoạt!)';
      g13El.textContent = `${lvlTxt} | Xung: ${d.gpio13_pulses || 0}`;
      g13El.style.color = d.gpio13_level ? '#38bdf8' : '#fbbf24';
    }

    const selIsp = document.getElementById('sel-isp-profile');
    if (selIsp && document.activeElement !== selIsp && d.cam_isp_profile !== undefined) selIsp.value = d.cam_isp_profile;
    const selWb = document.getElementById('sel-wb-mode');
    if (selWb && document.activeElement !== selWb && d.cam_wb_mode !== undefined) selWb.value = d.cam_wb_mode;
    const rngSharp = document.getElementById('rng-sharp');
    if (rngSharp && document.activeElement !== rngSharp && d.cam_sharpness !== undefined) {
      rngSharp.value = d.cam_sharpness;
      document.getElementById('lbl-sharp').textContent = d.cam_sharpness;
    }
    const rngDen = document.getElementById('rng-denoise');
    if (rngDen && document.activeElement !== rngDen && d.cam_denoise !== undefined) {
      rngDen.value = d.cam_denoise;
      document.getElementById('lbl-denoise').textContent = d.cam_denoise;
    }

    const b0 = document.getElementById('bar-ota0');
    const b1 = document.getElementById('bar-ota1');
    if (runP === 'ota_1') {
      b1.style.background = 'linear-gradient(90deg,#059669,#10b981)';
      b1.style.color = '#fff';
      b0.style.background = '#1e293b';
      b0.style.color = '#94a3b8';
    } else {
      b0.style.background = 'linear-gradient(90deg,#059669,#10b981)';
      b0.style.color = '#fff';
      b1.style.background = '#1e293b';
      b1.style.color = '#94a3b8';
    }
  } catch (e) {
    document.getElementById('online-dot').className = 'brand-dot offline';
  } finally {
    clearTimeout(timer);
    statusInFlight = false;
  }
}

// ============================================================================
// OTA, RESCUE & HARDWARE CONTROLS
// ============================================================================
function uploadOtaFile() {
  const inp = document.getElementById('ota-file-inp');
  if (!inp.files || !inp.files.length) {
    showToast('⚠️ Vui lòng chọn file .bin trước khi nạp!');
    return;
  }
  const file = inp.files[0];
  const stTxt = document.getElementById('ota-status-txt');
  const pWrap = document.getElementById('ota-pbar-wrap');
  const pBar = document.getElementById('ota-pbar');
  pWrap.style.display = 'block';
  pBar.style.width = '0%';
  stTxt.textContent = `⏳ Đang tải lên ${file.name} (${(file.size/1024).toFixed(0)} KB)...`;

  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/ota/upload', true);
  xhr.upload.onprogress = function(e) {
    if (e.lengthComputable) {
      const pct = Math.round((e.loaded * 100) / e.total);
      pBar.style.width = pct + '%';
      stTxt.textContent = `⏳ Đang ghi Flash OTA: ${pct}% (${(e.loaded/1024).toFixed(0)}/${(e.total/1024).toFixed(0)} KB)`;
    }
  };
  xhr.onload = function() {
    if (xhr.status === 200) {
      pBar.style.width = '100%';
      stTxt.textContent = '✅ Nạp OTA thành công! Đang khởi động lại...';
      showToast('🚀 Nạp OTA thành công! Đang khởi động lại...');
      setTimeout(() => location.reload(), 6500);
    } else {
      stTxt.textContent = '❌ Lỗi nạp OTA: ' + xhr.responseText;
      showToast('❌ Lỗi nạp OTA');
    }
  };
  xhr.onerror = function() {
    stTxt.textContent = '❌ Lỗi kết nối trong quá trình tải file OTA';
  };
  xhr.send(file);
}

async function pullOtaServer() {
  const url = document.getElementById('ota-pull-url').value.trim();
  if (!confirm('Kéo và nạp Firmware OTA từ:\n' + url + ' ?')) return;
  showToast('☁️ Đang kéo Firmware từ Server .171 và ghi Flash...');
  try {
    const r = await fetch(`/api/ota/pull?url=${encodeURIComponent(url)}`, {method: 'POST'});
    const d = await r.json();
    if (d.status === 'ok') {
      showToast(`✅ Đã kéo ${d.bytes} bytes vào ${d.next_part}! Đang khởi động lại...`);
      setTimeout(() => location.reload(), 6500);
    } else {
      showToast('❌ Lỗi Pull OTA: ' + (d.msg || 'Không kết nối được'));
    }
  } catch (e) { showToast('❌ Lỗi kết nối khi kéo OTA'); }
}

async function rollbackOta() {
  if (!confirm('Khôi phục (Rollback) về phân vùng Firmware trước đó và khởi động lại?')) return;
  const r = await fetch('/api/ota/rollback', {method: 'POST'});
  const d = await r.json();
  if (d.status === 'ok') {
    showToast('⏪ Đang Rollback về phân vùng trước...');
    setTimeout(() => location.reload(), 5500);
  } else {
    showToast('⚠️ ' + (d.msg || 'Chưa có bản dự phòng'));
  }
}

async function toggleRescueMode(enable) {
  const msg = enable ? 'Chuyển mạch sang Chế độ Cứu Hộ Khẩn Cấp (Tắt Camera DMA, Bật AP+STA để nạp OTA an toàn)?'
                     : 'Thoát Chế độ Cứu Hộ và khởi động lại bình thường?';
  if (!confirm(msg)) return;
  await fetch(`/control?var=rescue_mode&val=${enable}`);
  showToast('🔄 Đang khởi động lại mạch...');
  setTimeout(() => location.reload(), 5000);
}

async function rebootDevice() {
  if (!confirm('Khởi động lại mạch ESP32-CAM ngay bây giờ?')) return;
  await fetch('/control?var=reboot&val=1');
  showToast('🔄 Đang khởi động lại...');
  setTimeout(() => location.reload(), 5000);
}

async function factoryResetNvs() {
  if (!confirm('Khôi phục toàn bộ cấu hình NVS về mặc định (WiFi HA/123123123)?')) return;
  await fetch('/control?var=factory_reset&val=1');
  showToast('🏭 Đã khôi phục cấu hình gốc!');
  loadConfig();
}

async function setCamCtrl(varName, val) {
  await fetch(`/control?var=${varName}&val=${val}`);
  showToast(`Đã chỉnh ${varName} = ${val}`);
}

function setFocusPreset(val) {
  document.getElementById('rng-focus').value = val;
  document.getElementById('lbl-focus').textContent = val;
  setCamCtrl('focus', val);
}

async function triggerAF() {
  document.getElementById('hud-af').textContent = 'AF: Đang lấy nét...';
  await fetch('/control?var=af_trigger&val=1');
  showToast('🎯 Đã kích hoạt Auto Focus & ngắt dòng giữ cuộn VCM!');
  setTimeout(updateStatus, 600);
}

function toggleMirror(type) {
  if (type === 'hmirror') {
    hmirrorVal = hmirrorVal ? 0 : 1;
    setCamCtrl('hmirror', hmirrorVal);
  } else {
    vflipVal = vflipVal ? 0 : 1;
    setCamCtrl('vflip', vflipVal);
  }
}

async function captureAll(mode) {
  showToast('⏳ Đang chụp & đọc chỉ số Đèn báo / LED 7 thanh / LCD...');
  try {
    const r = await fetch(`/capture?mode=${mode}`);
    const d = await r.json();
    if (d.status === 'ok') {
      showToast(`✅ Đã xử lý ${d.file} | FTP: ${d.ftp}`);
      if (streamMode === 'snap') refreshSnapshotOnce();
      updateStatus();
    } else {
      showToast('❌ Lỗi chụp ảnh');
    }
  } catch (e) { showToast('❌ Lỗi kết nối khi chụp'); }
}

async function syncPhoneTime() {
  const epoch = Math.floor(Date.now() / 1000);
  const r = await fetch(`/api/time?epoch=${epoch}`, {method: 'POST'});
  if (r.ok) {
    showToast('🕒 Đã đồng bộ giờ chuẩn từ điện thoại!');
    updateStatus();
  }
}

async function triggerNtpSync() {
  await fetch('/api/time?ntp=1', {method: 'POST'});
  showToast('🌐 Đã gửi yêu cầu đồng bộ NTP!');
  setTimeout(updateStatus, 1500);
}

async function testNtfy() {
  showToast('🔔 Đang gửi thông báo thử...');
  await fetch('/control?var=test_ntfy&val=1');
  showToast('✅ Đã gửi thông báo Ntfy!');
}

async function scanWifi() {
  const box = document.getElementById('wifi-scan-box');
  box.style.display = 'block';
  box.innerHTML = '⏳ Đang quét mạng WiFi xung quanh...';
  try {
    const r = await fetch('/api/wifi/scan');
    const list = await r.json();
    if (!list.length) { box.innerHTML = 'Không tìm thấy mạng nào.'; return; }
    box.innerHTML = '<b>Chọn mạng WiFi để điền vào khe trống:</b><div style="display:flex;flex-wrap:wrap;gap:6px;margin-top:6px">' +
      list.map(ap => {
        const cls = ap.rssi > -65 ? 'ok' : (ap.rssi > -78 ? 'info' : 'warn');
        return `<button class="btn btn-sm" onclick="fillScannedSsid('${ap.ssid}')">📶 ${ap.ssid} <span class="badge ${cls}">${ap.rssi}dBm</span></button>`;
      }).join('') + '</div>';
  } catch (e) { box.innerHTML = 'Lỗi quét WiFi'; }
}

function fillScannedSsid(ssid) {
  for (let i = 0; i < 5; i++) {
    const el = document.getElementById(`wf-ssid-${i}`);
    if (el && !el.value) {
      el.value = ssid;
      document.getElementById(`wf-en-${i}`).checked = true;
      showToast(`Đã điền "${ssid}" vào Ưu tiên #${i+1}`);
      return;
    }
  }
  document.getElementById('wf-ssid-0').value = ssid;
  showToast(`Đã điền "${ssid}" vào Ưu tiên #1`);
}

async function connectWifiSlot(slot) {
  await saveFullConfig();
  showToast(`⏳ Đang chuyển sang WiFi #${slot+1}...`);
  await fetch(`/control?var=wifi_slot&val=${slot}`);
}

// ============================================================================
// SD CARD GALLERY, FILTERING & IN-PAGE LIGHTBOX PREVIEW
// ============================================================================
function setSdFilter(mode) {
  sdFilterMode = mode;
  ['all', 'file', 'dir'].forEach(m => {
    const b = document.getElementById('sd-flt-' + m);
    if (b) b.classList.toggle('active', m === mode);
  });
  renderSdGallery();
}

function openImgModal(name, sizeKb) {
  const m = document.getElementById('img-modal');
  document.getElementById('modal-title').textContent = '🖼️ ' + name;
  document.getElementById('modal-img').src = '/sdcard/' + encodeURIComponent(name);
  document.getElementById('modal-meta').textContent = `Dung lượng: ${sizeKb} KB | Đường dẫn: /sdcard/${name}`;
  const dl = document.getElementById('modal-dl');
  dl.href = '/sdcard/' + encodeURIComponent(name);
  dl.download = name;
  document.getElementById('modal-del').onclick = function() {
    closeImgModal();
    delSdFile(name);
  };
  m.style.display = 'flex';
}

function closeImgModal() {
  document.getElementById('img-modal').style.display = 'none';
  document.getElementById('modal-img').src = '';
}

function renderSdGallery() {
  const box = document.getElementById('sd-file-list');
  const filtered = sdCacheFiles.filter(f => sdFilterMode === 'all' ? true : f.type === sdFilterMode);
  document.getElementById('sd-count-lbl').textContent = `${filtered.length} / ${sdCacheFiles.length} mục`;
  if (!filtered.length) {
    box.innerHTML = '<div style="color:var(--muted);padding:12px">Không có mục nào trong bộ lọc này.</div>';
    return;
  }
  box.innerHTML = filtered.map(f => {
    if (f.type === 'dir') {
      return `
        <div class="file-row" style="background:rgba(245,158,11,0.06);border-color:rgba(245,158,11,0.22)">
          <div class="file-info">
            <span class="file-name" style="color:#fbbf24">📁 ${f.name}</span>
            <span class="file-meta">Thư mục cũ trên thẻ (Dùng Format FAT32 để dọn)</span>
          </div>
          <span class="badge warn">Thư mục</span>
        </div>`;
    }
    const kb = (f.size / 1024).toFixed(1);
    return `
      <div class="file-row">
        <div class="file-info" onclick="openImgModal('${f.name}', '${kb}')">
          <span class="file-name">🖼️ ${f.name}</span>
          <span class="file-meta">${kb} KB • Bấm để xem ảnh</span>
        </div>
        <div style="display:flex;gap:5px;flex-shrink:0">
          <button class="btn btn-sm" onclick="openImgModal('${f.name}', '${kb}')">👁️</button>
          <a class="btn btn-sm" href="/sdcard/${encodeURIComponent(f.name)}" download="${f.name}">⬇️</a>
          <button class="btn btn-sm btn-danger" onclick="delSdFile('${f.name}')">🗑️</button>
        </div>
      </div>`;
  }).join('');
}

async function loadSdFiles() {
  const box = document.getElementById('sd-file-list');
  try {
    const r = await fetch('/list');
    const files = await r.json();
    files.sort((a,b) => {
      if (a.type !== b.type) return a.type === 'file' ? -1 : 1;
      return b.name.localeCompare(a.name);
    });
    sdCacheFiles = files;
    renderSdGallery();
  } catch (e) { box.innerHTML = 'Lỗi đọc thẻ nhớ'; }
}

async function pruneSdFiles() {
  showToast('♻️ Đang dọn 5 ảnh chụp cũ nhất trên thẻ SD...');
  await fetch('/control?var=sd_prune&val=5');
  showToast('✅ Đã dọn 5 ảnh cũ!');
  updateStatus();
  loadSdFiles();
}

async function delSdFile(name) {
  if (!confirm('Xóa file ' + name + '?')) return;
  await fetch(`/delete?file=${encodeURIComponent(name)}`);
  showToast('🗑️ Đã xóa ' + name);
  loadSdFiles();
}

async function formatSdCard() {
  if (!confirm('Format lại toàn bộ phân vùng FAT32 trên thẻ nhớ MicroSD để giải phóng 100% dung lượng? (Dữ liệu cũ trên thẻ sẽ bị xóa)')) return;
  showToast('⏳ Đang Format thẻ nhớ FAT32...');
  await fetch('/control?var=sd_format&val=1');
  showToast('✅ Đã Format thẻ nhớ thành công!');
  updateStatus();
  loadSdFiles();
}

// ============================================================================
// FREERTOS PER-TASK PROFILER & WIREGUARD CONFIG
// ============================================================================
async function loadFreeRtosTasks() {
  const tbody = document.getElementById('rtos-task-body');
  if (!tbody) return;
  try {
    const r = await fetch('/api/tasks');
    const list = await r.json();
    list.sort((a,b) => b.cpu_pct - a.cpu_pct);
    tbody.innerHTML = list.map(t => {
      const hwm = t.stack_hwm || 0;
      const badgeCls = hwm < 512 ? 'warn' : 'ok';
      const badgeTxt = hwm < 512 ? `⚠️ ${hwm} B` : `✅ ${hwm} B`;
      const cpu = (t.cpu_pct || 0).toFixed(1);
      return `<tr>
        <td style="font-weight:700;color:var(--text)">${t.name}</td>
        <td><span class="badge info">${t.core}</span></td>
        <td>Prio ${t.prio}</td>
        <td>${t.state}</td>
        <td>
          <div style="display:flex;align-items:center;gap:6px">
            <span style="min-width:36px">${cpu}%</span>
            <div style="width:50px;height:5px;background:#050811;border-radius:99px;overflow:hidden">
              <div style="width:${Math.min(100, t.cpu_pct || 0)}%;height:100%;background:#0ea5e9"></div>
            </div>
          </div>
        </td>
        <td style="text-align:right"><span class="badge ${badgeCls}">${badgeTxt}</span></td>
      </tr>`;
    }).join('');
  } catch (e) {
    tbody.innerHTML = '<tr><td colspan="6">Lỗi tải danh sách tiến trình</td></tr>';
  }
}

async function loadWgConfig() {
  try {
    const r = await fetch('/api/wireguard');
    const d = await r.json();
    if (d.status === 'ok') {
      const chk = document.getElementById('wg-enable-chk');
      if (chk) chk.checked = !!d.enabled;
      const ta = document.getElementById('wg-conf-text');
      if (ta && d.conf) ta.value = d.conf;
    }
  } catch (e) {}
}

async function toggleWgEnabled(enabled) {
  try {
    await fetch('/api/wireguard', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({enabled: enabled})
    });
    showToast(enabled ? '🔒 Đã bật WireGuard VPN!' : '🔴 Đã tắt WireGuard VPN!');
    setTimeout(updateStatus, 600);
  } catch (e) { showToast('❌ Lỗi cập nhật WireGuard'); }
}

async function saveWgConfig() {
  const conf = document.getElementById('wg-conf-text').value.trim();
  const enabled = document.getElementById('wg-enable-chk').checked;
  if (!conf) {
    showToast('⚠️ Vui lòng nhập hoặc tải file cấu hình .conf!');
    return;
  }
  const r = await fetch('/api/wireguard', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({enabled: enabled, conf: conf})
  });
  const d = await r.json();
  if (d.status === 'ok') {
    showToast('🔒 Đã lưu cấu hình WireGuard & khởi tạo kết nối!');
    loadWgConfig();
    setTimeout(updateStatus, 1000);
  } else {
    showToast('❌ ' + (d.msg || 'Lỗi cấu hình WireGuard'));
  }
}

function loadWgConfFile(inp) {
  if (inp.files && inp.files[0]) {
    const reader = new FileReader();
    reader.onload = function(e) {
      document.getElementById('wg-conf-text').value = e.target.result;
      document.getElementById('wg-enable-chk').checked = true;
      showToast('📄 Đã đọc file ' + inp.files[0].name + '! Bấm Lưu để áp dụng.');
    };
    reader.readAsText(inp.files[0]);
  }
}

// ============================================================================
// AI PHYSICAL EYES & HARDWARE DEBUGGER CONSOLE
// ============================================================================
function renderAiLedArrayTable(ledArray, summaryText) {
  const tbody = document.getElementById('aieyes-led-tbody');
  const sumEl = document.getElementById('aieyes-led-summary');
  if (sumEl && summaryText) sumEl.textContent = summaryText;
  if (!tbody) return;
  if (!ledArray || !ledArray.length) {
    tbody.innerHTML = '<tr><td colspan="5" style="text-align:center;color:var(--muted);padding:12px">Không phát hiện điểm đèn LED phát sáng trong vùng ngắm ROI.</td></tr>';
    return;
  }
  tbody.innerHTML = ledArray.map((item, i) => {
    const col = item.color || 'TRẮNG SÁNG';
    let dotCol = '#f8fafc';
    if (col.includes('ĐỎ') || col.includes('RED')) dotCol = '#ef4444';
    else if (col.includes('XANH LÁ') || col.includes('GREEN')) dotCol = '#10b981';
    else if (col.includes('VÀNG') || col.includes('YELLOW')) dotCol = '#f59e0b';
    else if (col.includes('DƯƠNG') || col.includes('BLUE')) dotCol = '#0ea5e9';
    const tState = item.temporal_state || item.state || 'SOLID_ON';
    const isBlink = tState.includes('BLINK');
    const pos = item.pos_pct ? `(${item.pos_pct[0]}%, ${item.pos_pct[1]}%)` : '--';
    return `
      <tr>
        <td><b>#${item.index || (i+1)}</b></td>
        <td><span style="display:inline-block;width:9px;height:9px;border-radius:50%;background:${dotCol};box-shadow:0 0 8px ${dotCol};margin-right:6px"></span><b>${col}</b></td>
        <td style="color:#cbd5e1">${item.role_hint || 'INDICATOR'}</td>
        <td style="font-family:monospace;color:#94a3b8">${pos} (Luma ${item.peak_luma || '--'})</td>
        <td style="text-align:right">
          <span class="badge ${isBlink ? 'warn' : 'ok'}" style="font-size:10px">${isBlink ? '⚡ ĐANG NHÁY TX/RX' : '🟢 SÁNG TĨNH'}</span>
        </td>
      </tr>
    `;
  }).join('');
}

async function runAiBoardObserve() {
  window._burstLock = false;
  showToast('🔬 Đang chụp & phân tích toàn diện mạch điện / màn hình / đèn báo...');
  try {
    const r = await fetch('/api/ai/observe?flash=auto&focus=keep&mode=debug_board');
    const d = await r.json();
    if (d.deep_vision && d.deep_vision.meter) {
      const m = d.deep_vision.meter;
      renderAiLedArrayTable(m.led_array || [], m.led_detail || 'Đã quét xong');
      showAiDisplayCrop();
      showToast('✅ Đã phân tích mạch: ' + (m.led_status || 'OK'));
    } else {
      showToast('✅ Đã chụp & gửi phân tích AI!');
    }
    updateStatus();
  } catch (e) {
    showToast('❌ Lỗi gọi /api/ai/observe');
  }
}

async function runAiBurstBlinkCheck() {
  window._burstLock = true;
  showToast('⚡ Đang chụp liên thanh 3 ảnh (250ms) để bắt nhịp nháy đèn truyền thông TX/RX...');
  try {
    const rx = document.getElementById('rng-rx').value || 10;
    const ry = document.getElementById('rng-ry').value || 10;
    const rw = document.getElementById('rng-rw').value || 80;
    const rh = document.getElementById('rng-rh').value || 80;
    const r = await fetch(`http://10.25.7.171:8095/api/ai/burst_check?frames=3&interval_ms=250&rx=${rx}&ry=${ry}&rw=${rw}&rh=${rh}`);
    const d = await r.json();
    if (d.status === 'ok') {
      renderAiLedArrayTable(d.led_array || [], d.summary);
      showToast('⚡ ' + d.summary);
    } else {
      showToast('⚠️ ' + (d.message || 'Không chụp được chuỗi Burst'));
    }
  } catch (e) {
    showToast('❌ Không kết nối được Server AI .171:8095');
  }
}

function showAiDisplayCrop() {
  const img = document.getElementById('aieyes-crop-img');
  if (!img) return;
  const rx = document.getElementById('rng-rx').value || 20;
  const ry = document.getElementById('rng-ry').value || 25;
  const rw = document.getElementById('rng-rw').value || 60;
  const rh = document.getElementById('rng-rh').value || 45;
  const rot = document.getElementById('rng-rot-deg') ? document.getElementById('rng-rot-deg').value : 0;
  img.style.opacity = 1;
  img.src = `http://10.25.7.171:8095/api/ai/crop_display?rx=${rx}&ry=${ry}&rw=${rw}&rh=${rh}&rot=${rot}&t=${Date.now()}`;
  document.getElementById('aieyes-disp-meta').textContent = `ROI (${rx}%,${ry}%) | Rot ${rot}° | 2x CLAHE`;
}

function renderMultiScreensTable(msSummary, activeMs) {
  const tbody = document.getElementById('aieyes-ms-tbody');
  const sumEl = document.getElementById('aieyes-ms-summary');
  if (!tbody) return;
  const screens = (msSummary && Array.isArray(msSummary.screens)) ? msSummary.screens : [];
  if (sumEl) {
    if (activeMs && activeMs.is_transition) {
      sumEl.className = 'badge warn';
      sumEl.textContent = '⚠️ Chống nhòe chuyển màn: ' + (activeMs.transition_reason || 'Đang lật trang');
    } else if (screens.length > 0) {
      sumEl.className = 'badge ok';
      sumEl.textContent = `Đã tách ${screens.length} màn hình độc lập`;
    }
  }
  if (!screens.length) {
    tbody.innerHTML = '<tr><td colspan="6" style="text-align:center;color:var(--muted);padding:12px">Chưa có trang màn hình nào được lưu. Bấm <b>"🔄 Quét Trọn Vòng Đa Màn Hình"</b> để thu thập.</td></tr>';
    return;
  }
  const activeSlot = (activeMs && activeMs.slot_index) || (msSummary && msSummary.active_slot) || 0;
  tbody.innerHTML = screens.map(s => {
    const isAct = (s.slot_index === activeSlot);
    const codeTxt = s.page_code ? `<span class="badge info" style="font-family:monospace">${s.page_code}</span>` : `<span style="color:#64748b;font-family:monospace">${s.screen_sig || '--'}</span>`;
    const cumBadge = s.is_cumulative
      ? `<span class="badge ok" style="font-size:9.5px;margin-left:4px">📈 Lũy kế</span>`
      : `<span class="badge" style="font-size:9.5px;margin-left:4px">⚡ Tức thời</span>`;
    const unitStr = s.unit_hint ? ` <span style="color:#38bdf8;font-size:11px">${s.unit_hint}</span>` : '';
    const periodStr = s.est_cycle_period_s ? `~${s.est_cycle_period_s}s/vòng (${s.samples || 1} mẫu)` : `${s.samples || 1} mẫu`;
    return `
      <tr style="${isAct ? 'background:rgba(14,165,233,0.10)' : ''}">
        <td><b>Màn #${s.slot_index}</b> ${isAct ? '🟢' : ''}</td>
        <td>${codeTxt}</td>
        <td><b style="color:#f8fafc">${s.label || ('Màn #' + s.slot_index)}</b>${cumBadge}</td>
        <td style="font-family:monospace;font-size:13px;font-weight:800;color:#34d399">${s.last_value || '----'}${unitStr}</td>
        <td style="color:#94a3b8;font-size:11px">${periodStr}</td>
        <td style="text-align:right">
          <span style="color:#94a3b8;font-size:10.5px;margin-right:6px">${s.updated_at || '--'}</span>
          <button class="btn btn-sm" style="padding:2px 6px;font-size:10px" onclick="renameScreenSlot(${s.slot_index}, '${(s.label || '').replace(/'/g, '')}', '${(s.unit_hint || '').replace(/'/g, '')}')">✏️ Sửa</button>
        </td>
      </tr>
    `;
  }).join('');
}

async function loadMultiScreens() {
  try {
    const dev = (document.getElementById('cfg-mdns') && document.getElementById('cfg-mdns').value) || 'esp32-cam';
    const r = await fetch(`http://10.25.7.171:8095/api/meter/screens?device=${encodeURIComponent(dev)}`);
    const d = await r.json();
    renderMultiScreensTable(d, null);
  } catch (e) {}
}

async function runAiMultiScreenSweep() {
  window._msSweepLock = true;
  showToast('🔄 Đang chụp chuỗi 6 ảnh × 1.6s để gom trọn vòng lặp các màn hình đồng hồ...');
  const sumEl = document.getElementById('aieyes-ms-summary');
  if (sumEl) {
    sumEl.className = 'badge warn';
    sumEl.textContent = '⏳ Đang quét vòng lặp 6 ảnh (~9.5s)...';
  }
  try {
    const rx = document.getElementById('rng-rx').value || 15;
    const ry = document.getElementById('rng-ry').value || 20;
    const rw = document.getElementById('rng-rw').value || 70;
    const rh = document.getElementById('rng-rh').value || 55;
    const rot = document.getElementById('rng-rot-deg') ? document.getElementById('rng-rot-deg').value : 0;
    const dev = (document.getElementById('cfg-mdns') && document.getElementById('cfg-mdns').value) || 'esp32-cam';
    const r = await fetch(`http://10.25.7.171:8095/api/ai/multi_screen_sweep?frames=6&interval_ms=1600&rx=${rx}&ry=${ry}&rw=${rw}&rh=${rh}&rot=${rot}&device=${encodeURIComponent(dev)}`);
    const d = await r.json();
    if (d.status === 'ok') {
      renderMultiScreensTable(d.multi_screens, null);
      showAiDisplayCrop();
      showToast('✅ ' + (d.summary || 'Đã quét xong các màn hình!'));
    } else {
      showToast('⚠️ ' + (d.message || 'Không quét được chuỗi đa màn hình'));
    }
  } catch (e) {
    showToast('❌ Lỗi kết nối Server AI .171:8095 khi quét đa màn hình');
  } finally {
    setTimeout(() => { window._msSweepLock = false; }, 5000);
  }
}

async function renameScreenSlot(slotIdx, curLabel, curUnit) {
  const newLbl = prompt(`Đặt tên hiển thị cho Màn #${slotIdx} (VD: Tổng lũy kế, Lưu lượng tức thời, Điện áp pha A):`, curLabel || `Màn #${slotIdx}`);
  if (newLbl === null) return;
  const newUnit = prompt(`Đơn vị cho Màn #${slotIdx} (VD: m³, m³/h, kWh, V, A, Hz, bar):`, curUnit || '');
  if (newUnit === null) return;
  const dev = (document.getElementById('cfg-mdns') && document.getElementById('cfg-mdns').value) || 'esp32-cam';
  await fetch(`http://10.25.7.171:8095/api/meter/screen_label?device=${encodeURIComponent(dev)}&slot=${slotIdx}&label=${encodeURIComponent(newLbl)}&unit=${encodeURIComponent(newUnit)}`, { method: 'POST' });
  showToast(`✅ Đã cập nhật nhãn Màn #${slotIdx}: ${newLbl} (${newUnit})`);
  loadMultiScreens();
}

async function resetMultiScreens() {
  if (!confirm('Xóa toàn bộ bộ nhớ các khe màn hình xoay vòng để học lại từ đầu?')) return;
  const dev = (document.getElementById('cfg-mdns') && document.getElementById('cfg-mdns').value) || 'esp32-cam';
  await fetch(`http://10.25.7.171:8095/api/meter/screens_reset?device=${encodeURIComponent(dev)}`, { method: 'POST' });
  showToast('🗑️ Đã xóa các khe màn hình xoay vòng!');
  loadMultiScreens();
}

function copyText(txt) {
  navigator.clipboard.writeText(txt).then(() => showToast('📋 Đã copy: ' + txt));
}

function copyMcpConfig() {
  const txt = document.getElementById('mcp-json-snippet').textContent;
  navigator.clipboard.writeText(txt).then(() => showToast('📋 Đã copy cấu hình MCP settings.json!'));
}

function copyZeroPromptBlock() {
  const txt = document.getElementById('zero-prompt-snippet').textContent;
  navigator.clipboard.writeText(txt).then(() => showToast('📋 Đã copy câu lệnh Zero-Prompt cho AI!'));
}

// Initialize App: Fetch /status FIRST with zero contention so UI badges & clock populate in <80ms,
// then start video stream & config load after initial paint so iOS Safari completes page load immediately.
(async function initApp() {
  if (window.location.hostname === '10.212.156.7') {
    streamMode = 'snap';
    ['mjpeg', 'snap', 'pause'].forEach(m => {
      const b = document.getElementById('mode-btn-' + m);
      if (b) b.classList.toggle('active', m === 'snap');
    });
  }
  await updateStatus();
  setTimeout(() => {
    applyStreamSource();
    loadConfig();
  }, 60);
  setInterval(updateStatus, 2500);
})();
