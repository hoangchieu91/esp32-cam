#include "httpd_ai_mcp.h"
#include "app_httpd.h"
#include "app_config.h"
#include "osd_stamp.h"
#include "net_services.h"

#include "esp_camera.h"
#include "esp_camera_af.h"
#include "esp_log.h"
#include "cJSON.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ============================================================================
// AI PHYSICAL EYES & HARDWARE DEBUGGER: ZERO-PROMPT MANIFEST, OBSERVE & MCP
// ============================================================================
esp_err_t ai_manifest_handler(httpd_req_t *req) {
    app_sys_health_t h;
    app_get_sys_health(&h);

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "schema", "esp32cam-ai-eyes-v1");
    cJSON_AddStringToObject(root, "device", g_cfg.device_name);
    cJSON_AddStringToObject(root, "role", "Physical Eyes & Hardware Debug Probe for AI Agents (LEDs, Comm TX/RX, LCD, OLED, 7-Seg, Rotary Meters)");
    cJSON_AddStringToObject(root, "zero_prompt_instruction",
        "AI Agent: Call GET /api/ai/observe (or POST /mcp tool 'observe_hardware_target') to capture & inspect real-world hardware LEDs, LCD/OLED screens, 7-segment displays, and meters. For rapid blinking TX/RX comm LEDs, call http://10.25.7.171:8095/api/ai/burst_check?frames=3.");

    cJSON *urls = cJSON_CreateObject();
    char lan_url[48], vpn_url[48];
    snprintf(lan_url, sizeof(lan_url), "http://%s", h.ip[0] ? h.ip : "10.25.7.100");
    snprintf(vpn_url, sizeof(vpn_url), "http://%s", g_cfg.wg.address[0] ? g_cfg.wg.address : "10.212.156.7");
    cJSON_AddStringToObject(urls, "lan_base", lan_url);
    cJSON_AddStringToObject(urls, "vpn_base", vpn_url);
    cJSON_AddStringToObject(urls, "mdns_base", "http://esp32-cam.local");
    cJSON_AddStringToObject(urls, "ai_server_hub", "http://10.25.7.171:8095");
    cJSON_AddStringToObject(urls, "esp32_mcp_jsonrpc", "/mcp");
    cJSON_AddStringToObject(urls, "ai_server_mcp_jsonrpc", "http://10.25.7.171:8095/mcp");
    cJSON_AddStringToObject(urls, "observe_1call_api", "/api/ai/observe?flash=auto&focus=keep&mode=debug_board");
    cJSON_AddStringToObject(urls, "deep_board_debug_api", "http://10.25.7.171:8095/api/ai/debug_board");
    cJSON_AddStringToObject(urls, "burst_blink_led_api", "http://10.25.7.171:8095/api/ai/burst_check?frames=3&interval_ms=250");
    cJSON_AddStringToObject(urls, "multi_screen_sweep_api", "http://10.25.7.171:8095/api/ai/multi_screen_sweep?frames=6&interval_ms=1600");
    cJSON_AddStringToObject(urls, "display_crop_jpg", "http://10.25.7.171:8095/api/ai/crop_display");
    cJSON_AddStringToObject(urls, "instant_snapshot_jpg", "/snapshot");
    cJSON_AddStringToObject(urls, "live_mjpeg_stream", ":81/stream");
    cJSON_AddItemToObject(root, "endpoints", urls);

    cJSON *state = cJSON_CreateObject();
    cJSON_AddStringToObject(state, "edge_led_state", g_last_edge_ai.edge_led_state);
    cJSON_AddStringToObject(state, "edge_digits", g_last_edge_ai.edge_digits);
    cJSON_AddNumberToObject(state, "roi_luma", g_last_edge_ai.roi_luma);
    cJSON_AddNumberToObject(state, "edge_sharpness", g_last_edge_ai.sharpness_score);
    cJSON_AddNumberToObject(state, "vcm_focus_step", g_cfg.af_manual_pos);
    cJSON_AddNumberToObject(state, "flash_duty", app_flash_get_duty());
    cJSON_AddNumberToObject(state, "rot_deg", g_cfg.meter_rot_deg);
    cJSON *deep = cJSON_Parse(g_last_server_ai_json);
    if (deep) cJSON_AddItemToObject(state, "last_deep_ai", deep);
    cJSON_AddItemToObject(root, "live_observation", state);

    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t ret = httpd_resp_send(req, out, strlen(out));
    free(out);
    return ret;
}

esp_err_t ai_observe_handler(httpd_req_t *req) {
    char query[128] = {0};
    char mode[32] = "debug_board";
    char focus[24] = "keep";
    char flash[24] = "auto";
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "mode", mode, sizeof(mode));
        httpd_query_key_value(query, "focus", focus, sizeof(focus));
        httpd_query_key_value(query, "flash", flash, sizeof(flash));
    }

    app_cam_wake_if_sleeping();
    sensor_t *s = esp_camera_sensor_get();

    // Optional Focus adjustment before observation
    if (strcmp(focus, "roi") == 0) {
        app_cam_roi_focus_sweep();
    } else if (strcmp(focus, "af") == 0) {
        app_cam_trigger_af_and_lock();
    } else if (focus[0] >= '0' && focus[0] <= '9') {
        int fpos = atoi(focus);
        if (fpos >= 0 && fpos <= 1023 && s && s->af_set_manual_position) {
            g_cfg.af_manual_pos = (uint16_t)fpos;
            s->af_set_manual_position(s, (uint16_t)fpos);
            vTaskDelay(pdMS_TO_TICKS(120));
        }
    }

    // Optional Flash override during observation
    uint8_t saved_fmode = g_cfg.flash_mode;
    if (strcmp(flash, "on") == 0) g_cfg.flash_mode = 2;
    else if (strcmp(flash, "off") == 0) g_cfg.flash_mode = 0;

    char filename[64] = {0};
    char ftp_st[64] = {0};
    esp_err_t err = app_execute_capture_pipeline("ai_only", filename, sizeof(filename), ftp_st, sizeof(ftp_st));
    g_cfg.flash_mode = saved_fmode;

    app_sys_health_t *hp = (app_sys_health_t *)calloc(1, sizeof(app_sys_health_t));
    if (hp) app_get_sys_health(hp);
    const char *ip_str = (hp && hp->ip[0]) ? hp->ip : "10.25.7.100";

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", (err == ESP_OK) ? "ok" : "error");
    cJSON_AddStringToObject(root, "mode", mode);
    cJSON_AddStringToObject(root, "file", filename);

    char url_buf[128];
    snprintf(url_buf, sizeof(url_buf), "http://%s/snapshot", ip_str);
    cJSON_AddStringToObject(root, "snapshot_url", url_buf);
    snprintf(url_buf, sizeof(url_buf), "http://%s/sdcard/%s", ip_str, filename);
    cJSON_AddStringToObject(root, "sdcard_image_url", url_buf);
    snprintf(url_buf, sizeof(url_buf), "http://10.25.7.171:8095/annotated/%s", filename);
    cJSON_AddStringToObject(root, "annotated_ai_image_url", url_buf);

    cJSON *edge = cJSON_CreateObject();
    cJSON_AddStringToObject(edge, "led_state", g_last_edge_ai.edge_led_state);
    cJSON_AddStringToObject(edge, "edge_digits", g_last_edge_ai.edge_digits);
    cJSON_AddNumberToObject(edge, "roi_luma", g_last_edge_ai.roi_luma);
    cJSON_AddNumberToObject(edge, "roi_max_luma", g_last_edge_ai.roi_max_luma);
    cJSON_AddNumberToObject(edge, "sharpness", g_last_edge_ai.sharpness_score);
    cJSON_AddNumberToObject(edge, "vcm_focus_pos", g_cfg.af_manual_pos);
    cJSON_AddItemToObject(root, "edge_sensor", edge);

    cJSON *deep = cJSON_Parse(g_last_server_ai_json);
    if (deep) {
        cJSON_AddItemToObject(root, "deep_vision", deep);
    } else {
        cJSON_AddStringToObject(root, "deep_vision_raw", g_last_server_ai_json);
    }

    if (hp) free(hp);
    char *out = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t ret = httpd_resp_send(req, out, strlen(out));
    free(out);
    return ret;
}

esp_err_t mcp_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    if (req->method == HTTP_GET) {
        const char *info = "{\"protocol\":\"mcp-jsonrpc-2.0\",\"server\":\"esp32-cam-ai-eyes\",\"status\":\"ready\",\"manifest\":\"/api/ai/manifest\",\"ai_server_mcp\":\"http://10.25.7.171:8095/mcp\"}";
        return httpd_resp_send(req, info, strlen(info));
    }

    int total_len = req->content_len;
    if (total_len <= 0 || total_len > 2048) {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid MCP payload");
    }
    char *buf = (char *)calloc(1, total_len + 1);
    if (!buf) return ESP_FAIL;
    int r = httpd_req_recv(req, buf, total_len);
    if (r <= 0) { free(buf); return ESP_FAIL; }

    cJSON *rpc = cJSON_Parse(buf);
    free(buf);
    if (!rpc) {
        return httpd_resp_send(req, "{\"jsonrpc\":\"2.0\",\"error\":{\"code\":-32700,\"message\":\"Parse error\"},\"id\":null}", -1);
    }

    cJSON *id_item = cJSON_GetObjectItem(rpc, "id");
    cJSON *method_item = cJSON_GetObjectItem(rpc, "method");
    const char *method = (method_item && cJSON_IsString(method_item)) ? method_item->valuestring : "";

    cJSON *resp = cJSON_CreateObject();
    cJSON_AddStringToObject(resp, "jsonrpc", "2.0");
    if (id_item) cJSON_AddItemToObject(resp, "id", cJSON_Duplicate(id_item, true));
    else cJSON_AddNullToObject(resp, "id");

    if (strcmp(method, "initialize") == 0) {
        cJSON *res = cJSON_CreateObject();
        cJSON_AddStringToObject(res, "protocolVersion", "2024-11-05");
        cJSON *srv = cJSON_CreateObject();
        cJSON_AddStringToObject(srv, "name", "esp32-cam-physical-eyes");
        cJSON_AddStringToObject(srv, "version", "2.0.0");
        cJSON_AddItemToObject(res, "serverInfo", srv);
        cJSON *caps = cJSON_CreateObject();
        cJSON_AddItemToObject(caps, "tools", cJSON_CreateObject());
        cJSON_AddItemToObject(res, "capabilities", caps);
        cJSON_AddItemToObject(resp, "result", res);
    } else if (strcmp(method, "tools/list") == 0) {
        cJSON *res = cJSON_CreateObject();
        cJSON *tools = cJSON_CreateArray();

        cJSON *t1 = cJSON_CreateObject();
        cJSON_AddStringToObject(t1, "name", "observe_hardware_target");
        cJSON_AddStringToObject(t1, "description", "Chụp và phân tích trạng thái mạch điện tử, đèn báo LED (Nguồn, Status, Lỗi, TX/RX), màn hình LCD/OLED/7-Seg và đồng hồ đo ngoài đời thực.");
        cJSON *s1 = cJSON_Parse("{\"type\":\"object\",\"properties\":{\"mode\":{\"type\":\"string\",\"description\":\"debug_board | lcd_oled | led_array | meter_read\"},\"focus\":{\"type\":\"string\",\"description\":\"keep | roi | af | 0..1023\"},\"flash\":{\"type\":\"string\",\"description\":\"auto | on | off\"}}}");
        if (s1) cJSON_AddItemToObject(t1, "inputSchema", s1);
        cJSON_AddItemToArray(tools, t1);

        cJSON *t2 = cJSON_CreateObject();
        cJSON_AddStringToObject(t2, "name", "adjust_camera_optics");
        cJSON_AddStringToObject(t2, "description", "Chuyển đổi mềm hồ sơ mục tiêu (target_preset: water_meter | debug_board | led_status | lcd_oled | led_7seg), lấy nét VCM (0..1023 hoặc quét đỉnh nét ROI), bật/tắt đèn LED phụ PWM chống lóa (0..255) và góc xoay (-180..180).");
        cJSON *s2 = cJSON_Parse("{\"type\":\"object\",\"properties\":{\"target_preset\":{\"type\":\"string\"},\"focus_step\":{\"type\":\"integer\"},\"roi_focus_sweep\":{\"type\":\"boolean\"},\"flash_duty\":{\"type\":\"integer\"},\"rot_deg\":{\"type\":\"integer\"}}}");
        if (s2) cJSON_AddItemToObject(t2, "inputSchema", s2);
        cJSON_AddItemToArray(tools, t2);

        cJSON_AddItemToObject(res, "tools", tools);
        cJSON_AddItemToObject(resp, "result", res);
    } else if (strcmp(method, "tools/call") == 0) {
        cJSON *params = cJSON_GetObjectItem(rpc, "params");
        cJSON *tname_it = params ? cJSON_GetObjectItem(params, "name") : NULL;
        cJSON *args = params ? cJSON_GetObjectItem(params, "arguments") : NULL;
        const char *tname = (tname_it && cJSON_IsString(tname_it)) ? tname_it->valuestring : "";

        cJSON *res = cJSON_CreateObject();
        cJSON *content = cJSON_CreateArray();
        cJSON *item = cJSON_CreateObject();
        cJSON_AddStringToObject(item, "type", "text");

        if (strcmp(tname, "adjust_camera_optics") == 0 && args) {
            app_cam_wake_if_sleeping();
            sensor_t *s = esp_camera_sensor_get();
            cJSON *tp = cJSON_GetObjectItem(args, "target_preset");
            if (tp && cJSON_IsString(tp)) app_apply_target_preset(tp->valuestring);
            cJSON *sw = cJSON_GetObjectItem(args, "roi_focus_sweep");
            if (sw && cJSON_IsTrue(sw)) app_cam_roi_focus_sweep();
            cJSON *fs = cJSON_GetObjectItem(args, "focus_step");
            if (fs && cJSON_IsNumber(fs) && s && s->af_set_manual_position) {
                g_cfg.af_manual_pos = (uint16_t)fs->valueint;
                s->af_set_manual_position(s, g_cfg.af_manual_pos);
            }
            cJSON *fd = cJSON_GetObjectItem(args, "flash_duty");
            if (fd && cJSON_IsNumber(fd)) {
                g_cfg.flash_duty = (uint8_t)fd->valueint;
                app_flash_set_duty(g_cfg.flash_duty);
            }
            cJSON *rd = cJSON_GetObjectItem(args, "rot_deg");
            if (rd && cJSON_IsNumber(rd)) g_cfg.meter_rot_deg = (int16_t)rd->valueint;
            app_config_save();
            cJSON_AddStringToObject(item, "text", "{\"status\":\"ok\",\"msg\":\"Optics, Target Preset & Flash updated\"}");
        } else {
            // Default: observe_hardware_target
            char fn[64] = {0}, fst[64] = {0};
            app_execute_capture_pipeline("debug_board", fn, sizeof(fn), fst, sizeof(fst));
            cJSON_AddStringToObject(item, "text", g_last_server_ai_json);
        }
        cJSON_AddItemToArray(content, item);
        cJSON_AddItemToObject(res, "content", content);
        cJSON_AddItemToObject(resp, "result", res);
    } else {
        cJSON_AddItemToObject(resp, "result", cJSON_CreateObject());
    }

    cJSON_Delete(rpc);
    char *out = cJSON_PrintUnformatted(resp);
    cJSON_Delete(resp);
    esp_err_t ret = httpd_resp_send(req, out, strlen(out));
    free(out);
    return ret;
}
