#ifndef _HTTPD_OTA_SYS_H_
#define _HTTPD_OTA_SYS_H_

#include "esp_http_server.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ota_upload_handler(httpd_req_t *req);
esp_err_t ota_pull_handler(httpd_req_t *req);
esp_err_t ota_rollback_handler(httpd_req_t *req);
esp_err_t tasks_handler(httpd_req_t *req);
esp_err_t wg_get_handler(httpd_req_t *req);
esp_err_t wg_post_handler(httpd_req_t *req);

#ifdef __cplusplus
}
#endif

#endif /* _HTTPD_OTA_SYS_H_ */
