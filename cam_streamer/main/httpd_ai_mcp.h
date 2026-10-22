#ifndef _HTTPD_AI_MCP_H_
#define _HTTPD_AI_MCP_H_

#include "esp_http_server.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ai_manifest_handler(httpd_req_t *req);
esp_err_t ai_observe_handler(httpd_req_t *req);
esp_err_t mcp_handler(httpd_req_t *req);

#ifdef __cplusplus
}
#endif

#endif /* _HTTPD_AI_MCP_H_ */
