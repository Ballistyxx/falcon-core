#include "server.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "state.h"
#include "motors.h"
#include "magcal.h"

static const char *TAG = "server";

/* Dashboard HTML embedded at build time (see main/CMakeLists.txt EMBED_TXTFILES).
 * Served over plain HTTP from the board so there's no HTTPS mixed-content
 * upgrade on the ws:// and http:// stream sub-resources. */
extern const char index_html_start[] asm("_binary_index_html_start");

static httpd_handle_t s_httpd;          /* control + telemetry, port 80 */
static httpd_handle_t s_stream_httpd;   /* MJPEG stream, port 81 */

/* Connected WebSocket client sockets. */
#define MAX_WS_CLIENTS 4
static int s_ws_fds[MAX_WS_CLIENTS];
static SemaphoreHandle_t s_ws_lock;

/* ---- MJPEG stream ------------------------------------------------------- */
#define PART_BOUNDARY "frame"
static const char *STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) {
        return res;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "30");

    char part_buf[64];
    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGW(TAG, "frame capture failed");
            res = ESP_FAIL;
            break;
        }

        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (res == ESP_OK) {
            int hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, fb->len);
            res = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (res == ESP_OK) {
            res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        }
        esp_camera_fb_return(fb);

        if (res != ESP_OK) {
            break;   /* client disconnected */
        }
    }
    return res;
}

/* ---- Dashboard page ----------------------------------------------------- */
static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, index_html_start, HTTPD_RESP_USE_STRLEN);
}

/* ---- WebSocket ---------------------------------------------------------- */
static void ws_add_client(int fd)
{
    xSemaphoreTake(s_ws_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == fd) {
            goto done;          /* already tracked */
        }
    }
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] < 0) {
            s_ws_fds[i] = fd;
            ESP_LOGI(TAG, "ws client %d connected", fd);
            goto done;
        }
    }
    ESP_LOGW(TAG, "ws client table full, dropping %d", fd);
done:
    xSemaphoreGive(s_ws_lock);
}

static void ws_remove_client(int fd)
{
    xSemaphoreTake(s_ws_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_ws_fds[i] == fd) {
            s_ws_fds[i] = -1;
            ESP_LOGI(TAG, "ws client %d removed", fd);
        }
    }
    xSemaphoreGive(s_ws_lock);
}

/* Minimal JSON helpers — the inbound command messages are small and flat. */
static bool json_type_is(const char *s, const char *value)
{
    const char *p = strstr(s, "\"type\"");
    if (!p) {
        return false;
    }
    p = strchr(p, ':');
    if (!p) {
        return false;
    }
    p = strchr(p, '"');
    if (!p) {
        return false;
    }
    p++;
    size_t n = strlen(value);
    return strncmp(p, value, n) == 0 && p[n] == '"';
}

static bool json_get_number(const char *s, const char *key, float *out)
{
    char pat[16];
    int len = snprintf(pat, sizeof(pat), "\"%s\"", key);
    if (len <= 0 || len >= (int)sizeof(pat)) {
        return false;
    }
    const char *p = strstr(s, pat);
    if (!p) {
        return false;
    }
    p = strchr(p + len, ':');
    if (!p) {
        return false;
    }
    *out = strtof(p + 1, NULL);
    return true;
}

/* Command dispatch. Extend by adding branches keyed on "type". */
static void dispatch_command(const char *json)
{
    if (json_type_is(json, "motor_cmd")) {
        float cmd[4] = {0};
        const char *keys[4] = { "m1", "m2", "m3", "m4" };
        for (int i = 0; i < 4; i++) {
            json_get_number(json, keys[i], &cmd[i]);
        }
        state_set_motor_cmd(cmd);
    } else if (json_type_is(json, "mag_cal")) {
        /* Start from the current calibration so any key the dashboard omits
         * keeps its prior value. */
        magcal_t c;
        magcal_get(&c);
        json_get_number(json, "ox", &c.offset[0]);
        json_get_number(json, "oy", &c.offset[1]);
        json_get_number(json, "oz", &c.offset[2]);
        json_get_number(json, "sx", &c.scale[0]);
        json_get_number(json, "sy", &c.scale[1]);
        json_get_number(json, "sz", &c.scale[2]);
        magcal_set(&c);
    }
    /* Future: else if (json_type_is(json, "velocity_cmd")) { ... } */
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);

    /* The server completes the WS handshake internally; depending on the IDF
     * version the handler may or may not be invoked for it. Register the
     * client here, and again on the first data frame, so push-only dashboards
     * (which open the socket and just listen) still get telemetry. The
     * dashboard also sends a one-shot "hello" on open to force this path. */
    if (req->method == HTTP_GET) {
        ws_add_client(fd);
        return ESP_OK;
    }
    ws_add_client(fd);

    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_TEXT };
    /* First call with len=0 to learn the payload length. */
    esp_err_t ret = httpd_ws_recv_frame(req, &frame, 0);
    if (ret != ESP_OK) {
        return ret;
    }
    if (frame.len == 0 || frame.len > 512) {
        return ESP_OK;
    }

    uint8_t buf[513];
    frame.payload = buf;
    ret = httpd_ws_recv_frame(req, &frame, frame.len);
    if (ret != ESP_OK) {
        return ret;
    }

    if (frame.type == HTTPD_WS_TYPE_CLOSE) {
        ws_remove_client(fd);
        return ESP_OK;
    }
    if (frame.type == HTTPD_WS_TYPE_TEXT) {
        buf[frame.len] = '\0';
        dispatch_command((const char *)buf);
    }
    return ESP_OK;
}

void server_ws_broadcast(const char *text, size_t len)
{
    if (!s_httpd) {
        return;
    }
    int to_drop[MAX_WS_CLIENTS];
    int ndrop = 0;

    xSemaphoreTake(s_ws_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        int fd = s_ws_fds[i];
        if (fd < 0) {
            continue;
        }
        httpd_ws_frame_t frame = {
            .type = HTTPD_WS_TYPE_TEXT,
            .payload = (uint8_t *)text,
            .len = len,
        };
        if (httpd_ws_send_frame_async(s_httpd, fd, &frame) != ESP_OK) {
            to_drop[ndrop++] = fd;
        }
    }
    xSemaphoreGive(s_ws_lock);

    for (int i = 0; i < ndrop; i++) {
        ws_remove_client(to_drop[i]);
    }
}

/* ---- Lifecycle ---------------------------------------------------------- */
esp_err_t server_start(void)
{
    s_ws_lock = xSemaphoreCreateMutex();
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        s_ws_fds[i] = -1;
    }

    /* Main control/telemetry server (port 80). */
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_open_sockets = 7;
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    config.core_id = tskNO_AFFINITY;

    esp_err_t err = httpd_start(&s_httpd, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: 0x%x", err);
        return err;
    }

    httpd_uri_t root_uri = {
        .uri = "/", .method = HTTP_GET,
        .handler = root_handler, .user_ctx = NULL,
    };
    httpd_uri_t ws_uri = {
        .uri = "/ws", .method = HTTP_GET,
        .handler = ws_handler, .user_ctx = NULL,
        .is_websocket = true,
    };
    httpd_register_uri_handler(s_httpd, &root_uri);
    httpd_register_uri_handler(s_httpd, &ws_uri);

    /* The MJPEG stream handler runs an infinite loop that never returns, which
     * blocks esp_http_server's single handler task and starves the WebSocket.
     * Give it its own server instance on port 81 (with a distinct control
     * port) so video and telemetry run independently — the same split the
     * espressif camera web-server example uses. */
    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port = 81;
    scfg.ctrl_port = config.ctrl_port + 1;
    scfg.max_open_sockets = 3;
    scfg.lru_purge_enable = true;
    scfg.stack_size = 8192;
    scfg.core_id = tskNO_AFFINITY;

    err = httpd_start(&s_stream_httpd, &scfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "stream server start failed: 0x%x (video disabled)", err);
    } else {
        httpd_uri_t stream_uri = {
            .uri = "/stream", .method = HTTP_GET,
            .handler = stream_handler, .user_ctx = NULL,
        };
        httpd_register_uri_handler(s_stream_httpd, &stream_uri);
    }

    ESP_LOGI(TAG, "servers started (:80 /ws, :81 /stream)");
    return ESP_OK;
}
