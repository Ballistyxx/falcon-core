/*
 * HTTP + WebSocket server.
 *
 *   GET /stream  MJPEG multipart stream (multiple clients supported)
 *   GET /ws      WebSocket: telemetry out (broadcast) + commands in
 *
 * Inbound command dispatch is table-driven on the JSON "type" field so new
 * message types (e.g. a future "velocity_cmd") can be added without touching
 * the transport layer.
 */

#ifndef FALCON_SERVER_H_
#define FALCON_SERVER_H_

#include "esp_err.h"
#include <stddef.h>

esp_err_t server_start(void);

/* Broadcast a UTF-8 text frame (telemetry JSON) to every connected WS client.
 * Safe to call from a task other than the HTTP server's. */
void server_ws_broadcast(const char *text, size_t len);

#endif /* FALCON_SERVER_H_ */
