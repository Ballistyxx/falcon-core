/*
 * WiFi station. Connects to the configured SSID, retries on failure, and
 * exposes the assigned IP once the DHCP lease arrives.
 */

#ifndef FALCON_WIFI_H_
#define FALCON_WIFI_H_

#include <stdbool.h>

/* Bring up the netif/event stack and start connecting (non-blocking). */
void wifi_init_sta(void);

/* Block until connected or timeout_ms elapses. Returns true if connected. */
bool wifi_wait_connected(int timeout_ms);

/* Latest assigned IPv4 as a string (e.g. "192.168.1.42"); "0.0.0.0" if none. */
const char *wifi_ip_str(void);

/* Current AP signal strength in dBm (negative; stronger is closer to 0).
 * Returns 0 when not associated. */
int wifi_rssi(void);

#endif /* FALCON_WIFI_H_ */
