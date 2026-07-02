# Falcon Core – Implementation Prompt

Build the firmware (ESP-IDF v5+, C) and web dashboard (HTML/CSS/JS, served locally) for an autonomous blimp flight controller. The ESP32-S3 streams camera video, reads sensors, runs motor control, and communicates with a laptop dashboard over WiFi. The dashboard provides live telemetry, a camera feed, and keyboard-based manual flight control.

---

## Hardware Overview

**MCU:** ESP32-S3-WROOM-1 (dual-core, 8MB flash, PSRAM via octal SPI)

**Confirmed GPIO Pinout:**

| Function         | GPIO(s)                              | Notes                          |
|------------------|--------------------------------------|--------------------------------|
| I2C SDA          | IO8                                  | Shared bus: IMU, mag, ToF      |
| I2C SCL          | IO18                                 |                                |
| Camera SCCB SDA  | IO1                                  | Separate I2C for OV2640 config |
| Camera SCCB SCL  | IO2                                  |                                |
| Camera DVP data  | IO9(D8), IO10(D7), IO11(D6), IO12(D2), IO13(D5), IO14(D3), IO21(D4), IO40(D9) |    |
| Camera HREF      | IO41                                 |                                |
| Camera VSYNC     | IO42                                 |                                |
| Camera PCLK      | IO38                                 |                                |
| Camera XCLK      | IO39                                 | Generate 20MHz via LEDC        |
| Motor 1 IN1/IN2  | IO17 / IO6                           | Horizontal left                |
| Motor 2 IN1/IN2  | IO3 / IO47                           | Horizontal right               |
| Motor 3 IN1/IN2  | IO5 / IO4                            | Vertical (altitude)            |
| Motor 4 IN1/IN2  | IO15 / IO16                          | Reserved (future use)          |
| Battery ADC      | IO7                                  | Resistive divider, ADC1 channel|
| Status LED       | IO48                                 | Red LED, 1kΩ to GND            |
| USB D-/D+        | IO19 / IO20                          | USB-Serial-JTAG                |
| UART0 TX/RX      | IO43 / IO44                          | Debug output                   |

**Motor Drivers:** 4× DRV8212P (bidirectional H-bridge). Each has IN1 and IN2 pins. PWM on IN1 with IN2 LOW = forward. PWM on IN2 with IN1 LOW = reverse. Both LOW = coast. Both HIGH = brake. Use ESP-IDF LEDC PWM at 20kHz.

**Sensors (all on shared I2C bus at IO8/IO18):**

| Sensor     | Type                | I2C Address | Key Details                                    |
|------------|---------------------|-------------|------------------------------------------------|
| BMI270     | 6-axis IMU          | 0x68        | Accel + gyro. Requires config load on init.    |
| MMC5633NJL | 3-axis magnetometer | 0x30        | SET/RESET for offset, continuous mode.         |
| VL53L5CX   | Multizone ToF       | 0x29        | 8×8 ranging grid, I2C. Use ST's ULD driver.   |

**Camera:** OV2640 via DVP (parallel interface). PWDN tied low, RESETB tied high. 24-pin 0.5mm FFC. Use `esp_camera` component from Espressif's registry.

---

## Firmware Architecture (ESP-IDF, C)

### WiFi

Connect in STA mode to a configured network (SSID/password stored in `sdkconfig` or hardcoded for now). Log the assigned IP address on boot so the user knows where to point the dashboard.

### HTTP + WebSocket Server

Use `esp_http_server`. Register these endpoints:

- `GET /stream` — MJPEG stream. Each connected client receives `multipart/x-mixed-replace` with boundary-delimited JPEG frames from the camera. Multiple simultaneous clients must be supported.
- `GET /ws` — WebSocket upgrade. Single endpoint for all bidirectional real-time data.

### WebSocket Protocol

Use JSON messages. Every message has a `"type"` field.

**ESP32 → Dashboard (telemetry, ~20Hz):**

```json
{
  "type": "telemetry",
  "ts": 123456,
  "imu": {
    "ax": 0.01, "ay": -0.02, "az": 9.78,
    "gx": 0.5, "gy": -0.3, "gz": 0.1
  },
  "mag": { "x": 25.3, "y": -10.1, "z": 42.0, "heading": 127.5 },
  "tof": { "distance_mm": 1200 },
  "battery": { "voltage": 3.85, "percent": 72 },
  "attitude": { "roll": 1.2, "pitch": -0.5, "yaw": 127.5 },
  "motors": { "m1": 0, "m2": 0, "m3": 0, "m4": 0 }
}
```

**Dashboard → ESP32 (commands):**

```json
{ "type": "motor_cmd", "m1": 0.5, "m2": 0.5, "m3": 0.0, "m4": 0.0 }
```

Motor values are floats from -1.0 (full reverse) to +1.0 (full forward). The firmware maps these to LEDC duty cycle and direction on IN1/IN2.

**Future extension:** A `"type": "velocity_cmd"` message will be added later for optical-flow-based velocity control. Design the command dispatch so new message types can be added without restructuring.

### FreeRTOS Tasks

| Task               | Core | Priority | Rate   | Description                                                    |
|--------------------|------|----------|--------|----------------------------------------------------------------|
| `sensor_task`      | 0    | 5        | 50Hz   | Read BMI270 (accel+gyro) and MMC5633NJL (mag) via I2C.         |
| `fusion_task`      | 0    | 5        | 50Hz   | Madgwick or complementary filter. Compute attitude + heading.  |
| `motor_task`       | 0    | 6        | 50Hz   | Read latest command from shared state, set LEDC duty cycles.   |
| `tof_task`         | 0    | 4        | 10Hz   | Read VL53L5CX ranging data (center zone distance).             |
| `battery_task`     | 0    | 3        | 1Hz    | Read ADC on IO7, compute voltage from divider ratio.           |
| `camera_task`      | 1    | 5        | —      | Capture frames via `esp_camera`, push to frame buffer.         |
| `telemetry_task`   | 0    | 4        | 20Hz   | Serialize shared state to JSON, send to all WS clients.        |
| HTTP server        | any  | 5        | event  | Handles `/stream` and `/ws`. Started once at init.             |

### Shared State

A single struct protected by a FreeRTOS mutex. All tasks read/write through short critical sections (copy in, copy out).

```c
typedef struct {
    // Sensor data (written by sensor_task)
    float accel[3];
    float gyro[3];
    float mag[3];

    // Derived data (written by fusion_task)
    float roll, pitch, yaw;
    float heading_deg;

    // ToF (written by tof_task)
    uint16_t altitude_mm;

    // Battery (written by battery_task)
    float battery_voltage;
    uint8_t battery_percent;

    // Motor commands (written by WS receive handler)
    float motor_cmd[4];     // -1.0 to +1.0

    // Motor actual output (written by motor_task)
    float motor_output[4];
} falcon_state_t;
```

### Sensor Implementation Notes

**BMI270:** Requires a microcode config file upload after power-on before it produces data. The config binary is available from Bosch's BMI270 driver repo. Use burst I2C writes. After config, set accel to ±2g and gyro to ±500°/s at 100Hz ODR.

**MMC5633NJL:** Issue a SET then RESET command to initialize. Configure continuous measurement mode at 50Hz with auto SET/RESET enabled. Read XYZ registers. For now, heading = `atan2(y, x)` converted to degrees (tilt compensation will come later with fusion). Do not attempt calibration yet — just output raw values and heading.

**VL53L5CX:** Use ST's Ultra Lite Driver (ULD). Initialize, set resolution to 4×4 or 8×8, start ranging in continuous mode. For telemetry, report the center zone distance as altitude. The full zone grid will be used later.

**Battery ADC:** Use `adc_oneshot` API on ADC1. Apply the resistor divider ratio to convert raw reading to actual battery voltage. Map voltage to percentage using a simple linear approximation (3.0V = 0%, 4.2V = 100%).

### Motor Control

Initialize 4 LEDC channels at 20kHz. For each motor, two GPIO pins (IN1, IN2) map to forward and reverse:

```
if cmd > 0:  IN1 = PWM at |cmd| duty, IN2 = LOW   (forward)
if cmd < 0:  IN1 = LOW, IN2 = PWM at |cmd| duty    (reverse)
if cmd == 0: IN1 = LOW, IN2 = LOW                   (coast)
```

Duty cycle = `|cmd| * max_duty`. Clamp to a configurable maximum (start at 50% for safety during testing).

### Status LED

Blink patterns on IO48 to indicate state:
- Slow blink (1Hz): Connecting to WiFi
- Solid on: Connected, running
- Fast blink (5Hz): Error state

### sdkconfig.defaults

```
CONFIG_IDF_TARGET="esp32s3"
CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y
CONFIG_ESPTOOLPY_FLASHMODE_DIO=y
CONFIG_ESPTOOLPY_FLASHFREQ_80M=y
CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y
CONFIG_ESP_WIFI_SSID="Home-TMobile5g"
CONFIG_ESP_WIFI_PASSWORD="PSU4LIFE!"
CONFIG_HTTPD_MAX_REQ_HDR_LEN=1024
CONFIG_HTTPD_WS_SUPPORT=y
```

### Project Structure

```
src/
├── CMakeLists.txt
├── sdkconfig.defaults
├── main/
│   ├── CMakeLists.txt
│   ├── main.c              # app_main: init all subsystems, start tasks
│   ├── wifi.c / wifi.h      # STA connection, event handling
│   ├── server.c / server.h  # HTTP + WS server, MJPEG streaming
│   ├── camera.c / camera.h  # esp_camera init and frame capture
│   ├── sensors.c / sensors.h # BMI270, MMC5633NJL, VL53L5CX drivers
│   ├── fusion.c / fusion.h  # Attitude estimation (Madgwick filter)
│   ├── motors.c / motors.h  # LEDC PWM setup and motor command execution
│   ├── battery.c / battery.h # ADC reading, voltage/percent calculation
│   ├── state.c / state.h    # falcon_state_t definition, mutex helpers
│   └── led.c / led.h        # Status LED blink patterns
```

---

## Web Dashboard (HTML/CSS/JS)

A single `index.html` file served locally on the laptop (not from the ESP32). It connects to the blimp over the local network.

### Connection

On load, prompt for or use a hardcoded blimp IP. Open:
- WebSocket to `ws://<ip>/ws` for telemetry and commands.
- `<img>` tag with `src="http://<ip>/stream"` for the MJPEG video feed.

### Layout

A single-page dashboard with these sections:

1. **Camera feed** — large, dominant. The MJPEG `<img>` element. Aspect ratio 4:3 or 16:9 depending on camera resolution.

2. **Compass** — a circular compass rose rendered in SVG or canvas. The needle rotates to show `heading_deg` from telemetry. Display heading in degrees numerically below it. Cardinal directions (N/S/E/W) labeled around the ring.

3. **Attitude indicator** — show roll and pitch visually (an artificial horizon or simple tilt bars). Yaw can share the compass.

4. **Sensor readouts** — a panel showing:
   - Battery voltage and percentage (with a colored bar: green > 50%, yellow 20-50%, red < 20%)
   - Altitude (ToF distance in mm or cm)
   - Raw accelerometer XYZ
   - Raw gyroscope XYZ
   - Raw magnetometer XYZ
   - Motor outputs M1–M4 (as bars or values)

5. **Connection status** — indicator showing WebSocket state (connected/disconnected/reconnecting).

6. **Controls legend** — small reference showing the WASD/QE key bindings.

### Keyboard Controls (WASD/QE)

Capture `keydown` and `keyup` events. While a key is held, send the corresponding motor command at 10Hz. On key release, send zero for that axis.

**Key mapping (differential thrust for translation + yaw):**

| Key | Action         | Motor effect                                           |
|-----|----------------|--------------------------------------------------------|
| W   | Forward        | M1 = +throttle, M2 = +throttle (both horizontal fwd)  |
| S   | Backward       | M1 = -throttle, M2 = -throttle (both horizontal rev)  |
| A   | Yaw left       | M1 = -throttle, M2 = +throttle (differential)         |
| D   | Yaw right      | M1 = +throttle, M2 = -throttle (differential)         |
| Q   | Descend        | M3 = -throttle (vertical motor down)                  |
| E   | Ascend         | M3 = +throttle (vertical motor up)                    |

Throttle magnitude is a fixed value (start at 0.4). Multiple keys can be held simultaneously — sum the contributions per motor and clamp to [-1, 1]. M4 is always 0 for now.

Send commands as: `{ "type": "motor_cmd", "m1": 0.4, "m2": 0.4, "m3": 0.0, "m4": 0.0 }`

When no keys are held, send `{ "type": "motor_cmd", "m1": 0, "m2": 0, "m3": 0, "m4": 0 }` once, then stop sending.

### Telemetry Handling

Parse incoming `"type": "telemetry"` messages. Update all dashboard elements at the rate they arrive (~20Hz). Use `requestAnimationFrame` for smooth compass/attitude animation. Display a "stale data" warning if no telemetry arrives for >2 seconds.

### Reconnection

If the WebSocket drops, attempt reconnection every 3 seconds. Show connection state in the status indicator. The MJPEG `<img>` will stall on disconnect — provide a reload button or auto-retry via `onerror`.

### Styling

Dark theme. Monospace font for numeric readouts. The dashboard should be functional and readable, not flashy. Minimal dependencies — vanilla JS, no frameworks. CSS grid or flexbox for layout.

---

## What NOT to Implement Yet

- Optical flow (will be a separate Python script connecting to the same WebSocket later)
- Magnetometer calibration (raw values only for now)
- Tilt-compensated heading (use simple atan2 for now)
- Autonomous flight modes
- PID controllers
- AprilTag detection
- NVS storage for calibration data
- OTA firmware updates

Design all interfaces (WebSocket protocol, shared state struct, task architecture) so these can be added without restructuring existing code.