# NodeMCU LTM2985 simulator

Arduino sketch that pretends to be the LTM UART board and streams **live-style** temperature JSON lines: **299.5–300.5 K** in **0.1 K** steps (triangle wave).

## Protocol

One JSON object per line, newline-terminated, **230400** baud:

```json
{"channel":9,"type":"temperature_K","value":299.80,"raw_code":30000,"sensor_value":26.65,"fault":0,"valid":true}
```

(`value` ramps 299.5 → 300.5 → 299.5 … each 0.1 K per send cycle.)

Channels sent each cycle:

| Channel | Use in Delatometry |
|--------:|--------------------|
| 9 | Control (`ltm_control_channel`) |
| 3 | Monitor (`ltm_monitor_channel`) |

## Flash (Arduino IDE)

1. Board: **NodeMCU 1.0 (ESP-12E Module)** (or generic ESP8266).
2. Upload speed: 115200 (default).
3. Open `ltm_nodemcu_simulator.ino` → Upload.
4. After upload, the sketch runs at **230400** on `Serial` (USB).

To change the range or step, edit `TEMP_MIN_TENTH`, `TEMP_MAX_TENTH`, and `TEMP_STEP_TENTH` in the `.ino` file (tenths of kelvin, e.g. `3005` = 300.5 K).

## Pi setup

```bash
# Find port (often /dev/ttyUSB0 or ttyACM0)
ls -l /dev/ttyUSB*

# In /etc/default/delatometry or WebUI → Configuration → LTM:
#   DELATOMETRY_LTM2985_PORT=/dev/ttyUSB0
#   DELATOMETRY_LTM2985_BAUDRATE=230400

sudo systemctl restart delatometry-ltm2985.service
ros2 topic echo /ltm2985/measurement
```

You should see `value` stepping between **299.5** and **300.5** and `type: temperature_K` on channels 3 and 9.

## Notes

- Uses only `Serial` (USB UART). No WiFi.
- `sensor_value` is °C (300 K → 26.85 °C) for display compatibility.
- Increase `SEND_INTERVAL_MS` to slow the stream (e.g. `1000` for 1 Hz).
