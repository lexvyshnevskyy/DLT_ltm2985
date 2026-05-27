/*
 * LTM2985 board simulator for NodeMCU (ESP8266)
 *
 * Sends one JSON measurement per line at 230400 baud — same format as the real
 * Arduino LTM firmware expected by ros2_delatometry ltm2985_uart node.
 *
 * Wiring: NodeMCU USB-UART → Pi USB (e.g. /dev/ttyUSB0)
 * Pi config: DELATOMETRY_LTM2985_PORT=/dev/ttyUSB0  DELATOMETRY_LTM2985_BAUDRATE=230400
 *
 * See: src/ltm2985_uart/README.md
 */

#include <Arduino.h>

static const uint32_t SERIAL_BAUD = 230400;
static const uint32_t SEND_INTERVAL_MS = 250;  // 4 Hz (matches typical control loop)

// Live-style temperature: 299.5 … 300.5 K, step 0.1 K, triangle wave
static const int TEMP_MIN_TENTH = 2995;  // 299.5 K
static const int TEMP_MAX_TENTH = 3005;  // 300.5 K
static const int TEMP_STEP_TENTH = 1;    // 0.1 K

static int tempTenthK = TEMP_MIN_TENTH;
static int tempDir = 1;  // +1 ramp up, -1 ramp down

// Delatometry defaults (webui / core): control ch 9, monitor ch 3
static const int CHANNELS[] = {9, 3};
static const size_t CHANNEL_COUNT = sizeof(CHANNELS) / sizeof(CHANNELS[0]);

static float currentTempK() {
  return tempTenthK * 0.1f;
}

static void advanceTemperature() {
  tempTenthK += tempDir * TEMP_STEP_TENTH;
  if (tempTenthK >= TEMP_MAX_TENTH) {
    tempTenthK = TEMP_MAX_TENTH;
    tempDir = -1;
  } else if (tempTenthK <= TEMP_MIN_TENTH) {
    tempTenthK = TEMP_MIN_TENTH;
    tempDir = 1;
  }
}

static void sendTemperatureFrame(int channel, float tempK) {
  // fault:0 = no fault; valid:true — required fields for ltm2985_uart node
  char line[200];
  int n = snprintf(
      line, sizeof(line),
      "{\"channel\":%d,\"type\":\"temperature_K\",\"value\":%.2f,"
      "\"raw_code\":30000,\"sensor_value\":%.2f,\"fault\":0,\"valid\":true}",
      channel, tempK, tempK - 273.15f);
  if (n > 0 && (size_t)n < sizeof(line)) {
    Serial.println(line);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  Serial.println();
  Serial.println(F("LTM simulator ready, live T 299.5-300.5 K step 0.1"));
}

void loop() {
  const float tempK = currentTempK();
  for (size_t i = 0; i < CHANNEL_COUNT; i++) {
    sendTemperatureFrame(CHANNELS[i], tempK);
  }
  advanceTemperature();
  delay(SEND_INTERVAL_MS);
}
