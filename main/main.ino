/**
 * Project:
 * Platform: ESP8266 (NodeMCU)
 * Description: HC-SR04 distance sensor + PWM LED/buzzer via web sliders.
 *              Fully asynchronous — no delay() or blocking pulseIn().
 *              Echo is measured via hardware interrupt (IRAM_ATTR).
 *              Sensor and pump run on independent millis() timers.
 */

#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"

// ============================================================
// WIFI CREDENTIALS
// ============================================================

const char* ssid     = "SUA_REDE";
const char* password = "SUA_SENHA";

// ============================================================
// WEB SERVER
// ============================================================

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

String message = "";
String sliderValue1 = "0";   // LED  (PUMP_PWM_PIN)
String sliderValue2 = "0";   // Buzzer

uint8_t ledPwm    = 0;
uint8_t buzzerPwm = 0;

String distanceCm      = "--";
String distanceStatus  = "waiting";   // waiting | ok | timeout

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define HC_SR04_TRIG_PIN  D6    // GPIO12 — HC-SR04 Trigger
#define HC_SR04_ECHO_PIN  D7    // GPIO13 — HC-SR04 Echo (must support interrupts)
#define PUMP_PWM_PIN      D1    // GPIO5  — PWM Output
#define BUZZER_PWM_PIN    D2    // GPIO4  — Buzzer

// ============================================================
// PWM SETTINGS
// ============================================================

#define PWM_FREQ   1000   // PWM frequency in Hz
#define PWM_RANGE   255   // Resolution: 8-bit (0–255)

// ============================================================
// HC-SR04 SENSOR SETTINGS
// ============================================================

#define SOUND_SPEED      0.034f   // cm/µs (speed of sound in air)
#define TRIG_LOW_DELAY       2    // µs — line-clear time before trigger pulse
#define TRIG_HIGH_PULSE     10    // µs — trigger pulse duration
#define ECHO_TIMEOUT_MS     40    // ms  — max wait for echo before discarding

// ============================================================
// TIMING
// ============================================================

#define SENSOR_INTERVAL_MS   500   // ms between sensor readings

// ============================================================
// SENSOR STATE MACHINE
// ============================================================

enum SensorState {
  SENSOR_IDLE,       // waiting for the next scheduled reading
  SENSOR_TRIGGERED   // trigger sent, waiting for echo to complete
};

volatile unsigned long echoStartUs  = 0;   // micros() when echo went HIGH
volatile long          echoDuration = -1;  // µs of echo pulse (-1 = no result yet)

SensorState   sensorState = SENSOR_IDLE;
unsigned long sensorTimer = 0;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

void IRAM_ATTR echoISR();
uint8_t percentToPWM(uint8_t percent);
void initFS();
void initWiFi();
void initWebServer();
void initWebSocket();
void updateSensor();
void applyPwmOutputs();
String getStateJson();
void handleWebSocketMessage(void* arg, uint8_t* data, size_t len);
void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len);

// ============================================================
// INTERRUPT SERVICE ROUTINE — Echo pin (CHANGE)
// ============================================================

/**
 * Captures the echo pulse duration without blocking the main loop.
 * Runs in IRAM to avoid cache-miss crashes on ESP8266.
 */
void IRAM_ATTR echoISR() {
  if (digitalRead(HC_SR04_ECHO_PIN) == HIGH) {
    echoStartUs  = micros();   // rising edge — start timing
  } else {
    echoDuration = (long)(micros() - echoStartUs);   // falling edge — pulse done
  }
}

// ============================================================
// PWM HELPERS
// ============================================================

/**
 * Maps a percentage (0–100) to the PWM duty range (0–PWM_RANGE).
 * Values outside 0–100 are clamped.
 */
uint8_t percentToPWM(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  return (uint8_t)((percent * PWM_RANGE + 50) / 100);
}

String getStateJson() {
  return "{\"sliderValue1\":\"" + sliderValue1 +
         "\",\"sliderValue2\":\"" + sliderValue2 +
         "\",\"distance\":\"" + distanceCm +
         "\",\"distanceStatus\":\"" + distanceStatus + "\"}";
}

void notifyClients(const String& values) {
  ws.textAll(values);
}

void applyPwmOutputs() {
  analogWrite(PUMP_PWM_PIN, ledPwm);
  analogWrite(BUZZER_PWM_PIN, buzzerPwm);
}

void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
    return;
  }

  data[len] = 0;
  message = (char*)data;

  if (message.indexOf("1s") >= 0) {
    sliderValue1 = message.substring(2);
    ledPwm = percentToPWM((uint8_t)sliderValue1.toInt());
    Serial.print("[LED] ");
    Serial.print(sliderValue1);
    Serial.println("%");
    notifyClients(getStateJson());
  } else if (message.indexOf("2s") >= 0) {
    sliderValue2 = message.substring(2);
    buzzerPwm = percentToPWM((uint8_t)sliderValue2.toInt());
    Serial.print("[Buzzer] ");
    Serial.print(sliderValue2);
    Serial.println("%");
    notifyClients(getStateJson());
  } else if (strcmp((char*)data, "getValues") == 0) {
    notifyClients(getStateJson());
  }
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket #%u from %s\n", client->id(),
                    client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    default:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
}

// ============================================================
// WIFI
// ============================================================

void initWiFi() {
  Serial.println("\n=== WiFi Started ===");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("WiFi connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void initFS() {
  Serial.println("\n=== LittleFS Started ===");
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
    return;
  }
  Serial.println("LittleFS mounted");
}

void initWebServer() {
  Serial.println("\n=== Web Server Started ===");
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/index.html", "text/html");
  });
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  Serial.println("Web server started on port 80");
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);

  initWiFi();
  initFS();
  initWebSocket();
  initWebServer();

  // Ultrasonic sensor
  pinMode(HC_SR04_TRIG_PIN, OUTPUT);
  pinMode(HC_SR04_ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HC_SR04_ECHO_PIN), echoISR, CHANGE);

  // PWM outputs (LED + buzzer — valores definidos pela interface web)
  pinMode(PUMP_PWM_PIN, OUTPUT);
  pinMode(BUZZER_PWM_PIN, OUTPUT);
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  applyPwmOutputs();

  sensorTimer = millis();

  Serial.println("\n=== System Started ===");
}

// ============================================================
// MAIN LOOP — never blocks
// ============================================================

void loop() {
  updateSensor();
  applyPwmOutputs();
  ws.cleanupClients();
}

// ============================================================
// SENSOR UPDATE — non-blocking state machine
// ============================================================

/**
 * Manages the HC-SR04 read cycle without blocking.
 *
 * SENSOR_IDLE      → waits for SENSOR_INTERVAL_MS, then fires trigger pulse.
 * SENSOR_TRIGGERED → waits for ISR to fill echoDuration, then calculates distance.
 *                    Falls back to SENSOR_IDLE on ECHO_TIMEOUT_MS.
 */
void updateSensor() {
  unsigned long now = millis();

  switch (sensorState) {

    // ---------------------------------------------------------
    case SENSOR_IDLE:
      if (now - sensorTimer >= SENSOR_INTERVAL_MS) {
        echoDuration = -1;   // clear previous result

        // Send trigger pulse (~12 µs total — negligible blocking time)
        digitalWrite(HC_SR04_TRIG_PIN, LOW);
        delayMicroseconds(TRIG_LOW_DELAY);
        digitalWrite(HC_SR04_TRIG_PIN, HIGH);
        delayMicroseconds(TRIG_HIGH_PULSE);
        digitalWrite(HC_SR04_TRIG_PIN, LOW);

        sensorTimer = now;
        sensorState = SENSOR_TRIGGERED;
      }
      break;

    // ---------------------------------------------------------
    case SENSOR_TRIGGERED:
      // ISR has filled echoDuration on the falling edge
      if (echoDuration >= 0) {
        float distance = echoDuration * SOUND_SPEED / 2.0f;

        distanceCm = String(distance, 1);
        distanceStatus = "ok";

        Serial.print("[HC-SR04] Distance: ");
        Serial.print(distance, 1);
        Serial.println(" cm");

        notifyClients(getStateJson());

        sensorTimer = millis();
        sensorState = SENSOR_IDLE;
        break;
      }

      // Safety timeout — discard if echo never arrived
      if (now - sensorTimer > ECHO_TIMEOUT_MS) {
        distanceCm = "--";
        distanceStatus = "timeout";

        Serial.println("[HC-SR04] Timeout — no echo received");
        notifyClients(getStateJson());

        sensorTimer = millis();
        sensorState = SENSOR_IDLE;
      }
      break;
  }
}
