/**
 * Project: Controle PID de Nível de Água
 * Platform: ESP8266 (NodeMCU) + HC-SR04 (topo) + Bomba PWM + Buzzer
 *
 * Arquitetura modular em main.ino com structs coesas:
 *   SensorFilter   — filtragem EMA do nível
 *   PIDController  — cálculo PID com anti-windup
 *   BuzzerAlert    — alerta progressivo (opera na distância bruta)
 *   ModeManager    — alternância Manual/Auto com bumpless transfer
 *
 * Não-bloqueante: leitura do HC-SR04 via state machine + ISR no echo.
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

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define HC_SR04_TRIG_PIN  D6    // GPIO12 — HC-SR04 Trigger
#define HC_SR04_ECHO_PIN  D7    // GPIO13 — HC-SR04 Echo (must support interrupts)
#define PUMP_PWM_PIN      D1    // GPIO5  — Bomba (PWM)
#define BUZZER_PWM_PIN    D2    // GPIO4  — Buzzer (PWM)

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
// PARÂMETROS FÍSICOS DO TANQUE
// ============================================================

#define TANK_HEIGHT        15.5f   // cm — altura útil do cilindro
#define SENSOR_TO_BOTTOM   17.5f   // cm — distância sensor → fundo
#define LEVEL_MAX          TANK_HEIGHT

// ============================================================
// TIMING
// ============================================================

#define SENSOR_INTERVAL_MS   500   // ms entre leituras / passo do PID / broadcast WS

// ============================================================
// SENSOR STATE MACHINE
// ============================================================

enum SensorState {
  SENSOR_IDLE,       // aguardando próxima leitura agendada
  SENSOR_TRIGGERED   // trigger enviado, aguardando ISR completar o echo
};

volatile unsigned long echoStartUs  = 0;   // micros() na borda de subida
volatile long          echoDuration = -1;  // µs do pulso de echo (-1 = sem resultado)

SensorState   sensorState = SENSOR_IDLE;
unsigned long sensorTimer = 0;

// ============================================================
// MODELOS DE DADOS — STRUCTS
// ============================================================

// --- 3.1 Filtro EMA do sensor --------------------------------
struct SensorFilter {
  float alpha;        // fator de suavização (0..1)
  float filtered;     // valor filtrado atual
  bool  initialized;
};

SensorFilter levelFilter = { 0.3f, 0.0f, false };

// --- 3.2 Controlador PID -------------------------------------
struct PIDController {
  float Kp, Ki, Kd;            // ganhos ajustáveis pelo frontend
  float setpoint;              // altura desejada (cm)
  float integral;              // acúmulo do termo integral
  float prevError;             // erro anterior
  unsigned long lastTime;      // timestamp do último cálculo (millis)
  float outputMin;             // limite inferior da saída (%)
  float outputMax;             // limite superior da saída (%)
  float integralMax;           // limite anti-windup do integral
  bool  initialized;
};

PIDController pid = {
  /* Kp */          5.0f,
  /* Ki */          0.5f,
  /* Kd */          0.0f,
  /* setpoint */    0.0f,
  /* integral */    0.0f,
  /* prevError */   0.0f,
  /* lastTime */    0,
  /* outputMin */   0.0f,
  /* outputMax */   100.0f,
  /* integralMax */ 50.0f,
  /* initialized */ false
};

// --- 3.3 Buzzer de alerta progressivo ------------------------
struct BuzzerAlert {
  float topStart;      // 6.0 cm — distância bruta onde inicia alerta de água alta
  float topMax;        // 4.0 cm — alerta máximo (água muito alta)
  float bottomStart;   // 15.0 cm — distância bruta onde inicia alerta de água baixa
  float bottomMax;     // 18.0 cm — alerta máximo (água muito baixa)
  uint8_t pwmPercent;  // 0–100 — saída atual em % (para broadcast/log)
};

BuzzerAlert buzzer = { 6.0f, 4.0f, 15.0f, 18.0f, 0 };

// --- 3.4 Gerenciamento de modo -------------------------------
enum SystemMode { MODE_MANUAL, MODE_AUTO };

struct ModeManager {
  SystemMode current;
  uint8_t    manualPwmPercent;   // 0–100 — slider manual
};

ModeManager mode = { MODE_MANUAL, 0 };

// ============================================================
// ESTADO DERIVADO (para broadcast)
// ============================================================

float    rawDistanceCm = -1.0f;     // última leitura bruta válida (cm)
float    waterHeightCm = 0.0f;      // nível filtrado (cm)
uint8_t  pumpPwmPercent = 0;        // saída final da bomba (0–100%)
bool     hasValidReading = false;
const char* sensorStatus = "waiting";   // waiting | ok | timeout

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================

void IRAM_ATTR echoISR();
uint8_t percentToPWM(float percent);
float clampf(float v, float lo, float hi);

void initFS();
void initWiFi();
void initWebServer();
void initWebSocket();
void updateSensor();
void applyPwmOutputs();

float filterUpdate(SensorFilter& f, float reading);
void  pidReset(PIDController& c, float currentError, unsigned long now);
float pidCompute(PIDController& c, float pv, unsigned long now, bool& saturated);
uint8_t buzzerCompute(const BuzzerAlert& b, float rawDistance);
void  setMode(ModeManager& m, SystemMode target);

String getStateJson();
void notifyClients(const String& msg);
void handleWebSocketMessage(void* arg, uint8_t* data, size_t len);
void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len);

// ============================================================
// INTERRUPT SERVICE ROUTINE — Echo pin (CHANGE)
// ============================================================

void IRAM_ATTR echoISR() {
  if (digitalRead(HC_SR04_ECHO_PIN) == HIGH) {
    echoStartUs  = micros();
  } else {
    echoDuration = (long)(micros() - echoStartUs);
  }
}

// ============================================================
// HELPERS
// ============================================================

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

uint8_t percentToPWM(float percent) {
  percent = clampf(percent, 0.0f, 100.0f);
  return (uint8_t)((percent * PWM_RANGE + 50.0f) / 100.0f);
}

// ============================================================
// FILTRO EMA
// ============================================================

float filterUpdate(SensorFilter& f, float reading) {
  if (!f.initialized) {
    f.filtered = reading;
    f.initialized = true;
  } else {
    f.filtered = f.alpha * reading + (1.0f - f.alpha) * f.filtered;
  }
  return f.filtered;
}

// ============================================================
// CONTROLADOR PID
// ============================================================

/**
 * Reinicializa estado interno do PID (usado em transição Manual→Auto
 * para evitar saltos bruscos — bumpless transfer).
 */
void pidReset(PIDController& c, float currentError, unsigned long now) {
  c.integral    = 0.0f;
  c.prevError   = currentError;
  c.lastTime    = now;
  c.initialized = true;
}

/**
 * Calcula a saída do PID para a variável de processo `pv`.
 * Retorna a saída em percentual (0–100) e indica via `saturated`
 * se a saída foi limitada (para anti-windup condicional).
 */
float pidCompute(PIDController& c, float pv, unsigned long now, bool& saturated) {
  if (!c.initialized) {
    c.lastTime    = now;
    c.prevError   = c.setpoint - pv;
    c.initialized = true;
    saturated     = false;
    return clampf(c.Kp * c.prevError, c.outputMin, c.outputMax);
  }

  float dt = (now - c.lastTime) / 1000.0f;   // segundos
  if (dt <= 0.0f) dt = 0.001f;

  float error = c.setpoint - pv;

  // Anti-windup condicional: só integra se a saída anterior NÃO estava saturada
  // OU se o erro está empurrando o integral para fora da saturação.
  float tentativeIntegral = c.integral + error * dt;
  tentativeIntegral = clampf(tentativeIntegral, -c.integralMax, c.integralMax);
  c.integral = tentativeIntegral;

  float derivative = (error - c.prevError) / dt;

  float output = c.Kp * error
               + c.Ki * c.integral
               + c.Kd * derivative;

  float clamped = clampf(output, c.outputMin, c.outputMax);
  saturated = (output != clamped);

  // Se saturou empurrando o integral pra mesma direção, "puxa de volta"
  // o último incremento (clamping clássico de anti-windup).
  if (saturated && ((error > 0 && output > c.outputMax) ||
                    (error < 0 && output < c.outputMin))) {
    c.integral -= error * dt;
    c.integral = clampf(c.integral, -c.integralMax, c.integralMax);
  }

  c.prevError = error;
  c.lastTime  = now;
  return clamped;
}

// ============================================================
// BUZZER — alerta progressivo (rampa linear)
// ============================================================

/**
 * Retorna o nível de alerta em % (0–100) com base na distância bruta.
 *
 * Zona segura: bottomStart > raw > topStart   → 0%
 * Água alta:   raw <= topStart                → rampa 0..100% ao decrescer até topMax
 *              raw <= topMax                  → 100%
 * Água baixa:  raw >= bottomStart             → rampa 0..100% ao crescer até bottomMax
 *              raw >= bottomMax               → 100%
 */
uint8_t buzzerCompute(const BuzzerAlert& b, float rawDistance) {
  if (rawDistance < 0) return 0;   // sem leitura válida

  // Água alta (sensor próximo)
  if (rawDistance <= b.topStart) {
    if (rawDistance <= b.topMax) return 100;
    float ratio = (b.topStart - rawDistance) / (b.topStart - b.topMax);
    return (uint8_t)(clampf(ratio, 0.0f, 1.0f) * 100.0f);
  }

  // Água baixa (sensor distante)
  if (rawDistance >= b.bottomStart) {
    if (rawDistance >= b.bottomMax) return 100;
    float ratio = (rawDistance - b.bottomStart) / (b.bottomMax - b.bottomStart);
    return (uint8_t)(clampf(ratio, 0.0f, 1.0f) * 100.0f);
  }

  return 0;
}

// ============================================================
// MODE MANAGER
// ============================================================

void setMode(ModeManager& m, SystemMode target) {
  if (m.current == target) return;

  if (target == MODE_AUTO) {
    // Bumpless transfer: zera integral e alinha derivada com o erro atual
    pidReset(pid, pid.setpoint - waterHeightCm, millis());
  } else {
    // Auto → Manual: congela slider no valor que o PID estava aplicando
    m.manualPwmPercent = pumpPwmPercent;
  }
  m.current = target;
}

// ============================================================
// PWM OUTPUT
// ============================================================

void applyPwmOutputs() {
  analogWrite(PUMP_PWM_PIN,   percentToPWM(pumpPwmPercent));
  analogWrite(BUZZER_PWM_PIN, percentToPWM(buzzer.pwmPercent));
}

// ============================================================
// JSON STATE — broadcast para o frontend
// ============================================================

String getStateJson() {
  String j = "{";
  j += "\"h\":";   j += String(waterHeightCm, 2);
  j += ",\"raw\":"; j += (rawDistanceCm < 0 ? String("null") : String(rawDistanceCm, 2));
  j += ",\"sp\":"; j += String(pid.setpoint, 2);
  j += ",\"pwm\":"; j += String(pumpPwmPercent);
  j += ",\"manualPwm\":"; j += String(mode.manualPwmPercent);
  j += ",\"mode\":\""; j += (mode.current == MODE_AUTO ? "auto" : "manual"); j += "\"";
  j += ",\"kp\":"; j += String(pid.Kp, 3);
  j += ",\"ki\":"; j += String(pid.Ki, 3);
  j += ",\"kd\":"; j += String(pid.Kd, 3);
  j += ",\"buz\":"; j += String(buzzer.pwmPercent);
  j += ",\"status\":\""; j += sensorStatus; j += "\"";
  j += ",\"tank\":"; j += String(TANK_HEIGHT, 2);
  j += ",\"t\":"; j += String(millis());
  j += "}";
  return j;
}

void notifyClients(const String& msg) {
  ws.textAll(msg);
}

// ============================================================
// WEBSOCKET — handler do protocolo do frontend
// ============================================================

/**
 * Protocolo Frontend → ESP (texto curto):
 *   1s<v>      — PWM manual da bomba (0–100)
 *   sp<v>      — setpoint em cm
 *   kp<v>      — ganho proporcional
 *   ki<v>      — ganho integral
 *   kd<v>      — ganho derivativo
 *   modem      — modo Manual
 *   modea      — modo Auto
 *   getValues  — solicita estado atual
 */
void handleWebSocketMessage(void* arg, uint8_t* data, size_t len) {
  AwsFrameInfo* info = (AwsFrameInfo*)arg;
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
    return;
  }

  data[len] = 0;
  String msg = String((char*)data);

  if (msg == "getValues") {
    notifyClients(getStateJson());
    return;
  }

  if (msg.startsWith("1s")) {
    int v = msg.substring(2).toInt();
    mode.manualPwmPercent = (uint8_t)constrain(v, 0, 100);
    Serial.printf("[Manual PWM] %u%%\n", mode.manualPwmPercent);
    notifyClients(getStateJson());
    return;
  }

  if (msg.startsWith("sp")) {
    float v = msg.substring(2).toFloat();
    pid.setpoint = clampf(v, 0.0f, TANK_HEIGHT);
    Serial.printf("[Setpoint] %.2f cm\n", pid.setpoint);
    notifyClients(getStateJson());
    return;
  }

  if (msg.startsWith("kp")) {
    pid.Kp = msg.substring(2).toFloat();
    Serial.printf("[Kp] %.3f\n", pid.Kp);
    notifyClients(getStateJson());
    return;
  }

  if (msg.startsWith("ki")) {
    pid.Ki = msg.substring(2).toFloat();
    Serial.printf("[Ki] %.3f\n", pid.Ki);
    notifyClients(getStateJson());
    return;
  }

  if (msg.startsWith("kd")) {
    pid.Kd = msg.substring(2).toFloat();
    Serial.printf("[Kd] %.3f\n", pid.Kd);
    notifyClients(getStateJson());
    return;
  }

  if (msg.startsWith("mode")) {
    char c = msg.length() > 4 ? msg.charAt(4) : 'm';
    setMode(mode, c == 'a' ? MODE_AUTO : MODE_MANUAL);
    Serial.printf("[Mode] %s\n", mode.current == MODE_AUTO ? "AUTO" : "MANUAL");
    notifyClients(getStateJson());
    return;
  }
}

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket #%u from %s\n", client->id(),
                    client->remoteIP().toString().c_str());
      client->text(getStateJson());
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
// WIFI / FS / WEB SERVER
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
  Serial.println("\nWiFi connected!");
  Serial.print("IP address: ");
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

  pinMode(HC_SR04_TRIG_PIN, OUTPUT);
  pinMode(HC_SR04_ECHO_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(HC_SR04_ECHO_PIN), echoISR, CHANGE);

  pinMode(PUMP_PWM_PIN,   OUTPUT);
  pinMode(BUZZER_PWM_PIN, OUTPUT);
  analogWriteFreq(PWM_FREQ);
  analogWriteRange(PWM_RANGE);
  applyPwmOutputs();

  sensorTimer = millis();

  Serial.println("\n=== System Started ===");
  Serial.printf("Tank: %.1f cm | Sensor->bottom: %.1f cm | Step: %d ms\n",
                TANK_HEIGHT, SENSOR_TO_BOTTOM, SENSOR_INTERVAL_MS);
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
//   - Lê HC-SR04
//   - Atualiza filtro de nível
//   - Atualiza buzzer (na distância bruta)
//   - Em modo Auto, calcula PID
//   - Faz broadcast do estado completo
// ============================================================

void updateSensor() {
  unsigned long now = millis();

  switch (sensorState) {

    case SENSOR_IDLE:
      if (now - sensorTimer >= SENSOR_INTERVAL_MS) {
        echoDuration = -1;

        digitalWrite(HC_SR04_TRIG_PIN, LOW);
        delayMicroseconds(TRIG_LOW_DELAY);
        digitalWrite(HC_SR04_TRIG_PIN, HIGH);
        delayMicroseconds(TRIG_HIGH_PULSE);
        digitalWrite(HC_SR04_TRIG_PIN, LOW);

        sensorTimer = now;
        sensorState = SENSOR_TRIGGERED;
      }
      break;

    case SENSOR_TRIGGERED:
      if (echoDuration >= 0) {
        float distance = echoDuration * SOUND_SPEED / 2.0f;
        rawDistanceCm   = distance;
        hasValidReading = true;
        sensorStatus    = "ok";

        // 1) Buzzer opera na distância bruta — segurança não pode ter lag
        buzzer.pwmPercent = buzzerCompute(buzzer, rawDistanceCm);

        // 2) Calcula altura instantânea e aplica filtro EMA
        float instantHeight = clampf(SENSOR_TO_BOTTOM - rawDistanceCm,
                                     0.0f, TANK_HEIGHT);
        waterHeightCm = filterUpdate(levelFilter, instantHeight);

        // 3) Em modo Auto, calcula PID; em Manual, usa slider
        if (mode.current == MODE_AUTO) {
          bool sat = false;
          float out = pidCompute(pid, waterHeightCm, millis(), sat);
          pumpPwmPercent = (uint8_t)clampf(out, 0.0f, 100.0f);
        } else {
          pumpPwmPercent = mode.manualPwmPercent;
        }

        // 4) Broadcast do estado completo
        notifyClients(getStateJson());

        Serial.printf("[T] raw=%.1fcm h=%.2fcm sp=%.2f pwm=%u%% buz=%u%% mode=%s\n",
                      rawDistanceCm, waterHeightCm, pid.setpoint,
                      pumpPwmPercent, buzzer.pwmPercent,
                      mode.current == MODE_AUTO ? "AUTO" : "MAN");

        sensorTimer = millis();
        sensorState = SENSOR_IDLE;
        break;
      }

      // Timeout — descartar leitura sem corromper o filtro
      if (now - sensorTimer > ECHO_TIMEOUT_MS) {
        sensorStatus = "timeout";
        // Mantém último rawDistanceCm; em modo Auto o PID continua com
        // a última altura conhecida (degradação controlada).
        notifyClients(getStateJson());

        Serial.println("[HC-SR04] Timeout — no echo received");

        sensorTimer = millis();
        sensorState = SENSOR_IDLE;
      }
      break;
  }
}
