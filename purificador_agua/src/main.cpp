/* 
  ESP32: Sistema inteligente de captación y purificación - Código de referencia
  - Lecturas: turbidez (analógico), pH (analógico), nivel (ultrasonico JSN-SR04T), caudal (YF-S201), tipping bucket
  - Actuadores: bomba (PWM/relay), UV (relay), válvulas (relays), dosificador peristáltico (PWM/relay)
  - Comunicaciones: WiFi -> ThingSpeak (HTTP) o MQTT; opcional LoRa SX1278
  - Simulación: si no hay hardware, activar SIMULATE_SENSORS = true
  - Autor: Borrador para monografía de Joshs. Ajustar pines y calibraciones.
*/

#include <Arduino.h>

// --- CONFIG ----
#define SIMULATE_SENSORS false   // true = genera datos de prueba por Serial
#define USE_LORA false           // true = habilita LoRa TX (requiere librería LoRa)
#define USE_MQTT false           // true = habilita MQTT en vez de ThingSpeak
#define USE_THINGSPEAK true      // enviar datos a ThingSpeak con HTTP

// WiFi / ThingSpeak / MQTT
const char* WIFI_SSID = "TU_SSID";
const char* WIFI_PASS = "TU_PASS";
const char* THINGSPEAK_API_KEY = "TU_THINGSPEAK_APIKEY"; // si usas Thingspeak
const char* MQTT_BROKER = "broker.hivemq.com"; // ejemplo público
const int MQTT_PORT = 1883;
const char* MQTT_TOPIC = "monografia/agua";

// LoRa (si USE_LORA true) - ajustar pines según placa y antena
#if USE_LORA
  #include <SPI.h>
  #include <LoRa.h>
  const long LORA_FREQ = 433E6; // 433MHz o 915E6 según módulo
  const int LORA_SS = 18;
  const int LORA_RST = 14;
  const int LORA_DIO0 = 26;
#endif

#include <WiFi.h>
#include <HTTPClient.h>
#if USE_MQTT
  #include <PubSubClient.h>
#endif

// --- PINS (ajusta según tu conexión) ---
const int PIN_TURBIDITY = 34; // ADC1_CH6
const int PIN_PH = 35;        // ADC1_CH7
const int PIN_TRIG = 27;      // Ultrasonic trigger
const int PIN_ECHO = 33;      // Ultrasonic echo
const int PIN_FLOW = 25;      // Flow sensor pulse input
const int PIN_TIPPING = 32;   // Tipping bucket pulse input

const int PIN_PUMP = 16;      // Relay/MOSFET control (ON = HIGH or LOW según módulo)
const int PIN_UV = 17;        // Relay for UV lamp
const int PIN_VALVE_RAIN = 4; // Valve for rain inlet
const int PIN_DOSER = 5;      // Peristaltic doser control (PWM or relay)

// PWM config for pump/doser if needed
const int PWM_FREQ = 5000;
const int PWM_CHANNEL_PUMP = 0;
const int PWM_CHANNEL_DOSER = 1;
const int PWM_RESOLUTION = 8; // 8-bit (0-255)

// --- THRESHOLDS y parámetros ---
float TURBIDITY_THRESHOLD = 5.0; // NTU
float PH_MIN = 6.5;
float PH_MAX = 8.5;
float TANK_LEVEL_FULL_CM = 50.0; // ajustar
float TANK_LEVEL_EMPTY_CM = 5.0;

// Calibration factors (debes calibrar con tus sensores)
float turbidity_adc_to_NTU = 0.05; // ejemplo, ajustar
float ph_adc_to_ph = 0.01;         // ejemplo, mapping ADC->pH

// --- Variables globales ---
volatile unsigned long flow_pulse_count = 0;
volatile unsigned long tipping_count = 0;

unsigned long lastFlowCalc = 0;
float flowRate = 0.0; // L/min
float totalLiters = 0.0;

unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 30UL * 1000UL; // enviar cada 30s (ThemeSpeak rate limit cuidado: 15s mínimo)

// --- Function prototypes ---
void setupSensorsPins();
void setupActuators();
float readTurbidity();
float readPH();
float readUltrasonic_cm();
void IRAM_ATTR flowPulseISR();
void IRAM_ATTR tippingISR();
void calculateFlow();
void activateFilter();
void deactivateFilter();
void activateUV();
void deactivateUV();
void activateDoser(float seconds);
void sendDataToThingSpeak(float turb, float ph, float level, float flow);
void sendDataMQTT(float turb, float ph, float level, float flow);
void comportLogic(float turb, float ph, float level);

// --- MQTT client ---
WiFiClient wifiClient;
#if USE_MQTT
PubSubClient mqttClient(wifiClient);
#endif

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial.println("\n--- Iniciando sistema inteligente de captacion y purificacion ---");

  // Pines y periféricos
  setupSensorsPins();
  setupActuators();

  // PWM for pump & doser
  ledcSetup(PWM_CHANNEL_PUMP, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_PUMP, PWM_CHANNEL_PUMP);
  ledcSetup(PWM_CHANNEL_DOSER, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_DOSER, PWM_CHANNEL_DOSER);

  // WiFi
  if(!SIMULATE_SENSORS) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Conectando WiFi");
    unsigned long tstart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - tstart < 15000) {
      Serial.print(".");
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi conectado. IP: " + WiFi.localIP().toString());
    } else {
      Serial.println("\nNo se pudo conectar a WiFi. Modo offline (datos via Serial).");
    }
  } else {
    Serial.println("SIMULATE_SENSORS=true -> No se intentará conectar WiFi");
  }

  #if USE_MQTT
    mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  #endif

  #if USE_LORA
    if (!LoRa.begin(LORA_FREQ)) {
      Serial.println("Error inicializando LoRa. Revisa conexiones y frecuencia.");
    } else {
      Serial.println("LoRa iniciado.");
    }
  #endif

  lastSend = millis();
  lastFlowCalc = millis();
}

void loop() {
  // Calculo flujo cada 5s
  if (millis() - lastFlowCalc >= 5000) {
    calculateFlow();
    lastFlowCalc = millis();
  }

  // Lecturas (si simular = generar pseudoaleatorios)
  float turb = SIMULATE_SENSORS ? (random(50, 300) / 10.0) : readTurbidity();
  float ph = SIMULATE_SENSORS ? (random(60, 85) / 10.0) : readPH();
  float level = SIMULATE_SENSORS ? (random(5, 50)) : readUltrasonic_cm();

  // Lógica de control
  comportLogic(turb, ph, level);

  // Envío de datos
  if (millis() - lastSend >= SEND_INTERVAL) {
    if (WiFi.status() == WL_CONNECTED && USE_THINGSPEAK) {
      sendDataToThingSpeak(turb, ph, level, flowRate);
    } else if (USE_MQTT) {
      sendDataMQTT(turb, ph, level, flowRate);
    } else if (USE_LORA) {
      // Envío LoRa (punto a punto)
      #if USE_LORA
      String payload = String("{\"turb\":") + turb + ",\"ph\":" + ph + ",\"level\":" + level + ",\"flow\":" + flowRate + "}";
      LoRa.beginPacket();
      LoRa.print(payload);
      LoRa.endPacket();
      Serial.println("LoRa TX: " + payload);
      #endif
    } else {
      // solo Serial
      Serial.printf("DATA -> turb=%.2f NTU, pH=%.2f, level=%.1f cm, flow=%.2f L/min\n", turb, ph, level, flowRate);
    }
    lastSend = millis();
  }

  // MQTT loop
  #if USE_MQTT
  if (!mqttClient.connected()) {
    // reconectar
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("Reconectando MQTT...");
      if (mqttClient.connect("esp32_client_monografia")) {
        Serial.println("MQTT conectado");
      } else {
        Serial.println("MQTT fallo");
      }
    }
  } else {
    mqttClient.loop();
  }
  #endif

  delay(200); // ciclo suave
}

// ---------------- implementaciones ----------------

void setupSensorsPins() {
  pinMode(PIN_TURBIDITY, INPUT);
  pinMode(PIN_PH, INPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_FLOW, INPUT_PULLUP);
  pinMode(PIN_TIPPING, INPUT_PULLUP);

  // interrupciones para flow y tipping
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), flowPulseISR, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_TIPPING), tippingISR, RISING);
}

void setupActuators() {
  pinMode(PIN_PUMP, OUTPUT);
  pinMode(PIN_UV, OUTPUT);
  pinMode(PIN_VALVE_RAIN, OUTPUT);
  pinMode(PIN_DOSER, OUTPUT);

  // apagar por defecto
  digitalWrite(PIN_PUMP, LOW);
  digitalWrite(PIN_UV, LOW);
  digitalWrite(PIN_VALVE_RAIN, LOW);
  digitalWrite(PIN_DOSER, LOW);
}

// --- Lecturas sensores ---
float readTurbidity() {
  // lectura ADC y conversión a NTU - calibrar con curva del sensor
  int raw = analogRead(PIN_TURBIDITY); // 0-4095 (ESP32 ADC 12-bit)
  float voltage = (raw / 4095.0) * 3.3;
  // ejemplo transform simple: NTU ~ K * voltage
  float ntu = voltage / turbidity_adc_to_NTU; // ajustar turbidity_adc_to_NTU
  // filtro simple (promedio móvil, etc) - acá devolvemos directo
  Serial.printf("Raw turb=%d volt=%.3f NTU=%.2f\n", raw, voltage, ntu);
  return ntu;
}

float readPH() {
  int raw = analogRead(PIN_PH);
  float voltage = (raw / 4095.0) * 3.3;
  // conversión a pH depende de tu circuito de acondicionamiento
  float ph = voltage / ph_adc_to_ph; // ajustar ph_adc_to_ph
  Serial.printf("Raw pH=%d volt=%.3f pH=%.2f\n", raw, voltage, ph);
  return ph;
}

float readUltrasonic_cm() {
  // JSN-SR04T: trig 10us, medir echo
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(12);
  digitalWrite(PIN_TRIG, LOW);
  long duration = pulseIn(PIN_ECHO, HIGH, 30000); // timeout 30ms -> ~5m
  if (duration == 0) return -1; // no echo
  float distance_cm = (duration / 2.0) * 0.0343; // velocidad sonido ~343 m/s
  Serial.printf("Ultrasonic dur=%ld cm=%.1f\n", duration, distance_cm);
  return distance_cm;
}

// flujo (YF-S201): cada pulso corresponde a X L
IRAM_ATTR void flowPulseISR() {
  flow_pulse_count++;
}

IRAM_ATTR void tippingISR() {
  tipping_count++;
}

void calculateFlow() {
  // formula: flowRate(L/min) = (pulses / factor) * (60 / interval_seconds)
  // factor depende del sensor (YF-S201 ~ 450 pulses per liter?? verificar)
  static const float PULSES_PER_LITER = 450.0; // ajustar según sensor
  unsigned long pulses;
  noInterrupts();
  pulses = flow_pulse_count;
  flow_pulse_count = 0;
  interrupts();
  float interval_s = 5.0; // periodo en s
  float liters = pulses / PULSES_PER_LITER;
  flowRate = (liters / (interval_s / 60.0)); // L/min
  totalLiters += liters;
  Serial.printf("Flow pulses=%lu liters=%.3f flow(L/min)=%.3f totalL=%.3f\n", pulses, liters, flowRate, totalLiters);
}

// --- Acciones (actuadores) ---
void activateFilter() {
  // En diseño: activar bomba y abrir válvula lluvia si aplica
  Serial.println("Activando filtro: bomba ON, valve rain ON");
  // Si usas relay (HIGH=ON). Ajustar según módulo.
  digitalWrite(PIN_VALVE_RAIN, HIGH);
  // bomba con PWM (duty 200/255 aprox)
  ledcWrite(PWM_CHANNEL_PUMP, 200);
}

void deactivateFilter() {
  Serial.println("Desactivando filtro: bomba OFF");
  digitalWrite(PIN_VALVE_RAIN, LOW);
  ledcWrite(PWM_CHANNEL_PUMP, 0);
}

void activateUV() {
  Serial.println("Activando UV");
  digitalWrite(PIN_UV, HIGH);
}

void deactivateUV() {
  Serial.println("Desactivando UV");
  digitalWrite(PIN_UV, LOW);
}

void activateDoser(float seconds) {
  Serial.printf("Activando doser por %.1f s\n", seconds);
  digitalWrite(PIN_DOSER, HIGH);
  delay((unsigned long)(seconds * 1000.0));
  digitalWrite(PIN_DOSER, LOW);
}

// --- Lógica de control principal ---
void comportLogic(float turb, float ph, float level) {
  // Verificamos niveles inválidos
  if (turb < 0) turb = 999.0;
  if (ph < 0) ph = 7.0;

  // Control de filtración según turbidez
  if (turb > TURBIDITY_THRESHOLD) {
    activateFilter();
  } else {
    deactivateFilter();
  }

  // Control pH: si fuera de rango, activar dosificador por N segundos (ejemplo)
  if (ph < PH_MIN || ph > PH_MAX) {
    // en vida real: calcular cantidad de reactivo según volumen
    activateDoser(3.0); // dosar 3s como ejemplo
  }

  // UV solo si turbidez aceptable y pH en rango
  if ((turb <= TURBIDITY_THRESHOLD) && (ph >= PH_MIN && ph <= PH_MAX)) {
    activateUV();
  } else {
    deactivateUV();
  }

  // Protección por nivel: si tanque lleno, parar bomba
  if (level > TANK_LEVEL_FULL_CM) {
    Serial.println("Nivel tanque lleno -> parada de bombas");
    deactivateFilter();
  }

  // Si tanque vacío, no activar dosificador o UV
  if (level < TANK_LEVEL_EMPTY_CM) {
    Serial.println("Nivel tanque muy bajo -> suspender UV y dosificador");
    deactivateUV();
    // no dosificar
  }
}

// --- Comunicacion ThingSpeak (HTTP POST) ---
void sendDataToThingSpeak(float turb, float ph, float level, float flow) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi no conectado, no se envia a ThingSpeak");
    return;
  }
  HTTPClient http;
  String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_API_KEY);
  url += "&field1=" + String(turb, 2);
  url += "&field2=" + String(ph, 2);
  url += "&field3=" + String(level, 2);
  url += "&field4=" + String(flow, 2);
  http.begin(url);
  int httpCode = http.GET();
  if (httpCode > 0) {
    String payload = http.getString();
    Serial.printf("ThingSpeak OK code=%d payload=%s\n", httpCode, payload.c_str());
  } else {
    Serial.printf("Error ThingSpeak: %d\n", httpCode);
  }
  http.end();
}

void sendDataMQTT(float turb, float ph, float level, float flow) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi no conectado, MQTT skip");
    return;
  }
  if (!mqttClient.connected()) {
    if (mqttClient.connect("esp32_monografia")) {
      Serial.println("MQTT conectado");
    } else {
      Serial.println("MQTT fallo al conectar");
      return;
    }
  }
  String payload = String("{\"turb\":") + turb + ",\"ph\":" + ph + ",\"level\":" + level + ",\"flow\":" + flow + "}";
  mqttClient.publish(MQTT_TOPIC, payload.c_str());
  Serial.println("MQTT publish: " + payload);
}
