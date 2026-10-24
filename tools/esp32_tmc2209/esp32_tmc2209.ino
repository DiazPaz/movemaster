// Gripper de MoveMaster: muñeca (motor a pasos con TMC2209) y pinza (servo por PWM),
// controlados por CAN desde la Raspberry Pi como un dispositivo más del bus de los SPARK MAX.
//
// Identidad CAN: ID extendido de 29 bits con el formato FRC que usan los SPARK MAX.
//   Tipo 10 (Miscellaneous) | Fabricante 8 (Team Use) | Clase | Índice | Dispositivo 5
//   ID = 0x0A080000 | clase << 10 | índice << 6 | 5
//
// Tramas (bus a 1 Mbit/s, datos little-endian):
//   clase,índice  ID          dirección    trama                    datos
//   0,0           0x0A080005  Pi -> ESP32  WRIST_POSITION_SETPOINT  float32 objetivo en grados
//   0,1           0x0A080045  Pi -> ESP32  SERVO_SETPOINT           uint8 ángulo (0 a 180)
//   0,2           0x0A080085  Pi -> ESP32  STOP                     sin datos: frena con rampa
//   1,0           0x0A080405  Pi -> ESP32  SET_MOTION_PROFILE       float32 °/s, float32 °/s^2
//   1,1           0x0A080445  Pi -> ESP32  SET_WRIST_POSITION       float32 grados actuales
//   2,0           0x0A080805  ESP32 -> Pi  STATUS_0, cada 20 ms     float32 grados, uint8 servo,
//                                                                   uint8 banderas, 2 reservados
//   -             0x01011840  Pi -> todos  heartbeat                el que habilita a los SPARK
//
// Banderas de STATUS_0: bit 0 driver energizado, bit 1 muñeca en movimiento, bit 2 heartbeat
// presente, bit 3 posición sin referencia (desde el arranque hasta SET_WRIST_POSITION),
// bit 4 servo con señal.
//
// Seguridad: los setpoints (clase 0) solo se aceptan con heartbeat; la configuración (clase 1)
// siempre. Si pasan 100 ms sin heartbeat, la muñeca se detiene en seco y mantiene su torque, y
// el servo se queda en su último ángulo. Al volver el heartbeat nada se mueve hasta recibir un
// setpoint nuevo. Al arrancar, el driver está apagado hasta el primer heartbeat y el servo no
// recibe señal hasta su primer setpoint, así que nada se mueve solo.
//
// Conexiones:
//   GPIO 27 -> STEP del TMC2209   (VIO a 3V3, VM a 12 V, MS1 y MS2 a GND: 1/8 de micropaso)
//   GPIO 26 -> DIR
//   GPIO 25 -> EN                 (LOW = habilitado; pull-up de 10 kΩ a 3V3)
//   GPIO 13 -> señal del servo    (220-470 Ω en serie; servo alimentado a 7.4 V)
//   GPIO 21 -> TXD del MCP2551    (VDD a 5 V, Rs a GND)
//   GPIO 22 <- RXD del MCP2551    con divisor: 1 kΩ en serie desde RXD y 2 kΩ a GND

#include <Arduino.h>
#include "driver/twai.h"

const int PIN_STEP = 27;
const int PIN_DIR = 26;
const int PIN_EN = 25;
const int PIN_SERVO = 13;
const int PIN_CAN_TX = 21;
const int PIN_CAN_RX = 22;

// Conversión de la muñeca: pasos por grado = PASOS_MOTOR * MICROPASOS * REDUCCION / 360
const int PASOS_MOTOR = 200;  // pasos por vuelta del motor: 200 si es de 1.8°, 400 si es de 0.9°
// Debe coincidir con MS1/MS2: 8, 16, 32 o 64. Sin UART el TMC2209 no trabaja en paso completo;
// 1 solo si lo configuras por UART.
const int MICROPASOS = 64;
// Vueltas del motor por cada vuelta de la muñeca: 5.0 para una reducción 5:1. Con varias
// etapas, multiplícalas (por ejemplo 3.0 * 4.0).
const float REDUCCION = 1.0;
const bool INVERTIR_MUNECA = false;  // true si la reducción invierte el sentido de giro
const float PASOS_POR_GRADO = PASOS_MOTOR * MICROPASOS * REDUCCION / 360;
const float LIMITE_GRADOS = 360;     // recorrido permitido: -360° a 360°

const float VEL_INICIAL = 15000;   // pasos/s con que arranca y termina la rampa
const float VEL_MAXIMA = 15000;  // límite del generador de pasos

const int SERVO_FRECUENCIA = 50;   // Hz, periodo de 20 ms
const int SERVO_RESOLUCION = 14;   // bits del PWM
const int SERVO_CANAL = 0;         // canal LEDC (solo core 2.x)
const int SERVO_US_MIN = 500;      // pulso en 0°; ajústalo si tu servo no llega al tope
const int SERVO_US_MAX = 2500;     // pulso en 180°

const uint32_t CAN_TIPO = 10;        // Miscellaneous
const uint32_t CAN_FABRICANTE = 8;   // Team Use: reservado por FRC para dispositivos propios
const uint32_t CAN_DISPOSITIVO = 5;
const uint32_t CAN_HEARTBEAT = 0x01011840;
const unsigned long HEARTBEAT_TIMEOUT_MS = 100;
const unsigned long PERIODO_STATUS_MS = 20;

// API = clase << 4 | índice: los 10 bits del ID entre el fabricante y el dispositivo
const uint32_t API_WRIST_POSITION_SETPOINT = (0 << 4) | 0;
const uint32_t API_SERVO_SETPOINT = (0 << 4) | 1;
const uint32_t API_STOP = (0 << 4) | 2;
const uint32_t API_SET_MOTION_PROFILE = (1 << 4) | 0;
const uint32_t API_SET_WRIST_POSITION = (1 << 4) | 1;
const uint32_t API_STATUS_0 = (2 << 4) | 0;

constexpr uint32_t idCan(uint32_t api) {
  return (CAN_TIPO << 24) | (CAN_FABRICANTE << 16) | (api << 6) | CAN_DISPOSITIVO;
}

float velocidad = min(90 * PASOS_POR_GRADO, VEL_MAXIMA);  // pasos/s (90 °/s)
float aceleracion = 180 * PASOS_POR_GRADO;                // pasos/s^2 (180 °/s^2)
long posicion = 0;                          // pasos
long objetivo = 0;                          // pasos
float v = 0;                                // velocidad actual en pasos/s; 0 = detenida
int sentido = 1;
unsigned long periodo = 0;                  // µs entre pasos
unsigned long ultimoPaso = 0;
bool energizado = false;
bool sinReferencia = true;

int anguloServo = 90;
bool servoConSenal = false;

bool heartbeat = false;
unsigned long ultimoHeartbeat = 0;
unsigned long ultimoStatus = 0;
unsigned long ultimoReporte = 0;

void energizar(bool on) {
  energizado = on;
  digitalWrite(PIN_EN, on ? LOW : HIGH);
}

float velInicial() {
  return min(VEL_INICIAL, velocidad);
}

// Pasos que necesita la muñeca para frenar desde la velocidad actual.
float distanciaFreno() {
  float v0 = velInicial();
  return v > v0 ? (v * v - v0 * v0) / (2 * aceleracion) : 0;
}

void moverMuneca(float grados) {
  grados = constrain(grados, -LIMITE_GRADOS, LIMITE_GRADOS);
  objetivo = lroundf(grados * PASOS_POR_GRADO);
}

void frenarMuneca() {
  objetivo = v > 0 ? posicion + sentido * (long)ceilf(distanciaFreno()) : posicion;
}

void detenerEnSeco() {
  v = 0;
  objetivo = posicion;
}

// Da como máximo un paso por llamada, sin bloquear: loop() debe llamarla seguido. Acelera
// mientras le alcance la distancia para frenar antes del objetivo; si el objetivo queda atrás,
// frena, se detiene y regresa.
void actualizarMuneca() {
  if (!energizado) return;
  if (v == 0) {
    if (posicion == objetivo) return;
    sentido = objetivo > posicion ? 1 : -1;
    digitalWrite(PIN_DIR, (sentido > 0) != INVERTIR_MUNECA ? HIGH : LOW);
    delayMicroseconds(5);
    v = velInicial();
    periodo = 1000000.0f / v;
    ultimoPaso = micros() - periodo;
  }

  unsigned long ahora = micros();
  if (ahora - ultimoPaso < periodo) return;
  ultimoPaso = ahora;

  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(3);
  digitalWrite(PIN_STEP, LOW);
  posicion += sentido;

  long restante = (objetivo - posicion) * sentido;  // > 0 si el objetivo sigue adelante
  float v0 = velInicial();
  bool frenar = restante <= distanciaFreno() + 1;
  if (frenar || v > velocidad) {
    float piso = frenar ? v0 : velocidad;
    v = sqrtf(max(v * v - 2 * aceleracion, piso * piso));
  } else {
    v = min(sqrtf(v * v + 2 * aceleracion), velocidad);
  }
  if (restante <= 0 && v <= v0 * 1.001f) v = 0;  // llegó, o se pasó y ya puede regresar
  if (v > 0) periodo = 1000000.0f / v;
}

// El core 3.x de la ESP32 cambió la API de LEDC: los canales se asignan solos por pin.
// Con duty 0 no hay pulsos, así que el servo no se mueve hasta su primer setpoint.
void iniciarServo() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_SERVO, SERVO_FRECUENCIA, SERVO_RESOLUCION);
#else
  ledcSetup(SERVO_CANAL, SERVO_FRECUENCIA, SERVO_RESOLUCION);
  ledcAttachPin(PIN_SERVO, SERVO_CANAL);
#endif
}

void moverServo(int angulo) {
  anguloServo = constrain(angulo, 0, 180);
  servoConSenal = true;
  uint32_t us = map(anguloServo, 0, 180, SERVO_US_MIN, SERVO_US_MAX);
  uint32_t duty = us * ((1UL << SERVO_RESOLUCION) - 1) / (1000000UL / SERVO_FRECUENCIA);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(SERVO_CANAL, duty);
#endif
}

bool iniciarCan() {
  twai_general_config_t general =
      TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
  general.rx_queue_len = 32;
  twai_timing_config_t tiempos = TWAI_TIMING_CONFIG_1MBITS();
  // Filtro doble: con IDs extendidos cada filtro compara los bits 28-13 del ID. El primero deja
  // pasar a este tipo y fabricante (clases 0 a 7) y el segundo al heartbeat. El resto del bus,
  // como los STATUS de los SPARK MAX, se descarta por hardware.
  twai_filter_config_t filtro = {
      .acceptance_code = ((idCan(0) >> 13) << 16) | (CAN_HEARTBEAT >> 13),
      .acceptance_mask = 0,
      .single_filter = false,
  };
  return twai_driver_install(&general, &tiempos, &filtro) == ESP_OK && twai_start() == ESP_OK;
}

bool leerFloat(const uint8_t *datos, float &valor) {
  memcpy(&valor, datos, sizeof valor);
  return isfinite(valor);
}

void procesar(uint32_t api, const uint8_t *datos, uint8_t n) {
  float a, b;
  switch (api) {
    case API_WRIST_POSITION_SETPOINT:
      if (heartbeat && n >= 4 && leerFloat(datos, a)) moverMuneca(a);
      break;
    case API_SERVO_SETPOINT:
      if (heartbeat && n >= 1) moverServo(datos[0]);
      break;
    case API_STOP:
      frenarMuneca();
      break;
    case API_SET_MOTION_PROFILE:
      if (n >= 8 && leerFloat(datos, a) && leerFloat(datos + 4, b) && a > 0 && b > 0) {
        velocidad = constrain(a * PASOS_POR_GRADO, 1.0f, VEL_MAXIMA);
        aceleracion = b * PASOS_POR_GRADO;
      }
      break;
    case API_SET_WRIST_POSITION:
      // Solo con la muñeca detenida, para no cambiar la referencia a medio movimiento.
      if (n >= 4 && leerFloat(datos, a) && fabsf(a) <= LIMITE_GRADOS && v == 0) {
        posicion = objetivo = lroundf(a * PASOS_POR_GRADO);
        sinReferencia = false;
      }
      break;
  }
}

void atenderCan() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    if (!msg.extd || msg.rtr) continue;
    if (msg.identifier == CAN_HEARTBEAT) {
      ultimoHeartbeat = millis();
      if (!heartbeat) energizar(true);
      heartbeat = true;
    } else if ((msg.identifier & ~(0x3FFUL << 6)) == idCan(0)) {
      procesar((msg.identifier >> 6) & 0x3FF, msg.data, msg.data_length_code);
    }
  }
}

void revisarHeartbeat() {
  if (heartbeat && millis() - ultimoHeartbeat > HEARTBEAT_TIMEOUT_MS) {
    heartbeat = false;
    detenerEnSeco();  // el driver sigue energizado: la muñeca mantiene su posición
  }
}

uint8_t banderas() {
  return energizado | (v > 0 || posicion != objetivo) << 1 | heartbeat << 2 |
         sinReferencia << 3 | servoConSenal << 4;
}

void enviarStatus() {
  twai_message_t msg = {};
  msg.extd = 1;
  msg.identifier = idCan(API_STATUS_0);
  msg.data_length_code = 8;
  float grados = posicion / PASOS_POR_GRADO;
  memcpy(msg.data, &grados, sizeof grados);
  msg.data[4] = anguloServo;
  msg.data[5] = banderas();
  twai_transmit(&msg, 0);  // si la cola está llena, la siguiente STATUS_0 la reemplaza
}

// Tras demasiados errores (cable suelto, sin terminación) el controlador se apaga solo
// (bus-off); aquí se recupera y se vuelve a arrancar.
void vigilarCan() {
  twai_status_info_t info;
  if (twai_get_status_info(&info) != ESP_OK) return;
  if (info.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
  else if (info.state == TWAI_STATE_STOPPED) twai_start();
}

void reportar() {
  twai_status_info_t info = {};
  twai_get_status_info(&info);
  Serial.printf("muñeca %.1f° servo %d° hb %d ref %d | CAN %s TEC %lu REC %lu\n",
                posicion / PASOS_POR_GRADO, anguloServo, heartbeat, !sinReferencia,
                info.state == TWAI_STATE_RUNNING ? "ok" : "detenido",
                (unsigned long)info.tx_error_counter, (unsigned long)info.rx_error_counter);
}

void setup() {
  pinMode(PIN_EN, OUTPUT);
  energizar(false);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  iniciarServo();

  Serial.begin(115200);
  if (iniciarCan()) {
    Serial.printf("\nGripper: CAN a 1 Mbit/s, dispositivo %lu, STATUS_0 en 0x%08lX\n",
                  (unsigned long)CAN_DISPOSITIVO, (unsigned long)idCan(API_STATUS_0));
  } else {
    Serial.println("\nGripper: no se pudo iniciar el CAN");
  }
  Serial.printf("Muñeca: %.3f pasos por grado, hasta %.1f °/s\n", PASOS_POR_GRADO,
                VEL_MAXIMA / PASOS_POR_GRADO);
}

void loop() {
  atenderCan();
  revisarHeartbeat();
  actualizarMuneca();

  unsigned long ahora = millis();
  if (ahora - ultimoStatus >= PERIODO_STATUS_MS) {
    ultimoStatus = ahora;
    enviarStatus();
    vigilarCan();
  }
  if (ahora - ultimoReporte >= 1000) {
    ultimoReporte = ahora;
    reportar();
  }
}
