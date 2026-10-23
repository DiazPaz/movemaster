// Control sencillo de un motor a pasos con ESP32 + driver TMC2209 (modo STEP/DIR)
// y de un servo por PWM.
//
// No usa librerías: el TMC2209 trabaja en modo standalone, los micropasos se eligen
// con los pines MS1/MS2 y la corriente con el potenciómetro (Vref) del módulo. El
// servo usa el PWM por hardware de la ESP32 (LEDC) a 50 Hz.
//
// Conexiones del TMC2209:
//   ESP32 GPIO 27 -> STEP
//   ESP32 GPIO 26 -> DIR
//   ESP32 GPIO 25 -> EN       (LOW = driver habilitado)
//   ESP32 3V3     -> VIO/VDD  (lógica del driver)
//   ESP32 GND     -> GND      (común con la fuente del motor)
//   MS1, MS2      -> GND      (1/8 de micropaso, ver tabla abajo)
//   VM            -> fuente del motor (5-28 V) con un capacitor de 100 uF junto al driver
//   A1 A2 / B1 B2 -> bobinas del motor
// No conectes ni desconectes el motor con VM encendida: el driver se daña.
//
// Micropasos según MS1/MS2:  GND/GND = 8,  VIO/GND = 32,  GND/VIO = 64,  VIO/VIO = 16
//
// Conexiones del servo:
//   ESP32 GPIO 13 -> señal (cable naranja o amarillo)
//   5 V externos  -> rojo   (no lo alimentes del 3V3 de la ESP32)
//   GND           -> café o negro, común con la ESP32
//
// Comandos por el monitor serie (115200 baud):
//   1600     avanza 1600 pasos (una vuelta a 1/8 de micropaso); -1600 retrocede
//   v2000    velocidad máxima en pasos/s
//   a4000    aceleración en pasos/s^2
//   on/off   habilita / deshabilita el driver (deshabilitado, el eje gira libre)
//   s90      lleva el servo a 90° (0 a 180)
// Enviar cualquier cosa durante un movimiento del motor a pasos lo detiene.

#include <Arduino.h>

const int PIN_STEP = 27;
const int PIN_DIR = 26;
const int PIN_EN = 25;
const int PIN_SERVO = 13;

const int MICROPASOS = 8;                         // debe coincidir con MS1/MS2
const long PASOS_POR_VUELTA = 200L * MICROPASOS;  // motor de 1.8° por paso

const float VEL_INICIAL = 200;   // pasos/s con que arranca y termina la rampa
const float VEL_MAXIMA = 50000;  // límite del bucle de pulsos

const int SERVO_FRECUENCIA = 50;   // Hz, periodo de 20 ms
const int SERVO_RESOLUCION = 14;   // bits del PWM
const int SERVO_CANAL = 0;         // canal LEDC (solo core 2.x)
const int SERVO_US_MIN = 500;      // pulso en 0°; ajústalo si tu servo no llega al tope
const int SERVO_US_MAX = 2500;     // pulso en 180°

float velocidad = PASOS_POR_VUELTA;          // pasos/s (1 vuelta/s = 60 RPM)
float aceleracion = 2 * PASOS_POR_VUELTA;    // pasos/s^2
long posicion = 0;                           // pasos desde el arranque
bool habilitado = false;
int anguloServo = 90;

void habilitar(bool on) {
  habilitado = on;
  digitalWrite(PIN_EN, on ? LOW : HIGH);
}

// El core 3.x de la ESP32 cambió la API de LEDC: los canales se asignan solos por pin.
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
  uint32_t us = map(anguloServo, 0, 180, SERVO_US_MIN, SERVO_US_MAX);
  uint32_t duty = us * ((1UL << SERVO_RESOLUCION) - 1) / (1000000UL / SERVO_FRECUENCIA);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(SERVO_CANAL, duty);
#endif
}

// Mueve 'pasos' (negativo = sentido contrario) con rampa de aceleración y frenado.
// Bloquea hasta terminar y regresa los pasos que alcanzó a dar.
long mover(long pasos) {
  int sentido = pasos >= 0 ? 1 : -1;
  digitalWrite(PIN_DIR, sentido > 0 ? HIGH : LOW);
  delayMicroseconds(5);

  long total = labs(pasos);
  long i = 0;
  for (; i < total; ++i) {
    if (Serial.available()) break;

    // v = sqrt(v0^2 + 2·a·d), con d los pasos al extremo más cercano del recorrido
    long d = min(i, total - 1 - i);
    float v = sqrtf(VEL_INICIAL * VEL_INICIAL + 2 * aceleracion * d);
    if (v > velocidad) v = velocidad;

    unsigned long periodo = 1000000.0f / v;
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(periodo - 5);
  }
  posicion += sentido * i;
  return i;
}

void imprimirAyuda() {
  Serial.println("Comandos: <pasos> (ej. 1600, -800), v<pasos/s>, a<pasos/s^2>, on, off, "
                 "s<grados>");
  Serial.printf("Velocidad %.0f pasos/s, aceleración %.0f pasos/s^2, %ld pasos por vuelta\n",
                velocidad, aceleracion, PASOS_POR_VUELTA);
  Serial.printf("Servo en %d°\n", anguloServo);
}

void setup() {
  pinMode(PIN_EN, OUTPUT);
  habilitar(false);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_STEP, LOW);

  iniciarServo();
  moverServo(anguloServo);

  Serial.begin(115200);
  delay(500);
  habilitar(true);

  Serial.println("\nESP32 + TMC2209. Prueba: una vuelta adelante y una atrás.");
  mover(PASOS_POR_VUELTA);
  delay(500);
  mover(-PASOS_POR_VUELTA);
  imprimirAyuda();
}

void loop() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "on") {
    habilitar(true);
    Serial.println("Driver habilitado");
  } else if (cmd == "off") {
    habilitar(false);
    Serial.println("Driver deshabilitado");
  } else if (cmd[0] == 'v') {
    float v = cmd.substring(1).toFloat();
    if (v > 0) velocidad = min(v, VEL_MAXIMA);
    Serial.printf("Velocidad: %.0f pasos/s\n", velocidad);
  } else if (cmd[0] == 'a') {
    float a = cmd.substring(1).toFloat();
    if (a > 0) aceleracion = a;
    Serial.printf("Aceleración: %.0f pasos/s^2\n", aceleracion);
  } else if (cmd[0] == 's') {
    moverServo(cmd.substring(1).toInt());
    Serial.printf("Servo: %d°\n", anguloServo);
  } else {
    long pasos = cmd.toInt();
    if (pasos == 0) {
      imprimirAyuda();
      return;
    }
    if (!habilitado) Serial.println("Aviso: el driver está deshabilitado (envía 'on')");

    long dados = mover(pasos);
    if (dados < labs(pasos)) {
      while (Serial.available()) Serial.read();
      Serial.printf("Detenido tras %ld pasos. ", dados);
    }
    Serial.printf("Posición: %ld pasos\n", posicion);
  }
}
