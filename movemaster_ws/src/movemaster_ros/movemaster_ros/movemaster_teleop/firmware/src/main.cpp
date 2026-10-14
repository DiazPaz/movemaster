// Teach pendant de MoveMaster (ESP32).
//
// Lee los botones de articulación, el switch MANUAL/AUTO, el joystick y su botón, y
// envía a 50 Hz una trama de 8 bytes a la Raspberry Pi. El ESP32 solo informa lo que
// hace el usuario: velocidades, límites y posiciones los decide la Pi (jog_node).
//
// Trama (movemaster_teleop/pendant_frame.py):
//   byte 0     contador de secuencia
//   byte 1     banderas: bit 0 MANUAL, bit 1 gripper
//   byte 2     selección: nibble alto articulación X, nibble bajo articulación Y (0 = ninguna)
//   bytes 3-4  joystick X, int16 little-endian, -1000..1000
//   bytes 5-6  joystick Y, int16 little-endian, -1000..1000
//   byte 7     CRC-8 (polinomio 0x07, inicio 0) de los bytes 0-6
// Por USB/UART cada trama va precedida del byte 0xA5. En CAN, los 8 bytes son el payload.

#include <Arduino.h>

// Joystick (ADC1)
const int pinX = 34;
const int pinY = 35;
const int pinJoyBot = 18;

// Switch MANUAL/AUTO (LOW = MANUAL)
const int pinFreeDrive = 19;

// Un botón por articulación, J1..J5
const int pinJoint[5] = {21, 22, 23, 25, 26};
// Eje del joystick que mueve cada articulación: X la cintura (J1) y la muñeca (J5),
// Y las demás. Debe coincidir con x_joints / y_joints de config/jog.yaml.
const char ejeJoint[5] = {'X', 'Y', 'Y', 'Y', 'X'};

// Invertir si el joystick está montado al revés
const bool invertirX = false;
const bool invertirY = false;

const unsigned long periodoMs = 20;  // 50 Hz
const uint8_t SYNC = 0xA5;
const int ESCALA = 1000;

// 1: imprime texto legible en lugar de tramas, para probar el pendant sin la Pi
#define PENDANT_DEBUG 0

int centroX = 2048;
int centroY = 2048;
uint8_t jointX = 0;  // 0 = ninguna: el joystick no mueve nada hasta pulsar un botón
uint8_t jointY = 0;
bool gripperActivo = false;
bool estadoAnteriorJoystickBtn = HIGH;
unsigned long ultimoDebounce = 0;
const unsigned long debounceDelay = 50;
uint8_t secuencia = 0;
unsigned long ultimoEnvio = 0;

uint8_t crc8(const uint8_t *datos, size_t n) {
  uint8_t crc = 0;
  for (size_t i = 0; i < n; ++i) {
    crc ^= datos[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : crc << 1;
    }
  }
  return crc;
}

// Promedio con el joystick en reposo: el centro real del ADC no es 2048.
int calibrarCentro(int pin) {
  long suma = 0;
  for (int i = 0; i < 64; ++i) {
    suma += analogRead(pin);
    delay(2);
  }
  return suma / 64;
}

// Lectura del ADC a -1000..1000, con cada mitad escalada desde el centro calibrado.
int16_t normalizar(int raw, int centro, bool invertir) {
  long valor = raw >= centro
      ? (long)(raw - centro) * ESCALA / max(1, 4095 - centro)
      : (long)(raw - centro) * ESCALA / max(1, centro);
  valor = constrain(valor, -ESCALA, ESCALA);
  return invertir ? -valor : valor;
}

// Único punto que depende del transporte: hoy USB/UART; con CAN, un twai_transmit().
void enviarTrama(const uint8_t trama[8]) {
  Serial.write(SYNC);
  Serial.write(trama, 8);
}

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(pinFreeDrive, INPUT_PULLUP);
  pinMode(pinJoyBot, INPUT_PULLUP);
  for (int i = 0; i < 5; ++i) {
    pinMode(pinJoint[i], INPUT_PULLUP);
  }

  // No tocar el joystick al encender.
  centroX = calibrarCentro(pinX);
  centroY = calibrarCentro(pinY);
  ultimoEnvio = millis();
}

void loop() {
  if (millis() - ultimoEnvio < periodoMs) {
    return;
  }
  ultimoEnvio += periodoMs;

  bool manual = digitalRead(pinFreeDrive) == LOW;

  // Selección: el último botón pulsado de cada eje
  for (int i = 0; i < 5; ++i) {
    if (digitalRead(pinJoint[i]) == LOW) {
      if (ejeJoint[i] == 'X') {
        jointX = i + 1;
      } else {
        jointY = i + 1;
      }
    }
  }

  // Botón del joystick: alterna el gripper
  bool estadoActualJoystickBtn = digitalRead(pinJoyBot);
  if (estadoAnteriorJoystickBtn == HIGH && estadoActualJoystickBtn == LOW &&
      millis() - ultimoDebounce > debounceDelay) {
    gripperActivo = !gripperActivo;
    ultimoDebounce = millis();
  }
  estadoAnteriorJoystickBtn = estadoActualJoystickBtn;

  // Fuera de MANUAL el joystick no mueve nada.
  int16_t x = manual ? normalizar(analogRead(pinX), centroX, invertirX) : 0;
  int16_t y = manual ? normalizar(analogRead(pinY), centroY, invertirY) : 0;

  uint8_t trama[8];
  trama[0] = secuencia++;
  trama[1] = (manual ? 0x01 : 0) | (gripperActivo ? 0x02 : 0);
  trama[2] = (jointX << 4) | jointY;
  trama[3] = x & 0xFF;
  trama[4] = (x >> 8) & 0xFF;
  trama[5] = y & 0xFF;
  trama[6] = (y >> 8) & 0xFF;
  trama[7] = crc8(trama, 7);

#if PENDANT_DEBUG
  Serial.printf("MANUAL %d | X: J%d %5d | Y: J%d %5d | gripper %d\n",
                manual, jointX, x, jointY, y, gripperActivo);
#else
  enviarTrama(trama);
#endif
}
