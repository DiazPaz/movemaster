// Teach pendant de MoveMaster (ESP32), protocolo de texto.
//
// Cada 20 ms imprime por USB un bloque legible en el monitor serie, que la Raspberry Pi
// (jog_node, movemaster_teleop/pendant_text.py) lee y convierte en comandos de ROS:
//
//   Freedrive: ON
//   Joint Y: J1 | Angulo: 12.3 grados | Joystick Y: -450
//   Joint X: J4 | Angulo: -5.0 grados | Joystick X: 0
//   Gripper: OFF
//   -------------------------
//
// Freedrive es el switch MANUAL/AUTO. Joystick va de -1000 a 1000 alrededor del centro
// calibrado al encender, y es 0 en AUTO. Gripper es el estado que alterna el botón del
// joystick: ON cierra el gripper. El ESP32 no mueve nada ni calcula ángulos. La Raspberry Pi
// le manda, 10 veces por segundo, los ángulos medidos en el robot (/joint_states) en grados
// con Home en 0 y el estado real del gripper (ON cerrado, OFF abierto):
//
//   ANGLES,12.3,-20.5,33.3,0.0,45.0
//   GRIPPER,ON
//
// Antes de recibir ángulos, se imprime "--". Gripper sigue al estado real, así cada presión
// lo alterna aunque el HMI lo haya movido; durante un segundo tras una presión se ignora
// GRIPPER, que aún puede ser el de antes. El formato de las líneas debe coincidir con
// pendant_text.py: si cambia uno, cambia el otro.

#include <Arduino.h>

// Joystick
const int pinX = 34;
const int pinY = 35;
const int pinJoyBot = 18;

// Switch Freedrive
const int pinFreeDrive = 19;

// Botones grupo Y
// J1, J2, J3
const int pinJ1 = 21;
const int pinJ2 = 22;
const int pinJ3 = 23;

// Botones grupo X
// J4, J5
const int pinJ4 = 25;
const int pinJ5 = 26;

// Ángulos medidos que manda la Raspberry Pi
const int numJoints = 5;
float jointAngles[numJoints] = {0.0, 0.0, 0.0, 0.0, 0.0};
bool angulosRecibidos = false;
char lineaRx[96];
int largoRx = 0;

// Joint seleccionado
int jointY = 1;
int jointX = 4;

// Joystick: centro calibrado al encender y escala de salida
const int adc_max = 4095;
int centroX = 2048;
int centroY = 2048;
const int escala = 1000;

// Envío
const unsigned long periodoMs = 20;
unsigned long ultimoEnvio = 0;

// Gripper
bool gripperActivo = false;

bool estadoAnteriorJoystickBtn = HIGH;

unsigned long ultimoDebounce = 0;  // también es la última presión
const unsigned long debounceDelay = 50;

// Tras una presión, el GRIPPER de la Raspberry Pi puede ser aún el de antes: se ignora este
// tiempo, de sobra para que el comando llegue al robot y este reporte el estado nuevo.
const unsigned long retencionGripperMs = 1000;

// Promedio con el joystick en reposo: el centro real del ADC no es 2048.
int calibrarCentro(int pin) {

  long suma = 0;

  for (int i = 0; i < 64; ++i) {
    suma += analogRead(pin);
    delay(2);
  }

  return suma / 64;
}

// Lectura del ADC a -1000..1000, cada mitad escalada desde el centro calibrado.
int normalizar(int raw, int centro) {

  long valor;

  if (raw >= centro) {
    valor = (long)(raw - centro) * escala / max(1, adc_max - centro);
  } else {
    valor = (long)(raw - centro) * escala / max(1, centro);
  }

  return constrain(valor, -escala, escala);
}

// Una línea de la Raspberry Pi: "ANGLES,a1,a2,..." o "GRIPPER,ON|OFF".
void procesarLinea(char *linea) {

  bool cerrado = strcmp(linea, "GRIPPER,ON") == 0;

  if (cerrado || strcmp(linea, "GRIPPER,OFF") == 0) {
    if (millis() - ultimoDebounce >= retencionGripperMs) {
      gripperActivo = cerrado;
    }
    return;
  }

  if (strncmp(linea, "ANGLES,", 7) != 0) {
    return;
  }

  char *resto = linea + 7;

  for (int i = 0; i < numJoints && resto != NULL && *resto != '\0'; ++i) {
    jointAngles[i] = atof(resto);
    resto = strchr(resto, ',');
    if (resto != NULL) {
      resto++;
    }
  }

  angulosRecibidos = true;
}

// Lee sin esperar lo que haya llegado por USB, línea por línea.
void leerSerial() {

  while (Serial.available() > 0) {

    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (largoRx > 0) {
        lineaRx[largoRx] = '\0';
        procesarLinea(lineaRx);
        largoRx = 0;
      }
    } else if (largoRx < (int)sizeof(lineaRx) - 1) {
      lineaRx[largoRx++] = c;
    } else {
      largoRx = 0;  // línea demasiado larga: se descarta
    }
  }
}

void imprimirJoint(char eje, int joint, int joystick) {

  Serial.print("Joint ");
  Serial.print(eje);
  Serial.print(": J");
  Serial.print(joint);

  Serial.print(" | Angulo: ");

  if (angulosRecibidos && joint >= 1 && joint <= numJoints) {
    Serial.print(jointAngles[joint - 1], 1);
  } else {
    Serial.print("--");
  }

  Serial.print(" grados");

  Serial.print(" | Joystick ");
  Serial.print(eje);
  Serial.print(": ");
  Serial.println(joystick);
}


void setup() {

  Serial.begin(115200);
  analogReadResolution(12);

  pinMode(pinFreeDrive, INPUT_PULLUP);

  pinMode(pinJoyBot, INPUT_PULLUP);

  pinMode(pinJ1, INPUT_PULLUP);
  pinMode(pinJ2, INPUT_PULLUP);
  pinMode(pinJ3, INPUT_PULLUP);

  pinMode(pinJ4, INPUT_PULLUP);
  pinMode(pinJ5, INPUT_PULLUP);

  // No tocar el joystick al encender.
  centroX = calibrarCentro(pinX);
  centroY = calibrarCentro(pinY);

  ultimoEnvio = millis();
}


void loop() {

  leerSerial();

  if (millis() - ultimoEnvio < periodoMs) {
    return;
  }
  ultimoEnvio += periodoMs;

  // Freedrive
  bool freedrive = (
    digitalRead(pinFreeDrive) == LOW
  );

  // Selección grupo Y
  if (digitalRead(pinJ1) == LOW) {
    jointY = 1;
  }

  if (digitalRead(pinJ2) == LOW) {
    jointY = 2;
  }

  if (digitalRead(pinJ3) == LOW) {
    jointY = 3;
  }

  // Selección grupo X
  if (digitalRead(pinJ4) == LOW) {
    jointX = 4;
  }

  if (digitalRead(pinJ5) == LOW) {
    jointX = 5;
  }

  // Botón joystick / Gripper: funciona en MANUAL y en AUTO
  bool estadoActualJoystickBtn =
    digitalRead(pinJoyBot);

  if (
    estadoAnteriorJoystickBtn == HIGH &&
    estadoActualJoystickBtn == LOW &&
    millis() - ultimoDebounce > debounceDelay
  ) {

    gripperActivo = !gripperActivo;

    ultimoDebounce = millis();
  }

  estadoAnteriorJoystickBtn = estadoActualJoystickBtn;

  // Fuera de Freedrive el joystick no mueve nada.
  int joyX = freedrive ? normalizar(analogRead(pinX), centroX) : 0;
  int joyY = freedrive ? normalizar(analogRead(pinY), centroY) : 0;

  // Bloque completo, siempre igual: la Raspberry Pi lo lee entero o lo descarta.
  Serial.print("Freedrive: ");
  Serial.println(freedrive ? "ON" : "OFF");

  imprimirJoint('Y', jointY, joyY);
  imprimirJoint('X', jointX, joyX);

  Serial.print("Gripper: ");
  Serial.println(gripperActivo ? "ON" : "OFF");

  Serial.println("-------------------------");
}
