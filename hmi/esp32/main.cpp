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

// Rango angular
float deg_min = 0.0;
float deg_max = 360.0;

// Posiciones iniciales
float jointAngles[5] = {
  180.0,   // J1
  180.0,   // J2
  180.0,   // J3
  180.0,   // J4
  180.0    // J5
};

// Joint seleccionado
int jointY = 1;
int jointX = 4;

// Joystick
const int adc_min = 0;
const int adc_max = 4095;
const int adc_center = 2048;

// Zona muerta alrededor del centro
const int zonaMuerta = 300;

// Velocidad

// Tiempo mínimo entre pasos cuando
// el joystick está al máximo
const unsigned long delayMin = 20;

// Tiempo máximo cuando apenas sale
// de la zona muerta
const unsigned long delayMax = 250;

unsigned long ultimoMovimientoX = 0;
unsigned long ultimoMovimientoY = 0;

// Gripper
bool gripperActivo = false;

bool estadoAnteriorJoystickBtn = HIGH;

unsigned long ultimoDebounce = 0;
const unsigned long debounceDelay = 50;

// Función para limitar ángulo
float limitarAngulo(float angulo) {

  if (angulo > deg_max) {
    return deg_max;
  }

  if (angulo < deg_min) {
    return deg_min;
  }

  return angulo;
}

// Calcular intervalo de movimiento
unsigned long calcularDelay(int valorJoystick) {

  int magnitud = abs(valorJoystick - adc_center);

  if (magnitud < zonaMuerta) {
    return delayMax;
  }

  magnitud = constrain(
    magnitud,
    zonaMuerta,
    adc_center
  );

  return map(
    magnitud,
    zonaMuerta,
    adc_center,
    delayMax,
    delayMin
  );
}


void setup() {

  Serial.begin(115200);

  pinMode(pinFreeDrive, INPUT_PULLUP);

  pinMode(pinJoyBot, INPUT_PULLUP);

  pinMode(pinJ1, INPUT_PULLUP);
  pinMode(pinJ2, INPUT_PULLUP);
  pinMode(pinJ3, INPUT_PULLUP);

  pinMode(pinJ4, INPUT_PULLUP);
  pinMode(pinJ5, INPUT_PULLUP);
}


void loop() {

  // Freedrive
  bool freedrive = (
    digitalRead(pinFreeDrive) == LOW
  );

  if (!freedrive) {

    Serial.println("Freedrive: OFF");

    delay(100);
    return;
  }

  // Freedrive ON
  Serial.println("Freedrive: ON");

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

  // Lectura joystick
  int rawX = analogRead(pinX);
  int rawY = analogRead(pinY);

  // Movimiento eje Y
  unsigned long delayY =
    calcularDelay(rawY);

  if (
    abs(rawY - adc_center) > zonaMuerta &&
    millis() - ultimoMovimientoY >= delayY
  ) {

    if (rawY > adc_center) {

      jointAngles[jointY - 1] += 1.0;

    } else {

      jointAngles[jointY - 1] -= 1.0;
    }

    jointAngles[jointY - 1] =
      limitarAngulo(
        jointAngles[jointY - 1]
      );

    ultimoMovimientoY = millis();
  }

  // Movimiento eje X
  unsigned long delayX =
    calcularDelay(rawX);

  if (
    abs(rawX - adc_center) > zonaMuerta &&
    millis() - ultimoMovimientoX >= delayX
  ) {

    if (rawX > adc_center) {

      jointAngles[jointX - 1] += 1.0;

    } else {

      jointAngles[jointX - 1] -= 1.0;
    }

    jointAngles[jointX - 1] =
      limitarAngulo(
        jointAngles[jointX - 1]
      );

    ultimoMovimientoX = millis();
  }

  // Botón joystick / Gripper
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

  // Mostrar información
  Serial.print("Joint Y: J");
  Serial.print(jointY);

  Serial.print(" | Angulo: ");
  Serial.print(
    jointAngles[jointY - 1],
    1
  );

  Serial.print(" grados");

  Serial.print(" | Velocidad Y: ");
  Serial.println(
    abs(rawY - adc_center)
  );


  Serial.print("Joint X: J");
  Serial.print(jointX);

  Serial.print(" | Angulo: ");
  Serial.print(
    jointAngles[jointX - 1],
    1
  );

  Serial.print(" grados");

  Serial.print(" | Velocidad X: ");
  Serial.println(
    abs(rawX - adc_center)
  );


  Serial.print("Gripper: ");

  if (gripperActivo) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }

  Serial.println("-------------------------");

  delay(20);
}