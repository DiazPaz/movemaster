const int PIN_STEP = 27;
const int PIN_DIR = 26;
const int PIN_EN = 25;

const int PASOS = 1600 * 10;       // pasos por vuelta del motor
const int ESPERA_US = 250;  // µs entre pasos: más grande = más lento

void girar(bool adelante) {
  digitalWrite(PIN_DIR, adelante ? HIGH : LOW);
  for (int i = 0; i < PASOS; i++) {
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(ESPERA_US);
  }
}

void setup() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, LOW);  // LOW = driver habilitado
}

void loop() {
  girar(true);
  delay(1000);
  girar(false);
  delay(1000);
}
