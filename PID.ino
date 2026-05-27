#include <Servo.h>

// ╔══════════════════════════════════════════════════════════════╗
// ║           BOLA-VIGA  —  PID  —  HC-SR04 + SG-5010          ║
// ╚══════════════════════════════════════════════════════════════╝

// ─── Pines ────────────────────────────────────────────────────
const int PIN_TRIG = 7;
const int PIN_ECHO = 8;
const int PIN_SERVO = 9;

// ─── Servo SG-5010 ────────────────────────────────────────────
const int SERVO_CENTER    = 90;   // centro mecánico real
const int SERVO_MAX_DELTA = 20;   // ±20° máximo — protege engranajes
Servo miServo;

// ─── Sensor HC-SR04 ───────────────────────────────────────────
const float DIST_MIN = 3.0;
const float DIST_MAX = 50.0;

// ─── Setpoint ─────────────────────────────────────────────────
float setpoint = 19;            // cm desde el sensor

// ─── Ganancias PID ────────────────────────────────────────────
// Punto de partida conservador — ajustar con Ziegler-Nichols
float Kp = 9.0;
float Ki = 0.1;
float Kd = 4.0;

// ─── Tiempo de muestreo ───────────────────────────────────────
const unsigned long TS_MS = 40;   // 40 ms — mínimo seguro HC-SR04
float dt;

// ─── Estado PID ───────────────────────────────────────────────
float integral        = 0;
float distAnterior    = 0;
float ultimaValida    = 0;
unsigned long tAnt    = 0;

// ─── Filtro de mediana (ventana 5) ────────────────────────────
const int MED_N = 5;
float medBuf[MED_N];
int   medIdx = 0;
bool  medLleno = false;

// ══════════════════════════════════════════════════════════════
//  FUNCIONES AUXILIARES
// ══════════════════════════════════════════════════════════════

// Ordena y devuelve la mediana — robusto contra spikes del sensor
float mediana() {
  float tmp[MED_N];
  int n = medLleno ? MED_N : medIdx;
  if (n == 0) return ultimaValida;
  memcpy(tmp, medBuf, n * sizeof(float));
  for (int i = 0; i < n - 1; i++)
    for (int j = 0; j < n - 1 - i; j++)
      if (tmp[j] > tmp[j+1]) { float t = tmp[j]; tmp[j] = tmp[j+1]; tmp[j+1] = t; }
  return tmp[n / 2];
}

// Lectura limpia del HC-SR04
// Devuelve -1 si el eco no llega o está fuera de rango
float leerSensor() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  long us = pulseIn(PIN_ECHO, HIGH, 38000);
  if (us == 0) return -1.0;

  float d = us / 58.0;
  if (d < DIST_MIN || d > DIST_MAX) return -1.0;

  // Solo ingresar al buffer si es válido
  medBuf[medIdx] = d;
  medIdx = (medIdx + 1) % MED_N;
  if (medIdx == 0) medLleno = true;

  return mediana();
}

// Escribe el servo con µs para máxima precisión en el SG-5010
// y aplica límite físico antes de mover
void escribirServo(float angulo) {
  int ang = constrain((int)angulo,
                      SERVO_CENTER - SERVO_MAX_DELTA,
                      SERVO_CENTER + SERVO_MAX_DELTA);
  miServo.writeMicroseconds(map(ang, 0, 180, 1000, 2000));
}

// Parser de comandos serie — cambia ganancias sin recompilar
// Formato: p8.0  i0.04  d4.0  s20.0
void leerSerial() {
  if (!Serial.available()) return;
  char cmd = Serial.read();
  float val = Serial.parseFloat();
  bool ok = true;
  switch (cmd) {
    case 'p': Kp = val; break;
    case 'i': Ki = val; integral = 0; break;
    case 'd': Kd = val; break;
    case 's': setpoint = constrain(val, DIST_MIN, DIST_MAX);
              integral = 0; break;
    default:  ok = false; break;
  }
  if (ok) {
    Serial.print("Kp="); Serial.print(Kp, 3);
    Serial.print(" Ki="); Serial.print(Ki, 4);
    Serial.print(" Kd="); Serial.print(Kd, 3);
    Serial.print(" SP="); Serial.println(setpoint, 1);
  }
}

// ══════════════════════════════════════════════════════════════
//  SETUP
// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  miServo.attach(PIN_SERVO, 1000, 2000);
  escribirServo(SERVO_CENTER);
  delay(500);

  // Prellenar buffer con primera lectura real
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  long d0 = pulseIn(PIN_ECHO, HIGH, 38000);
  float inicio = (d0 > 0) ? d0 / 58.0 : setpoint;
  inicio = constrain(inicio, DIST_MIN, DIST_MAX);
  for (int i = 0; i < MED_N; i++) medBuf[i] = inicio;
  medLleno     = true;
  distAnterior = inicio;
  ultimaValida = inicio;

  tAnt = millis();
  dt   = TS_MS / 1000.0;

  Serial.println("=== Bola-Viga listo ===");
  Serial.println("Comandos: p<Kp>  i<Ki>  d<Kd>  s<setpoint>");
  Serial.println("dist,sp,P,I,D,salida");
}

// ══════════════════════════════════════════════════════════════
//  LOOP
// ══════════════════════════════════════════════════════════════
void loop() {
  leerSerial();

  if (millis() - tAnt < TS_MS) return;
  tAnt = millis();

  // ── 1. Lectura ─────────────────────────────────────────────
  float dist = leerSensor();

  if (dist < 0) {
    // Fuera de rango: el PID sigue con la última posición conocida
    // El error sigue siendo grande → el servo empuja de vuelta
    dist = ultimaValida;
  } else {
    ultimaValida = dist;
  }

  // ── 2. Error ───────────────────────────────────────────────
  float error = setpoint - dist;

  // ── 3. Proporcional ────────────────────────────────────────
  float P = Kp * error;

  // ── 4. Derivativo sobre la MEDIDA (no sobre el error)
  //    Evita "derivative kick": correcciones bruscas al
  //    cambiar setpoint o cuando la bola se acerca rápido
  float D = -Kd * (dist - distAnterior) / dt;

  // ── 5. Anti-windup condicional ─────────────────────────────
  //    Solo acumula integral si la salida P+D no está saturada
  //    Evita que el integrador se dispare en los extremos
  float salidaPD = P + D;
  if (abs(salidaPD) < SERVO_MAX_DELTA) {
    integral += error * dt;
    integral  = constrain(integral, -8.0, 8.0);
  }
  float I = Ki * integral;

  // ── 6. Salida total ────────────────────────────────────────
  float salida = constrain(P + I + D, -SERVO_MAX_DELTA, SERVO_MAX_DELTA);

  // ── 7. Mover servo ─────────────────────────────────────────
  // Signo negativo: ajustar si la viga compensa al revés
  escribirServo(SERVO_CENTER - salida);

  // ── 8. Actualizar estado ───────────────────────────────────
  distAnterior = dist;

  // ── 9. Telemetría — compatible con Serial Plotter ──────────
  // Formato: dist,sp,P,I,D,salida
  Serial.print(dist,    2); Serial.print(",");
  Serial.print(setpoint,1); Serial.print(",");
  Serial.print(P,       2); Serial.print(",");
  Serial.print(I,       2); Serial.print(",");
  Serial.print(D,       2); Serial.print(",");
  Serial.println(salida,2);
}