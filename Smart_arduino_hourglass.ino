#include <Arduino.h>
#include <TM1637Display.h>
#include "LedControl.h"
#include "Delay.h"

// --- Constantes hardware ---
#define  MATRIX_A  0
#define  MATRIX_B  1

#define ACC_THRESHOLD_LOW   335
#define ACC_THRESHOLD_HIGH  400

// Matrices MAX7219
#define PIN_DATAIN 5
#define PIN_CLK    4
#define PIN_LOAD   6

// Acelerómetro
#define PIN_X A1
#define PIN_Y A2

// Botones
#define PIN_BUTTON   3   // botón pin 3
#define PIN_BUTTON2  7   // botón pin 7

// Buzzer
#define PIN_BUZZER   8

// Rotación física de las matrices
#define ROTATION_OFFSET 90

// Tiempo entre frames base
#define DELAY_FRAME 100

// Long press
#define PRESS_LONG 3000UL

// Auto-start tras inactividad
#define START_AFTER_IDLE_MS 5000UL

// Número de granos
const int NUM_PARTICLES = 64;

// Orientación estable
const unsigned long ORIENTATION_STABLE_MS = 500UL;

// --- Estado ---
enum State {
  STATE_IDLE,
  STATE_RUNNING,
  STATE_PAUSED,
  STATE_FINISHED
};

State state = STATE_IDLE;

// --- Tiempo configurado / restante ---
int setMinutes = 0;
int setSeconds = 0;

unsigned long configuredSeconds = 0;
long          remainingSeconds  = 0;

unsigned long lastUserActionMs = 0;

// --- Modo configuración (min/seg) ---
bool modeMinutes = false; // false=segundos, true=minutos

// --- Gravedad ---
int gravity = 0;
int lastRawGravity = 0;
unsigned long gravityChangeMillis = 0;

// --- Control matrices y delays ---
LedControl lc(PIN_DATAIN, PIN_CLK, PIN_LOAD, 2);
NonBlockDelay d; // ahora sólo se usa como pequeño limitador puntual

// --- Display TM1637 ---
TM1637Display tm(10, 11);

// --- Botones (polling con tiempo) ---
bool b1Down = false;
unsigned long b1DownMs = 0;

bool b2Down = false;
unsigned long b2DownMs = 0;

// --- Sincronización exacta display + arena ---
unsigned long runStartMs   = 0;    // momento en que arranca la cuenta
unsigned long pauseAccumMs = 0;    // tiempo acumulado en pausas
unsigned long pauseStartMs = 0;    // inicio de la pausa
unsigned long totalRunMs   = 0;    // configuredSeconds * 1000
int           droppedCount = 0;    // granos que ya han caído (0..64)

// -----------------------------------------------------------------------------
// AUX: Display
// -----------------------------------------------------------------------------
void displayTimeMMSS(int minutes, int seconds) {
  minutes = constrain(minutes, 0, 59);
  seconds = constrain(seconds, 0, 59);
  int value = minutes * 100 + seconds;
  tm.showNumberDecEx(value, 0b01000000, true); // enciende ':'
}

// -----------------------------------------------------------------------------
// AUX: Partículas (movimiento interno)
// -----------------------------------------------------------------------------
coord getDown(int x, int y) { coord xy; xy.x = x - 1; xy.y = y + 1; return xy; }
coord getLeft(int x, int y) { coord xy; xy.x = x - 1; xy.y = y;     return xy; }
coord getRight(int x, int y){ coord xy; xy.x = x;     xy.y = y + 1; return xy; }

bool canGoLeft(int addr, int x, int y) {
  if (x == 0) return false;
  return !lc.getXY(addr, getLeft(x, y));
}
bool canGoRight(int addr, int x, int y) {
  if (y == 7) return false;
  return !lc.getXY(addr, getRight(x, y));
}
bool canGoDown(int addr, int x, int y) {
  if (y == 7) return false;
  if (x == 0) return false;
  if (!canGoLeft(addr, x, y))  return false;
  if (!canGoRight(addr, x, y)) return false;
  return !lc.getXY(addr, getDown(x, y));
}

void goDown(int addr, int x, int y)  { lc.setXY(addr, x, y, false); lc.setXY(addr, getDown(x, y), true); }
void goLeft(int addr, int x, int y)  { lc.setXY(addr, x, y, false); lc.setXY(addr, getLeft(x, y), true); }
void goRight(int addr, int x, int y) { lc.setXY(addr, x, y, false); lc.setXY(addr, getRight(x, y), true); }

bool moveParticle(int addr, int x, int y) {
  if (!lc.getXY(addr, x, y)) return false;

  bool can_L = canGoLeft(addr, x, y);
  bool can_R = canGoRight(addr, x, y);
  if (!can_L && !can_R) return false;

  bool can_D = canGoDown(addr, x, y);
  if (can_D) {
    goDown(addr, x, y);
  } else if (can_L && !can_R) {
    goLeft(addr, x, y);
  } else if (can_R && !can_L) {
    goRight(addr, x, y);
  } else if (random(2) == 1) {
    goLeft(addr, x, y);
  } else {
    goRight(addr, x, y);
  }
  return true;
}

bool updateMatrix() {
  int n = 8;
  bool moved = false;
  byte x, y;
  bool direction;

  for (byte slice = 0; slice < 2 * n - 1; ++slice) {
    direction = (random(2) == 1);
    byte z = (slice < n) ? 0 : slice - n + 1;
    for (byte j = z; j <= slice - z; ++j) {
      y = direction ? (7 - j) : (7 - (slice - j));
      x = direction ? (slice - j) : j;
      if (moveParticle(MATRIX_B, x, y)) moved = true;
      if (moveParticle(MATRIX_A, x, y)) moved = true;
    }
  }
  return moved;
}

int countParticles(int addr) {
  int c = 0;
  for (byte y = 0; y < 8; y++)
    for (byte x = 0; x < 8; x++)
      if (lc.getXY(addr, x, y)) c++;
  return c;
}

void fill(int addr, int maxcount) {
  int n = 8;
  byte x, y;
  int count = 0;
  for (byte slice = 0; slice < 2 * n - 1; ++slice) {
    byte z = slice < n ? 0 : slice - n + 1;
    for (byte j = z; j <= slice - z; ++j) {
      y = 7 - j;
      x = (slice - j);
      lc.setXY(addr, x, y, (++count <= maxcount));
    }
  }
}

// -----------------------------------------------------------------------------
// Gravedad / orientación
// -----------------------------------------------------------------------------
int getGravityRaw() {
  int x = analogRead(PIN_X);
  int y = analogRead(PIN_Y);

  if (y < ACC_THRESHOLD_LOW)  return 0;
  if (x > ACC_THRESHOLD_HIGH) return 90;
  if (y > ACC_THRESHOLD_HIGH) return 180;
  if (x < ACC_THRESHOLD_LOW)  return 270;
  return 0;
}

bool isOpposite180(int g0, int g1) {
  return ((g0 + 180) % 360) == (g1 % 360);
}

int getTopMatrix() {
  return (gravity == 90) ? MATRIX_A : MATRIX_B;
}

// -----------------------------------------------------------------------------
// Tiempo transcurrido real de la cuenta (sin pausas)
// -----------------------------------------------------------------------------
unsigned long elapsedRunMs() {
  if (runStartMs == 0) return 0;
  unsigned long now = millis();
  unsigned long base = now - runStartMs;
  if (base > pauseAccumMs) base -= pauseAccumMs;
  else base = 0;
  return base;
}

// -----------------------------------------------------------------------------
// Drop de 1 grano: mueve un LED de arriba a abajo
//   → la matriz de abajo se rellena con el MISMO patrón de fill()
// -----------------------------------------------------------------------------
bool dropOneGrainStep() {
  int top    = getTopMatrix();
  int bottom = (top == MATRIX_A) ? MATRIX_B : MATRIX_A;

  bool dropped = false;

  // Agujero central si está vertical
  if (gravity == 0 || gravity == 180) {
    if ((lc.getRawXY(MATRIX_A, 0, 0) && !lc.getRawXY(MATRIX_B, 7, 7)) ||
        (!lc.getRawXY(MATRIX_A, 0, 0) && lc.getRawXY(MATRIX_B, 7, 7))) {

      lc.invertRawXY(MATRIX_A, 0, 0);
      lc.invertRawXY(MATRIX_B, 7, 7);
      dropped = true;
    }
  }

  // Fallback: garantizar que un LED pasa de top a bottom
  if (!dropped && countParticles(top) > 0) {
    bool removed = false;
    for (byte y = 0; y < 8 && !removed; y++) {
      for (byte x = 0; x < 8 && !removed; x++) {
        if (lc.getRawXY(top, x, y)) {
          lc.setRawXY(top, x, y, false);
          removed = true;
        }
      }
    }
    if (removed) dropped = true;
  }

  if (dropped) {
    droppedCount++;
    if (droppedCount > NUM_PARTICLES) droppedCount = NUM_PARTICLES;

    // La matriz inferior se dibuja con el mismo patrón de fill()
    lc.clearDisplay(bottom);
    fill(bottom, droppedCount);

    tone(PIN_BUZZER, 440, 8);
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
// Reset total
// -----------------------------------------------------------------------------
void resetTotalToZero() {
  setMinutes = 0;
  setSeconds = 0;

  configuredSeconds = 0;
  remainingSeconds  = 0;

  runStartMs   = 0;
  pauseAccumMs = 0;
  pauseStartMs = 0;
  totalRunMs   = 0;
  droppedCount = 0;

  lc.clearDisplay(0);
  lc.clearDisplay(1);

  displayTimeMMSS(0, 0);
  state = STATE_IDLE;

  lastUserActionMs = millis();
  noTone(PIN_BUZZER);
}

// -----------------------------------------------------------------------------
// Preparar / iniciar cuenta (auto-start)
// -----------------------------------------------------------------------------
void startRunFromConfigured() {
  configuredSeconds = (unsigned long)setMinutes * 60UL + (unsigned long)setSeconds;
  configuredSeconds = constrain(configuredSeconds, 0UL, 3599UL);
  if (configuredSeconds == 0) return;

  remainingSeconds = (long)configuredSeconds;

  totalRunMs   = configuredSeconds * 1000UL;
  runStartMs   = millis();
  pauseAccumMs = 0;
  pauseStartMs = 0;
  droppedCount = 0;

  // Top lleno, bottom vacío
  lc.clearDisplay(0);
  lc.clearDisplay(1);
  fill(getTopMatrix(), NUM_PARTICLES);

  // Pequeño delay inicial simbólico
  d.Delay(50);

  displayTimeMMSS((int)(remainingSeconds / 60), (int)(remainingSeconds % 60));
  state = STATE_RUNNING;
}

// -----------------------------------------------------------------------------
// Actualización sincronizada: display + caída de arena
// -----------------------------------------------------------------------------
void updateRunSynced() {
  if (state != STATE_RUNNING) return;
  if (totalRunMs == 0) return;

  unsigned long e = elapsedRunMs();
  if (e > totalRunMs) e = totalRunMs;

  // 1) Display: remainingSeconds = ceil((totalRunMs - e)/1000)
  unsigned long remainingMs = totalRunMs - e;
  long newRemainingSeconds = (long)((remainingMs + 999UL) / 1000UL); // ceil
  if (newRemainingSeconds < 0) newRemainingSeconds = 0;

  if (newRemainingSeconds != remainingSeconds) {
    remainingSeconds = newRemainingSeconds;
    int m = (int)(remainingSeconds / 60);
    int s = (int)(remainingSeconds % 60);
    displayTimeMMSS(m, s);
  }

  // 2) Granos esperados según tiempo: floor(e * 64 / totalRunMs)
  int expectedDropped = (int)((e * (unsigned long)NUM_PARTICLES) / totalRunMs);
  if (expectedDropped > NUM_PARTICLES) expectedDropped = NUM_PARTICLES;

  // 🔹 NUEVA VERSIÓN:
  // Sin d.Timeout/d.Delay dentro: dejamos que caigan tantos granos
  // como "pida" el tiempo. Para tiempos largos, el bucle se ejecuta
  // muchas veces pero expectedDropped sólo aumenta de uno en uno,
  // así que visualmente sigue siendo 1 grano cada cierto tiempo.
  while (droppedCount < expectedDropped) {
    if (!dropOneGrainStep()) break;
  }

  // 3) Final
  if (e >= totalRunMs) {
    displayTimeMMSS(0, 0);
    tone(PIN_BUZZER, 200, 300);
    state = STATE_FINISHED;
  }
}

// -----------------------------------------------------------------------------
// Auto-start tras 5s sin pulsar botones (solo en IDLE)
// -----------------------------------------------------------------------------
void handleAutoStart() {
  if (state != STATE_IDLE) return;
  if (setMinutes == 0 && setSeconds == 0) return;

  if (millis() - lastUserActionMs >= START_AFTER_IDLE_MS) {
    startRunFromConfigured();
  }
}

// -----------------------------------------------------------------------------
// Botones
//   PIN3 long  -> cambia modo (min/seg)
//   PIN7 short -> +1 en el modo activo (sólo en IDLE)
//   PIN7 long  -> reset total
// -----------------------------------------------------------------------------
void handleButtons() {
  unsigned long now = millis();

  // PIN 3 (modo)
  if (digitalRead(PIN_BUTTON) == LOW) {
    if (!b1Down) { b1Down = true; b1DownMs = now; }
  } else if (b1Down) {
    unsigned long dt = now - b1DownMs;
    b1Down = false;
    lastUserActionMs = now;

    if (dt > PRESS_LONG) {
      modeMinutes = !modeMinutes;
      tone(PIN_BUZZER, 600, 50);
      if (state == STATE_IDLE) displayTimeMMSS(setMinutes, setSeconds);
    }
  }

  // PIN 7 (incremento / reset)
  if (digitalRead(PIN_BUTTON2) == LOW) {
    if (!b2Down) { b2Down = true; b2DownMs = now; }
  } else if (b2Down) {
    unsigned long dt = now - b2DownMs;
    b2Down = false;
    lastUserActionMs = now;

    if (dt > PRESS_LONG) {
      resetTotalToZero();
      return;
    }

    if (state != STATE_IDLE) return;

    if (modeMinutes) setMinutes = (setMinutes + 1) % 60;
    else            setSeconds = (setSeconds + 1) % 60;

    displayTimeMMSS(setMinutes, setSeconds);
  }
}

// -----------------------------------------------------------------------------
// Orientación: RESET en cualquier giro 180°, PAUSE a 90/270
// -----------------------------------------------------------------------------
void handleOrientation() {
  unsigned long now = millis();
  int raw = getGravityRaw();

  if (raw != lastRawGravity) {
    lastRawGravity = raw;
    gravityChangeMillis = now;
  }

  if (raw != gravity && (now - gravityChangeMillis >= ORIENTATION_STABLE_MS)) {
    int oldGravity = gravity;
    gravity = raw;

    lc.setRotation((ROTATION_OFFSET + gravity) % 360);

    // Giro 180° estable => reset total
    if (isOpposite180(oldGravity, gravity)) {
      resetTotalToZero();
      return;
    }

    // PAUSE al ponerlo de lado
    if (state == STATE_RUNNING && (gravity == 90 || gravity == 270)) {
      state = STATE_PAUSED;
      pauseStartMs = millis();
      return;
    }

    // Reanudar desde PAUSED si vuelve a vertical sin 180°
    if (state == STATE_PAUSED) {
      if (gravity == 0 || gravity == 180) {
        unsigned long now2 = millis();
        pauseAccumMs += (now2 - pauseStartMs);
        state = STATE_RUNNING;
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Setup / Loop
// -----------------------------------------------------------------------------
void setup() {
  pinMode(PIN_BUTTON,  INPUT_PULLUP);
  pinMode(PIN_BUTTON2, INPUT_PULLUP);
  pinMode(PIN_BUZZER,  OUTPUT);
  noTone(PIN_BUZZER);

  tm.setBrightness(2, true);
  randomSeed(analogRead(A0));

  for (byte i = 0; i < 2; i++) {
    lc.shutdown(i, false);
    lc.setIntensity(i, 0);
  }

  gravity = getGravityRaw();
  lastRawGravity = gravity;
  lc.setRotation((ROTATION_OFFSET + gravity) % 360);

  resetTotalToZero();
  lastUserActionMs = millis();
}

void loop() {
  // 🔹 Delay dinámico: para tiempos largos se queda en 100 ms,
  // para tiempos cortos se reduce para poder tirar 64 granos
  // sin tener que "amontonarlos" en el mismo frame.
  unsigned long frameDelay = DELAY_FRAME;

  if (state == STATE_RUNNING && totalRunMs > 0) {
    unsigned long ideal = totalRunMs / NUM_PARTICLES; // ms entre granos
    if (ideal < frameDelay) {
      if (ideal < 5) ideal = 5;   // no bajar de 5 ms para no saturar
      frameDelay = ideal;
    }
  }

  delay(frameDelay);

  handleButtons();
  handleOrientation();

  if (state == STATE_IDLE) {
    displayTimeMMSS(setMinutes, setSeconds);
    handleAutoStart();
    return;
  }

  if (state == STATE_PAUSED) {
    int m = (int)(remainingSeconds / 60);
    int s = (int)(remainingSeconds % 60);
    displayTimeMMSS(m, s);
    return;
  }

  if (state == STATE_FINISHED) {
    displayTimeMMSS(0, 0);
    return;
  }

  // RUNNING
  updateRunSynced();   // display + arena sincronizados
  updateMatrix();      // movimiento interno "reloj de arena"
}
