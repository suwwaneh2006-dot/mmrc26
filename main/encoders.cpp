// =============================================================================
//  encoders.cpp - see encoders.h
// =============================================================================
#include "encoders.h"

#include <initializer_list>

#include "motors.h"

namespace {

struct Wheel {
  uint8_t           pin;
  volatile uint32_t edges;       // written by the ISR only
  uint32_t          lastEdges;   // at the previous tick
  int8_t            dir;         // +1 / -1, from the motor command
  float             mm;
  float             speed;
  // speed window
  uint32_t          windowEdges;
  int32_t           windowSigned;
  float             windowTime;
  // health
  uint32_t          silentMs;
  bool              faulty;
  uint32_t          recoverEdges;
  uint32_t          drivenSilentMs;   // silent while driven (any other wheel state)
};

Wheel g_left  = {PIN_ENC_L_A, 0, 0, 1, 0.0f, 0.0f, 0, 0, 0.0f, 0, false, 0, 0};
Wheel g_right = {PIN_ENC_R_A, 0, 0, 1, 0.0f, 0.0f, 0, 0, 0.0f, 0, false, 0, 0};
uint32_t g_bothSilentMs = 0;

void IRAM_ATTR edgeIsr(void* arg) {
  Wheel* w = static_cast<Wheel*>(arg);
  w->edges = w->edges + 1;
}

// Direction for the edges of this tick: the sign of the applied voltage,
// unchanged while the motor is (almost) unpowered and still rolling.
void updateDir(Wheel& w, float appliedV) {
  if (appliedV > MOTOR_ZERO_V) w.dir = 1;
  else if (appliedV < -MOTOR_ZERO_V) w.dir = -1;
}

void updateWheel(Wheel& w, float appliedV, bool otherCounting, float dt_s) {
  updateDir(w, appliedV);
  const uint32_t e = w.edges;          // 32-bit read: atomic on the ESP32
  const uint32_t delta = e - w.lastEdges;
  w.lastEdges = e;
  w.mm += w.dir * static_cast<float>(delta) * ENC_MM_PER_EDGE;

  // Speed over a short window, then low-pass.
  w.windowSigned += w.dir * static_cast<int32_t>(delta);
  w.windowTime += dt_s;
  if (w.windowTime * 1000.0f >= ENC_SPEED_WINDOW_MS) {
    const float raw = w.windowSigned * ENC_MM_PER_EDGE / w.windowTime;
    w.speed += (raw - w.speed) * ENC_SPEED_FILTER;
    w.windowSigned = 0;
    w.windowTime = 0.0f;
  }

  // Health: silent while clearly driven and the other wheel is turning.
  const bool driven = fabsf(motors::model().speedFor(appliedV)) >= ENC_DRIVEN_MIN_MM_S;
  if (delta > 0) w.drivenSilentMs = 0;
  else if (driven) w.drivenSilentMs += static_cast<uint32_t>(dt_s * 1000.0f + 0.5f);
  if (delta == 0 && driven && otherCounting) {
    w.silentMs += static_cast<uint32_t>(dt_s * 1000.0f + 0.5f);
    if (w.silentMs >= ENC_SILENT_MS && !w.faulty) {
      w.faulty = true;
      DBG_PRINTF("ENCODER FAULT on GPIO %u - falling back to the motor model\n", w.pin);
    }
  } else if (delta > 0) {
    w.silentMs = 0;
  }
  // Self-healing: a faulty encoder that counts steadily again is trusted again.
  if (w.faulty && delta > 0) {
    w.recoverEdges += delta;
    if (w.recoverEdges >= ENC_RECOVER_EDGES) {
      w.faulty = false;
      w.recoverEdges = 0;
      DBG_PRINTF("encoder on GPIO %u counts again - using it\n", w.pin);
    }
  } else if (!w.faulty) {
    w.recoverEdges = 0;
  }
}

}  // namespace

namespace encoders {

void begin() {
  for (Wheel* w : {&g_left, &g_right}) {
    pinMode(w->pin, INPUT_PULLUP);   // hall outputs are often open collector
    w->edges = 0;
    w->lastEdges = 0;
    attachInterruptArg(w->pin, edgeIsr, w, CHANGE);
  }
}

void update(float dt_s) {
  const bool leftCounting = g_left.edges != g_left.lastEdges;
  const bool rightCounting = g_right.edges != g_right.lastEdges;
  updateWheel(g_left, motors::appliedLeftV(), rightCounting, dt_s);
  updateWheel(g_right, motors::appliedRightV(), leftCounting, dt_s);

  // Both silent while both motors are clearly driven: both encoders dead
  // (supply or ground lost). Blocked wheels are caught by stuck detection.
  const bool bothDriven = fabsf(motors::model().speedFor(motors::appliedLeftV())) >= ENC_DRIVEN_MIN_MM_S &&
                          fabsf(motors::model().speedFor(motors::appliedRightV())) >= ENC_DRIVEN_MIN_MM_S;
  if (!leftCounting && !rightCounting && bothDriven) {
    g_bothSilentMs += static_cast<uint32_t>(dt_s * 1000.0f + 0.5f);
    if (g_bothSilentMs >= ENC_SILENT_MS && healthy()) {
      g_left.faulty = g_right.faulty = true;
      DBG_PRINTF("ENCODER FAULT on both wheels - falling back to the motor model\n");
    }
  } else if (leftCounting || rightCounting) {
    g_bothSilentMs = 0;
  }
}

float leftMm() { return g_left.mm; }
float rightMm() { return g_right.mm; }
float leftSpeed() { return g_left.speed; }
float rightSpeed() { return g_right.speed; }
float speed() { return 0.5f * (g_left.speed + g_right.speed); }
uint32_t leftEdges() { return g_left.edges; }
uint32_t rightEdges() { return g_right.edges; }
bool healthy() { return !g_left.faulty && !g_right.faulty; }
bool leftFaulty() { return g_left.faulty; }
bool rightFaulty() { return g_right.faulty; }
bool leftUsable() { return !g_left.faulty && g_left.drivenSilentMs < ENC_SUSPECT_MS; }
bool rightUsable() { return !g_right.faulty && g_right.drivenSilentMs < ENC_SUSPECT_MS; }

void clearFaults() {
  g_left.faulty = g_right.faulty = false;
  g_left.silentMs = g_right.silentMs = 0;
  g_bothSilentMs = 0;
}

}  // namespace encoders
