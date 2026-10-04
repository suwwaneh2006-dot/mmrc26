#include <algorithm>
#include <cstdarg>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "Arduino.h"
#include "Preferences.h"
#include "Wire.h"
#include "esp_timer.h"
#include "soc/gpio_struct.h"

#include "config.h"
#include "world.h"

gpio_dev_t GPIO;
SimSerial Serial;
TwoWire Wire;

namespace {

constexpr uint32_t CALL_COST_US = 2;
constexpr uint32_t PHYS_DT_US   = 500;
constexpr float    PI_F = 3.14159265f;
constexpr float    DEG = PI_F / 180.0f;
constexpr float    WALL = 12.0f;
constexpr float    HALF_WALL = 6.0f;

constexpr float    BODY_FRONT = ROBOT_NOSE_X_MM;
constexpr float    BODY_BACK = ROBOT_TAIL_X_MM;
constexpr float    BODY_HALF_W = ROBOT_BODY_HALF_WIDTH_MM;
constexpr float    FRONT_HALF_W = ROBOT_FRONT_HALF_WIDTH_MM;
constexpr float    WHEEL_HALF_W = ROBOT_HALF_WIDTH_MM;
constexpr float    WHEEL_HALF_L = ROBOT_WHEEL_HALF_LENGTH_MM;
constexpr float    TAPER_X = BODY_FRONT - 33.0f;
constexpr float    SOUND_MM_PER_US = 0.3434f;
constexpr uint32_t ECHO_RISE_DELAY_US = 450;

uint64_t g_now = 0;
bool     g_inAdvance = false;
uint8_t  g_level[40];
uint32_t g_duty[40];
void   (*g_isr[40])(void*);
void*    g_isrArg[40];

struct Timer {
  esp_timer_cb_t cb;
  void* arg;
  uint64_t period;
  uint64_t next;
  bool running;
};
Timer    g_timers[4];
int      g_timerCount = 0;

std::mt19937 g_rng(1);
float gauss(float sigma) {
  std::normal_distribution<float> n(0.0f, sigma);
  return n(g_rng);
}

struct Rect {
  float x0, x1, y0, y1;
};
TrueMaze g_maze;
world::Params g_p;
std::vector<float> g_lx, g_ly;
std::vector<Rect>  g_rects;
std::vector<std::vector<int>> g_bucket;
int g_startX = 0;

void addRect(float x0, float x1, float y0, float y1) { g_rects.push_back(Rect{x0, x1, y0, y1}); }

void buildGeometry() {
  std::uniform_real_distribution<float> inner(CELL_INNER_MM * (1.0f - g_p.cellTolerance),
                                              CELL_INNER_MM * (1.0f + g_p.cellTolerance));
  g_lx.assign(1, 0.0f);
  g_ly.assign(1, 0.0f);
  for (int i = 0; i < g_maze.width; ++i) g_lx.push_back(g_lx.back() + inner(g_rng) + WALL);
  for (int j = 0; j < g_maze.height; ++j) g_ly.push_back(g_ly.back() + inner(g_rng) + WALL);
  g_rects.clear();
  for (int i = 0; i <= g_maze.width; ++i) {
    for (int j = 0; j <= g_maze.height; ++j) {
      addRect(g_lx[i] - HALF_WALL, g_lx[i] + HALF_WALL, g_ly[j] - HALF_WALL, g_ly[j] + HALF_WALL);
    }
  }
  for (int x = 0; x < g_maze.width; ++x) {
    for (int y = 0; y < g_maze.height; ++y) {
      if (g_maze.wall[x][y][mm::WEST])
        addRect(g_lx[x] - HALF_WALL, g_lx[x] + HALF_WALL, g_ly[y] + HALF_WALL, g_ly[y + 1] - HALF_WALL);
      if (g_maze.wall[x][y][mm::SOUTH])
        addRect(g_lx[x] + HALF_WALL, g_lx[x + 1] - HALF_WALL, g_ly[y] - HALF_WALL, g_ly[y] + HALF_WALL);
      if (x == g_maze.width - 1 && g_maze.wall[x][y][mm::EAST])
        addRect(g_lx[x + 1] - HALF_WALL, g_lx[x + 1] + HALF_WALL, g_ly[y] + HALF_WALL, g_ly[y + 1] - HALF_WALL);
      if (y == g_maze.height - 1 && g_maze.wall[x][y][mm::NORTH])
        addRect(g_lx[x] + HALF_WALL, g_lx[x + 1] - HALF_WALL, g_ly[y + 1] - HALF_WALL, g_ly[y + 1] + HALF_WALL);
    }
  }

  g_bucket.assign(static_cast<size_t>(g_maze.width * g_maze.height), std::vector<int>());
  for (int x = 0; x < g_maze.width; ++x) {
    for (int y = 0; y < g_maze.height; ++y) {
      const float cx0 = g_lx[x] - WALL, cx1 = g_lx[x + 1] + WALL;
      const float cy0 = g_ly[y] - WALL, cy1 = g_ly[y + 1] + WALL;
      for (size_t r = 0; r < g_rects.size(); ++r) {
        const Rect& q = g_rects[r];
        if (q.x1 >= cx0 && q.x0 <= cx1 && q.y1 >= cy0 && q.y0 <= cy1)
          g_bucket[static_cast<size_t>(x * g_maze.height + y)].push_back(static_cast<int>(r));
      }
    }
  }
}

int cellIndexAt(float x, float y) {
  int cx = 0, cy = 0;
  while (cx < g_maze.width - 1 && x > g_lx[cx + 1]) ++cx;
  while (cy < g_maze.height - 1 && y > g_ly[cy + 1]) ++cy;
  return cx * g_maze.height + cy;
}

struct Robot {
  float x, y, th;
  float vl, vr;
  float omegaDps;
  bool  crashedNow;
};
Robot g_r;
world::Stats g_stats = {0, 0, 0.0, 1e9f, 0, 0.0, 0.0f, 0.0, 0.0f, 0.0f};
bool  g_wasMoving = false;
float g_gyroBias = 0.0f;
bool  g_hand = false;
bool  g_button = false;

float batteryVolts() {
  const float t = static_cast<float>(g_now * 1e-6);
  return 8.1f - 0.7f * fminf(t / 480.0f, 1.0f);
}

void startPose() {
  const int sx = g_startX;
  g_r.x = 0.5f * (g_lx[sx] + g_lx[sx + 1]) + gauss(2.0f);
  g_r.y = 0.5f * (g_ly[0] + g_ly[1]) + gauss(2.0f);
  g_r.th = PI_F / 2.0f + gauss(1.0f) * DEG;
  g_r.vl = g_r.vr = 0.0f;
  g_r.omegaDps = 0.0f;
  g_r.crashedNow = false;
}

float wheelTarget(uint8_t pwm, uint8_t in1, uint8_t in2, float gain, float& tau) {
  const bool stby = g_level[PIN_MOTOR_STBY] != 0;
  const bool a = g_level[in1] != 0, b = g_level[in2] != 0;
  if (!stby || (!a && !b)) {
    tau = 0.15f;
    return 0.0f;
  }
  if (a && b) {
    tau = 0.015f;
    return 0.0f;
  }
  const float v = batteryVolts() * (g_duty[pwm] / static_cast<float>(MOTOR_PWM_MAX)) * (a ? 1.0f : -1.0f);
  if (fabsf(v) < g_p.motorDeadbandV) {
    tau = 0.03f;
    return 0.0f;
  }
  tau = g_p.motorTauS;
  return (v > 0 ? 1.0f : -1.0f) * (fabsf(v) - g_p.motorDeadbandV) * g_p.motorMmSPerV * gain;
}

bool pointInRect(float px, float py, const Rect& r) { return px > r.x0 && px < r.x1 && py > r.y0 && py < r.y1; }

float pointRectDist(float px, float py, const Rect& r) {
  const float dx = fmaxf(fmaxf(r.x0 - px, 0.0f), px - r.x1);
  const float dy = fmaxf(fmaxf(r.y0 - py, 0.0f), py - r.y1);
  return sqrtf(dx * dx + dy * dy);
}

void checkCollision() {
  const float c = cosf(g_r.th), s = sinf(g_r.th);
  const float pts[14][2] = {{BODY_FRONT, FRONT_HALF_W},  {BODY_FRONT, -FRONT_HALF_W}, {TAPER_X, BODY_HALF_W},
                            {TAPER_X, -BODY_HALF_W},     {-BODY_BACK, BODY_HALF_W},   {-BODY_BACK, -BODY_HALF_W},
                            {WHEEL_HALF_L, WHEEL_HALF_W}, {WHEEL_HALF_L, -WHEEL_HALF_W}, {-WHEEL_HALF_L, WHEEL_HALF_W},
                            {-WHEEL_HALF_L, -WHEEL_HALF_W}, {BODY_FRONT, 0},            {-BODY_BACK, 0},
                            {0, WHEEL_HALF_W},            {0, -WHEEL_HALF_W}};
  const std::vector<int>& near = g_bucket[static_cast<size_t>(cellIndexAt(g_r.x, g_r.y))];
  bool hit = false;
  float minGap = 1e9f;
  for (const auto& p : pts) {
    const float wx = g_r.x + p[0] * c - p[1] * s;
    const float wy = g_r.y + p[0] * s + p[1] * c;
    for (int idx : near) {
      const Rect& r = g_rects[static_cast<size_t>(idx)];
      if (pointInRect(wx, wy, r)) hit = true;
      minGap = fminf(minGap, pointRectDist(wx, wy, r));
    }
  }
  if (fabsf(g_r.vl) + fabsf(g_r.vr) > 1.0f) g_stats.minWallClearanceMm = fminf(g_stats.minWallClearanceMm, minGap);
  if (hit && !g_r.crashedNow) {
    ++g_stats.crashes;
    if (fabsf(0.5f * (g_r.vl + g_r.vr)) < 30.0f && fabsf(g_r.omegaDps) > 20.0f) ++g_stats.pivotCrashes;
    Serial.printf("SIM CRASH at x=%.0f y=%.0f heading=%.1f deg\n", g_r.x, g_r.y, g_r.th / DEG);
  }
  g_r.crashedNow = hit;
  if (hit) g_r.vl = g_r.vr = 0.0f;
}

void recordStop(float v) {
  const float speed = fabsf(v) + fabsf(g_r.vr - g_r.vl);
  if (speed > 50.0f) g_wasMoving = true;
  if (!g_wasMoving || speed > 1.0f) return;
  g_wasMoving = false;
  int cx = 0, cy = 0;
  while (cx < g_maze.width - 1 && g_r.x > g_lx[cx + 1]) ++cx;
  while (cy < g_maze.height - 1 && g_r.y > g_ly[cy + 1]) ++cy;
  const float ex = g_r.x - 0.5f * (g_lx[cx] + g_lx[cx + 1]);
  const float ey = g_r.y - 0.5f * (g_ly[cy] + g_ly[cy + 1]);
  const float cardinal = roundf(g_r.th / (PI_F / 2.0f)) * (PI_F / 2.0f);
  const float along = ex * cosf(cardinal) + ey * sinf(cardinal);
  const float lateral = -ex * sinf(cardinal) + ey * cosf(cardinal);
  const float headErr = (g_r.th - cardinal) / DEG;
  ++g_stats.stops;
  g_stats.sumAbsAlongMm += fabsf(along);
  g_stats.maxAbsAlongMm = fmaxf(g_stats.maxAbsAlongMm, fabsf(along));
  g_stats.sumAbsLateralMm += fabsf(lateral);
  g_stats.maxAbsLateralMm = fmaxf(g_stats.maxAbsLateralMm, fabsf(lateral));
  g_stats.maxAbsHeadingDeg = fmaxf(g_stats.maxAbsHeadingDeg, fabsf(headErr));
}

float g_encAcc[2] = {0.0f, 0.0f};
void encoderEdges(int i, uint8_t pin, float travelMm) {
  const float mmPerEdge = ENC_MM_PER_EDGE * g_p.wheelDiameterFactor;
  g_encAcc[i] += fabsf(travelMm);
  while (g_encAcc[i] >= mmPerEdge) {
    g_encAcc[i] -= mmPerEdge;
    g_level[pin] = g_level[pin] ? 0 : 1;
    if (g_isr[pin] != nullptr) g_isr[pin](g_isrArg[pin]);
  }
}

void encoderStep(float leftMm, float rightMm) {
  if (!g_p.encoders) return;
  const bool leftDead = g_p.encoderFailAtS >= 0.0f && g_now * 1e-6 >= g_p.encoderFailAtS;
  if (!leftDead) encoderEdges(0, PIN_ENC_L_A, leftMm);
  encoderEdges(1, PIN_ENC_R_A, rightMm);
}

void physicsStep(float dt) {
  float tauL, tauR;
  const float tl = wheelTarget(PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, 1.0f, tauL);
  const float tr = wheelTarget(PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, g_p.rightWheelGain, tauR);
  g_r.vl += (tl - g_r.vl) * fminf(dt / tauL, 1.0f);
  g_r.vr += (tr - g_r.vr) * fminf(dt / tauR, 1.0f);
  const float v = 0.5f * (g_r.vl + g_r.vr);
  const float w = (g_r.vr - g_r.vl) / WHEEL_TRACK_MM;
  g_r.omegaDps = w / DEG;
  g_r.th += w * dt;
  g_r.x += v * cosf(g_r.th) * dt;
  g_r.y += v * sinf(g_r.th) * dt;
  g_stats.distanceMm += fabs(v * dt);
  g_gyroBias += gauss(0.002f) * sqrtf(dt);
  encoderStep(g_r.vl * dt, g_r.vr * dt);
  checkCollision();
  recordStop(v);
}

struct Sonar {
  uint8_t trig, echo;
  float x, y, angleDeg;
  bool front;
  uint64_t trigHighAt;
  bool pending;
  uint64_t riseAt, fallAt;
};
Sonar g_sonar[3];

bool castRay(float ox, float oy, float ang, float& dist) {
  const float dx = cosf(ang), dy = sinf(ang);
  float best = 1e9f;
  bool bestEcho = false;
  for (const Rect& r : g_rects) {
    float tmin = -1e9f, tmax = 1e9f;
    int axis = -1;
    if (fabsf(dx) < 1e-9f) {
      if (ox <= r.x0 || ox >= r.x1) continue;
    } else {
      float t1 = (r.x0 - ox) / dx, t2 = (r.x1 - ox) / dx;
      if (t1 > t2) std::swap(t1, t2);
      if (t1 > tmin) { tmin = t1; axis = 0; }
      tmax = fminf(tmax, t2);
    }
    if (fabsf(dy) < 1e-9f) {
      if (oy <= r.y0 || oy >= r.y1) continue;
    } else {
      float t1 = (r.y0 - oy) / dy, t2 = (r.y1 - oy) / dy;
      if (t1 > t2) std::swap(t1, t2);
      if (t1 > tmin) { tmin = t1; axis = 1; }
      tmax = fminf(tmax, t2);
    }
    if (tmax < tmin || tmin <= 0.0f || tmin >= best) continue;
    best = tmin;

    const float cosInc = axis == 0 ? fabsf(dx) : fabsf(dy);
    bestEcho = cosInc > cosf(40.0f * DEG);
  }
  dist = best;
  return bestEcho && best < 3500.0f;
}

bool measure(const Sonar& s, float& d) {
  if (s.front && g_hand) {
    d = 50.0f + gauss(2.0f);
    return true;
  }
  const float c = cosf(g_r.th), sn = sinf(g_r.th);
  const float ox = g_r.x + s.x * c - s.y * sn;
  const float oy = g_r.y + s.x * sn + s.y * c;
  const float base = g_r.th + s.angleDeg * DEG;
  bool any = false;
  float best = 1e9f;
  for (float off : {-10.0f, -5.0f, 0.0f, 5.0f, 10.0f}) {
    float dd;
    if (castRay(ox, oy, base + off * DEG, dd) && dd < best) {
      best = dd;
      any = true;
    }
  }
  if (!any) return false;
  d = best + gauss(g_p.sonarNoiseMm) + best * gauss(0.003f);
  return true;
}

void setEchoLevel(Sonar& s, bool high) {
  g_level[s.echo] = high ? 1 : 0;
  const uint32_t bit = 1u << (s.echo - 32);
  if (high) GPIO.in1.val = GPIO.in1.val | bit;
  else GPIO.in1.val = GPIO.in1.val & ~bit;
  if (g_isr[s.echo] != nullptr) g_isr[s.echo](g_isrArg[s.echo]);
}

void triggerSonar(Sonar& s) {
  if (s.pending || g_level[s.echo]) return;
  float d;
  const bool echo = measure(s, d);
  s.pending = true;
  s.riseAt = g_now + ECHO_RISE_DELAY_US;
  s.fallAt = s.riseAt + (echo ? static_cast<uint64_t>(2.0f * d / SOUND_MM_PER_US)
                              : static_cast<uint64_t>(g_p.sonarNoEchoMs * 1000.0f));
}

uint8_t  g_i2cAddr = 0;
uint8_t  g_i2cTx[8];
int      g_i2cTxLen = 0;
uint8_t  g_regPtr = 0;
uint8_t  g_gyroConfig = 0;
uint8_t  g_rx[32];
int      g_rxLen = 0, g_rxPos = 0;

uint8_t mpuRegister(uint8_t reg) {
  static const float lsb[4] = {131.0f, 65.5f, 32.8f, 16.4f};
  const float rate = g_r.omegaDps + g_gyroBias + gauss(g_p.gyroNoiseDps);
  float counts = rate * lsb[(g_gyroConfig >> 3) & 3];
  counts = fmaxf(fminf(counts, 32767.0f), -32768.0f);
  const int16_t gz = static_cast<int16_t>(counts);
  const int16_t az = 16384;
  switch (reg) {
    case 0x75: return 0x68;
    case 0x3F: return static_cast<uint8_t>(static_cast<uint16_t>(az) >> 8);
    case 0x40: return static_cast<uint8_t>(az & 0xFF);
    case 0x47: return static_cast<uint8_t>(static_cast<uint16_t>(gz) >> 8);
    case 0x48: return static_cast<uint8_t>(gz & 0xFF);
    case 0x1B: return g_gyroConfig;
    default:   return 0;
  }
}

void advanceTo(uint64_t target);

}

namespace {
uint64_t g_nextPhys = PHYS_DT_US;
uint64_t g_nextTick = 1000;
world::TickHook g_tickHook = nullptr;
world::LineHook g_lineHook = nullptr;
bool g_echoStdout = true;

void advanceTo(uint64_t target) {
  if (g_inAdvance) return;
  g_inAdvance = true;
  while (true) {
    uint64_t next = g_nextPhys;
    int kind = 0;
    int which = -1;
    if (g_nextTick < next) { next = g_nextTick; kind = 4; }
    for (int i = 0; i < 3; ++i) {
      const Sonar& s = g_sonar[i];
      if (!s.pending) continue;
      const bool risen = g_level[s.echo] != 0;
      const uint64_t t = risen ? s.fallAt : s.riseAt;
      if (t < next) { next = t; kind = risen ? 2 : 1; which = i; }
    }
    for (int i = 0; i < g_timerCount; ++i) {
      if (g_timers[i].running && g_timers[i].next < next) { next = g_timers[i].next; kind = 3; which = i; }
    }
    if (next > target) break;
    g_now = next;
    switch (kind) {
      case 0: physicsStep(PHYS_DT_US * 1e-6f); g_nextPhys += PHYS_DT_US; break;
      case 1: setEchoLevel(g_sonar[which], true); break;
      case 2: g_sonar[which].pending = false; setEchoLevel(g_sonar[which], false); break;
      case 3: g_timers[which].next += g_timers[which].period; g_timers[which].cb(g_timers[which].arg); break;
      case 4: g_nextTick += 1000; if (g_tickHook != nullptr) g_tickHook(g_now); break;
    }
  }
  g_now = target;
  g_inAdvance = false;
}

std::map<std::string, std::string> g_nvs;
std::string g_line;
}

uint32_t micros() {
  if (!g_inAdvance) advanceTo(g_now + CALL_COST_US);
  return static_cast<uint32_t>(g_now);
}
uint32_t millis() {
  if (!g_inAdvance) advanceTo(g_now + CALL_COST_US);
  return static_cast<uint32_t>(g_now / 1000u);
}
void delay(uint32_t ms) { advanceTo(g_now + static_cast<uint64_t>(ms) * 1000u); }
void delayMicroseconds(uint32_t us) { advanceTo(g_now + us); }

void pinMode(uint8_t, uint8_t) {}

void digitalWrite(uint8_t pin, uint8_t val) {
  const uint8_t old = g_level[pin];
  g_level[pin] = val ? 1 : 0;
  for (Sonar& s : g_sonar) {
    if (pin != s.trig) continue;
    if (!old && val) s.trigHighAt = g_now;
    if (old && !val && g_now - s.trigHighAt >= 10) triggerSonar(s);
  }
}

int digitalRead(uint8_t pin) {
  if (pin == PIN_BUTTON) return g_button ? LOW : HIGH;
  return g_level[pin];
}

void attachInterruptArg(uint8_t pin, void (*fn)(void*), void* arg, int) {
  g_isr[pin] = fn;
  g_isrArg[pin] = arg;
}

bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
bool ledcWrite(uint8_t pin, uint32_t duty) {
  g_duty[pin] = duty;
  return true;
}
uint32_t ledcWriteTone(uint8_t, uint32_t freq) { return freq; }

uint32_t analogReadMilliVolts(uint8_t) {
  return static_cast<uint32_t>(batteryVolts() / 3.0f * 1000.0f + gauss(4.0f));
}
void analogSetPinAttenuation(uint8_t, adc_attenuation_t) {}

void SimSerial::begin(unsigned long) {}
void SimSerial::setTxBufferSize(size_t) {}
int SimSerial::printf(const char* fmt, ...) {
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  for (const char* p = buf; *p; ++p) {
    if (*p == '\n') {
      if (g_echoStdout) std::printf("[%8.3f] %s\n", g_now * 1e-6, g_line.c_str());
      if (g_lineHook != nullptr) g_lineHook(g_line.c_str());
      g_line.clear();
    } else {
      g_line += *p;
    }
  }
  return n;
}

bool TwoWire::begin(int, int, uint32_t) { return true; }
void TwoWire::setTimeOut(uint16_t) {}
void TwoWire::beginTransmission(uint8_t address) {
  g_i2cAddr = address;
  g_i2cTxLen = 0;
}
size_t TwoWire::write(uint8_t data) {
  if (g_i2cTxLen < 8) g_i2cTx[g_i2cTxLen++] = data;
  return 1;
}
uint8_t TwoWire::endTransmission(bool) {
  delayMicroseconds(60);
  if (g_i2cAddr != 0x68) return 2;
  if (g_i2cTxLen >= 1) g_regPtr = g_i2cTx[0];
  if (g_i2cTxLen >= 2 && g_regPtr == 0x1B) g_gyroConfig = g_i2cTx[1];
  return 0;
}
size_t TwoWire::requestFrom(uint8_t address, size_t len) {
  delayMicroseconds(static_cast<uint32_t>(25 + 23 * len));
  if (address != 0x68 || len > sizeof(g_rx)) return 0;
  for (size_t i = 0; i < len; ++i) g_rx[i] = mpuRegister(static_cast<uint8_t>(g_regPtr + i));
  g_rxLen = static_cast<int>(len);
  g_rxPos = 0;
  return len;
}
int TwoWire::read() { return g_rxPos < g_rxLen ? g_rx[g_rxPos++] : -1; }

bool Preferences::begin(const char* name, bool readOnly) {
  strncpy(ns_, name, sizeof(ns_) - 1);
  readOnly_ = readOnly;
  if (readOnly) {
    const std::string prefix = std::string(ns_) + "/";
    bool found = false;
    for (const auto& kv : g_nvs) found = found || kv.first.compare(0, prefix.size(), prefix) == 0;
    if (!found) return false;
  }
  open_ = true;
  return true;
}
void Preferences::end() { open_ = false; }
size_t Preferences::getBytes(const char* key, void* buf, size_t maxLen) {
  const auto it = g_nvs.find(std::string(ns_) + "/" + key);
  if (!open_ || it == g_nvs.end() || it->second.size() > maxLen) return 0;
  memcpy(buf, it->second.data(), it->second.size());
  return it->second.size();
}
size_t Preferences::putBytes(const char* key, const void* value, size_t len) {
  if (!open_ || readOnly_) return 0;
  delay(4);
  g_nvs[std::string(ns_) + "/" + key] = std::string(static_cast<const char*>(value), len);
  return len;
}
bool Preferences::clear() {
  if (!open_ || readOnly_) return false;
  const std::string prefix = std::string(ns_) + "/";
  for (auto it = g_nvs.begin(); it != g_nvs.end();) {
    if (it->first.compare(0, prefix.size(), prefix) == 0) it = g_nvs.erase(it);
    else ++it;
  }
  return true;
}

esp_err_t esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* out) {
  if (g_timerCount >= 4) return ESP_FAIL;
  g_timers[g_timerCount] = Timer{args->callback, args->arg, 0, 0, false};
  *out = reinterpret_cast<esp_timer_handle_t>(static_cast<intptr_t>(g_timerCount + 1));
  ++g_timerCount;
  return ESP_OK;
}
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us) {
  const int i = static_cast<int>(reinterpret_cast<intptr_t>(timer)) - 1;
  if (i < 0 || i >= g_timerCount) return ESP_FAIL;
  g_timers[i].period = period_us;
  g_timers[i].next = g_now + period_us;
  g_timers[i].running = true;
  return ESP_OK;
}

namespace world {

void init(const TrueMaze& maze, const Params& p) {
  g_p = p;
  g_rng.seed(p.seed);
  g_maze = p.mirrored ? maze.mirrored() : maze;
  g_startX = p.mirrored ? g_maze.width - 1 : 0;
  buildGeometry();
  g_gyroBias = p.gyroBiasDps;
  const struct { uint8_t trig, echo; float x, y, a; bool front; } cfg[3] = {
    {PIN_SONAR_F_TRIG, PIN_SONAR_F_ECHO, SONAR_F_X_MM, SONAR_F_Y_MM, SONAR_F_ANGLE_DEG, true},
    {PIN_SONAR_L_TRIG, PIN_SONAR_L_ECHO, SONAR_L_X_MM, SONAR_L_Y_MM, SONAR_L_ANGLE_DEG, false},
    {PIN_SONAR_R_TRIG, PIN_SONAR_R_ECHO, SONAR_R_X_MM, SONAR_R_Y_MM, SONAR_R_ANGLE_DEG, false},
  };
  for (int i = 0; i < 3; ++i) {
    g_sonar[i] = Sonar{cfg[i].trig, cfg[i].echo, cfg[i].x, cfg[i].y, cfg[i].a, cfg[i].front, 0, false, 0, 0};
  }
  startPose();
}

uint64_t nowUs() { return g_now; }
void setButton(bool pressed) { g_button = pressed; }
void setHand(bool present) { g_hand = present; }
void teleportToStart() { startPose(); }
void truePose(float& x, float& y, float& thDeg, float& vl, float& vr) {
  x = g_r.x;
  y = g_r.y;
  thDeg = g_r.th / DEG;
  vl = g_r.vl;
  vr = g_r.vr;
}
void setLineHook(LineHook h) { g_lineHook = h; }
void setTickHook(TickHook h) { g_tickHook = h; }
void setEcho(bool echo) { g_echoStdout = echo; }
Stats stats() { return g_stats; }

bool nvsGet(const std::string& ns, const std::string& key, std::string& out) {
  const auto it = g_nvs.find(ns + "/" + key);
  if (it == g_nvs.end()) return false;
  out = it->second;
  return true;
}

}
