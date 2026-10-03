#include "motors.h"

#include <Preferences.h>
#include <esp_timer.h>

#include <initializer_list>

#include "battery.h"

namespace {

constexpr uint32_t MODEL_MAGIC = 0x4D4D5631;

struct Side {
  uint8_t pwmPin, in1Pin, in2Pin;
  bool    invert;
  float   target_v;
  float   applied_v;
};

Side g_left  = {PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, MOTOR_L_INVERT, 0.0f, 0.0f};
Side g_right = {PIN_MOTOR_R_PWM, PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, MOTOR_R_INVERT, 0.0f, 0.0f};

bool g_enabled = false;

volatile Fault    g_fault        = Fault::NONE;
volatile uint32_t g_lastFeedMs   = 0;
volatile bool     g_watchEnabled = false;
volatile uint32_t g_graceUntilMs = 0;

VelocityModel g_model;
bool          g_modelCalibrated = false;
esp_timer_handle_t g_watchdogTimer = nullptr;

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

void writeSide(const Side& s, float volts, float batt_v) {
  const float v = s.invert ? -volts : volts;
  if (fabsf(v) < MOTOR_ZERO_V) {

    digitalWrite(s.in1Pin, HIGH);
    digitalWrite(s.in2Pin, HIGH);
    ledcWrite(s.pwmPin, 0);
    return;
  }
  digitalWrite(s.in1Pin, v > 0.0f ? HIGH : LOW);
  digitalWrite(s.in2Pin, v > 0.0f ? LOW : HIGH);
  const float duty = clampf(fabsf(v) / batt_v, 0.0f, 1.0f);
  ledcWrite(s.pwmPin, static_cast<uint32_t>(duty * MOTOR_PWM_MAX + 0.5f));
}

void slew(Side& s, float dt_s) {
  const float step = MOTOR_SLEW_V_PER_S * dt_s;
  s.applied_v += clampf(s.target_v - s.applied_v, -step, step);
}

void watchdogCallback(void*) {
  if (!g_watchEnabled) return;
  const uint32_t now = millis();
  if (static_cast<int32_t>(g_graceUntilMs - now) > 0) return;
  if (now - g_lastFeedMs > CONTROL_WATCHDOG_MS) {
    digitalWrite(PIN_MOTOR_STBY, LOW);
    g_watchEnabled = false;
    if (g_fault == Fault::NONE) g_fault = Fault::WATCHDOG;
  }
}

void loadModel() {
  Preferences prefs;
  VelocityModel m{};
  bool ok = false;
  if (prefs.begin(NVS_CAL_NAMESPACE, true)) {
    ok = prefs.getBytes("vmodel", &m, sizeof(m)) == sizeof(m) && m.valid();
    prefs.end();
  }
  g_modelCalibrated = ok;
  g_model = ok ? m : motors::defaultModel();
}

}

const char* faultName(Fault f) {
  switch (f) {
    case Fault::NONE:         return "none";
    case Fault::WATCHDOG:     return "watchdog";
    case Fault::LOOP_OVERRUN: return "loop overrun";
    case Fault::BATTERY:      return "battery";
    case Fault::IMU:          return "imu";
    case Fault::SONAR:        return "sonar";
    case Fault::COLLISION:    return "collision";
    case Fault::STUCK:        return "stuck";
    case Fault::USER_ABORT:   return "user abort";
  }
  return "?";
}

bool VelocityModel::valid() const {
  if (magic != MODEL_MAGIC || count < 2 || count > MODEL_MAX_POINTS) return false;
  if (!(tau_s > 0.0f && tau_s < 1.0f)) return false;
  if (!(volts[0] >= 0.0f) || mm_s[0] != 0.0f) return false;
  for (int i = 1; i < count; ++i) {
    if (!(volts[i] > volts[i - 1]) || !(mm_s[i] > mm_s[i - 1])) return false;
  }
  return true;
}

float VelocityModel::speedFor(float v) const {
  const float a = fabsf(v);
  if (a <= volts[0]) return 0.0f;
  int i = 1;
  while (i < count - 1 && a > volts[i]) ++i;
  const float t = (a - volts[i - 1]) / (volts[i] - volts[i - 1]);
  const float s = mm_s[i - 1] + t * (mm_s[i] - mm_s[i - 1]);
  return v < 0.0f ? -s : s;
}

float VelocityModel::voltsFor(float speed_mm_s) const {
  const float a = fabsf(speed_mm_s);
  if (a < 1e-3f) return 0.0f;
  int i = 1;
  while (i < count - 1 && a > mm_s[i]) ++i;
  const float t = (a - mm_s[i - 1]) / (mm_s[i] - mm_s[i - 1]);
  const float v = volts[i - 1] + t * (volts[i] - volts[i - 1]);
  return speed_mm_s < 0.0f ? -v : v;
}

namespace motors {

void begin() {

  pinMode(PIN_MOTOR_STBY, OUTPUT);
  digitalWrite(PIN_MOTOR_STBY, LOW);
  g_enabled = false;

  for (Side* s : {&g_left, &g_right}) {
    pinMode(s->in1Pin, OUTPUT);
    pinMode(s->in2Pin, OUTPUT);
    digitalWrite(s->in1Pin, LOW);
    digitalWrite(s->in2Pin, LOW);

    ledcAttach(s->pwmPin, MOTOR_PWM_HZ, MOTOR_PWM_BITS);
    ledcWrite(s->pwmPin, 0);
    s->target_v = s->applied_v = 0.0f;
  }

  const esp_timer_create_args_t args = {
    .callback = &watchdogCallback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "motor_wdog",
    .skip_unhandled_events = true,
  };
  if (esp_timer_create(&args, &g_watchdogTimer) == ESP_OK) {
    esp_timer_start_periodic(g_watchdogTimer, CONTROL_WATCHDOG_POLL_US);
  }

  loadModel();
}

bool enable() {
  if (g_fault != Fault::NONE) return false;
  g_left.target_v = g_left.applied_v = 0.0f;
  g_right.target_v = g_right.applied_v = 0.0f;
  g_lastFeedMs = millis();
  g_watchEnabled = true;
  g_enabled = true;
  digitalWrite(PIN_MOTOR_STBY, HIGH);
  return true;
}

void disable() {
  digitalWrite(PIN_MOTOR_STBY, LOW);
  g_watchEnabled = false;
  g_enabled = false;
  for (Side* s : {&g_left, &g_right}) {
    s->target_v = s->applied_v = 0.0f;
    writeSide(*s, 0.0f, MOTOR_MIN_BATT_FOR_DUTY_V);
  }
}

bool enabled() { return g_enabled; }

void setVolts(float left_v, float right_v) {
  g_left.target_v  = clampf(left_v,  -MOTOR_VMAX_V, MOTOR_VMAX_V);
  g_right.target_v = clampf(right_v, -MOTOR_VMAX_V, MOTOR_VMAX_V);
}

void update(float dt_s) {
  if (!g_enabled || g_fault != Fault::NONE) {
    if (g_enabled) disable();
    return;
  }
  slew(g_left, dt_s);
  slew(g_right, dt_s);
  const float batt = fmaxf(battery::volts(), MOTOR_MIN_BATT_FOR_DUTY_V);
  writeSide(g_left, g_left.applied_v, batt);
  writeSide(g_right, g_right.applied_v, batt);
}

void feedWatchdog() { g_lastFeedMs = millis(); }

void watchdogGrace(uint32_t ms) { g_graceUntilMs = millis() + ms; }

float appliedLeftV()  { return g_left.applied_v; }
float appliedRightV() { return g_right.applied_v; }

bool idle() {
  return fabsf(g_left.applied_v) < MOTOR_ZERO_V && fabsf(g_right.applied_v) < MOTOR_ZERO_V &&
         fabsf(g_left.target_v) < MOTOR_ZERO_V && fabsf(g_right.target_v) < MOTOR_ZERO_V;
}

void fault(Fault f) {
  digitalWrite(PIN_MOTOR_STBY, LOW);
  g_watchEnabled = false;
  if (g_fault == Fault::NONE) g_fault = f;
}

Fault faultCode() { return g_fault; }

void clearFault() {
  disable();
  g_fault = Fault::NONE;
}

const VelocityModel& model() { return g_model; }
bool modelIsCalibrated() { return g_modelCalibrated; }

bool saveModel(const VelocityModel& m) {
  VelocityModel copy = m;
  copy.magic = MODEL_MAGIC;
  if (!copy.valid()) return false;
  Preferences prefs;
  if (!prefs.begin(NVS_CAL_NAMESPACE, false)) return false;
  const bool ok = prefs.putBytes("vmodel", &copy, sizeof(copy)) == sizeof(copy);
  prefs.end();
  if (ok) {
    g_model = copy;
    g_modelCalibrated = true;
  }
  return ok;
}

VelocityModel defaultModel() {
  VelocityModel m{};
  m.magic = MODEL_MAGIC;
  m.count = 2;
  m.volts[0] = MODEL_DEFAULT_DEADBAND_V;
  m.mm_s[0]  = 0.0f;
  m.volts[1] = MOTOR_VMAX_V;
  m.mm_s[1]  = MODEL_DEFAULT_V6_MM_S;
  m.tau_s    = MODEL_DEFAULT_TAU_S;
  return m;
}

}
