// =============================================================================
//  sonar.cpp - see sonar.h
// =============================================================================
#include "sonar.h"

#include <soc/gpio_struct.h>

namespace {

using sonar::Id;

// Echo state, advanced by the ISR (ARMED -> HIGH -> DONE) and reset by the
// main loop (-> ARMED) only while ECHO is low, so the two never race.
enum IsrState : uint8_t { ISR_IDLE = 0, ISR_ARMED, ISR_HIGH, ISR_DONE };

struct Channel {
  // Configuration
  uint8_t  trigPin;
  uint8_t  echoPin;
  float    maxMm;
  uint32_t maxEchoUs;
  float    wallOnMm;
  float    wallOffMm;

  // Shared with the ISR
  volatile uint8_t  isrState;
  volatile uint32_t riseUs;
  volatile uint32_t fallUs;

  // Main-loop state
  uint32_t trigUs;
  bool     highSeen;
  uint32_t highSinceUs;
  float    hist[3];
  uint8_t  histCount;
  uint8_t  histNext;
  bool     haveAccepted;
  float    lastAcceptedMm;
  uint32_t lastAcceptedUs;
  uint8_t  rejectRun;
  float    lastRejectedMm;

  sonar::Reading out;
  sonar::Stats   st;
};

Channel g_ch[sonar::COUNT];

// F, L, F, R: front gets every second slot.
constexpr Id ORDER[] = {sonar::FRONT, sonar::LEFT, sonar::FRONT, sonar::RIGHT};
constexpr uint8_t ORDER_LEN = sizeof(ORDER) / sizeof(ORDER[0]);

bool     g_frontOnly = false;  // calibration: ping only the front sensor
uint8_t  g_orderIdx  = 0;
int8_t   g_active    = -1;     // sensor currently in flight, -1 = none
uint32_t g_lastEndUs = 0;      // end of the previous measurement
float    g_speedHint = 0.0f;   // mm/s, from the motion controller
float    g_rateHint  = 0.0f;   // deg/s

// ECHO edge interrupt. Reads the pin level instead of trusting the edge
// direction: this makes it immune to the ESP32 GPIO36/39 errata (an ~80 ns
// low glitch when ADC1 powers up), because by the time the ISR runs the pin
// is back to its true level and a spurious "fall" is ignored.
void IRAM_ATTR echoIsr(void* arg) {
  Channel* c = static_cast<Channel*>(arg);
  const uint32_t now = micros();
  const bool high = ((GPIO.in1.val >> (c->echoPin - 32)) & 1u) != 0;   // GPIO 32-39 input register
  if (high) {
    if (c->isrState == ISR_ARMED) {
      c->riseUs = now;
      c->isrState = ISR_HIGH;
    }
  } else if (c->isrState == ISR_HIGH) {
    c->fallUs = now;
    c->isrState = ISR_DONE;
  }
}

bool echoHigh(const Channel& c) { return digitalRead(c.echoPin) == HIGH; }

void initChannel(Channel& c, uint8_t trig, uint8_t echo, float maxMm, float onMm, float offMm) {
  c.trigPin = trig;
  c.echoPin = echo;
  c.maxMm = maxMm;
  c.maxEchoUs = static_cast<uint32_t>(maxMm / SONAR_MM_PER_ECHO_US);
  c.wallOnMm = onMm;
  c.wallOffMm = offMm;
  c.isrState = ISR_IDLE;
  c.riseUs = 0;
  c.fallUs = 0;
  c.trigUs = 0;
  c.highSeen = false;
  c.highSinceUs = 0;
  c.histCount = 0;
  c.histNext = 0;
  c.haveAccepted = false;
  c.lastAcceptedMm = 0.0f;
  c.lastAcceptedUs = 0;
  c.rejectRun = 0;
  c.lastRejectedMm = 0.0f;
  c.out = sonar::Reading{};
  c.st = sonar::Stats{};
}

float median3(const float* h, uint8_t n, uint8_t newest) {
  if (n < 3) return h[newest];
  const float a = h[0], b = h[1], c = h[2];
  return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

// A complete raw measurement: gate, median, wall flag. raw > maxMm = NO_WALL.
void publish(Channel& c, float raw, uint32_t tUs) {
  c.out.rawMm = raw;
  c.out.tUs = tUs;
  c.out.seq++;
  c.out.valid = true;

  const bool far = raw > c.maxMm;
  if (!far && c.haveAccepted && fabsf(g_rateHint) < SONAR_GATE_MAX_RATE_DPS) {
    const float dt = (tUs - c.lastAcceptedUs) * 1e-6f;
    const float allowed = fabsf(g_speedHint) * dt + SONAR_GATE_MARGIN_MM;
    if (fabsf(raw - c.lastAcceptedMm) > allowed) {
      // Consecutive rejected readings that agree with each other mean the
      // world really changed: after SONAR_GATE_RELOCK_COUNT of them, accept.
      const bool consistent = c.rejectRun > 0 && fabsf(raw - c.lastRejectedMm) <= allowed;
      c.rejectRun = consistent ? c.rejectRun + 1 : 1;
      c.lastRejectedMm = raw;
      if (c.rejectRun < SONAR_GATE_RELOCK_COUNT) {
        c.st.gateRejects++;
        return;
      }
    }
  }
  c.rejectRun = 0;
  if (far) {
    c.haveAccepted = false;   // the next wall seen may legitimately be anywhere
  } else {
    c.haveAccepted = true;
    c.lastAcceptedMm = raw;
    c.lastAcceptedUs = tUs;
  }

  const uint8_t newest = c.histNext;
  c.hist[newest] = far ? c.maxMm + 1.0f : raw;
  c.histNext = (c.histNext + 1) % 3;
  if (c.histCount < 3) c.histCount++;

  const float m = median3(c.hist, c.histCount, newest);
  c.out.inRange = m <= c.maxMm;
  c.out.mm = c.out.inRange ? m : c.maxMm;

  if (!c.out.wall && c.out.inRange && c.out.mm < c.wallOnMm) {
    c.out.wall = true;
  } else if (c.out.wall && (!c.out.inRange || c.out.mm > c.wallOffMm)) {
    c.out.wall = false;
  }
}

void finishActive(uint32_t nowUs) {
  g_active = -1;
  g_lastEndUs = nowUs;
}

// Check the measurement in flight.
void serviceActive(uint32_t now) {
  Channel& c = g_ch[g_active];
  switch (c.isrState) {
    case ISR_ARMED:
      if (now - c.trigUs > SONAR_RISE_TIMEOUT_US) {   // sensor never answered
        c.isrState = ISR_IDLE;
        c.st.misses++;
        if (c.st.missRun < 255) c.st.missRun++;
        finishActive(now);
      }
      break;
    case ISR_HIGH:
      // Still high beyond the maximum range: NO_WALL now, without waiting for
      // the (up to 200 ms) timeout fall. The sensor stays BUSY until it falls.
      if (now - c.riseUs > c.maxEchoUs) {
        c.st.echoes++;
        c.st.missRun = 0;
        publish(c, c.maxMm + 1.0f, now);
        finishActive(now);
      }
      break;
    case ISR_DONE: {
      const uint32_t dur = c.fallUs - c.riseUs;
      float mm = dur * SONAR_MM_PER_ECHO_US;
      if (mm < SONAR_MIN_MM) mm = SONAR_MIN_MM;
      c.st.echoes++;
      c.st.missRun = 0;
      publish(c, mm, c.fallUs);
      c.isrState = ISR_IDLE;
      finishActive(now);
      break;
    }
    default:
      finishActive(now);
      break;
  }
}

void trigger(Channel& c) {
  c.isrState = ISR_ARMED;
  digitalWrite(c.trigPin, HIGH);
  delayMicroseconds(SONAR_TRIG_PULSE_US);   // the only busy-wait: 10 us
  digitalWrite(c.trigPin, LOW);
  c.trigUs = micros();
  c.st.pings++;
}

// Fire the next sensor in F, L, F, R order, skipping BUSY ones.
void fireNext() {
  for (uint8_t tries = 0; tries < ORDER_LEN; ++tries) {
    const Id id = g_frontOnly ? sonar::FRONT : ORDER[g_orderIdx];
    g_orderIdx = (g_orderIdx + 1) % ORDER_LEN;
    Channel& c = g_ch[id];
    if (echoHigh(c)) continue;   // still holding ECHO from an earlier ping
    trigger(c);
    g_active = static_cast<int8_t>(id);
    return;
  }
}

void monitorStuck(Channel& c, uint32_t now) {
  if (echoHigh(c)) {
    if (!c.highSeen) {
      c.highSeen = true;
      c.highSinceUs = now;
    } else if (now - c.highSinceUs > SONAR_STUCK_MS * 1000u) {
      c.st.stuck = true;
    }
  } else {
    c.highSeen = false;
    c.st.stuck = false;
  }
  c.st.faulty = c.st.stuck || c.st.missRun >= SONAR_MISS_FAULT_COUNT;
}

}  // namespace

namespace sonar {

void begin() {
  initChannel(g_ch[FRONT], PIN_SONAR_F_TRIG, PIN_SONAR_F_ECHO, FRONT_MAX_MM,
              FRONT_WALL_ON_MM, FRONT_WALL_OFF_MM);
  initChannel(g_ch[LEFT], PIN_SONAR_L_TRIG, PIN_SONAR_L_ECHO, SIDE_MAX_MM,
              SIDE_L_EXPECT_MM + SIDE_WALL_ON_MARGIN_MM, SIDE_L_EXPECT_MM + SIDE_WALL_OFF_MARGIN_MM);
  initChannel(g_ch[RIGHT], PIN_SONAR_R_TRIG, PIN_SONAR_R_ECHO, SIDE_MAX_MM,
              SIDE_R_EXPECT_MM + SIDE_WALL_ON_MARGIN_MM, SIDE_R_EXPECT_MM + SIDE_WALL_OFF_MARGIN_MM);
  for (Channel& c : g_ch) {
    pinMode(c.trigPin, OUTPUT);
    digitalWrite(c.trigPin, LOW);
    pinMode(c.echoPin, INPUT);   // GPIO 34-39 have no pulls; the divider pulls down
    attachInterruptArg(c.echoPin, echoIsr, &c, CHANGE);
  }
  g_active = -1;
  g_orderIdx = 0;
  g_lastEndUs = micros();
}

void update() {
  const uint32_t now = micros();
  for (Channel& c : g_ch) monitorStuck(c, now);
  if (g_active >= 0) serviceActive(now);
  if (g_active < 0 && now - g_lastEndUs >= SONAR_GAP_MS * 1000u) fireNext();
}

Reading read(Id id) { return g_ch[id].out; }
Stats stats(Id id) { return g_ch[id].st; }
bool faulty(Id id) { return g_ch[id].st.faulty; }

uint32_t ageMs(Id id) {
  const Reading& r = g_ch[id].out;
  if (!r.valid) return UINT32_MAX;
  return (micros() - r.tUs) / 1000u;
}

void setMotionHint(float speed_mm_s, float rate_dps) {
  g_speedHint = speed_mm_s;
  g_rateHint = rate_dps;
}

void setFrontOnly(bool frontOnly) { g_frontOnly = frontOnly; }

void discardInFlight() {
  if (g_active < 0) return;
  // The ISR may have been delayed (flash write): this echo's timing is not
  // trustworthy. Drop it; the sensor stays BUSY until its ECHO falls.
  g_ch[g_active].isrState = ISR_IDLE;
  finishActive(micros());
}

uint8_t selfTest() {
  for (Channel& c : g_ch) c.st = Stats{};
  const uint32_t t0 = millis();
  while (millis() - t0 < SONAR_SELFTEST_MS) update();
  uint8_t mask = 0;
  for (uint8_t i = 0; i < COUNT; ++i) {
    const Stats& s = g_ch[i].st;
    if (s.echoes >= SONAR_SELFTEST_MIN_PINGS && !s.stuck) mask |= static_cast<uint8_t>(1u << i);
  }
  return mask;
}

const char* name(Id id) {
  switch (id) {
    case FRONT: return "front";
    case LEFT:  return "left";
    case RIGHT: return "right";
    default:    return "?";
  }
}

}  // namespace sonar
