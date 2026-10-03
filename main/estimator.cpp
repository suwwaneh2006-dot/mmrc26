// =============================================================================
//  estimator.cpp - see estimator.h
// =============================================================================
#include "estimator.h"

#include "encoders.h"
#include "imu.h"
#include "motors.h"
#include "sonar.h"

namespace {

constexpr float RAD2DEG = 57.29578f;
constexpr int   HISTORY = 256;          // 256 ms of s history at 1 kHz
constexpr int   PARALLEL_BUF = 16;

struct HistoryEntry {
  uint32_t tUs;
  float    s;
};

enum Class : int8_t { CLS_NONE = -1, CLS_FAR = 0, CLS_NEAR = 1 };

// Per side-sensor tracking: edges, parallel-wall heading fix, lateral offset.
struct SideTrack {
  sonar::Id id;
  bool      left;
  float     sensorX;     // forward offset of the sensor from the axle
  float     expectMm;    // reading when centred in a cell
  float     nearMm;      // raw below this = wall
  float     farMm;       // raw above this = gap (between: ambiguous)
  uint32_t  lastSeq;
  // edge detection
  Class     cls;
  uint8_t   run;
  float     lastS;
  bool      pending;
  Class     pendingTo;
  float     pendingS;
  // parallel-wall heading fix
  float     bufS[PARALLEL_BUF];
  float     bufD[PARALLEL_BUF];
  int       bufN;
  // lateral offset
  float     lat;
  uint32_t  latUs;
  bool      latValid;
};

HistoryEntry g_hist[HISTORY];
int      g_histNext = 0;
bool     g_straight = false;
bool     g_useSonar = true;     // sonar fixes allowed on this straight
uint32_t g_straightStartUs = 0;
float    g_s = 0.0f;
float    g_v = 0.0f;
float    g_var = 0.0f;
float    g_encScale = 1.0f;     // learned wheel-diameter correction (encoders)
float    g_modelScale = 1.0f;   // learned volts->speed model correction (fallback)
float    g_lastEncMm = 0.0f;
float    g_lastLeftMm = 0.0f;
float    g_lastRightMm = 0.0f;
float    g_vModel = 0.0f;       // model speed, always tracked (fallback)
float    g_silentModelMm = 0.0f; // model travel while the encoders showed nothing
bool     g_encWasHealthy = true;
// The scale the position fixes currently teach (depends on the odometry in use).
float& activeScale() { return encoders::healthy() ? g_encScale : g_modelScale; }
float    g_cardinal = 0.0f;
float    g_face = -1.0f;
float    g_distSinceFix = 0.0f;
uint8_t  g_frontRejectRun = 0;
uint32_t g_frontSeq = 0;
estimator::Stats g_stats{};

SideTrack g_sides[2];

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Position of a point relative to the centre of the cell it is in (-p/2..p/2).
float cellRelative(float s) {
  float u = fmodf(s + CELL_PITCH_MM * 0.5f, CELL_PITCH_MM);
  if (u < 0.0f) u += CELL_PITCH_MM;
  return u - CELL_PITCH_MM * 0.5f;
}

// Correction applied to s since the last scale update. Over a distance d the
// corrections add up to (true - predicted) travel, so the robot moves
// (1 + drift / d) times as far as the model predicts.
float g_driftMm = 0.0f;

void kalmanUpdate(float innovation, float r, float scaleGain) {
  const float k = g_var / (g_var + r);
  g_s += k * innovation;
  g_var *= (1.0f - k);
  g_driftMm += k * innovation * (scaleGain / EST_SCALE_ADAPT_GAIN);   // edges count for less
  if (g_distSinceFix >= EST_SCALE_MIN_DIST_MM) {
    float& sc = activeScale();
    sc = clampf(sc * (1.0f + EST_SCALE_ADAPT_GAIN * g_driftMm / g_distSinceFix), EST_SCALE_MIN, EST_SCALE_MAX);
    g_driftMm = 0.0f;
    g_distSinceFix = 0.0f;
  }
}

bool gateOk(float innovation, float r) {
  return fabsf(innovation) < EST_GATE_MIN_MM + EST_GATE_SIGMAS * sqrtf(g_var + r);
}

float headingError() { return imu::headingDeg() - g_cardinal; }

// ---------------------------------------------------------------------------
//  b) front wall fixes
// ---------------------------------------------------------------------------
void frontReading(const sonar::Reading& r) {
  if (!g_straight || !g_useSonar) return;
  if (r.rawMm > FRONT_TRUST_MM || r.rawMm > FRONT_MAX_MM) return;
  if (fabsf(headingError()) > EST_FRONT_TRUST_DEG) return;
  if (static_cast<int32_t>(r.tUs - g_straightStartUs) < 0) return;   // older than this straight

  const float sThen = estimator::sAt(r.tUs);
  if (g_face < 0.0f) {
    // No map-predicted wall: whatever reflects straight ahead (wall or post
    // face) lies on a cell boundary. Snap to the nearest boundary face if the
    // estimate is close enough for the match to be unambiguous.
    if (!EST_LATTICE_FRONT_FIX) return;
    const float face = sThen + SONAR_F_X_MM + r.rawMm;
    const float k = roundf((face - HALF_CELL_INNER_MM) / CELL_PITCH_MM);
    const float innovation = (k * CELL_PITCH_MM + HALF_CELL_INNER_MM) - face;
    if (k < 0.0f || fabsf(innovation) > EDGE_GATE_MM || !gateOk(innovation, EST_LATTICE_R_MM2)) {
      ++g_stats.frontRejects;
      return;
    }
    kalmanUpdate(innovation, EST_LATTICE_R_MM2, EST_SCALE_ADAPT_GAIN);
    ++g_stats.frontFixes;
    return;
  }
  const float sMeas = g_face - SONAR_F_X_MM - r.rawMm;
  const float innovation = sMeas - sThen;
  if (!gateOk(innovation, EST_FRONT_R_MM2)) {
    ++g_stats.frontRejects;
    if (++g_frontRejectRun >= EST_REJECT_RUN) {
      g_var += EST_REJECT_INFLATE_MM2;   // let the gate widen and re-lock
      g_frontRejectRun = 0;
    }
    return;
  }
  g_frontRejectRun = 0;
  kalmanUpdate(innovation, EST_FRONT_R_MM2, EST_SCALE_ADAPT_GAIN);
  ++g_stats.frontFixes;
}

// ---------------------------------------------------------------------------
//  c) post edges, d) parallel-wall heading, lateral offset
// ---------------------------------------------------------------------------
void commitEdge(SideTrack& t, bool wallToGap, float sMid) {
  // Axle position at which this sensor sees the switch at a post on the
  // boundary b = (k + 1/2) * pitch.
  const float off = wallToGap ? (POST_HALF_MM + EDGE_WALL_TO_GAP_MM) : -(POST_HALF_MM + EDGE_GAP_TO_WALL_MM);
  const float k = roundf((sMid + t.sensorX - off) / CELL_PITCH_MM - 0.5f);
  const float predicted = (k + 0.5f) * CELL_PITCH_MM + off - t.sensorX;
  const float innovation = predicted - sMid;
  if (fabsf(innovation) > EDGE_GATE_MM || !gateOk(innovation, EST_EDGE_R_MM2)) {
    ++g_stats.edgeRejects;
    return;
  }
  kalmanUpdate(innovation, EST_EDGE_R_MM2, EST_SCALE_ADAPT_GAIN_EDGE);
  ++g_stats.edgeFixes;
}

void trackEdges(SideTrack& t, Class cls, float sR) {
  if (t.cls == CLS_NONE) {
    t.cls = cls;
    t.run = 1;
    t.lastS = sR;
    return;
  }
  if (cls == t.cls) {
    if (t.pending && t.pendingTo == cls) {   // second reading confirms the edge
      commitEdge(t, cls == CLS_FAR, t.pendingS);
      t.pending = false;
    }
    if (t.run < 255) ++t.run;
    t.lastS = sR;
    return;
  }
  // Class changed: candidate edge between the last old and first new reading.
  t.pending = t.run >= 2 && fabsf(sR - t.lastS) <= EDGE_MAX_SPAN_MM;
  t.pendingTo = cls;
  t.pendingS = 0.5f * (t.lastS + sR);
  t.cls = cls;
  t.run = 1;
  t.lastS = sR;
}

void trackParallel(SideTrack& t, float sR, float raw) {
  if (t.bufN == PARALLEL_BUF) {   // drop the oldest sample
    for (int i = 1; i < PARALLEL_BUF; ++i) {
      t.bufS[i - 1] = t.bufS[i];
      t.bufD[i - 1] = t.bufD[i];
    }
    --t.bufN;
  }
  t.bufS[t.bufN] = sR;
  t.bufD[t.bufN] = raw;
  ++t.bufN;
  if (t.bufN < PARALLEL_MIN_SAMPLES || t.bufS[t.bufN - 1] - t.bufS[0] < PARALLEL_SPAN_MM) return;

  // Least-squares slope of distance vs travel = tan(angle to the wall).
  float ms = 0.0f, md = 0.0f;
  for (int i = 0; i < t.bufN; ++i) {
    ms += t.bufS[i];
    md += t.bufD[i];
  }
  ms /= t.bufN;
  md /= t.bufN;
  float sxy = 0.0f, sxx = 0.0f;
  for (int i = 0; i < t.bufN; ++i) {
    const float ds = t.bufS[i] - ms;
    sxy += ds * (t.bufD[i] - md);
    sxx += ds * ds;
  }
  t.bufN = 0;
  if (sxx <= 1.0f) return;
  const float slope = sxy / sxx;
  if (fabsf(slope) > PARALLEL_MAX_SLOPE) return;
  // Left wall getting further away = robot pointing right of the wall line.
  const float angle = atanf(slope) * RAD2DEG;
  const float heading = t.left ? g_cardinal - angle : g_cardinal + angle;
  if (fabsf(heading - imu::headingDeg()) > PARALLEL_MAX_FIX_DEG) return;   // implausible
  imu::setHeading(heading);
  ++g_stats.headingFixes;
}

void sideReading(SideTrack& t, const sonar::Reading& r) {
  const bool trusted = g_straight && g_useSonar && sonar::sideTrusted(headingError()) &&
                       fabsf(imu::rateDps()) < SIDE_MAX_RATE_DPS &&
                       static_cast<int32_t>(r.tUs - g_straightStartUs) >= 0;
  if (!trusted) {
    t.cls = CLS_NONE;
    t.pending = false;
    t.bufN = 0;
    return;
  }
  const float raw = r.rawMm;
  const Class cls = raw <= t.nearMm ? CLS_NEAR : (raw >= t.farMm ? CLS_FAR : CLS_NONE);
  const float sR = estimator::sAt(r.tUs);
  if (cls != CLS_NONE) trackEdges(t, cls, sR);

  if (cls == CLS_NEAR) {
    trackParallel(t, sR, raw);
    // Lateral offset only from readings that face a wall segment.
    if (fabsf(cellRelative(sR + t.sensorX)) < CELL_PITCH_MM * 0.5f - SIDE_POST_KEEPOUT_MM) {
      t.lat = t.left ? t.expectMm - raw : raw - t.expectMm;
      t.latUs = r.tUs;
      t.latValid = true;
    }
  } else {
    t.bufN = 0;
  }
}

void initSide(SideTrack& t, sonar::Id id, bool left, float x, float expect) {
  t = SideTrack{};
  t.id = id;
  t.left = left;
  t.sensorX = x;
  t.expectMm = expect;
  t.nearMm = expect + SIDE_WALL_ON_MARGIN_MM;
  t.farMm = expect + SIDE_WALL_OFF_MARGIN_MM;
  t.cls = CLS_NONE;
  t.lastSeq = sonar::read(id).seq;
}

void resetSideTracking() {
  for (SideTrack& t : g_sides) {
    t.cls = CLS_NONE;
    t.pending = false;
    t.bufN = 0;
    t.latValid = false;
  }
}

}  // namespace

namespace estimator {

void begin() {
  initSide(g_sides[0], sonar::LEFT, true, SONAR_L_X_MM, SIDE_L_EXPECT_MM);
  initSide(g_sides[1], sonar::RIGHT, false, SONAR_R_X_MM, SIDE_R_EXPECT_MM);
  g_frontSeq = sonar::read(sonar::FRONT).seq;
  g_straight = false;
  g_s = g_v = 0.0f;
  g_var = START_SIGMA_MM * START_SIGMA_MM;
  g_encScale = 1.0f;
  g_modelScale = 1.0f;
  g_lastEncMm = 0.5f * (encoders::leftMm() + encoders::rightMm());
  g_lastLeftMm = encoders::leftMm();
  g_lastRightMm = encoders::rightMm();
  g_face = -1.0f;
  g_stats = Stats{};
  for (HistoryEntry& h : g_hist) h = HistoryEntry{micros(), 0.0f};
}

void beginStraight(float sMm, float sigmaMm, float cardinalDeg, bool useSonar) {
  g_useSonar = useSonar;
  g_s = sMm;
  g_var = sigmaMm * sigmaMm;
  g_cardinal = cardinalDeg;
  g_face = -1.0f;
  g_frontRejectRun = 0;
  g_straightStartUs = micros();
  for (HistoryEntry& h : g_hist) h = HistoryEntry{g_straightStartUs, g_s};
  resetSideTracking();
  g_straight = true;
}

void endStraight() {
  g_straight = false;
  resetSideTracking();
}

bool straightActive() { return g_straight; }
void setFrontWallFace(float faceMm) { g_face = faceMm; }
float frontWallFace() { return g_face; }

void tick(float dt_s) {
  // a) predict: wheel encoders when they work, else the volts->speed model.
  const float encMm = 0.5f * (encoders::leftMm() + encoders::rightMm());
  const float encDelta = encMm - g_lastEncMm;
  g_lastEncMm = encMm;
  const VelocityModel& m = motors::model();
  const float vAvg = 0.5f * (motors::appliedLeftV() + motors::appliedRightV());
  g_vModel += (g_modelScale * m.speedFor(vAvg) - g_vModel) * fminf(dt_s / m.tau_s, 1.0f);
  float ds;
  float q;
  const float dl = encoders::leftMm() - g_lastLeftMm;
  const float dr = encoders::rightMm() - g_lastRightMm;
  g_lastLeftMm = encoders::leftMm();
  g_lastRightMm = encoders::rightMm();
  const bool lu = encoders::leftUsable(), ru = encoders::rightUsable();
  const bool encOk = encoders::healthy();
  if (encOk && !lu && !ru) {
    // Both wheels momentarily silent while driven: model until they count.
    g_v = g_vModel;
    ds = g_vModel * dt_s;
    q = EST_Q_MM2_PER_MM;
    g_silentModelMm = 0.0f;
  } else if (encOk) {
    // Both wheels, or the one that still counts.
    const float d = lu && ru ? 0.5f * (dl + dr) : (lu ? dl : dr);
    const float sp = lu && ru ? encoders::speed() : (lu ? encoders::leftSpeed() : encoders::rightSpeed());
    g_v = g_encScale * sp;
    ds = g_encScale * d;
    q = EST_Q_ENC_MM2_PER_MM;
    // Travel the model predicts while the encoders report nothing: needed if
    // they are then declared dead (the robot moved meanwhile).
    g_silentModelMm = encDelta == 0.0f ? g_silentModelMm + g_vModel * dt_s : 0.0f;
  } else {
    g_v = g_vModel;
    ds = g_v * dt_s;
    q = EST_Q_MM2_PER_MM;
    if (g_encWasHealthy) {   // switched to the fallback just now
      ds += g_silentModelMm;
      const float sd = ENC_FALLBACK_SIGMA_FRAC * g_silentModelMm;
      g_var += sd * sd;
      g_silentModelMm = 0.0f;
    }
  }
  g_encWasHealthy = encOk;
  if (g_straight) {
    g_s += ds;
    g_var += q * fabsf(ds);
    g_distSinceFix += fabsf(ds);
  }
  g_hist[g_histNext] = HistoryEntry{micros(), g_s};
  g_histNext = (g_histNext + 1) % HISTORY;

  // b) c) d) new sonar readings
  const sonar::Reading f = sonar::read(sonar::FRONT);
  if (f.seq != g_frontSeq) {
    g_frontSeq = f.seq;
    frontReading(f);
  }
  for (SideTrack& t : g_sides) {
    const sonar::Reading r = sonar::read(t.id);
    if (r.seq != t.lastSeq) {
      t.lastSeq = r.seq;
      sideReading(t, r);
    }
  }
}

float s() { return g_s; }
float v() { return g_v; }
float sigma() { return sqrtf(g_var); }
float scale() { return encoders::healthy() ? g_encScale : g_modelScale; }
float modelScale() { return g_modelScale; }

Confidence confidence() {
  const float sg = sigma();
  if (sg < CONF_GOOD_SIGMA_MM) return Confidence::GOOD;
  if (sg < CONF_LOST_SIGMA_MM) return Confidence::REDUCED;
  return Confidence::LOST;
}

void inflate(float sigmaMm) { g_var += sigmaMm * sigmaMm; }

bool lateralMm(float& out) {
  const uint32_t now = micros();
  float sum = 0.0f;
  int n = 0;
  for (const SideTrack& t : g_sides) {
    if (t.latValid && now - t.latUs < SIDE_SAMPLE_MAX_AGE_MS * 1000u) {
      sum += t.lat;
      ++n;
    }
  }
  if (n == 0) return false;
  out = sum / n;
  return true;
}

float sAt(uint32_t tUs) {
  // Newest entry not later than tUs.
  for (int i = 1; i <= HISTORY; ++i) {
    const HistoryEntry& h = g_hist[(g_histNext - i + HISTORY) % HISTORY];
    if (static_cast<int32_t>(h.tUs - tUs) <= 0) return h.s;
  }
  return g_hist[g_histNext].s;   // older than the history: oldest value
}

Stats stats() { return g_stats; }

const char* confidenceName(Confidence c) {
  switch (c) {
    case Confidence::GOOD: return "good";
    case Confidence::REDUCED:  return "low";
    case Confidence::LOST: return "lost";
  }
  return "?";
}

}  // namespace estimator
