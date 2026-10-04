#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "Arduino.h"
#include "config.h"
#include "maze.h"
#include "motion.h"
#include "estimator.h"
#include "sonar.h"
#include "world.h"

void setup();
void loop();

namespace {

TrueMaze g_truth;
bool     g_mirror = false;
bool     g_mode8 = false;

struct Act {
  uint64_t at;
  int      kind;
  bool     on;
};
std::vector<Act> g_acts;
void act(uint64_t at, int kind, bool on) { g_acts.push_back(Act{at, kind, on}); }
void press(uint64_t at, uint32_t holdMs) {
  act(at, 0, true);
  act(at + holdMs * 1000u, 0, false);
}
void handWave(uint64_t at) {
  act(at, 1, true);
  act(at + 800000u, 1, false);
}

int runsOk = 0, returnsOk = 0, aborts = 0;
double bestS = 1e9;
std::vector<std::string> abortReasons;
bool finished = false;
bool sawMenu = false, sawMirror = false;

double numberAfter(const char* line, const char* key) {
  const char* p = strstr(line, key);
  return p ? atof(p + strlen(key)) : -1.0;
}

void onLine(const char* line) {
  const uint64_t now = world::nowUs();
  if (g_mode8) {
    if (!sawMenu && strstr(line, "ready: click 1-8")) {
      sawMenu = true;
      for (int i = 0; i < 8; ++i) press(now + 300000u + i * 300000u, 100);
    } else if (strstr(line, "mode 8: short press = start")) {
      press(now + 500000u, 100);
    } else if (strstr(line, "saved: forward") || strstr(line, "calibration not saved")) {
      const world::Stats st = world::stats();
      std::printf("MODE8 %s | crashes %d\n", strstr(line, "saved: forward") ? "SAVED" : "FAILED", st.crashes);
      std::fflush(stdout);
      std::exit(strstr(line, "saved: forward") && st.crashes == 0 ? 0 : 1);
    }
    return;
  }
  if (!sawMenu && strstr(line, "ready: click 1-8")) {
    sawMenu = true;
    for (int i = 0; i < 7; ++i) press(now + 300000u + i * 300000u, 100);
  } else if (!sawMirror && strstr(line, "mirror: short press toggles")) {
    sawMirror = true;
    uint64_t t = now + 800000u;
    if (g_mirror) {
      press(t, 100);
      t += 800000u;
    }
    press(t, BUTTON_LONG_MS + 300);
    handWave(t + 2500000u);
  } else if (strstr(line, "goal reached in")) {
    ++runsOk;
    const double ms = numberAfter(line, "goal reached in");
    if (ms > 0 && ms / 1000.0 < bestS) bestS = ms / 1000.0;
  } else if (strstr(line, "start reached in")) {
    ++returnsOk;
    const double ms = numberAfter(line, "start reached in");
    if (ms > 0 && ms / 1000.0 < bestS) bestS = ms / 1000.0;
  } else if (strstr(line, "ABORT:")) {
    ++aborts;
    abortReasons.push_back(line);
    act(now + 2000000u, 2, true);
    press(now + 2500000u, 100);
    handWave(now + 4000000u);
  } else if (strstr(line, "match time used up")) {
    finished = true;
  }
}

int checkStoredMap(int& known) {
  known = 0;
  std::string blob;
  if (!world::nvsGet(NVS_MAP_NAMESPACE, "map", blob) || blob.size() != 1 + mm::Maze::SERIAL_BYTES) return -1;
  static mm::Maze m;
  if (!m.deserialize(reinterpret_cast<const uint8_t*>(blob.data()) + 1, mm::Maze::SERIAL_BYTES)) return -1;
  int wrong = 0;
  for (int x = 0; x < g_truth.width; ++x) {
    for (int y = 0; y < g_truth.height; ++y) {
      const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      for (int d = 0; d < 4; ++d) {
        if (!m.isKnown(c, static_cast<mm::Dir>(d))) continue;
        ++known;
        if (m.hasWall(c, static_cast<mm::Dir>(d)) != g_truth.wall[x][y][d]) {
          ++wrong;
          std::printf("WRONG WALL cell (%d,%d) side %c: map says %s\n", x, y, mm::dirChar(static_cast<mm::Dir>(d)),
                      m.hasWall(c, static_cast<mm::Dir>(d)) ? "wall" : "open");
        }
      }
    }
  }
  return wrong;
}

void finish(int code, const char* why) {
  const world::Stats st = world::stats();
  int known = 0;
  const int wrong = checkStoredMap(known);
  const double score = bestS < 1e8 ? (runsOk + 1.5 * returnsOk) / bestS * 1000.0 : 0.0;
  std::printf("\n================ SIMULATION RESULT (%s) ================\n", why);
  std::printf("sim time        %.1f s, distance driven %.1f m\n", world::nowUs() * 1e-6, st.distanceMm / 1000.0);
  std::printf("runs to goal    %d\nreturns         %d\naborts          %d\n", runsOk, returnsOk, aborts);
  for (const std::string& r : abortReasons) std::printf("   %s\n", r.c_str());
  std::printf("CRASHES         %d\n", st.crashes);
  std::printf("pivot crashes   %d\n", st.pivotCrashes);
  std::printf("min clearance   %.1f mm (footprint to wall while moving)\n", st.minWallClearanceMm);
  if (st.stops > 0) {
    std::printf("stop accuracy   %d stops: along-track mean %.1f / max %.1f mm, lateral mean %.1f / max %.1f mm,"
                " heading max %.1f deg\n", st.stops, st.sumAbsAlongMm / st.stops, st.maxAbsAlongMm,
                st.sumAbsLateralMm / st.stops, st.maxAbsLateralMm, st.maxAbsHeadingDeg);
  }
  std::printf("stored map      %d known wall sides, %d WRONG\n", known, wrong);
  std::printf("best time       %.2f s   score = (runs + 1.5 returns) / best * 1000 = %.0f\n", bestS, score);
  const bool ok = code == 0 && st.crashes == 0 && wrong == 0 && runsOk > 0 && returnsOk > 0;
  std::printf("VERDICT         %s\n", ok ? "PASS" : "FAIL");
  std::fflush(stdout);
  std::exit(ok ? 0 : 1);
}

uint64_t g_limitUs = 0;
double g_traceFrom = -1.0, g_traceTo = -1.0;
void onTick(uint64_t now) {
  const double t = now * 1e-6;
  if (t >= g_traceFrom && t <= g_traceTo && (now / 1000u) % 20u == 0u) {
    float x, y, th, vl, vr;
    world::truePose(x, y, th, vl, vr);
    const motion::Telemetry m = motion::telemetry();
    const estimator::Stats es = estimator::stats();
    std::printf("TRACE %.3f true x=%.0f y=%.0f th=%.1f vl=%.0f vr=%.0f | fw s=%.0f/%.0f v=%.0f sig=%.1f hdg=%.1f tgt=%.1f V=%.2f/%.2f face=%.0f edges %lu/%lu front %lu/%lu L=%.0f R=%.0f F=%.0f/%.0f\n",
                t, x, y, th, vl, vr, m.sEstMm, m.sRefMm, m.vEstMmS, m.sigmaMm, m.headingDeg, m.targetDeg, m.leftV,
                m.rightV, estimator::frontWallFace(), static_cast<unsigned long>(es.edgeFixes), static_cast<unsigned long>(es.edgeRejects), static_cast<unsigned long>(es.frontFixes), static_cast<unsigned long>(es.frontRejects), sonar::read(sonar::LEFT).rawMm, sonar::read(sonar::RIGHT).rawMm, sonar::read(sonar::FRONT).rawMm, sonar::read(sonar::FRONT).mm);
  }
  for (size_t i = 0; i < g_acts.size();) {
    if (g_acts[i].at <= now) {
      const Act a = g_acts[i];
      g_acts.erase(g_acts.begin() + static_cast<long>(i));
      if (a.kind == 0) world::setButton(a.on);
      else if (a.kind == 1) world::setHand(a.on);
      else world::teleportToStart();
    } else {
      ++i;
    }
  }
  if (finished) finish(0, "match time used up");
  if (now > g_limitUs) finish(2, "simulation time limit");
}

}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("usage: robot_sim <maze.txt> [--mirror] [--seed N] [--quiet] [--model-error F]\n");
    return 2;
  }
  world::Params p;
  bool quiet = false;
  for (int i = 2; i < argc; ++i) {
    if (!strcmp(argv[i], "--mirror")) g_mirror = true;
    else if (!strcmp(argv[i], "--quiet")) quiet = true;
    else if (!strcmp(argv[i], "--no-enc")) p.encoders = false;
    else if (!strcmp(argv[i], "--mode8")) g_mode8 = true;
    else if (!strcmp(argv[i], "--enc-fail") && i + 1 < argc) p.encoderFailAtS = static_cast<float>(atof(argv[++i]));
    else if (!strcmp(argv[i], "--harsh")) {
      p.sonarNoiseMm = 4.0f;
      p.gyroBiasDps = 2.0f;
      p.gyroNoiseDps = 0.2f;
      p.rightWheelGain = 1.08f;
    }
    else if (!strcmp(argv[i], "--seed") && i + 1 < argc) p.seed = static_cast<unsigned>(atoi(argv[++i]));
    else if (!strcmp(argv[i], "--trace") && i + 2 < argc) {
      g_traceFrom = atof(argv[++i]);
      g_traceTo = atof(argv[++i]);
    } else if (!strcmp(argv[i], "--model-error") && i + 1 < argc) p.motorMmSPerV *= static_cast<float>(atof(argv[++i]));
  }
  std::string err;
  if (!g_truth.load(argv[1], err)) {
    std::printf("cannot load %s: %s\n", argv[1], err.c_str());
    return 2;
  }
  if (g_truth.width != MAZE_SIZE_CELLS || g_truth.height != MAZE_SIZE_CELLS) {
    std::printf("the firmware is built for %dx%d mazes\n", MAZE_SIZE_CELLS, MAZE_SIZE_CELLS);
    return 2;
  }
  p.mirrored = g_mirror;
  world::init(g_truth, p);
  world::setEcho(!quiet);
  world::setLineHook(&onLine);
  world::setTickHook(&onTick);
  g_limitUs = static_cast<uint64_t>(MATCH_DURATION_MS + 120000u) * 1000u;
  setup();
  for (;;) loop();
}
