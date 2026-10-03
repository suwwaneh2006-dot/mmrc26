#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "config.h"
#include "fake_api.h"
#include "mouse.h"

namespace {

int wallsAtPost(const TrueMaze& m, int i, int j) {

  return m.wall[i - 1][j - 1][mm::EAST] + m.wall[i - 1][j][mm::EAST] + m.wall[i - 1][j - 1][mm::NORTH] +
         m.wall[i][j - 1][mm::NORTH];
}

TrueMaze generateIsland(int n, unsigned seed) {
  std::mt19937 rng(seed);
  TrueMaze m;
  m.width = m.height = n;
  for (int x = 0; x < n; ++x)
    for (int y = 0; y < n; ++y)
      for (int d = 0; d < 4; ++d) m.wall[x][y][d] = true;

  const int g0 = (n - 1) / 2, g1 = n / 2;
  auto isGoal = [&](int x, int y) { return x >= g0 && x <= g1 && y >= g0 && y <= g1; };
  for (int x = g0; x <= g1; ++x) {
    for (int y = g0; y <= g1; ++y) {
      const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      m.goal.add(c);
      if (x < g1) m.setWall(x, y, mm::EAST, false);
      if (y < g1) m.setWall(x, y, mm::NORTH, false);
    }
  }

  std::vector<int> stack;
  std::vector<bool> seen(static_cast<size_t>(n * n), false);
  stack.push_back(0);
  seen[0] = true;
  while (!stack.empty()) {
    const int cur = stack.back();
    const int x = cur / n, y = cur % n;
    int options[4], count = 0;
    for (int d = 0; d < 4; ++d) {
      const int nx = x + mm::dx(static_cast<mm::Dir>(d)), ny = y + mm::dy(static_cast<mm::Dir>(d));
      if (nx < 0 || ny < 0 || nx >= n || ny >= n || isGoal(nx, ny) || seen[static_cast<size_t>(nx * n + ny)]) continue;
      if (x == 0 && y == 0 && d == mm::EAST) continue;
      options[count++] = d;
    }
    if (count == 0) {
      stack.pop_back();
      continue;
    }
    const int d = options[rng() % static_cast<unsigned>(count)];
    m.setWall(x, y, static_cast<mm::Dir>(d), false);
    const int nx = x + mm::dx(static_cast<mm::Dir>(d)), ny = y + mm::dy(static_cast<mm::Dir>(d));
    seen[static_cast<size_t>(nx * n + ny)] = true;
    stack.push_back(nx * n + ny);
  }

  struct Opening { int x, y; mm::Dir d; };
  std::vector<Opening> doors;
  for (int x = g0; x <= g1; ++x) {
    for (int y = g0; y <= g1; ++y) {
      for (int d = 0; d < 4; ++d) {
        const int nx = x + mm::dx(static_cast<mm::Dir>(d)), ny = y + mm::dy(static_cast<mm::Dir>(d));
        if (!isGoal(nx, ny)) doors.push_back(Opening{x, y, static_cast<mm::Dir>(d)});
      }
    }
  }
  const Opening& door = doors[rng() % doors.size()];
  m.setWall(door.x, door.y, door.d, false);

  const int loops = n * n / 8;
  for (int tries = 0, added = 0; added < loops && tries < 2000; ++tries) {
    const int x = static_cast<int>(rng() % static_cast<unsigned>(n));
    const int y = static_cast<int>(rng() % static_cast<unsigned>(n));
    const mm::Dir d = (rng() & 1u) ? mm::EAST : mm::NORTH;
    const int nx = x + mm::dx(d), ny = y + mm::dy(d);
    if (nx >= n || ny >= n || !m.wall[x][y][d]) continue;
    if (isGoal(x, y) || isGoal(nx, ny)) continue;
    if (x == 0 && y == 0 && d == mm::EAST) continue;

    int p1i, p1j, p2i, p2j;
    if (d == mm::EAST) { p1i = x + 1; p1j = y; p2i = x + 1; p2j = y + 1; }
    else               { p1i = x; p1j = y + 1; p2i = x + 1; p2j = y + 1; }
    auto interior = [&](int i, int j) { return i >= 1 && j >= 1 && i <= n - 1 && j <= n - 1; };
    if (interior(p1i, p1j) && wallsAtPost(m, p1i, p1j) < 2) continue;
    if (interior(p2i, p2j) && wallsAtPost(m, p2i, p2j) < 2) continue;
    m.setWall(x, y, d, false);
    ++added;
  }
  return m;
}

int goalEntrances(const TrueMaze& m, const mm::CellSet& goal) {
  int doors = 0;
  for (int x = 0; x < m.width; ++x) {
    for (int y = 0; y < m.height; ++y) {
      const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      if (!goal.has(c)) continue;
      for (int d = 0; d < 4; ++d) {
        const mm::Cell nb = mm::neighbour(c, static_cast<mm::Dir>(d));
        const bool inside = nb.x >= 0 && nb.y >= 0 && nb.x < m.width && nb.y < m.height;
        if (inside && !goal.has(nb) && !m.wall[x][y][d]) ++doors;
      }
    }
  }
  return doors;
}

struct Outcome {
  bool ok = false;
  std::string why;
  int searchCells = 0;
  float speedS = 0.0f, optimalS = 0.0f;
  bool  pending = false;
  float finalS = 0.0f;
};

Outcome runOne(const TrueMaze& canonical, const mm::CellSet& goal, bool mirror) {
  Outcome o;
  const TrueMaze world = mirror ? canonical.mirrored() : canonical;
  fake::setWorld(world, mirror ? world.width - 1 : 0);
  MouseOptions opt;
  opt.loops = 3;
  opt.mirror = mirror;
  opt.useGoal = true;
  opt.goal = goal;
  opt.visualize = false;
  opt.logSummary = false;
  MouseReport rep;
  try {
    rep = runMouse(opt);
  } catch (const Crash& e) {
    o.why = e.what();
    return o;
  }
  o.searchCells = rep.searchMoves;
  o.speedS = rep.speedPathS;
  o.pending = rep.explorePending;
  if (!rep.allOk) {
    o.why = rep.failure;
    return o;
  }
  if (rep.conflicts != 0) {
    o.why = "wall conflicts with perfect sensors";
    return o;
  }

  const mm::Maze& learned = mouseMaze();
  for (int x = 0; x < canonical.width; ++x) {
    for (int y = 0; y < canonical.height; ++y) {
      const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      for (int d = 0; d < 4; ++d) {
        const mm::Dir dir = static_cast<mm::Dir>(d);
        if (learned.isKnown(c, dir) && learned.hasWall(c, dir) != canonical.wall[x][y][d]) {
          o.why = "learned map disagrees with the maze";
          return o;
        }
      }
    }
  }

  static mm::Maze full;
  static mm::Router router(full);
  static mm::Path best;
  canonical.toMaze(full);
  full.setGoal(goal);
  const mm::Cell start = {0, 0};
  if (router.fastest(start, mm::NORTH, goal, false, mouseTimeModel(1), best)) o.optimalS = best.timeS;

  static mm::Maze learnedCopy;
  static mm::Router learnedRouter(learnedCopy);
  learnedCopy = learned;
  learnedCopy.setGoal(goal);
  if (!learnedRouter.fastest(start, mm::NORTH, goal, false, mouseTimeModel(1), best)) {
    o.why = "no verified path after all runs";
    return o;
  }
  o.finalS = best.timeS;

  if (!o.pending && o.finalS > o.optimalS / (1.0f - PLAN_EXPLORE_GAIN) + 1e-3f) {
    o.why = "verified path exceeds the exploration bound";
    return o;
  }
  o.ok = true;
  return o;
}

std::vector<std::string> expandArgs(int argc, char** argv, bool& quiet) {
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--quiet") {
      quiet = true;
    } else if (!a.empty() && a[0] == '@') {
      std::ifstream list(a.substr(1).c_str());
      std::string line;
      while (std::getline(list, line)) {
        while (!line.empty() && (line[line.size() - 1] == '\r' || line[line.size() - 1] == ' ')) line.erase(line.size() - 1);
        if (!line.empty()) files.push_back(line);
      }
    } else {
      files.push_back(a);
    }
  }
  return files;
}

}

int main(int argc, char** argv) {
  if (argc >= 4 && std::strcmp(argv[1], "--gen") == 0) {
    const int count = std::atoi(argv[2]);
    const std::string dir = argv[3];
    for (int i = 1; i <= count; ++i) {
      const TrueMaze m = generateIsland(10, static_cast<unsigned>(1000 + i));
      char name[64];
      std::snprintf(name, sizeof(name), "/mmrc26-island-10x10-%02d.txt", i);
      if (!m.save(dir + name)) {
        std::cerr << "cannot write " << dir << name << std::endl;
        return 2;
      }
      if (goalEntrances(m, m.goal) != 1) {
        std::cerr << "generator bug: goal entrances != 1" << std::endl;
        return 2;
      }
    }
    std::cout << "generated " << count << " mazes in " << dir << std::endl;
    return 0;
  }

  bool quiet = false;
  const std::vector<std::string> files = expandArgs(argc, argv, quiet);
  int tested = 0, passed = 0, skipped = 0, optimal = 0, pendingCount = 0;
  double worstRatio = 1.0, sumRatio = 0.0;
  long searchCells = 0;
  std::string worstName;
  std::vector<std::string> failures;

  for (const std::string& f : files) {
    TrueMaze m;
    std::string err;
    if (!m.load(f, err)) {
      ++skipped;
      if (!quiet) std::cout << "SKIP " << f << " (" << err << ")" << std::endl;
      continue;
    }
    mm::CellSet goal = m.goal;
    if (goal.count() == 0) {
      mm::Maze tmp;
      tmp.reset(m.width, m.height);
      goal = tmp.goal();
    }

    {
      static mm::Maze full;
      static mm::Router router(full);
      m.toMaze(full);
      full.setGoal(goal);
      router.flood(goal, false);
      const mm::Cell start = {0, 0};
      if (router.dist(start) == mm::UNREACHABLE) {
        ++skipped;
        if (!quiet) std::cout << "SKIP " << f << " (goal unreachable in the file)" << std::endl;
        continue;
      }
    }
    bool ok = true;
    std::string why;
    Outcome first;
    for (int mirror = 0; mirror <= 1 && ok; ++mirror) {
      const Outcome o = runOne(m, goal, mirror == 1);
      if (!o.ok) {
        ok = false;
        why = std::string(mirror ? "[mirrored] " : "") + o.why;
      } else if (mirror == 0) {
        first = o;
      }
    }
    ++tested;
    if (!ok) {
      failures.push_back(f + ": " + why);
      if (!quiet) std::cout << "FAIL " << f << ": " << why << std::endl;
      continue;
    }
    ++passed;
    searchCells += first.searchCells;
    const double ratio = first.optimalS > 0.0f ? first.finalS / first.optimalS : 1.0;
    if (first.pending) ++pendingCount;
    sumRatio += ratio;
    if (ratio < 1.0005) ++optimal;
    if (ratio > worstRatio) {
      worstRatio = ratio;
      worstName = f;
    }
    if (!quiet) {
      std::printf("ok   %-60s search %4d cells, speed path %6.2f s (optimum %6.2f s)\n", f.c_str(), first.searchCells,
                  first.finalS, first.optimalS);
    }
  }

  std::printf("\n==== %d mazes tested (x2: normal + mirrored), %d passed, %d failed, %d skipped ====\n", tested,
              passed, tested - passed, skipped);
  if (passed > 0) {
    std::printf("average search length: %.1f cells\n", static_cast<double>(searchCells) / passed);
    std::printf("verified speed path == true optimum on %d of %d mazes; mean ratio %.4f, worst %.4f (%s)\n",
                optimal, passed, sumRatio / passed, worstRatio, worstName.c_str());
    std::printf("bound 1/(1-PLAN_EXPLORE_GAIN) = %.4f is enforced when exploring no longer pays (%d mazes still had worthwhile exploration left after 4 returns)\n", 1.0 / (1.0 - PLAN_EXPLORE_GAIN), pendingCount);
  }
  for (const std::string& s : failures) std::printf("FAIL %s\n", s.c_str());
  return failures.empty() ? 0 : 1;
}
