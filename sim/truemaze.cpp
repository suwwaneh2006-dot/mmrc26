// =============================================================================
//  sim/truemaze.cpp - see truemaze.h
// =============================================================================
#include "truemaze.h"

#include <fstream>
#include <vector>

// ---------------------------------------------------------------------------
//  TrueMaze (mazefiles text format: posts 'o', '---' and '|' walls, 'G' goal)
// ---------------------------------------------------------------------------
void TrueMaze::setWall(int x, int y, mm::Dir d, bool present) {
  wall[x][y][d] = present;
  const int nx = x + mm::dx(d), ny = y + mm::dy(d);
  if (nx >= 0 && ny >= 0 && nx < width && ny < height) wall[nx][ny][mm::turnBack(d)] = present;
}

bool TrueMaze::load(const std::string& path, std::string& error) {
  std::ifstream in(path.c_str());
  if (!in) {
    error = "cannot open";
    return false;
  }
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(in, line)) {
    while (!line.empty() && (line[line.size() - 1] == '\r' || line[line.size() - 1] == ' ')) {
      line.erase(line.size() - 1);
    }
    lines.push_back(line);
  }
  while (!lines.empty() && lines.back().empty()) lines.pop_back();
  if (lines.size() < 3 || lines[0].size() < 5) {
    error = "not a maze file";
    return false;
  }
  height = static_cast<int>(lines.size() - 1) / 2;
  width = static_cast<int>(lines[0].size() - 1) / 4;
  if (width < 1 || height < 1 || width > mm::MAX_SIZE || height > mm::MAX_SIZE) {
    error = "size exceeds MAZE_MAX_SIZE";
    return false;
  }
  for (std::string& l : lines) l.resize(static_cast<size_t>(width * 4 + 1), ' ');
  goal.clear();
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const int r = height - 1 - y;
      const std::string& top = lines[2 * r];
      const std::string& mid = lines[2 * r + 1];
      const std::string& bot = lines[2 * r + 2];
      wall[x][y][mm::NORTH] = top[4 * x + 2] == '-';
      wall[x][y][mm::SOUTH] = bot[4 * x + 2] == '-';
      wall[x][y][mm::WEST] = mid[4 * x] == '|';
      wall[x][y][mm::EAST] = mid[4 * x + 4] == '|';
      if (mid[4 * x + 2] == 'G') {
        const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
        goal.add(c);
      }
    }
  }
  // Boundary is always walled.
  for (int x = 0; x < width; ++x) {
    wall[x][0][mm::SOUTH] = true;
    wall[x][height - 1][mm::NORTH] = true;
  }
  for (int y = 0; y < height; ++y) {
    wall[0][y][mm::WEST] = true;
    wall[width - 1][y][mm::EAST] = true;
  }
  return true;
}

bool TrueMaze::save(const std::string& path) const {
  std::ofstream out(path.c_str());
  if (!out) return false;
  for (int r = 0; r <= height; ++r) {
    // Horizontal wall line above text row r (cell row y = height - r).
    const int yAbove = height - r;   // cell whose SOUTH wall this is (if < height)
    std::string h;
    for (int x = 0; x < width; ++x) {
      bool w;
      if (yAbove == height) w = wall[x][height - 1][mm::NORTH];
      else w = wall[x][yAbove][mm::SOUTH];
      h += w ? "o---" : "o   ";
    }
    h += "o";
    out << h << "\n";
    if (r == height) break;
    const int y = height - 1 - r;
    std::string m;
    for (int x = 0; x < width; ++x) {
      const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      const char mark = (x == 0 && y == 0) ? 'S' : (goal.has(c) ? 'G' : ' ');
      m += wall[x][y][mm::WEST] ? "|" : " ";
      m += " ";
      m += mark;
      m += " ";
    }
    m += wall[width - 1][y][mm::EAST] ? "|" : " ";
    out << m << "\n";
  }
  return static_cast<bool>(out);
}

TrueMaze TrueMaze::mirrored() const {
  TrueMaze m;
  m.width = width;
  m.height = height;
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const int mx = width - 1 - x;
      m.wall[mx][y][mm::NORTH] = wall[x][y][mm::NORTH];
      m.wall[mx][y][mm::SOUTH] = wall[x][y][mm::SOUTH];
      m.wall[mx][y][mm::EAST] = wall[x][y][mm::WEST];
      m.wall[mx][y][mm::WEST] = wall[x][y][mm::EAST];
      const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      if (goal.has(c)) {
        const mm::Cell mc = {static_cast<int8_t>(mx), static_cast<int8_t>(y)};
        m.goal.add(mc);
      }
    }
  }
  return m;
}

void TrueMaze::toMaze(mm::Maze& m) const {
  m.reset(width, height);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      const mm::Cell c = {static_cast<int8_t>(x), static_cast<int8_t>(y)};
      for (int d = 0; d < 4; ++d) m.setWall(c, static_cast<mm::Dir>(d), wall[x][y][d]);
    }
  }
  if (goal.count() > 0) m.setGoal(goal);
}

