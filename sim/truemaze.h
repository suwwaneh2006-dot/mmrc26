// =============================================================================
//  sim/truemaze.h - a maze with every wall known (mazefiles text format),
//  shared by the mms batch tester and the closed-loop robot simulator.
// =============================================================================
#pragma once

#include <string>

#include "maze.h"

// A maze with every wall known, in mazefiles text format terms.
struct TrueMaze {
  int width = 0, height = 0;
  bool wall[mm::MAX_SIZE][mm::MAX_SIZE][4] = {};   // [x][y][dir]
  mm::CellSet goal;                                // 'G' cells from the file

  bool load(const std::string& path, std::string& error);
  bool save(const std::string& path) const;
  void setWall(int x, int y, mm::Dir d, bool present);
  TrueMaze mirrored() const;                       // reflect x -> width-1-x
  void toMaze(mm::Maze& m) const;                  // fully-known map
};
