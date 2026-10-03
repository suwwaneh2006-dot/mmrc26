#pragma once

#include <string>

#include "maze.h"

struct TrueMaze {
  int width = 0, height = 0;
  bool wall[mm::MAX_SIZE][mm::MAX_SIZE][4] = {};
  mm::CellSet goal;

  bool load(const std::string& path, std::string& error);
  bool save(const std::string& path) const;
  void setWall(int x, int y, mm::Dir d, bool present);
  TrueMaze mirrored() const;
  void toMaze(mm::Maze& m) const;
};
