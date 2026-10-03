// =============================================================================
//  sim/mms_main.cpp - entry point for the mms simulator
//  (https://github.com/mackorone/mms). Build with build_mms.bat and point the
//  mms mouse config at mouse.exe (see BRINGUP.md / the README in this folder).
//
//  The goal is the centre of the maze (2x2 for even sizes), exactly as on the
//  robot. Pressing "Reset" in mms restarts the whole sequence.
// =============================================================================
#include <iostream>

#include "API.h"
#include "mouse.h"

int main() {
  std::cerr << "MMRC26 maze brain in mms: " << API::mazeWidth() << "x" << API::mazeHeight() << std::endl;
  while (true) {
    MouseOptions options;
    options.loops = 3;
    const MouseReport rep = runMouse(options);
    if (!rep.allOk) std::cerr << "FAILED: " << rep.failure << std::endl;
    else std::cerr << "all runs complete - press Reset in mms to run again" << std::endl;
    // Idle until the user resets the simulation, then start over.
    while (!API::wasReset()) {
    }
    API::ackReset();
  }
}
