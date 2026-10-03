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

    while (!API::wasReset()) {
    }
    API::ackReset();
  }
}
