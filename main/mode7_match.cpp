#include "modes.h"

#include <Preferences.h>

#include "battery.h"
#include "calib.h"
#include "encoders.h"
#include "estimator.h"
#include "imu.h"
#include "motion.h"
#include "motors.h"
#include "sched.h"
#include "sonar.h"
#include "strategy.h"
#include "ui.h"

namespace modes {

void mode7Match() {
  DBG_PRINTF("\n== MODE 7: MATCH ==\n");
  if (!ROBOT_MAZE_READY) {
    DBG_PRINTF("match refused: chassis cannot pivot in a cell (corner swing %.0f mm).\n", sqrtf(PIVOT_CORNER_RADIUS_SQ_MM2));
    ui::soundError();
    return;
  }
  strategy::runMatch();
  motors::disable();
}

}
