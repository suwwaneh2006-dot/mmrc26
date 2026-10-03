#pragma once

#include <Arduino.h>
#include "config.h"

namespace modes {

void bootWipeCheck();

void bootSelfCheck(bool imuOk);

void menu();

}
