#pragma once

#include <Arduino.h>
#include "config.h"

namespace sched {

using Controller = void (*)(float dt_s);

void begin();

void setController(Controller fn);

void service();

void waitMs(uint32_t ms);

void blockingSection(void (*fn)());

void resetStats();
uint32_t overruns();
uint32_t maxLateUs();
uint32_t tickCount();

}
