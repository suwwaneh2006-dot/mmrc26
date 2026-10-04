#pragma once

#include <Arduino.h>
#include "config.h"

namespace encoders {

void begin();

void update(float dt_s);

float leftMm();
float rightMm();
float leftSpeed();
float rightSpeed();
float speed();
uint32_t leftEdges();
uint32_t rightEdges();

bool healthy();
bool leftFaulty();
bool rightFaulty();
void clearFaults();
void setMmPerEdge(int wheel, float forward, float backward);

bool leftUsable();
bool rightUsable();

}
