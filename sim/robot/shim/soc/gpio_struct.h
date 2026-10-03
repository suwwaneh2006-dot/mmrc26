// =============================================================================
//  sim/robot/shim/soc/gpio_struct.h - the GPIO input register the echo ISR
//  reads (GPIO 32-39 = bits 0-7 of in1), driven by the simulated sonars.
// =============================================================================
#pragma once

#include <stdint.h>

typedef struct {
  struct {
    volatile uint32_t val;
  } in1;
} gpio_dev_t;

extern gpio_dev_t GPIO;
