#pragma once

#include <stdint.h>

typedef struct {
  struct {
    volatile uint32_t val;
  } in1;
} gpio_dev_t;

extern gpio_dev_t GPIO;
