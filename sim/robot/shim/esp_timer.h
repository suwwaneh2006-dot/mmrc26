// =============================================================================
//  sim/robot/shim/esp_timer.h - periodic timers, called by the simulated clock.
// =============================================================================
#pragma once

#include <stdint.h>

typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1

typedef void (*esp_timer_cb_t)(void* arg);
typedef enum { ESP_TIMER_TASK, ESP_TIMER_ISR } esp_timer_dispatch_t;
typedef struct esp_timer* esp_timer_handle_t;

typedef struct {
  esp_timer_cb_t callback;
  void* arg;
  esp_timer_dispatch_t dispatch_method;
  const char* name;
  bool skip_unhandled_events;
} esp_timer_create_args_t;

esp_err_t esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* out);
esp_err_t esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us);
