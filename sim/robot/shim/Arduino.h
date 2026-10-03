#pragma once

#include <cstdint>
#include <cmath>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef ARDUINO
#define ARDUINO 10800
#endif

#define IRAM_ATTR
#define ARDUINO_ISR_ATTR

typedef uint8_t byte;

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define CHANGE 0x03

enum adc_attenuation_t { ADC_0db, ADC_2_5db, ADC_6db, ADC_11db };

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void delayMicroseconds(uint32_t us);

void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int  digitalRead(uint8_t pin);
void attachInterruptArg(uint8_t pin, void (*fn)(void*), void* arg, int mode);

bool     ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
bool     ledcWrite(uint8_t pin, uint32_t duty);
uint32_t ledcWriteTone(uint8_t pin, uint32_t freq);

uint32_t analogReadMilliVolts(uint8_t pin);
void     analogSetPinAttenuation(uint8_t pin, adc_attenuation_t attenuation);

class SimSerial {
 public:
  void begin(unsigned long baud);
  void setTxBufferSize(size_t size);
  int printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};
extern SimSerial Serial;
