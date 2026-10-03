#pragma once

#include "Arduino.h"

class TwoWire {
 public:
  bool    begin(int sda, int scl, uint32_t frequency = 0);
  void    setTimeOut(uint16_t timeOutMillis);
  void    beginTransmission(uint8_t address);
  void    beginTransmission(int address) { beginTransmission(static_cast<uint8_t>(address)); }
  size_t  write(uint8_t data);
  uint8_t endTransmission(bool sendStop = true);
  size_t  requestFrom(uint8_t address, size_t len);
  uint8_t requestFrom(uint8_t address, uint8_t len) {
    return static_cast<uint8_t>(requestFrom(address, static_cast<size_t>(len)));
  }
  int     read();
};
extern TwoWire Wire;
