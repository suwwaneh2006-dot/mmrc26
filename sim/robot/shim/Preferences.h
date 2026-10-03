#pragma once

#include "Arduino.h"

class Preferences {
 public:
  bool   begin(const char* name, bool readOnly = false);
  void   end();
  size_t getBytes(const char* key, void* buf, size_t maxLen);
  size_t putBytes(const char* key, const void* value, size_t len);
  bool   clear();
 private:
  char ns_[32] = {0};
  bool open_ = false;
  bool readOnly_ = false;
};
