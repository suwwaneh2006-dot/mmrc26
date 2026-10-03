#include "imu.h"

#include <MPU6050_light.h>
#include <Wire.h>

namespace {

constexpr uint8_t REG_CONFIG     = 0x1A;
constexpr uint8_t REG_GYRO_ZOUT  = 0x47;
constexpr uint8_t REG_WHO_AM_I   = 0x75;

MPU6050 g_mpu(Wire);

bool     g_ok        = false;
uint8_t  g_whoAmI    = 0;
uint8_t  g_failRun   = 0;
uint32_t g_errors    = 0;
float    g_bias      = 0.0f;
float    g_rate      = 0.0f;
float    g_heading   = 0.0f;

float    g_stillSum   = 0.0f;
uint32_t g_stillCount = 0;
float    g_stillTime  = 0.0f;

bool readRawRate(float& out) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(REG_GYRO_ZOUT);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<uint8_t>(MPU6050_ADDR), static_cast<size_t>(2)) != 2) return false;
  const uint8_t hi = static_cast<uint8_t>(Wire.read());
  const uint8_t lo = static_cast<uint8_t>(Wire.read());
  const int16_t raw = static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
  out = IMU_Z_SIGN * IMU_GYRO_SCALE * (raw / IMU_GYRO_LSB_PER_DPS);
  return true;
}

void resetStill() {
  g_stillSum = 0.0f;
  g_stillCount = 0;
  g_stillTime = 0.0f;
}

}

namespace imu {

bool begin() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);

  g_ok = false;
  if (g_mpu.begin(IMU_GYRO_CONFIG, 0) != 0) return false;
  g_whoAmI = g_mpu.readData(REG_WHO_AM_I);
  if (g_whoAmI == 0x00 || g_whoAmI == 0xFF) return false;
  if (g_mpu.writeData(REG_CONFIG, IMU_DLPF_CFG) != 0) return false;

  delay(50);
  float sum = 0.0f;
  uint32_t n = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < IMU_BOOT_BIAS_MS) {
    float r;
    if (readRawRate(r)) {
      sum += r;
      ++n;
    }
    delayMicroseconds(1000);
  }
  if (n < IMU_BOOT_BIAS_MS / 2) return false;
  g_bias = sum / n;
  g_heading = 0.0f;
  g_rate = 0.0f;
  g_failRun = 0;
  resetStill();
  g_ok = true;
  return true;
}

void update(float dt_s, bool motorsIdle) {
  float raw;
  if (!readRawRate(raw)) {
    ++g_errors;
    if (g_failRun < 255) ++g_failRun;
    if (g_failRun >= IMU_FAIL_LIMIT) g_ok = false;

    g_heading += g_rate * dt_s;
    return;
  }
  g_failRun = 0;
  g_rate = raw - g_bias;
  g_heading += g_rate * dt_s;

  if (motorsIdle && fabsf(g_rate) < IMU_STILL_RATE_DPS) {
    g_stillSum += raw;
    ++g_stillCount;
    g_stillTime = fminf(g_stillTime + dt_s, 60.0f);
    if (g_stillTime * 1000.0f >= IMU_STILL_MS && g_stillCount * CONTROL_TICK_US >= IMU_STILL_MS * 1000u) {
      g_bias = g_stillSum / g_stillCount;
      g_stillSum = 0.0f;
      g_stillCount = 0;
    }
  } else {
    resetStill();
  }
}

bool  ok()          { return g_ok; }
float rateDps()     { return g_rate; }
float headingDeg()  { return g_heading; }
void  setHeading(float deg) { g_heading = deg; }
float biasDps()     { return g_bias; }
bool  stationary()  { return g_stillTime * 1000.0f >= IMU_STILL_MS; }
uint32_t errorCount() { return g_errors; }
uint8_t  whoAmI()   { return g_whoAmI; }

}
