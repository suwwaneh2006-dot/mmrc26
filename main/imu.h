// =============================================================================
//  imu.h - MPU6050 yaw-rate gyro and heading integration.
//
//  MPU6050_light configures the chip (range, wake-up) and probes it. The 1 kHz
//  read path is our own 2-byte burst of GYRO_ZOUT: the library's update() reads
//  all 14 bytes (~0.4 ms per call at 400 kHz) and integrates with millis(),
//  which has only 1 ms resolution - useless at a 1 kHz tick. Heading here is
//  integrated with the exact tick dt from micros().
//
//  Heading convention: degrees, CCW (left turn) positive, not wrapped.
//  Gyro bias is measured at boot and re-estimated automatically every time the
//  robot has been stationary (motors idle and rate small) for IMU_STILL_MS.
// =============================================================================
#pragma once

#include <Arduino.h>
#include "config.h"

namespace imu {

// Start I2C, configure the MPU6050, measure the initial bias (robot must be
// still for IMU_BOOT_BIAS_MS). Returns false if the chip does not answer.
bool begin();

// Called once per control tick. motorsIdle allows bias re-estimation.
void update(float dt_s, bool motorsIdle);

bool  ok();                 // chip answered and has not failed since
float rateDps();            // bias-corrected yaw rate, CCW positive
float headingDeg();         // integrated heading
void  setHeading(float deg);
float biasDps();
bool  stationary();         // still long enough for a bias update
uint32_t errorCount();      // total I2C errors since boot
uint8_t  whoAmI();          // WHO_AM_I register (0x68 genuine, clones differ)

}  // namespace imu
