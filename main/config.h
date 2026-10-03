// =============================================================================
//  MMRC26 micromouse - config.h
//
//  Every pin, physical constant, threshold and tuning value lives here.
//  Units are always part of the name: _MM, _MM_S, _MM_S2, _MS, _US, _DEG,
//  _DPS, _DPS2, _V, _HZ.
//
//  Tags used in comments:
//    TODO_MEASURE  value that must be measured on the real robot before the
//                  match (the comment says how).
//    TUNE          controller gain or margin, tuned with the test modes.
//    AUTO-CAL      default only; overwritten in NVS by mode 5.
//
//  This header is deliberately free of Arduino includes so that the pure C++
//  maze brain (maze.*) and the PC simulator harness can include it too.
// =============================================================================
#pragma once

#include <stdint.h>

// -----------------------------------------------------------------------------
//  Build flags
// -----------------------------------------------------------------------------
// 1 = Serial debug output (115200 baud). 0 = competition build: every debug
// print compiles out completely (see DBG_* macros at the bottom of this file).
#define MMRC_DEBUG 1

// -----------------------------------------------------------------------------
//  Maze geometry (MMRC26 rules)
// -----------------------------------------------------------------------------
constexpr int   MAZE_SIZE_CELLS   = 10;
constexpr float CELL_INNER_MM     = 180.0f;   // free space between walls
constexpr float WALL_THICK_MM     = 12.0f;
constexpr float CELL_PITCH_MM     = CELL_INNER_MM + WALL_THICK_MM;   // 192 mm
constexpr float HALF_CELL_INNER_MM = CELL_INNER_MM * 0.5f;           // 90 mm

// -----------------------------------------------------------------------------
//  Pin map (WEMOS LOLIN32 Lite)
// -----------------------------------------------------------------------------
constexpr uint8_t PIN_I2C_SDA      = 19;
constexpr uint8_t PIN_I2C_SCL      = 23;

constexpr uint8_t PIN_MOTOR_L_PWM  = 25;   // TB6612 PWMA
constexpr uint8_t PIN_MOTOR_L_IN1  = 26;   // TB6612 AIN1
constexpr uint8_t PIN_MOTOR_L_IN2  = 27;   // TB6612 AIN2
constexpr uint8_t PIN_MOTOR_R_PWM  = 32;   // TB6612 PWMB
constexpr uint8_t PIN_MOTOR_R_IN1  = 33;   // TB6612 BIN1
constexpr uint8_t PIN_MOTOR_R_IN2  = 13;   // TB6612 BIN2
constexpr uint8_t PIN_MOTOR_STBY   = 4;    // LOW = both motors disabled

constexpr uint8_t PIN_SONAR_F_TRIG = 16;
constexpr uint8_t PIN_SONAR_L_TRIG = 17;
constexpr uint8_t PIN_SONAR_R_TRIG = 18;
constexpr uint8_t PIN_SONAR_F_ECHO = 35;   // via 10k/15k divider (5 V -> 3.0 V)
constexpr uint8_t PIN_SONAR_L_ECHO = 36;   // via 10k/15k divider
constexpr uint8_t PIN_SONAR_R_ECHO = 39;   // via 10k/15k divider

constexpr uint8_t PIN_BATT_SENSE   = 34;   // 20k/10k divider, ADC1
constexpr uint8_t PIN_BUTTON       = 14;   // INPUT_PULLUP, pressed = LOW
constexpr uint8_t PIN_BUZZER       = 15;
// Wheel encoders (N20 magnetic hall), ONE channel per wheel (channel A).
// Direction comes from the motor command. GPIO 22 is the on-board LED pin:
// the LED is no longer used (error codes are beeped) and will flicker with
// the right wheel - harmless.
constexpr uint8_t PIN_ENC_L_A      = 5;
constexpr uint8_t PIN_ENC_R_A      = 22;

// Compile-time guard against the ESP32 pins that must not be used here:
// 0/2/12 are strapping pins, 6-11 are wired to the SPI flash and
// 34-39 are input-only (fine for ECHO/battery, fatal for an output).
constexpr bool pinUsableAsOutput(uint8_t p) {
  return p != 0 && p != 2 && p != 12 && !(p >= 6 && p <= 11) && p < 34;
}
constexpr bool pinUsableAsInput(uint8_t p) {
  return p != 0 && p != 2 && p != 12 && !(p >= 6 && p <= 11) && p <= 39;
}
static_assert(pinUsableAsOutput(PIN_MOTOR_L_PWM) && pinUsableAsOutput(PIN_MOTOR_L_IN1) &&
              pinUsableAsOutput(PIN_MOTOR_L_IN2) && pinUsableAsOutput(PIN_MOTOR_R_PWM) &&
              pinUsableAsOutput(PIN_MOTOR_R_IN1) && pinUsableAsOutput(PIN_MOTOR_R_IN2) &&
              pinUsableAsOutput(PIN_MOTOR_STBY), "illegal motor pin");
static_assert(pinUsableAsOutput(PIN_SONAR_F_TRIG) && pinUsableAsOutput(PIN_SONAR_L_TRIG) &&
              pinUsableAsOutput(PIN_SONAR_R_TRIG), "illegal TRIG pin");
static_assert(pinUsableAsOutput(PIN_BUZZER), "illegal UI pin");
static_assert(pinUsableAsInput(PIN_ENC_L_A) && pinUsableAsInput(PIN_ENC_R_A), "illegal encoder pin");

// Every GPIO may be used once only.
constexpr uint8_t ALL_PINS[] = {
  PIN_I2C_SDA, PIN_I2C_SCL, PIN_MOTOR_L_PWM, PIN_MOTOR_L_IN1, PIN_MOTOR_L_IN2, PIN_MOTOR_R_PWM,
  PIN_MOTOR_R_IN1, PIN_MOTOR_R_IN2, PIN_MOTOR_STBY, PIN_SONAR_F_TRIG, PIN_SONAR_L_TRIG, PIN_SONAR_R_TRIG,
  PIN_SONAR_F_ECHO, PIN_SONAR_L_ECHO, PIN_SONAR_R_ECHO, PIN_BATT_SENSE, PIN_BUTTON, PIN_BUZZER,
  PIN_ENC_L_A, PIN_ENC_R_A};
constexpr int ALL_PIN_COUNT = sizeof(ALL_PINS) / sizeof(ALL_PINS[0]);
constexpr bool pinUsedTwice(int i = 0, int j = 1) {
  return i >= ALL_PIN_COUNT - 1 ? false
         : j >= ALL_PIN_COUNT   ? pinUsedTwice(i + 1, i + 2)
         : ALL_PINS[i] == ALL_PINS[j] ? true
                                      : pinUsedTwice(i, j + 1);
}
static_assert(!pinUsedTwice(), "a GPIO is assigned twice in the pin map");
static_assert(pinUsableAsInput(PIN_BUTTON) && pinUsableAsInput(PIN_BATT_SENSE), "illegal input pin");
// The echo ISR reads the GPIO in1 register directly, which covers GPIO 32-39.
static_assert(PIN_SONAR_F_ECHO >= 32 && PIN_SONAR_L_ECHO >= 32 && PIN_SONAR_R_ECHO >= 32 &&
              PIN_SONAR_F_ECHO <= 39 && PIN_SONAR_L_ECHO <= 39 && PIN_SONAR_R_ECHO <= 39,
              "ECHO pins must be in GPIO 32-39");

// -----------------------------------------------------------------------------
//  Robot geometry
// -----------------------------------------------------------------------------
// TODO_MEASURE: calipers across the tyre.
constexpr float WHEEL_DIAMETER_MM = 34.0f;
// TODO_MEASURE: centre of left tyre contact to centre of right tyre contact.
constexpr float WHEEL_TRACK_MM    = 80.0f;
// TODO_MEASURE: axle centre to the very front of the robot (bumper/sonar face).
constexpr float ROBOT_NOSE_X_MM   = 45.0f;
// TODO_MEASURE: axle centre to the very back of the robot.
constexpr float ROBOT_TAIL_X_MM   = 45.0f;
// TODO_MEASURE: half of the widest point of the robot (wheels included).
constexpr float ROBOT_HALF_WIDTH_MM = 42.0f;
// Longer of the two half-lengths (front or back of the axle).
constexpr float ROBOT_HALF_LENGTH_MM = ROBOT_NOSE_X_MM > ROBOT_TAIL_X_MM ? ROBOT_NOSE_X_MM : ROBOT_TAIL_X_MM;

// Sonar mounting relative to the axle centre. x = forward, y = left,
// angle = CCW from straight ahead. Distances are measured from the sensor's
// transducer face.
// TODO_MEASURE: all nine values below, with a ruler, robot on a flat table.
constexpr float SONAR_F_X_MM = 40.0f, SONAR_F_Y_MM = 0.0f,   SONAR_F_ANGLE_DEG = 0.0f;
constexpr float SONAR_L_X_MM = 20.0f, SONAR_L_Y_MM = 30.0f,  SONAR_L_ANGLE_DEG = 90.0f;
constexpr float SONAR_R_X_MM = 20.0f, SONAR_R_Y_MM = -30.0f, SONAR_R_ANGLE_DEG = -90.0f;

// -----------------------------------------------------------------------------
//  Control loop and watchdog
// -----------------------------------------------------------------------------
constexpr uint32_t CONTROL_TICK_US          = 1000;  // fixed 1 kHz tick
constexpr float    CONTROL_TICK_S           = CONTROL_TICK_US * 1e-6f;
// A tick that starts later than this counts as an overrun.
constexpr uint32_t CONTROL_LATE_US          = 2000;
// A tick later than this is treated as a frozen loop: motors are disabled.
constexpr uint32_t CONTROL_OVERRUN_FAULT_US = 10000;
// Independent esp_timer watchdog: STBY is pulled LOW if the control tick has
// not fed it for this long (covers a completely hung main loop).
constexpr uint32_t CONTROL_WATCHDOG_MS      = 30;
constexpr uint32_t CONTROL_WATCHDOG_POLL_US = 5000;

// -----------------------------------------------------------------------------
//  Motors (TB6612FNG + 2x N20 6 V 1000 RPM, no encoders)
// -----------------------------------------------------------------------------
constexpr uint32_t MOTOR_PWM_HZ        = 20000;
constexpr uint8_t  MOTOR_PWM_BITS      = 10;
constexpr uint32_t MOTOR_PWM_MAX       = (1u << MOTOR_PWM_BITS) - 1u;   // 1023
constexpr float    MOTOR_VMAX_V        = 6.0f;    // never exceed the N20 rating
// Soft start: the applied voltage may change by at most this rate (both
// directions) to limit current spikes on the 500 mAh cells. TUNE
constexpr float    MOTOR_SLEW_V_PER_S  = 60.0f;
// Below this |volts| the H-bridge is put in short-brake.
constexpr float    MOTOR_ZERO_V        = 0.05f;
// Battery reading used for duty computation is clamped below at this value so
// a bad ADC sample can never produce a huge duty.
constexpr float    MOTOR_MIN_BATT_FOR_DUTY_V = 5.0f;
// TODO_MEASURE: run mode 2 (wheels in the air). Set true for a wheel that
// spins backwards when told "forward".
constexpr bool     MOTOR_L_INVERT      = false;
constexpr bool     MOTOR_R_INVERT      = false;
// -----------------------------------------------------------------------------
//  Wheel encoders
// -----------------------------------------------------------------------------
// Counting both edges of channel A. TODO_MEASURE (mode 1): mark a wheel, turn
// it exactly 10 turns by hand, set this to (edges shown) / 10.
// Typical N20: 7 magnet pulses x 2 edges x gear ratio (1:30 -> 420).
constexpr float    ENC_EDGES_PER_WHEEL_REV = 420.0f;
constexpr float    ENC_MM_PER_EDGE  = 3.14159265f * WHEEL_DIAMETER_MM / ENC_EDGES_PER_WHEEL_REV;
// Wheel speed = edges over this window, then a low-pass filter.
constexpr uint32_t ENC_SPEED_WINDOW_MS = 8;
constexpr float    ENC_SPEED_FILTER = 0.3f;     // 0..1, higher = faster/noisier
// Encoder fault: a wheel that shows no edge for this long while its motor
// is driven above STUCK_MIN_VOLTS AND the other wheel is counting is marked
// faulty; the firmware then falls back to the voltage model (no stop).
constexpr uint32_t ENC_SILENT_MS    = 300;
// A wheel silent this long while driven is left out at once (suspect) until
// it counts again; the fault above is only declared after ENC_SILENT_MS.
constexpr uint32_t ENC_SUSPECT_MS   = 60;
constexpr float    ENC_DRIVEN_MIN_MM_S = 60.0f;
// Uncertainty added for the distance driven while a dead encoder was still
// trusted (fraction of that distance; the motor model is ~10 % accurate).
constexpr float    ENC_FALLBACK_SIGMA_FRAC = 0.15f;
// A faulty encoder that counts this many edges again is trusted again.
constexpr uint32_t ENC_RECOVER_EDGES = 200;
// Closed-loop wheel speed (volts per mm/s of error, and integral).
// TUNE (mode 6 CSV: v_est vs v_ref): sluggish -> raise KP; buzzing or
// oscillating speed -> lower KP. KI removes the steady speed error.
constexpr float    SPEED_KP_V_PER_MM_S  = 0.004f;
constexpr float    SPEED_KI_V_PER_MM    = 0.02f;
constexpr float    SPEED_I_MAX_V        = 0.6f;
// Total speed feedback limit: a wrong encoder reading can never add more.
constexpr float    SPEED_FB_MAX_V       = 1.0f;
constexpr bool     PIVOT_WHEEL_FEEDBACK = false;

// Nominal N20 no-load speed at 6 V, used only for the default velocity model.
constexpr float    MOTOR_NOLOAD_RPM_AT_6V = 1000.0f;

// -----------------------------------------------------------------------------
//  Velocity model v = f(volts) + first-order lag (AUTO-CAL, mode 5)
//  Default: straight line from the dead-band voltage to 75 % of the no-load
//  speed at 6 V. Mode 5 replaces this with a measured table in NVS.
// -----------------------------------------------------------------------------
constexpr int   MODEL_MAX_POINTS          = 10;
constexpr float MODEL_DEFAULT_DEADBAND_V  = 0.8f;    // AUTO-CAL
constexpr float MODEL_DEFAULT_LOAD_FACTOR = 0.75f;   // AUTO-CAL
constexpr float MODEL_DEFAULT_TAU_S       = 0.060f;  // AUTO-CAL
constexpr float MODEL_DEFAULT_V6_MM_S =
    MOTOR_NOLOAD_RPM_AT_6V / 60.0f * 3.14159265f * WHEEL_DIAMETER_MM * MODEL_DEFAULT_LOAD_FACTOR;

// -----------------------------------------------------------------------------
//  Ultrasonic sensors (3x HC-SR04)
// -----------------------------------------------------------------------------
// Speed of sound: c = 331.3 + 0.606 * T [m/s]; distance = echo_us * c / 2000.
// At 19.6 C this is the classic 0.1715 mm/us.
// TODO_MEASURE: air temperature at the venue (thermometer, degrees C).
constexpr float    SONAR_AIR_TEMP_C        = 20.0f;
constexpr float    SONAR_MM_PER_ECHO_US    = (331.3f + 0.606f * SONAR_AIR_TEMP_C) / 2000.0f;
constexpr uint32_t SONAR_TRIG_PULSE_US     = 10;     // HC-SR04 datasheet minimum
constexpr uint32_t SONAR_GAP_MS            = 12;     // settle gap between pings. TUNE
// ECHO must rise this soon after TRIG or the ping counts as "missing".
constexpr uint32_t SONAR_RISE_TIMEOUT_US   = 6000;
// ECHO high for longer than this = sensor stuck -> flagged faulty.
constexpr uint32_t SONAR_STUCK_MS          = 250;
// Consecutive missing pings before a sensor is flagged faulty.
constexpr uint8_t  SONAR_MISS_FAULT_COUNT  = 5;
constexpr float    SONAR_MIN_MM            = 20.0f;  // HC-SR04 blind zone
constexpr float    SIDE_MAX_MM             = 250.0f; // beyond this: NO_WALL
constexpr float    FRONT_MAX_MM            = 1000.0f;
// Plausibility gate: a reading may differ from the last accepted one by at most
// |speed| * dt + this margin. TUNE
constexpr float    SONAR_GATE_MARGIN_MM    = 25.0f;
// After this many consecutive gate rejections the new value is accepted
// (re-lock), so a genuine step can never be locked out forever.
constexpr uint8_t  SONAR_GATE_RELOCK_COUNT = 3;
// While turning faster than this the gate is disabled (ranges change quickly).
constexpr float    SONAR_GATE_MAX_RATE_DPS = 60.0f;
// Side readings are only trusted when the heading error is below this.
constexpr float    SIDE_TRUST_DEG          = 12.0f;
// Front sonar is trusted as an odometer only below this range (stage 3).
constexpr float    FRONT_TRUST_MM          = 600.0f;
// Self-test at boot: listen for this long and require this many echo pulses.
// Long enough for 3 pings even if every ping is a 200 ms no-echo timeout.
constexpr uint32_t SONAR_SELFTEST_MS       = 1000;
constexpr uint8_t  SONAR_SELFTEST_MIN_PINGS = 3;

// Wall-present thresholds derived from geometry. With the robot centred in a
// cell a side sensor sees its wall at (90 mm - |sensor y offset|). The next
// wall beyond an open side is a full pitch further away, so "present" and
// "absent" are separated by margins that tolerate +-30 mm of off-centre
// position, with hysteresis between them.
constexpr float SIDE_L_EXPECT_MM  = HALF_CELL_INNER_MM - SONAR_L_Y_MM;
constexpr float SIDE_R_EXPECT_MM  = HALF_CELL_INNER_MM + SONAR_R_Y_MM;   // y is negative
constexpr float SIDE_WALL_ON_MARGIN_MM  = 60.0f;    // TUNE
constexpr float SIDE_WALL_OFF_MARGIN_MM = 85.0f;    // TUNE
// Front wall of the *current* cell, robot at cell centre: (90 - sensor x).
// Front wall one cell further: + one pitch. Threshold sits midway.
constexpr float FRONT_EXPECT_MM         = HALF_CELL_INNER_MM - SONAR_F_X_MM;
constexpr float FRONT_WALL_ON_MM        = FRONT_EXPECT_MM + CELL_PITCH_MM * 0.5f - 15.0f;
constexpr float FRONT_WALL_OFF_MM       = FRONT_EXPECT_MM + CELL_PITCH_MM * 0.5f + 15.0f;
static_assert(SIDE_L_EXPECT_MM + SIDE_WALL_OFF_MARGIN_MM < SIDE_MAX_MM, "side thresholds beyond range");
static_assert(SIDE_R_EXPECT_MM + SIDE_WALL_OFF_MARGIN_MM < SIDE_MAX_MM, "side thresholds beyond range");

// -----------------------------------------------------------------------------
//  IMU (MPU6050 / GY-521 via MPU6050_light)
// -----------------------------------------------------------------------------
constexpr uint32_t I2C_FREQ_HZ          = 400000;
constexpr uint16_t I2C_TIMEOUT_MS       = 2;      // never let I2C stall the tick
// Gyro full scale: 0 = 250, 1 = 500, 2 = 1000, 3 = 2000 deg/s (library codes).
constexpr int      IMU_GYRO_CONFIG      = 2;
constexpr float    IMU_GYRO_LSB_PER_DPS = 32.8f;  // must match IMU_GYRO_CONFIG
// Digital low-pass filter, register 0x1A: 2 = 94 Hz gyro bandwidth, 1 kHz rate.
constexpr uint8_t  IMU_DLPF_CFG         = 2;
// TODO_MEASURE: run mode 1 and turn the robot CCW (left) by hand. The heading
// must increase. If it decreases, set -1.
constexpr float    IMU_Z_SIGN           = 1.0f;
// TODO_MEASURE: run mode 4. If 4x90 deg is visibly off by X deg in total,
// multiply this by 360 / (360 + X) (X positive = over-rotated).
constexpr float    IMU_GYRO_SCALE       = 1.0f;
constexpr uint32_t IMU_BOOT_BIAS_MS     = 1000;   // robot must be still at boot
constexpr float    IMU_STILL_RATE_DPS   = 2.0f;   // "stationary" threshold
constexpr uint32_t IMU_STILL_MS         = 500;    // re-estimate bias after this
constexpr uint8_t  IMU_FAIL_LIMIT       = 10;     // consecutive I2C errors -> fault

// -----------------------------------------------------------------------------
//  Battery (2S LiPo, 20k/10k divider)
// -----------------------------------------------------------------------------
constexpr float    BATT_DIVIDER_RATIO     = 3.0f;
// TODO_MEASURE: multimeter on the pack / voltage printed by mode 1.
constexpr float    BATT_CAL_FACTOR        = 1.0f;
constexpr uint8_t  BATT_SAMPLES           = 16;    // moving average window
constexpr uint32_t BATT_SAMPLE_PERIOD_MS  = 2;     // 16 samples = 32 ms window
constexpr float    BATT_WARN_V            = 7.0f;
constexpr float    BATT_BLOCK_FAST_V      = 6.8f;  // no T3/T4 below this
constexpr float    BATT_REFUSE_ARM_V      = 6.6f;
constexpr float    BATT_CUTOFF_LOAD_V     = 6.4f;  // motors stop immediately

// -----------------------------------------------------------------------------
//  User interface
// -----------------------------------------------------------------------------
constexpr uint32_t BUTTON_DEBOUNCE_MS   = 25;
constexpr uint32_t BUTTON_LONG_MS       = 1500;
constexpr uint32_t CLICK_GAP_MS         = 800;    // clicks closer than this = one number
constexpr uint32_t BOOT_WIPE_HOLD_MS    = 2000;   // hold at boot to wipe the map
constexpr uint32_t MODE_START_DELAY_MS  = 1000;   // hands-off time before moving
// TODO_MEASURE: true for an active buzzer (beeps on plain DC), false for a
// passive one (needs a tone). Check: an active buzzer has a sealed back and
// beeps when connected to 3.3 V.
constexpr bool     BUZZER_ACTIVE        = true;
constexpr uint32_t BUZZER_TONE_HZ       = 2700;
constexpr uint32_t BUZZER_LOW_TONE_HZ   = 1200;   // error tone (passive buzzer only)
constexpr uint8_t  BUZZER_PWM_BITS      = 10;

// -----------------------------------------------------------------------------
//  Speed tiers
// -----------------------------------------------------------------------------
struct SpeedTier {
  float speed_mm_s;      // straight cruise speed
  float accel_mm_s2;     // straight acceleration limit
  float turnRate_dps;    // pivot peak rate
};
// Acceleration limit on a slick floor with small cells: 1.5 m/s^2.
constexpr float MAX_ACCEL_MM_S2 = 1500.0f;
constexpr SpeedTier TIERS[4] = {
  {250.0f, MAX_ACCEL_MM_S2, 360.0f},   // T1 search
  {400.0f, MAX_ACCEL_MM_S2, 450.0f},   // T2 safe
  {550.0f, MAX_ACCEL_MM_S2, 540.0f},   // T3 fast
  {700.0f, MAX_ACCEL_MM_S2, 630.0f},   // T4 hero
};
// Pivot angular acceleration such that each wheel stays within the linear
// acceleration limit: alpha = a / (track / 2).
constexpr float TURN_ACCEL_DPS2 = MAX_ACCEL_MM_S2 / (WHEEL_TRACK_MM * 0.5f) * 57.29578f;

// -----------------------------------------------------------------------------
//  Motion control (TUNE with modes 3, 4 and 6)
// -----------------------------------------------------------------------------
// Gyro heading hold on straights (differential volts per deg of error).
// TUNE (mode 3): robot weaves left-right -> lower KP or raise KD; drifts
// slowly off the line -> raise KP. Start low; never above 0.3.
constexpr float HEADING_KP_V_PER_DEG   = 0.08f;
constexpr float HEADING_KD_V_PER_DPS   = 0.004f;
constexpr float HEADING_MAX_CORR_V     = 1.5f;    // safety clamp, leave
// Integral term: removes the constant heading error caused by unequal motors.
// TUNE (mode 3): heading at the end of the straight still off to one side ->
// raise; slow weaving -> lower. HEADING_I_MAX_V limits wind-up.
constexpr float HEADING_KI_V_PER_DEG_S = 0.3f;
constexpr float HEADING_I_MAX_V        = 0.6f;
// Pivot controller: wheel speed command (mm/s) per deg of error / per deg/s.
// TUNE (mode 4): overshoot or oscillation at the end -> lower KP / raise KD;
// stops short and creeps for a long time -> raise KP.
constexpr float PIVOT_KP_MM_S_PER_DEG  = 6.0f;
constexpr float PIVOT_KD_MM_S_PER_DPS  = 0.25f;
constexpr float TURN_SETTLE_DEG        = 2.0f;    // spec: settled when |error| < 2 deg
constexpr float TURN_SETTLE_RATE_DPS   = 15.0f;   // ... and |rate| below this
constexpr uint32_t TURN_SETTLE_HOLD_MS = 40;      // ... for this long
// A pivot that has not settled this long after its profile ended reports a
// timeout. TUNE: raise only if mode 4 shows good turns timing out.
constexpr uint32_t TURN_TIMEOUT_MS     = 1500;
// Collision guard: brake when the gap in front of the nose is smaller than
// the stopping distance at BRAKE_DECEL + latency + COLLISION_MARGIN_MM.
// It is a backstop for UNEXPECTED walls: it must assume harder braking than
// the planned profile (accel limit), otherwise every planned stop in front of
// a wall would trip it. TUNE: if mode 3/6 shows the robot cannot really stop
// this hard (skids into the wall), lower it AND lower the tier speeds.
constexpr float BRAKE_DECEL_MM_S2      = 3000.0f;
constexpr float SONAR_LATENCY_S        = 0.04f;   // reading age + one slot
// Must stay below the nose-to-wall gap of a centred robot, or every planned
// stop in front of a wall would trip the guard.
constexpr float COLLISION_MARGIN_MM    = 20.0f;
static_assert(COLLISION_MARGIN_MM < HALF_CELL_INNER_MM - ROBOT_NOSE_X_MM - 5.0f,
              "collision margin would trip on every stop in front of a wall");
constexpr uint32_t SONAR_FRESH_MS      = 100;     // older readings are ignored
constexpr uint32_t MOTION_STOP_SETTLE_MS = 150;   // still time after a stop

// Continuous straights (stage 3). Positions are measured from the centre of
// the cell where the straight started.
// Walls of a cell are read when the axle is this far before the cell centre:
// the side sensors then face the middle of the wall segment, not a post.
// TUNE: must satisfy the static_assert below; larger = earlier decision but
// side sensors closer to the previous post.
constexpr float READ_POINT_BEFORE_CENTRE_MM = 40.0f;
// Side sensors closer than this to a post are ignored (beam hits the post).
constexpr float SIDE_POST_KEEPOUT_MM   = 30.0f;
static_assert(READ_POINT_BEFORE_CENTRE_MM - SONAR_L_X_MM < CELL_PITCH_MM * 0.5f - SIDE_POST_KEEPOUT_MM,
              "read point puts the side sensors on a post");
// The straight is finished when the remaining distance is below this.
constexpr float STOP_TOLERANCE_MM      = 2.0f;
// Slowest crawl before the end of a straight (prevents stalling short).
// TUNE: >= the speed of the lowest calibrated model point (mode 5 printout).
constexpr float PROFILE_MIN_SPEED_MM_S = 30.0f;
// Hardest deceleration the profile may ask for when a position correction
// suddenly shortens the remaining distance.
constexpr float PROFILE_MAX_DECEL_MM_S2 = 3000.0f;
static_assert(BRAKE_DECEL_MM_S2 >= 1.9f * MAX_ACCEL_MM_S2,
              "collision guard would fire on planned stops (see BRAKE_DECEL_MM_S2)");
// Wall centring: heading offset (deg) per mm of lateral error, blended with the
// gyro heading hold. TUNE (mode 3 in a corridor): robot swings across the
// corridor -> lower KP; stays off-centre -> raise KP. KD damps the swing.
constexpr float CENTER_KP_DEG_PER_MM   = 0.15f;
constexpr float CENTER_KD_DEG_S_PER_MM = 0.0f;
constexpr float CENTER_MAX_DEG         = 6.0f;    // never steer more than this
constexpr float CENTER_MIN_SPEED_MM_S  = 20.0f;   // no centring when (almost) stopped
constexpr uint32_t SIDE_SAMPLE_MAX_AGE_MS = 80;   // older side readings unused
constexpr float SIDE_MAX_RATE_DPS      = 30.0f;   // ignore sides while turning
// Front alignment (dead ends, start cell, before pivots with a wall ahead).
// Target: the front reading with the axle exactly on the cell centre.
constexpr float FRONT_CENTRE_READING_MM = HALF_CELL_INNER_MM - SONAR_F_X_MM;
// TUNE (mode 6 / match logs): oscillation in front of the wall -> lower KP.
constexpr float ALIGN_KP_PER_S         = 4.0f;    // mm/s of speed per mm of error
constexpr float ALIGN_MAX_SPEED_MM_S   = 60.0f;
constexpr float ALIGN_TOLERANCE_MM     = 3.0f;
constexpr uint32_t ALIGN_HOLD_MS       = 100;
constexpr uint32_t ALIGN_TIMEOUT_MS    = 1500;
// Stuck detection (spec: 400 ms). Only armed while the command is clearly
// above the motor dead-band, so a slow final creep is never "stuck".
// TUNE: STUCK_MIN_VOLTS = about 2x the dead-band voltage mode 5 prints.
// Pivot clearance: in the smallest legal cell (-5 %) a pivot only clears the
// walls if the axle is within PIVOT_LATERAL_LIMIT_MM of the corridor centre.
// Before every pivot the sideways offset is measured at rest; if it is larger,
// the robot re-centres: turn RECENTRE_ANGLE_DEG away from the near wall, back
// up |offset| / sin(angle), turn back.
// Squared radius swept by the farthest body corner during a pivot.
constexpr float PIVOT_CORNER_RADIUS_SQ_MM2 =
    ROBOT_HALF_LENGTH_MM * ROBOT_HALF_LENGTH_MM + ROBOT_HALF_WIDTH_MM * ROBOT_HALF_WIDTH_MM;
constexpr float PIVOT_LATERAL_LIMIT_MM = 12.0f;   // TUNE: never above the assert below
constexpr float RECENTRE_ANGLE_DEG     = 20.0f;
constexpr float RECENTRE_MAX_OFFSET_MM = 30.0f;   // larger offsets: misreading, don't act
constexpr bool  PIVOT_RECENTRE         = true;
constexpr uint32_t RECENTRE_READING_MAX_AGE_MS = 100;
static_assert(PIVOT_CORNER_RADIUS_SQ_MM2 < (HALF_CELL_INNER_MM * (1.0f - 0.05f) - PIVOT_LATERAL_LIMIT_MM) *
                                           (HALF_CELL_INNER_MM * (1.0f - 0.05f) - PIVOT_LATERAL_LIMIT_MM),
              "robot too large to pivot in a -5 % cell with this lateral limit");
constexpr uint32_t STUCK_MS            = 400;
constexpr float STUCK_MIN_RANGE_CHANGE_MM = 5.0f;
constexpr float STUCK_MAX_RATE_DPS     = 3.0f;
constexpr float STUCK_MIN_VOLTS        = 2.0f;
// With encoders: both wheels slower than this while driven = blocked wheels.
constexpr float STUCK_MIN_WHEEL_MM_S   = 10.0f;

// -----------------------------------------------------------------------------
//  Localisation without encoders (estimator, stage 3)
// -----------------------------------------------------------------------------
// The maze tolerance is +-5 % per cell (171..189 mm inside), so dead
// reckoning on the nominal pitch drifts up to ~9 mm per cell on top of the
// motor-model error: absolute corrections (front wall, posts) are essential.
constexpr float CELL_TOLERANCE_FRAC    = 0.05f;
// Process noise: position variance added per mm driven (random walk).
// Default: sigma grows to ~16 mm after one cell. TUNE (mode 6 CSV / match
// log): if good corrections get rejected as "too far", raise it; if the
// estimate jumps around on every reading, lower it.
constexpr float EST_Q_MM2_PER_MM       = 1.3f;
// Same with working encoders (wheel slip and diameter error only): sigma
// grows to ~7 mm per cell. TUNE like EST_Q_MM2_PER_MM.
constexpr float EST_Q_ENC_MM2_PER_MM   = 0.25f;
// Measurement noise (variance) of a front-wall fix and of a post edge.
// TUNE: front = (std. dev. of mode 1 front reading at rest)^2, x2 margin.
constexpr float EST_FRONT_R_MM2        = 64.0f;   // 8 mm sigma
constexpr float EST_EDGE_R_MM2         = 144.0f;  // 12 mm sigma
// Innovation gate: accept a fix if |error| < MIN + SIGMAS * predicted sigma.
constexpr float EST_GATE_MIN_MM        = 20.0f;
constexpr float EST_GATE_SIGMAS        = 3.0f;
// Front fixes only while nearly parallel (oblique walls reflect badly).
constexpr float EST_FRONT_TRUST_DEG    = 10.0f;
// After this many consecutive rejected front fixes, widen the gate by
// adding variance (lets the filter re-lock instead of ignoring a real wall).
constexpr uint8_t EST_REJECT_RUN       = 4;
constexpr float EST_REJECT_INFLATE_MM2 = 100.0f;
// Motor-model speed scale learned from front fixes (model error, battery).
constexpr float EST_SCALE_ADAPT_GAIN   = 0.2f;
constexpr float EST_SCALE_MIN          = 0.8f;
constexpr float EST_SCALE_MAX          = 1.25f;
constexpr float EST_SCALE_MIN_DIST_MM  = 150.0f;  // scale update every this many mm driven
// Post edges are less precise than a front wall: learn more slowly from them.
constexpr float EST_SCALE_ADAPT_GAIN_EDGE = 0.1f;
// Front fix on a wall the map does NOT know yet: everything that reflects
// straight ahead is a wall or post face on a cell boundary, so the reading is
// snapped to the nearest boundary (gated by EDGE_GATE_MM, so it can never
// jump by a cell). TUNE: set false if mode 7 logs show front rejects piling up.
constexpr bool  EST_LATTICE_FRONT_FIX  = true;
constexpr float EST_LATTICE_R_MM2      = 100.0f;  // 10 mm sigma
// Post landmarks: a side reading switches between wall and gap when the
// sensor passes a post. Post = 12 x 12 mm at every cell boundary.
constexpr float POST_HALF_MM           = 6.0f;
// Where the switch is seen relative to the post face, because of the beam
// width (+ = later). TODO_MEASURE with mode 6: drive past a known wall end;
// in the CSV compare the s where the side flips with the post position.
constexpr float EDGE_WALL_TO_GAP_MM    = 10.0f;
constexpr float EDGE_GAP_TO_WALL_MM    = 10.0f;
constexpr float EDGE_MAX_SPAN_MM       = 40.0f;   // readings too far apart: no edge
constexpr float EDGE_GATE_MM           = 60.0f;   // well below half a cell: unambiguous post
// Heading re-zero: a side wall seen parallel over this distance re-sets the
// gyro heading to the nearest 90 deg plus the measured wall angle.
constexpr float PARALLEL_SPAN_MM       = CELL_PITCH_MM;
constexpr int   PARALLEL_MIN_SAMPLES   = 5;
constexpr float PARALLEL_MAX_SLOPE     = 0.035f;  // ~2 deg
// A heading fix that would move the heading by more than this is discarded as
// implausible (e.g. a wall that is not straight, a missed post).
constexpr float PARALLEL_MAX_FIX_DEG   = 5.0f;
// Position confidence from the estimate's sigma.
// TUNE: if the robot slows down often in mode 7, raise GOOD a little.
constexpr float CONF_GOOD_SIGMA_MM     = 25.0f;   // above: slow to T1 speed
constexpr float CONF_LOST_SIGMA_MM     = 60.0f;   // above: stop and beep
constexpr float CONF_LOW_SPEED_MM_S    = 250.0f;
constexpr float START_SIGMA_MM         = 10.0f;   // robot placed by hand
constexpr float ALIGNED_SIGMA_MM       = 4.0f;    // after a front alignment
// TODO_MEASURE: where the operator places the robot in the start cell:
// axle distance in front of the cell centre (0 = centred; if the robot is
// pushed back against the rear wall: (axle-to-tail) - 90).
constexpr float START_OFFSET_MM        = 0.0f;

// -----------------------------------------------------------------------------
//  Match strategy (stage 4)
// -----------------------------------------------------------------------------
constexpr uint32_t MATCH_DURATION_MS   = 8u * 60u * 1000u;   // rules: 8 minutes
// No new run is started if (estimated run + return) * factor + margin does not
// fit in the remaining time.
constexpr float    MATCH_TIME_FACTOR   = 1.5f;
constexpr uint32_t MATCH_END_MARGIN_MS = 10000;
// Start trigger (spec): hand < 80 mm in front for 0.5 s, then removed.
constexpr float    TRIGGER_NEAR_MM     = 80.0f;
constexpr uint32_t TRIGGER_HOLD_MS     = 500;
constexpr float    TRIGGER_CLEAR_MM    = 150.0f;  // hand counts as removed
constexpr uint32_t TRIGGER_DELAY_MS    = 1000;    // beep -> STBY HIGH
// true: after each return the robot starts the next run by itself.
// false: wait for the hand-wave at the start cell between loops.
// Ask the judges which one is allowed (pre-match checklist).
constexpr bool     AUTO_RESTART        = true;
constexpr uint32_t START_PAUSE_MS      = 1000;    // pause at start before a run
// Wall reading at rest waits this long for 3 fresh readings per side (one
// side reading every ~55 ms with the F,L,F,R order).
constexpr uint32_t STATIONARY_READ_MS  = 200;
// Time budget assumed for a search + return when deciding whether the next
// loop still fits in the match. TUNE: the search time of your first match.
constexpr float    MATCH_SEARCH_EST_S  = 90.0f;
constexpr uint32_t BIAS_WAIT_MAX_MS    = 1500;    // wait for gyro stillness
// Tier policy (spec): 2 clean loops at T2, then one T3, then one T4.
constexpr int      CLEAN_LOOPS_BEFORE_HERO = 2;
// Map contradictions: each one inflates the position sigma by this much and
// drops a tier; this many in one run stops the robot.
constexpr float    CONFLICT_SIGMA_MM   = 20.0f;
constexpr int      CONFLICTS_STOP      = 3;
// Speed runs also check walls at each read point (conflict detection).
// TUNE: set false if mode 7 logs show false conflicts at T3/T4.
constexpr bool     SPEEDRUN_WALL_CHECK = true;
// Map mirrored to NVS after every cell (spec). Flash writes stall the CPU for
// a few ms; the control loop tolerates up to BLOCKING_SECTION_MAX_MS. TUNE:
// set false (map saved only when stopped) if writes cause "loop overrun".
constexpr bool     NVS_MIRROR_EVERY_CELL = true;
constexpr uint32_t BLOCKING_SECTION_MAX_MS = 80;

// -----------------------------------------------------------------------------
//  Exploring method (search run, and returns while the map is incomplete)
// -----------------------------------------------------------------------------
// true  = WALL HUGGING (hand rule). Simple, but it CANNOT reach a goal that is
//         an island (goal walls not connected to the walls around the start).
//         The firmware detects the circling and stops with
//         "wall follower looped" instead of driving forever.
// false = FLOOD FILL (shortest route, unknown walls open). Reaches any goal.
// Speed runs always use the shortest verified path on the learned map.
constexpr bool EXPLORE_WALL_HUG   = false;
// Which hand stays on the wall. The start cell has the outer wall on the
// LEFT (normal maze), so the left hand follows the outer boundary.
constexpr bool WALL_HUG_LEFT_HAND = true;

// -----------------------------------------------------------------------------
//  Path planning (maze brain time model)
// -----------------------------------------------------------------------------
// Settle time added to every pivot and fixed overhead per straight in the
// speed-run cost. They only change WHICH path is chosen (more/fewer turns).
// TUNE: set PLAN_TURN_SETTLE_S to the settle time mode 4 prints minus the
// profile time; PLAN_SEGMENT_S to the stop time seen in the mode 6 CSV.
constexpr float PLAN_TURN_SETTLE_S = 0.15f;
constexpr float PLAN_SEGMENT_S     = 0.15f;
// A return run explores unknown cells only if the optimistic route promises
// to be more than this fraction faster than the verified route (spec: 10 %).
constexpr float PLAN_EXPLORE_GAIN  = 0.10f;

// -----------------------------------------------------------------------------
//  Auto-calibration (mode 5): robot faces a wall ~3 cells away.
// -----------------------------------------------------------------------------
constexpr int   CAL_STEP_COUNT            = 7;
constexpr float CAL_STEP_V[CAL_STEP_COUNT] = {1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f};
constexpr float CAL_START_MIN_MM          = 450.0f;  // start range window
constexpr float CAL_START_MAX_MM          = 900.0f;
constexpr float CAL_START_MM              = 600.0f;  // reverse back to this
constexpr uint32_t CAL_REVERSE_TIMEOUT_MS = 4000;
constexpr float CAL_STOP_MM               = 140.0f;  // + stopping distance
constexpr float CAL_REVERSE_V             = 1.8f;
constexpr uint32_t CAL_SETTLE_MS          = 150;     // ignore samples before this
constexpr uint32_t CAL_STEP_TIMEOUT_MS    = 3000;
constexpr uint32_t CAL_NO_MOTION_MS       = 1200;    // dead-band detection
constexpr float CAL_NO_MOTION_MM          = 15.0f;
constexpr int   CAL_MIN_SAMPLES           = 5;
constexpr float CAL_MIN_SPEED_MM_S        = 20.0f;
constexpr float CAL_MAX_SPEED_MM_S        = 900.0f;  // stop stepping above this
constexpr int   CAL_MIN_POINTS            = 3;

// -----------------------------------------------------------------------------
//  Persistent storage (NVS / Preferences)
// -----------------------------------------------------------------------------
#define NVS_CAL_NAMESPACE "mmrc_cal"   // velocity model
#define NVS_MAP_NAMESPACE "mmrc_map"   // maze map mirror (stage 2)

// -----------------------------------------------------------------------------
//  Debug output. With MMRC_DEBUG == 0 nothing is printed and no Serial code is
//  generated, but the arguments are still type-checked (inside if (0)) so the
//  competition build has no unused-variable warnings.
// -----------------------------------------------------------------------------
#ifdef ARDUINO
  #if MMRC_DEBUG
    #define DBG_BEGIN(baud)  do { Serial.setTxBufferSize(4096); Serial.begin(baud); } while (0)
    #define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
  #else
    #define DBG_BEGIN(baud)  do { (void)(baud); } while (0)
    #define DBG_PRINTF(...)  do { if (0) { (void)snprintf(nullptr, 0, __VA_ARGS__); } } while (0)
  #endif
#endif
