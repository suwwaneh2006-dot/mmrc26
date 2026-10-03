#pragma once

#include <stdint.h>

#define MMRC_DEBUG 1

constexpr int   MAZE_SIZE_CELLS   = 10;
constexpr float CELL_INNER_MM     = 180.0f;
constexpr float WALL_THICK_MM     = 12.0f;
constexpr float CELL_PITCH_MM     = CELL_INNER_MM + WALL_THICK_MM;
constexpr float HALF_CELL_INNER_MM = CELL_INNER_MM * 0.5f;

constexpr uint8_t PIN_I2C_SDA      = 19;
constexpr uint8_t PIN_I2C_SCL      = 23;

constexpr uint8_t PIN_MOTOR_L_PWM  = 25;
constexpr uint8_t PIN_MOTOR_L_IN1  = 26;
constexpr uint8_t PIN_MOTOR_L_IN2  = 27;
constexpr uint8_t PIN_MOTOR_R_PWM  = 32;
constexpr uint8_t PIN_MOTOR_R_IN1  = 33;
constexpr uint8_t PIN_MOTOR_R_IN2  = 13;
constexpr uint8_t PIN_MOTOR_STBY   = 4;

constexpr uint8_t PIN_SONAR_F_TRIG = 16;
constexpr uint8_t PIN_SONAR_L_TRIG = 17;
constexpr uint8_t PIN_SONAR_R_TRIG = 18;
constexpr uint8_t PIN_SONAR_F_ECHO = 35;
constexpr uint8_t PIN_SONAR_L_ECHO = 36;
constexpr uint8_t PIN_SONAR_R_ECHO = 39;

constexpr uint8_t PIN_BATT_SENSE   = 34;
constexpr uint8_t PIN_BUTTON       = 14;
constexpr uint8_t PIN_BUZZER       = 15;

constexpr uint8_t PIN_ENC_L_A      = 5;
constexpr uint8_t PIN_ENC_R_A      = 22;

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

static_assert(PIN_SONAR_F_ECHO >= 32 && PIN_SONAR_L_ECHO >= 32 && PIN_SONAR_R_ECHO >= 32 &&
              PIN_SONAR_F_ECHO <= 39 && PIN_SONAR_L_ECHO <= 39 && PIN_SONAR_R_ECHO <= 39,
              "ECHO pins must be in GPIO 32-39");

constexpr float WHEEL_DIAMETER_MM = 34.0f;

constexpr float WHEEL_TRACK_MM    = 80.0f;

constexpr float ROBOT_NOSE_X_MM   = 45.0f;

constexpr float ROBOT_TAIL_X_MM   = 45.0f;

constexpr float ROBOT_HALF_WIDTH_MM = 42.0f;

constexpr float ROBOT_HALF_LENGTH_MM = ROBOT_NOSE_X_MM > ROBOT_TAIL_X_MM ? ROBOT_NOSE_X_MM : ROBOT_TAIL_X_MM;

constexpr float SONAR_F_X_MM = 40.0f, SONAR_F_Y_MM = 0.0f,   SONAR_F_ANGLE_DEG = 0.0f;
constexpr float SONAR_L_X_MM = 20.0f, SONAR_L_Y_MM = 30.0f,  SONAR_L_ANGLE_DEG = 90.0f;
constexpr float SONAR_R_X_MM = 20.0f, SONAR_R_Y_MM = -30.0f, SONAR_R_ANGLE_DEG = -90.0f;

constexpr uint32_t CONTROL_TICK_US          = 1000;
constexpr float    CONTROL_TICK_S           = CONTROL_TICK_US * 1e-6f;

constexpr uint32_t CONTROL_LATE_US          = 2000;

constexpr uint32_t CONTROL_OVERRUN_FAULT_US = 10000;

constexpr uint32_t CONTROL_WATCHDOG_MS      = 30;
constexpr uint32_t CONTROL_WATCHDOG_POLL_US = 5000;

constexpr uint32_t MOTOR_PWM_HZ        = 20000;
constexpr uint8_t  MOTOR_PWM_BITS      = 10;
constexpr uint32_t MOTOR_PWM_MAX       = (1u << MOTOR_PWM_BITS) - 1u;
constexpr float    MOTOR_VMAX_V        = 6.0f;

constexpr float    MOTOR_SLEW_V_PER_S  = 60.0f;

constexpr float    MOTOR_ZERO_V        = 0.05f;

constexpr float    MOTOR_MIN_BATT_FOR_DUTY_V = 5.0f;

constexpr bool     MOTOR_L_INVERT      = false;
constexpr bool     MOTOR_R_INVERT      = false;

constexpr float    ENC_EDGES_PER_WHEEL_REV = 420.0f;
constexpr float    ENC_MM_PER_EDGE  = 3.14159265f * WHEEL_DIAMETER_MM / ENC_EDGES_PER_WHEEL_REV;

constexpr uint32_t ENC_SPEED_WINDOW_MS = 8;
constexpr float    ENC_SPEED_FILTER = 0.3f;

constexpr uint32_t ENC_SILENT_MS    = 300;

constexpr uint32_t ENC_SUSPECT_MS   = 60;
constexpr float    ENC_DRIVEN_MIN_MM_S = 60.0f;

constexpr float    ENC_FALLBACK_SIGMA_FRAC = 0.15f;

constexpr uint32_t ENC_RECOVER_EDGES = 200;

constexpr float    SPEED_KP_V_PER_MM_S  = 0.004f;
constexpr float    SPEED_KI_V_PER_MM    = 0.02f;
constexpr float    SPEED_I_MAX_V        = 0.6f;

constexpr float    SPEED_FB_MAX_V       = 1.0f;
constexpr bool     PIVOT_WHEEL_FEEDBACK = false;

constexpr float    MOTOR_NOLOAD_RPM_AT_6V = 1000.0f;

constexpr int   MODEL_MAX_POINTS          = 10;
constexpr float MODEL_DEFAULT_DEADBAND_V  = 0.8f;
constexpr float MODEL_DEFAULT_LOAD_FACTOR = 0.75f;
constexpr float MODEL_DEFAULT_TAU_S       = 0.060f;
constexpr float MODEL_DEFAULT_V6_MM_S =
    MOTOR_NOLOAD_RPM_AT_6V / 60.0f * 3.14159265f * WHEEL_DIAMETER_MM * MODEL_DEFAULT_LOAD_FACTOR;

constexpr float    SONAR_AIR_TEMP_C        = 20.0f;
constexpr float    SONAR_MM_PER_ECHO_US    = (331.3f + 0.606f * SONAR_AIR_TEMP_C) / 2000.0f;
constexpr uint32_t SONAR_TRIG_PULSE_US     = 10;
constexpr uint32_t SONAR_GAP_MS            = 12;

constexpr uint32_t SONAR_RISE_TIMEOUT_US   = 6000;

constexpr uint32_t SONAR_STUCK_MS          = 250;

constexpr uint8_t  SONAR_MISS_FAULT_COUNT  = 5;
constexpr float    SONAR_MIN_MM            = 20.0f;
constexpr float    SIDE_MAX_MM             = 250.0f;
constexpr float    FRONT_MAX_MM            = 1000.0f;

constexpr float    SONAR_GATE_MARGIN_MM    = 25.0f;

constexpr uint8_t  SONAR_GATE_RELOCK_COUNT = 3;

constexpr float    SONAR_GATE_MAX_RATE_DPS = 60.0f;

constexpr float    SIDE_TRUST_DEG          = 12.0f;

constexpr float    FRONT_TRUST_MM          = 600.0f;

constexpr uint32_t SONAR_SELFTEST_MS       = 1000;
constexpr uint8_t  SONAR_SELFTEST_MIN_PINGS = 3;

constexpr float SIDE_L_EXPECT_MM  = HALF_CELL_INNER_MM - SONAR_L_Y_MM;
constexpr float SIDE_R_EXPECT_MM  = HALF_CELL_INNER_MM + SONAR_R_Y_MM;
constexpr float SIDE_WALL_ON_MARGIN_MM  = 60.0f;
constexpr float SIDE_WALL_OFF_MARGIN_MM = 85.0f;

constexpr float FRONT_EXPECT_MM         = HALF_CELL_INNER_MM - SONAR_F_X_MM;
constexpr float FRONT_WALL_ON_MM        = FRONT_EXPECT_MM + CELL_PITCH_MM * 0.5f - 15.0f;
constexpr float FRONT_WALL_OFF_MM       = FRONT_EXPECT_MM + CELL_PITCH_MM * 0.5f + 15.0f;
static_assert(SIDE_L_EXPECT_MM + SIDE_WALL_OFF_MARGIN_MM < SIDE_MAX_MM, "side thresholds beyond range");
static_assert(SIDE_R_EXPECT_MM + SIDE_WALL_OFF_MARGIN_MM < SIDE_MAX_MM, "side thresholds beyond range");

constexpr uint32_t I2C_FREQ_HZ          = 400000;
constexpr uint16_t I2C_TIMEOUT_MS       = 2;

constexpr int      IMU_GYRO_CONFIG      = 2;
constexpr float    IMU_GYRO_LSB_PER_DPS = 32.8f;

constexpr uint8_t  IMU_DLPF_CFG         = 2;

constexpr float    IMU_Z_SIGN           = 1.0f;

constexpr float    IMU_GYRO_SCALE       = 1.0f;
constexpr uint32_t IMU_BOOT_BIAS_MS     = 1000;
constexpr float    IMU_STILL_RATE_DPS   = 2.0f;
constexpr uint32_t IMU_STILL_MS         = 500;
constexpr uint8_t  IMU_FAIL_LIMIT       = 10;

constexpr float    BATT_DIVIDER_RATIO     = 3.0f;

constexpr float    BATT_CAL_FACTOR        = 1.0f;
constexpr uint8_t  BATT_SAMPLES           = 16;
constexpr uint32_t BATT_SAMPLE_PERIOD_MS  = 2;
constexpr float    BATT_WARN_V            = 7.0f;
constexpr float    BATT_BLOCK_FAST_V      = 6.8f;
constexpr float    BATT_REFUSE_ARM_V      = 6.6f;
constexpr float    BATT_CUTOFF_LOAD_V     = 6.4f;

constexpr uint32_t BUTTON_DEBOUNCE_MS   = 25;
constexpr uint32_t BUTTON_LONG_MS       = 1500;
constexpr uint32_t CLICK_GAP_MS         = 800;
constexpr uint32_t BOOT_WIPE_HOLD_MS    = 2000;
constexpr uint32_t MODE_START_DELAY_MS  = 1000;

constexpr bool     BUZZER_ACTIVE        = true;
constexpr uint32_t BUZZER_TONE_HZ       = 2700;
constexpr uint32_t BUZZER_LOW_TONE_HZ   = 1200;
constexpr uint8_t  BUZZER_PWM_BITS      = 10;

struct SpeedTier {
  float speed_mm_s;
  float accel_mm_s2;
  float turnRate_dps;
};

constexpr float MAX_ACCEL_MM_S2 = 1500.0f;
constexpr SpeedTier TIERS[4] = {
  {250.0f, MAX_ACCEL_MM_S2, 360.0f},
  {400.0f, MAX_ACCEL_MM_S2, 450.0f},
  {550.0f, MAX_ACCEL_MM_S2, 540.0f},
  {700.0f, MAX_ACCEL_MM_S2, 630.0f},
};

constexpr float TURN_ACCEL_DPS2 = MAX_ACCEL_MM_S2 / (WHEEL_TRACK_MM * 0.5f) * 57.29578f;

constexpr float HEADING_KP_V_PER_DEG   = 0.08f;
constexpr float HEADING_KD_V_PER_DPS   = 0.004f;
constexpr float HEADING_MAX_CORR_V     = 1.5f;

constexpr float HEADING_KI_V_PER_DEG_S = 0.3f;
constexpr float HEADING_I_MAX_V        = 0.6f;

constexpr float PIVOT_KP_MM_S_PER_DEG  = 6.0f;
constexpr float PIVOT_KD_MM_S_PER_DPS  = 0.25f;
constexpr float TURN_SETTLE_DEG        = 2.0f;
constexpr float TURN_SETTLE_RATE_DPS   = 15.0f;
constexpr uint32_t TURN_SETTLE_HOLD_MS = 40;

constexpr uint32_t TURN_TIMEOUT_MS     = 1500;

constexpr float BRAKE_DECEL_MM_S2      = 3000.0f;
constexpr float SONAR_LATENCY_S        = 0.04f;

constexpr float COLLISION_MARGIN_MM    = 20.0f;
static_assert(COLLISION_MARGIN_MM < HALF_CELL_INNER_MM - ROBOT_NOSE_X_MM - 5.0f,
              "collision margin would trip on every stop in front of a wall");
constexpr uint32_t SONAR_FRESH_MS      = 100;
constexpr uint32_t MOTION_STOP_SETTLE_MS = 150;

constexpr float READ_POINT_BEFORE_CENTRE_MM = 40.0f;

constexpr float SIDE_POST_KEEPOUT_MM   = 30.0f;
static_assert(READ_POINT_BEFORE_CENTRE_MM - SONAR_L_X_MM < CELL_PITCH_MM * 0.5f - SIDE_POST_KEEPOUT_MM,
              "read point puts the side sensors on a post");

constexpr float STOP_TOLERANCE_MM      = 2.0f;

constexpr float PROFILE_MIN_SPEED_MM_S = 30.0f;

constexpr float PROFILE_MAX_DECEL_MM_S2 = 3000.0f;
static_assert(BRAKE_DECEL_MM_S2 >= 1.9f * MAX_ACCEL_MM_S2,
              "collision guard would fire on planned stops (see BRAKE_DECEL_MM_S2)");

constexpr float CENTER_KP_DEG_PER_MM   = 0.15f;
constexpr float CENTER_KD_DEG_S_PER_MM = 0.0f;
constexpr float CENTER_MAX_DEG         = 6.0f;
constexpr float CENTER_MIN_SPEED_MM_S  = 20.0f;
constexpr uint32_t SIDE_SAMPLE_MAX_AGE_MS = 80;
constexpr float SIDE_MAX_RATE_DPS      = 30.0f;

constexpr float FRONT_CENTRE_READING_MM = HALF_CELL_INNER_MM - SONAR_F_X_MM;

constexpr float ALIGN_KP_PER_S         = 4.0f;
constexpr float ALIGN_MAX_SPEED_MM_S   = 60.0f;
constexpr float ALIGN_TOLERANCE_MM     = 3.0f;
constexpr uint32_t ALIGN_HOLD_MS       = 100;
constexpr uint32_t ALIGN_TIMEOUT_MS    = 1500;

constexpr float PIVOT_CORNER_RADIUS_SQ_MM2 =
    ROBOT_HALF_LENGTH_MM * ROBOT_HALF_LENGTH_MM + ROBOT_HALF_WIDTH_MM * ROBOT_HALF_WIDTH_MM;
constexpr float PIVOT_LATERAL_LIMIT_MM = 12.0f;
constexpr float RECENTRE_ANGLE_DEG     = 20.0f;
constexpr float RECENTRE_MAX_OFFSET_MM = 30.0f;
constexpr bool  PIVOT_RECENTRE         = true;
constexpr uint32_t RECENTRE_READING_MAX_AGE_MS = 100;
static_assert(PIVOT_CORNER_RADIUS_SQ_MM2 < (HALF_CELL_INNER_MM * (1.0f - 0.05f) - PIVOT_LATERAL_LIMIT_MM) *
                                           (HALF_CELL_INNER_MM * (1.0f - 0.05f) - PIVOT_LATERAL_LIMIT_MM),
              "robot too large to pivot in a -5 % cell with this lateral limit");
constexpr uint32_t STUCK_MS            = 400;
constexpr float STUCK_MIN_RANGE_CHANGE_MM = 5.0f;
constexpr float STUCK_MAX_RATE_DPS     = 3.0f;
constexpr float STUCK_MIN_VOLTS        = 2.0f;

constexpr float STUCK_MIN_WHEEL_MM_S   = 10.0f;

constexpr float CELL_TOLERANCE_FRAC    = 0.05f;

constexpr float EST_Q_MM2_PER_MM       = 1.3f;

constexpr float EST_Q_ENC_MM2_PER_MM   = 0.25f;

constexpr float EST_FRONT_R_MM2        = 64.0f;
constexpr float EST_EDGE_R_MM2         = 144.0f;

constexpr float EST_GATE_MIN_MM        = 20.0f;
constexpr float EST_GATE_SIGMAS        = 3.0f;

constexpr float EST_FRONT_TRUST_DEG    = 10.0f;

constexpr uint8_t EST_REJECT_RUN       = 4;
constexpr float EST_REJECT_INFLATE_MM2 = 100.0f;

constexpr float EST_SCALE_ADAPT_GAIN   = 0.2f;
constexpr float EST_SCALE_MIN          = 0.8f;
constexpr float EST_SCALE_MAX          = 1.25f;
constexpr float EST_SCALE_MIN_DIST_MM  = 150.0f;

constexpr float EST_SCALE_ADAPT_GAIN_EDGE = 0.1f;

constexpr bool  EST_LATTICE_FRONT_FIX  = true;
constexpr float EST_LATTICE_R_MM2      = 100.0f;

constexpr float POST_HALF_MM           = 6.0f;

constexpr float EDGE_WALL_TO_GAP_MM    = 10.0f;
constexpr float EDGE_GAP_TO_WALL_MM    = 10.0f;
constexpr float EDGE_MAX_SPAN_MM       = 40.0f;
constexpr float EDGE_GATE_MM           = 60.0f;

constexpr float PARALLEL_SPAN_MM       = CELL_PITCH_MM;
constexpr int   PARALLEL_MIN_SAMPLES   = 5;
constexpr float PARALLEL_MAX_SLOPE     = 0.035f;

constexpr float PARALLEL_MAX_FIX_DEG   = 10.0f;

constexpr float CONF_GOOD_SIGMA_MM     = 25.0f;
constexpr float CONF_LOST_SIGMA_MM     = 60.0f;
constexpr float CONF_LOW_SPEED_MM_S    = 250.0f;
constexpr float START_SIGMA_MM         = 10.0f;
constexpr float ALIGNED_SIGMA_MM       = 4.0f;

constexpr float START_OFFSET_MM        = 0.0f;

constexpr uint32_t MATCH_DURATION_MS   = 8u * 60u * 1000u;

constexpr float    MATCH_TIME_FACTOR   = 1.5f;
constexpr uint32_t MATCH_END_MARGIN_MS = 10000;

constexpr float    TRIGGER_NEAR_MM     = 80.0f;
constexpr uint32_t TRIGGER_HOLD_MS     = 500;
constexpr float    TRIGGER_CLEAR_MM    = 150.0f;
constexpr uint32_t TRIGGER_DELAY_MS    = 1000;

constexpr bool     AUTO_RESTART        = true;
constexpr uint32_t START_PAUSE_MS      = 1000;

constexpr uint32_t STATIONARY_READ_MS  = 200;

constexpr float    MATCH_SEARCH_EST_S  = 90.0f;
constexpr uint32_t BIAS_WAIT_MAX_MS    = 1500;

constexpr int      CLEAN_LOOPS_BEFORE_HERO = 2;

constexpr float    CONFLICT_SIGMA_MM   = 20.0f;
constexpr int      CONFLICTS_STOP      = 3;

constexpr bool     SPEEDRUN_WALL_CHECK = true;

constexpr bool     NVS_MIRROR_EVERY_CELL = true;
constexpr uint32_t BLOCKING_SECTION_MAX_MS = 80;

constexpr bool EXPLORE_WALL_HUG   = false;

constexpr bool WALL_HUG_LEFT_HAND = true;

constexpr float PLAN_TURN_SETTLE_S = 0.15f;
constexpr float PLAN_SEGMENT_S     = 0.15f;

constexpr float PLAN_EXPLORE_GAIN  = 0.10f;

constexpr int   CAL_STEP_COUNT            = 7;
constexpr float CAL_STEP_V[CAL_STEP_COUNT] = {1.0f, 1.5f, 2.0f, 2.5f, 3.0f, 3.5f, 4.0f};
constexpr float CAL_START_MIN_MM          = 450.0f;
constexpr float CAL_START_MAX_MM          = 900.0f;
constexpr float CAL_START_MM              = 600.0f;
constexpr uint32_t CAL_REVERSE_TIMEOUT_MS = 4000;
constexpr float CAL_STOP_MM               = 140.0f;
constexpr float CAL_REVERSE_V             = 1.8f;
constexpr uint32_t CAL_SETTLE_MS          = 150;
constexpr uint32_t CAL_STEP_TIMEOUT_MS    = 3000;
constexpr uint32_t CAL_NO_MOTION_MS       = 1200;
constexpr float CAL_NO_MOTION_MM          = 15.0f;
constexpr int   CAL_MIN_SAMPLES           = 5;
constexpr float CAL_MIN_SPEED_MM_S        = 20.0f;
constexpr float CAL_MAX_SPEED_MM_S        = 900.0f;
constexpr int   CAL_MIN_POINTS            = 3;

#define NVS_CAL_NAMESPACE "mmrc_cal"
#define NVS_MAP_NAMESPACE "mmrc_map"

#ifdef ARDUINO
  #if MMRC_DEBUG
    #define DBG_BEGIN(baud)  do { Serial.setTxBufferSize(4096); Serial.begin(baud); } while (0)
    #define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
  #else
    #define DBG_BEGIN(baud)  do { (void)(baud); } while (0)
    #define DBG_PRINTF(...)  do { if (0) { (void)snprintf(nullptr, 0, __VA_ARGS__); } } while (0)
  #endif
#endif
