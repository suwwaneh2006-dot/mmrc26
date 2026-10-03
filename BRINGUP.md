# MMRC26 bring-up checklist

Work through the steps in order. Each step lists what to wire, which test mode
to run, what a pass looks like, and which `main/config.h` values to change if
it fails. Don't move on until the current step passes.

Re-flash after every `config.h` change: Arduino IDE, board **WEMOS LOLIN32 Lite**,
esp32 core 3.x, Serial Monitor at **115200**. Keep `MMRC_DEBUG 1` until the
match day (step 12).

**Choosing a mode:** click the button N times. The robot beeps N back. For
modes 2–6, a short press starts and a long press cancels. You then get 1 s to
take your hand away. A short press during a test aborts it.

---

## 0. Before powering anything: measure the robot

Write these into `config.h`. They are the `TODO_MEASURE` values:

| Value | How |
|---|---|
| `WHEEL_DIAMETER_MM` | calipers across the tyre |
| `WHEEL_TRACK_MM` | centre of left tyre contact to centre of right tyre contact |
| `ROBOT_NOSE_X_MM`, `ROBOT_TAIL_X_MM` | axle centre to the very front / very back |
| `ROBOT_HALF_WIDTH_MM` | half the widest point, wheels included |
| `SONAR_*_X_MM`, `SONAR_*_Y_MM`, `SONAR_*_ANGLE_DEG` | transducer face position relative to the axle centre (x forward, y left) |
| `SONAR_AIR_TEMP_C` | thermometer at the venue (re-check on match day) |

If the robot is too big to pivot in a 171 mm cell, the build fails with
*"robot too large to pivot"*. A compile error is a hardware problem, not a
code problem.

## 1. Power and board

**Wire:** 2S pack → XT30 → switch → TB6612 VM, and buck (5 V) → board 5V pin and
sonar VCC. Common GND everywhere. **Don't connect the motors yet.**

| Signal | GPIO | Notes |
|---|---|---|
| I2C SDA / SCL | 19 / 23 | MPU6050 on 3.3 V |
| Left motor PWMA / AIN1 / AIN2 | 25 / 26 / 27 | |
| Right motor PWMB / BIN1 / BIN2 | 32 / 33 / 13 | |
| TB6612 STBY | 4 | LOW = motors off. A 10k pull-down to GND is recommended |
| TRIG front / left / right | 16 / 17 / 18 | direct, 3.3 V is enough for HC-SR04 |
| ECHO front / left / right | 35 / 36 / 39 | **through a divider each: ECHO–10k–GPIO–15k–GND** (5 V → 3.0 V) |
| Battery sense | 34 | **Bat+–20k–GPIO34–10k–GND** |
| Button | 14 | to GND (internal pull-up) |
| Status LED | 22 | on-board, active LOW |
| Buzzer | 15 | |

**Pass:** at power-on the LED lights for about 1 s (gyro calibration; keep the
robot still). Serial shows `MMRC26 firmware`, a self-check list, and
`ready: click 1-7`.

**If it resets in a loop:** check the buck output under load, and that ECHO
pins never see 5 V.

## 2. Self-check (boot beeps)

**Pass:** 4 beeps (3 sonars + IMU), Serial shows `ok` for all four, and the LED
blinks slowly.

**If fewer beeps:** the LED blinks an error code. 1 = front sonar, 2 = left,
3 = right, 4 = IMU, 5 = battery below 6.6 V, 6 = motor fault.
- Sonar `MISSING`: check VCC 5 V, the TRIG wire, and the echo divider. With the
  divider wired backwards the ECHO pin never rises.
- IMU `MISSING`: check SDA/SCL and 3.3 V. `WHO_AM_I` other than 0x68 is OK on
  clones.
- Buzzer silent or clicking: flip `BUZZER_ACTIVE`.

## 3. Mode 1: sensor dump

**Do:**
- Hold a hand or box in front of each sonar at 50, 100 and 200 mm.
- Turn the robot **left** by hand.
- Measure the pack with a multimeter.

**Pass:**
- Each range is within ±5 mm of the ruler.
- `W` appears below about 120 mm on the sides and about 130 mm in front
  (with the default sensor offsets).
- `NW` appears when nothing is in range.
- `hdg` increases when turning left.
- The `[loop] overruns` count stays at 0.

**If it fails:**

| Symptom | Change |
|---|---|
| `hdg` decreases when turning left | `IMU_Z_SIGN = -1.0f` |
| Battery differs from the multimeter | `BATT_CAL_FACTOR = multimeter / printed` |
| Ranges consistently off by a factor | check `SONAR_AIR_TEMP_C` |
| One sensor jumps or reads 20 mm | crosstalk or a loose echo divider; raise `SONAR_GAP_MS` (12 → 20) |

## 4. Mode 2: motor directions (wheels in the air)

**Wire:** the motors to the TB6612 now.

**Pass:** phase N is announced with N beeps:
1. left wheel forward
2. left wheel backward
3. right wheel forward
4. right wheel backward
5. both forward

**If it fails:**
- A wheel spins the wrong way: set its `MOTOR_L_INVERT` / `MOTOR_R_INVERT`.
- The wrong wheel moves: swap the motor connectors (A ↔ B).

## 5. Mode 5: velocity model (do this before modes 3, 4 and 6)

**Do:** put the robot in a straight corridor facing a wall 450–900 mm away
(about 3 cells), with clear floor behind it.

**Pass:**
- It drives forward and backs up 7 times at rising voltages.
- It prints `x V -> y mm/s` lines and `model: tau ...`.
- It beeps 5 rapid beeps (`saved to NVS`).
- After a reboot, the boot line says `model: calibrated (NVS)`.

**If it fails:**
- `outside the start window`: move the robot.
- `only N usable points`: the floor is too slick or the wall too far. Raise
  `CAL_STEP_V` values, or check that the front sonar sees the wall in mode 1.

Note the **dead-band volts** it prints. You need it below.

## 6. Mode 3: straight 5 cells at T1

**Do:** start centred in a cell of a 5-cell corridor.

**Pass:**
- It stops 5 cells on (960 mm ± 15 mm, measured with a ruler).
- `heading error at end` is below 2°.
- It stays centred in the corridor without weaving.

**If it fails:**

| Symptom | Change |
|---|---|
| Weaves left-right | lower `HEADING_KP_V_PER_DEG` or raise `HEADING_KD_V_PER_DPS` |
| Slowly drifts to one side | raise `HEADING_KI_V_PER_DEG_S` |
| Swings across the corridor | lower `CENTER_KP_DEG_PER_MM` |
| Stays off-centre | raise `CENTER_KP_DEG_PER_MM` |
| Stalls just short of the end | raise `PROFILE_MIN_SPEED_MM_S` to the speed of the lowest model point |
| `stuck` abort | set `STUCK_MIN_VOLTS` to about 2× the dead-band volts |
| `collision guard` while stopping near a wall | robot can't brake as hard as assumed: lower `BRAKE_DECEL_MM_S2` and the tier speeds |

## 7. Mode 4: pivot 4 × 90° left

**Pass:**
- Each `turn N` shows `done` with a gyro error under 2°.
- After 4 turns the robot faces where it started, within about 3° by eye.

**If it fails:**

| Symptom | Change |
|---|---|
| Visibly over- or under-rotates X° per full turn | `IMU_GYRO_SCALE *= 360 / (360 + X)` (X positive = over-rotated) |
| Overshoot or oscillation at the end | lower `PIVOT_KP_MM_S_PER_DEG`, raise `PIVOT_KD_MM_S_PER_DPS` |
| Stops short and creeps | raise `PIVOT_KP_MM_S_PER_DEG` |
| `timeout` | as above; raise `TURN_TIMEOUT_MS` only if the turns look good |

Then set `PLAN_TURN_SETTLE_S` to the printed turn time minus the profile time.
This only affects which speed path gets chosen.

## 8. Mode 6: one-cell step + post edges

**Do:** run it with a side wall that **ends** halfway (wall → gap), then copy
the CSV into a spreadsheet.

**Pass:**
- `s_est` ends at about 192, and the front sonar moved about 192 mm.
- `sigma` stays below 25.

**Tuning:**
- `EDGE_WALL_TO_GAP_MM` / `EDGE_GAP_TO_WALL_MM`: find the row where
  `left_raw` / `right_raw` jumps from about 60 to 251. The value is
  `s_est` at that row + `SONAR_*_X_MM` − (96 + 6).
- `PLAN_SEGMENT_S`: the time between v_ref reaching 0 and the robot standing
  still.

## 9. Mode 7 on a small test maze (T1 and T2 only)

**Do:**
1. Build a few cells including the start corner.
2. **Wipe the map:** hold the button while switching on, until the long beep.
3. Click 7. One beep means normal, two means mirrored; short press toggles,
   long press confirms.
4. Hold your hand in front of the robot (under 8 cm) for half a second, then
   take it away.

**Pass:**
- It searches, reaches the goal, turns 180° and returns.
- It aligns in the start cell, turns, and starts a speed run.
- No wall contact.
- No `ABORT` lines in Serial. Every `cell (x,y) ... walls` line matches the
  real walls.

**If it fails:**

| Log line | Meaning / change |
|---|---|
| `ABORT: position lost` | raise `EST_Q_MM2_PER_MM` a little. If front fixes are rejected, check the `SONAR_F_X_MM` measurement |
| `conflict:` during speed runs | false side-wall reads at speed; set `SPEEDRUN_WALL_CHECK = false` |
| Wrong `walls` in a cell | check `SIDE_WALL_ON/OFF_MARGIN_MM` against the mode 1 readings |
| `re-centring:` very often | centring is too weak (step 6) |
| Wall touches during pivots | lower `PIVOT_LATERAL_LIMIT_MM` |
| `loop overrun` fault | set `NVS_MIRROR_EVERY_CELL = false` |

## 10. Mode 7 on a full maze

**Pass:** several loops, tier T2 → T3 → T4 according to the policy, no contact.
If T3/T4 touch walls, lower `TIERS[2]` / `TIERS[3]` speeds. The tier policy
drops a tier after any abort anyway.

## 11. Rescue drill

During a run, pick the robot up (it aborts). Put it in the start cell and
short-press. Then hand-wave: it must continue with the map it has learned.

## 12. Match day

- Set `MMRC_DEBUG 0` and re-flash (no Serial code in the competition build).
- Re-measure `SONAR_AIR_TEMP_C`.
- Ask the judges whether automatic restart is allowed. If not, set
  `AUTO_RESTART = false`, and the robot then waits for a hand-wave between
  loops.

### Pre-match checklist
1. Battery **> 7.8 V** (mode 1, or the multimeter).
2. **Wipe the map**: hold the button at power-on until the long beep.
3. Robot still for about 2 s at power-on: **4 beeps**.
4. Click 7. **Check the mirror setting** (1 beep = maze extends right, 2 =
   extends left), then long press.
5. Place it centred in the start cell, facing out (`START_OFFSET_MM` assumes
   centred).
6. Hand-wave to start.

### Beep meanings
| Sound | Meaning |
|---|---|
| N short beeps | a number: working parts at boot, selected mode, mirror (1/2) |
| 1 short high | OK / confirmed / start trigger accepted |
| 3 long low | error or refused (the LED shows the code) |
| 2 long low | aborted (touch, collision guard, stuck, lost, fault): waiting for rescue |
| 1 very long | map wiped |
| 5 rapid | calibration saved |
| 2 rapid low | battery below 7.0 V |
| 3 short (LED steady on) | match time used up |

### LED
- Slow blink: waiting.
- On: hand detected.
- Fast blink: about to move, or waiting for rescue.
- N blinks + pause: error code (see step 2).
