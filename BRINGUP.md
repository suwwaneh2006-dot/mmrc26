# MMRC26 bring-up checklist

Work through the steps in order. Re-flash after every `main/config.h` change:
Arduino IDE, board **WEMOS LOLIN32 Lite**, esp32 core 3.x, Serial Monitor at
**115200**.

**Choosing a mode:** click the button N times; the robot beeps N back. For
modes 2–6, a short press starts and a long press cancels; you then get 1 s to
take your hand away. A short press during a test aborts it.

> **Chassis limit: match mode (7) is locked.** With the axle 27 mm from the
> back, the nose is 99 mm ahead of it. A pivot then swings the front corners
> out to about 103 mm, but a cell only gives 85–94 mm from its centre to a
> wall. Mode 7 refuses with 3 low beeps. **Fix:** move the wheels/axle towards
> the middle (axle about 60 mm from the back). Then update `ROBOT_NOSE_X_MM` /
> `ROBOT_TAIL_X_MM` and mode 7 unlocks by itself. Modes 1–6 work now.

## 1. Wiring

| Signal | GPIO | Notes |
|---|---|---|
| I2C SDA / SCL | 19 / 23 | MPU6050 on 3.3 V |
| Left motor PWMA / AIN1 / AIN2 | 25 / 26 / 27 | |
| Right motor PWMB / BIN1 / BIN2 | 32 / 33 / 13 | |
| TB6612 STBY | 4 | LOW = motors off; 10k pull-down to GND recommended |
| TRIG front / left / right | 16 / 17 / 18 | direct |
| ECHO front / left / right | 35 / 36 / 39 | **each: ECHO–10k–GPIO–15k–GND** |
| Left encoder channel A | 5 | **5 V encoder: A–10k–GPIO–15k–GND** |
| Right encoder channel A | 22 | **5 V encoder: A–10k–GPIO–15k–GND** (the on-board LED flickers; harmless) |
| Encoder VCC / GND | 5 V buck / GND | channel B unused |
| Battery sense | 34 | **Bat+–20k–GPIO34–10k–GND** |
| Button | 14 | to GND |
| Buzzer | 15 | |

**Never connect a 5 V signal straight to an ESP32 pin.** Power is 2S pack →
switch → TB6612 VM, and buck 5 V → board 5V, sonars and encoders, with common
GND everywhere.

**Pass:** at power-on, keep the robot still for 2 s. You should hear **4
beeps** (3 sonars + IMU) and see `ready: click 1-7` in Serial.

**If it fails:** the error code is beeped as N long low beeps, repeated:
- 1 / 2 / 3: front / left / right sonar. Check 5 V, TRIG and the echo divider.
- 4: IMU. Check SDA/SCL and 3.3 V.
- 5: battery below 6.6 V.
- 6: motor fault.

## 2. Mode 1: sensors

**Do:**
- Hold a hand at 50, 100 and 200 mm in front of each sonar.
- Turn the robot **left** by hand.
- Turn one wheel exactly **10 turns** by hand.

**Pass:**
- Ranges within ±5 mm of the ruler.
- `hdg` increases when turning left.
- The encoder edge count of the turned wheel rises.
- The battery voltage matches a multimeter.

**If it fails:**

| Symptom | Change in `config.h` |
|---|---|
| `hdg` decreases when turning left | `IMU_Z_SIGN = -1.0f` |
| Battery differs from the multimeter | `BATT_CAL_FACTOR = multimeter / printed` |
| Encoder edges per turn | `ENC_EDGES_PER_WHEEL_REV = (edges after 10 turns) / 10` |
| Encoder count doesn't move | check the encoder 5 V, GND and the divider |
| A sonar jumps | raise `SONAR_GAP_MS` (12 → 20) |
| Buzzer silent or clicking | flip `BUZZER_ACTIVE` |

## 3. Mode 2: motors (wheels in the air)

**Pass:** 5 phases (L fwd, L back, R fwd, R back, both fwd). Each prints
`encoders: ... ok` for the driven wheel.

**If it fails:**

| Symptom | Fix |
|---|---|
| Wheel spins the wrong way | flip `MOTOR_L_INVERT` / `MOTOR_R_INVERT` |
| The wrong wheel moves | swap the motor connectors |
| `CHECK` on a wheel that turned | that encoder's wiring or pin |

## 4. Mode 5: motor model

**Do:** put the robot in a straight corridor facing a wall 450–900 mm away.

**Pass:** 5 rapid beeps (`saved to NVS`).

## 5. Mode 3: straight 5 cells

**Pass:** it stops 960 mm ± 15 mm along and stays centred.

**If it fails:**

| Symptom | Change |
|---|---|
| Distance off | `WHEEL_DIAMETER_MM *= ruler / encoder mm` (both printed) |
| Weaves | lower `HEADING_KP_V_PER_DEG` |
| Drifts to one side | raise `HEADING_KI_V_PER_DEG_S` |
| Swings across the corridor | lower `CENTER_KP_DEG_PER_MM` |
| Speed hunting | lower `SPEED_KP_V_PER_MM_S` |

## 6. Mode 4: pivot 4 × 90° (open table only, never inside the maze)

**Pass:** each turn shows `done`, error under 2°, and the robot ends facing
where it started.

**If it fails:**
- Off by X° over 360°: `IMU_GYRO_SCALE *= 360 / (360 + X)`.
- Overshoot or oscillation: lower `PIVOT_KP_MM_S_PER_DEG`.
- Also measure `WHEEL_TRACK_MM` (wheel centre to wheel centre). The current
  value, 105, is an estimate.

## 7. Mode 6: one-cell step

**Pass:** `s_est` ends at about 192 and the CSV shows a smooth speed curve.

## 8. Mode 7: match (after the axle fix)

**Do:**
1. Wipe the map: hold the button at power-on until the long beep.
2. Click 7. One beep = normal, two = mirrored; short press toggles, long
   press confirms.
3. Hand under 8 cm in front for half a second, then take it away.

**Pass:** search → goal → return → speed runs, with no wall contact.

### Match day
1. Set `MMRC_DEBUG 0`.
2. Battery **above 7.8 V**.
3. Wipe the map.
4. Check the mirror setting.
5. Ask the judges about `AUTO_RESTART`.

### Sounds
| Sound | Meaning |
|---|---|
| N short beeps | a number: working parts at boot, selected mode, mirror (1/2) |
| 1 short high | OK, or start trigger seen |
| 3 long low | refused / error (the code follows) |
| 2 long low | aborted: waiting for rescue (place the robot at the start, short press, hand-wave) |
| 1 very long | map wiped |
| 5 rapid | calibration saved |
| 2 rapid low | battery below 7.0 V |
| 3 short | match time over |
