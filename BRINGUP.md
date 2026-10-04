# MMRC26 bring-up checklist

Flash with the Arduino IDE: board **WEMOS LOLIN32 Lite**, esp32 core 3.x.
Keep Serial Monitor open at **115200**. With no buzzer and no LED, Serial is
the only feedback, so keep `MMRC_DEBUG 1`.

## The "button" is the front sonar

There is no physical button. While the motors are off, the front sonar acts
as one:

| Action | What to do |
|---|---|
| **Click** | move a hand to under 5 cm in front of the robot, then away again. It only counts after the sensor has seen open space (over 10 cm) first, so a wall in front never counts as a press |
| **Long press** | hold the hand there for 1.5 s |
| **Choose a mode** | click N times; Serial prints `mode N` |
| **Start a test** | one more click; you then get 1 s to step back |
| **Cancel** | long press |
| **Wipe the map** | long press in the main menu (`map wiped.`). Do this before every match |
| **Start a match** | hold a hand under 8 cm for about 1 s, then remove it |
| **Leave match mode** | hold for 3 s |
| **Rescue after an abort** | put the robot in the start cell, then click |

## 1. Wiring

| Signal | GPIO | Notes |
|---|---|---|
| I2C SDA / SCL | 19 / 23 | MPU6050 on 3.3 V |
| Left motor PWMA / AIN1 / AIN2 | 25 / 26 / 27 | |
| Right motor PWMB / BIN1 / BIN2 | 32 / 33 / 13 | |
| TB6612 STBY | 4 | LOW = motors off |
| TRIG front / left / right | 16 / 17 / 18 | |
| ECHO front / left / right | 35 / 36 / 39 | each: ECHO–10k–GPIO–15k–GND |
| Encoder A left / right (5 V) | 5 / 22 | each: A–10k–GPIO–15k–GND; channel B unused |
| Battery sense | 34 | Bat+–20k–GPIO34–10k–GND |

**Never connect a 5 V signal straight to an ESP32 pin.**

**Pass:** the robot waits 1 s at power-on (keep it still for about 3 s in
total). Serial then shows all four parts `ok` and `ready: click 1-8`.

## 2. Mode 1: sensors (1 click)

**Do:**
- Hold a hand at 70, 100 and 200 mm in front of each sonar. Stay above 5 cm,
  or the front sonar reads it as a click.
- Turn the robot left by hand: `hdg` must rise. Mode 8 also fixes this
  automatically.

## 3. Mode 2: motors (wheels in the air, 2 clicks)

**Pass:** 5 phases. The driven wheel shows `encoders: ... ok`.

A reversed motor shows `CHECK`. Mode 8 corrects it automatically.

## 4. Mode 8: full auto-calibration (8 clicks)

**Do:** put the robot on open floor (it spins in place), facing a flat wall
250–800 mm away, with room behind it.

**What it does,** each step checked before the next:
1. finds which motor is wired backwards,
2. finds the gyro sign,
3. squares up on the wall,
4. **forward:** mm per encoder edge, each wheel,
5. **right:** clockwise gyro scale,
6. **left:** counter-clockwise gyro scale,
7. **back:** mm per encoder edge, each wheel, in reverse,
8. **track width**,
9. **motor model:** dead-band, speed table and lag, by spinning in place.

**Pass:** Serial prints `saved: wiring ... track ...`. The values are stored
and used on every boot.

**If it fails:** it names the failed step and saves nothing. Retry with a
flatter, wider wall and more open floor.

## 5. Modes 3, 6 and 4

| Mode | Pass | If not |
|---|---|---|
| 3: straight 5 cells | stops about 960 mm on, centred | weaving: lower `HEADING_KP_V_PER_DEG`; swinging across the corridor: lower `CENTER_KP_DEG_PER_MM` |
| 6: one cell | `s_est` ends at about 192 | |
| 4: four 90° pivots, **open floor** | 4 × `done`, error under 2° | overshoot: lower `PIVOT_KP_MM_S_PER_DEG` |

## 6. Mode 7: match (7 clicks)

1. **Wipe the map:** long press in the menu.
2. **Choose the mirror setting:** click to toggle (Serial shows normal or
   MIRRORED), then long press to confirm.
3. **Start:** hand-wave (under 8 cm for about 1 s, then remove).

Match speed is capped at T1/T2 (`MATCH_MAX_TIER = 1`). Raise it to 3 only
after clean runs.

### Match day
1. Keep `MMRC_DEBUG 1`; Serial is the only feedback.
2. Battery **above 7.8 V**.
3. Wipe the map.
4. Check the mirror setting.
5. Ask the judges about `AUTO_RESTART`.

## Still to measure

- **Axle position:** `ROBOT_NOSE_X_MM` / `ROBOT_TAIL_X_MM` assume the axle is
  centred (63 / 63).
- **Side sonar position:** assumed 45 mm ahead of the axle, 27 mm to each
  side.
- **Track width:** mode 8 measures it, so nothing to do.
