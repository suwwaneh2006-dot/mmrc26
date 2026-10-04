# MMRC26 micromouse firmware: handoff

## Layout

| Path | Contents |
|---|---|
| `main/` | Arduino sketch (ESP32, LOLIN32 Lite, esp32 core 3.x): `config.h`, `motors.*`, `encoders.*`, `sonar.*`, `imu.*`, `battery.*`, `ui.*`, `sched.*`, `estimator.*`, `motion.*`, `maze.*`, `strategy.*`, `modes.*`, `main.ino` |
| `sim/` | maze-brain tests and the mms simulator mouse (`build_mms.bat` → `mouse.exe`, `run_tests.bat`) |
| `sim/robot/` | closed-loop simulator of the complete firmware (`build_robot_sim.bat`, `robot_sim.exe`, `run_matrix.ps1`) |
| `BRINGUP.md` | hardware test checklist; **out of date** (written before the encoders, still mentions the LED) |

## Current state

- **Exploring method:** flood fill (`EXPLORE_WALL_HUG = false`). Wall hugging is still in the code but can't reach an island goal: it reached 3 of 20 simulated island mazes.
- **Encoders:** N20 hall, one channel per wheel. Left on GPIO 5, right on GPIO 22 (the on-board LED pin is reused, so error codes are beeped).
  - Encoder odometry plus sonar corrections (front wall, any wall ahead, post edges).
  - Closed-loop forward speed. Pivot wheel feedback is off (`PIVOT_WHEEL_FEEDBACK = false`), because the gyro alone settles faster.
  - Stuck detection from the wheels.
  - Automatic fallback to the voltage model if an encoder is silent while driven. A wheel is left out after 60 ms and declared faulty after 300 ms; a fault clears itself once the encoder counts again.
- **Build:** ESP32 compiles with zero warnings, `MMRC_DEBUG` 1 and 0.
- **Verified in simulation:**
  - **Brain:** 590 mazefiles mazes and 20 generated 10×10 island mazes pass (normal and mirrored); 10 skipped as unreachable in the file.
  - **Full-firmware matches, normal conditions:** 60/60 pass, 0 crashes.
  - **Harsh conditions:** 57/60 pass, 3 crashes (maze 11 mirrored ×0.85, 14 ×1.15, 18 mirrored ×0.85). Not investigated.
  - **Encoder failures:** both dead from power-on, and one dying mid-match, both pass with 0 aborts.
  - **Best simulated score** on `sim/mazes/mmrc26-island-10x10.txt`: about 2440 (14 runs, 14 returns, best 14.35 s).
- **Not hardware-tested:** nothing has run on the real robot yet.

## Chassis (measured 2026-10-04)

- **Size:** 126 mm long with sensors; 122 mm wide with wheels (88 mm without); front 54 mm wide; 98 mm tall.
- **Axle:** about 27 mm from the back, so the nose is 99 mm ahead of it. A pivot swings about 103 mm, more than the 85–94 mm a cell allows. `ROBOT_MAZE_READY` is false and **mode 7 is locked** until the axle moves towards the middle. Modes 1–6 work.
- **Encoders:** run on 5 V, with 10k/15k dividers on GPIO 5 and 22.
- **Track:** `WHEEL_TRACK_MM` = 105 is an estimate; measure it.

## Open items

1. **Robot measurements** (the user will provide them). Fill in every `TODO_MEASURE` in `main/config.h`:
   - wheel diameter, track,
   - nose, tail and half-width,
   - sonar positions,
   - `ENC_EDGES_PER_WHEEL_REV`, `IMU_Z_SIGN`, `BATT_CAL_FACTOR`, `BUZZER_ACTIVE`,
   - `START_OFFSET_MM`, `SONAR_AIR_TEMP_C`, edge offsets.

   Then recompile; `static_assert`s catch a robot too big to pivot or a duplicated GPIO.
2. **Encoder supply voltage unknown.** If the encoders run on 5 V, each output needs a 10k/15k divider to 3.3 V before GPIO 5/22.
3. **Comments removed** from every code file and build script. The ESP32 binary is the same size as before, and all simulated tests still pass.
4. **Harsh crashes (3 of 60), cause partly found:** under harsh gyro noise the firmware's heading picks up a jump of about 2° during some pivots, which builds up to about 5°. Wall centring and that heading error then cancel out, and the robot clips a wall at the next pivot. Not fixed. `PARALLEL_MAX_FIX_DEG` was raised to 10, which did not remove these crashes. Reproduce with `robot_sim.exe ..\mazes\generated\mmrc26-island-10x10-14.txt --seed 1098 --model-error 1.15 --harsh`.
5. **`BRINGUP.md`:** update it for the encoders and beeped error codes, or delete it per the no-notes request.
6. **Hardware bring-up order:** modes 1 → 2 (check that the encoders count in the right direction) → 5 → 3 → 4 → 6 → 7 on a small maze → full maze.
7. **Match day:** set `MMRC_DEBUG 0`, wipe the map (hold the button at power-on), check the mirror setting, battery above 7.8 V, and ask the judges about `AUTO_RESTART`.

## Repository

https://github.com/suwwaneh2006-dot/mmrc26 (branch `main`)
