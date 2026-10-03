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

## Open items

1. **Robot measurements** (the user will provide them). Fill in every `TODO_MEASURE` in `main/config.h`:
   - wheel diameter, track,
   - nose, tail and half-width,
   - sonar positions,
   - `ENC_EDGES_PER_WHEEL_REV`, `IMU_Z_SIGN`, `BATT_CAL_FACTOR`, `BUZZER_ACTIVE`,
   - `START_OFFSET_MM`, `SONAR_AIR_TEMP_C`, edge offsets.

   Then recompile; `static_assert`s catch a robot too big to pivot or a duplicated GPIO.
2. **Encoder supply voltage unknown.** If the encoders run on 5 V, each output needs a 10k/15k divider to 3.3 V before GPIO 5/22.
3. **Comment removal requested by the user:** remove all comments and notes from every code copy. A string-aware stripper exists at the session scratchpad (`strip.cpp`) but has not been run. After stripping, recompile the ESP32 (both debug modes), `build_mms.bat` and `build_robot_sim.bat`, and rerun one simulated match to prove nothing changed.
4. **Harsh crashes:** optionally investigate the 3 remaining crashes, with `robot_sim.exe <maze> --seed N --model-error F --harsh [--mirror] --trace FROM TO`.
5. **`BRINGUP.md`:** update it for the encoders and beeped error codes, or delete it per the no-notes request.
6. **Hardware bring-up order:** modes 1 → 2 (check that the encoders count in the right direction) → 5 → 3 → 4 → 6 → 7 on a small maze → full maze.
7. **Match day:** set `MMRC_DEBUG 0`, wipe the map (hold the button at power-on), check the mirror setting, battery above 7.8 V, and ask the judges about `AUTO_RESTART`.

## Not a git repository

`mmrc26/` isn't under version control yet. To start, run `git init` and commit the current state.
