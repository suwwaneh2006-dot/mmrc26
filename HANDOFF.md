# MMRC26 micromouse firmware: handoff

Repository: https://github.com/suwwaneh2006-dot/mmrc26 (branch `main`)

## Layout

| Path | Contents |
|---|---|
| `main/` | Arduino sketch (ESP32 LOLIN32 Lite, esp32 core 3.x): `config.h`, `motors`, `encoders`, `sonar`, `imu`, `battery`, `ui`, `sched`, `calib`, `estimator`, `motion`, `maze`, `strategy`, `modes`, `main.ino` |
| `sim/` | maze-brain batch tests (`run_tests.bat`) and the mms simulator mouse (`build_mms.bat`) |
| `sim/robot/` | full-firmware simulator (`build_robot_sim.bat`, `robot_sim.exe`, `run_matrix.ps1`) |
| `BRINGUP.md` | wiring and test procedure |

There are no comments anywhere in the code, as requested.

## Hardware assumed

- **Chassis:** 126 mm long, 122 mm wide with wheels, 88 mm body, 54 mm wide at the front.
- **Axle:** assumed centred (nose and tail 63 mm). Pivoting inside a cell was verified on the real robot: `ROBOT_PIVOT_VERIFIED`.
- **Sensors:** front sonar at the nose; side sonars 45 mm ahead of the axle, 27 mm to each side.
- **Encoders:** N20 hall, 5 V, channel A only, on GPIO 5 / 22 through 10k/15k dividers.
- **Not fitted:** no buzzer, no button, no power switch. The front sonar acts as the button, and all feedback is over Serial.
- **Battery:** already calibrated.

## Firmware state

- **Maze brain:** flood fill (`EXPLORE_WALL_HUG = false`); speed runs take the fastest path on verified walls only.
- **Localisation:** encoder odometry, corrected by the sonar against the front wall, any wall straight ahead, and post edges. If an encoder fails, the motor model takes over (wheel suspect after 60 ms, fault after 300 ms, automatic recovery).
- **Safety:**
  - every stop in front of a wall keeps the nose at least 15 mm away (collision guard margin 6 mm);
  - before every pivot: fresh readings, front back-off if needed, re-centring if more than 5 mm off-centre;
  - match speed capped at T1/T2.
- **Mode 8 (full auto-calibration):** motor wiring, gyro sign, mm per edge for each wheel and direction, CW and CCW gyro scale, track width, and the motor model. Saved in NVS and applied at every boot.
- **Boot:** STBY is pulled low first, then a 1000 ms delay (`BOOT_DELAY_MS`).
- **Builds:** ESP32 with zero warnings in both `MMRC_DEBUG` modes.

## Verified in simulation (real chassis shape)

- **Maze brain:** 590/590 maze files pass, normal and mirrored.
- **Auto-calibration:** passes with normal wiring and with a reversed left motor plus a flipped gyro; values within about 0–2% of the truth; no wall contact.
- **Full matches (normal, mirrored with slow motors, dead encoders):** 0 aborts, 0 crashes, 0 wrong walls.
- **60-match matrices:** see the final commit message for the normal and harsh results.

## Next steps

Work through `BRINGUP.md`:
1. wiring,
2. mode 1, then mode 2,
3. **mode 8**,
4. modes 3, 6, 4,
5. mode 7.
