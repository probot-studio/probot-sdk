# QEMU speed benchmark (QemuBench)

`test/target/QemuBench` is an Arduino sketch that times prosdk's precompiled archive on an emulated
ESP32-S3. It uses the same setup as the smoke test `test/target/QemuSmoke`
(`test/target/README.md`): Espressif's QEMU on machine `esp32s3`, the real Arduino-ESP32 3.3.10 /
ESP-IDF 5.5 binaries, `src/esp32s3/libprosdk.a`, no probot-core (a fake `CoreOps` table from
`test/target/shared/FakeCore.hpp`), and `-icount shift=2,sleep=off`.

**Read this first.** The numbers are instruction counts, not ESP32-S3 wall time. They are good for
comparing two builds of prosdk and for catching regressions. They do not say how fast a robot is.
See [Caveats](#caveats).

## Running it

```
tools/qemu/run.sh --bench                    # compile (under .build/build.lock), boot once, print the table
tools/qemu/run.sh --bench --boots 2          # two boots from the same image: determinism check
tools/qemu/run.sh --bench --check            # fail (exit 1) on a >10 % instr_per_op regression
tools/qemu/run.sh --bench --check --threshold 5
tools/qemu/run.sh --bench --save-baseline    # rewrite tools/qemu/bench-baseline.json from this run
tools/qemu/run.sh --bench --no-build         # reuse .build/qemu/QemuBench
```

The compile uses the build lock and the same settings as the smoke test: `flock .build/build.lock
nice -n 19 ionice -c3 arduino-cli compile --jobs 2`, with the persistent build path
`.build/qemu/QemuBench`. A boot takes about 60 s of host time. The files are:

- `.build/qemu/bench-compile.log`: the compile output, including static RAM and flash use;
- `.build/qemu/bench-uart-<n>.log`: the raw UART output of boot n;
- `.build/qemu/bench.json`: every BENCH line plus clock, memory, stack, task, compile sizes and the
  determinism and check results.

`tools/qemu/bench.py` parses the logs, and you can also run it on its own (`bench.py --help`).

A run passes when every boot prints `QEMU-BENCH END boot=1 groups=4 failed=0` and no `BENCH-FAIL`
line. With `--check`, it must also be within the threshold of the baseline. The baseline
(`tools/qemu/bench-baseline.json`) stores only `instr_per_op` for each benchmark. A benchmark that
got slower by more than the threshold, or that is missing, fails the check. A benchmark that got
more than the threshold faster prints a note; rewrite the baseline with `--save-baseline` once
the speedup is intended. Smoke mode (`tools/qemu/run.sh` without `--bench`) is unchanged.

## Output lines

```
BENCH <name> iters=<n> total_us=<..> per_op_us=<..> instr_per_op=<..> [key=value ...]
```

- `total_us` is the guest time of all `iters` timed calls.
- `per_op_us = total_us / iters`.
- `instr_per_op = total ns / 4 / iters`: the guest instructions executed per operation, because
  under `-icount shift=2` each instruction advances guest time by 4 ns. This is the field to
  compare between builds.

`Run()` in `Bench.hpp` makes one untimed warm-up call first, so first-call allocations and static
initialization are left out. It then times `iters` calls between two clock reads. Optimizer guards:

- every result goes through `DoNotOptimize` (an `asm volatile` with a `"+m"`/`"m"` operand and a
  memory clobber);
- each iteration ends with `ClobberMemory()`;
- inputs change on every iteration wherever a constant input could be hoisted out of the loop.

The other lines are `CLOCK`, `MEM <where>`, `STACK <group>`, `TASK <name>`, `BENCH-CHECK` (a
correctness check of a benchmark's output) and `BENCH-INFO`. The protocol is documented at the
top of `QemuBench.ino`.

### Clock: esp_timer, not CCOUNT

`CalibrateClock()` compares the CPU cycle counter (CCOUNT) with esp_timer in two ways: while busy
(spinning on esp_timer for 20 ms) and across a 100 ms `vTaskDelay`. CCOUNT would be used only if
it ran at the same whole number of MHz in both cases, within 0.5 %. In QEMU it does not:

```
CLOCK ccount_mhz_busy=43.491 ccount_mhz_idle=40.003 source=esp_timer
```

CCOUNT does not follow icount time consistently: it reads 43.5 MHz while busy and 40.0 MHz while
idle, neither of which is 240 MHz. A cycle count would therefore depend on how much idle time an
interval contains. esp_timer follows QEMU's virtual clock, which is the icount clock. Its
resolution is 1 us, so every benchmark's iteration count is chosen to make its timed total at
least about 20 ms (the shortest, `ms5611_compensate_sample`, totals 6.8 ms over 10000 iterations;
a 1 us read error is 0.015 % there). The 32-bit CCOUNT would also wrap after 17.9 s at 240 MHz;
`Stopwatch` falls back to esp_timer beyond 15 s even when CCOUNT is chosen.

## What it measures

The benchmarks run as four groups. Each group gets its own FreeRTOS task with probot-core's
user-task parameters: 24576 bytes of stack (`PROBOT_USER_STACK`), priority 1, core 1. The groups
run in this order: robot, scheduler, math, drivers.

### (a) Robot loop under load (`BenchRobot.cpp`)

The robot is a hosted `TimedRobot`, set up the way a command-based differential-drive robot uses
the SDK. On every loop it runs:

- `CommandScheduler::Run()` with a default drive command (differential kinematics, two
  `PIDController`s, `SimpleMotorFeedforward`), a status command, a repeating
  `Sequence(Wait 250 ms, RunOnce)` and a `Trigger` binding;
- differential odometry in the subsystem's `Periodic()`;
- telemetry: pose, wheel distances and voltages, the scheduler, and one loop counter;
- tunables: both PIDs and a speed, published once.

There is no motor or encoder hardware, because QEMU has no MCPWM or PCNT. The wheels follow their
setpoints.

The group task stands in for core's user task, as QemuSmoke's case d does:

- it calls `pf::detail::Boot()`, then one `PreCycle()`/`PostCycle()` per pass;
- passes run on a `xTaskDelayUntil` fixed-rate grid, at the period the robot asked for through
  `CoreOps` (`core_period_ms` in the output).

The periods are 20, 10, 5 and 2 ms. The 2 ms period was added because 5 ms still had plenty of
headroom. Each period gets a fresh robot, through the test-only robot-host reset:

- 200 ms disabled, then one TeleopInit pass, then teleop;
- the measurement window is 400 to 2400 ms (2 s of teleop after a 200 ms settle).

The fields of a `robot_loop_<P>ms` line:

| field | meaning |
|-------|---------|
| `iters`, `per_op_us` | core passes in the window and the mean time of one pass, `PreCycle` through `PostCycle` (HAL/DS pre-cycle, robot loop, dashboard drain and flush) |
| `max_us` | longest pass |
| `loops`, `user_mean_us`, `user_max_us` | robot loops in the window; time inside `RobotPeriodic` (scheduler run plus telemetry) |
| `jitter_max_us`, `jitter_rms_us` | deviation of the spacing between two loop starts from the period |
| `overruns`, `overruns_total` | late passes (`xTaskDelayUntil` returned false) in the window and overall |
| `headroom_pct`, `headroom_min_pct` | `(period - mean pass) / period` and `(period - longest pass) / period` |

### (b) CommandScheduler (`BenchScheduler.cpp`)

These cases time one `CommandScheduler::Run()` with 0, 10, 50 and 200 scheduled commands. The
`10sub_default` cases add 10 subsystems, each with a `Periodic()` and a default command. Every
command is a `cmd::Run(...)` that never ends and ignores the disabled state, so each `Run()`
executes all of them. The bench checks that the execute count equals `(iters + 1) * commands`.
`heap_bytes` is the heap held by a case's subsystems and commands, plus the scheduler's
bookkeeping, once they are scheduled.

### (c) wpimath and Sleipnir (`BenchMath.cpp`)

| benchmark | operation |
|-----------|-----------|
| `rotation2d_rotateby`, `pose2d_transformby` | geometry composition |
| `pose2d_exp_log` | `Pose2d::Exp` of a twist and `Log` back |
| `diff_odometry_update`, `diff_kinematics_to_wheels` | differential odometry update, `ToWheelSpeeds` |
| `swerve_kinematics_to_module_velocities` | 4-module `ToSwerveModuleVelocities` (2027 name) |
| `swerve_pose_estimator_update` | `UpdateWithTime` in 5 ms steps, after 400 untimed steps fill the history |
| `swerve_pose_estimator_vision` | `AddVisionMeasurement` 100 ms in the past (history replay) |
| `lqr_construct_2x1` | `LinearQuadraticRegulator` construction for a 2-state, 1-input plant (DARE) |
| `kalman_2state_predict_correct`, `linear_system_loop_2state` | `KalmanFilter` predict and correct, `LinearSystemLoop` correct, predict and output |
| `pid_calculate`, `simple_motor_ff_calculate`, `profiled_pid_calculate`, `trapezoid_profile_calculate` | one `Calculate` each |
| `trajectory_generate_2waypoints` | upstream's 2018 cross-scale spline trajectory (the same one as QemuSmoke case a) |
| `sleipnir_qp_2var` | a 2-variable QP compiled in the sketch (setup and solve) |
| `sleipnir_arm_feedforward` | `ArmFeedforward::Calculate(angle, v0, v1)`, the Newton solve in the archive |
| `sleipnir_ellipse2d_nearest` | `Ellipse2d::Nearest`, the archive's Sleipnir solve |

### (d) Boardoza driver decode paths (`BenchDrivers.cpp`)

Only paths that are pure computation are timed. The canned inputs come from the host tests in
`test/prosdk/drivers`.

| benchmark | operation |
|-----------|-----------|
| `nmea_epoch_gga_rmc_gsa` | NMEA parser over one epoch (GGA, RMC and GSA sentences, byte by byte) |
| `ubx_navpvt_frame` | UBX NAV-PVT frame, byte by byte, with checksum and decode |
| `mlx90640_extract_parameters` | Melexis `MLX90640_ExtractParameters` on the datasheet example EEPROM |
| `mlx90640_frame_to_768px` | `GetTa` plus `CalculateTo` for a whole frame (768 pixels). `BENCH-CHECK` prints Ta 39.184 and pixel 367 = 80.18, the host test's values |
| `bme688_compensate_sample` | BME688 temperature, pressure, humidity and gas compensation |
| `ms5611_compensate_sample` | MS5611 second-order compensation |
| `tmag5170_crc4_frame` | TMAG5170 32-bit SPI frame CRC4 |
| `opt4048_decode_channel_crc` | OPT4048 channel decode (mantissa, exponent, counter) and its CRC |
| `vl53l7cx_swap_buffer_1k` | VL53L7CX platform `VL53L7CX_SwapBuffer` over 1 KB |

Drivers left out, and why:

- **FT8xx (EVE display):** the constructor opens SPI and GPIO and starts a bus thread. QEMU's
  esp32s3 machine has no GPSPI controller, and nothing in the driver is pure decode.
- **VL53L7CX frame parse:** the parse is inside `vl53l7cx_get_ranging_data`, after the I2C read of
  the result buffer, so it cannot be reached without a bus. Only the byte swap it uses is timed.
- **TMAG5170 conversion; DPS310, ENS210, HIH8000, LM75A, BMI088, BNO055, LSM303AGR, MPU6050,
  NAU7802, HX711, LoadCell, CaliPile and OPT3001 conversions:** the raw-to-unit conversion is
  private and reached only after a bus transfer. A fake bus would mostly time the bus layer.
  `OPT3001::ResultToLux` is public and static but was not added; it is a single multiply and
  shift.
- **MLX90641:** the same Melexis library family as MLX90640, at a smaller size. Left out as a
  duplicate.
- **SX126x (LoRa), IR and OOK (RMT, pulse capture), W25Q/W25N flash, NT3H2111, PCF8563, PCA9685,
  motor drivers, ProHubEncoders, PYD1598, GoBildaPinpoint:** these are bus or peripheral
  transactions (SPI, I2C, RMT, GPIO timing) with no decode step worth timing. QEMU does not model
  the peripherals.

### (e) Memory

- `MEM start`, then `MEM after-<group>`: `heap_caps` 8-bit free, minimum ever free, and largest
  free block.
- `STACK <group> used=<bytes>/24576`: the group task's peak stack.
- `TASK` lines at the end: every FreeRTOS task's stack high-water mark.
- Static RAM and flash use come from the compile log.

## Results

Measured on 2026-10-22, prosdk-2027 at `0294919` plus this change, Arduino-ESP32 3.3.10, QEMU
`esp-develop-9.2.2-20260417`. Commands: `tools/qemu/run.sh --bench --boots 2 --save-baseline`,
then `tools/qemu/run.sh --bench --no-build --check` (PASS). Every `instr_per_op` was identical
across the three boots.

| benchmark | iters | per_op_us | instr_per_op | notes |
|-----------|------:|----------:|-------------:|-------|
| robot_loop_20ms | 100 | 886.560 | 221640.0 | max 972 us, user mean/max 447.7/534 us, jitter max 3 us (rms 2.7), overruns 0, headroom 95.6 % (min 95.1 %) |
| robot_loop_10ms | 199 | 883.839 | 220959.8 | max 974 us, user 444.1/533 us, jitter max 998 us (rms 71.0), overruns 0, headroom 91.2 % (min 90.3 %) |
| robot_loop_5ms | 400 | 893.747 | 223436.9 | max 1885 us, user 446.3/965 us, jitter max 1019 us (rms 662.7), overruns 0, headroom 82.1 % (min 62.3 %) |
| robot_loop_2ms | 1000 | 886.231 | 221557.8 | max 1423 us, user 443.7/699 us, 999 loops, jitter max 1990 us (rms 170.8), overruns 0, headroom 55.7 % (min 28.9 %) |
| scheduler_run_0cmd | 2000 | 142.227 | 35556.9 | heap_bytes -656 (nothing held) |
| scheduler_run_10cmd | 1000 | 278.577 | 69644.2 | heap_bytes 9664 |
| scheduler_run_50cmd | 400 | 884.683 | 221170.6 | heap_bytes 48212 |
| scheduler_run_200cmd | 100 | 4919.570 | 1229892.5 | heap_bytes 196488 |
| scheduler_run_10sub_default | 1000 | 639.863 | 159965.8 | heap_bytes 10884 |
| scheduler_run_200cmd_10sub_default | 100 | 5454.490 | 1363622.5 | heap_bytes 205800 |
| rotation2d_rotateby | 20000 | 10.170 | 2542.4 | |
| pose2d_transformby | 20000 | 12.343 | 3085.9 | |
| pose2d_exp_log | 10000 | 93.565 | 23391.2 | |
| diff_odometry_update | 10000 | 75.158 | 18789.5 | |
| diff_kinematics_to_wheels | 20000 | 1.337 | 334.2 | |
| swerve_kinematics_to_module_velocities | 5000 | 67.730 | 16932.4 | |
| swerve_pose_estimator_update | 2000 | 235.130 | 58782.4 | |
| swerve_pose_estimator_vision | 200 | 629.370 | 157342.5 | |
| lqr_construct_2x1 | 100 | 1013.970 | 253492.5 | |
| kalman_2state_predict_correct | 2000 | 282.642 | 70660.4 | |
| linear_system_loop_2state | 2000 | 314.715 | 78678.8 | |
| pid_calculate | 20000 | 9.615 | 2403.7 | |
| simple_motor_ff_calculate | 20000 | 16.942 | 4235.4 | |
| profiled_pid_calculate | 10000 | 11.197 | 2799.2 | |
| trapezoid_profile_calculate | 20000 | 8.060 | 2014.9 | |
| trajectory_generate_2waypoints | 10 | 62251.700 | 15562925.0 | |
| sleipnir_qp_2var | 20 | 8967.700 | 2241925.0 | |
| sleipnir_arm_feedforward | 20 | 4466.300 | 1116575.0 | |
| sleipnir_ellipse2d_nearest | 20 | 4428.750 | 1107187.5 | |
| nmea_epoch_gga_rmc_gsa | 1000 | 241.759 | 60439.8 | |
| ubx_navpvt_frame | 2000 | 15.996 | 3999.0 | |
| mlx90640_extract_parameters | 20 | 63345.300 | 15836325.0 | |
| mlx90640_frame_to_768px | 20 | 11976.350 | 2994087.5 | Ta 39.184, pixel 367 80.18 (host values) |
| bme688_compensate_sample | 2000 | 26.653 | 6663.2 | |
| ms5611_compensate_sample | 10000 | 0.682 | 170.6 | |
| tmag5170_crc4_frame | 20000 | 1.529 | 382.3 | |
| opt4048_decode_channel_crc | 20000 | 1.045 | 261.1 | |
| vl53l7cx_swap_buffer_1k | 2000 | 19.726 | 4931.5 | |

Memory (boot of the `--check` run; the other boots differ by a few hundred bytes of heap):

| where | heap free | min ever free | largest block |
|-------|----------:|--------------:|--------------:|
| start | 333436 | 324340 | 262132 |
| after robot | 275424 | 206432 | 176116 |
| after scheduler | 273964 | 23248 | 139252 |
| after math | 273964 | 23248 | 139252 |
| after drivers | 273964 | 23248 | 139252 |

- **Peak group stack (of 24576):** robot 4708, scheduler 3728, math 9088, drivers 5168.
- **Minimum free stack of the other tasks:** runner 14180 (of 16384), `prosdk` (HAL notifier)
  2784, `watchdog` 4648, `esp_timer` 7784, `ipc0`/`ipc1` 232/224, `IDLE0`/`IDLE1` 208/312.
- **Static RAM** (`.data` + `.bss`, from the compile log): 41548 bytes (12 % of 327680).
- **Flash:** 2034246 bytes (64 % of the 3 MB huge_app partition).
- **Robot dashboard:** 22 topics, about 15800 sets over the four robots (`BENCH-INFO`).

Determinism: `run.sh --bench --boots 2` printed `== determinism: 2 boots, every instr_per_op
identical`, and the following `--check` boot matched the baseline exactly. Builds of the same
sources that differ elsewhere (for example a debug line added in another file) moved some numbers
by up to 2.4 % between earlier runs, most of all `scheduler_run_0cmd` and `robot_loop_5ms`. That
comes from code and data layout: alignment, and literal pools in flash-mapped code. It is why the
default threshold is 10 %, not 1 %.

## What the numbers mean

- **The robot loop is dominated by fixed per-pass cost, not the period.** One pass costs about
  886 us of guest time at every period. About 445 us of it is the user `RobotPeriodic`: the
  scheduler with a drive command, kinematics, two PIDs, feedforward, odometry and telemetry. The
  rest is the HAL/DS pre-cycle, `LoopFunc`, and the dashboard drain and flush.
  - At 20 ms that leaves 95 % headroom.
  - At 5 ms it is 82 % on average and 62 % in the worst pass.
  - At 2 ms the mean headroom is 56 % and the worst pass leaves 29 %; earlier runs had 1 to 2
    overruns at 2 ms.
  - Given the caveats below, 5 ms looks comfortable and 2 ms marginal. A board has to confirm
    both.
- **Jitter:** at 20 ms the loop starts land within 3 us of the period. At 10 ms and below some
  loops start one tick (about 1 ms) late. At 5 ms every other loop is about 1 ms late
  (spacing alternating about 4 ms and 6 ms, rms 663 us). At 2 ms one loop in the window was
  skipped (999 loops in 1000 passes, jitter max 1990 us). See the open items.
- **The scheduler scales linearly.** Each `Run()` costs about 35.6k instructions with no commands,
  plus about 6k per scheduled command; 200 commands take about 1.23M instructions, or 4.9 ms.
  Each subsystem with a periodic method and a default command costs about 12k instructions.
  Each scheduled command holds about 980 bytes of heap. With 200 commands the minimum free heap
  dropped to 23 KB, so a 200-command program is a memory problem before it is a time problem.
- **Math:**
  - Geometry and PID-type calls cost a few thousand instructions each (2 to 17 us here).
  - Odometry costs about 19k instructions and a swerve estimator update about 59k.
  - A vision correction with history replay costs 157k instructions.
  - An LQR construction costs about 250k instructions (1 ms). Build it once at init, not in the
    loop.
  - Trajectory generation costs 15.6M instructions (62 ms). Generate trajectories at init or in a
    thread.
  - Every Sleipnir solve costs 1.1 to 2.2M instructions (4.4 to 9 ms). `ArmFeedforward`'s
    three-argument `Calculate` solves one Newton problem per call, too slow for a 5 ms loop.
    Use it at init or at a slower rate.
- **Drivers:**
  - Decode paths per sample are cheap: MS5611 170 instructions, OPT4048 260, TMAG5170 CRC 380,
    UBX NAV-PVT 4k, BME688 6.7k, an NMEA epoch 60k.
  - MLX90640 is the outlier. Extracting parameters costs 15.8M instructions (63 ms, once at
    init). A full 768-pixel frame costs 3M instructions (12 ms), which is several 5 ms loops. Run
    it in its own thread.

## Caveats

- **Each instruction counts 4 ns.** `-icount shift=2` makes guest time 4 ns per executed guest
  instruction, which is 250 MIPS. The absolute `*_us` numbers are instruction counts times 4 ns.
  They are estimates of a CPU that retires one instruction every 4 ns, not ESP32-S3 measurements.
- **No cache, flash or pipeline modeling.** QEMU's TCG does not model the instruction and data
  caches, flash and PSRAM wait states over the MMU cache, pipeline stalls, branch penalties,
  load-use hazards or interrupt latency. Every instruction costs the same.
- **A real ESP32-S3 at 240 MHz is usually slower than these numbers.** The LX7 retires at most
  one instruction per cycle, and code and constants run from flash through a cache, so cache misses
  cost tens of cycles. Expect real numbers to be equal or worse, but there is no fixed factor.
  It depends on the code's cache behavior.
- **double math is software on the ESP32-S3.** The S3's FPU is single precision only, so every
  double operation (all of wpimath, Eigen and Sleipnir) is a libgcc soft-float call of tens of
  instructions. That cost is in the counts here. What QEMU does not show is how those calls
  behave in the cache on the chip.
- **The point is relative numbers and regressions.** Instruction counts are deterministic for one
  binary. Compare two prosdk builds with the same benchmark (`--check`), or compare two operations
  with each other. Do not compare them with a deadline on a real robot.
- **Real numbers need a board.** The same sketch on a board, with esp_timer or CCOUNT at
  240 MHz, would give them. QEMU also does not run probot-core's WiFi, Driver Station or real
  hooks, nor any peripheral HAL path, so the robot loop here leaves out motor, encoder and network
  work.
- Two cores run round-robin on one TCG thread. Tasks on core 0 (`esp_timer`, IPC) take turns
  with the benchmark task instead of running in parallel.

## Open items

- **Robot host late boot:** a robot registered after `Boot()` is constructed inside the next
  `PreCycle()`, and the schedule is aligned with a `now` that includes the construction time
  (`src/prosdk/platform`, robot host). With the half-pass lookahead this put the 5 ms loop
  deadline at the lookahead edge and made it alternate between two passes. The bench works
  around it: it constructs the robot in a separate pass, then bumps `core.overruns` to force a
  realignment. The robot host itself is not changed (no `src/` edits in this change). On a robot,
  core boots the robot before the first pass, so this matters only for a re-registered robot.
- **5 ms jitter of about 1 ms:** even after the workaround, every other 5 ms loop runs about 1 ms
  late (spacing 3981 / 6016 us). Likely contributors:
  - the FreeRTOS tick and esp_timer drift apart in QEMU by about 150 ppm (a tick measures
    0.99985 ms on esp_timer);
  - the robot host decides due callbacks with a half-pass lookahead on esp_timer, while passes
    run on the tick grid.

  It is not explained further here; it needs a look at the robot host's `RunDue` on a board.
- **2 ms is marginal:** worst-pass headroom is 29 %, there were 1 to 2 overruns in earlier runs,
  and one skipped loop in this run.
- **Scheduler heap:** about 980 bytes per scheduled command (`cmd::Run(...).IgnoringDisable().
  WithName()`, plus the scheduler's bookkeeping); 200 commands brought the minimum free heap to
  23 KB. Worth a look at the command and scheduler allocations if large programs matter.
- **Test-only API:** the bench uses `pf::detail::ResetRobotMainForTest()` to run several robots in
  one boot. That is the test-only reset path, which a robot never uses.
- **No hardware paths:** QEMU has no MCPWM, PCNT, LEDC, RMT, TWAI, GPSPI or I2C devices, so no
  motor, encoder or sensor bus time is in these numbers. The driver benchmarks cover decode only.
- **CCOUNT:** CCOUNT is not usable as a clock in QEMU (43.5 MHz busy, 40.0 MHz idle). A board
  run should use CCOUNT at 240 MHz for sub-microsecond numbers. `CalibrateClock()` already picks
  it when it is consistent.
- **Not in CI:** the CI `qemu` job (manual dispatch) still runs only the smoke test. Adding
  `tools/qemu/run.sh --bench --check` there is a separate decision.
