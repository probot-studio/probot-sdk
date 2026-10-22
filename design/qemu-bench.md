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
tools/qemu/run.sh --bench --boots 2          # two boots, same image: fail unless identical
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
line, and, with two or more boots, every boot gives the same `instr_per_op` for every benchmark
(any difference fails the run). With `--check`, it must also be within the threshold of the
baseline; the check reads boot 1, which stands for every boot once they are identical. The
baseline (`tools/qemu/bench-baseline.json`) stores only `instr_per_op` for each benchmark. A
benchmark that got slower by more than the threshold, or that is missing, fails the check. A
benchmark that got more than the threshold faster prints a note; rewrite the baseline with
`--save-baseline` once the speedup is intended. `--save-baseline` writes the file only when the
run passed (boots, determinism and, if given, the check), so `--check --save-baseline` never
accepts a regression. Smoke mode (`tools/qemu/run.sh` without `--bench`) rejects the bench-only
flags `--check`, `--threshold` and `--save-baseline` with the usage error (exit 2) and is
otherwise unchanged.

## Output lines

```
BENCH <name> iters=<n> total_us=<..> per_op_us=<..> instr_per_op=<..> [key=value ...]
```

- `total_us` is the guest time of all `iters` timed calls.
- `per_op_us = total_us / iters`.
- `instr_per_op = total ns / 4 / iters`: the guest instructions executed per operation, because
  under `-icount shift=2` each instruction advances guest time by 4 ns. This is the field to
  compare between builds. It counts every guest instruction executed during the timed interval
  on either core, not only the benchmark's: the 1 kHz tick interrupts of both cores and the
  priority-3 `watchdog` and `prosdk` tasks, which share core 1 with the priority-1 benchmark task
  and preempt it, are included. A static count of `tmag5170_crc4_frame` from the disassembly
  (the `Run` loop 5 instructions, the lambda 12, `TMAG5170::Crc4` 361, so 378 per call) against the
  measured 382.3 puts this overhead at about 1 % for a short loop, and confirms the
  4 ns-per-instruction model to the same precision.

`Run()` in `Bench.hpp` makes one untimed warm-up call first, so first-call allocations and static
initialization are left out. It then times `iters` calls between two clock reads. Optimizer guards:

- every result goes through `DoNotOptimize` (an `asm volatile` with a `"+m"`/`"m"` operand and a
  memory clobber);
- each iteration ends with `ClobberMemory()`;
- inputs change on every iteration wherever a constant input could be hoisted out of the loop.

The other lines are `CLOCK`, `MEM <where>`, `STACK <group>`, `TASK <name>`, `BENCH-CHECK` (a
benchmark's output compared with known values; a mismatch also prints `BENCH-FAIL`, which fails
the run) and `BENCH-INFO`. The protocol is documented at the
top of `QemuBench.ino`.

### Clock: esp_timer, not CCOUNT

`CalibrateClock()` compares the CPU cycle counter (CCOUNT) with esp_timer in two ways: while busy
(spinning on esp_timer for 20 ms) and across a 100 ms `vTaskDelay`. CCOUNT would be used only if
it ran at the same whole number of MHz in both cases, within 0.5 %. In QEMU it does not:

```
CLOCK ccount_mhz_busy=43.492 ccount_mhz_idle=40.002 source=esp_timer
```

CCOUNT does not follow icount time consistently: it reads 43.5 MHz while busy and 40.0 MHz while
idle, neither of which is 240 MHz. A cycle count would therefore depend on how much idle time an
interval contains. (The third decimal changes between builds: 43.491 to 43.493 busy and 40.002
to 40.003 idle have been seen. The esp_timer counts do not depend on it.) esp_timer follows
QEMU's virtual clock, which is the icount clock. Its
resolution is 1 us, so every benchmark's iteration count is chosen to make its timed total at
least 20 ms (the shortest, `ms5611_compensate_sample`, totals 20.5 ms over 30000 iterations; a
1 us read error is 0.005 % there). The 32-bit CCOUNT would also wrap after 17.9 s at 240 MHz;
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

- it calls `pf::detail::Boot()`, then one `PreCycle()`/`PostCycle()` per pass; each later robot
  is registered after that boot, so the first pass of its period constructs it, as on a robot
  that registers late;
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
| `pass_jitter_max_us` | deviation of the spacing between two pass starts (the task's wake-ups) from the period |
| `loop_offset_min_us`, `loop_offset_max_us` | time from the pass start (`CoreOps::CycleStartTime()`) to `RobotPeriodic` |

### (b) CommandScheduler (`BenchScheduler.cpp`)

These cases time one `CommandScheduler::Run()` with 0, 10, 50 and 200 scheduled commands. The
`10sub_default` cases add 10 subsystems, each with a `Periodic()` and a default command. Every
command is a `cmd::Run(...)` that never ends and ignores the disabled state, so each `Run()`
executes all of them, and so does every default command. The bench checks that the execute count
equals `(iters + 1) * (commands + subsystems)` and prints `BENCH-FAIL` otherwise.
`heap_bytes` is the heap held by a case's subsystems and commands, plus the scheduler's
bookkeeping, once they are scheduled and the watchdog's epochs are in their steady state (three
`Run()` calls: the tracer keeps an epoch for one loop after its last use).

### (c) wpimath and Sleipnir (`BenchMath.cpp`)

| benchmark | operation |
|-----------|-----------|
| `rotation2d_rotateby`, `pose2d_transformby` | geometry composition |
| `pose2d_exp_log` | `Pose2d::Exp` of a twist and `Log` back |
| `diff_odometry_update`, `diff_kinematics_to_wheels` | differential odometry update, `ToWheelVelocities` |
| `swerve_kinematics_to_module_velocities` | 4-module `ToSwerveModuleVelocities` (2027 name) |
| `swerve_pose_estimator_update` | `UpdateWithTime` in 5 ms steps, after 400 untimed steps fill the history |
| `swerve_pose_estimator_update_vision` | one loop with vision: the `UpdateWithTime` above, then `AddVisionMeasurement` 100 ms in the past. The 2027 estimator samples its odometry buffer and its compensated estimate at that time and applies the Kalman-scaled correction; it does not replay odometry. The vision part alone is the difference with `swerve_pose_estimator_update` |
| `lqr_construct_2x1` | `LinearQuadraticRegulator` construction for a 2-state, 1-input plant (DARE) |
| `kalman_2state_predict_correct`, `linear_system_loop_2state` | `KalmanFilter` predict and correct, `LinearSystemLoop` correct, predict and output |
| `pid_calculate`, `simple_motor_ff_calculate`, `profiled_pid_calculate`, `trapezoid_profile_calculate` | one `Calculate` each. The two profiles move 0 to 3 m at 2 m/s and 1 m/s^2, a triangular profile of 3.46 s, and restart after 3 s (600 steps of 5 ms, 150 of 20 ms), so every call samples the moving part, not the settled goal |
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
| `mlx90640_frame_to_768px` | `GetTa` plus `CalculateTo` for a whole frame (768 pixels). `BENCH-CHECK` prints the extract error, Ta and pixel 367, and `BENCH-FAIL` follows unless they match the host test (`MLX90640Test.cpp`: error 0, Ta 39.184 +- 0.01, pixel 367 80.18 +- 0.1) |
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

Measured on 2026-10-22 with the committed sources (prosdk-2027 at 5aa0494, Arduino-ESP32 3.3.10,
QEMU `esp-develop-9.2.2-20260417`): `tools/qemu/run.sh --bench --boots 2 --save-baseline` (compile
and two boots). Both boots printed identical `BENCH` lines, and the table is the baseline in
`tools/qemu/bench-baseline.json`. These results include the robot host's pass-start alignment and
the scheduler's epoch reuse (wpilibc `0014`, commandsv2 `0005`); the previous baseline (b7cd4b9)
is compared under [Changes from the previous baseline](#changes-from-the-previous-baseline).

| benchmark | iters | per_op_us | instr_per_op | notes |
|-----------|------:|----------:|-------------:|-------|
| robot_loop_20ms | 100 | 747.380 | 186845.0 | max 874 us, user mean/max 351.6/478 us, jitter max 3 us (rms 2.9), pass jitter max 3 us, loop offset 187-188 us, overruns 0, headroom 96.3 % (min 95.6 %) |
| robot_loop_10ms | 200 | 743.895 | 185973.8 | max 873 us, user mean/max 347.7/477 us, jitter max 998 us (rms 70.8), pass jitter max 998 us, loop offset 188-189 us, overruns 0, headroom 92.6 % (min 91.3 %) |
| robot_loop_5ms | 400 | 742.090 | 185522.5 | max 865 us, user mean/max 348.7/474 us, jitter max 1020 us (rms 814.9), pass jitter max 1027 us, loop offset 180-189 us, overruns 0, headroom 85.2 % (min 82.7 %) |
| robot_loop_2ms | 1000 | 753.279 | 188319.8 | max 1711 us, user mean/max 351.1/867 us, 996 loops, jitter max 1990 us (rms 279.5), pass jitter max 1027 us, loop offset 180-871 us, overruns 1 (2 overall), headroom 62.3 % (min 14.4 %) |
| scheduler_run_0cmd | 2000 | 128.366 | 32091.5 | heap_bytes -632 (nothing held) |
| scheduler_run_10cmd | 1000 | 169.765 | 42441.2 | heap_bytes 9972 |
| scheduler_run_50cmd | 400 | 375.765 | 93941.2 | heap_bytes 49792 |
| scheduler_run_200cmd | 100 | 1206.190 | 301547.5 | heap_bytes 202164 |
| scheduler_run_10sub_default | 1000 | 264.048 | 66012.0 | heap_bytes 11196 |
| scheduler_run_200cmd_10sub_default | 100 | 1320.170 | 330042.5 | heap_bytes 211736 |
| rotation2d_rotateby | 20000 | 10.170 | 2542.5 |  |
| pose2d_transformby | 20000 | 12.344 | 3085.9 |  |
| pose2d_exp_log | 10000 | 93.565 | 23391.3 |  |
| diff_odometry_update | 10000 | 75.158 | 18789.5 |  |
| diff_kinematics_to_wheels | 20000 | 1.338 | 334.4 |  |
| swerve_kinematics_to_module_velocities | 5000 | 67.726 | 16931.4 |  |
| swerve_pose_estimator_update | 2000 | 235.127 | 58781.8 |  |
| swerve_pose_estimator_update_vision | 200 | 629.285 | 157321.2 | vision part alone about 98.5k (157321.2 - 58781.8) |
| lqr_construct_2x1 | 100 | 1022.410 | 255602.5 |  |
| kalman_2state_predict_correct | 2000 | 282.261 | 70565.2 |  |
| linear_system_loop_2state | 2000 | 314.445 | 78611.1 |  |
| pid_calculate | 20000 | 9.616 | 2403.9 |  |
| simple_motor_ff_calculate | 20000 | 16.941 | 4235.3 |  |
| profiled_pid_calculate | 10000 | 20.048 | 5011.9 |  |
| trapezoid_profile_calculate | 20000 | 15.906 | 3976.6 |  |
| trajectory_generate_2waypoints | 10 | 62251.000 | 15562750.0 |  |
| sleipnir_qp_2var | 20 | 8960.850 | 2240212.5 |  |
| sleipnir_arm_feedforward | 20 | 4462.300 | 1115575.0 |  |
| sleipnir_ellipse2d_nearest | 20 | 4407.900 | 1101975.0 |  |
| nmea_epoch_gga_rmc_gsa | 1000 | 241.761 | 60440.2 |  |
| ubx_navpvt_frame | 2000 | 16.006 | 4001.6 |  |
| mlx90640_extract_parameters | 20 | 63345.000 | 15836250.0 |  |
| mlx90640_frame_to_768px | 20 | 11976.450 | 2994112.5 | BENCH-CHECK Ta 39.184, pixel 367 80.18 (host values) |
| bme688_compensate_sample | 2000 | 26.642 | 6660.5 |  |
| ms5611_compensate_sample | 30000 | 0.684 | 170.9 |  |
| tmag5170_crc4_frame | 20000 | 1.529 | 382.3 |  |
| opt4048_decode_channel_crc | 20000 | 1.044 | 260.9 |  |
| vl53l7cx_swap_buffer_1k | 2000 | 19.732 | 4933.0 |  |
Memory (boot 1; other builds differ by a few hundred bytes of heap, and the scheduler's minimum by
a few KB):

| where | heap free | min ever free | largest block |
|-------|----------:|--------------:|--------------:|
| start | 333372 | 324276 | 262132 |
| after robot | 276300 | 190208 | 176116 |
| after scheduler | 274808 | 18952 | 147444 |
| after math | 274808 | 18952 | 147444 |
| after drivers | 274808 | 18952 | 147444 |

- **Peak group stack (of 24576):** robot 4884, scheduler 3744, math 9104, drivers 4736.
- **Minimum free stack of the other tasks:** runner 14180 (of 16384), `prosdk` (HAL notifier)
  2800, `watchdog` 4664, `esp_timer` 7800, `ipc0`/`ipc1` 248/240, `IDLE0`/`IDLE1` 224/328.
- **Static RAM** (`.data` + `.bss`, from the compile log): 41612 bytes (12 % of 327680).
- **Flash:** 2036154 bytes (64 % of the 3 MB huge_app partition).
- **Robot dashboard:** 22 topics, about 15800 sets over the four robots (`BENCH-INFO`).
- The robot group's minimum free heap is about 16 KB lower than in the previous baseline
  (206300): the bench keeps a third per-loop stamp vector (`loopOffset`, up to 1600 values of
  8 bytes, grown by doubling), not the SDK. The scheduler's minimum (the 200-command case) is
  about 0.9 KB higher.

Determinism: one binary gives the same counts on every boot. The run above printed
`== determinism: 2 boots, every instr_per_op identical`, and the `BENCH` lines of both boots
are byte for byte the same, robot jitter and heap included. The previous binaries behaved the same.
A rebuild changes this. After adding only the two-line copyright comment at the top of
`QemuBench.ino`, which shifts the sketch's line numbers, the counts moved by up to 2.4 %:
`scheduler_run_0cmd` -2.4 %, `scheduler_run_10cmd` -1.8 %, `robot_loop_5ms` +1.1 %,
`lqr_construct_2x1` +0.7 %, and under 0.25 % for the rest. Static RAM and flash size were
unchanged. Moving the runner and user-task code to `test/target/shared/QemuTasks.hpp` moved the
benchmarks it did not touch by up to 1.6 % (`scheduler_run_10cmd` +1.6 %,
`scheduler_run_200cmd_10sub_default` +1.1 %, `scheduler_run_0cmd` +0.9 %, `lqr_construct_2x1`
-0.6 %, the rest under 0.5 %). The likely cause is code and data placement (alignment, literal
pools, flash-mapped code), and this is why the default threshold is 10 %, not 1 %.

## What the numbers mean

- **The robot loop is dominated by fixed per-pass cost, not the period.** One pass costs about
  745 to 755 us of guest time at every period. About 350 us of it is the user `RobotPeriodic`: the
  scheduler with a drive command, kinematics, two PIDs, feedforward, odometry and telemetry. The
  rest is the HAL/DS pre-cycle, `LoopFunc`, and the dashboard drain and flush. `RobotPeriodic`
  starts 180 to 189 us after the pass start in every ordinary pass.
  - At 20 ms that leaves 96 % headroom.
  - At 5 ms it is 85 % on average and 83 % in the worst pass.
  - At 2 ms the mean headroom is 62 %, but the worst pass took 1711 us of the 2000 us (14 % left;
    20 %, 10 % and 29 % in earlier builds), and the window had one overrun.
  - Given the caveats below, 5 ms looks comfortable and 2 ms marginal. A board has to confirm
    both.
- **Jitter is in the wake-ups, not in the robot host.** At 20 ms the loop starts land within 3 us
  of the period. At 10 ms and 5 ms the loop spacing jitter equals the spacing jitter of the pass
  starts (`pass_jitter_max_us` 998 and 1027 us), while `RobotPeriodic` keeps the same offset
  inside every pass (`loop_offset` 188-189 and 180-189 us), and every pass runs exactly one loop
  (200 of 200, 400 of 400). So the bench task's `xTaskDelayUntil` wakes up to one tick late: at
  5 ms every other pass (spacing alternating about 4 ms and 6 ms, rms 815 us). At 2 ms a wake
  that late is more than half a pass, so that pass runs its own loop and the next one, and the
  following pass runs none. The data fit this (`loop_offset_max_us` 871: a loop that started
  about 690 us after the usual offset, one loop's length), but the bench does not count loops per
  pass. With the one overrun (no catch-up) the window had 996 loops in 1000 passes. See the open
  items.
- **The scheduler's cost per command is now nearly linear.** Each `Run()` costs about 32k
  instructions with no commands. The marginal cost per command is about 1.0k from 0 to 10
  commands, 1.3k from 10 to 50 and 1.4k from 50 to 200, so 200 commands take about 0.30M
  instructions (1.2 ms). Before wpilibc `0014` and commandsv2 `0005` it was 3.4k, 3.8k and 6.8k:
  almost all of it was the watchdog epoch per command (a new `GetName() + ".Execute()"` string,
  and a string-map node and key allocated again every `Run()` because `Tracer::ClearEpochs()`
  freed them all). What is left per
  command is the epoch itself (a clock read, the `GetName()` copy, an O(log n) map lookup), the
  command's wrapper calls and `std::function`, the scheduled-set lookup, and the copy of the
  scheduled set that `Run()` makes (upstream: a `SmallSet<Command*, 12>` is a `SmallPtrSet`, one
  pointer table per copy above 12 commands, not one node per command). Each subsystem with a
  periodic method and a default command costs about 3.4k instructions (12.6k before). Each
  scheduled command still holds about 1 KB of heap, mostly the command objects the user builds
  (`cmd::Run(...).IgnoringDisable().WithName()`: three objects, plus the composition stack trace
  upstream stores per decorator). With 200 commands the minimum free heap dropped to 19 KB, so a
  200-command program is a memory problem before it is a time problem.
- **Math:**
  - Geometry and PID-type calls cost 2.4k to 5k instructions each (10 to 20 us here). A profile
    step that is still moving (`TrapezoidProfile` 4.0k, `ProfiledPIDController` 5.0k) costs about
    twice as much as one that has reached its goal.
  - Odometry costs about 19k instructions and a swerve estimator update about 59k.
  - A vision correction costs about 99k instructions on top of the update (157k for the loop
    with both).
  - An LQR construction costs about 254k instructions (1 ms). Build it once at init, not in the
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

## Changes from the previous baseline

The previous baseline (b7cd4b9, the table it replaced) and this one, for the benchmarks that
moved by more than 0.5 %:

| benchmark | instr_per_op before | after | change | other fields, before -> after |
|-----------|--------------------:|------:|-------:|-------------------------------|
| robot_loop_20ms | 221630.0 | 186845.0 | -15.7 % | jitter max 3 -> 3 us (rms 2.7 -> 2.9); headroom 95.6/95.1 -> 96.3/95.6 % |
| robot_loop_10ms | 221072.9 | 185973.8 | -15.9 % | jitter max 998 -> 998 us (rms 71.0 -> 70.8); headroom 91.2/90.3 -> 92.6/91.3 % |
| robot_loop_5ms | 227057.5 | 185522.5 | -18.3 % | jitter max 1019 -> 1020 us (rms 645.8 -> 814.9); max pass 1949 -> 865 us; headroom 81.8/61.0 -> 85.2/82.7 % |
| robot_loop_2ms | 221974.8 | 188319.8 | -15.2 % | loops 999 -> 996; jitter max 1990 -> 1990 us (rms 226.7 -> 279.5); overruns 0 -> 1; max pass 1607 -> 1711 us; headroom 55.6/19.6 -> 62.3/14.4 % |
| scheduler_run_0cmd | 35034.5 | 32091.5 | -8.4 % | heap_bytes -644 -> -632 |
| scheduler_run_10cmd | 69491.8 | 42441.2 | -38.9 % | heap_bytes 9980 -> 9972 |
| scheduler_run_50cmd | 221137.5 | 93941.2 | -57.5 % | heap_bytes 49792 -> 49792 |
| scheduler_run_200cmd | 1236187.5 | 301547.5 | -75.6 % | heap_bytes 202168 -> 202164 |
| scheduler_run_10sub_default | 160620.2 | 66012.0 | -58.9 % | heap_bytes 11240 -> 11196 |
| scheduler_run_200cmd_10sub_default | 1381765.0 | 330042.5 | -76.1 % | heap_bytes 211808 -> 211736 |
| lqr_construct_2x1 | 253710.0 | 255602.5 | +0.7 % | code placement (see Determinism) |

- The robot loop got about 35k instructions (140 us) cheaper per pass because its watchdog
  epochs (`IterativeRobotBase`, the scheduler's) no longer allocate.
- The scheduler cases are the epoch change (wpilibc `0014`, commandsv2 `0005`). `heap_bytes`
  is now taken in the epochs' steady state (three warm-up `Run()` calls) and is the same within a
  few dozen bytes. The minimum free heap after the scheduler group is 18952 bytes (18068
  before): the kept epoch entries do not raise the peak, because the first new epoch of a loop
  frees the entries that loop has not reused before it allocates.
- The robot cases' jitter is the bench task's late wake-ups (see What the numbers mean). The
  5 ms worst pass no longer stretches to 1949 us: in the old bench a robot was constructed in a
  separate pass and the host aligned on a clock reading taken after the construction; the robot
  host now aligns on the pass start, and the bench constructs each later robot in its first timed
  pass, as a robot that registers late. The 2 ms changes (996 loops, one overrun, worst pass)
  are within what late wake-ups of a tick do to a 2 ms pass; 2 ms varied from build to build
  before too.
- The rest moved by less than 0.4 % (code placement).

## Caveats

- **Each instruction counts 4 ns.** `-icount shift=2` makes guest time 4 ns per executed guest
  instruction, which is 250 MIPS. The absolute `*_us` numbers are instruction counts times 4 ns.
  They are estimates of a CPU that retires one instruction every 4 ns, not ESP32-S3 measurements.
- **No cache, flash or pipeline modeling.** QEMU's TCG does not model the instruction and data
  caches, flash and PSRAM wait states over the MMU cache, pipeline stalls, branch penalties,
  load-use hazards or interrupt latency. Every instruction costs the same.
- **A real ESP32-S3 at 240 MHz is slower than these numbers.** The LX7 retires at most one
  instruction per cycle, 4.17 ns at 240 MHz against QEMU's 4 ns, so even with no stall a real
  number is at least about 4 % higher. Code and constants in flash (prosdk, libm functions such
  as `hypot`, `sin` and `cos`) run through a cache, and cache misses cost tens of cycles on top.
  There is no fixed factor; it depends on the code's cache behavior.
- **double math is software on the ESP32-S3.** The S3's FPU is single precision only, so every
  double operation (all of wpimath, Eigen and Sleipnir) is a libgcc soft-float call of tens of
  instructions. That cost is in the counts here. The soft-float helpers (`__adddf3`, `__muldf3`,
  `__divdf3` and the rest) and `memcpy` that the benchmarks call are in the chip's mask ROM
  (0x4000xxxx in the ELF), not in flash, so the flash cache does not apply to them.
- **The point is relative numbers and regressions.** Instruction counts are deterministic for one
  binary. Compare two prosdk builds with the same benchmark (`--check`), or compare two operations
  with each other. Do not compare them with a deadline on a real robot.
- **Real numbers need a board.** The same sketch on a board, with esp_timer or CCOUNT at
  240 MHz, would give them. QEMU also does not run probot-core's WiFi, Driver Station or real
  hooks, nor any peripheral HAL path, so the robot loop here leaves out motor, encoder and network
  work.
- **Two cores share one instruction count.** They run round-robin on one TCG thread, so tasks on
  core 0 (`esp_timer`, IPC) take turns with the benchmark task instead of running in parallel, and
  their instructions, like those of the tick interrupts and of the `watchdog` and `prosdk` tasks
  on core 1, are counted in the benchmark's interval (about 1 %, see `instr_per_op` above).

## Open items

- **Late wake-ups in QEMU:** the bench task's `xTaskDelayUntil` wakes up to one tick late
  (`pass_jitter_max_us` 998 at 10 ms, 1027 at 5 and 2 ms; 3 at 20 ms), every other pass at 5 ms.
  The robot host is not the cause: every pass at 5 and 10 ms runs one loop at the same offset.
  Not identified: QEMU running both cores round-robin on one TCG thread (core 1's tick
  interrupt delivered late), or the priority-3 `prosdk` and `watchdog` tasks on core 1 at the
  wake tick. The tick and esp_timer also drift apart by about 150 ppm in QEMU (0.75 us per 5 ms
  pass, too small for this). A board run with the same fields would show whether it exists
  outside QEMU; on the ESP32-S3 the tick and esp_timer both come from SYSTIMER.
- **2 ms is marginal:** worst-pass headroom is 14 % in this build (20 %, 10 % and 29 % in
  earlier builds), the window had one overrun, and a wake-up more than half a pass late most
  likely makes that pass run two loops and the next none (996 loops in 1000 passes).
- **Scheduler heap:** about 1 KB per scheduled command, mostly the user's command objects and
  upstream's composition stack trace per decorator (`Command::SetComposed()`); 200 commands
  brought the minimum free heap to 19 KB. Reducing the stack-trace cost would change upstream's
  `Command` member layout and `GetPreviousCompositionSite()`, which is a design decision.
- **Test-only API:** the bench uses `pf::detail::ResetRobotMainForTest()` to run several robots in
  one boot. That is the test-only reset path, which a robot never uses.
- **No hardware paths:** QEMU has no MCPWM, PCNT, LEDC, RMT, TWAI, GPSPI or I2C devices, so no
  motor, encoder or sensor bus time is in these numbers. The driver benchmarks cover decode only.
- **CCOUNT:** CCOUNT is not usable as a clock in QEMU (43.5 MHz busy, 40.0 MHz idle). A board
  run should use CCOUNT at 240 MHz for sub-microsecond numbers. `CalibrateClock()` already picks
  it when it is consistent.
- **Not in CI:** the CI `qemu` job (manual dispatch) still runs only the smoke test. Adding
  `tools/qemu/run.sh --bench --check` there is a separate decision.
