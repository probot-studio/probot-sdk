# On-target smoke tests (QEMU)

`QemuSmoke/` is an Arduino sketch that runs prosdk's precompiled archive on an emulated ESP32-S3
(Espressif's QEMU, machine `esp32s3`): the real Xtensa LX7 instruction set, the real Arduino-ESP32
3.3.10 / ESP-IDF 5.5 binaries (FreeRTOS SMP, newlib, libstdc++, esp_timer, NVS, LittleFS), the
same archive and the same partition table (huge_app) as a robot sketch. There is no ESP32-S3 board
on the build server; this is the closest it gets. The board options differ from a robot sketch's
(`tools/build/arduino_recipe.py --fqbn`, the Boardoza Pulse settings): QEMU builds with
`FlashMode=dio`, USB CDC On Boot Disabled and no `BOARD_HAS_PSRAM` (`tools/qemu/run.sh`), so
`Serial` is UART0 here and the Pulse's `Serial` on USB-C (HWCDC) is not exercised.

```
tools/qemu/setup.sh            # pinned QEMU (esp-develop-9.2.2-20260417, sha256) -> tools/bin/qemu
tools/qemu/run.sh              # compile, merge the 4 MB flash image, boot it twice, parse the UART
tools/qemu/run.sh --no-build   # reuse .build/qemu/QemuSmoke
```

`run.sh` exits 0 only when both boots print `QEMU-SMOKE END boot=<n> pass=8 fail=0`. The UART logs
are in `.build/qemu/uart-<n>.log`, the compile log in `.build/qemu/compile.log`.

## Without probot-core

probot-core needs WiFi, which QEMU does not emulate, so the sketch includes neither `<prosdk.h>`
nor `<probot.h>`. `shared/FakeCore.hpp` installs a fake `CoreOps` table at static initialization, as the
sketch-side bridge (`prosdk/probot/CoreBridge.hpp`) does, and `CaseRobot.cpp` calls core's
framework hooks (`detail::Boot`, `PreCycle`, `PostCycle`, `EmergencyStop`) the way core's user
task and e-stop task do. Arduino's library discovery keys on a library's top-level header
(`prosdk.h`, which pulls in `<probot.h>`), so `run.sh` gives the sketch `-I<repo>/src` and links
`src/esp32s3/libprosdk.a` through `build.extra_libs`, which is what discovery of a precompiled
library adds. No libraries folder is passed: probot-core cannot leak in.

## Shared code and the speed benchmark (`QemuBench/`)

`shared/` holds what both sketches use: `FakeCore.hpp` (the fake core, header-only, installed
by a static initializer), `QemuUart.hpp` (`qemu::Line`/`Linef` to UART0) and `QemuTasks.hpp`
(the runner task, `RunUserTask` for a case or benchmark group on probot-core's user-task
parameters, the `TASK` lines, `kUserStack`, `HeapFree`). Arduino compiles only the sketch
folder, so `run.sh` adds `-I<repo>/test/target/shared`.

`QemuBench/` is the on-target speed benchmark with the same setup. It covers the hosted
`TimedRobot` under load at 20/10/5/2 ms through core's hooks, `CommandScheduler::Run()` with 0 to
200 commands, wpimath and Sleipnir, Boardoza driver decode paths, and heap and stack use. It
prints one `BENCH <name> iters= total_us= per_op_us= instr_per_op=` line per benchmark.

```
tools/qemu/run.sh --bench [--boots 2] [--check [--threshold 10]] [--save-baseline]
```

Its numbers are QEMU instruction counts (4 ns each), not ESP32-S3 times, so use them to compare
builds. `--check` fails on a regression above the threshold against
`tools/qemu/bench-baseline.json`. Results, caveats and the drivers it leaves out:
`design/qemu-bench.md`.

## QEMU-only settings

- `FlashMode=dio`: QEMU's SPI flash model does not take the QIO enable sequence (a default QIO
  build asserts in `esp_flash` init). The sketch links Arduino-ESP32's `dio_qspi` variant of the
  precompiled ESP-IDF libraries and bootloader instead of `qio_qspi`; prosdk's archive and the
  sketch code are unchanged.
- eFuse image (`.build/qemu/efuse.bin`, written by `run.sh`): QEMU's eFuses are all zero, and
  ESP-IDF's `esp_adc` constructor then finds no factory ADC calibration and self-calibrates on the
  SAR ADC, which QEMU does not model, so the app hangs before `setup()`. The image sets
  `BLK_VERSION_MAJOR = 1` (eFuse BLK2 bit 128), what a production chip carries. Nothing of this
  reaches a real chip.
- `-icount shift=2,sleep=off`: guest time is instruction-count time. Without it QEMU's virtual
  clock follows the host clock, so when the loaded host does not schedule the vCPU thread for
  30 ms, 30 ms of guest time pass between two instructions: periods are skipped and the timing
  checks of cases c and d measure the host's load (case d failed once at load 10.5). With it the
  clock advances 4 ns per executed guest instruction (about 250 MIPS, the order of a 240 MHz
  LX7) and jumps to the next timer deadline while both cores wait. The two cores run round-robin
  on one TCG thread. A boot pinned to one host CPU next to seven busy loops took 235 s instead of
  29 s and printed the same case c and d numbers; the same boot without `-icount` next to three
  busy loops failed 2 checks of case c and 5 of case d (26 late passes, loop spacing off by up to
  31 ms). Small differences between runs remain (a loop spacing of 2 ms instead of 1 ms in one
  of six boots), far inside the bounds below.

## Cases

Each case runs on its own FreeRTOS task with probot-core's user-task parameters as prosdk
configures them: 24576 bytes of stack (`PROBOT_USER_STACK` in `prosdk.h`), priority 1, core 1.
The `CASE` line reports the task's peak stack use and the heap change.

| id | what runs on the target |
|----|-------------------------|
| a | LQR gains (Eigen DARE) against upstream's test values, a Kalman filter, a Sleipnir problem compiled in the sketch, Sleipnir in the archive (`Ellipse2d::Nearest`, `ArmFeedforward`), the upstream 2018 spline trajectory, TrapezoidProfile, odometry |
| b | exceptions: thrown in the sketch and unwound through the archive's trajectory parameterizer, thrown in the archive (`std::runtime_error`), the wpilibc HAL error path (`DigitalInput{22}` -> `prosdk::RuntimeError`, also initializes the HAL), nested exceptions; `std::format` (floats, units, chrono); `prosdk::cxx::println` and the platform console (Driver Station console + UART; the fake core has no R9 leveled console, so an error-level write checks the text fallback) |
| c | `std::thread` under `ScopedThreadConfig` (name, priority, core, stack checked from inside the thread), mutex/condition variable, prosdk time against `esp_timer`, `prosdk::Notifier` periodic (10 ms for 1 s) and one-shot (50 ms) |
| e | `prosdk::Preferences` on NVS: boot 1 writes every type, boot 2 reads them back from the same flash image |
| f | `DataLogManager` on LittleFS (`/littlefs/logs`, the huge_app `spiffs` partition), mounted by prosdk's auto-mount (nothing mounts it before; the sketch does not include `<LittleFS.h>`): boot 1 finds the partition blank and formats it (`automount=formatted`), boot 2 mounts it without formatting (`automount=mounted`); writer thread, flush, the `.wpilog` read back; boot 2 finds boot 1's log; the fake core's file sharing (CoreOps ABI 8) records the folder shared as `logs` on the `datalogmgr` task and taken back on the task that called `Stop()` |
| g | `CommandScheduler`: subsystem periodic, default command, sequence with a 100 ms wait, race, requirement interruption, the disabled rule |
| d | the hosted `TimedRobot` (`PROSDK_ROBOT_MAIN`, 20 ms loop + 10 ms `AddPeriodic`) through disabled -> autonomous -> disabled -> teleop -> disabled -> e-stop: lifecycle order, `RobotState` in each mode, the HAL output gate (`HAL_GetSystemActive`), core loop period requested by the robot (10 ms), timing (below), nothing runs after the e-stop; then match info set on the fake core (CoreOps ABI 9) reaches `HAL_GetAllianceStation`/`HAL_GetGameData`/`HAL_GetMatchInfo` through the Driver Station pump without a packet, and is read once. Runs last (the e-stop latches) |
| h | every FreeRTOS task: priority, core, stack high-water mark; prosdk threads must be below priority 4 and no thread may use the default pthread config; the static TLS block (at most 64 bytes, see below); heap free / minimum / largest block |

### Case d timing

The fake core runs a pass every 10 ms on the 1 ms FreeRTOS tick grid (`xTaskDelayUntil`), as core
does. The bounds follow from that schedule, not from the host:

- no late pass (`overruns == 0`): in instruction-count time a late pass means the passes need
  more than 10 ms of guest instructions;
- without late passes the robot host runs every callback on the pass of its expiration (it looks
  half a pass ahead), so a stamp is that pass's tick plus a latency below half a pass (the pass
  work before the callback, plus up to a tick of round-robin with the priority-1 `loopTask` on
  core 1). Two consecutive stamps differ from the period by less than 5 ms; a skipped or doubled
  period is a whole period off;
- the mean over N stamps is then within 5 ms / (N - 1) of the period (about 110 us for the
  45 loop stamps of a window, 56 us for the 90 `AddPeriodic` stamps);
- a window holds window / period loop stamps, plus or minus one, and twice as many
  `AddPeriodic` stamps, plus or minus two.

Measured: loop 45 stamps, mean 20020 us, largest deviation 998 us (2011 us in one boot of six);
`AddPeriodic` 90 stamps, mean 10007 us; teleop loop mean 19998 us; 281 passes, the longest
2.4 ms; no late pass.

## Measured (2026-10-22, `tools/qemu/run.sh --boots 2`, both boots `pass=8 fail=0`)

Peak stack of the 24576-byte case task and heap change per case (boot 1 / boot 2):

| case | stack used (bytes) | heap change (bytes) |
|------|--------------------|---------------------|
| a math | 12544 / 12544 | -260 |
| b exceptions, format, HAL init | 3756 / 3756 | -16692 (HAL initialized, stays) |
| c threads, Notifier | 2672 / 2672 (worker 1220 of 6144) | -276 |
| e Preferences | 2428 / 3152 | -912 |
| f DataLog | 3424 / 3428 (2026-10-23, with file sharing) | -3412 / -3404 |
| g CommandScheduler | 3788 / 3788 | -9460 |
| d hosted TimedRobot | 5816 / 5816 (2026-10-23, with the match info checks) | -22772 / -22800 (robot stays registered) |

Heap free 328968 bytes at start, 279368 after every case, minimum ever 238548, largest free block
208884 (boot 1). Minimum free stack of the remaining tasks: `prosdk` (HAL notifier) 2804,
`watchdog` 4652, `esp_timer` 7836, `ipc0`/`ipc1` 248/240, `IDLE0`/`IDLE1` 228/220. Static TLS 16
bytes. Static RAM 44060 bytes, flash 1985122 bytes. The
timing numbers in the `CASE` lines are instruction-count numbers, not ESP32-S3 numbers.

## Found by this test: thread_local in the archive overflowed the IPC tasks at boot

ESP-IDF copies the static TLS block (every `thread_local` in the image) into the top of every
FreeRTOS task stack, the 1024-byte `ipc0`/`ipc1` tasks included. The HAL's per-thread last-error
storage (`thread_local LastErrorStorage`, a 512-byte `SmallString`) made that block 544 bytes, and
the first boot stopped with `A stack overflow in task ipc0 has been detected` before `setup()`
(a gdb watchpoint on the canary showed `pxPortInitialiseStack` overwriting it while creating
`ipc0`). Every example built against probot-core had the same 537 to 544 byte block, so a real
board would have stopped the same way. Fixed in `src/prosdk/impl/hal/cpp/ErrorHandling.cpp`
(vendor patch `tools/upstream/patches/hal/0003-*`): only a `std::unique_ptr` is thread-local, the
storage is allocated on a thread's first HAL error. Case h keeps the block at 64 bytes or less.

## What QEMU proves and what still needs a board

Proven by a passing run: the archive links and boots without probot-core symbols in the archive;
code generation, soft-double math and libm give upstream's numbers on Xtensa; C++ exception
tables and unwinding work across archive and sketch with `-fno-rtti`; libstdc++'s `std::format`
works on newlib; the stack and heap that the math, the robot loop and the scheduler need (the
numbers in the `CASE` lines); FreeRTOS task parameters from `ScopedThreadConfig`; NVS and LittleFS
through the ESP-IDF drivers on a flash image, persisted across a restart; the hosted robot
loop's control flow and timing on esp_timer and FreeRTOS ticks.

Not proven (QEMU does not emulate these, or not faithfully; they still need a board):

- every peripheral HAL path: PWM (MCPWM), LEDC, pulse counter (PCNT, encoders/counters), capture,
  RMT (AddressableLED), TWAI (CAN), ADC, I2C and UART devices, GPIO electrical behaviour, and
  every `drivers/` device. No case touches them; a HAL init of those paths in QEMU would only
  show that register writes do not crash.
- probot-core itself: WiFi AP, Driver Station WebSocket, gamepads, dashboard store, the real
  framework hooks, e-stop line and RSL, battery sensor. The `CoreOps` table here is a fake.
- real timing: guest time counts instructions at 4 ns each, without caches, flash wait states,
  pipeline stalls or interrupt latency, so the `*_us` numbers (pass time, Sleipnir solve time)
  are not ESP32-S3 numbers. The timing checks prove the schedule (which pass runs which
  callback), not how long the code takes on the chip.
- PSRAM, dual-core contention under real interrupts, brownout, flash wear, power loss during an
  NVS or LittleFS write.
