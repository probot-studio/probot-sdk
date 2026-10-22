# On-target smoke tests (QEMU)

`QemuSmoke/` is an Arduino sketch that runs prosdk's precompiled archive on an emulated ESP32-S3
(Espressif's QEMU, machine `esp32s3`): the real Xtensa LX7 instruction set, the real Arduino-ESP32
3.3.10 / ESP-IDF 5.5 binaries (FreeRTOS SMP, newlib, libstdc++, esp_timer, NVS, LittleFS), the
same compiler flags and the same partition table (huge_app) as a robot sketch. There is no
ESP32-S3 board on the build server; this is the closest it gets.

```
tools/qemu/setup.sh            # pinned QEMU (esp-develop-9.2.2-20260417, sha256) -> tools/bin/qemu
tools/qemu/run.sh              # compile, merge the 4 MB flash image, boot it twice, parse the UART
tools/qemu/run.sh --no-build   # reuse .build/qemu/QemuSmoke
```

`run.sh` exits 0 only when both boots print `QEMU-SMOKE END boot=<n> pass=8 fail=0`. The UART logs
are in `.build/qemu/uart-<n>.log`, the compile log in `.build/qemu/compile.log`.

## Without probot-core

probot-core needs WiFi, which QEMU does not emulate, so the sketch includes neither `<prosdk.h>`
nor `<probot.h>`. `FakeCore.cpp` installs a fake `CoreOps` table at static initialization, as the
sketch-side bridge (`prosdk/probot/CoreBridge.hpp`) does, and `CaseRobot.cpp` calls core's
framework hooks (`detail::Boot`, `PreCycle`, `PostCycle`, `EmergencyStop`) the way core's user
task and e-stop task do. Arduino's library discovery keys on a library's top-level header
(`prosdk.h`, which pulls in `<probot.h>`), so `run.sh` gives the sketch `-I<repo>/src` and links
`src/esp32s3/libprosdk.a` through `build.extra_libs`, which is what discovery of a precompiled
library adds. No libraries folder is passed: probot-core cannot leak in.

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

## Cases

Each case runs on its own FreeRTOS task with probot-core's user-task parameters as prosdk
configures them: 24576 bytes of stack (`PROBOT_USER_STACK` in `prosdk.h`), priority 1, core 1.
The `CASE` line reports the task's peak stack use and the heap change.

| id | what runs on the target |
|----|-------------------------|
| a | LQR gains (Eigen DARE) against upstream's test values, a Kalman filter, a Sleipnir problem compiled in the sketch, Sleipnir in the archive (`Ellipse2d::Nearest`, `ArmFeedforward`), the upstream 2018 spline trajectory, TrapezoidProfile, odometry |
| b | exceptions: thrown in the sketch and unwound through the archive's trajectory parameterizer, thrown in the archive (`std::runtime_error`), the wpilibc HAL error path (`DigitalInput{22}` -> `prosdk::RuntimeError`, also initializes the HAL), nested exceptions; `std::format` (floats, units, chrono); `prosdk::cxx::println` and the platform console (Driver Station console + UART) |
| c | `std::thread` under `ScopedThreadConfig` (name, priority, core, stack checked from inside the thread), mutex/condition variable, prosdk time against `esp_timer`, `prosdk::Notifier` periodic (10 ms for 1 s) and one-shot (50 ms) |
| e | `prosdk::Preferences` on NVS: boot 1 writes every type, boot 2 reads them back from the same flash image |
| f | `DataLogManager` on LittleFS (`/littlefs/logs`, the huge_app `spiffs` partition): writer thread, flush, the `.wpilog` read back; boot 2 finds boot 1's log |
| g | `CommandScheduler`: subsystem periodic, default command, sequence with a 100 ms wait, race, requirement interruption, the disabled rule |
| d | the hosted `TimedRobot` (`PROSDK_ROBOT_MAIN`, 20 ms loop + 10 ms `AddPeriodic`) through disabled -> autonomous -> disabled -> teleop -> disabled -> e-stop: lifecycle order, `RobotState` in each mode, the HAL output gate (`HAL_GetSystemActive`), loop and `AddPeriodic` spacing on esp_timer, core loop period requested by the robot (10 ms), nothing runs after the e-stop. Runs last (the e-stop latches) |
| h | every FreeRTOS task: priority, core, stack high-water mark; prosdk threads must be below priority 4 and no thread may use the default pthread config; the static TLS block (at most 64 bytes, see below); heap free / minimum / largest block |

## Measured (2026-10-22, `tools/qemu/run.sh --boots 2`, both boots `pass=8 fail=0`)

Peak stack of the 24576-byte case task and heap change per case (boot 1 / boot 2):

| case | stack used (bytes) | heap change (bytes) |
|------|--------------------|---------------------|
| a math | 12648 / 12520 | -260 |
| b exceptions, format, HAL init | 3688 / 3688 | -23336 (HAL initialized, stays) |
| c threads, Notifier | 2636 / 2636 (worker 1220 of 6144) | -264 |
| e Preferences | 2504 / 3116 | -924 |
| f DataLog | 3320 / 3384 | -3440 |
| g CommandScheduler | 3752 / 3752 | -9472 |
| d hosted TimedRobot | 5308 / 5308 | -21060 (robot stays registered) |

Heap free 327452 bytes at start, 269172 after every case, minimum ever 237032, largest free block
196596. Minimum free stack of the remaining tasks: `prosdk` (HAL notifier) 2800, `watchdog` 2924,
`esp_timer` 7740, `ipc0`/`ipc1` 240/248, `IDLE0`/`IDLE1` 184 to 224. Static TLS 16 bytes. The
timing numbers in the `CASE` lines are QEMU numbers, not ESP32-S3 numbers.

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
- real timing: QEMU's virtual clock follows the host clock and the guest runs at emulation speed,
  so the `*_us` numbers (pass time, Sleipnir solve time) are not ESP32-S3 numbers, and jitter
  depends on the load of the host. The timing checks use generous tolerances.
- PSRAM, dual-core contention under real interrupts, brownout, flash wear, power loss during an
  NVS or LittleFS write.
