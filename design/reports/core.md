# probot-core v0.4.0 (dev) - investigation for the prosdk (WPILib port) layer

Source: `/srv/probot/core`, branch `dev`, HEAD `308849f` (stable = `e5907a5`). Read in full:
`API.md`, `CHANGELOG.md`, `llms.txt`, `README.md`, `CLAUDE.md`, `FUTURE_WORK.md`, all of `src/`
(5386 lines; 2203 of which are the `index_html.h` UI), plus the examples and test list.
Nothing was built. Line numbers refer to the files as they are at HEAD.

---

## 1. Package facts

| Item | Value |
|---|---|
| Library name | `probot` (Arduino `library.properties`, `includes=probot.h`, `depends=Adafruit NeoPixel`), PlatformIO `library.json` (Adafruit NeoPixel >= 1.12.0), ESP-IDF component (`CMakeLists.txt`: `INCLUDE_DIRS src`, `REQUIRES arduino`, defines `ARDUINO=10820 ESP32 ESP32S3`; `idf_component.yml`: idf >= 5.0, arduino-esp32 >= 3.0.0) |
| Form | **Header-only**. Only `.h/.hpp` under `src/`. Namespace `probot`. Entry `src/probot.h` |
| Version macro | **None** (`VERSION` file only; no `PROBOT_VERSION_*` in `src/`) |
| Language level | Uses C++17 inline variables, nested namespaces, `if` with lambdas; host tests compile with `-std=c++17` |
| RTTI/exceptions | No `dynamic_cast`, `typeid`, `throw`, `try`. One virtual interface (`IGamepadSource`), fine with `-fno-rtti` |
| Heap | Only `String ssid`/`ap_ssid_` in DS setup; hot paths are fixed buffers |
| Partition | UI is ~105 KB (`index_html.h` is 107,296 bytes) - README requires "Huge APP (3MB No OTA)" |
| Tests | `tests/test_{lifecycle,robot_state,io,telemetry,battery,status_led}.cpp`, custom harness, stubs for Arduino/FreeRTOS |

File map (`src/`):

| File | Lines | Role |
|---|---:|---|
| `probot.h` | 45 | Declares user hooks (4 strong, 6 weak), `robotInit()/robotEnd() = delete`, includes everything |
| `probot/core/core_config.hpp` | 22 | `CORE_UI=0`, `CORE_CTRL=1`, `PRIO_CTRL=4`, `PRIO_USER=1`, `STACK_CTRL=4096`, `STACK_USER=8192`, `USER_LOOP_PERIOD_MS` (20) |
| `probot/core/lifecycle.hpp` | 276 | Pure logic: `Stage`, `DesiredState`, `Hooks`, phase predicates, command acceptance, `PhaseMachine`, `Supervisor`, `isStalled` |
| `probot/core/runtime.hpp` | 341 | FreeRTOS plumbing: user task, sysloop, e-stop, `runtime_setup()`, **defines `setup()`/`loop()`** |
| `probot/core/status_led.hpp` | 41 | Pure LED colour policy `statusColor()`, `rslBlinking()` |
| `probot/core/wdt.hpp` | 28 | `wdt_init_no_idle()` |
| `probot/core/lock.hpp` | 61 | `core::Mux` (portMUX critical section on ESP32), `MuxGuard` |
| `probot/robot/state.hpp` | 107 | `Status`, `Phase`, `OpMode`, `StateSnapshot`, `StateService`, `state()` singleton, global flags |
| `probot/robot/system.hpp` | 33 | `io::gamepad()` singleton, `driverstation::start_driver_station()`, `g_driver_station` (includes DS header) |
| `probot/telemetry/telemetry.hpp` | 149 | 256 B text ring buffer + `probot::print/println/printf/clearTelemetry` |
| `probot/io/gamepad.hpp` | 94 | `GamepadSnapshot`, `IGamepadSource`, `GamepadService` |
| `probot/io/joystick_api.hpp` | 125 | `joystick_api::Joystick`, `Options`, `makeDefault()` |
| `probot/io/joystick_mappings/mapping.hpp` | 108 | `MappingProfile`, 4 built-in profiles, global active mapping |
| `probot/io/battery.hpp` | 366 | ADC divider / INA219/226 battery task, `setBatteryVoltage`, `currentAmps` |
| `probot/devices/leds/builtin.hpp` | 49 | Status NeoPixel `render()` (no user API) |
| `driverstation/esp32s3/driver_station_esp32.hpp` | 1051 | WiFi AP, httpd, all HTTP routes, owner model, captive portal, push task |
| `driverstation/esp32s3/ws_joystick.hpp` | 287 | `/joystick` WS handler, frame parse, broadcast |
| `driverstation/esp32s3/index_html.h` | 2203 | `const char MAIN_page[] PROGMEM` - the whole DS SPA |

---

## 2. Full public API

### 2.1 User hooks (global namespace, C++ linkage, declared in `probot.h`)

| Hook | Linkage | When it runs |
|---|---|---|
| `autonomousInit()` | weak, optional | once on INIT with AUTO selected (phase `AUTO_INIT` already published before the call) |
| `autonomousInitLoop()` | weak, optional | every user-task iteration while stage INIT |
| `autonomousStart()` | weak, optional | once on START, **before** phase `AUTO_RUN` is published and before the auto clock starts |
| `autonomousLoop()` | **strong, required** (PB-E201) | every iteration while stage RUN |
| `autonomousStop()` | **strong, required** | once on every exit from AUTO INIT/RUN (STOP, auto time expiry, DS timeout, e-stop) |
| `teleopInit/InitLoop/Start/Loop/Stop()` | same pattern | same for TELEOP |
| `robotInit()`, `robotEnd()` | `= delete` (PB-E106) | cannot be defined as free functions in any TU that includes `probot.h` |
| `setup()`, `loop()` | defined by the library (`runtime.hpp:333-341`, `inline __attribute__((used))`, `#if defined(ARDUINO)`) | user defining them = PB-E202 |

There is **no** hook that runs in STOPPED or TRANSITION (no `robotPeriodic`/`disabledPeriodic`
equivalent) and **no** test mode.

### 2.2 Functions / objects

| Symbol | Notes |
|---|---|
| `probot::runtime_setup()` | Called by `setup()`; for ESP-IDF builds the app calls it |
| `probot::emergencyStop()` | Sets `robot::g_estop_requested`; sysloop runs the terminal sequence. Safe from any task |
| `probot::print(const char*)`, `println(const char* = "")`, `printf(fmt, ...)` (format-checked, 128 B tmp, truncates), `clearTelemetry()` | Telemetry shortcuts |
| `probot::telemetry::{print, println, printf, clear, writeBytes, copyBuffer, getBuffer, getLength, getSeq}` | Raw buffer API |
| `probot::setBatteryVoltage(float)` | ESP32 only; writes state only if decivolts changed |
| `probot::io::battery::currentAmps()` | **Only exists when `PROBOT_BATTERY_ENABLED`** (ADC or INA); meaningful only for INA |
| `probot::io::battery::poll(uint32_t)`, `startTask()` | internal-ish, public names |
| `probot::io::joystick_api::makeDefault(const Options& = {})` -> `Joystick` | lightweight wrapper around `io::gamepad()` |
| `Joystick` methods | `getLeftX/LeftY/RightX/RightY`, `getLeftTriggerAxis/RightTriggerAxis` (0/1 only), `getA/B/X/Y`, `getCross/Circle/Square/Triangle`, `getLB/RB`, `getBack/Start/Options`, `getLeftStickButton/RightStickButton`, `getPOV()` (-1/0/90/180/270), `getDpadUp/Right/Down/Left`, `getRawAxis(i)`, `getRawButton(i)`, `isConnected()` (axisCount+buttonCount>0), `getSeq()`, `getMs()`, `getAxisCount()`, `getButtonCount()` |
| `joystick_api::Options` | `deadzone=0.08f` (hard cutoff, no rescale), `invertY=true`, `useMappingInvert=true` |
| `probot::io::joystick_mapping::{setActiveByName, setActive, getActive, findByName, mapping_*}` | profiles `logitech-f310`/`f310` (default), `standard`/`xbox`/`xbox-one`/`ds4`/`dualshock-4`/`ps`, `axis9-dpad`, `tuna-default`/`tuna`. **Global**, not per joystick |
| `probot::io::gamepad()` -> `GamepadService&` | in `robot/system.hpp`; `read()`, `write()`, `setTimeoutMs()`, `lastWriteMs()` |
| `probot::robot::state()` -> `StateService&` | documented as "advanced" in API.md: `read()` snapshot + setters + getters `status() phase() selectedMode() autoPeriodSeconds() autoStartMs()` |
| `probot::robot::g_loop_heartbeat_ms`, `g_ds_last_activity_ms`, `g_estop_requested`, `g_estop_latched`, `g_reboot_requested` | `inline volatile uint32_t` globals, undocumented but reachable |
| `probot::core::{isInitPhase, isRunPhase, isActivePhase, modeOfPhase, canSelectMode, canAccept*, inputsEnabled, phaseFor, isStalled, statusColor, rslBlinking}` | pure predicates, reachable but not documented as API |
| `probot::builtinled::render(r,g,b)` | internal (sysloop only); README: no user colour API |
| `probot::driverstation::detail::g_driver_station` | `DriverStation*`; public methods only `begin()`, `processDns()`, `expireOwnerIfIdle()`, `forceDisconnect()`; the `WsJoystick _ws` and httpd handle are **private** |

### 2.3 Configuration macros (must be defined before `#include <probot.h>`)

WiFi/DS: `PROBOT_WIFI_AP_SSID` (default "Probot" + MAC suffix + `#warning`), `PROBOT_WIFI_AP_PASSWORD`
(**required**, >= 8, PB-E101), `PROBOT_WIFI_AP_CHANNEL` (**required**, 1-13, PB-E102),
`PROBOT_WIFI_AUTO_CHANNEL` (0), `PROBOT_WIFI_AP_SSID_MAC_SUFFIX`, `PROBOT_WIFI_ENABLE_11B` (0),
`PROBOT_WIFI_PMF_REQUIRED` (0), `PROBOT_CAPTIVE_PORTAL` (1), `PROBOT_DS_OWNER_TIMEOUT_MS` (5000),
`PROBOT_DS_TIMEOUT_MS` (10000), `PROBOT_DS_TIMEOUT_FORCE_STOP` (1), `PROBOT_INPUT_TIMEOUT_MS` (500).
Lifecycle/safety: `USER_LOOP_PERIOD_MS` (20), `PROBOT_LOOP_DEADLINE_MS` (2000),
`PROBOT_HOOK_DEADLINE_MS` (10000, static_assert >= loop deadline), `PROBOT_WDT_TIMEOUT_S` (8),
`PROBOT_ESTOP_END_MS` (500), `PROBOT_ESTOP_ENABLE_PIN` (-1), `PROBOT_RSL_PIN` (-1).
LED: `NEOPIXEL_PIN` (3), `NEOPIXEL_COUNT` (1), `NEOPIXEL_BRIGHTNESS` (32).
Battery: `PROBOT_BATTERY_ADC_PIN` (1-10) + `_R_TOP_K`/`_R_BOT_K`, or `PROBOT_BATTERY_INA` (219/226) +
`_INA_ADDR` (0x40), `_INA_SDA/_SCL`, `_INA_SHUNT_MOHM` (100); `PROBOT_BATTERY_TRIM` (1.0).
Not configurable: `STACK_USER` (8192), `STACK_CTRL` (4096), priorities, cores, push period,
telemetry buffer size (256).

---

## 3. Lifecycle and threading model (exact)

### 3.1 Who defines what

- `setup()` (`runtime.hpp:334`) = `probot::runtime_setup()`; `loop()` (`:338`) = `delay(100)`.
  Both `inline __attribute__((used))`, only under `ARDUINO`.
- The Arduino `loopTask` (arduino-esp32 default: core 1, prio 1, 8 KB) keeps existing and just
  sleeps 100 ms per pass.

### 3.2 `runtime_setup()` order (`runtime.hpp:301-329`)

1. `Serial.begin(115200)`, `delay(200)`, banner.
2. Optional `PROBOT_ESTOP_ENABLE_PIN` -> OUTPUT HIGH; optional `PROBOT_RSL_PIN` -> OUTPUT HIGH.
3. `wdt_init_no_idle(PROBOT_WDT_TIMEOUT_S, true)`: `esp_task_wdt_deinit()` (result ignored) then
   `ESP_ERROR_CHECK(esp_task_wdt_init({timeout, idle_core_mask=0, panic=true}))`. Idle tasks are
   **not** watched. Gotcha: if anything subscribed a task to the TWDT before `setup()`, deinit fails
   and the `ESP_ERROR_CHECK` init aborts.
4. `driverstation::start_driver_station()` -> static `DriverStation ds` -> `ds.begin()` (WiFi AP,
   httpd, routes, DNS, WS, creates `ds_push` task).
5. `battery::startTask()` if `PROBOT_BATTERY_ENABLED`.
6. Create `user` task and `sysloop` task.

### 3.3 FreeRTOS tasks

| Task | Created by | Core | Prio | Stack | Period | Watchdog |
|---|---|---:|---:|---:|---|---|
| `user` | `runtime_setup` | 1 (`CORE_CTRL`) | 1 (`PRIO_USER`) | 8192 | `step()` then `vTaskDelay(USER_LOOP_PERIOD_MS)` -> **period = 20 ms + hook time** (not `vTaskDelayUntil`) | Not TWDT; software stall check by sysloop |
| `sysloop` | `runtime_setup` | 0 (`CORE_UI`) | 4 (`PRIO_CTRL`) | 4096 | `vTaskDelay(5)` -> ~200 Hz | **Only TWDT subscriber** (8 s, panic -> reboot) |
| `ds_push` | `DriverStation::begin` | 0 | 3 | 4096 | `vTaskDelay(250)` -> 4 Hz | none |
| httpd | `httpd_start` | 0 (`cfg.core_id=0`) | IDF default (tskIDLE+5 = 5) | 8192 | event driven | none |
| `battery` | `battery::startTask` (only if enabled) | 0 | 1 | 4096 | 50 ms tick, sample every 250 ms (5000 ms after 4 failures) | none |
| `estop` | `runEmergencyStop` (only on e-stop) | 1 | 1 | 8192 | one shot, then `vTaskSuspend` | sysloop timer `PROBOT_ESTOP_END_MS` -> `ESP.restart()` |
| Arduino `loopTask` | core | 1 | 1 | 8192 | `delay(100)` | none |
| System (not core's): WiFi, lwIP tcpip, esp_timer, arduino events | IDF/Arduino | mostly 0 | high | | | |

Core 1 is therefore shared by `user` (prio 1), `loopTask` (prio 1, idle) and `estop`; any sdk task
on core 1 with prio >= 2 would preempt user hooks.

### 3.4 Data flow of a command

`HTTP GET /robotControl` (httpd task) -> validates with `core::canAccept*` -> writes
`StateService.status` / `selectedMode` / `autoPeriodSeconds` -> sysloop `Supervisor::update()` every
5 ms turns `(status, selectedMode, phase)` into a `DesiredState{mode, stage}` packed into
`detail::g_requested_state` -> user task reads it at the top of every iteration ->
`PhaseMachine::step()` performs **at most one transition per iteration** and calls hooks ->
`publish()` writes `phase` into `StateService`.

So `status` = "last accepted command" (INIT=0, START=1, STOP=2) and `phase` = "what the user task
has actually completed" (STOPPED=0, AUTO_INIT=1, AUTO_RUN=2, TELEOP_INIT=3, TELEOP_RUN=4,
TRANSITION=5). The phase lags the status by up to one iteration (+hook duration).

### 3.5 `PhaseMachine::step` (`lifecycle.hpp:126-231`)

Per user-task iteration:
1. If requested != current -> `transition()` (always calls `neutralize()` = zero gamepad first):
   - to STOPPED/TRANSITION: if current is INIT/RUN call that mode's `stop()` (heartbeat primed for
     the 10 s one-shot budget), then set current, publish phase.
   - mode differs while INIT/RUN (not a legal protocol path): call old `stop()`, settle STOPPED.
   - to INIT: only from STOPPED/TRANSITION; set current, **publish phase first**, then `init()`.
   - to RUN: only from INIT of the same mode; call `start()`, then stamp `autoStartMs` (AUTO only),
     set current, publish phase, prime heartbeat.
2. If current stage != RUN -> `neutralize()` (gamepad zeroed every non-RUN iteration).
3. Stage INIT -> `initLoop()`; stage RUN -> `loop()`; STOPPED/TRANSITION -> **nothing**.
4. (runtime) heartbeat = now; `vTaskDelay(USER_LOOP_PERIOD_MS)`.

### 3.6 Semantics

- **Mode select** (`cmd=mode&val=auto|teleop`): only when phase is STOPPED/TRANSITION **and**
  status STOP (`canAcceptModeChange`), else 409 PB-E409.
- **INIT** (`cmd=init[&autoLen=N]`): same precondition. Sets `autoPeriodSeconds` if N>0 (clamped
  3600), status INIT, clears `deadlineMiss`.
- **START** (`cmd=start[&autoLen=N]`): only if phase is AUTO_INIT/TELEOP_INIT and status INIT.
- **STOP** (`cmd=stop`): accepted if phase INIT/RUN or status != STOP (aborts a pending INIT/START).
  Cooperative: takes effect after the current hook returns (<= one period, longer if the hook
  blocks).
- **Auto timing** (`Supervisor::update`): when status START, selectedMode AUTO, phase AUTO_RUN,
  `autoStartMs != 0`, `autoPeriodSeconds > 0` and `now - autoStartMs >= period*1000` ->
  `setControl(STOP, TELEOP)` and desired `{TELEOP, TRANSITION}`. The user task then runs
  `autonomousStop()` and publishes TRANSITION. TeleOp is only preselected; a new INIT is needed.
  The auto clock starts **after** `autonomousStart()` returns. `StateService` default period is 30 s;
  the UI default is 15 s and is sent as `autoLen`. TeleOp has **no** duration / match clock on the
  robot (the UI match clock is client-side only).
- **Stall (PB-E301)**: sysloop marks `deadlineMiss` if phase is INIT/RUN and heartbeat is older
  than `PROBOT_LOOP_DEADLINE_MS` (one-shot hooks are given `PROBOT_HOOK_DEADLINE_MS` by priming the
  heartbeat 8 s into the future). Effects: zero inputs, telemetry line once, red blinking LED,
  `inputsEnabled()` false, UI banner. Clears automatically when the hook returns. No kill, no reboot.
- **DS timeout (PB-E303/E304)**: if `g_ds_last_activity_ms != 0` and status != STOP and silent >
  `PROBOT_DS_TIMEOUT_MS` (10 s): FORCE_STOP=1 -> status STOP (cooperative stop path); always
  `forceDisconnect()` (release owner, zero gamepad, close all WS); reset activity to 0.
- **E-stop (PB-E302)**: `cmd=estop` or `probot::emergencyStop()` sets `g_estop_requested`. Sysloop
  `runEmergencyStop()`: latch `g_estop_latched`, enable pin LOW, **`vTaskDelete(user task)`**, if
  phase was INIT/RUN spawn `estop` task running that mode's `stop()` with a 500 ms deadline
  (timeout -> PB-E305 `ESP.restart()`), status STOP, telemetry/Serial line. Afterwards INIT/START
  return 409 "EMERGENCY STOPPED"; only `cmd=reboot` (sets `g_reboot_requested`, sysloop
  `ESP.restart()`) or power cycle clears it. **Gotcha**: since the user task is dead nobody
  publishes the phase again, so `state().read().phase` keeps its pre-e-stop value (e.g.
  TELEOP_RUN) while `status == STOP`; the only reliable e-stop indicator is
  `robot::g_estop_latched` (undocumented global) or the `estop` JSON field.
- **Owner loss**: owner idle > 5 s -> slot released, gamepad zeroed, `clientCount` 0.
- **Input gate** (`core::inputsEnabled`): joystick frames reach the gamepad only when phase is
  AUTO_RUN/TELEOP_RUN **and** status START **and** not `deadlineMiss`; otherwise a zero-count
  snapshot is written. So **user code cannot read the joystick in INIT, STOPPED, TRANSITION or while
  stalled** (unlike WPILib disabled mode).

### 3.7 LED / RSL (`status_led.hpp`, `runtime.hpp:182-193`, 2 Hz from sysloop)

E-stop solid red; stalled blinking red; STOPPED solid blue (no DS) / blinking blue (`clientCount>0`);
INIT solid yellow; TRANSITION blinking yellow; AUTO_RUN blinking orange; TELEOP_RUN blinking green.
RSL blinks only in RUN phases, solid otherwise. No user colour API (removed in 0.3.0).

---

## 4. Robot state model and getters

`probot::robot::StateSnapshot` (`state.hpp:13-24`): `ms, seq, status, phase, selectedMode,
batteryVoltage, autoPeriodSeconds, autoStartMs, clientCount, deadlineMiss`.
`StateService` is a double buffer guarded by `core::Mux` (portMUX critical section) for both reads
and writes; every setter bumps `seq` (usable as a change counter).

**Public getter exists**: `probot::robot::state().read()` (documented in API.md "Robot durumu
(ileri seviye)"), plus convenience `status()`, `phase()`, `selectedMode()`, `autoPeriodSeconds()`,
`autoStartMs()`. Missing: no `isEnabled()/isDisabled()/isAutonomous()/isTeleop()/isTest()`
wrappers (the predicates exist as undocumented `probot::core::isRunPhase` etc.), no e-stop getter
(only the raw global), no DS-attached getter (only `clientCount` 0/1 which means "owner IP set",
not "WS open"), no match-time getter (the auto remaining time is computed in the private
`DriverStation::computeAutoRemainingMs`; reproducible from `autoStartMs`/`autoPeriodSeconds`), no
alliance / station / match number / event / game-specific message, no FMS concept.

---

## 5. Joystick data model

- **One gamepad only.** `GamepadService` holds a single `GamepadSnapshot{ms, seq, axisCount,
  buttonCount, float axes[20], bool buttons[20]}`. The UI sends exactly one source (selected
  physical gamepad via `<select id="gamepadSelect">`, or the virtual keyboard/touch source).
- **Wire**: WS binary `'J'` (0x4A): `[1]=nAxes<=20`, `[2]=nButtons<=20`, `[3]=reserved 0`,
  then `int16` big-endian axes (`round(v*32767)`), then bit-packed buttons LSB first. Frames with
  counts > 20 are rejected. HTTP fallback `POST /updateController` JSON `{"axes":[...],
  "buttons":[true,...]}` (512 B body limit).
- **Buttons are boolean only**: the UI sends `gamepad.buttons[i].pressed`, so analog trigger values
  (W3C buttons 6/7 `.value`) are lost; `getLeftTriggerAxis()` returns 0 or 1.
- **POV**: one POV, derived from mapping D-pad buttons (12-15 in the standard mapping), axis 9
  (`axis9-dpad`/`tuna-default`) or legacy buttons 4-7. Only 4 directions (no diagonals).
- **Rate**: the UI `inputLoop` runs on `requestAnimationFrame` and sends at most every 20 ms
  (`INPUT_SEND_INTERVAL=20`) whenever a source exists, i.e. ~50 Hz continuously (rAF is throttled
  when the tab is hidden). Without a source the client sends a 1-byte `'P'` keepalive every 2 s.
  A neutral frame is sent immediately on source change / gamepad disconnect / window blur.
- **Zeroing**: `GamepadService::read()` returns an all-zero snapshot (counts 0) when the last
  write is older than `PROBOT_INPUT_TIMEOUT_MS` (500). Also zeroed on owner release, DS timeout,
  stall, every non-RUN user-task iteration, and by the input gate in INIT/STOPPED.
- No joystick name/type, no axis/POV count beyond raw counts, no rumble/output channel, no
  per-port mapping, no button edge (pressed/released) helpers.

---

## 6. Telemetry transport

- `probot::telemetry` = one 256-byte ring buffer (`detail::g_buffer`) with `seq`, guarded by a
  portMUX. Writers anywhere; `printf` formats into a 128-byte stack buffer (longer output is
  truncated). Overflow silently drops oldest bytes. Library runtime errors write lines prefixed
  `"!! [PB-E3xx]"` into the same buffer (user spam can evict them).
- `ds_push` task (4 Hz) sends a `'T'` (0x54) WS frame containing the **entire** buffer snapshot
  whenever `seq` changed; the UI redraws the whole console each frame, colouring lines starting with
  `!!` as errors. HTTP fallback `GET /telemetry` (UI polls every 2 s only while the WS is down).
- Broadcasts go only to sockets whose peer IP equals the owner IP.
- There is **no structured key/value channel**, no typed values, no history, no log levels other
  than the `!!` convention.

---

## 7. Battery API

- Sources (compile-time, mutually exclusive, PB-E104/E105 checks): divider on ADC1 pin 1-10 with
  `_R_TOP_K/_R_BOT_K` (8-sample `analogReadMilliVolts` average, ADC_11db), or INA219/INA226 on
  `Wire` (100 kHz, `setTimeOut(20)`, config register written, bus V and shunt current).
- `battery` task samples every 250 ms, EMA alpha 0.2, x `PROBOT_BATTERY_TRIM`, publishes into
  `StateService.batteryVoltage` only on a decivolt change with >= 0.06 V hysteresis. After 4 failed
  reads publishes 0.0, PB-E306 once, polls every 5 s.
- Manual: `probot::setBatteryVoltage(v)` (do not combine with automatic sources).
- Read: `probot::robot::state().read().batteryVoltage` (0.0 = no data), `'S'` field `batt`
  (decivolt fixed point), `GET /getBattery` (open). Current: `probot::io::battery::currentAmps()`
  (symbol only exists when a battery source is compiled in; real value only with INA).
- The library claims the `Wire` bus in INA mode; user I2C devices must move to `Wire1`.

---

## 8. Error code system (PB-Exxx)

Bands: E1xx compile (`#error`/`static_assert`), E2xx link (documented in `probot.h` comments),
E3xx runtime (telemetry line with `!!` + Serial), E4xx HTTP. Each message carries
`probotstudio.com/docs/hatalar/#pb-eXXX`.

| Code | Meaning |
|---|---|
| PB-E101 | AP password missing / < 8 chars |
| PB-E102 | AP channel missing / out of 1-13 (0-13 with auto channel) |
| PB-E103 | SSID length (<= 25 with MAC suffix, <= 32 without) |
| PB-E104 | Battery ADC config (pin not ADC1, resistors missing/non-positive, pin collides with e-stop/RSL/NeoPixel) |
| PB-E105 | Battery source conflict / bad INA type / half-defined SDA/SCL / trim out of 0.5-2.0 / shunt / address |
| PB-E106 | `robotInit()`/`robotEnd()` defined (deleted functions) |
| PB-E201 | Missing required hook (link error) |
| PB-E202 | `setup()`/`loop()` defined by the sketch |
| PB-E301 | Loop/hook stall, inputs zeroed, held safe |
| PB-E302 | Emergency stop latched |
| PB-E303 | DS lost -> robot stopped (FORCE_STOP=1) |
| PB-E304 | DS lost -> joystick neutral, waiting (FORCE_STOP=0) |
| PB-E305 | E-stop `stop()` exceeded `PROBOT_ESTOP_END_MS` -> reboot |
| PB-E306 | INA battery sensor unreachable |
| PB-E409 | Command in wrong phase (HTTP 409 body) |

The registry lives in `API.md` + the docs site; codes are a stable public contract
(`CLAUDE.md`: never renumber). An sdk-side code band (e.g. `PS-Exxx`) would avoid collisions.

---

## 9. Driver Station web UI and protocol

### 9.1 HTTP endpoints (single httpd on port 80, `max_uri_handlers=24`, 18 used + 1 WS = 19, 5 spare; `max_open_sockets=10`; ctrl port 32768)

| Endpoint | Owner-gated | Response |
|---|---|---|
| `GET /` | yes | `MAIN_page` chunked (2 KB chunks) |
| `GET /joystick` (WS) | yes, at handshake and on every frame | see 9.2 |
| `POST /updateController` | yes | `"OK"` |
| `GET /robotControl?cmd=mode&val=auto\|teleop` / `init[&autoLen]` / `start[&autoLen]` / `stop` / `estop` / `reboot` | yes | `"OK"`, `"ESTOP"`, `"REBOOT"`, 409 text with PB-E409, 400 |
| `GET /setChannel?ch=0..13` | yes | `{"ok":b,"ch":N,"live":b}` (NVS namespace `probot`, key `ch`; CSA live switch, `csa_count=3`) |
| `GET /getState` | yes | `{"status","phase","selectedMode","autoPeriodSeconds","autoRemainingMs","batt","estop"}` |
| `GET /telemetry` | yes | buffer text |
| `GET /getBattery` | no | `"12.3"` |
| `GET /health` | no | `{"rssi","up","heap","dm","joyAgeMs","sta","disc"}` |
| `GET /info` | no | `{"ssid","ch","chSource","ip","chip","cpuMhz","sdk","totalHeap","totalFlash","sketchSize","freeSketch","psram"}` |
| Captive portal (if `PROBOT_CAPTIVE_PORTAL`): `/portal`, `/generate_204`, `/gen_204`, `/hotspot-detect.html`, `/connecttest.txt`, `/ncsi.txt`, `/redirect`, `/canonical.html` (302 to portal), `/wpad.dat` (404), 404 handler -> 302 portal; DNSServer on UDP 53 answering `*` | no | |

Owner model: first client IP becomes owner; other IPs get 403 / WS reject / are closed at broadcast
time (`wsPeerFilter`). Any owner request or WS frame refreshes `g_ds_last_activity_ms`.

### 9.2 WebSocket `/joystick` (binary, first byte = type)

- Client -> robot: `'J'` 0x4A joystick (section 5), `'P'` 0x50 keepalive. Any other first byte is
  silently ignored by `WsJoystick::parseFrame` (no extension hook). Max frame 4+40+3 = 47 bytes;
  bigger frames close the session.
- Robot -> client (from `ds_push`, 4 Hz tick): `'S'` 0x53 JSON, sent when `StateService.seq`
  changed or every 1000 ms (heartbeat): `status, phase, selectedMode, autoPeriodSeconds,
  autoRemainingMs, batt, rssi, up, heap, dm, estop, joyAgeMs, sta, disc` (352-byte buffer);
  `'T'` 0x54 telemetry text when `telemetry::getSeq()` changed.
- Sends serialized with a mutex (esp-idf #14495); per-fd 3 consecutive failures close the socket;
  `MAX_CLIENTS=13`. Client treats >= 5 s without frames as dead and reconnects (2 s backoff).

### 9.3 UI (`index_html.h`, one raw string `R"=====( ... )====="`, ~105 KB, vanilla JS, no external assets)

- Header nav with 3 pages: **Dashboard**, **Joystick**, **Logs**; overlays: touch drive view
  (`#driveOverlay`: two virtual sticks, ABXY, mode/Init/Stop/E-STOP), `#disconnectOverlay` (6 s no
  data), `#estopOverlay` (+ Reboot button), PB-E301 `#dmBanner`. Space bar = E-stop always.
- Dashboard widgets (`.widget[data-w=...]`), positioned by the single `LAYOUT` object on a
  12-column grid via `applyLayout()`:
  `phase {col1 span6 row1}`, `matchControl {col1 span6 row2}` (mode list Auto/Teleop, Init/Start
  button `#mcEnable`, Stop `#mcDisable`, auto duration stepper `#autoPeriod` default 15, countdown,
  E-stop), `statusStrip {col7 span2 row1}` (Comms / Robot Code / Joystick lights),
  `battery {col9 span2 row1}` (arc gauge, ~8 s average), `signal {col11 span2 row1}` (RSSI bars,
  ping, channel, clients), `telemetry {col7 span6 row2}` (console, Clear, Auto-scroll).
- Joystick page: Input Source cards (Gamepad / Keyboard WASD+arrows+1-4+Q/E / Touch), keymap,
  axes visualiser (4 axes), button grid, raw data with gamepad selector.
- Logs page: WiFi config + channel change, network status, History charts (battery/RSSI/ping, 2 min,
  canvas), Event Log (client-side DS events), ESP32 system info (`/info`).
- There is no generic widget/plugin mechanism; adding a widget means editing the HTML, the
  `LAYOUT` object and the protocol together in core.

---

## 10. Hardware resources used by core

| Resource | Usage |
|---|---|
| WiFi | softAP only (mode `WIFI_AP`; briefly STA for auto-channel scan), country TR ch 1-13 manual policy, HT20, `WIFI_PS_NONE`, TX power 20 dBm (`WIFI_POWER_21dBm` request), 11b rates disabled, optional PMF required. IP 192.168.4.1. STA connect/disconnect event handlers registered via `WiFi.onEvent` |
| Network | httpd :80 (+ctrl 32768), DNS UDP :53, TCP_NODELAY + keepalive on sockets |
| NVS | `Preferences` namespace `"probot"`, key `"ch"` |
| GPIO | `NEOPIXEL_PIN` (default **GPIO 3**; S3 DevKitC boards use 48), optional `PROBOT_RSL_PIN`, optional `PROBOT_ESTOP_ENABLE_PIN`, optional battery ADC pin (GPIO 1-10), optional INA SDA/SCL |
| RMT | Adafruit NeoPixel `show()` on arduino-esp32 3.x uses the RMT TX driver: **one RMT TX channel** on the NeoPixel pin, driven only from sysloop at 2 Hz |
| LEDC | **None.** Docs tell users to put servos on LEDC channel 7 via `ledcAttachChannel` and motors on `analogWrite` (auto-allocated low channels) |
| Hardware timers / esp_timer / ISRs | None |
| I2C | `Wire` (I2C0) claimed in INA battery mode only |
| ADC | ADC1 in divider battery mode only |
| UART | `Serial.begin(115200)` in `runtime_setup` |
| TWDT | Re-initialised: 8 s, panic, idle cores not watched, only sysloop subscribed |

---

## 11. Header-only implications for multi-TU sketches / libraries

Everything is `inline` functions, `inline` variables and function-local statics, so a single,
consistently configured definition set is ODR-safe. The problems come from macros and from the
library owning global symbols:

1. **Including `<probot.h>` (or `runtime.hpp`, `robot/system.hpp`) from any .cpp requires the WiFi
   macros in that TU.** `system.hpp` includes `driver_station_esp32.hpp`, which has `#error`
   PB-E101/E102 if `PROBOT_WIFI_AP_PASSWORD`/`PROBOT_WIFI_AP_CHANNEL` are undefined and emits a
   `#warning` for a missing SSID. A prosdk `.cpp` including `<probot.h>` will not compile unless the
   macros come from global build flags (PlatformIO `build_flags`, ESP-IDF compile definitions, or
   arduino-esp32's sketch-level `build_opt.h`). Arduino IDE `#define`s in the .ino do not reach
   library TUs.
2. **Silent ODR violations when macro values differ between TUs.** The inline bodies of
   `runtime_setup`, `sysloopTask`, `userTask`, `updateLed`, `setEnablePin`, `DriverStation::begin`
   etc. depend on `USER_LOOP_PERIOD_MS`, `PROBOT_RSL_PIN`, `PROBOT_ESTOP_ENABLE_PIN`,
   `PROBOT_DS_TIMEOUT_*`, `PROBOT_LOOP/HOOK_DEADLINE_MS`, `NEOPIXEL_*`, `PROBOT_CAPTIVE_PORTAL`,
   `PROBOT_BATTERY_*` (which even decides whether the battery task is started and whether
   `battery::currentAmps` exists). The linker keeps one arbitrary COMDAT copy, so a library TU
   compiled with defaults can silently override the sketch's configuration.
3. **`setup()`/`loop()` are emitted in every TU that includes `probot.h`** (`inline` +
   `__attribute__((used))`). Fine for COMDAT merging, but the sdk can never define its own
   `setup()`/`loop()` while anything includes `probot.h` (PB-E202 / redefinition).
4. **Hook ownership.** The four required hooks are strong: if prosdk defines `teleopLoop()` etc.
   (to drive a TimedRobot/CommandScheduler), a user who also defines them gets a multiple-definition
   link error, and a user who uses the sdk cannot omit them unless the sdk defines them. The six
   optional hooks are declared `weak`; a definition in a TU that saw that declaration is itself weak,
   so if both sdk and user define e.g. `teleopInit()` the linker silently picks one.
5. **`void robotInit() = delete; void robotEnd() = delete;`** at global scope: any TU including
   `probot.h` cannot declare/define free functions with those names (WPILib-style member functions
   `TimedRobot::robotInit` are not affected).
6. **`const char MAIN_page[] PROGMEM`** is a namespace-scope `const` (internal linkage): every TU
   that includes `probot.h` compiles its own ~105 KB copy. `--gc-sections` drops unreferenced copies
   so flash is fine, but compile time and RAM of the compiler grow per TU.
7. Transitive includes in every TU: `WiFi.h`, `esp_wifi.h`, `esp_http_server.h`, `lwip/sockets.h`,
   `Preferences.h`, `DNSServer.h`, `Adafruit_NeoPixel.h`, `Wire.h` (INA mode).
8. **Headers that are safe to include without configuration**: `probot/core/lock.hpp`,
   `probot/robot/state.hpp`, `probot/core/lifecycle.hpp` (needs only its own deadline macros;
   static_assert), `probot/core/status_led.hpp`, `probot/telemetry/telemetry.hpp`,
   `probot/io/gamepad.hpp` (class only), `probot/io/joystick_mappings/mapping.hpp`.
   **Not safe**: `probot/io/joystick_api.hpp` alone (its `makeDefault()` calls `io::gamepad()`, which
   is declared only in `robot/system.hpp` -> DS header -> macro errors), `robot/system.hpp`,
   `core/runtime.hpp`, `probot.h`, `io/battery.hpp` (per-TU battery config). So an sdk .cpp can read
   `robot::state()` and write telemetry without the macros, but cannot reach the gamepad singleton,
   `emergencyStop()` or `setBatteryVoltage()` without them.

Practical consequence for prosdk today: either keep all core-touching code in headers included only
from the sketch TU (header-only sdk, like core), or require global build flags; and prosdk cannot
own the hooks without colliding with user hooks.

---

## 12. What a WPILib-style layer needs from core that is missing

Ordered by how blocking it is for the port.

| # | Need | Current state | Suggested core change |
|---|---|---|---|
| 1 | **Every-cycle hook for the sdk independent of user hooks** (WPILib `robotPeriodic`, `CommandScheduler.run()`, telemetry/alert flush, `runsWhenDisabled` commands) | Nothing runs in STOPPED/TRANSITION; in INIT/RUN only the user hook runs | `probot::lifecycle::setPeriodicCallback(fn)` (or weak `probotFrameworkPeriodic()`) called from `userTask` every iteration, all phases, before/after the user hook, covered by the stall heartbeat |
| 2 | **Let the sdk own the lifecycle hooks** (TimedRobot dispatch: `disabledInit/Periodic/Exit`, `autonomousInit/Periodic/Exit`, `teleop*`, `test*`, `*Exit`) | 4 strong required hooks + `robotInit/robotEnd = delete`; no registration API | `probot::setHooks(const core::Hooks&)` or a framework-hooks struct registered before `runtime_setup()`, plus a `PROBOT_FRAMEWORK_OWNS_HOOKS` mode that drops the required strong references; transition callbacks `onPhaseChange(prev,next)` |
| 3 | **Joystick readable while disabled/INIT** (auto selection, button bindings in disabled, `DriverStation.getStickButton` in disabled) | Input gate writes zeros unless RUN+START+!stall; `neutralize()` every non-RUN iteration | Keep raw last frame in a separate buffer (`gamepadRaw()`), keep the gated one for actuator safety; expose `isEnabled()` for output gating |
| 4 | **Multiple joysticks (WPILib 6 ports)** | One `GamepadService`, UI sends one source | Use the reserved byte `[3]` of `'J'` as port index, `GamepadService[N]` (N=4..6), UI port assignment list |
| 5 | **Analog triggers, POV diagonals, joystick name/type** | Buttons bool only; 4-way POV; no metadata | Send W3C triggers as extra axes or button values; send gamepad id/type in a metadata frame |
| 6 | **Public typed status API**: `isEnabled/isDisabled/isAutonomous/isTeleop/isTest/isEStopped/isDSAttached/getMatchTime` | Only `state().read()`; e-stop only via undocumented `g_estop_latched`; phase stale after e-stop; no teleop/match time | `probot::ds::` getters built on `core::` predicates; publish a `ESTOPPED` view or set phase on e-stop; match time for both modes (optional teleop length from UI) |
| 7 | **Test mode** (LiveWindow, test commands) | `OpMode` = AUTO/TELEOP only | Add `OpMode::TEST` + phases (protocol change, Phase numbering is a contract) or document as unsupported |
| 8 | **Fixed-rate, configurable loop period** (TimedRobot `kDefaultPeriod` 20 ms, `addPeriodic`, loop-overrun Watchdog) | `vTaskDelay(USER_LOOP_PERIOD_MS)` after the work -> period drifts with hook time; macro only | `vTaskDelayUntil`, runtime-settable period, report overruns |
| 9 | **Bigger / configurable user stack** (command-based, kinematics, `std::function`, trajectories) | `STACK_USER = 8192` constexpr | `PROBOT_USER_STACK` macro (and for the e-stop task) |
| 10 | **Structured key/value telemetry channel** (SmartDashboard/NetworkTables-lite, typed numbers/bools/strings/arrays, struct-like Pose2d) | 256 B text ring, whole-buffer push at 4 Hz | New robot->DS frame type (e.g. `'K'`: topic id + type + value, delta updates, topic announce), bounded table, push-rate budget; UI widget area that renders entries |
| 11 | **DS -> robot tunable / command channel** (SmartDashboard `get*` with UI edits, `Preferences`, `SendableChooser` auto chooser, run-command buttons) | Unknown WS frame types are dropped; no endpoint for values | Client->robot frame (e.g. `'V'` set value / `'C'` invoke) with owner check, delivered to a registered sdk callback; optional NVS persistence in a separate namespace |
| 12 | **Alerts** (WPILib `Alert` error/warning/info, persistent, deduplicated) | Only `!!` telemetry lines and fixed banners (PB-E301, e-stop) | Alert list in the key/value channel + UI alert widget, or a dedicated frame |
| 13 | **Field2d / Mechanism2d / plots** | None | UI widgets fed by the key/value channel (pose arrays, ligament trees); or allow the sdk to ship its own page |
| 14 | **UI/protocol extensibility for the sdk** | `MAIN_page` monolithic; `WsJoystick` and httpd handle private; 5 free URI slots | Public `driverstation::registerUri()`, `sendFrame(type, data, len)` broadcast API, `onFrame(type, cb)` dispatcher for unknown types, optional extra page/asset served from the sdk |
| 15 | **Text log channel with levels / larger buffer** (`DriverStation.reportError/Warning`, DataLog) | 256 B shared buffer, `!!` prefix only | Larger configurable buffer (`PROBOT_TELEMETRY_BUFFER`), level prefix, incremental push instead of full snapshot |
| 16 | **Programmatic soft disable / request stop** | Only terminal `emergencyStop()` | `probot::requestStop()` (cooperative STOP from code) |
| 17 | **Output-enable predicate for MotorSafety / PWM disable** | `core::inputsEnabled()` exists but undocumented and input-oriented | `probot::outputsEnabled()` = RUN && !estop && !stall, documented as API |
| 18 | **RobotController data**: battery V (exists), input current (INA only, symbol missing otherwise), brownout flag, FPGA time | Partial | Always declare `currentAmps()` (return 0 / NaN), add brownout/low-battery flag, expose uptime in us |
| 19 | **Multi-TU friendliness** (prosdk is a multi-file library) | Section 11 | Split config-free API headers from one "runtime instantiation" TU (e.g. `PROBOT_RUNTIME_IMPL` guarded or a `probot_runtime.cpp`), move WiFi `#error`s there, make `MAIN_page` `inline constexpr`/`extern`, make `robotInit/robotEnd = delete` opt-out (`PROBOT_NO_LEGACY_HOOK_GUARD`), expose `io::gamepad()` from a config-free header |
| 20 | **Version macro for compatibility checks** | None | `PROBOT_VERSION_MAJOR/MINOR/PATCH` generated by `sync_version.py` |
| 21 | **Match info** (alliance, station, match number/type, event name, game message, FMS attached) | None | Optional fields in `'S'` + UI fields (or document as unsupported; FTC-style kit has no FMS) |
| 22 | **Rumble / DS outputs** (`GenericHID.setRumble`, `setOutput`) | None | Robot->DS frame mapped to Gamepad `vibrationActuator` (nice to have) |

Items that can be done purely in prosdk without core changes (workarounds):
- Read state/phase/battery: `probot::robot::state().read()`; e-stop: `probot::robot::g_estop_latched`;
  enabled = `core::isRunPhase(phase) && status==START && !deadlineMiss && !estop`.
- Auto match time: `autoStartMs + autoPeriodSeconds*1000 - millis()` in AUTO_RUN.
- Periodic work: an sdk FreeRTOS task (but it runs concurrently with user hooks on another
  priority, is not stopped by e-stop's user-task kill, and is not covered by the stall/heartbeat
  logic) - acceptable only as a stopgap.
- Hook ownership: sdk can ship a header that defines the 10 hooks and dispatches to a user
  `TimedRobot` instance, at the cost of users never defining hooks themselves; still nothing runs in
  STOPPED/TRANSITION.
- Persistent tunables: `Preferences` with a namespace other than `"probot"`.

---

## 13. Other gotchas worth knowing

- `status` vs `phase`: always use `phase` for "what is running"; `status` is the last accepted command.
- After e-stop, `phase` is stale (user task deleted, no publish).
- `USER_LOOP_PERIOD_MS` is a delay after the work, and the README example additionally does
  `delay(20)` inside `teleopLoop()` (effective ~40 ms + work).
- `probot::printf` truncates at 127 chars; telemetry overflow drops the oldest bytes, including
  library PB-E3xx lines.
- `GamepadService::read()` calls `millis()`; `Joystick` getters each take a fresh snapshot (each
  `getX()` locks the portMUX once; reading 20 values = 20 snapshots of ~130 bytes).
- `joystick_mapping` active profile is one global pointer shared by all readers.
- `sysloop` comment says DNS is processed at ~1 kHz; actual tick is 5 ms (~200 Hz).
- The INA path puts library I2C traffic on `Wire` from the battery task concurrently with user code.
- `wdt_init_no_idle` aborts via `ESP_ERROR_CHECK` if the TWDT was initialised with subscribers
  before `setup()` (relevant if prosdk ever touches the TWDT in a static constructor).
- `FUTURE_WORK.md` lists (core-owned) current telemetry in UI, bigger telemetry buffer, disconnect
  replay, ESP-NOW `IGamepadSource`, ESP32-C5 5 GHz - none of items 1-22 above are planned there.
