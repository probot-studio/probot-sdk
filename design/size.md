# Flash and RAM size of prosdk sketches

Date: 2026-10-22. Branch `prosdk-2027`. Toolchain: Arduino-ESP32 3.3.10, `xtensa-esp32s3-elf-gcc` 14
(esp-x32 2601), `esp32:esp32:esp32s3:PartitionScheme=huge_app` (3,145,728 B app, 327,680 B static
RAM). probot-core: `.upstream/probot-core` `sdk-target` at `4484a97`.

Question: where do the 1.95 MB (`TimedRobotDifferential`) and 2.14 MB (`CommandBasedRobot`) of flash
go, why did the wave-3 robot examples grow by 19-36 KB in wave 4, and what can be removed without
removing API or features and without a global flag change that recompiles the whole archive?

## 1. Method

- Baseline archive: `src/esp32s3/libprosdk.a` built from `dddef2f` (2026-10-22 09:21, md5
  `84ab2c7459e6382f9a86b11c8e85dd52`), copied aside. Baseline sketch builds:
  `tools/build/build.sh examples TimedRobotDifferential CommandBasedRobot`.
- Variants: the changed TUs are compiled with the archive's exact compile command (from
  `ninja -t commands`), replaced in a copy of the archive (`ar r`), and the example is relinked with
  arduino-cli's own link command (from `arduino-cli compile -v`) against that copy. The sketch
  objects, probot-core and the Arduino core stay identical, so a difference is only the change.
- Flash = `.iram0.text + .iram0.vectors + .dram0.data + .flash.text + .flash.rodata + .flash.appdesc`
  (what arduino-cli prints as "Sketch uses"); RAM = `.dram0.data + .dram0.bss + .noinit` ("Global
  variables"). Relinked baselines reproduce `build.sh examples` exactly.
- Attribution: the linker map (`-Wl,--Map`) is parsed per input section. Merged string sections
  (`.rodata.*.str1.1`), which GNU ld lists with overlapping ranges, are attributed proportionally to
  their pre-merge sizes; the parsed totals match `xtensa-esp32s3-elf-size -A` to within 1 %.
  Families are assigned by archive member and by demangled symbol.

## 2. Where the flash goes (baseline)

TimedRobotDifferential (TRD): 1,952,092 B flash / 70,628 B RAM. CommandBasedRobot (CBR):
2,135,008 B / 73,756 B.

| Family | TRD (B) | CBR (B) |
|---|---:|---:|
| ESP-IDF / newlib / WiFi / lwip (probot-core's AP + web server) | 940,810 | 935,871 |
| `.eh_frame` of all objects (see section 3) | 187,332 | 241,828 |
| probot-core Driver Station page (`MAIN_page`, uncompressed HTML) | 138,056 | 138,364 |
| libstdc++ floating `to_chars` (Ryu tables, `floating_to_chars.o`) | 129,944 | 130,007 |
| libstdc++ locale and facets (char and wchar_t, `locale_init.o`) | 121,470 | 122,099 |
| prosdk: std containers and strings (instantiations) | 45,096 | 52,779 |
| prosdk: tunables | 39,095 | 43,424 |
| Arduino core + libraries | 40,062 | 39,971 |
| prosdk: double-conversion | 36,564 | 36,282 |
| prosdk: dashboard / platform | 34,420 | 35,439 |
| prosdk: other | 32,334 | 41,233 |
| prosdk: `std::format` instantiations | 31,903 | 64,327 |
| sketch + probot-core code | 31,152 | 32,957 |
| prosdk: hal (ESP32 backend) | 21,595 | 21,539 |
| `.gcc_except_table` of all objects | 17,602 | 24,324 |
| prosdk: telemetry | 17,023 | 19,921 |
| prosdk: llvm / util | 16,074 | 18,281 |
| prosdk: wpilibc | 14,540 | 15,163 |
| libstdc++ other | 12,115 | 24,621 |
| prosdk: json | 10,345 | 10,332 |
| libstdc++ string instantiations | 8,378 | 8,649 |
| prosdk: commands2 | 0 | 32,580 |
| prosdk: datalog | 1,825 | 15,446 |
| prosdk: protobuf / nanopb | 1,045 | 1,043 |
| prosdk: struct schemas | 606 | 792 |
| prosdk: Eigen / Sleipnir | 0 | 0 |
| libstdc++ iostream (`sstream`, `ostream`, `ios`; part of "libstdc++ other") | under 1,000 | 9,950 |
| static initializers (`.init_array`, prosdk + sketch) | 31 | 30 |

Largest prosdk archive members in TRD (code + rodata + their unwind tables): `TunableRegistry` 43,507,
`ProbotTunableBackend` 36,822, `CoreOps` 36,350 (most of it is the first instantiation of shared
`std::format` templates, which the linker keeps from the first object that defines them),
`TelemetryRegistry` 18,543, `ProbotTelemetryBackend` 16,115, `json` 13,368, `DriverStationBackend`
11,776, `Encoder` 11,214, `DriverStation` 10,633, `Synchronization` 9,436. CBR adds `DataLogManager`
50,994, `CommandScheduler` 20,436, `DataLogBackgroundWriter` 12,344, `DataLog` 10,629,
`SysIdRoutineLog` 5,894.

What this rules out:

- **iostream** is under 1 KB in TRD (no `ios_base::Init`, no `std::cout`). In CBR it was about 10 KB
  (`sstream`, `ostream`, `ios` instantiations), all pulled by one chrono `std::format` in
  `DataLogManager`; section 5 removes it.
- **Static initialization** is 31 B of `.init_array`. Nothing to gain.
- **Eigen, Sleipnir, `LinearSystemUtil`, `LinearSystemLoop`** are not linked into either example (0 B):
  archive linking pulls them only into sketches that use state-space classes.
- **protobuf/nanopb and struct schemas** are about 1 KB.
- **`std::format` itself** is the largest prosdk-attributable cost but is inherent: any formatting of a
  `double` links `floating_to_chars.o` (130 KB, of which the Ryu `POW10_SPLIT_2` table is 75 KB and
  `POW10_SPLIT` 29 KB, needed for precision formatting), and `std::format`'s locale support links the
  char and wchar_t facets (121 KB). Both are libstdc++ objects, linked once per sketch. Replacing
  `std::to_chars` would be an ODR hack against libstdc++; rejected.

## 3. Finding: unwind tables of discarded code stay in flash

`.eh_frame` is 187 KB in TRD and 242 KB in CBR, more than any prosdk library. Arduino-ESP32's linker
script (`tools/esp32s3-libs/3.3.10/<flash mode>/sections.ld`, line 743) places it inside the
`.flash.rodata` output section:

```
 . = ALIGN(4);
 __eh_frame = ABSOLUTE(.);
    KEEP(*(.eh_frame))
    LONG(0);
```

GNU ld edits `.eh_frame` (drops the FDEs of functions removed by `--gc-sections` or by COMDAT
deduplication) only when the output section is itself called `.eh_frame`. Here it is not, so every FDE of
every linked object stays, with `pc_begin = 0` for dead functions. Measured: TRD has 4,016 dead FDEs
(113,276 B), CBR 5,323 (150,016 B). In TRD the prosdk objects alone carry 77,600 B of dead FDEs
against 30,052 B of live ones. A minimal test link (one object, one gc'd function) confirms it: 160 B
kept with the `.flash.rodata` placement, 88 B with an own `.eh_frame` output section.

Consequences:

- The fix belongs in the linker script (an own output section named `.eh_frame`, placed in flash
  with `__eh_frame` at its start, so ld can edit it). It would save
  about 113 KB (TRD) and 150 KB (CBR) for every sketch, with no source change. It is outside this
  repository (arduino-esp32 / ESP-IDF); recorded as a recommendation, not done.
- Until then, pulling an archive member for one symbol links all of that member's unwind tables, live
  or not. This makes the per-member reductions below worth more than their code size suggests.
- `-fno-asynchronous-unwind-tables` gains nothing: on Xtensa GCC 14 asynchronous unwind tables are
  already off by default (`g++ -Q --help=common`), and a leaf function without calls gets no FDE.
  The FDEs are there because the archive is built with `-fexceptions` (WPILib throws), which is a
  feature and stays.

## 4. Wave-3 to wave-4 growth

Wave 3 measured at `84e4c43` (core `ab3697c`), wave 4 in the audit (`acc5abc`, core `71a5c19`). The core
sources are identical between the wave-3 measurement and `71a5c19`, the examples and the flags did not
change, so the growth is prosdk archive members that the example links.

TRD went from 1,926,441 B to 1,946,028 B (+19,587), `OpModeRobotExample` (OMR) from 1,904,333 to
1,940,332 (+35,999), `TimesliceRobotExample` from 1,902,981 to 1,922,336 (+19,355). Measured by
relinking TRD and OMR against the current archive with the wave-3 (`84e4c43`) builds of the members
that changed and are linked (`Console`, `PreferencesStore`, `RobotMain`, `Dashboard`, `SafeThread`,
ESP32 `DIO`, `DriverStation`, `SmartIo`) and without `DashboardPreferences`: TRD 1,932,713 B flash /
70,116 B RAM against 1,952,092 / 70,628 (19,379 B of the 19,587 B), OMR 1,904,465 / 65,772 against
1,940,332 / 66,268 (35,867 B of the 35,999 B; the OMR relink reproduces the audit's 1,940,332 exactly).
By member (map, code + rodata + unwind tables):

| Member | TRD (B) | OMR (B) | Why it is linked |
|---|---:|---:|---|
| `DashboardPreferences.cpp` (new) | 7,153 | 7,175 | `Dashboard.cpp` mirrors `/Preferences` on the DS as robust tunables (upstream's persistent NT table) |
| `TunableRegistry.cpp` | 78 | 16,250 | the robust-tunable paths `DashboardPreferences` uses; TRD already linked them for its own tunables, OMR has none |
| `PreferencesStore.cpp` | 5,909 | 5,883 | newly linked: the NVS store behind `DashboardPreferences` |
| ESP32 `DIO.cpp` | 2,985 | 2,985 | digital PWM generators on LEDC (`7e6a7db`); `DIO.cpp` is always linked (`InitializeDIO`, output gate) |
| ESP-IDF `ledc.c`, `ledc_hal_iram.c`, `esp32-hal-ledc.c` | 1,611 | 1,609 | the LEDC driver behind DIO PWM |
| `json.cpp` | 1,257 | 1,249 | the JSON reader's `get_*` accessors, used by `PreferencesStore` |
| others (`CoreOps`, `Tunables`, `Console`, `SmartIo`, `SafeThread`, ...) | about 400 | about 450 | small changes |

All of it is wave-4 features that are always on: the preferences mirror matches upstream, where
`/Preferences` is always published, and DIO PWM is part of the DIO HAL. Nothing is linked by accident,
so the growth stays. The 19 KB versus 36 KB difference is only whether the sketch already used robust
tunables.

Since the audit, core R8 (`4484a97`) added 4,955 B rodata, 1,116 B code and 4,384 B bss on the sketch
side (TRD 1,946,028 -> 1,952,092 B flash, 66,276 -> 70,628 B RAM; CBR 2,128,900 -> 2,135,008,
69,340 -> 73,756): not prosdk.

## 5. Reductions made

Each change was measured alone and all together by relinking against the baseline archive with only
the changed members replaced (section 1). AFP = `examples/AprilTagFieldPose` (2,102,008 B / 70,836 B at
baseline), the only example that links the AprilTag detector. Flash / RAM in bytes; the delta is against
the baseline.

| Change | TRD flash / RAM | CBR flash / RAM | AFP flash / RAM |
|---|---|---|---|
| baseline | 1,952,092 / 70,628 | 2,135,008 / 73,756 | 2,102,008 / 70,836 |
| 5.1 HAL module initializers as weak references | 1,938,024 / 70,356 (-14,068 / -272) | 2,120,896 / 73,476 (-14,112 / -280) | 2,083,048 / 69,956 (-18,960 / -880) |
| 5.2 `DataLogManager` file name without the chrono formatter | 1,952,092 / 70,628 (0 / 0) | 2,087,880 / 73,756 (-47,128 / 0) | not linked |
| 5.3 AprilTag code tables `const` | not linked | not linked | 2,102,016 / 65,908 (+8 / -4,928) |
| all three | 1,938,024 / 70,356 (-14,068 / -272) | 2,073,704 / 73,476 (-61,304 / -280) | 2,083,056 / 65,028 (-18,952 / -5,808) |

No API or feature is removed and no compile flag changed; only the five TUs involved recompile.

The AFP relink uses the sketch objects of the audit build (core `71a5c19`, before R8), which is why its
baseline RAM matches the audit's 70,836. Full builds of the working tree with the three changes
(`tools/build/build.sh esp32s3 prosdk header_check`, then `tools/build/build.sh examples
TimedRobotDifferential CommandBasedRobot AprilTagFieldPose`; core `4484a97`; the tree also held other
agents' uncommitted edits): TRD 1,938,788 B / 70,740 B, CBR 2,074,360 / 73,796, AFP 2,090,012 / 69,764.

### 5.1 HAL module initializers as weak references

`HAL_Initialize` called every module's initializer (`InitializeCAN`, `InitializeSerialPort`, ...), so
every sketch linked every HAL module with its ESP-IDF driver and unwind tables, whether it used the
module or not; SmartIo's e-stop handler did the same for CAN (TWAI). The ESP32 `HAL.cpp` now declares
the initializers of the optional modules (AddressableLED, AnalogInput, CAN, CANAPI, Counter, DutyCycle,
Encoder, I2C, Notifier, SerialPort) `[[gnu::weak]]` and calls each only when it is non-null;
`SmartIo.cpp` does the same for `can::OnEmergencyStop`. A weak reference never extracts an archive
member, so a module is linked exactly when the sketch (or a library it uses) calls one of the module's
functions, and then its initializer is defined and runs in `HAL_Initialize` as before. Every
initializer only sets up its own module's state (handle table; Notifier also starts its thread), so
nothing outside the module depends on it. A sketch that links no notifier code therefore no longer starts the
idle HAL notifier thread (one task fewer). `xtensa-esp32s3-elf-nm HAL.cpp.obj` shows the ten initializers as `w`.

Dropped from TRD (map): `SerialPort` 2,849, `DutyCycle` 2,137, `AddressableLED` 2,121, `I2C` 2,030,
`CANAPI` 1,688, `AnalogInput` 369, `Counter` 341 and the ESP-IDF ADC calibration code they pulled (about 2.6 KB).
AFP also drops `Encoder` 2,669 and `PulseCounter` 2,136. `CAN.cpp` stays linked in all three:
wpilibc's `RobotController.cpp` (linked for the battery voltage) references `HAL_CAN_GetCANStatus` from
`GetCANStatus` in the same TU, and archive extraction happens before `--gc-sections`. `Notifier` stays:
`TimedRobot` uses it.

### 5.2 `DataLogManager` file name without the chrono formatter (`wpilibc/0013`)

Upstream names the log `std::format("WPILIB_{:%Y%m%d_%H%M%OS}.wpilog", system_clock::now())`. The
chrono formatter instantiates the whole `std::chrono` formatting machinery with `ostringstream` and
locale support: in CBR `DataLogManager.cpp` shrank by 37,603 B and libstdc++'s `sstream`, `ostream`,
`streambuf`, `ios` and locale instantiations (9,950 B) dropped out. The patch formats the same text with
`gmtime_r` and `snprintf` (`FileTimestamp`). Same output: `%OS` is the locale's alternative seconds,
which libstdc++ formats with `strftime` (two digits, no fraction, unlike `%S`); compared on the host against
`std::format("{:%Y%m%d_%H%M%OS}")` for 2,000,000 random times with 0 mismatches.

### 5.3 AprilTag code tables `const` (`apriltag/0004`)

The AprilTag C library's `tag36h11.c` and `tag16h5.c` define `static uint64_t codedata[]` without
`const`, so the tables (4,696 B and 240 B) were copied to RAM at boot. The detector only reads
`apriltag_family_t::codes`, so the patch makes the tables `const` (they stay in flash) and casts
when it stores the pointer in the family struct. 4,928 B less static RAM in every sketch with the
detector; flash +8 B (alignment).

## 6. Not done, with numbers

- **Driver Station page compressed** (probot-core): `MAIN_page` is 138,439 B of uncompressed HTML in
  flash; `gzip -9` makes it 39,421 B. Serving it with `Content-Encoding: gzip` saves about 99 KB in
  every sketch. Core's code, so core request R14 (`design/core-requests.md`).
- **Linker script `.eh_frame`** (section 3): about 113-150 KB per sketch, arduino-esp32.
- **Split the explicit instantiations of the state-space classes** into one TU per instantiation.
  `LinearQuadraticRegulator.cpp` (12,612 B of `.eh_frame`), `KalmanFilter` (11,224),
  `SteadyStateKalmanFilter` (10,800), `LinearSystemUtil` (9,344), `LinearSystemLoop` (5,712) each
  instantiate several sizes in one TU, so a sketch that uses `LinearQuadraticRegulator<2, 1>` links the
  member and, because of section 3, the unwind tables of every size (`--gc-sections` drops the unused
  code, not its FDEs). Splitting needs patch-created files in the
  vendored tree (and `generated.txt` entries); it only helps sketches that use state-space control,
  which neither robot example does. Left for when such an example exists.
- **Sleipnir** (`sqp` 9,000, `variable_matrix` 8,840, `ipm` 7,144, `problem` 5,996 B of `.eh_frame`)
  and the UKFs (`S3UKF` 8,616, `MerweUKF` 8,524) are linked only by sketches that use them.

## 7. Global flags (measured on a few TUs only, not applied)

Measured by compiling three archive TUs with the archive's exact command plus one extra flag and
comparing the object sizes (code + literals + rodata + `.eh_frame` + `.gcc_except_table`):

| TU | `-Os` (current) | `-O2` | change |
|---|---:|---:|---:|
| `tunables/cpp/TunableRegistry.cpp` | 90,594 | 119,426 | +32 % |
| `commandsv2/cpp/CommandScheduler.cpp` | 30,055 | 34,437 | +15 % |
| `platform/hal/esp32/Encoder.cpp` | 39,114 | 57,957 | +48 % |

- **Keep `-Os`.** The archive (`tools/build/esp32s3.cmake` recipe) and the sketches (Arduino-ESP32's
  `compiler.optimization_flags`) are already `-Os`; `-O2` costs 15-48 % more per TU. A robot loop at
  50 Hz does not need the speed.
- **`-fno-asynchronous-unwind-tables`: no effect** (section 3): it is already off by default on Xtensa.
- **`-fno-exceptions`: not an option.** WPILib reports errors by throwing; it would remove a feature.
- The large remaining items are outside prosdk's flags: the linker script's `.eh_frame` placement
  (section 3, 113-150 KB), probot-core's uncompressed DS page (R14, 99 KB) and libstdc++'s `std::format`
  support (to_chars tables and locale facets, 250 KB, linked once).
