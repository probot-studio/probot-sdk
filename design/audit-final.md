# Final completeness audit: whole project

Date: 2026-10-22. Branch `prosdk-2027` at `91b1652` (origin/dev at `7109bf1`; the 8 commits from
`c4060fb` to `91b1652`, among them the examples port, are local only). Upstream: allwpilib
`b68effca4ddc9742ac208daa7dd1d11816137882`. probot-core: `.upstream/probot-core`, branch `sdk-target`
at `4484a97` (= `feat/multi-gamepad`, draft PRs #26-#31). Besides the untracked `CLAUDE.md`, the
working tree had no uncommitted changes when the final checks ran. Other agents committed
`f1530ab`..`91b1652` while the audit ran. The checks in section 1 were repeated on `91b1652` where
those commits touched their inputs. Only this file was written by the audit.

Questions: are the gaps and blockers of `audit-wave4.md` closed? For every C++ library in
allwpilib and every upstream example: ported fully, ported with documented differences, partially,
excluded with a recorded reason, or missing without one? What is still unjustified, and what must
happen before a stable release?

Short answer:

- **Wave 4.** All 6 unjustified gaps of wave 4 (section 7) are closed. Of the end-to-end blockers
  (section 6), the two user-facing rough edges, the R8 gamepad request and the CI backlog up to
  `7109bf1` are closed. Still open:
  - no run on a board (a QEMU smoke test now covers the parts that do not touch peripherals);
  - probot-core not released;
  - core requests R5 remainder and R9-R14 not in prosdk;
  - no prosdk release (section 2).
- **Libraries.** 11 upstream C++ libraries are vendored: drivers is ported fully, the other 10 are
  ported with documented differences. Every other upstream directory with C/C++ is excluded with a
  reason in PORT.md section 10 and API-DIFFERENCES.md section 5. In the 12 vendored roots, none of the
  4,674 upstream files lacks both a rule and a recorded reason (wave 4: 64 did). No library is missing
  without a reason (section 3).
- **Examples.** All 38 examples, 31 snippets, 12 templates and 8 command templates of
  `wpilibcExamples` have a decision in `design/examples.md`: 63 sketches, 71 items ported or merged,
  18 not ported with reasons. 2 of the 5 upstream example test programs are ported (section 4).
- **Builds and tests.** All of these pass:
  - vendoring check;
  - host: 19 test binaries;
  - ESP32 archive and header check are up to date (504 objects for 504 sources);
  - QEMU smoke test: 8/8 cases on both boots;
  - all 70 example sketches compile;
  - the 731-function HAL link test links (section 1).

  Nothing has run on an ESP32-S3 board.
- **Unjustified gaps left:** 4, all small (section 5):
  - the two upstream snippet tests;
  - NOTICE does not cover the ported examples;
  - the ported examples have no drift check against the pin;
  - stale status text in API-DIFFERENCES.md section 6 and one size in PORT.md section 10.
- **Release checklist:** section 6.

## 1. Method and evidence

All commands ran in `/srv/probot/sdk` on 2026-10-22 with `PROSDK_UPSTREAM` set to the pinned
checkout (`git rev-parse HEAD` = `b68effca...`).

1. **Vendoring drift check** (at `7d94844`; the later commits do not touch `tools/upstream` or any
   generated file):
   ```
   $ flock .build/vendor.lock nice -n 19 python3 tools/upstream/wpilib.py --check
   check OK: 2406 generated files in 16 modules match (apriltag, catch2, commandsv2, datalog, drivers,
   eigen, fields, gcem, hal, licenses, sleipnir, telemetry, tunables, wpilibc, wpimath, wpiutil)
   ```
2. **File classification.** A script loads `tools/upstream/modules/*.py` through
   `wpilib.load_modules()`. It is in the session scratchpad, not committed. It takes every file of the
   pinned tree from `git ls-tree -r HEAD`, so paths outside the sparse checkout count too, and assigns
   it to one class:
   - VENDORED: selected by a rule (`wpilib.select_files`);
   - EXCLUDED: matched by a rule's `exclude`;
   - NOTVEND: matched by a `NOT_VENDORED` key (exact path, directory prefix or glob, with `{a,b}`
     expanded);
   - UNCOVERED: none of the above.

   Result in section 3.2.
3. **Host build and tests** (incremental, at `91b1652`):
   ```
   $ tools/build/build.sh host all --test          # exit 0
   == prosdk_tests                    All tests passed (335 assertions in 24 test cases)
   == prosdk_dashboard_tests          All tests passed (432 assertions in 61 test cases)
   == prosdk_robot_tests              All tests passed (171 assertions in 7 test cases)
   == wpiutil_tests                   All tests passed (23221 assertions in 993 test cases)
   == telemetry_tests                 All tests passed (210 assertions in 35 test cases)
   == tunables_tests                  All tests passed (485 assertions in 77 test cases)
   == hal_tests                       All tests passed (392 assertions in 13 test cases)
   == datalog_tests                   All tests passed (205 assertions in 52 test cases)
   == units_tests                     All tests passed (1347 assertions in 177 test cases)
   == wpimath_tests                   All tests passed (172412 assertions in 799 test cases)
   == fields_tests                    All tests passed (46 assertions in 7 test cases)
   == apriltag_tests                  All tests passed (9 assertions in 8 test cases)
   == wpilibc_tests                   All tests passed (1574968 assertions in 609 test cases)
   == commandsv2_tests                All tests passed (705 assertions in 221 test cases)
   == prosdk_commands2_tests          All tests passed (46 assertions in 4 test cases)
   == drivers_tests                   All tests passed (275 assertions in 23 test cases)
   == prosdk_drivers_tests            All tests passed (192 assertions in 13 test cases)
   == example_ArmSimulation_tests     All tests passed (36 assertions in 1 test case)
   == example_ElevatorSimulation_tests All tests passed (9 assertions in 1 test case)
   ```
4. **ESP32 archive and header check** (incremental, at `91b1652`):
   ```
   $ tools/build/build.sh esp32s3 prosdk header_check
   [0/2] Re-checking globbed directories...
   ninja: no work to do.
   ```
   `xtensa-esp32s3-elf-ar t src/esp32s3/libprosdk.a` (built 13:51) lists 504 objects. Every
   `.c`/`.cpp` source under `src/` has an object with the same base name; that is 504 sources
   (generated.txt sources plus tracked hand-written ones), 0 missing.
5. **On-target smoke test in QEMU** (PORT.md 11.1) against that archive, at `91b1652`:
   ```
   $ tools/qemu/run.sh --boots 2 --timeout 600     # exit 0
   QEMU-SMOKE END boot=1 pass=8 fail=0
   QEMU-SMOKE END boot=2 pass=8 fail=0
   QEMU smoke: PASS (2 boots)
   ```
   An earlier run of this audit, on the same archive, passed too. Its boot 2 measured:
   - peak stack 12,684 / 24,576 B (math case) and 5,500 B (hosted `TimedRobot`);
   - heap free after all cases 277,508 B, minimum 238,612 B, largest block 204,788 B;
   - static TLS 20 B.

   A parallel verification agent ran the same test at 14:20 while the 70-sketch build loaded the
   server. Its boot 1 failed 3 timing checks of case d (`CaseRobot.cpp:272-275`: loop mean
   20,971 us against the 19,500-20,500 window, `AddPeriodic` mean 10,977 us, 82 fast vs 43 loop
   calls); boot 2 passed. The timing checks of the QEMU test depend on host load (QEMU runs on the
   host clock; PORT.md 11.1 already says it proves no real-time behaviour).
6. **ESP32 sketches.** The parallel verification agent ran `tools/build/build.sh examples` for all 70
   sketches in `examples/` (huge_app, against the archive above and `.upstream/probot-core`), started
   13:52, then the HAL link test. Its log is in the session scratchpad (`w5v/examples.log`); results
   in section 1.1. After `aba613a` changed `CommandBasedRobot`, this audit rebuilt that sketch itself.
7. **Upstream test macros** (`TEST_CASE`, `TEST_CASE_METHOD`, `TEMPLATE_TEST_CASE*`, `TEST`,
   `TEST_F`, ... at line start), counted with grep. Upstream: the `src/test/native` and
   `src/generated/test/native` trees. Vendored: `test/<module>` after patches. Results in
   section 4.2.
8. **Upstream example lists.** The `foldername` entries of `examples.json`, `snippets.json`,
   `templates.json` and `commands.json` were compared with the decision tables of
   `design/examples.md` (`diff`: identical) and with `ls examples/`.
9. **probot-core and CI.** Sources:
   - `git log` of the `sdk-target` clone;
   - `gh pr list -R probot-studio/probot-core` and `gh release list` of both repositories;
   - `gh run list -R probot-studio/probot-sdk`.

### 1.1 ESP32 sketch builds

```
$ tools/build/build.sh examples                  # 70 sketches, rc=0, 0 "error:" lines
$ python3 tools/build/hal_link_test.py --out <scratch>/hallink --compile      # rc=0
$ tools/build/build.sh examples CommandBasedRobot  # this audit, after aba613a
== CommandBasedRobot
Sketch uses 2074560 bytes (65%) of program storage space. Maximum is 3145728 bytes.
Global variables use 73796 bytes (22%) of dynamic memory, leaving 253884 bytes for local variables.
```

The 7 prosdk examples, flash / static RAM in bytes:

| Sketch | Flash | Static RAM |
|---|---|---|
| HalSmoke | 1,685,149 | 71,740 |
| TimedRobotDifferential | 1,938,980 | 70,740 |
| OpModeRobotExample | 1,933,104 | 70,732 |
| TimesliceRobotExample | 1,915,364 | 70,668 |
| CommandBasedRobot | 2,074,560 | 73,796 |
| DriversShowcase | 1,991,108 | 71,308 |
| AprilTagFieldPose | 2,090,204 | 69,764 |

Largest sketch: SimpleDifferentialDriveSimulation, 2,280,768 B (73 % of the huge_app partition).
Most static RAM: 74,436 B (MecanumDrivePoseEstimator, SimpleDifferentialDriveSimulation).

These figures match PORT.md 8.1 and all 63 rows of the examples.md size table as updated by
`91b1652` (compared with a script). None of the sketches was run.

## 2. Wave-4 gaps and blockers re-checked

### 2.1 audit-wave4.md section 7 (gaps not justified by a design document)

| Wave-4 item | Status at `91b1652` | Evidence |
|---|---|---|
| 1 upstream C++ examples/templates/snippets: no decision | **closed** | `design/examples.md` has one decision per upstream item (38 + 31 + 12 + 8) and 63 sketches (`c4060fb`, `7d94844`); its name lists are identical to the upstream JSON files. New finding: two upstream snippet tests are neither ported nor mentioned (section 5, item 1) |
| 2 desktop applications and infrastructure not named | **closed** | PORT.md section 10 names `glass`, `wpigui`, `imgui_suite`, `tools/sysid`, `wpical`, `datalogtool`, `outlineviewer`, `benchmark`, `developerRobot`, `shared/singlelib` and the Java-only `epilogue-*`, `wpiunits`, `wpiannotations`; so does API-DIFFERENCES.md section 5 |
| 3 API-DIFFERENCES.md said CAN errors on construction | **closed** | the pneumatics/PD/IMU row no longer names CAN; a separate `prosdk::CAN` row describes TWAI |
| 4 PORT.md listed a DS DataLog sink | **closed** | PORT.md section 10 now reads "DataLog sink to flash/SD (the DS sink ... is pending core R11)" |
| 5 64 non-C++ files without rule or reason | **closed** | `842d52a`; the classification finds 0 UNCOVERED in all 12 vendored roots (section 3.2) |
| 6 PORT.md 8.1 sizes stale; `build.sh examples` and the CI step uncommitted | **closed** | `build.sh examples` and the CI jobs `examples` and `qemu` are committed; sizes updated by `91b1652` and checked against the build (section 1.1) |

### 2.2 audit-wave4.md section 6 (missing for a user to run a robot end to end)

**1. No hardware run: open.** No sketch has run on an ESP32-S3 board; none is attached to this
server.

New since wave 4: the QEMU smoke test (`3c2e925`, PORT.md 11.1) runs the archive on the emulated
chip, 8/8 cases on both boots (section 1). It covers:
- Eigen, Sleipnir, LQR, Kalman filter and trajectory;
- exceptions and `std::format`;
- threads and `Notifier`;
- NVS `Preferences` and LittleFS `DataLogManager`;
- `CommandScheduler`;
- the hosted `TimedRobot` through disabled, auto, teleop and e-stop;
- stack, heap and TLS.

It found and fixed a real boot crash: the HAL's TLS block overflowed the IPC tasks (hal patch
0003).

Still unverified on a board:
- every peripheral path of the HAL backend;
- probot-core itself (WiFi, DS, gamepads, real hooks, e-stop timing);
- real-time behaviour;
- every `drivers/` device.

The kit electronics are not final (KIT.md section 3).

**2. probot-core not released: open.**
- PRs #26-#31 are still drafts, and #32 (0.4.0 release polish) is open.
- The latest probot-core release is 0.3.0 (2026-06-26).
- prosdk builds only against the read-only `sdk-target` clone (`4484a97`, `library.properties`
  version 0.4.0).
- CI uses `PROBOT_CORE_REF: "feat/multi-gamepad"`.

**3. Pending core requests: partly closed.**
- R8 is **closed**: delivered in #31 and wired (`d2e33cf`, `f1530ab`, CoreOps ABI 3). The HAL
  gets joysticks 0-5, analog triggers, an 8-way POV, the name and the standard-mapping flag, and a
  joystick dropped by core's timeout reads as disconnected. Rumble is still open (nice to have).
- R9 (#33) and R10 (#34) are delivered as core drafts, but they are not in `sdk-target` and **not
  wired**:
  - `ConsoleWrite` still uses `probot::telemetry::writeBytes` (`CoreBridge.hpp:250`);
  - `HAL_GetSystemTimeValid` still uses the 2024-01-01 heuristic (`DriverStation.cpp`).
- **Open, not delivered:**
  - R5 remainder: DS widgets `Field2d`, `Mechanism2d`, `Scheduler`;
  - R11: file download;
  - R12: `requestStop`, `rslState`, `commsDisableCount`, `currentAmps`, brownout flag,
    robust-tunable persistence;
  - R13: schema size;
  - R14: gzip DS page.

**4. Not pushed, no CI since `48beaf2`: partly closed.**
- CI passed on `dev` at `7109bf1` (run 39763011834, host + esp32s3 + examples) and at `2475301`.
- The 8 commits after `7109bf1` (63 example sketches, 2 example test programs, the R8 follow-up
  and documentation) are not pushed and have never run in CI. CI has therefore not yet built the 63
  ported sketches.
- Pushing needs Tuna's approval.

**5. No release: open.**
- `src/esp32s3/libprosdk.a` is still gitignored ("release artifact, not tracked for now").
- There is no GitHub release and no Library Manager entry.
- `library.properties` has version `2027.0.0-alpha.1`.
- README says "Not yet usable for robots".

**6a. `if consteval` warnings in every sketch with tunables: closed.** `842d52a` added the `cxx23`
transform; `grep -rn "if consteval\|if !consteval" src/` finds nothing.

**6b. Host tests write `.wpilog` files into `logs/`: closed.** Since `7109bf1` the tests run in
`.build/host/test-run`; there is no `logs/` directory in the repository after this audit's runs.

### 2.3 audit-wave4.md sections 3 and 4 (stubbed API and tests not run)

Changes since wave 4:

- **Closed:** Driver Station joysticks 1-5 and analog triggers (sections 3.2 and 3.4 of wave 4).
- **Closed:** the upstream example tests `ArmSimulation` and `ElevatorSimulation` run on the host
  (`example_*_tests`). `UnitTest` (4 cases) is not ported, with a recorded reason: the example is
  about pneumatics.
- **Correction of wave 4 section 4.** The 14 upstream example test macros are in five programs, not
  three:
  - `examples/ArmSimulation` (1), `examples/ElevatorSimulation` (1), `examples/UnitTest` (4);
  - `snippets/DigitalCommunication` (4), `snippets/I2CCommunication` (4).

  The two snippet tests are an open gap (section 5).
- **Unchanged and documented** in API-DIFFERENCES.md section 3 and hal-esp32.md. The ESP32
  stubs are:
  - pneumatics, PowerDistribution and OnboardIMU;
  - DIO pulses and PPS;
  - SimDevice and `HALSIM_*`;
  - RSL state, comms-disable count and 3.3 V current;
  - serial flow control and buffer sizes;
  - rumble, LEDs and touchpads;
  - alliance, match and game data;
  - team number and comments;
  - the `DashboardOpMode` no-op and the field images.
- **Hand-written messages.** Every "not supported" message in the hand-written code
  (`src/prosdk/platform`, `drivers`, `probot`, `cxx`) matches a documented limit. A grep for
  TODO/FIXME/not implemented/not supported found no TODO or FIXME left.

## 3. Library classification

### 3.1 Upstream libraries and directories

| Upstream | Status | Where recorded |
|---|---|---|
| wpiutil | ported with documented differences (10 patches); mmap, JNI, argparse, debugging, MessagePack, RawFrame and upb are excluded | API-DIFFERENCES.md 3, 4 |
| telemetry, tunables | ported with documented differences (1 patch each; the DS backends are hand-written over core's dashboard store). The DS display is partial until the core R5 remainder: `Field2d`, `Mechanism2d` and `Scheduler` show as a topic tree | API-DIFFERENCES.md 3; core-requests.md R5 |
| wpimath (incl. units, Eigen, gcem, Sleipnir, struct, protobuf, json) | ported with documented differences (wpimath 4, Eigen 1 and Sleipnir 3 patches, plus config switches) | API-DIFFERENCES.md 3, 4 |
| hal | ported with documented differences (3 patches). Common code and the sim backend (host) are vendored; the ESP32 backend is hand-written (731 functions, link test). Device classes the kit does not have are stubs | hal-esp32.md, hal-esp32-functions.txt |
| datalog | ported with documented differences (3 patches): writer side only; the reader, the reader thread and `FileLogger` are excluded | API-DIFFERENCES.md 3, 4 |
| wpilibc | ported with documented differences (13 patches). The NT backends, `SystemServer` and ExpansionHub are excluded; `NetworkBooleanEvent` is re-implemented over tunables instead of NT; the robot entry point is the hosted `PROSDK_ROBOT_MAIN` | API-DIFFERENCES.md 3, 4; PORT.md 8.1 |
| commandsv2 | ported with documented differences (4 patches; `NetworkButton` over tunables; command names without RTTI) | API-DIFFERENCES.md 3 |
| drivers | **ported fully**: unchanged; only `empty.cpp` and the generators and build files are not vendored | drivers.md |
| fields | ported with documented differences (1 patch; field images not vendored) | API-DIFFERENCES.md 3, 4 |
| apriltag | ported with documented differences (4 patches; the OpenCV adapter and the image generator are excluded) | API-DIFFERENCES.md 3, 4 |
| thirdparty/catch2 | vendored for the host tests only | NOTICE |
| wpilibcExamples | ported as sketches, not vendored by the script (section 4) | examples.md |
| ntcore, wpinet | excluded: replaced by the probot-core DS link. An NT4 server for AdvantageScope/Elastic is "a later work item" | PORT.md 10, API-DIFFERENCES.md 5 |
| cscore, cameraserver | excluded: no camera on the kit | same |
| romiVendordep, xrpVendordep | excluded: other robots | same |
| simulation/halsim_gui, halsim_ds_socket, halsim_ws_core, halsim_ws_server, halsim_ws_client, halsim_xrp | excluded: dynamically loaded desktop plugins on wpinet | same |
| aos_sdk, aosnt, ntcoreffi | excluded: Linux/desktop libraries on ntcore/wpinet | same |
| commandsv3 | excluded: Java only (JVM continuations) | same |
| glass, wpigui, thirdparty/imgui_suite, tools/sysid, tools/wpical, tools/datalogtool, tools/outlineviewer | excluded: desktop GUI applications | same |
| benchmark, developerRobot, shared/singlelib | excluded: upstream build infrastructure | same |
| epilogue-runtime, epilogue-processor, wpiunits, wpiannotations | excluded: Java only | same |
| wpilibj, wpilibjExamples, robotpyExamples, allwpilib-java, javacPlugin | excluded: Java/Python ("JNI/Java/Python bindings (no JVM)"). This includes the two `src/dev/native/cpp/main.cpp` scratch files of wpilibj and wpilibjExamples | PORT.md 10 |
| cmake, gradle, buildSrc, shared/{bazel,java,javacpp,jni,plugins}, upstream_utils, styleguide, msvcruntime, docs, design-docs, .wpilib | build system and documentation; no library code | not listed individually |

**Missing without a reason: none.**

**Partially ported:** no library as a whole. Two pieces are partial: the DS display of three
telemetry types (core R5) and DataLog download from the DS (core R11). core-requests.md records both.

C/C++ files in the excluded directories, from `git ls-tree`:

| Directory | C/C++ files |
|---|---:|
| ntcore | 166 |
| wpinet | 306 |
| cscore | 106 |
| cameraserver | 11 |
| glass | 127 |
| wpigui | 10 |
| thirdparty/imgui_suite | 64 |
| tools/sysid | 41 |
| tools/wpical | 40 |
| tools/datalogtool | 10 |
| tools/outlineviewer | 2 |
| benchmark | 44 |
| simulation/halsim_* | 102 |
| aosnt | 8 |
| ntcoreffi | 3 |
| romiVendordep | 11 |
| xrpVendordep | 15 |
| developerRobot | 1 |
| shared/singlelib | 1 |
| wpilibj | 1 |
| wpilibjExamples | 1 |
| wpilibcExamples (ported as sketches, section 4) | 283 |

### 3.2 Files of the vendored libraries

Each cell lists vendored / excluded / not vendored / uncovered.

| Upstream root | C/C++ files | Other files |
|---|---|---|
| wpiutil | 217 / 14 / 185 / 0 | 1 / 0 / 154 / 0 |
| telemetry | 19 / 0 / 8 / 0 | 0 / 0 / 43 / 0 |
| tunables | 25 / 0 / 17 / 0 | 0 / 0 / 48 / 0 |
| wpimath (incl. Eigen, gcem, Sleipnir) | 1020 / 13 / 48 / 0 | 23 / 0 / 776 / 0 |
| hal | 159 / 52 / 75 / 0 | 0 / 1 / 124 / 0 |
| datalog | 14 / 7 / 4 / 0 | 0 / 0 / 54 / 0 |
| wpilibc | 362 / 22 / 12 / 0 | 0 / 0 / 249 / 0 |
| commandsv2 | 137 / 0 / 1 / 0 | 0 / 0 / 196 / 0 |
| drivers | 23 / 1 / 2 / 0 | 0 / 0 / 50 / 0 |
| fields | 29 / 0 / 2 / 0 | 18 / 0 / 59 / 0 |
| apriltag | 67 / 4 / 2 / 0 | 0 / 0 / 41 / 0 |
| thirdparty/catch2 | 290 / 0 / 0 / 0 | 0 / 0 / 6 / 0 |

Every upstream file in these roots has a rule or a recorded reason (wave 4: 64 uncovered).

## 4. Examples

### 4.1 Decisions (design/examples.md)

| Kind | Upstream | Ported | Merged | Not ported (reason) |
|---|---|---|---|---|
| Examples (`examples.json`) | 38 | 30 | 1 (the HAL C program, covered by HalSmoke) | 7: HatchbotTraditional, HatchbotInlined, RapidReactCommandBot, UnitTest (pneumatics); RomiReference, XRPReference, Xrptimed (other robots) |
| Snippets (`snippets.json`) | 31 | 25 | 0 | 6: AprilTagsVision, HttpCamera, IntermediateVision, QuickVision (camera); CANPDP (PowerDistribution is a stub); Solenoid (pneumatics) |
| Templates (`templates.json`) | 12 | 7 | 0 | 5: robotbaseskeleton (a blocking `StartCompetition()` cannot run inside core's loop); 4 Romi/XRP templates |
| Command templates (`commands.json`) | 8 | 0 | 8 (TemplateCommandItems) | 0 |

`examples/` has 70 folders (checked with `ls`): the 63 ported sketches and the 7 hand-written
prosdk examples.
- Every upstream item has exactly one row, and the names match the JSON `foldername` entries.
- Every "not ported" reason rests on a documented exclusion: the pneumatics and PD stubs, cscore,
  Romi/XRP, or the hosted loop.
- Spot check of fidelity: after the mechanical rename, `MecanumBot/Drivetrain.cpp` differs from the
  upstream file in one include spelling only (`"wpi/..."` -> `<prosdk/...>`).

Observation (not a gap): SwerveBot and SwerveDrivePoseEstimator are ported although they cannot run
on the robot (8 encoders, 4 PCNT units). The pneumatics examples are not ported, although they would
run in the host simulator. Both choices are recorded with reasons.

### 4.2 Upstream tests

| Module | Upstream test macros | Vendored after patches | Difference (reason) |
|---|---|---|---|
| wpiutil | 556 | 552 | JNI 2, argparse 1, debugging 1 (API-DIFFERENCES.md 4) |
| telemetry | 35 | 35 | |
| tunables | 77 | 77 | |
| wpimath | 939 | 939 | |
| hal | 13 | 13 | |
| datalog | 62 | 52 | DataLogReaderThread 2, FileLogger 8 |
| wpilibc | 343 | 294 | NT backends 47 and UnitNetworkTables 3 removed; ExpansionHub 2 removed; 2 TimedRobot cases replaced by 1; NetworkBooleanEvent 1 -> 5 |
| commandsv2 | 159 | 159 | 2 NT cases removed, NetworkButtonTest 1 -> 3 |
| drivers, fields, apriltag | 23, 7, 8 | 23, 7, 8 | |
| wpilibcExamples | 14 (5 programs) | 2 | UnitTest 4 (example not ported, reason recorded); **DigitalCommunication 4 and I2CCommunication 4: no reason** (section 5) |

This grep pattern differs from wave 4's, so some single numbers differ from that table. Within
each row both columns use the same pattern.

Every upstream test runs against the sim HAL with `PROSDK_TARGET_HW=0`. On the target, only the
QEMU smoke cases run.

## 5. Unjustified gaps left

1. **Upstream snippet tests not ported and not mentioned.** Two upstream test programs cover
   snippets that are ported:
   - `wpilibcExamples/src/test/cpp/snippets/DigitalCommunication` tests `SnippetDigitalCommunication`;
   - `wpilibcExamples/src/test/cpp/snippets/I2CCommunication` tests `SnippetI2CCommunication`.

   Each has 4 `TEST_CASE_METHOD`s (alliance, enabled, autonomous, alert). The "Example tests"
   section of `design/examples.md` says upstream has three example tests and does not mention these
   two. They would run on the host sim, where `DriverStationSim` sets the alliance (the robot's DS
   never sends one). Either port them like `test/examples/ArmSimulation` or record why not.
2. **NOTICE does not cover the ported examples.** NOTICE's component table does not name:
   - `examples/`, which now holds 63 sketches derived from `wpilibcExamples` (WPILib BSD-3; each
     `.ino` keeps upstream's copyright lines);
   - `test/examples`, which holds two upstream tests.

   Its "hand-written prosdk code" line also omits the hand-written part of `src/prosdk/drivers`,
   `src/prosdk/probot`, the 7 prosdk examples and `test/target`.
3. **Ported examples have no drift check against the pin.** The 63 sketches are hand copies of
   upstream files after the mechanical rename (examples.md "Porting rules"). They are not output of
   `tools/upstream/wpilib.py`, so `--check` does not cover them and an upstream pin bump updates
   nothing. Neither PORT.md section 2 ("Updating upstream") nor examples.md says how they are kept
   in step with the pin.
4. **Stale status text.**
   - API-DIFFERENCES.md section 6 ("Status of this file") stops at wave 4. It mentions neither the
     examples port nor the R8 joystick change, although the joystick change is described in
     section 3.
   - PORT.md section 10 still gives `examples/AprilTagFieldPose` as 2,102,184 B flash and
     70,836 B RAM. The current build gives 2,090,204 / 69,764 (PORT.md 8.1 already has the new
     figure).

Not counted as unjustified: the following are recorded with a reason, but they are still feature
gaps against the project goal "every WPILib 2027 C++ feature":
- NT4, AdvantageScope and Elastic connectivity (ntcore is "a later work item");
- DS widgets for `Field2d`, `Mechanism2d` and `Scheduler`;
- DataLog download from the DS;
- robust tunable persistence;
- RSL, comms-disable, current and brownout data;
- rumble.

They depend on probot-core (R5, R11, R12, R8) or on a scope decision (NT4).

## 6. Release checklist (before a stable prosdk release)

### 6.1 Hardware validation (needs an ESP32-S3 board and the final kit electronics)

1. **Electronics and pins.** Fix the kit electronics and pin map (KIT.md section 3: "Elektronik NET
   DEGIL"). Then update the ProHub constants, the I2C and serial defaults and the examples pin map
   (examples.md).
2. **HAL peripherals.** Flash `HalSmoke` and run every HAL peripheral on the board:
   - DIO in and out;
   - PWM (MCPWM, period groups, 12 outputs);
   - counters and encoders (PCNT, 32-bit extension, rate window);
   - duty-cycle capture;
   - ADC1 calibration;
   - AddressableLED (RMT);
   - I2C ports 0 and 1 (sharing `Wire` and the INA sensor);
   - UART1 and UART2;
   - CAN on TWAI with a transceiver;
   - LEDC digital PWM.
3. **Output safety.**
   - The gate closes within about 10 ms of a disable or a DS loss.
   - An e-stop drives every HAL output low, and the framework e-stop path stays inside core's
     500 ms budget (PB-E305).
   - The PCA9685 e-stop limitation is confirmed and documented for users.
4. **Hosted loop on real probot-core**, with WiFi and the DS connected:
   - `TimedRobot` at 20 ms and 10 ms, `AddPeriodic`, `TimesliceRobot`, and `OpModeRobot` (opmode
     list, UTILITY);
   - overrun counts;
   - headroom of the 24 KiB user stack and of the heap under load (QEMU: 12.7 KB peak stack,
     238 KB minimum free heap).

   Real-time behaviour is checked here, not in QEMU (see the timing flake in section 1).
5. **Driver Station end to end:**
   - 6 gamepads, analog triggers, 8-way POV and names;
   - telemetry and tunables topics, and the Selectable, Command, PIDController and Alerts widgets;
   - the Preferences mirror;
   - console text;
   - an exception halts the program with the error shown.
6. **Storage:**
   - NVS Preferences across power cycles;
   - LittleFS DataLog with real file sizes and old-log deletion.
7. **Every `drivers/` device on real hardware:**
   - PWM ESCs (Spark and the other PWM controllers) and Servo;
   - BTS7960 and VNH5019;
   - PCA9685 with ProHubMotor;
   - MPU6050 (calibration, drift);
   - GoBildaPinpoint;
   - the ProHub encoder stream at 921600 baud.
8. **A few examples on a robot:** TimedRobotDifferential, CommandBasedRobot, ArcadeDriveGamepad, one
   state-space example, and SysIdRoutine (the log is written and readable).

### 6.2 probot-core

9. **PRs #26-#31 merged and released** as probot-core 0.4.0 or later, on stable and in Library
   Manager. These carry R1-R4 (framework, ds, api), R5 (dashboard store and DS page), R7 (opmodes),
   R6 (loop rate) and R8 (gamepads). The latest release today is 0.3.0. core-requests.md holds the
   PRs until 0.4.0 itself is on stable, so the release that carries them may be numbered above
   0.4.0. Then:
   - change CI's `PROBOT_CORE_REF` from `feat/multi-gamepad` to the release tag;
   - set the `CoreBridge.hpp` gate to the released version number. Today it checks `__has_include`
     of the framework headers and `PROBOT_VERSION >= 0.4.0` (the `sdk-target` number), plus
     `__has_include` feature probes such as `PROSDK_CORE_HAS_LOOP_RATE`;
   - set `depends=probot (>=<that version>)` in `library.properties`.
10. **R9 and R10 wired in prosdk.** R9 (#33, console levels) and R10 (#34, wall clock) are merged,
    then:
    - `ConsoleWrite` calls `console::write` with levels;
    - `HAL_GetSystemTimeValid` calls `ds::systemTimeValid()`.
11. **R11-R14 delivered and wired, or explicitly deferred** with the limitation in the user docs:
    - R11: file download (`.wpilog` from the DS);
    - R12: `requestStop`, `rslState`, `commsDisableCount`, `battery::currentAmps`, brownout flag,
      robust-tunable persistence;
    - R13: schema size limit;
    - R14: gzip DS page (about 99 KB of flash per sketch).
12. **DS widgets `Field2d`, `Mechanism2d`, `Scheduler`** (R5 remainder), or explicitly deferred.

### 6.3 prosdk repository and packaging

13. Close the 4 gaps of section 5.
14. With Tuna's approval, push the local commits. CI must be green on `dev` for host, esp32s3 and
    examples (70 sketches), plus a manual `qemu` dispatch. Decide how the QEMU timing checks of
    case d tolerate a loaded runner (section 1, item 5).
15. Decide the branch and release layout. `main` still holds Probot Lib 0.2.x. prosdk is a new
    library name (`name=prosdk`) and needs its own Library Manager registration.
16. Package the precompiled archive for Library Manager (`precompiled=full`):
    - ship `src/esp32s3/libprosdk.a` in the release. Today it is gitignored, and at 51 MB
      (debug-stripped) it is above GitHub's 50 MB warning and under its 100 MB limit;
    - set the release version (now `2027.0.0-alpha.1`);
    - test a clean install from the release zip in a fresh Arduino environment (Arduino IDE and
      arduino-cli) that links the archive with the released probot-core.

    Building from sources inside a sketch (without the archive) has never been tested.
17. Record the final size table (PORT.md 8.1, examples.md) from the release archive.

### 6.4 Documentation

18. Publish user docs on probotstudio.com through probot-docs (`bp msg probot-docs`):
    - installation;
    - first robot (`PROSDK_ROBOT_MAIN`, no `setup()`/`loop()`);
    - the pin model (channel = GPIO, capacities);
    - Driver Station use;
    - API differences from WPILib (API-DIFFERENCES.md);
    - drivers;
    - examples.

    Today the docs mention probot-sdk only as a future library (`docs/ornekler.md:917`).
19. Update the README status line and examples list for the release, and complete NOTICE (section 5,
    item 2).

### 6.5 Scope decisions for Tuna

20. Decide whether an NT4-compatible server for AdvantageScope/Elastic is in the first stable release
    (ntcore is excluded as "a later work item").
