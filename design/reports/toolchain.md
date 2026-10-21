# prosdk toolchain / build investigation (ESP32-S3, Arduino-ESP32 3.3.10)

Read-only investigation, 2026-10-22. Host load average was ~16 on 8 cores during this check. No builds were run.
The only compiler calls were `-dM -E` and `-fsyntax-only` on 3-line files, run under `nice`.

---

## 1. Exact Arduino-ESP32 3.3.10 compile recipe for esp32s3 C++

### 1.1 Paths

| Item | Path |
|---|---|
| Core (platform) | `~/.arduino15/packages/esp32/hardware/esp32/3.3.10` (`platform.txt`, `boards.txt`, `cores/esp32`, `variants/esp32s3`, `libraries/*`) |
| SDK libs ("compiler.sdk.path") | `~/.arduino15/packages/esp32/tools/esp32s3-libs/3.3.10` (`flags/`, `include/`, `lib/` = 153 `.a`, `ld/`, `{qio,dio}_{qspi,opi}/`, `sdkconfig`, `versions.txt`) |
| Toolchain | `~/.arduino15/packages/esp32/tools/esp-x32/2601/bin/xtensa-esp32s3-elf-{gcc,g++,gcc-ar}`, which reports `xtensa-esp-elf-g++ (crosstool-NG esp-14.2.0_20260121) 14.2.0` |
| SDK versions | `versions.txt`: esp-idf **v5.5.4** (735507283d), lib-builder master 43a8f6d, arduino master 6cb835025 |
| Other tools | esptool_py 5.3.0, openocd-esp32 v0.12.0-esp32-20251215, xtensa-esp-elf-gdb 16.3_20250913 |

`compiler.prefix={build.tarch}-{build.target}-elf-` resolves to `xtensa-esp32s3-elf-`. The archiver is `xtensa-esp32s3-elf-gcc-ar` with flags `cr`.

### 1.2 The C++ object recipe (`platform.txt:153`)

```
g++ {compiler.cpp.extra_flags}
    -MMD -c @SDK/flags/cpp_flags {warning_flags} -Os -Werror=return-type
    -DF_CPU=240000000L -DARDUINO=<ide ver, e.g. 10607> -DARDUINO_ESP32S3_DEV -DARDUINO_ARCH_ESP32
    -DARDUINO_BOARD="ESP32S3_DEV" -DARDUINO_VARIANT="esp32s3" -DARDUINO_PARTITION_default
    {build.extra_flags}
    @SDK/flags/defines -I<sketch> -iprefix SDK/include/ @SDK/flags/includes -ISDK/<memory_type>/include
    {includes: -I cores/esp32 -I variants/esp32s3 -I <each lib src>}
    @<build>/build_opt.h @<build>/file_opts
    src.cpp -o src.o
```

- `warning_flags` defaults to `-w`. With `--warnings all` (the probot-core Makefile uses this) it becomes `-Wall -Wextra`.
- `build.extra_flags` = `-DARDUINO_HOST_OS="linux" -DARDUINO_FQBN="esp32:esp32:esp32s3" -DESP32=ESP32 -DCORE_DEBUG_LEVEL=0 {loop_core} {event_core} {build.defines} -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=0 -DARDUINO_USB_MSC_ON_BOOT=0 -DARDUINO_USB_DFU_ON_BOOT=0`. These are the generic `esp32s3` board defaults: usb_mode=1 (HW CDC), cdc_on_boot=0. `build.defines` is `-DBOARD_HAS_PSRAM` when the PSRAM menu is enabled.
- `memory_type = {build.boot}_{build.psram_type}`, which defaults to **`qio_qspi`**. That directory holds `include/sdkconfig.h` plus 6 memory-specific `.a` files (bootloader_support, esp_hw_support, esp_psram, esp_system, freertos, spi_flash) and `sections.ld`.
- `file_opts` contains `-DARDUINO_CORE_BUILD` only while the core itself is being built.

### 1.3 `flags/cpp_flags` (verbatim)

```
-mlongcalls -fno-builtin-memcpy -fno-builtin-memset -fno-builtin-bzero -mdisable-hardware-atomics
-fno-rtti -ffunction-sections -fdata-sections -Wno-error=unused-function -Wno-error=unused-variable
-Wno-error=unused-but-set-variable -Wno-error=deprecated-declarations -Wno-error=extra
-Wno-unused-parameter -Wno-sign-compare -Wno-enum-conversion -gdwarf-4 -ggdb -freorder-blocks
-Wwrite-strings -fstack-protector -fstrict-volatile-bitfields -fno-jump-tables
-fno-tree-switch-conversion -std=gnu++2b -fexceptions -fuse-cxa-atexit -std=gnu++2a -DCHIP_HAVE_CONFIG_H
```

**IMPORTANT: the effective standard is C++20, not C++23.** The file passes `-std=gnu++2b` and then `-std=gnu++2a`, and GCC uses the last one. I checked this with `-dM -E`: this flag order gives `__cplusplus 202002L`. `-std=gnu++2b` alone gives `202302L`.

Every Arduino sketch, the core and every library therefore compiles as **gnu++20** by default. Other predefined macros under these flags: `__GXX_RTTI` is absent, `__cpp_exceptions` is defined, `__XTENSA_WINDOWED_ABI__=1`, `_GLIBCXX_RELEASE 14`, `_GLIBCXX_HAS_GTHREADS 1`, `_GLIBCXX_USE_C99_MATH_TR1 1`.

`c_flags` contains the same machine flags plus `-std=gnu17 -Wno-old-style-declaration -Wno-strict-prototypes`. `S_flags` contains the machine flags only.

### 1.4 `flags/defines` (verbatim)

```
-DCJSON_CIRCULAR_LIMIT=10000 -DCJSON_NESTING_LIMIT=1000 -DESP32_ARDUINO_LIB_BUILDER
-DESP_MDNS_VERSION_NUMBER=\"1.11.1\" -DESP_PLATFORM -DIDF_VER=\"v5.5.4\"
-DMBEDTLS_CONFIG_FILE=\"mbedtls/esp_config.h\" -DMB_MDNS_IS_INCLUDED -DMD5_ENABLED=1
-DOPENTHREAD_CONFIG_FILE=\"openthread-core-esp32x-spinel-config.h\"
-DOPENTHREAD_PROJECT_LIB_CONFIG_FILE=\"openthread-core-esp32x-spinel-config.h\"
-DSERIAL_FLASHER_BOOT_HOLD_TIME_MS=50 -DSERIAL_FLASHER_RESET_HOLD_TIME_MS=100
-DSOC_MMU_PAGE_SIZE=CONFIG_MMU_PAGE_SIZE -DSOC_XTAL_FREQ_MHZ=CONFIG_XTAL_FREQ -DUNITY_INCLUDE_CONFIG_H
-D_GLIBCXX_HAVE_POSIX_SEMAPHORE -D_GLIBCXX_USE_POSIX_SEMAPHORE -D_GNU_SOURCE -D_POSIX_READER_WRITER_LOCKS
-DTF_LITE_STATIC_MEMORY -DCHIP_CONFIG_SOFTWARE_VERSION_NUMBER=0 -DCHIP_DNSSD_DEFAULT_PLATFORM=true
-DCHIP_DNSSD_DEFAULT_NONE=false -DCHIP_DNSSD_DEFAULT_MINIMAL=false
```

### 1.5 `flags/includes`

This is 17.6 KB containing **342** entries of the form `-iwithprefixbefore <component>/<subdir>`. They resolve against `-iprefix SDK/include/`, for example `newlib/platform_include`, `freertos/config/include`, `freertos/FreeRTOS-Kernel/include`, `esp_hw_support/include`, `soc/esp32s3/include`, `hal/esp32s3/include`, `lwip/...`, `driver/...`, `arduino_tinyusb/...`, and so on.

Use the file as-is through `@file`. Do not expand it. It must come after `-iprefix`, and `-I SDK/qio_qspi/include` (which supplies `sdkconfig.h`) must also be present.

### 1.6 Link (for reference only; a static library does not link)

- `ld_flags`: `-mlongcalls ... -fno-rtti -nostartfiles -Wl,--gc-sections -fno-lto -u _Z5setupv -u _Z4loopv -u app_main ...`
- `ld_libs`: about 9.8 KB of `-l...`, including `-lcxx -lnewlib -lpthread -lfreertos ...`
- `ld_scripts`: `-T esp32s3.*.ld -T memory.ld -T sections.ld`
- `compiler.libraries.ldflags` is placed inside `-Wl,--start-group ... --end-group` (`platform.txt:162`). Because of that, a precompiled `libprosdk.a` does not depend on link order relative to the core and libraries.

The `-fno-rtti` in `ld_flags` matters because it **selects the multilib**. `g++ -print-multi-lib` gives `esp32s3;@mdynconfig=xtensa_esp32s3.so` and `esp32s3/no-rtti;...@fno-rtti`. Arduino links `xtensa-esp-elf/lib/esp32s3/no-rtti/libstdc++.a` (30 MB, about 205 typeinfo objects; only exception types such as `std::exception`, `runtime_error`, `bad_alloc`, `out_of_range` and `bad_function_call` keep their typeinfo). The RTTI build is `esp32s3/libstdc++.a` (about 570 typeinfo objects).

### 1.7 sdkconfig facts relevant to WPILib

From `qio_qspi/include/sdkconfig.h` and `sdkconfig`:

- `CONFIG_COMPILER_CXX_EXCEPTIONS=y`, `EMG_POOL_SIZE=0`, `# CONFIG_COMPILER_CXX_RTTI is not set`.
- `CONFIG_FREERTOS_HZ=1000`, `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240`, `CONFIG_ARDUINO_LOOP_STACK_SIZE=8192`.
- `CONFIG_LIBC_NEWLIB=1`. Optimization is set to size, assertions are enabled at level 2, and `CONFIG_COMPILER_STACK_CHECK_MODE_NORM=y`.
- `CONFIG_SPIRAM=y` (quad).
- The `sdkconfig.h` files for the 4 memory types differ only in flash and PSRAM mode macros (`FLASHMODE_QIO/DIO`, `SPIRAM_MODE_QUAD/OCT`, a few `SPIRAM_BOOT_*`). **One `.a` built against `qio_qspi` works for every memory type** as long as prosdk code never tests those macros.

### 1.8 Recipe for an out-of-Arduino, ABI-compatible build of the static library

Use a CMake toolchain file, or a plain Makefile.

```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR xtensa)
set(TC  $ENV{HOME}/.arduino15/packages/esp32/tools/esp-x32/2601/bin)
set(SDK $ENV{HOME}/.arduino15/packages/esp32/tools/esp32s3-libs/3.3.10)
set(CORE $ENV{HOME}/.arduino15/packages/esp32/hardware/esp32/3.3.10)
set(CMAKE_C_COMPILER   ${TC}/xtensa-esp32s3-elf-gcc)
set(CMAKE_CXX_COMPILER ${TC}/xtensa-esp32s3-elf-g++)
set(CMAKE_AR           ${TC}/xtensa-esp32s3-elf-gcc-ar)
set(CMAKE_RANLIB       ${TC}/xtensa-esp32s3-elf-gcc-ranlib)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
# Put these in a single CMAKE_CXX_FLAGS_INIT string so CMake does not dedupe or reorder -iprefix:
#  @${SDK}/flags/cpp_flags -std=gnu++23 -Os -Werror=return-type
#  -DF_CPU=240000000L -DARDUINO=10607 -DARDUINO_ESP32S3_DEV -DARDUINO_ARCH_ESP32
#  -DARDUINO_BOARD=\"ESP32S3_DEV\" -DARDUINO_VARIANT=\"esp32s3\" -DESP32=ESP32 -DCORE_DEBUG_LEVEL=0
#  -DARDUINO_USB_MODE=1 -DARDUINO_USB_CDC_ON_BOOT=0 -DARDUINO_USB_MSC_ON_BOOT=0 -DARDUINO_USB_DFU_ON_BOOT=0
#  @${SDK}/flags/defines -iprefix ${SDK}/include/ @${SDK}/flags/includes -I${SDK}/qio_qspi/include
#  -I${CORE}/cores/esp32 -I${CORE}/variants/esp32s3 -I${CORE}/libraries/Wire/src ... -I/srv/probot/core/src
```

Rules for staying ABI-compatible:

1. Keep every machine and code-generation flag from `cpp_flags`. Do not drop any of these:
   - `-mlongcalls`
   - `-mdisable-hardware-atomics`: atomics must go through the IDF libcalls. This is required for correctness on PSRAM and for consistency with the core.
   - `-fno-jump-tables`, `-fno-tree-switch-conversion`: IRAM/flash-cache safety.
   - `-fstrict-volatile-bitfields`
   - `-fexceptions`, `-fuse-cxa-atexit`
   - `-ffunction-sections -fdata-sections`: lets `--gc-sections` strip unused WPILib code.
2. Do not use LTO (`ld_flags` contains `-fno-lto`). Do not use `-fshort-enums`, `-fpack-struct` or `-mtext-section-literals` unless the core uses them too (it does not).
3. Putting `-std=gnu++23` after `@cpp_flags` is fine for the `.a`. Mixing gnu++23 objects with gnu++20 objects from the same GCC 14.2 and libstdc++ build is link-compatible. The exception is a type whose layout or presence depends on `__cplusplus` and that crosses the API boundary (for example `std::expected`). See section 6.1 for the header problem this creates.
4. **ODR / config hazards in a precompiled `.a`:**
   - `Serial` changes type with `ARDUINO_USB_CDC_ON_BOOT`/`ARDUINO_USB_MODE` (`cores/esp32/HardwareSerial.h:436-450`, `HWCDC.h:109`, `USBCDC.h:135`). The `.a` must **never reference `Serial`** directly. Log through `esp_log`/`printf`, a `Print&` that is injected at runtime, or probot-core.
   - `CORE_DEBUG_LEVEL` is fixed when the `.a` is built (`log_x` macros).
   - probot-core is header-only. Its `setup()`/`loop()` are `__attribute__((used)) inline` (`runtime.hpp:334-338`), so they are COMDAT and safe to duplicate. However, the `.a` bakes in the probot-core version it was compiled against. Pin an exact version in `depends=` and add a `static_assert` or runtime check on a probot-core version macro.
5. With an archive, a TU that only registers itself through a static constructor gets dropped by the linker. Use explicit init calls, or `ldflags=-Wl,-u,<sym>`.

---

## 2. arduino-cli precompiled library support (arduino-cli 1.5.1 installed at `/usr/local/bin/arduino-cli`)

The binary contains `Using precompiled library in %[1]s`, `Skipping dependencies detection for precompiled library %[1]s`, `does not support '%[1]s' for precompiled libraries` and `dot_a_linkage`, which confirms this behaviour.

### 2.1 library.properties fields

- `precompiled=true` puts the `.a` on the link line **and also compiles every source file** in `src/` ("mixed" libraries).
- `precompiled=full` uses the `.a` and **skips source compilation** when `src/{build.mcu}/` exists for the current board. If that folder is missing, it compiles the sources instead. This lets us ship one library that works for esp32s3 (prebuilt) and anything else (source).
- `ldflags=...` holds extra linker flags, which are appended to `compiler.libraries.ldflags`. arduino-cli already adds `-L"<lib>/src/esp32s3" -l<name>` automatically for every `lib<name>.a`/`.so` in the folder. A `.a` whose name does not start with `lib` is added as an object file instead. So `ldflags` is only needed for extras such as `-Wl,-u,sym` or `-lm`.
- The platform must define `compiler.libraries.ldflags`. ESP32 3.3.10 does (`platform.txt:75`), and it sits inside the `--start-group` block (`platform.txt:162`).
- `dot_a_linkage=true` archives the library's compiled sources before linking, so unreferenced objects are not pulled in. This is useful for the from-source fallback, but it carries the same static-constructor dropping risk as the `.a`.
- `depends=` is used **only by `arduino-cli lib install`** to fetch dependencies. It plays no part in build-time discovery.

### 2.2 Folder layout and how the arch folder is resolved

```
prosdk/
  library.properties      (name=prosdk, architectures=esp32, precompiled=full, depends=probot (=0.4.0), ...)
  src/
    prosdk.h              umbrella header
    prosdk/...            public headers (always on the include path, -I<lib>/src)
    *.cpp                 sources (compiled only when no precompiled folder matches)
    esp32s3/
      libprosdk.a         picked when build.mcu == esp32s3
```

arduino-cli looks for `src/{build.mcu}/` first, then `src/{build.mcu}/{fpu}-{float-abi}/`. The FPU subfolder only applies when `recipe.cpp.o.pattern` contains `-mfpu=`/`-mfloat-abi=`, which is never the case on xtensa. For this board, `esp32s3.build.mcu=esp32s3` (`boards.txt:1183`), so the path is **`src/esp32s3/libprosdk.a`**.

### 2.3 How dependencies are resolved at build time

- arduino-cli discovers libraries **only by `#include` resolution**: it preprocesses the sketch, then the sources of each library it adds.
- For a `precompiled=full` library, it logs "Skipping dependencies detection for precompiled library". **It does not scan the library's `.cpp` files.**
- It does still see every header the sketch transitively includes. The public umbrella header `prosdk.h` must therefore `#include` everything the `.a` needs at link time: `<probot.h>` (probot-core), `<Wire.h>`, `<WiFi.h>`, `<SPI.h>`, `<Preferences.h>`, `<Adafruit_NeoPixel.h>` (pulled in through probot-core), and so on. If it does not, the link fails with undefined references to Wire, WiFi, etc.
- The Arduino core (`core.a`) is always linked.

### 2.4 How a sketch can raise the C++ standard (needed for the headers, see 6.1)

- `--build-property compiler.cpp.extra_flags=-std=gnu++2b` **does not work**. `extra_flags` comes *before* `@cpp_flags`, so the trailing `-std=gnu++2a` wins.
- These do work, because they come after `cpp_flags`:
  - **`build_opt.h`** in the sketch folder, for example containing `-std=gnu++2b` (copied by `platform.txt:129-130`, used as `@build_opt.h`). It is also applied to the core and every library, and to C/S files, which emit a harmless "valid for C++ but not for C" warning.
  - `--build-property build.defines=-std=gnu++2b`. Note that this replaces the PSRAM menu define.
- Caveat: the arduino-cli core cache may not notice `build_opt.h` changes. Use `--clean` once after changing it.

---

## 3. RTTI usage in WPILib (allwpilib b68effca) and the -fno-rtti decision

Patterns searched: `typeid`, `dynamic_cast`, `std::any`, `any_cast`, `target_type`, `dynamic_pointer_cast`, `type_info`, `std::type_index`. Searched in wpimath, wpiutil, wpilibc, commandsv2, hal, telemetry, tunables. commandsv3, epilogue-runtime, ntcore and datalog: 0 hits outside tests.

| Module (files) | Hits outside tests/thirdparty | Thirdparty hits |
|---|---|---|
| wpimath (1081) | 0 | 3 `typeid` in Eigen (`Redux.h:71`, `AssignEvaluator.h:143-144`), only inside `EIGEN_DEBUG_ASSIGN` blocks, which are never enabled |
| wpiutil (369) | **1**: `wpi/util/Demangle.hpp:27` `GetTypeName(const T&)` → `Demangle(typeid(type).name())` | `argparse.hpp`: 6 `std::any` + 19 `any_cast` (CLI parser, drop it on the MCU) |
| wpilibc (393) | 0 | – |
| commandsv2 (138) | 0 direct, but **3 call sites** of `wpi::util::GetTypeName(*this)` | – |
| hal (286), telemetry (27), tunables (42) | 0 | – |

The commandsv2 call sites:

- `commandsv2/src/main/native/cpp/Command.cpp:20`: `Command::Command() : m_name{GetTypeName(*this)}`
- `commandsv2/src/main/native/cpp/SubsystemBase.cpp:16`: same pattern
- `commandsv2/src/main/native/cpp/Subsystem.cpp:24`: `GetName()` default

These give default names for commands and subsystems. With `-fno-rtti`, GCC rejects `typeid` ("cannot use 'typeid' with '-fno-rtti'") once the template is instantiated.

There are 90 other `GetTypeName` hits. They are `static constexpr GetTypeName()` in the `*Struct.hpp` serialization traits and are unrelated to RTTI.

**Impact of -fno-rtti: very small.** Only `GetTypeName` (1 template, 3 callers) and argparse are affected.

- `std::function` works without RTTI. Only `target_type()`/`target<T>()` degrade, and neither is used.
- libstdc++'s `std::any` and `make_shared` have no-RTTI code paths.
- Exceptions still work with `-fno-rtti`: GCC emits typeinfo for thrown types, and the no-rtti multilib keeps typeinfo for std exception classes.

**Should prosdk use `-frtti`?** No. Keep `-fno-rtti`:

1. Arduino links the `no-rtti` libstdc++ multilib, because `ld_flags` contains `-fno-rtti`. That copy lacks typeinfo for most std types, so an `-frtti` object that needs, for example, `typeinfo for std::basic_streambuf` can fail to link.
2. The core and libraries are built with `-fno-rtti`, so typeinfo is **never emitted** for their polymorphic classes (`Print`, `Stream`, `Client`, `Server`, `HardwareSerial`, `TwoWire`, `WiFiClient`, and so on). An `-frtti` prosdk class that derives from any of them references `typeinfo for Print` (`_ZTI5Print`), which leads to **undefined reference at link**. `dynamic_cast` across such a base cannot work either.
3. The vtable layout is the same with or without RTTI (the typeinfo slot holds 0), so plain virtual calls are compatible. The breakage is limited to link-time typeinfo symbols, but it cannot be predicted per sketch.
4. The only safe way to enable RTTI is a sketch-wide `-frtti` through `build_opt.h`, which recompiles the core and libraries. It also costs flash for type names and typeinfo, and the link would still select the no-rtti multilib unless `ld_flags` changes. **Not recommended.**

**Recommendation:** keep `-fno-rtti`. Replace `wpi::util::GetTypeName(*this)` with:

- a virtual `GetName()` that defaults to `"Command"`/`"Subsystem"`, plus `SetName()`; or
- a `__PRETTY_FUNCTION__`-based `prosdk::TypeName<T>()` used through a CRTP helper or an optional `PROSDK_NAMED(Class)` macro.

Implement `Demangle()` with `abi::__cxa_demangle`, which is present in libsupc++ and needs no RTTI (it was used for stack traces), or make it a stub. Drop argparse.

Side note: calling `typeid(*this)` in a base constructor yields the base type anyway. Upstream WPILib gets "Command" there, so a static string is the same behaviour.

---

## 4. Espressif QEMU for ESP32-S3 (prebuilt; not downloaded)

- Repo: https://github.com/espressif/qemu/releases. Latest at check time: **`esp-develop-9.2.2-20260417`**, published 2026-04-19.
- Linux x86_64 asset: `qemu-xtensa-softmmu-esp_develop_9.2.2_20260417-x86_64-linux-gnu.tar.xz`, **15,619,396 bytes (≈15.6 MB)**.
  URL: https://github.com/espressif/qemu/releases/download/esp-develop-9.2.2-20260417/qemu-xtensa-softmmu-esp_develop_9.2.2_20260417-x86_64-linux-gnu.tar.xz
- Previous release: `esp-develop-9.2.2-20250817` (15,648,448 bytes). There is also an aarch64-linux build (about 15.1 MB).
- Release notes for 20260417 mention ESP32-S3 SDMMC emulation and extended ESP32-S3 TIE/vector instructions, so S3 support is active.
- Binary: `qemu-system-xtensa`, machine **`-M esp32s3`**.
- Host runtime libraries are already present: libSDL2-2.0.so.0, libgcrypt.so.20, libslirp.so.0, libglib-2.0, libpixman-1.
- Typical run: `qemu-system-xtensa -nographic -M esp32s3 -drive file=<sketch>.ino.merged.bin,if=mtd,format=raw`. Arduino already produces `*.merged.bin` padded to the flash size (`platform.txt:181`).
- Limitations to verify:
  - **No WiFi emulation**, so probot-core's AP and Driver Station will not come up. Ethernet-over-openeth is documented mainly for ESP32/C3.
  - PSRAM support (`-m` option) needs checking.
  - GPIO, LEDC, PCNT and I2C peripherals are mostly not modelled.
- **Use QEMU for headless self-tests over UART**: wpimath, the command scheduler, timing and FreeRTOS integration, and exceptions and static init on the real toolchain. Do not use it for DS, network or motor I/O.

---

## 5. Old SDK (`/srv/probot/sdk/src/probot`, v0.2.6, ~7.9k lines): drivers worth carrying over

| File | Lines | Assessment |
|---|---|---|
| `devices/motors/boardoza_vnh5019_motor_controller.hpp` | 374 | **Carry over.** INA/INB/PWM (+ optional ENA/ENB) H-bridge using `analogWrite*` (LEDC), brake mode and brake strength, timeout. Strip its built-in velocity/position PID (`IEncoder` + `PidConfig`) and rebase it on the prosdk `MotorController`/`PWMMotorController` + HAL PWM. WPILib PIDController/feedforward replace the PID. |
| `devices/motors/bts7960b_motor_controller.hpp` | 139 | **Carry over (small).** 20 kHz LEDC. It puts the PWM on REN/LEN and uses RPWM/LPWM as direction, which is unusual. Make the wiring mode configurable (classic: EN high, PWM on RPWM/LPWM). It becomes a thin `MotorController` subclass. |
| `devices/motors/imotor_controller.hpp`, `motor_controller_group.hpp` | ~30 / 84 | **Drop.** WPILib's `MotorController` and composition / `addFollower` replace them. Keep only as a compatibility shim if old sketches matter. |
| `devices/sensors/imu/mpu6050.hpp` | 159 | **Carry over (rework).** Raw Wire register driver: ±2 g / ±250 dps fixed, gyro bias calibration, complementary filter, yaw by plain integration (`mpu6050.hpp:129`, so it drifts). Expose it as a WPILib-style gyro (`GetRotation2d()`, `GetRate()`, `Reset()`) with sim support (SimDevice). Make the ranges configurable. |
| `devices/sensors/encoder.hpp` | ~10 | **Drop** (`IEncoder` interface only, no concrete GPIO encoder). Write a new `prosdk::Encoder` on the ESP32-S3 **PCNT** peripheral (`esp_driver_pcnt` is linked by default), plus an `EncoderSim`. |
| `prohub/prohub_board.hpp`, `prohub_config.hpp` | 51 / ~40 | **Carry over** as vendor board constants: I2C SDA=1/SCL=2 @400 kHz, PCA9685 at 0x40/0x41, encoder UART RX=10/TX=11 @921600, 8 encoders and 8 controllers, 3 ms inner loop. |
| `prohub/encoder_frame.hpp`, `uart_encoder.hpp` | ~20 / 205 | **Carry over.** Reads the Pico co-processor encoder protocol: 108-byte frame `AA 55 seq ts dt 8×i64 total 8×i32 delta crc8-xor`, background task pinned to a core, CRC error counter. Becomes the vendor `ProHubEncoder` (WPILib encoder API) backed by this stream. The protocol must stay wire-compatible with the Pico firmware. |
| `prohub/pca_motor.hpp` | 159 | **Carry over, with a bug fix.** PCA9685 motor (LPWM/RPWM channel pairs) and servo output. **Bug:** `setPWM()` calls `Wire.beginTransmission/.../endTransmission()` inside `portENTER_CRITICAL(&mux_)` (`pca_motor.hpp:41-70`). The blocking IDF I2C driver cannot run with interrupts disabled; it will assert (task cannot block) or trigger the interrupt watchdog. Replace with a FreeRTOS mutex / `std::mutex`, or a dedicated I2C task with a queue. |
| `prohub/double_buffer.hpp` | 48 | **Rewrite.** Same-core SPSC "double buffer" whose `read()` returns a reference. After two writer swaps the writer overwrites the buffer the reader holds, so reads can tear. Use a seqlock or a triple buffer, or copy under a short spinlock (no blocking calls inside). |
| `prohub/inner_loop.hpp`, `prohub.hpp` | 279 / 207 | **Carry over the concept only.** A fast (3 ms) on-board PID inner loop task over PCA motors and UART encoders, with an FTC-style `ProHub.motor(n)` facade. In prosdk this maps to a vendor "smart motor controller" (closed-loop setpoints like REV/CTRE) running its own FreeRTOS task, with WPILib `PIDController` math. |
| `test/test_motor.hpp`, `test_plant.hpp`, `test_encoder.hpp` | small | Optional. Replaced by WPILib sim classes (`DCMotorSim`, `FlywheelSim`, `EncoderSim`). |

The old `control/*` (PID, feedforwards, kinematics, odometry, LQR/Kalman, Ramsete) and `command/*` are all superseded by wpimath and commandsv2. Do not carry them over.

---

## 6. Additional findings and recommendations

### 6.1 C++23 in WPILib public headers vs Arduino's effective C++20

WPILib main targets `-std=c++23` (`shared/bazel/compiler_flags/linux_flags.rc:10`). In the ported modules, the public headers using C++23 features are:

- `if consteval` in 7 headers, including `wpi/math/geometry/Rotation2d.hpp`, `Rotation3d.hpp`, `Transform3d.hpp`, `Twist3d.hpp`, `linalg/ct_matrix.hpp`, `wpi/util/Color.hpp`. Under gnu++20, GCC 14 **accepts this as an extension** (only a `-Wc++23-extensions` pedwarn, which `-w` silences). Checked with `-fsyntax-only`.
- `std::expected` in `wpi/math/linalg/DARE.hpp`, which is included by 19 headers (LQR, Kalman, and so on), plus 3 hal handle headers. `<expected>` **fails under gnu++20** (checked) and works under gnu++2b.
- `std::print` in `wpi/util/print.hpp`.
- `std::format` (17 public headers) works in C++20 with GCC 14.

Options, in order of preference:

- **(a) Recommended:** prosdk provides `prosdk::expected` (`std::expected` when `__cpp_lib_expected`, else a vendored tl::expected). Replace `wpi::print` with an fmt/printf shim. Public headers then compile as gnu++20, and nothing is required of users.
- (b) Ship a template `build_opt.h` containing `-std=gnu++2b` in every example and document it.

In both cases, build the `.a` itself with gnu++23.

### 6.2 Build tools on the host

- Present: `cmake 3.28.3`, GNU make, `python3 3.12.3` + pip 24.0, `arduino-cli 1.5.1`, 8 CPUs (load ~10-16 during this check).
- Missing: `ninja` and `ccache`.
  - Ninja: apt candidate `ninja-build 1.11.1`, or a PyPI wheel (`ninja 1.13.2`, official) in a venv. No root needed.
  - ccache: **not on PyPI**. Apt candidate `ccache 4.9.1`, or the GitHub static binary `ccache-4.14.2-linux-x86_64-musl-static.tar.xz` (1.59 MB, https://github.com/ccache/ccache/releases/download/v4.14.2/ccache-4.14.2-linux-x86_64-musl-static.tar.xz), unpacked into the scratch or project `tools/` directory. No root needed.
  - apt installs are system changes; ask Tuna first.

### 6.3 Recommended low-load incremental build

1. **Out-of-Arduino CMake build of `libprosdk.a`** with the toolchain file in section 1.8, in a persistent build dir such as `/srv/probot/sdk/.build/lib-esp32s3`.
   - Run as `nice -n 19 ionice -c3 cmake --build . -j2`.
   - Use `-G Ninja` once ninja is available; otherwise Make with `-j2`.
   - `CMAKE_CXX_COMPILER_LAUNCHER=ccache` once ccache is present, with `CCACHE_DIR` in the project and a `max_size` of about 2G.
   - Rebuilds are incremental by object (`-MMD` depfiles), so only touched TUs recompile.
2. **Split per module** (wpiutil, wpimath, hal, wpilibc, commandsv2, telemetry, tunables, vendor) into OBJECT libraries, merged into one `libprosdk.a` with `gcc-ar`. Eigen-heavy wpimath TUs are the expensive ones; keep them separate so edits elsewhere do not touch them.
3. **Host-native unit tests**: compile the platform-independent parts (wpimath, commandsv2 logic, telemetry) with host g++ and a stub HAL, so most iteration never touches xtensa. Use QEMU for on-target smoke tests.
4. **Sketch builds**: use `arduino-cli compile --build-path <persistent> --jobs 2` under `nice`. Use `precompiled=full` so sketches only compile the sketch, the core (cached) and small libraries. Avoid `--clean` and avoid the `make clean`/`cache clean` in the existing Makefile.
5. Put `.build/`, `ccache/` and the QEMU tarball under `.gitignore`. Hold `/run/lock/agir-tarayici.lock` only for browser jobs; it is not needed for compiles.

### 6.4 Other notes

- `library.properties` for both the old sdk and probot-core has `name=probot`. The new library must be `name=prosdk` with `depends=probot (=0.4.0)` (pinned) to avoid the name clash.
- Make `includes=prosdk.h` the umbrella header.
- Keep `architectures=esp32`, and gate on `CONFIG_IDF_TARGET_ESP32S3` with a clear `#error` on other chips, because only `src/esp32s3/` is precompiled. Alternatively, let the `full` fallback compile from source.
