# P4: multizone time-of-flight (VL53L7CX)

Package P4 of the Boardoza plan. One class, `prosdk::VL53L7CX`, serves the four Boardoza
VL53L7CX breakout boards. They carry the same ST VL53L7CX module and differ only in supply and
level shifting. The VL53L7CX evaluation board (STM32 with closed firmware) has no class; see
P14.

| Board | Ozdisan id | Supply | Signals | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| VL53L7CX-1 | 1577675 | 3.3 V only (3V3 pin, no regulator) | 3.3 V, no shifter | VL53L7CX-1_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board |
| VL53L7CX-2 | 1577676 | 3.3-5 V on "5V" (AP2112K-3.3 LDO) | SDA/SCL shifted (BSS138) | VL53L7CX-2_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board |
| VL53L7CX-3 | 1577677 | 3.3-5 V on "5V" (AP2112K-3.3 LDO) | SDA/SCL/INT shifted (BSS138) | VL53L7CX-3_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board |
| VL53L7CX-4 | 1577678 | 3.3-5 V on "5V" (AP2112K-3.3 LDO) | 3.3 V, no shifter | VL53L7CX-4_Multizone_Time-of-Flight_Distance_Sensor_Breakout_Board |

Sources:
- ST datasheet DS13865 Rev 8 (September 2024), the copy in each board repo's `assets/`.
- ST VL53L7CX Ultra Lite Driver (ULD) 2.0.0 from
  `github.com/STMicroelectronics/x-cube-tof1`, folder
  `Drivers/BSP/Components/vl53l7cx/modules` (last commit there:
  9d721708c394de16aaee9d1579e1f79f2a1e085a, 2024-07-30), BSD-3-Clause
  (`Drivers/BSP/Components/vl53l7cx/LICENSE.md`).
- ST's multi-sensor example in the same repository
  (`Projects/NUCLEO-F401RE/Examples/53L7A1/53L7A1_MultiSensorRanging/Src/app_tof.c`) for the
  LPn and address order.
- The board READMEs and KiCad schematics (research records in the Boardoza plan).

The board repos ship the SparkFun VL53L5CX Arduino library (VL53L5CX ULD 1.1.1 and its firmware),
not ST's VL53L7CX driver. The L7CX reports the same ID bytes, but ST publishes separate L7CX
firmware, so prosdk vendors ST's VL53L7CX ULD and does not use the repo code.

## Wiring and 5 V hazards

J1 (4 pins: GND, SCL, SDA, supply) and J2 (7 pins: 1 supply, 2 GND, 3 SDA, 4 SCL, 5 RST, 6 INT,
7 LPn) are wired in parallel for daisy chains. On the -1 the supply pin is "3V3", on the others
"5V".

- The sensor's pins are 3.6 V absolute maximum (DS13865 table 11), and the ESP32-S3 is not 5 V
  tolerant.
- **VL53L7CX-1**: 3.3 V only. 5 V on its 3V3 pin exceeds the sensor's 3.6 V maximum.
- **VL53L7CX-2 and -3**: power them from **3.3 V** when an ESP32-S3 is the host. The BSS138
  shifters pull the host side of SDA and SCL (and INT on the -3) up to the supply pin, so a 5 V
  supply puts 5 V on the controller's pins. (The schematics show 2.2 kOhm resistors on both sides
  of the shifters and a 5V rail; the individual nets were not traced: unverified.) With 3.3 V in,
  the AP2112K-3.3 output sits a little below 3.3 V; the sensor needs 3.0 V or more on AVDD and
  IOVDD (table 12). That this holds while ranging is unverified.
- RST and LPn are not shifted on any board: drive them only with 3.3 V.
- **VL53L7CX-4**: 3.3 V or 5 V on "5V"; all signals stay at 3.3 V.
- SDA and SCL have 2.2 kOhm pull-ups on every board. Several boards on one bus put them in
  parallel (three boards: about 730 Ohm); stay within the bus's sink current.
- RST (I2C_RST) resets only the I2C interface (table 3); leave it unconnected (pulled low on the
  board). INT is not used by the driver; leave it unconnected.
- LPn is pulled up on the board (I2C on). Wire it to a GPIO only for several sensors on one bus.

Current: about 45-50 mA on AVDD and 50-80 mA on IOVDD while ranging, peaks 10 mA higher per
rail (table 14); budget about 150 mA per board.

## Class

`src/prosdk/drivers/range/VL53L7CX.hpp` and `.cpp`.

```cpp
VL53L7CX(I2C::Port port, int lpnChannel = -1, int address = kDefaultAddress /* 0x29 */,
         Resolution resolution = Resolution::k4x4, units::hertz_t rangingFrequency = 15_Hz);
bool IsReady();                                  // frames are arriving
int GetDistances(std::span<int16_t> distances,   // mm; returns the zone count (16, 64, or 0)
                 std::span<uint8_t> status = {});  // optional: the same frame's target status
int GetTargetStatus(std::span<uint8_t> status);  // 5 or 9: valid; 255: no target
void SetAddress(int newAddress);                 // 0x08-0x77, applied by the thread
// TelemetryLoggable: type "Distance Sensor", entries "Ready" and "Distances"
```

- **Startup.** The constructor does not use the bus. It only opens the I2C port and, with an
  LPn channel, drives LPn low (sensor I2C off). The thread starts at the first call of
  `IsReady()`, `GetDistances()`, `GetTargetStatus()` or `SetAddress()`. A robot's sensors are
  therefore all constructed (all LPn pins low) before any of them talks at the default address.
  The thread then:
  1. takes the port's default-address turn (one sensor per port at a time);
  2. drives LPn high and waits 2 ms (ST's multi-sensor example; the datasheet gives no time);
  3. looks for the sensor at its target address first (it keeps an address over a program
     restart without a power cycle), otherwise at 0x29 (`vl53l7cx_is_alive`: ID 0xF0, revision
     0x02);
  4. runs `vl53l7cx_init()`: software reboot, firmware upload (86016 bytes), NVM offsets,
     default crosstalk and configuration;
  5. moves the sensor to its address with `vl53l7cx_set_i2c_address()` and ends its turn;
  6. sets the resolution, the frequency and continuous mode, and starts ranging.
- **Frames.** The thread polls `vl53l7cx_check_data_ready()` every 5 ms and reads each new frame
  with `vl53l7cx_get_ranging_data()`. The getters copy the latest frame under a mutex.
  `IsReady()` is true while frames arrive. With no frame for 3 frame periods (at least 1 s), the
  thread starts over from step 1. That covers a sensor that lost power and with it its firmware
  and address. Any failure prints one console line (only when the problem changes), drives LPn
  low (if wired) and the thread tries again after 1 s. LPn low keeps a sensor that came back at
  0x29 after a power loss off the bus during another sensor's turn. Sensors on one supply lose
  power together; with the same frame rate they time out within one frame period of each other,
  which is shorter than the 1 s pause above 1 Hz, so the others' LPn is low before the first
  one's next turn (not tried on hardware).
- **One frame.** `GetDistances(distances, status)` copies the distances and the target status
  of the same frame. Separate `GetDistances()` and `GetTargetStatus()` calls can see two
  different frames when a new one arrives between them.
- **Zones.** Entry i is zone i as numbered in DS13865 section 7.1: row by row, as seen from
  above the sensor looking into its lens. The lens flips the image horizontally and vertically,
  so zone 0 sees a target at the top right-hand side of the scene (section 7.1.3, figure 19).
  ST's examples print each row in reverse to show the scene as seen from behind the sensor. The
  zone order relative to the board's connectors is unverified.
- **Distance and status.** Distances are mm, and the ULD clamps negative values to 0. A distance
  is valid only with target status 5 or 9 (comment in `vl53l7cx_api.h`). 255 means that the zone
  saw no target (the ULD sets it from the target count).
- **Several sensors on one bus.** Wire every LPn and give every sensor its own address other
  than 0x29, for example 0x30, 0x31. A sensor without LPn, or one that keeps 0x29, stays at the
  default address and collides with the next sensor's turn. The driver cannot detect that. The
  turns take about 2 s each at 400 kHz. Addresses are volatile.
- **SetAddress()** stores the new address. While ranging, the thread stops ranging, moves the
  sensor and restarts ranging. Before that, the address is used at the next setup.
- **Destructor.** It stops the thread at once: a ULD call in progress sees its transfers fail and
  its waits return. The sensor keeps ranging until it loses power or the next setup reboots it.
  The thread leaves LPn at its last level, and the `DigitalOutput` is then freed; what level the
  GPIO keeps after that is up to the HAL (not checked).
- **Emergency stop.** DIO outputs go low, so a sensor with LPn wired loses I2C until the program
  restarts. I2C itself is not gated.

Defaults: address 0x29 (8-bit 0x52, DS13865 section 4); 4x4 at 15 Hz in continuous mode (the
datasheet's ranging figures use 15 Hz continuous with the 14 % default sharpener, table 2 note).
The frequency is rounded to whole hertz and limited to 1-60 Hz for 4x4 and 1-15 Hz for 8x8 (ULD
`vl53l7cx_set_ranging_frequency_hz` documentation). The ULD's other settings (sharpener, target
order, integration time) keep ST's defaults.

## Vendored ULD and platform layer

`src/prosdk/drivers/range/st/`:

| File | Origin |
|---|---|
| `vl53l7cx_api.c`, `vl53l7cx_api.h`, `vl53l7cx_buffers.h` | ST ULD 2.0.0, unmodified (BSD-3-Clause; `licenses/st-vl53l7cx/LICENSE`) |
| `platform.h` | hand-written: the ULD's platform interface, output selection |
| `platform.cpp` | hand-written: that interface on `prosdk::I2C` |
| `VL53L7CXPlatform.hpp` | hand-written: the I2C device behind `VL53L7CX_Platform::bus` |

ST's BSP wrapper (`vl53l7cx.c`), its `porting/` template and the plugins (detection thresholds,
motion indicator, crosstalk calibration) are not vendored. `drivers.cmake` compiles the `.c`
file through its `*.c` glob (F2).

`platform.h` keeps only three outputs on: the target count, the distance and the target status.
It does this with the ULD's own `VL53L7CX_DISABLE_*` macros, which shrinks a frame from 1444 to
128 bytes (4x4) or 320 bytes (8x8).

Platform layer (`platform.cpp`):
- Every transfer starts with the 16-bit register index, MSB first. The sensor auto-increments the
  index (DS13865 section 4).
- Blocks longer than 1 KiB (`kVL53L7CXChunkSize`) are split into transfers of at most 1 KiB, each
  carrying the index of its first byte. This affects the three firmware pages (32 KiB, 32 KiB,
  20 KiB). Reads never exceed 1 KiB in practice: a frame is at most 320 bytes, the NVM block 492.
- Why 1 KiB: a 1 KiB write is about 23 ms at 400 kHz, under the ESP32 HAL's 50 ms timeout per
  transfer. At 100 kHz it would take about 92 ms and fail, so **the port must run at 400 kHz or
  faster**. The sensor allows 1 MHz (DS13865 table 7).
- `vl53l7cx_set_i2c_address()` changes the platform's 8-bit address. The next transfer reopens
  the `prosdk::I2C` device at the new address.
- `VL53L7CX_SwapBuffer` reverses each 32-bit word. A `static_assert` requires a little-endian host
  (ESP32-S3 and the x86/ARM hosts are little-endian).
- `VL53L7CX_WaitMs` sleeps the driver thread. During the destructor's abort, transfers fail at
  once and waits return at once.

## Resource use

- Flash: the firmware and its tables, 86016 + 971 + 776 + 40 bytes, plus the ULD code. The
  library is precompiled, so the linker takes `vl53l7cx_api.o` only when a sketch uses the class.
- RAM per sensor: the ULD state (about 2.3 KB: offset, crosstalk and a 1 KiB temporary buffer),
  the results (about 260 bytes), a 1 KiB write buffer, two 64-entry frame copies, and one thread
  (6 KiB stack, prosdk priority, core 1). The 6 KiB stack is not measured on hardware.
- I2C: about 2 s of bus time for each setup at 400 kHz (about 0.8 s at 1 MHz), plus the ULD's
  fixed waits (at least 100 ms). While ranging: a 4-byte poll every 5 ms (about 0.2 ms at
  400 kHz) and one frame read per frame (about 3 ms for 4x4, 7 ms for 8x8 at 400 kHz). Put the
  sensor on its own port (port 1) if latency-sensitive devices share port 0.
- GPIO: none required; one DIO output per sensor for LPn when several share a bus.

## Verified and unverified

Verified against DS13865 Rev 8:
- 8-bit address 0x52 (section 4);
- 1 MHz maximum (section 4, table 7);
- 16-bit index with auto-increment (section 4);
- the pin functions of LPn, I2C_RST and INT (table 3);
- the absolute maximum and operating supplies (tables 11, 12);
- the current (table 14);
- the zone numbering (section 7.1);
- 2-350 cm and the 60 Hz maximum (table 1).

Verified against the ST ULD 2.0.0 source:
- the firmware size;
- the init, address-change and data-ready sequences (used unmodified);
- the frequency limits;
- the valid status codes 5 and 9.

Unverified (hardware items):
- The whole driver on a real sensor: the firmware upload through the ESP32 I2C HAL in 1 KiB
  writes, the 50 ms timeout per transfer, frame decoding and distances.
- Whether `vl53l7cx_init()`'s software reboot resets a moved address. The driver handles both
  cases: a failed setup at the target address is retried at 0x29.
- The 2 ms LPn delay, taken from ST's example; the datasheet gives no value.
- The clock of port 0 when `Wire` or probot-core started it first (the HAL keeps that clock). A
  100 kHz bus breaks the 1 KiB writes.
- The -2/-3 pull-up nets, and the AP2112K output with a 3.3 V supply (see wiring).
- The 6 KiB thread stack, the zone orientation relative to the board, and several sensors on one
  bus (including the recovery after a shared power loss).
- Frame integrity: the ULD trusts the block headers in a frame. `vl53l7cx_get_ranging_data()`
  copies each block with the size from its header, and I2C has no checksum. A corrupted header
  (a bit error in a type or size field of an enabled block) can make that copy overrun the
  results structure. Every ULD-based driver has this; the header/footer id check catches a
  corrupted frame only after the copy. Not seen in practice; a noisy bus (long wires, several
  boards) raises the risk.

## Tests

`test/prosdk/drivers/VL53L7CXTest.cpp` (FakeI2CBus, 16-bit register pointer). Run on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check --test`): 60
assertions in 4 test cases pass.
- platform layer: index framing, 2500-byte write split into 1024/1024/452 with indexes
  0x1000/0x1400/0x1800, chunked read, single registers, word swap, NACK and abort;
- ULD identity check and address change (writes at the old address, the last one at the new);
- frame decode: a synthetic 4x4 frame, laid out the way ULD 2.0.0 parses it (not a recording),
  goes through the platform read, the word swap and `vl53l7cx_get_ranging_data()`: distances
  /4, a negative distance clamped to 0, status 255 for a zone with no target, and a footer
  mismatch reported as a corrupted frame;
- no sensor: the constructor keeps LPn low and does not use the bus, the first `IsReady()`
  starts the thread, which raises LPn, finds nothing at the target address 0x30, probes 0x29 with
  LPn high (a chip with ID 0x00 there gets exactly the four `vl53l7cx_is_alive` transfers and no
  firmware), then drives LPn low; `IsReady()` stays false and the destructor returns at once
  (under 500 ms; the retry pause is 1 s).

A full ULD setup cannot be simulated meaningfully, because the firmware handshakes are sensor
behaviour. That stays a hardware item.

## Example

`examples/BoardozaMultizoneToF/BoardozaMultizoneToF.ino`: one board on port 1 (SDA 17, SCL 16,
400 kHz), 8x8 at 15 Hz, with telemetry of the zones and the nearest valid distance. The
comments show two boards with LPn on GPIO 38 and 39 at 0x30 and 0x31.

## Shared edits (integration)

- `NOTICE`: add the ST VL53L7CX ULD (BSD-3-Clause, STMicroelectronics), located in
  `src/prosdk/drivers/range/st/vl53l7cx_api.{c,h}` and `vl53l7cx_buffers.h`, with its license at
  `licenses/st-vl53l7cx/LICENSE`. This package added that license file; it is the unmodified
  `LICENSE.md` of the ST component.
- `design/drivers.md`: a link to this file. `tools/build/modules/drivers.cmake`: the comment line
  listing hand-written drivers.
- `design/API-DIFFERENCES.md` section 7: one row for `prosdk::VL53L7CX`.
- `tools/upstream/examples.json` ("handwritten") and `design/examples.md`: BoardozaMultizoneToF.
- `design/size.md`: the archive size and one sketch with the class (the firmware is about 84 KiB
  of flash).
