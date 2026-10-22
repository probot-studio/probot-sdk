# P6: Load cells and ADCs

Package P6 of the Boardoza plan. It adds two bridge-ADC drivers and a load-cell helper in
`src/prosdk/drivers/adc/`:

| Board | Ozdisan id | Chip | Class | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| NAU7802 | 1206517 | Nuvoton NAU7802SGI | `prosdk::NAU7802` (+ `prosdk::LoadCell`) | NAU7802_24bit_Dual_Channel_ADC_Breakout_Board |
| HX711 | 1206514 | Avia HX711 | `prosdk::HX711` (+ `prosdk::LoadCell`) | HX711_Load_Cell_Amplifier_Breakout_Board |

Files:
- `src/prosdk/drivers/adc/NAU7802.hpp` and `.cpp`, `HX711.hpp` and `.cpp`, `LoadCell.hpp`
  (header only)
- Tests (host only): `test/prosdk/drivers/NAU7802Test.cpp`, `HX711Test.cpp`, `LoadCellTest.cpp`
- Example: `examples/BoardozaLoadCell/BoardozaLoadCell.ino`

Dependencies: F2 (`test/prosdk/drivers/FakeI2CBus.hpp`, used by the NAU7802 test) and F3
(`prosdk::DigitalShift`, used by `HX711`).

Sources (copies kept outside the repo):
- Nuvoton NAU7802 data sheet Revision 1.7 (from the board repo's `assets/`): 7.2 (supply ranges),
  8.3.2 (address 0101010), 8.6 (calibration), 8.8 (reset, power-down, six conversions), 9.1
  (power-on sequencing), 9.2 (full scale), 10 (register map), 11.1 to 11.15 (register details).
- Avia Semiconductor HX711 data sheet, English, 9 pages (the board repo has page 1 only):
  "Serial Interface", Table 3, Fig. 2 and its timing table (T1 to T4), "Reset and Power-Down" and
  Fig. 3, "Output Data Rate and Format", the electrical table (output settling time).
- The two board READMEs and KiCad schematics.

No vendor code was used. Both board repos are CC BY-SA 4.0 and their libraries have known bugs
(no timeouts, a mislabelled HX711 gain enum, PGA_CAP_EN set on a dual-channel board), so both
drivers are written from the data sheets. NOTICE and `licenses/` do not change.

## 1. Wiring and hazards

**Power both boards from 3.3 V when they are wired to the ESP32-S3.** The ESP32-S3 pins are not
5 V tolerant.

| Board | Host connector | Supply | Host-side lines |
|---|---|---|---|
| NAU7802 | J2: 1 GND, 2 SDA, 3 SCL, 4 3.3V | the J2 3.3V pin feeds DVDD; the README says 2.7 to 5.5 V | 10k pull-ups from SDA/SCL to the J2 supply pin: 5 V there puts 5 V on SDA/SCL |
| HX711 | J1: 1 VCC, 2 VDD, 3 DAT, 4 CLK, 5 GND | 2.7 to 5.5 V on both pins | DAT (DOUT) is push-pull at DVDD: 5 V on DVDD puts 5 V on the GPIO |

NAU7802 board:
- Address 0x2A, fixed in silicon. Two boards need two I2C ports. The chip supports 400 kHz.
- J1 = channel 1 (VIN1): 1 WHT (A-), 2 GRN (A+), 3 BLK (E-), 4 RED (E+). J4 = channel 2 (VIN2):
  1 GRN (A+), 2 WHT (A-), 3 BLK, 4 RED. The signal pin order differs between J1 and J4.
- RED (E+) is the VDDA net (AVDD/LDO pin), not 3.3 V, so the bridge is excited only when the
  internal LDO runs. The driver always enables it (AVDDS = 1) at 3.0 V: AVDD must not exceed DVDD
  (7.2), and DVDD must be 0.3 V above the LDO output (section 3, Features), so 3.0 V is the
  highest setting with a 3.3 V DVDD. The LDO drives at least 10 mA at 3.0 V (section 3): one
  350 ohm bridge (about 8.6 mA) fits; two 350 ohm bridges (about 17 mA) exceed the guaranteed
  current.
- J3: 1 VDDA (LDO output), 2 INT (DRDY, active high), 3 GND. The driver polls the CR status bit
  and does not use INT. Do not feed VDDA from outside: the driver turns the LDO on.
- The schematic connectivity (REFP to VDDA, REFN to GND) was read from label positions in the
  KiCad file, not from a netlist (**unverified**).

HX711 board:
- J3 (load cell): 1 RED (E+), 2 BLK (E-), 3 WHT (A-), 4 GRN (A+). Channel B (J2) needs R7, R8 and
  C8 fitted; they are not populated by default.
- Which J1 pin feeds which HX711 supply pin is **unverified** (assumed: VCC to VSUP/the regulator,
  VDD to DVDD). Tie both to 3.3 V. With 3.3 V on VCC the analog regulator runs in dropout; the BOM
  has 20k and 8.2k resistors, which suggests AVDD of about 4.3 V when VCC is 5 V (**unverified**).
  Only after checking with a meter that the VDD pin reaches HX711 pin 16 (DVDD) and not VSUP,
  VCC may go to 5 V for a larger AVDD.
- JP1 sets RATE: 1-2 = 80 SPS, 2-3 = 10 SPS. The schematic symbol is an open jumper, so RATE may
  float as shipped (**unverified**): bridge one side.
- Any two free GPIOs work: DAT to a `DigitalInput`, CLK to a `DigitalOutput`.

Full scale (both chips): +/-0.5 * AVDD / gain. The NAU7802 at 3.0 V and x128 gives +/-11.7 mV; the
HX711 at AVDD 4.3 V gives +/-16.8 mV (gain 128), +/-33.6 mV (64) and +/-67 mV (channel B, 32).

## 2. API

Both ADC classes follow the hand-written driver pattern (`design/drivers.md`): the constructor
does not throw for a missing chip, `IsConnected()` reports the last read, values are cached under
a mutex, each class is a `TelemetryLoggable`, and none starts a thread. `Update()` is non-blocking:
call it from a periodic method. It returns the new sample, or nothing.

| Class | Constructor | Methods | Telemetry type, entries |
|---|---|---|---|
| `NAU7802` | `(I2C::Port)`, address `kAddress` = 0x2A | `SetGain(int)` (1 to 128, powers of 2), `SetSampleRate(int)` (10, 20, 40, 80, 320), `SetChannel(int)` (1, 2), `CalibrateInternal() -> bool`, `IsReady()`, `Update() -> optional<int32_t>`, `GetRaw() -> int32_t`, `IsConnected()` | "ADC": Raw, Channel, Connected |
| `HX711` | `(int dataGpio, int clockGpio, Gain = Gain::k128A)` | `SetGain(Gain)`, `GetGain()`, `Update() -> optional<int32_t>`, `GetRaw() -> int32_t`, `IsConnected()`, `PowerDown()`, `PowerUp()`; `Gain { k128A, k64A, k32B }` | "ADC": Raw, Connected |
| `LoadCell` | `(std::function<std::optional<int32_t>()> source)` | `Update() -> bool`, `Tare(int samples = 10)`, `IsTaring()`, `SetScale(double countsPerKg)`, `GetScale()`, `GetOffset()`, `GetCounts()`, `GetMass() -> kilogram_t`, `HasData()` | "Load Cell": Mass (kg), Counts, Taring |

Raw values are signed 24-bit counts (-8388608 to 8388607). Invalid arguments throw
`prosdk::RuntimeError` (`ParameterOutOfRange`, or `ChannelIndexOutOfRange` for the NAU7802
channel), as in the other drivers.

Typical use (the example):

```cpp
prosdk::NAU7802 m_nau7802{prosdk::I2C::Port::PORT_1};
prosdk::LoadCell m_cell{[this] { return m_nau7802.Update(); }};
// startup: m_nau7802.CalibrateInternal(); m_cell.SetScale(countsPerKg); m_cell.Tare();
// every loop: m_cell.Update(); m_cell.GetMass();
```

## 3. Behaviour and defaults

**NAU7802.** The constructor runs the 9.1 power-on sequence:
1. PU_CTRL = RR (0x01), then PU_CTRL = PUD (0x02);
2. poll PUR every 200 us, at most 50 times (the data sheet says "about 200 microseconds");
3. read the revision register 0x1F: bits 3:0 must be 1111 (11.15.3). A mismatch only prints a
   warning;
4. CTRL1 = 0x2F (VLDO 3.0 V, CRP 0 = DRDY active high, gain x128), written before the analog part
   and its LDO power up;
5. PU_CTRL = 0x86 (AVDDS, PUA, PUD);
6. REG0x15 = 0x30 (REG_CHPS = 11, the only valid value, 11.10);
7. CTRL2 = 0x30 (80 SPS, channel 1, CALMOD 00);
8. PU_CTRL = 0x96 (+ CS: conversions start).

Registers 0x11, 0x1B and 0x1C keep their reset values: standard I2C (no streaming modes), the
weak internal pull-ups, LDOMODE 0, and **PGA_CAP_EN off**, because that capacitor sits across the
VIN2 pins and would break channel 2 (the Boardoza library sets it). The chip itself holds CR back
for the 6 conversions after a reset (8.8).

`Update()` reads PU_CTRL (1 byte). With CR set it reads 3 bytes from 0x12 (reading 0x12 latches
the full result, 11.9) and sign-extends them. The driver assumes that this read clears CR; the
data sheet does not say what clears CR (**unverified**). If PUR is clear, the chip was missing
at construction or reset by a supply dip: the driver runs the power-on sequence again, at most
once per second, and prints a warning. The calibration is lost then. A missing chip fails on the
first write at once; a chip that answers but never sets PUR makes that `Update()` wait up to
about 10 ms.

`SetGain`, `SetSampleRate` and `SetChannel` rewrite CTRL1 or CTRL2. When the value changes, the
next 6 conversions are dropped (the data sheet gives no settling count for these changes, so the
reset value of 6 is reused: **unverified**). At 80 SPS that is 75 ms, at 10 SPS 0.6 s. The two
channels share one converter: switching channels every loop never yields a sample. Stay on one
channel for at least 7 conversions.

`CalibrateInternal()` writes CTRL2 with CALS = 1 and CALMOD = 00 (internal offset), polls CTRL2
every 1 ms until CALS clears (at most 2 s; the data sheet gives no calibration time:
**unverified**) and returns false if CAL_ERR is set (8.6.3). It calibrates the selected channel;
each channel has its own offset and gain registers. The next 6 conversions are dropped. The data
sheet recommends calibrating after power-up and after a gain, rate, channel or supply change, or a
large temperature change (8.6). The system offset and system gain modes are not offered.
Call it at startup (the robot constructor or an init method), not from a periodic method: 2 s is
probot-core's loop stall deadline (`PROBOT_LOOP_DEADLINE_MS`).

**HX711.** The constructor allocates the pins and drives PD_SCK low. `Update()` reads DOUT. When it
is low (a conversion is ready), one `DigitalShift::ShiftIn` clocks 25, 26 or 27 bits with 1 us
high and 1 us low times (the typical T3 and T4; T3 must stay within 0.2 to 50 us). Then:
- the first 24 bits are the result, MSB first, two's complement; the chip saturates at 0x800000
  and 0x7FFFFF;
- the 25th pulse pulls DOUT back high, so the remaining 1 to 3 bits must read 1. Zeros there mean
  DOUT is stuck low (no chip or a broken line): the read is dropped and `IsConnected()` turns false;
- the pulse count selects the next conversion: 25 = channel A gain 128, 26 = channel B gain 32,
  27 = channel A gain 64 (Table 3). Fig. 2 labels the 27-pulse case "CH.B, Gain:64"; Table 3 and
  the "Analog Inputs" text (channel A takes 128 or 64, channel B a fixed 32) agree on channel A.

A conversion made with another selection is dropped, and so are the next 4: the output settling
time is 400 ms at 10 SPS and 50 ms at 80 SPS, 4 periods at either rate. At construction the chip's
selection is unknown, so the first 5 conversions are dropped and the first value comes with the
6th (about 75 ms at 80 SPS, 0.6 s at 10 SPS).
With DOUT high nothing is clocked. `IsConnected()` turns false when no conversion was ready for
0.5 s (5 periods at 10 SPS), checked on each `Update()`.

`PowerDown()` drives PD_SCK high; the chip powers down after 60 us (Fig. 3) and `Update()` does
nothing. `PowerUp()` drives it low: the chip resets to channel A gain 128, and the 4 settling
conversions are dropped (5 if another gain is selected). With the chip's regulator feeding the
bridge (as on this board), power-down also turns the bridge excitation off ("Reset and
Power-Down").

**LoadCell.** Mass = (counts - offset) / countsPerKg. `Update()` calls the source once and stores
a new sample. `Tare(n)` is non-blocking: the next n new samples are averaged into the offset; the
old offset (0 at first) stays in use until then, and `IsTaring()` is true meanwhile. The default
scale is 1 count/kg, so `GetMass()` reads counts until `SetScale()`. A negative scale is
allowed (a cell mounted the other way round); 0, NaN and infinity throw. Calibrate with a known
mass m: tare unloaded, load the cell, then `SetScale(GetCounts() / m)`. The offset and scale are
not stored across restarts; put the scale in code (the example) and tare at startup.

## 4. Verification

Host tests (`prosdk_drivers_tests`):
- **NAU7802** (`[nau7802]`, FakeI2CBus with an emulated chip: RR resets every register, PUR
  shows two PU_CTRL reads after PUD, CR only while CS is set, reading 0x12 latches and clears CR,
  CALS clears after 3 reads):
  - the exact power-on write sequence, the PUR poll, PGA_CAP_EN cleared by the reset;
  - conversions 0x7FFFFF, 0x800000, 0xFFFFFF, 0x000001 (+8388607, -8388608, -1, +1), no result
    read without CR;
  - the CTRL1/CTRL2 values for gain, rate and channel, the 6 dropped conversions, no drop for an
    unchanged value, argument errors;
  - calibration (CTRL2 = 0x34, then 0xB4 on channel 2), CAL_ERR, a NACK;
  - a missing chip, a board plugged in later (set up again after 1 s), a NACK and recovery.
- **HX711** (`[hx711]`, a fake chip on the DIO sim driven by the PD_SCK edges, as in F3's
  DigitalShiftTest):
  - 25-pulse reads and the sign extension, 5 dropped conversions at start;
  - 26 and 27 pulses after `SetGain`, with the switch read and 4 settling conversions dropped;
  - the 0.5 s timeout, DOUT stuck low;
  - power down (PD_SCK high, nothing clocked) and up (4 or 5 dropped).
- **LoadCell** (`[loadcell]`): a running tare over 4 samples with a poll that brings nothing,
  scale, a negative scale, argument errors.

In this package the files were checked only with `-fsyntax-only` (host, and ESP32-S3 for
`src/`). The example's robot class was checked the same way with `<prosdk.h>` replaced by the
telemetry header (probot-core is not on the syntax checker's include path). The tests have not
been built or run yet, and the example has not been compiled with arduino-cli.

## 5. Unverified

- **No board has been tested on hardware.** The drivers are checked only against data sheet
  behaviour on the simulation HAL.
- **NAU7802:**
  - that CR clears when the result is read (the data sheet does not say). If it stayed set until
    the next conversion, `Update()` would return the same conversion more than once;
  - the LDO's current above its guaranteed minimum of 10 mA at 3.0 V (section 3), so whether
    more than one 350 ohm bridge (about 8.6 mA each) can be excited;
  - the 6 dropped conversions after a gain, rate or channel change, and after a calibration;
  - the 2 s calibration timeout (no calibration time in the data sheet);
  - that the revision register reads 0xF in bits 3:0 on real chips (only a warning);
  - the schematic reading: REFP/REFN, VDDA on RED and J3, no filter capacitor across VIN2;
  - that the board's 3.3V pin may take 5 V (README: 2.7 to 5.5 V). It would pull SDA/SCL to 5 V:
    do not.
- **HX711:**
  - the board's VCC/VDD assignment, the AVDD regulator divider and the JP1 default (Wiring);
  - that DOUT is already high when the 25th pulse is sampled (1 us after its rising edge; the
    data sheet gives T2 <= 0.1 us for data, and Fig. 2 shows DOUT going high at the 25th pulse).
    If not, every read would be dropped as "stuck low";
  - the real bit-bang timing on the ESP32-S3 and the masked windows (see F3);
  - that a read never straddles the end of the next conversion (a read takes about 60 us of a
    12.5 ms or 100 ms period);
  - power-down: the plan lists "HX711 power-down avoidance" as a hardware item. On the robot an
    emergency stop drives PD_SCK low, which only resets the chip; the first ShiftIn after the
    e-stop fails (one console line), after which `Update()` no longer clocks and `IsConnected()`
    stays false until reboot.
- **LoadCell:** linearity, creep and temperature drift of real cells.

## 6. Resource use

- **No threads, no heap allocation** (except `LoadCell`'s `std::function`, which may allocate for
  a large capture; a `[this]` lambda fits the small-object buffer of common implementations).
- **NAU7802:** one I2C device on a port. Per `Update()`: a 1-byte status read (4 bytes on the bus
  with address and register bytes), plus a 3-byte result read (6 bytes) when a conversion is
  ready: about 0.25 ms at 400 kHz, 0.9 ms at 100 kHz. The constructor waits up to about 10 ms;
  `CalibrateInternal()` blocks until the chip finishes (at most 2 s).
- **HX711:** two GPIOs (one `DigitalOutput`, one `DigitalInput`); no LEDC, RMT or PCNT unit. A
  read busy-waits about 60 us on the calling thread (25 to 27 bits at 2 us each plus call
  overhead), with interrupts masked for about 1 us per bit on the robot (F3). Without a ready
  conversion `Update()` is one GPIO read.
- **LoadCell:** a few doubles and a mutex.

## 7. Deviations from the plan

- `Update() -> std::optional<int32_t>` on both ADCs is the non-blocking read (the plan's HX711
  `Update()` plus a "new sample" result), so either can be a `LoadCell` source directly.
  `GetRaw()` returns the last sample, also on the NAU7802 (the plan wrote `GetRaw() -> int32_t`
  without saying whether it reads the chip).
- `LoadCell::Tare(n)` is non-blocking (it averages the next n samples from `Update()`), because
  a blocking tare would stall the robot loop for n conversion periods (1 s for 10 samples at
  10 SPS). Additions: `IsTaring()`, `GetCounts()` (needed to calibrate), `GetScale()`,
  `GetOffset()`, `HasData()`.
- Additions on the ADCs: `IsConnected()`, `HX711::GetGain()`, and telemetry.
- Not done (minimal): NAU7802 voltage conversion, temperature sensor (REG0x11 TS), system offset
  and gain calibration, DRDY pin, power-down; HX711 `IsReady()`.

## 8. Shared edits (integration)

- `design/drivers.md`: a link to this file in the "Boardoza boards" section, and the three test
  files in the "Tests" list.
- `design/API-DIFFERENCES.md` section 7: rows for `prosdk::NAU7802`, `prosdk::HX711` and
  `prosdk::LoadCell` (hand-written, no WPILib counterpart).
- `tools/upstream/examples.json` ("handwritten") and `design/examples.md`: `BoardozaLoadCell`
  (`examples/BoardozaLoadCell/BoardozaLoadCell.ino`), then `python3 tools/upstream/examples.py
  --check`.
- `design/boardoza.md` board index: the NAU7802 and HX711 rows link here.
