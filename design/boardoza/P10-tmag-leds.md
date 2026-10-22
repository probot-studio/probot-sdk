# P10: SPI magnetic sensor and clocked LEDs

Package P10 of the Boardoza plan. It adds two drivers on the prosdk SPI bus (F1,
`design/boardoza/F1-spi.md`):

- `prosdk::TMAG5170`: Texas Instruments TMAG5170 3-axis Hall-effect sensor (32-bit frames with a
  CRC4 on every frame);
- `prosdk::ClockedAddressableLED`: clocked (data + clock) addressable RGB LEDs, the Harvatek
  T3A33BRG of the Phoenix family and the Rotary Encoder ARGB board. The frame is the APA102 one
  (0xE0 | brightness, B, G, R per LED); APA102 and SK9822 strips were not tested (**unverified**).

| Board | Ozdisan id | Chip | prosdk | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| TMAG5170 | 1564797 | TI TMAG5170A1QDGKR | `TMAG5170` | TMAG5170_3D_Hall_Effect_Sensor_Breakout_Board |
| Phoenix4 | 1564796 | 4 x T3A33BRG | `ClockedAddressableLED` | Phoenix4_Modular_ARGB_LED_Breakout_Board |
| Phoenix6 | - | 6 x T3A33BRG | `ClockedAddressableLED` | Phoenix6_Modular_ARGB_LED_Breakout_Board |
| Phoenix12 | 1564794 | 12 x T3A33BRG | `ClockedAddressableLED` | Phoenix12_Modular_ARGB_LED_Breakout_Board |
| Phoenix24 | 1564800 | 24 x T3A33BRG | `ClockedAddressableLED` | Phoenix24_Modular_ARGB_LED_Breakout_Board |
| Phoenix12-WENC | - | 12 x T3A33BRG + EC11 encoder | `ClockedAddressableLED` + `Encoder` + `DigitalInput` | Phoenix12_WENC_ARGB_Encoder_Breakout_Board |
| Phoenix24-WENC | - | 24 x T3A33BRG + EC11 encoder | `ClockedAddressableLED` + `Encoder` + `DigitalInput` | Phoenix24_WENC_ARGB_Encoder_Breakout_Board |
| Rotary Encoder ARGB | - | 14 x T3A33BRG + KLS EC1101S | `ClockedAddressableLED` + `Encoder` + `DigitalInput` | Rotary_Encoder_ARGB_Control_Breakout_Board |

Files:
- `src/prosdk/drivers/magnetic/TMAG5170.hpp` and `.cpp`
- `src/prosdk/drivers/led/ClockedAddressableLED.hpp` and `.cpp`
- Tests (host only): `test/prosdk/drivers/TMAG5170Test.cpp`, `ClockedAddressableLEDTest.cpp`
- Example: `examples/BoardozaRotaryArgb/BoardozaRotaryArgb.ino` (encoder, button and LED ring)

Sources:
- TI TMAG5170 datasheet SBASAF4, September 2021 (in the board repo's assets): 6.5, 6.7, 6.8,
  7.4, 7.5.1, 7.5.2 (Figures 7-10 and 7-11, CRC equations 10 to 13), 7.6 (Tables 7-4 to 7-26).
- Harvatek T3A33BRG-H9C0002X1U1930 datasheet v2.0, 2024-10-24 (in each Phoenix repo's assets):
  page 7 (electrical), pages 9 and 10 (frame, fields, sleep).
- KLS EC11 / EC1101S encoder datasheets (repo assets) and the board READMEs, KiCad schematics
  and the Rotary Encoder ARGB PCB netlist, as summarized in the plan's research records.

No vendor code was used. The Boardoza TMAG5170 and T3A33BRG/EC1101S libraries (BSD-3-Clause)
were read but not copied; the TMAG5170 library has scaling and bit-field errors, and the LED
library sends no end frame. Both drivers are written from the datasheets.

## 1. Wiring and hazards

Each SPI bus gets its pins once with `prosdk::platform::SetSPIPins(bus, sck, mosi, miso)` before
the first device opens. Tie every board's GND to the controller's GND.

**The clocked LEDs need an SPI bus to themselves** (F1 rule: a device without chip select is
alone on its bus, because the LEDs latch every clock edge). Use bus 1 (SPI3) with MISO -1:
`SetSPIPins(1, sck, mosi, -1)`; host SCK goes to the board's CIN, MOSI to DIN. Every device with
a chip select (TMAG5170, flash, SD, LoRa, EVE) then shares bus 0.

### TMAG5170

| Connector | Pins |
|---|---|
| J1 | 1 nCS, 2 MISO (SDO), 3 MOSI (SDI), 4 SCLK, 5 nALERT |
| J2 | 1 VCC (2.3 to 5.5 V), 2 GND |

- **Power it from 3.3 V.** The chip's logic runs at VCC and the board has no level shifter: at
  5 V the chip drives SDO (MISO) up to VCC (VOH VCC - 0.4 V, 6.5), and the ESP32-S3 pins are not
  5 V tolerant.
- nALERT is an open-drain output; the driver does not use it (it reads over SPI). The board's
  4.7 kOhm R1 is probably its pull-up, and the four 0 Ohm links R4 to R7 and the two LEDs D1/D2
  were not traced (**unverified**).
- The bill of materials says TMAG5170A1 (+/-25/50/100 mT). The driver reads the version
  (TEST_CONFIG VER) and scales an A2 part (+/-75/150/300 mT) as well.

### Clocked LED boards

| Board | Connector | Pins |
|---|---|---|
| Phoenix4 | J2 (in) | 1 VCC, 2 CIN, 3 DIN, 4 GND; J1 (out) VCC, COUT, DOUT, GND; pads J3 to J8 repeat the nets |
| Phoenix6 | J2 (in) | 1 DIN, 2 CIN; J1 (out) 1 DOUT, 2 COUT; J3/J4 (power) 1 VCC, 2 GND |
| Phoenix12 | pads | J1/J2 VCC, J3 COUT, J4 DOUT, J5 CIN, J6 DIN, J7/J8 GND (README; the schematic has J1 to J16, SMD and through-hole copies, mapping **unverified**) |
| Phoenix24 | pads | J1/J2 VCC, J3 DIN, J4 CIN, J5 DOUT, J6 COUT, J7/J8 GND |
| Phoenix12-WENC | pads | J1 DOUT, J2 COUT, J3 DIN, J4 CIN, J5/J6 GND, J7/J8 VCC, J9 encoder B, J10 encoder A, J11 SW |
| Phoenix24-WENC | 1x11 | 1/2 VCC, 3 encoder B, 4 encoder A, 5 SW, 6 DOUT, 7 COUT, 8 DIN, 9 CIN, 10/11 GND |
| Rotary Encoder ARGB | J1 1x9 | 1 VCC, 2 SW, 3 DOUT, 4 COUT, 5 CIN, 6 DIN, 7 encoder B, 8 encoder A, 9 GND |

- LED supply: VDD 3.3 to 5.5 V (absolute maximum 6.5 V). Each colour sinks a constant 20 mA at
  full DIMMING, so a LED takes up to 60 mA at full white: Phoenix4 about 240 mA, Phoenix6 360 mA,
  Phoenix12 720 mA, Rotary board 840 mA, Phoenix24 1.44 A. Feed long chains from a separate 5 V
  rail, not from the controller's 3.3 V regulator, or keep the brightness low
  (`SetBrightness`).
- Logic: DIN/CIN VIH is 2.7 V minimum as an absolute value (page 7), so 3.3 V GPIOs drive the
  LEDs at VDD 5 V too, with about 0.6 V margin; keep the wires short.
- **5 V hazard:** DOUT/COUT are driven at VDD (VOH 4.5 V minimum at VDD 5 V). Never wire them
  back to an ESP32-S3 pin when the board runs at 5 V; they only feed the next board's DIN/CIN.
- The full 20 mA needs VDD - Vf >= 1.5 V (page 7). At 3.3 V supply this is probably not met for
  green and blue, which may be dimmer (an inference, **unverified**).
- **Encoder boards (WENC, Rotary): power them from 3.3 V.** The A, B and SW pull-ups go to VCC
  (Rotary board: from the PCB netlist; WENC boards: from schematic placement, **unverified**), so
  at 5 V those lines would be 5 V at the ESP32-S3 pins.
- Chaining: DOUT/COUT of one board go to DIN/CIN of the next; the length is the sum of the LED
  counts (for example three Phoenix4 boards: 12).

## 2. `prosdk::TMAG5170`

```cpp
TMAG5170(SPI::Port bus, int csGpio, int clockHz = 1'000'000);   // clamped to 78.125 kHz..10 MHz
bool Configure(Range range, Averaging averaging, AnglePlane plane = AnglePlane::kXY);
Field GetField();                      // {x, y, z} in units::millitesla_t
units::celsius_t GetTemperature();
units::degree_t GetAngle();            // 0 to 360, 1/16 degree steps
units::millitesla_t GetRange() const;
bool IsConnected() const;
int GetCrcErrors() const;
static uint8_t Crc4(uint32_t frame);
// Range: kLow (+/-25 mT A1, +/-75 mT A2), kMedium (+/-50 / 150), kHigh (+/-100 / 300)
// Averaging: k1x, k2x, k4x, k8x, k16x, k32x (10.0 to 0.4 ksps on 3 axes)
// AnglePlane: kXY, kYZ, kXZ
// TelemetryLoggable, type "Magnetic Sensor": X, Y, Z (mT), Angle (deg), Temperature (C),
// Connected, CRC Errors
```

- Frames (7.5.2.3, 7.5.2.4.1): 32 bits, MSB first, SPI mode 0, SCK at most 10 MHz (6.8). SDI:
  bit 31 read, bits 30..24 address, bits 23..8 data, bits 7..4 command (always 0: no trigger),
  bits 3..0 CRC. A read answers in the same frame: bits 31..24 status STAT11..STAT4
  (PREV_CRC_STAT, CFG_RESET, ALRT_STATUS1/0, X, Y, Z, T), bits 23..8 the register, bits 7..4
  STAT3..STAT0 (ERROR_STAT, then the bits that CMD1 selects), bits 3..0 CRC.
- CRC (7.5.2.5): polynomial x^4 + x + 1, initial value 1111b, over bits 31..4 followed by four
  zero bits; the same CRC protects both directions. It is on at power-up and the driver keeps it
  on. The chip ignores a frame with a wrong CRC and inverts the last CRC bit of its answer, so
  the driver's check of every answer's CRC catches both directions. A wrong answer CRC counts in
  `GetCrcErrors()`, fails the read and keeps the last good value.
- Construction: SPI mode 0 and the clock, then a write of TEST_CONFIG = 0 (CRC on; an earlier
  program, such as the vendor library, may have turned it off, which lasts until power-down),
  then the configuration:
  1. read AFE_STATUS (clears CFG_RESET, which is set after power-up);
  2. read TEST_CONFIG: VER 0 is A1, 1 is A2, anything else is "not connected";
  3. SENSOR_CONFIG = ANGLE_EN (plane) | MAG_CH_EN 7 (X, Y, Z) | the range code on all three
     axes; SYSTEM_CONFIG = 0 (regular 32-bit reads, trigger by SPI command, no diagnostics, no
     limit checks); DEVICE_CONFIG = CONV_AVG | OPERATING_MODE 2 (continuous) | T_CH_EN 1,
     MAG_TEMPCO 0 (no magnet temperature compensation);
  4. read the three registers back. `Configure` returns false (and `IsConnected()` is false)
     when a frame failed or a value differs.
  A missing chip (or an unknown VER) prints one console line; the constructor does not throw for
  it (the SPI
  constructor itself throws for a bus without pins or a chip select in use).
- Getters read the chip on every call (no thread): `GetField` three frames (X, Y and Z can come
  from consecutive conversions), `GetTemperature` and `GetAngle` one frame each. Before reading,
  a chip that is not configured (missing at construction, connected later) or that reported
  CFG_RESET since (a brown-out resets every register) is configured again; a reset prints one
  console line. A value read in a frame that shows CFG_RESET is not taken.
- Scaling: field B = signed result x range / 32768 mT (equation 1); with `k1x` the low 4 bits
  are 0 (12-bit data). Temperature T = 25 + (TEMP_RESULT - 17522) / 60 C (equation 3 with the
  typical TSENS_T0, TADCT0 and TADCRES of 6.5; no per-chip calibration, so expect a few degrees
  of offset and up to 3 % slope error: TSENS_T0 is 23 to 27 C, TADCRES 58.2 to 61.8 LSB per C).
  Angle = (ANGLE_RESULT & 0x1FFF) / 16 degrees (equation 7; the CORDIC resolves 1/4 degree).
- The angle needs a magnet whose field turns in the chosen plane, for example a diametrically
  magnetized magnet on a shaft end above the chip (XY plane). Gain and offset correction
  (MAG_GAIN_CONFIG, MAG_OFFSET_CONFIG), thresholds, ALERT, trigger and sleep modes and the
  special 12-bit two-channel frames are not used.

## 3. `prosdk::ClockedAddressableLED`

```cpp
ClockedAddressableLED(SPI::Port bus, int length, int clockHz = 4'000'000);  // 78.125 kHz..15 MHz
int GetLength() const;
void SetData(std::span<const AddressableLED::LEDData> ledData);
void SetData(std::initializer_list<AddressableLED::LEDData> ledData);
void SetBrightness(int level);   // 0 to 31, default 31
int GetBrightness() const;
// TelemetryLoggable, type "Addressable LED": Length, Brightness
```

```cpp
prosdk::platform::SetSPIPins(1, /*sck=*/39, /*mosi=*/40, /*miso=*/-1);
prosdk::ClockedAddressableLED ring{prosdk::SPI::kPort1, 12};        // Phoenix12
std::array<prosdk::AddressableLED::LEDData, 12> buffer;
prosdk::LEDPattern::Rainbow(255, 64).ApplyTo(buffer);
ring.SetData(buffer);
```

- The colours use `AddressableLED::LEDData`, so `LEDPattern::ApplyTo` fills the buffer as for a
  WS2812 strip. Element 0 goes to the LED nearest the DIN input. A shorter buffer turns the
  remaining LEDs off; a longer one is cut at the length.
- Frame (pages 9 and 10): 4 bytes 0x00 (32 zero bits), then per LED 0xE0 | brightness, blue,
  green, red (FLAG 111, DIMMING, B, G, R; MSB first), then 4 bytes 0xFF (end of frame) and
  ceil(N / 16) more bytes 0xFF (the datasheet asks for N/2 extra clocks for N LEDs so the last
  LED latches). Byte counts: 1 LED 13, 4 LEDs 25, 14 LEDs 65, 24 LEDs 106. DIN is latched on the
  rising CIN edge: SPI mode 0. CIN is at most 15 MHz.
- Set the length to the real number of LEDs in the chain. Each IC passes on what follows its own
  word (page 9), so a LED past the length probably takes the 0xFF end frame as a command (FLAG
  111, DIMMING 31, white) and lights full white (inferred, **unverified**).
- `SetData` and `SetBrightness` send a whole frame at once, on the caller's thread: 106 bytes at
  4 MHz is about 0.2 ms. The constructor sends an all-off frame (the power-up state is not
  stated in the datasheet).
- Brightness is the 5-bit DIMMING field (32-level current control, applied to every LED on top
  of the colours). Whether 0 turns the LEDs off or is the lowest current, and whether the 32
  levels are linear, are not stated (**unverified**). The driver never sends the sleep word
  (first byte 0xA0, page 10): its first byte is always 0xE0 or more. Sleep mode is not offered.
- Not gated: SPI is neither gated nor stopped by the emergency stop (F1), so the LEDs keep their
  colours while the robot is disabled and after an emergency stop. LEDs are not actuators, and
  LED strips are not gated in prosdk either (design/hal-esp32.md). One difference: after an
  emergency stop the RMT `AddressableLED` drives its data line low and refuses new sends, while
  this driver can still send from a thread that outlives the user task.
- The no-chip-select device and the 15 MHz cap are the only bus requirements; the LEDs give no
  answer, so there is no `IsConnected()`.

## 4. Encoder and button (Rotary, WENC boards)

These need no new class:
- `prosdk::Encoder(aGpio, bGpio)` (x4 decoding on a PCNT unit). The board has a 10 kOhm pull-up,
  a 10 kOhm series resistor and 100 nF on A and B, the RC filter the encoder datasheet recommends;
  the PCNT 1 us glitch filter is enough. Rotary Encoder ARGB (EC1101S): 20 pulses and 20 detents
  per turn, so 4 counts per detent and 80 per turn: `SetDistancePerPulse(0.25)` counts detents.
  WENC boards: the encoder variant (15 pulses / 30 detents or 20 / 20) is **unverified**.
- `prosdk::DigitalInput(swGpio)`: the button is active low (pressed = `!Get()`), with a pull-up
  and 100 nF on the board.
- Which way counts up, and which LED (index 0) sits where on the ring, are **unverified**; the
  Rotary board's chain runs U14 (index 0) to U1 (from the PCB netlist), their angles were not
  checked. `SetReverseDirection(true)` or an index offset fixes either.

`examples/BoardozaRotaryArgb` turns the knob into a lit pointer on the ring and steps the colour
with the button, at brightness 4.

## 5. Verified and unverified

Verified from the datasheets:
- TMAG5170: frame layout, status bit positions (Figure 7-11 read from the rendered page:
  STAT11 PREV_CRC_STAT, STAT10 CFG_RESET, STAT3 ERROR_STAT), CRC (the serial form matches the
  XOR equations 10 to 13 on 2000 random frames, and the disable command 0x0F000407 has CRC 7),
  register fields, field, temperature and angle equations (the 354.50 and 17.25 degree examples
  of 7.5.1.4 are test vectors), 10 MHz SCK, mode 0.
- T3A33BRG: frame, field order (B, G, R), end frame and N/2 clocks, sleep word, mode 0, 15 MHz,
  VIH 2.7 V, VOH 4.5 V at 5 V, 20 mA per channel.
- Board facts as listed in section 1 (from the READMEs, schematics and, for the Rotary board, the
  PCB netlist).

Unverified (needs hardware):
- None of this ran on hardware: both drivers are syntax-checked for the robot and tested on the
  host simulation only.
- TMAG5170: whether SDO carries a CRC while CRC_DIS = 1 (the construction write does not check
  its answer for this reason); whether STAT10 clears in the frame right after the AFE_STATUS
  read (the driver also clears its reset flag after a good read-back); writing the configuration
  while the chip converts (no datasheet restriction found); per-chip temperature accuracy; the
  board's R1, R4 to R7 and D1/D2.
- LEDs: DIMMING 0 semantics and linearity, the power-up state, colour at 3.3 V supply, what a
  LED past the configured length shows, APA102/SK9822 strips, reliable
  clock over long wires between chained boards (4 MHz default chosen conservatively), LED index
  positions, the Phoenix4 count (README says 24, the schematic has U1 to U4: 4 used here),
  Phoenix12 pad mapping.
- Encoders: counting direction; WENC encoder variant and pull-up topology; whether the slow RC
  edges cause extra counts without GPIO hysteresis.

## 6. Resource use

- TMAG5170: one chip select on an SPI bus (normally bus 0, shared), no thread, no timer. Each
  getter call is 1 to 3 frames of 4 bytes (about 35 us per frame at 1 MHz); a reconfiguration is
  8 frames.
- ClockedAddressableLED: a whole SPI bus (normally SPI3) with its SCK and MOSI pins, one GDMA
  channel pair while the bus runs (F1), a frame buffer of 4 N + 8 + ceil(N / 16) bytes plus N
  colours. No thread, no RMT channel (the RMT `AddressableLED` budget is unaffected).
- Encoder boards: one PCNT unit (of 4, shared with counters) and one DIO input.

## 7. Shared edits (integration)

- design/drivers.md: a link to this file.
- design/API-DIFFERENCES.md section 7: two rows:
  - `prosdk::TMAG5170`: not in WPILib; hand-written SPI driver, `(SPI::Port, csGpio)`.
  - `prosdk::ClockedAddressableLED`: not in WPILib (WPILib's `AddressableLED` is one-wire only);
    takes `AddressableLED::LEDData` for `LEDPattern`; whole SPI bus without chip select.
- tools/upstream/examples.json and design/examples.md: `BoardozaRotaryArgb` (prosdk-only
  example, Boardoza Rotary Encoder ARGB board: ClockedAddressableLED, Encoder, DigitalInput).
