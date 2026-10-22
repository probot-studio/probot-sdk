# P5: I2C actuators and utilities

Package P5 of the Boardoza plan. Five hand-written I2C drivers, each a small class on
`prosdk::I2C` with no thread:

- `prosdk::DRV2605L`: haptic driver for an ERM or LRA vibration motor;
- `prosdk::PCF8563`: real-time clock;
- `prosdk::GP8403`: dual 0-5 V / 0-10 V DAC;
- `prosdk::I2CSwitch`: 1-to-4 I2C switch (TPT29545), so devices with the same address can share
  a port;
- `prosdk::NT3H2111`: the I2C host side of an NTAG I2C plus NFC tag.

| Board | Ozdisan id | Chip | Class | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| DRV2605L haptic driver | 1564792 | TI DRV2605L | `DRV2605L` | DRV2605L_Haptic_Driver_Breakout_Board |
| PCF8563T RTC | none (repo only) | NXP PCF8563T | `PCF8563` | PCF8563T_RTC_I2C_Breakout_Board |
| GP8403 dual DAC | 1595259 | Guestgood GP8403 | `GP8403` | GP8403_Dual_Channel_I2C_DAC_Breakout_Board |
| TPT29545 I2C bus switch | 1591106 | 3PEAK TPT29545 | `I2CSwitch` | TPT29545_I2C_Bus_Switch_Breakout_Board |
| NT3H2111 NFC tag | 1206521 | NXP NT3H2111W0FHK | `NT3H2111` | none (Ozdisan only) |

Files:
- `src/prosdk/drivers/haptic/DRV2605L.hpp` and `.cpp`
- `src/prosdk/drivers/rtc/PCF8563.hpp` and `.cpp`
- `src/prosdk/drivers/dac/GP8403.hpp` and `.cpp`
- `src/prosdk/drivers/bus/I2CSwitch.hpp` and `.cpp`
- `src/prosdk/drivers/memory/NT3H2111.hpp` and `.cpp`
- Tests (host only): `test/prosdk/drivers/DRV2605LTest.cpp`, `PCF8563Test.cpp`,
  `GP8403Test.cpp`, `I2CSwitchTest.cpp`, `NT3H2111Test.cpp`
- No example.

Sources (manufacturer data sheets; no vendor code was used):
- TI DRV2605L, SLOS854D (March 2018), from the board repo: register map (8.6, tables 3 and 4), I2C
  (8.5.3.1), GO bit (8.4.3), standby (8.4.1.4), voltage programming (8.5.2.1 and 8.5.2.2), RTP
  format (8.5.8.1, figures 26 and 28), waveform sequencer (8.5.8.2.5).
- NXP PCF8563 product data sheet rev. 10 (3 April 2012), from the board repo: tables 4 to 17,
  22 and 27; sections 8.5 (reading and setting the time), 8.10 (STOP bit), 9.5.1 (address).
- GP8403 data sheet (Keyi, translated by StanStrong), from the board repo: sections 3.3.3 to
  3.3.7. The register figures are images; the byte layout was cross-checked with the Boardoza
  GP8403 library (read only).
- 3PEAK TPT29545 data sheet CA20241229A1, from the board repo: "Control Register", table 2 and
  the timing table (RESET).
- NXP NT3H2111/NT3H2211 product data sheet **rev. 3.0 (3 February 2016)**, from a Farnell copy
  (nxp.com refused the download; the current revision is 3.6 of July 2023, not read): sections
  2.4, 8.3.2 (table 6), 8.3.4, 8.3.12 (tables 11 to 14), 8.4, 8.5, 9.6 (table 15), 9.7, 9.8, 11,
  13.1.
- The board repos (README, KiCad schematic and PCB) for pins, pull-ups and jumpers.

## 1. Wiring and hazards

**Power every P5 board from 3.3 V when it is on an ESP32-S3 I2C bus.** The ESP32-S3 pins are not
5 V tolerant, and these boards pull SDA and SCL up to their own VCC:

- DRV2605L: 2.2k pull-ups to VCC (R1, R2). VCC also feeds the motor H-bridge (2.0-5.2 V), so at
  3.3 V the motor gets at most about 3.3 V.
- GP8403: 4.7k pull-ups to the VCC input (R6, R7), and VCC may be 3.3-12 V on this board. Use
  3.3 V: the on-board MT3608 boost still makes the 12 V the GP8403 needs. The VOUT0/VOUT1 screw
  terminals reach 10 V: never connect them to an ESP32 pin.
- PCF8563T: four 10k resistors, assumed to be the SDA, SCL, nINT and CLK_O pull-ups to VCC
  (the nets were not traced; **unverified**). VCC 2.5-5.5 V.
- TPT29545: 4.7k pull-ups to VCC on the host side and on every downstream channel (16 resistors).
  All four downstream buses therefore run at the board VCC.
- NT3H2111: the chip's VCC range is 1.67-3.6 V (absolute maximum 4.6 V, datasheet 12 and 13.1).

Every board adds its pull-ups in parallel with the others on the same bus (and with probot-core's
battery sensor on port 0). Several boards with 2.2k or 4.7k pull-ups on one bus lower the
resistance; check the total stays above about 1 kOhm (3 mA sink at 3.3 V).

| Board | Connector | Pins |
|---|---|---|
| DRV2605L | J1 | 1 VCC, 2 IN (IN/TRIG; tie to GND if unused), 3 SCL, 4 SDA, 5 GND |
| DRV2605L | J2 | 1 OUT+, 2 OUT- (to the motor) |
| PCF8563T | J1 | 1 VCC, 2 nINT (open drain), 3 SDA, 4 SCL, 5 CLK_O, 6 GND |
| GP8403 | J1 | 1 VCC, 2 SCLK, 3 SDA, 4 GND |
| GP8403 | J2 (screw) | 1 OUT0, 2 GND, 3 OUT1 |
| TPT29545 | J2 | 1 VCC, 2 /INT, 3 SCL, 4 SDA, 5 A0, 6 A1, 7 /RESET, 8 GND |
| TPT29545 | J1 | 1 SCL1, 2 SDA1, 3 /INT1, 4 SCL0, 5 SDA0, 6 /INT0 |
| TPT29545 | J3 | 1 SCL3, 2 SDA3, 3 /INT3, 4 SCL2, 5 SDA2, 6 /INT2 |

Addresses (7-bit):

| Board | Address | Notes |
|---|---|---|
| DRV2605L | 0x5A | fixed (datasheet 8.5.3.1) |
| PCF8563T | 0x51 | fixed (A2h/A3h, datasheet 9.5.1) |
| GP8403 | 0x58 | 0x58 + (A2 << 2 \| A1 << 1 \| A0), up to 0x5F, with the A0/A1/A2 solder jumpers moved to the VCC side; from the silkscreen table and the Boardoza library (**unverified** against the chip: the datasheet's address figure is an image) |
| TPT29545 | 0x70 | 0x70-0x73 with A0/A1 tied to VCC (10k pull-downs on the board); from the README (the datasheet's address figure is an image) |
| NT3H2111 | 0x55 | default, 1010101b (datasheet table 15); can be changed by writing block 0 byte 0, which this driver never does |

The PCF8563 at 0x51 and the GP8403 at 0x58-0x5F do not clash with each other or with the
DRV2605L. A PCA9685 all-call address (0x70) clashes with the TPT29545 default.

### NT3H2111 board

The Ozdisan NT3H2111 product is a credit-card-sized PCB tag with an etched antenna. Its photo shows
no header, so the I2C side (SDA, SCL, VCC, FD) is probably not reachable on that card, and it
works as a passive tag read by phones (**unverified**: no schematic or pinout is published). The
class serves any NT3H2111 whose I2C pins are wired: VCC 3.3 V, SDA, SCL, GND, and optionally FD
(open drain, pull-up to VCC; the datasheet suggests more than 2 kOhm) to a GPIO.

## 2. `prosdk::DRV2605L`

```cpp
DRV2605L haptic{I2C::Port::PORT_0, DRV2605L::Actuator::kERM};
haptic.SetWaveform(0, 47);   // ROM effect 47 in slot 0
haptic.SetWaveform(1, 0);    // end of sequence
haptic.Go();                 // returns at once; IsPlaying() polls the GO bit
haptic.SetRealtime(0.6);     // continuous drive, -1 to 1
haptic.Stop();
```

- Constructor: reads STATUS (DEVICE_ID must be 7, DRV2605L), leaves standby in internal-trigger
  mode, sets FEEDBACK N_ERM_LRA for the actuator, selects library 1 (TS2200 A) for ERM or 6 (LRA),
  sets RTP_INPUT to 0 and clears the 8 sequencer slots. A missing chip or another id prints one
  console line; `IsConnected()` is then false. When the chip did not answer, the next call
  (`SetLibrary`, `SetWaveform`, `Go`, `SetRealtime`) repeats this configuration first.
- `SetLibrary(0..7)`, `SetWaveform(slot 0..7, effect 0..123)` (0 ends the sequence), `Go()`,
  `Stop()`, `IsPlaying()`. Effect names are in TI's waveform library effects list (not part of
  SLOS854D); prosdk does not list them.
- `SetRealtime(value)`: MODE 5, signed RTP data `round(value x 127)`: 0x7F full scale, 0x00 no
  drive. Negative values: ERM (open loop, figure 26) 0x81 is full reverse drive (braking); LRA
  (closed loop with bidirectional input, the reset setting, figure 28) treats them as 0, no
  drive. NaN gives 0. `Go()` and `Stop()` leave real-time mode (MODE 0).
- Destructor: GO 0, RTP 0, then STANDBY.
- Defaults kept from reset: ERM open loop with OD_CLAMP 0x8C as full scale (21.59 mV x 140, about
  3.0 V, section 8.5.2.1, equation 6); LRA closed loop (bidirectional input) with the reset
  compensation values; the LRA peak clamp is 21.22 mV x 140, about 3.0 V (equation 9). There is
  **no auto-calibration**, no rated-voltage setting and no IN/TRIG modes (the plan keeps the class
  minimal). Whether the reset clamp suits a given LRA, and how well an uncalibrated LRA plays, is
  **unverified**: check the motor's rating before use.
- **Not gated**: the HAL does not know about the chip. An effect or a real-time amplitude keeps
  playing while the robot is disabled or e-stopped, until `Stop()` or the destructor. It drives a
  small vibration motor only.

## 3. `prosdk::PCF8563`

```cpp
PCF8563 rtc{I2C::Port::PORT_0};
if (auto now = rtc.GetTime()) { /* std::tm, 2000-2099 */ }
rtc.SetTime(tm);  // tm_year 100..199, tm_mon, tm_mday, tm_hour, tm_min, tm_sec
```

- Constructor: Control_status_1 = 0 (clears TESTC, STOP, TEST1), Control_status_2 = 0 (alarm and
  timer interrupts off), CLKOUT_control = 0 (FE = 0: the 32.768 kHz output that runs after
  power-on is turned off). The time is kept.
- `GetTime()`: one burst read of registers 02h-08h (section 8.5 requires one access). Empty when
  the chip does not answer, when VL is set (the clock lost power: no battery; invalid until
  `SetTime()`), or when a register holds an impossible value. tm_wday and tm_yday are computed
  from the date; the chip's weekday register is ignored.
- `SetTime()`: validates the fields (throws otherwise; a leap second 60 is written as 59),
  computes the weekday (0 = Sunday), sets STOP, writes 02h-08h in one burst with VL = 0 and C = 0,
  then releases STOP. The first tick follows 0.507813-0.507935 s after the release (table 26).
  Returns false when the chip did not answer (the release is still attempted).
- Century: years 2000-2099 only. C is written 0 and ignored on read.
- Not implemented (plan: minimal): alarm, countdown timer, clock output, nINT.

## 4. `prosdk::GP8403`

```cpp
GP8403 dac{I2C::Port::PORT_0, 0x58, GP8403::Range::k0to10V};
dac.SetVoltage(0, 2.5_V);
dac.SetBoth(0_V);
```

- Encoding (sections 3.3.3-3.3.6): register 0x02 = VOUT0, 0x04 = VOUT1, two bytes each, low byte
  first, 12-bit code left-justified (code << 4; the chip ignores the low 4 bits). Register 0x02
  with 4 data bytes writes both. Register 0x01 = range: 0x00 = 0-5 V, 0x11 = 0-10 V, one range for
  both outputs. Code = round(V / full scale x 4095); 2.5 V is 2048 (bytes 00 80) at 0-5 V and 1024
  (bytes 00 40) at 0-10 V.
- Constructor: both outputs 0 V, then the range (the chip's power-on range and value are not
  documented). Destructor: both outputs 0 V. Until a range write has been acknowledged (for
  example the board was not powered at construction), every setter first writes 0 V to both
  outputs, the range, then the stored values, so no code is applied with an unknown range.
- `SetVoltage(channel, V)` and `SetBoth(V)` clamp to 0 .. full scale (NaN gives 0) and write
  only when the code changed or the last write failed. `GetVoltage()` returns the last commanded
  value after rounding (the chip is write-only: no read-back). `IsConnected()` is the last write's
  acknowledge.
- `SetRange()`: both outputs to 0 V, the range, then the stored voltages again (clamped to the new
  range), so an output never jumps to twice its value during the change.
- The non-volatile "store" command (3.3.7) is not implemented: its sequence is only in a figure,
  and a robot DAC should not come back at a nonzero voltage after power-up.
- **Not gated**: the outputs keep their voltage while the robot is disabled or e-stopped (the HAL
  gate covers GPIO outputs only). Do not use the DAC as the only command of an actuator that must
  stop on disable (for example a VFD or motor controller speed input) unless that device has its
  own enable that is gated.
- Output error 0.5 % of range and 20 mA per output (Table-C). The data sheet gives no settling
  time for I2C writes (the 100-200 us in Table-B is for its PWM input).

## 5. `prosdk::I2CSwitch`

```cpp
I2CSwitch mux{I2C::Port::PORT_0};          // 0x70, 4 channels
PCF8563 rtc{I2C::Port::PORT_0};             // behind channel 1
mux.SelectOnly(1);
auto now = rtc.GetTime();
```

- Protocol: one control byte, no register address; bit n enables channel n, any combination; the
  last byte of a write is kept; the new selection takes effect at the STOP; 0 (no channel) after
  power-up or /RESET. `Select(mask)` writes it, `SelectOnly(channel)` writes one bit,
  `GetSelected()` reads it back masked to the channel bits (B7-B4 are undefined on the TPT29545,
  table 2).
- `channels` (default 4) may be 1-8, so 8-channel switches with the same one-byte protocol
  (TCA9548A style) work too (not tested on hardware).
- Constructor: deselects every channel.
- **Threading rule**: the selection is global bus state. Select, then access the device from the
  same thread. Drivers that read the bus on their own thread (MPU6050, PCA9685 with an update
  period > 0, GNSS, ...) must not sit behind the switch unless every driver on that port is
  constructed with an update period of 0 and updated after `Select()`. There is no bus-wide lock.
- Devices with the same address on different channels must never be enabled together. The
  switch itself answers at 0x70-0x73 on every enabled segment.
- /INT and /RESET are not handled: both are pulled up on the board. /RESET can be driven by a
  `DigitalOutput` (a low pulse; tW(RST)L is at least 4 ns, timing table "RESET") to recover a
  stuck downstream bus; /INT can be read with a `DigitalInput`.

## 6. `prosdk::NT3H2111`

```cpp
NT3H2111 tag{I2C::Port::PORT_0, 0x55, /*fdGpio*/ 4};
std::array<uint8_t, 16> block;
tag.ReadBlock(1, block);
tag.WriteBlock(1, block);
tag.ReleaseToNfc();
bool phone = tag.IsFieldPresent();
```

Block map from the I2C side, NT3H2111 (1k) (table 6):

| Block | Content | Driver |
|---|---|---|
| 0x00 | byte 0: I2C address (reads 04h), serial number, static lock bytes, capability container | read only |
| 0x01-0x37 | user memory | read and write |
| 0x38 | bytes 0-7 user memory; dynamic lock bytes, AUTH0 | read only |
| 0x39 | ACCESS, PWD, PACK, PT_I2C | read only |
| 0x3A | configuration registers | read only |
| 0xF8-0xFB | SRAM, 64 bytes, volatile, only while VCC is powered | read and write |
| 0xFE | session registers (register access only) | NS_REG only |

- Block read: block address in one write, STOP, 16-byte read (figure 18). Block write: block
  address and 16 bytes in one write; the STOP starts the EEPROM write cycle.
- EEPROM write time: 4.5 ms per block including overhead (section 2.4); the chip ignores the bus
  meanwhile. The driver does not block in `WriteBlock()`; the next access to the chip waits until
  5 ms after the write (the 0.5 ms margin is ours, not the datasheet's). SRAM writes do not wait.
- Never written: block 0 (its byte 0 changes the I2C address), 0x38-0x3A (lock, password and
  configuration bytes, whose effects can be permanent). `WriteBlock()` throws for them.
- Constructor: reads block 0 and checks byte 0 = 04h (NXP manufacturer id, table 6 note 1).
- `IsFieldPresent()`: with an FD pin, FD low (reset configuration FD_ON = 00b, FD_OFF = 00b:
  pulled low when the field switches on, released when it switches off, table 13); without, the
  RF_FIELD_PRESENT bit of NS_REG (session block byte 6, table 14).
- `ReleaseToNfc()`: register write [FEh, 06h, mask 40h, 00h] clears I2C_LOCKED (section 11.1.1),
  so a phone can use the memory at once; otherwise the watchdog clears it 20 ms after the last
  I2C access (section 8.5, reset value).
- Arbitration (section 11): while a phone holds the memory (RF_LOCKED), block reads and writes
  are not acknowledged and return false; only the session registers answer.
- The 50 us pause the chip needs between the STOP and the next START when I2C_CLOCK_STR = 0
  (section 9.7) is not added: the driver relies on the reset value I2C_CLOCK_STR = 1.
- Not implemented (plan: minimal): NDEF parsing, pass-through and mirror modes, passwords, lock
  bits, the configuration registers, the NT3H2211 (2k) blocks 0x40-0x7F.
- Not read: the current datasheet revision 3.6 and the NXP errata sheet, which describes possible
  EEPROM data corruption when RF and I2C accesses overlap. Avoid writing from I2C while a phone is
  in the field.

## 7. Resource use

- I2C only (one port, any of the two); no thread, no HAL resource. One optional GPIO input for
  the NT3H2111 FD pin.
- Bus time at 400 kHz, approximate: DRV2605L register write about 70 us; PCF8563 time read about
  0.25 ms; GP8403 channel write about 75 us (both channels about 0.13 ms); I2CSwitch select about
  45 us; NT3H2111 block read about 0.45 ms and write about 0.4 ms, plus the 5 ms EEPROM cycle
  before the next access.
- Every call runs on the caller's thread; none blocks longer than one transfer, except an
  NT3H2111 access right after an EEPROM write (up to 5 ms).

## 8. Verified and unverified facts

Verified against the data sheets (sections cited above and in the source comments): every
register address, bit and reset value used; the DRV2605L, PCF8563 and NT3H2111 addresses; the
TPT29545 control byte protocol; the GP8403 range values and 12-bit left-justified encoding (text
of 3.3.3-3.3.6); the NT3H2111 block map, block and register framing, session register layout,
FD polarity at reset configuration and EEPROM write time.

Unverified (marked in the code too):
- GP8403 base address 0x58 and the jumper-to-bit order (silkscreen and library only); its
  power-on range and value.
- TPT29545 address range (README; the figure is an image). The read bits B7-B4 are "x" in
  table 2 and are masked.
- PCF8563 board pull-up nets; the coin-cell size (README CR1220, schematic CR1225).
- DRV2605L LRA playback without auto-calibration.
- NT3H2111: whether the Ozdisan card exposes I2C at all; the 0.5 ms margin; rev. 3.6 and errata.
- Everything on hardware: none of the five drivers has run on a real board yet. The host tests
  cover register sequences, framing, encoding and error paths only.

## 9. Tests

Host tests in `prosdk_drivers_tests`, on `test::FakeI2CBus` (F2) and the DIO sim:

- `DRV2605LTest.cpp` (`[drv2605l]`): "DRV2605L configures the chip for the actuator", "DRV2605L
  sequencer and GO", "DRV2605L real-time playback", "DRV2605L reports a missing or different
  chip", "DRV2605L retries the configuration".
- `PCF8563Test.cpp` (`[pcf8563]`): "PCF8563 configures the chip and reports the VL flag",
  "PCF8563 BCD round trip", "PCF8563 rejects invalid times and a missing chip".
- `GP8403Test.cpp` (`[gp8403]`): "GP8403 sets the range and 0 V at construction", "GP8403 codes
  for 2.5 V at both ranges", "GP8403 retries a failed write", "GP8403 writes the range before any
  code after a failed construction".
- `I2CSwitchTest.cpp` (`[i2cswitch]`): "I2CSwitch writes the channel mask", "I2CSwitch reaches a
  device behind a channel" (a PCF8563 that answers only while channel 1 is selected), "I2CSwitch
  reports a missing switch".
- `NT3H2111Test.cpp` (`[nt3h2111]`): "NT3H2111 identifies the chip from block 0", "NT3H2111
  16-byte block framing" (including the EEPROM wait, measured on the real clock), "NT3H2111
  session register: field and release", "NT3H2111 field detect pin".

## 10. Shared edits for the integration step

- design/drivers.md: a link to this file in the "Boardoza boards" section, and the five test
  files in the "Tests" list.
- design/API-DIFFERENCES.md section 7: one row each for `prosdk::DRV2605L`, `prosdk::PCF8563`,
  `prosdk::GP8403`, `prosdk::I2CSwitch`, `prosdk::NT3H2111` (hand-written, no WPILib
  counterpart).
- design/boardoza.md (board index): the five rows above, linked here.
