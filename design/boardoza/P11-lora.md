# P11: LoRa radio

Package P11 of the Boardoza plan. It adds `prosdk::SX126x`, a driver for the Semtech SX1262 LoRa
transceiver on the prosdk SPI bus (F1, `design/boardoza/F1-spi.md`), written for the Ebyte
E22-900M22S module on the Boardoza E22-900M22S board.

| Board | Ozdisan id | Chip | prosdk | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| E22-900M22S | - (repo only) | Ebyte E22-900M22S module (Semtech SX1262, 32 MHz TCXO) | `SX126x` | E22-900M22S_Long_Range_LoRa_Breakout_Board |
| RFM68LCW (SenseTech) | - (repo only) | HopeRF RFM68LCW-868S2, chip not stated | none yet | SenseTech-Competition-2026 (folder RFM68LCW) |

The RFM68LCW's chip is unknown. Its specifications (+22 dBm, -129 dBm, LoRa 1.76-62.5 kbit/s)
look like an SX126x or LLCC68 part, but nothing confirms it. It is **not** supported until a
hardware test shows that it answers the SX126x command set (for example `Begin()` returning true
and two boards exchanging packets). If it does, `SX126x` applies unchanged, with the TCXO setting
and RF switch pins taken from its own datasheet.

Files:
- `src/prosdk/drivers/radio/SX126x.hpp` and `.cpp`
- Test (host only): `test/prosdk/drivers/SX126xTest.cpp`
- Example: `examples/BoardozaLoRa/BoardozaLoRa.ino`

Sources:
- Semtech SX1261/2 data sheet rev. 1.2, June 2019 (DS.SX1261-2.W.APP, 111 pages; the full data
  sheet with the command interface).
- Ebyte E22-900M22S user manual v1.4 (2020-11-26), in the board repo's assets.
- The board README and KiCad schematic (labels and parts), as summarized in the plan's research
  record.

Clean-room: no vendor code was used. The Boardoza Arduino library in the repo (its code appears
to be derived from LoRaRF-Arduino; licence statements disagree) and RadioLib were not copied.
One value comes only from the Boardoza example sketch and is marked unverified below: the TCXO
voltage (1.8 V).

## 1. Wiring and hazards

| Connector | Pins |
|---|---|
| J1 (SPI) | 1 VCC (3.3-5.5 V in, to the onboard TLV73333 3.3 V LDO), 2 MISO, 3 MOSI, 4 SCK, 5 nCS, 6 GND |
| J2 (control) | 1 NRST (active low), 2 BUSY, 3 DIO1 |
| J3 (RF switch) | 1 RXEN, 2 TXEN, 3 DIO2 |
| J4 | SMA female, 50 ohm antenna |

- **3.3 V signals.** The module runs at 3.3 V behind the LDO and the board has no level shifters,
  so every signal on J1-J3 is a 3.3 V line straight to the module (absolute maximum 3.7 V,
  Ebyte table 2.1). The ESP32-S3 drives them directly. Never connect them to 5 V logic. VCC may
  be 5 V; the regulator takes care of it.
- **RF switch.** The module's antenna switch is controlled by RXEN and TXEN (both high-active,
  Ebyte pin table). The board breaks them out on J3 and the schematic shows no pull resistor or
  jumper, so if neither is wired the switch is undriven and the range is poor or zero. Wire both
  to GPIOs (`rxEn`, `txEn`), or bridge TXEN to DIO2 on J3, pass `txEn = -1` and set
  `Config::dio2AsRfSwitch` (the chip then drives TXEN: DIO2 high only in TX, datasheet table
  13-33). RXEN always needs a GPIO in this driver. The Ebyte manual says DIO2 must not go to an
  MCU pin when it drives TXEN.
- **Antenna.** Never transmit without an antenna on J4.
- **Supply.** About 119 mA while sending at 22 dBm, 6.8 mA receiving (Ebyte table 2.2). The
  onboard LDO is rated about 300 mA.
- **Emergency stop.** The HAL drives every DIO output low on an e-stop: NRST goes low (the chip
  resets and stops transmitting) and RXEN/TXEN go low. SPI itself is not gated (F1). Nothing is
  sent again until the program restarts and `Begin()` runs.

Host needs: the SPI bus (SCK, MOSI, MISO; `prosdk::platform::SetSPIPins`), one chip select and 4
or 5 GPIOs: BUSY in, DIO1 in, NRST out (optional), RXEN out, TXEN out (unless on DIO2). The
example uses SPI bus 0 on the Pulse J5 header (SCK 17, MOSI 15, MISO 16, CS 7), BUSY 38, DIO1 37,
NRST 36, RXEN 35, TXEN 39.

## 2. `prosdk::SX126x`

```cpp
prosdk::platform::SetSPIPins(0, /*sck=*/12, /*mosi=*/11, /*miso=*/13);
prosdk::SX126x radio{prosdk::SPI::kPort0, /*cs=*/10, /*busy=*/4, /*dio1=*/5, /*reset=*/6,
                     /*rxEn=*/7, /*txEn=*/15};
radio.Begin();          // false: no chip, BUSY stuck, or a calibration error
radio.StartReceive();   // continuous reception
radio.Send(bytes);      // returns at once; IsTxDone() when sent
while (auto packet = radio.Poll()) { /* packet->data, ->rssi (dBm), ->snr (dB) */ }
```

- `SX126x(SPI::Port bus, int csGpio, int busy, int dio1, int reset, int rxEn = -1,
  int txEn = -1, const Config& config = {})` opens the SPI device (mode 0, `spiClockHz`) and the
  pins, holds NRST high and the RF switch off, and starts the polling thread. It does not talk to
  the chip and does not throw for a missing chip; `prosdk::SPI` and the DIO classes throw for an
  invalid or used pin, as everywhere. `reset = -1` means NRST is not wired (no reset pulse).
- `Config` (`prosdk::SX126xConfig`; both ends of a link need the same values):

  | Field | Default | Range |
  |---|---|---|
  | `frequency` | 868.1 MHz | 150-960 MHz (chip); 850-930 MHz on the E22-900M22S |
  | `spreadingFactor` | 7 | 5-12 |
  | `bandwidth` | `k125kHz` | `k7_8kHz` ... `k500kHz` (table 13-48) |
  | `codingRate` | 5 (4/5) | 5-8 |
  | `powerDbm` | 14 | -9 to +22 (conducted) |
  | `tcxoVoltage` | `k1_8V` | `kNone` (crystal), 1.6-3.3 V (table 13-35) |
  | `syncWord` | 0x1424 (private) | 0x3444 is public/LoRaWAN (table 12-1) |
  | `preambleLength` | 8 | 1-65535 symbols |
  | `dio2AsRfSwitch` | false | see wiring |
  | `spiClockHz` | 8 MHz | chip up to 16 MHz (8.2), Ebyte 0-10 Mbit/s |

  The packet format is fixed: explicit header, payload CRC on, standard IQ. FSK, implicit header,
  CAD, sleep and duty-cycled reception are not offered.
- `bool Begin()`: resets and configures the chip; blocks about 15 ms. Returns false and prints one
  console line when a setting is out of range, BUSY stays high (100 ms), the chip does not report
  STDBY_RC, or a calibration error is reported (for example XOSC_START_ERR when the TCXO voltage
  is wrong). Calling it again starts over.
- `bool Send(std::span<const uint8_t>)`: 1 to 255 bytes. Does not wait for the air time (it may
  wait a few ms for BUSY while the chip is still starting a reception: TCXO start-up); false
  before a successful `Begin()`, while a send is running, for a bad size or a failed command. A
  send during reception interrupts it (no listen-before-talk), and reception resumes after the
  send, or at once when the send fails.
  The chip ends a send itself after twice the time on air plus 100 ms (timeout counted in
  `GetTxTimeoutCount()`).
- `bool IsTxDone()`: no send running.
- `bool StartReceive()` / `void StopReceive()`: continuous reception (`SetRx(0xFFFFFF)`) with a
  255-byte limit; stop puts the chip in standby and the RF switch off.
- `std::optional<Packet> Poll()`: the oldest received packet: `data`, `rssi` (dBm, average over
  the packet), `snr` (dB). Up to 16 packets are queued; a newer one drops the oldest
  (`GetDroppedCount()`). Packets with a payload CRC error or a header error are dropped
  (`GetCrcErrorCount()`); the chip raises RxDone for them too (14.3 note).
- `units::millisecond_t GetTimeOnAir(int payloadBytes)`: data sheet 6.1.4 for this
  configuration. Use it for duty-cycle limits.
- `void Update()`: what the thread does every 2 ms when DIO1 is high (read and clear the IRQ
  flags, finish a send, read a packet). Calling it from another thread is safe. While BUSY is
  high it returns at once and tries again on the next poll, so it never holds the lock for a
  BUSY wait (an unplugged board reads BUSY and DIO1 high through the ESP32 input pull-ups).
- `IsConnected()`: `Begin()` succeeded and BUSY has not stayed high for 100 ms since it was last
  seen low. After such a timeout every command fails at once (no new 100 ms wait) until BUSY is
  low again; reception is not restarted by itself then (call `Begin()`).
- `GetSentCount()`, `GetTxTimeoutCount()`, `GetReceivedCount()`, `GetCrcErrorCount()`,
  `GetDroppedCount()`.
- Telemetry (type "LoRa"): Connected, RSSI, SNR (last packet), Sent, Received, CRC errors.

## 3. What the driver sends

`Begin()` (data sheet 14.2, 14.4 and section 15):

1. NRST low 1 ms, high, 1 ms wait (8.1: typically 100 us; the cold start takes about 3.5 ms with
   BUSY high).
2. `SetStandby(STDBY_RC)`, `GetStatus`: chip mode must be STDBY_RC (presence check).
3. `SetDIO3AsTcxoCtrl(voltage, 5 ms)`, `ClearDeviceErrors` (the power-on XOSC_START_ERR is
   expected with a TCXO, 13.3.6), `Calibrate(0x7F)` (9.2.1: the power-on calibration fails with a
   TCXO), `CalibrateImage` for the band (table 9-2: 430-440, 470-510, 779-787, 863-870 or
   902-928 MHz; outside these the 902-928 MHz calibration stays), `GetDeviceErrors`: bits 0-6
   must be 0.
4. `SetDIO2AsRfSwitchCtrl(1)` when configured.
5. `SetPacketType(LoRa)`, `SetRfFrequency(f x 2^25 / 32 MHz)`, `SetPaConfig(0x04, 0x07, SX1262,
   0x01)`, `SetTxParams(power, 200 us ramp)`.
6. TxClampConfig (0x08D8) bits 4-1 set (15.2, better tolerance of antenna mismatch).
7. `SetBufferBaseAddress(0, 0)`, `SetModulationParams(SF, BW, CR, LDRO)` with the low data rate
   optimization on when a symbol lasts 16.38 ms or more (6.1.1.4), `SetPacketParams(preamble,
   explicit, 255, CRC on, standard IQ)`.
8. IQ polarity (0x0736) bit 2 set (15.4, standard IQ); sync word to 0x0740/0x0741.
9. `SetDioIrqParams`: TxDone, RxDone, HeaderErr, CrcErr and Timeout, all on DIO1;
   `ClearIrqStatus(all)`.

Every command first waits for BUSY low (8.3.1), and BUSY is not sampled within 1 us of the end of
the previous command (it rises up to 600 ns after NSS). The wait polls every 100 us for the first
millisecond, then every 1 ms (ESP-IDF busy-waits sleeps shorter than a tick), and gives up after
100 ms.

Send: `SetStandby`, `SetPacketParams(length)`, TxModulation (0x0889) bit 2 cleared at 500 kHz and
set otherwise (15.1 asks for it before each transmission), `WriteBuffer(0, data)`,
`ClearIrqStatus`, TXEN high and RXEN low, `SetTx(timeout)`. On TxDone or Timeout: `ClearIrqStatus`, both switch pins
low, and reception restarts if it was on.

Receive: `SetStandby`, `SetPacketParams(255)`, `ClearIrqStatus`, RXEN high and TXEN low,
`SetRx(0xFFFFFF)`. On RxDone: `ClearIrqStatus` (14.3 step 12), then without a CRC or header error
`GetRxBufferStatus`, `ReadBuffer(start, length)`, `GetPacketStatus` (RSSI = -RssiPkt/2, SNR =
SnrPkt/4).

PA: the driver always uses the +22 dBm PA setting of table 13-21 (paDutyCycle 0x04, the maximum
allowed for the SX1262, hpMax 0x07) and sets the power with `SetTxParams`. The table's "optimal"
settings for 14, 17 and 20 dBm need a matching network built for that power; the E22's matching
is not documented, so they are not used. The regulator stays in LDO mode (the reset default):
whether the E22 module has the DC-DC inductor is not documented.

Not applied: the implicit-header timeout workaround (15.3), because the driver uses neither the
implicit header nor an RX timeout.

## 4. Regulatory

The driver does not enforce any band, power or duty-cycle rule. The user is responsible for the
settings. What follows is guidance, not legal advice.

- **EU (868 MHz).** The default 868.1 MHz at 125 kHz lies in the 868.0-868.6 MHz sub-band, which
  ERC Recommendation 70-03 (annex 1) allows for non-specific short range devices at 25 mW
  e.r.p. with a 1 % duty cycle. These figures are as commonly published; they were not re-checked
  against the current edition here, and national rules apply on top.
- **Duty cycle.** 1 % means that after a packet of air time t the transmitter stays silent for
  99 t. `GetTimeOnAir()` gives t; the example waits 100 t (at least 5 s) between sends. At SF7 and
  125 kHz a 20-byte packet takes about 57 ms; at SF12 it takes about 1.3 s, so one packet per
  2 minutes.
- **Power and antenna gain.** 25 mW e.r.p. is 14 dBm relative to a half-wave dipole (2.15 dBi).
  The default 14 dBm conducted is within it only with an antenna of about 2.15 dBi or less after
  cable loss. With the Boardoza 5.8 dBi omni antenna lower `powerDbm` to about 10; with the 9 dBi
  Yagi to about 7 (14 - (gain - 2.15), cable loss ignored).
- **Turkey.** Turkey is a CEPT member; BTK's short range device rules were not checked here.
- **US (915 MHz).** The 902-928 MHz band falls under FCC Part 15. A single fixed 125 kHz channel
  at high power does not by itself meet the Part 15 rules for frequency hopping or digital
  modulation; check before transmitting there (not verified here).
- The E22's CE and FCC marks (Ebyte manual 1.1) cover the module, not a robot built with it.

## 5. Verified and unverified

Verified against the Semtech data sheet rev. 1.2: every opcode and frame layout used (section 13
tables), the register addresses and reset values (table 12-1), the IRQ bits (table 13-29), the
status byte (table 13-76), the device error bits (table 13-85), the TCXO voltage codes (table
13-35), the image calibration bands (table 9-2), the bandwidth, spreading factor and coding rate
codes (tables 13-47 to 13-49), the PA limits (13.1.14), the TX power range (13.4.4), the timeout
step and limits (13.1.4, 13.1.5), the time-on-air formula (6.1.4) and LDRO threshold (6.1.1.4),
the reset pulse (8.1), SPI mode and clock limit (8.2), BUSY timing (8.3.1, table 8-2), the
command order (14.2-14.4) and the known limitations (15.1, 15.2, 15.4).

Verified against the Ebyte manual: SX1262 inside, TCXO supplied from DIO3 (4.1, 4.2), RXEN and
TXEN high-active and the TXEN-to-DIO2 option (pin table), SPI 0-10 Mbit/s, supply limits, current
figures.

Unverified (needs hardware or a document that states it):
- The TCXO voltage of the E22-900M22S (1.8 V from the Boardoza example; the Ebyte manual gives
  none). If it is wrong, `Begin()` should fail with XOSC_START_ERR (device error bit 5), which is
  how a hardware test can confirm it.
- The TCXO start-up delay (5 ms chosen; Ebyte gives none; the Boardoza example uses 10 ms). Too
  short shows as XOSC_START_ERR in `Begin()`. Every switch from standby to TX or RX waits this
  delay with BUSY high.
- Whether the module has the DC-DC inductor (LDO mode used).
- The PA output power and harmonics with the E22's matching network at the +22 dBm PA setting
  below 22 dBm.
- Image rejection outside the table 9-2 bands (850-863 and 870-902 MHz on this module).
- RF switch timing: the driver switches RXEN/TXEN just before `SetTx`/`SetRx` and after
  TxDone; the switch's settling time is not documented.
- Range, sensitivity and packet loss on real hardware; interoperability with other LoRa radios
  using the same settings.
- BUSY polling cost on the robot: the first millisecond of a wait is polled every 100 us, which
  ESP-IDF may busy-wait; longer waits sleep 1 ms per poll.
- RFM68LCW compatibility (see the top of this file).

The code builds for the host and the ESP32-S3, and the `BoardozaLoRa` example builds with arduino-cli
(1,920,648 B flash, 71,872 B static RAM). The host tests ran on 2026-10-22 (`bd0d149`, `tools/build/build.sh host all header_check --test`): `[sx126x]` passes 82
assertions in 5 test cases. Nothing has run on hardware.

## 6. Resource use

- One `prosdk::SPI` device (one chip select on the bus; it can share a bus with the flash and SD
  devices, not with the clocked LEDs), 2 DIO inputs and up to 3 DIO outputs.
- One thread (`sx126x`, `platform::ScopedThreadConfig` defaults) that reads DIO1 every 2 ms and
  uses the SPI bus only when DIO1 is high.
- Memory: up to 16 queued packets of at most 255 bytes (about 4.5 KiB worst case) plus the class
  (under 0.5 KiB).
- No RMT, LEDC, PCNT, ADC or timer.
- Example `BoardozaLoRa`: SPI2 on GPIO 7 and 15-17 (Pulse J5), GPIO 35-39.

## 7. Tests

`SX126xTest.cpp` (`[sx126x]`), with a model of the chip on `FakeSPIDevice` (status, IRQ register
behind DIO1, device errors, registers, data buffer) and `DIOSim` for BUSY, DIO1 and the outputs:

- `Begin` sends the full configuration in the data sheet order, with the exact frames (TCXO,
  calibration, image calibration, frequency bytes for 868 MHz, PA, register workarounds, sync
  word, IRQ masks); NRST ends high and the RF switch off.
- `Send`: the frames (standby, packet length, TxModulation, buffer, IRQ clear, SetTx with the
  timeout from the time on air), TXEN high, one send at a time, TxDone handled by the thread (IRQ read and
  cleared, TXEN low, count), and a chip timeout.
- Reception: the RX frames and RXEN, a packet with RSSI and SNR, a CRC error dropped, a send
  during reception and reception resuming, `StopReceive`.
- BUSY: a command waits until BUSY goes low; BUSY stuck high sends nothing and fails; an
  unplugged board after `Begin()` (BUSY and DIO1 high): `IsConnected()` turns false, `Poll()` and
  `Send()` return at once, and plugging back in recovers; a bus without a chip; a calibration
  error (XOSC_START_ERR).
- Time on air for SF7/125 kHz, SF12/125 kHz (LDRO on, checked in `SetModulationParams`) and
  SF6/500 kHz (TxModulation bit 2 cleared on a send); an out-of-range power.

## 8. Shared edits (integration)

- design/drivers.md: a link to this file in the Boardoza section; the "Tests" list gains SX126x.
- design/API-DIFFERENCES.md section 7: one row, `prosdk::SX126x` (hand-written, no WPILib
  counterpart; `(SPI::Port, cs, busy, dio1, reset, rxEn, txEn, Config)`; LoRa only; a thread
  polls DIO1).
- tools/upstream/examples.json ("handwritten") and design/examples.md: `BoardozaLoRa`, with the
  regulatory note (868 MHz, 1 % duty cycle, 14 dBm default) in its description.
- design/boardoza.md (board index): E22-900M22S -> `SX126x`; RFM68LCW -> doc only until a
  hardware test.
- The threads budget (plan section 7): one more thread.
