# P7: GNSS receivers

Package P7 of the Boardoza plan. It adds two receiver classes, a fix struct shared by both, and
two protocol utilities with no HAL dependency (plan section G5):

- `prosdk::UbloxGnss`: u-blox M8 and M10 receivers over a UART or I2C, reading UBX NAV-PVT;
- `prosdk::NmeaGnss`: any receiver that streams NMEA 0183 over a UART;
- `prosdk::GnssFix`: the solution both classes return;
- `prosdk::nmea` (`Nmea.hpp`) and `prosdk::ubx` (`Ubx.hpp`): sentence and frame parsers and
  encoders, also usable on their own (for example by a data logger or a future receiver).

| Board | Ozdisan id | Receiver | Class | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| SAM-M8Q GNSS module | 1206512 | u-blox SAM-M8Q (u-blox M8) | `UbloxGnss` | Ublox_Sam_M8Q_GNSS_Module_Breakout_Board |
| SAM-M10Q GNSS | none (repo only) | u-blox SAM-M10Q (M10, ROM SPG 5.10) | `UbloxGnss` | SAM-M10Q_Multi_GNSS_Positioning_Breakout_Board |
| GP-02 GNSS | 1206509 | Ai-Thinker GP-02 (chip not named) | `NmeaGnss` | GP-02_GNSS_Satellite_Navigation_Breakout_Board |

Files:
- `src/prosdk/drivers/gnss/GnssFix.hpp`
- `src/prosdk/drivers/gnss/Nmea.hpp` and `.cpp`, `Ubx.hpp` and `.cpp`
- `src/prosdk/drivers/gnss/UbloxGnss.hpp` and `.cpp`, `NmeaGnss.hpp` and `.cpp`
- Tests (host only): `test/prosdk/drivers/NmeaTest.cpp`, `UbxTest.cpp`, `UbloxGnssTest.cpp`,
  `NmeaGnssTest.cpp`
- Example: `examples/BoardozaGnss/BoardozaGnss.ino`

Sources (read in full for the facts used; copies kept outside the repo):
- u-blox M10 SPG 5.10 Interface Description, UBX-21035062 R03 ("M10" below): UBX frame and
  checksum (sections 3.2, 3.4), CFG-VALSET (3.10.5), MON-VER (3.14.9), NAV-PVT (3.15.11),
  configuration items and keys (4.2, 4.5, 4.9), NMEA GGA/GSA/RMC (2.7.5, 2.7.13, 2.7.17) and the
  NMEA fix flags (2.5.5).
- u-blox 8 / M8 Receiver Description incl. Protocol Specification, UBX-13003221 R28 ("M8"):
  DDC (I2C) interface (section 11.5), port numbers (11), CFG-MSG (32.10.18), CFG-PRT
  (32.10.25), CFG-RATE (32.10.27), NAV-PVT (32.17.17), protocol versions (2), default port
  settings (appendix C.15), NMEA checksum (31.1.1).
- SAM-M8Q data sheet (in the board repo), SAM-M10Q data sheet UBX-22013293 R05, Ai-Thinker GP-02
  data sheet, and the three board READMEs and schematics.

No vendor code was used. The u-blox library in the board repos (a rebranded SparkFun library) was
read only to cross-check message ids.

## 1. Wiring and hazards

**Power every board from 3.3 V when it is wired to the ESP32-S3.** The ESP32-S3 pins are not
5 V tolerant. All three boards have a TLV73333 3.3 V regulator and accept 3.3 to 5.5 V on VCC,
but their level shifters (2N7002) pull the host-side lines up to VCC:

- SAM-M8Q: I2C (J2) goes through the shifter with 10k pull-ups to VCC, so at VCC = 5 V SDA/SCL
  idle at 5 V. Whether the UART (J1) is shifted could not be read from the schematic net names;
  treat TXD/RXD as 3.3 V logic (**unverified**).
- SAM-M10Q: J1 (UART) and J2 (I2C) are shifted ("5 V tolerant" in the README), with the same
  pull-up-to-VCC consequence. J3 (PPS, RST, INT, SAFE) is 3.3 V only.
- GP-02: J1 (UART) goes through two 2N7002. With VCC = 5 V the module's TX line may idle at 5 V
  on the host side (**unverified** from the nets). J3 and J4 are 3.3 V.

With 3.3 V on VCC the regulator drops out and the module gets slightly less than 3.3 V, which is
inside every module's 2.7 to 3.6 V range. Tie the board GND to the controller GND.

| Board | Connector | Pins |
|---|---|---|
| SAM-M8Q | J1 UART | 1 GND, 2 TXD (to host RX), 3 RXD (from host TX), 4 VCC |
| SAM-M8Q | J2 I2C | 1 GND, 2 SDA, 3 SCL, 4 VCC |
| SAM-M8Q | J3 (3.3 V) | PPS, RST, INT, SAFE (leave SAFE open) |
| SAM-M10Q | J1 UART | 1 VCC, 2 RXD (from host TX), 3 TXD (to host RX), 4 GND (direction naming **unverified**: check the board labels) |
| SAM-M10Q | J2 I2C | 1 VCC, 2 SCL, 3 SDA, 4 GND |
| SAM-M10Q | J3 (3.3 V) | 1 PPS, 2 RST, 3 INT, 4 SAFE (leave SAFE open) |
| GP-02 | J1 UART | 1 VCC, 2 RX (from host TX), 3 TX (to host RX), 4 GND |
| GP-02 | J2 | U.FL for an active L1 antenna: connect it before power-up (README) |
| GP-02 | J3 (3.3 V) | 1 PPS, 2 N/F (low = sleep; leave open), 3 RST (leave open) |
| GP-02 | J4 | jumper: open or 1-2 = BDS + GPS, 2-3 = GPS + GLONASS (read at boot) |

Note that the two u-blox boards order their I2C pins differently.

Use one link per u-blox board: I2C or UART, not both. The driver silences NMEA only on the port it
uses.

**I2C clock.** The SAM-M10Q allows at most 320 kHz and may stretch the clock for up to 20 ms
(SAM-M10Q data sheet section 5.2 and table 17). The SAM-M8Q allows up to 400 kHz. I2C port 0
defaults to 400 kHz (and may be shared with `Wire`), so put a SAM-M10Q on a port set to 320 kHz
or less, for example `prosdk::platform::SetI2CPins(1, 17, 18, 100000)` as in the example. Table 17
gives fSCL 0 to 320 kHz, but section 5.2 also says Standard-mode operation is not supported.
100 kHz with Fast-mode timing is expected to work, and common libraries use it, but this is
**unverified** on hardware. The PPS, RST and INT pins are not used by the drivers.

## 2. `prosdk::GnssFix`

```cpp
enum class GnssFixType : uint8_t { kNoFix, kDeadReckoning, k2D, k3D, kGnssDeadReckoning, kTimeOnly };
struct GnssTime { int year, month, day, hour, minute; double second; bool dateValid, timeValid; };
struct GnssFix {
  bool valid;                         // usable position
  GnssFixType fixType;
  units::degree_t latitude, longitude;  // north and east positive
  units::meter_t heightMsl;
  units::meters_per_second_t groundSpeed;
  units::degree_t heading;            // of motion, clockwise from true north
  int numSatellites;
  units::meter_t hAcc;                // NaN when the receiver does not report it (NMEA)
  double hdop;                        // NaN when the receiver does not report it (UBX)
  GnssTime utc;
  units::second_t timestamp;          // monotonic time the solution arrived
};
```

`fixType` uses the NAV-PVT numbering (M10 section 3.15.11).

## 3. `prosdk::UbloxGnss`

```cpp
UbloxGnss(SerialPort::Port port, int baudRate = 9600, millisecond_t updatePeriod = 20_ms);
UbloxGnss(I2C::Port port, int address = 0x42, millisecond_t updatePeriod = 20_ms);
GnssFix GetFix() const;
void SetRate(hertz_t rate);      // navigation rate, 50 ms to 65.535 s period
bool IsConnected() const;        // any UBX frame within the timeout
bool IsConfigured() const;       // NAV-PVT configuration acknowledged
void Update();                   // the thread's work (call it yourself with updatePeriod 0)
void Feed(std::span<const uint8_t> bytes);   // test hook: parse bytes
uint32_t GetChecksumErrors() const;
// TelemetryLoggable, type "GNSS": Valid, Fix Type, Latitude, Longitude, Height MSL,
// Ground Speed, Heading, Satellites, Horizontal Accuracy
```

Construction never blocks. A thread (name `ublox-gnss`, one per instance) runs `Update()` every
`updatePeriod`: it reads what the receiver sent, parses it, and advances this sequence:

1. **Probe**, once a second until the receiver answers UBX-MON-VER: send UBX-CFG-PRT for the port
   in use (UART1 with the given baud rate and 8N1, or DDC with the given address; input UBX +
   NMEA + RTCM, output UBX only) and a MON-VER poll. CFG-PRT matters on a u-blox 8: the SAM-M8Q
   data sheet (table 9) lists NMEA as the only default DDC output, while M8 appendix C.15.4 lists
   UBX + NMEA; the probe works either way. The M10 message list has no CFG-PRT (it maps to
   configuration keys, M10 section 4.10), so an M10 is expected to answer ACK-NAK
   (**unverified**); any answer to the probe's CFG-PRT is ignored. After five unanswered probes
   the driver prints one console warning.
2. **Configure.** MON-VER carries "PROTVER=xx.yy" (or "PROTVER xx.yy" on old M8 firmware).
   - Protocol below 24 (u-blox 8 is 15 to 23.01, M8 section 2): UBX-CFG-MSG NAV-PVT rate 1 on
     the current port, and UBX-CFG-RATE (measurement period, navRate 1, timeRef GPS).
   - Protocol 24 or later (M10 is 34.10): one UBX-CFG-VALSET to the RAM layer with
     CFG-<port>OUTPROT-NMEA = 0, CFG-MSGOUT-UBX_NAV_PVT_<port> = 1 and CFG-RATE-MEAS
     (keys from M10 section 4.9).
   Every message must be answered with ACK-ACK. No answer within a second resends; three failed
   attempts go back to the probe. An ACK-NAK (for example a rate the receiver cannot do) prints
   one console warning.
3. **Running**: each NAV-PVT updates the fix. If no NAV-PVT arrives within the timeout
   (the larger of 2 s and three navigation periods), the driver probes and configures again.

The configuration goes to RAM only. The boards have no backup battery, so a receiver power cycle
loses it, and step 3 restores it. Nothing is saved to flash, and the UART baud rate is never
changed.

NAV-PVT to `GnssFix`: valid = gnssFixOK (flags bit 0) and fixType 2, 3 or 4; latitude and
longitude 1e-7 deg; hMSL, hAcc and gSpeed mm and mm/s; headMot 1e-5 deg; second = sec + nano
(nano may be negative); dateValid and timeValid are valid bits 0 and 1.

`SetRate()` takes effect at once if the receiver is configured. The receivers' own limits apply:
a u-blox M8 does up to 10 Hz with GPS + GLONASS (SAM-M8Q data sheet table 1); the SAM-M10Q does
4 Hz with its default four constellations (data sheet). At 9600 baud one NAV-PVT frame
(100 bytes) takes about 104 ms on the wire, so stay at 5 Hz or below on a UART at the default
baud rate.

**I2C reads** (M8 section 11.5): write register 0xFD, read 2 bytes (0xFD high byte, 0xFE low
byte) for the number of bytes waiting, then read register 0xFF in chunks of 128 bytes, at most
2048 bytes per update. A count of 0xFFFF (an idle 0xFF fill) is treated as 0. UBX messages are
written as raw frames to the address. The M10 Interface Description does not repeat the DDC
register model; the driver assumes it is the same (**unverified** on hardware).

## 4. `prosdk::NmeaGnss`

```cpp
NmeaGnss(SerialPort::Port port, int baudRate = 9600, millisecond_t updatePeriod = 20_ms);
GnssFix GetFix() const;
bool IsConnected() const;        // a good sentence within 2 s
void Feed(std::span<const uint8_t> bytes);   // test hook / other transports
uint32_t GetSentenceCount() const;
uint32_t GetChecksumErrors() const;
// TelemetryLoggable, type "GNSS": Valid, Fix Type, Latitude, Longitude, Height MSL,
// Ground Speed, Heading, Satellites, HDOP
```

Listen-only: nothing is sent to the receiver, so it works with any NMEA receiver, including a
u-blox left at its defaults. A thread (`nmea-gnss`) reads the UART every `updatePeriod` and prints
one console warning if no valid sentence arrived within 5 s of the start (wrong pins or baud
rate). `updatePeriod` 0 opens no port and starts no thread (the `ProHubEncoders` pattern), so
the only input is `Feed()`.

- RMC: status A = valid; UTC time and date; position (only while valid); speed (knots to m/s);
  course as heading.
- GGA: quality > 0 = valid, quality 6 = dead reckoning; satellites; HDOP; position and altitude
  above mean sea level (only while valid).
- GSA: navMode 2 or 3 sets k2D or k3D.
- Any talker id is accepted (GP, GN, BD, GL, GA ...). Other sentences (GSV, VTG, TXT, $P...)
  count as received but are ignored.
- `fixType` is kNoFix when not valid, kDeadReckoning after a GGA with quality 6, otherwise k3D
  when the latest GSA said 3D and k2D otherwise. A receiver that sends no GSA reports k2D for
  every valid fix.
- `hAcc` is NaN (NMEA has no accuracy estimate).

## 5. Protocol utilities

`prosdk::nmea` (`Nmea.hpp`):
- `Checksum(body)`, `Encode(body)` (adds `$`, `*hh` and CR LF), `SentenceType(body)` (the three
  letters after the talker id; empty for proprietary `$P...` sentences);
- `ParseGga`, `ParseRmc`, `ParseGsa`: return `std::optional` structs; empty fields are NaN;
- `Reader`: byte-at-a-time sentence assembler. A sentence starts at `$` and ends at CR or LF and
  must end in `*hh` with the right XOR checksum (upper- or lower-case hex). A control or non-ASCII
  byte, a line over 120 characters, or a `$` before the end drops the sentence and counts an error.

`prosdk::ubx` (`Ubx.hpp`):
- `Checksum`, `Encode(class, id, payload)`;
- `KeyValueSize(key)` (key bits 30..28), `EncodeValset(layers, items)`;
- `EncodeCfgPrtDdc(address)`, `EncodeCfgPrtUart1(baud)`, `EncodeCfgMsg`, `EncodeCfgRate`;
- `NavPvt`, `DecodeNavPvt`, `EncodeNavPvt` (for tests and simulators);
- `ParseProtocolVersion(monVerPayload)`;
- `Parser`: byte-at-a-time frame parser. It skips NMEA text and 0xFF fill between frames, drops a
  frame with a bad checksum, and drops a header that announces more than 512 payload bytes at
  once, so a corrupted length cannot swallow the stream.

## 6. Defaults and what is verified

| Fact | Value | Status |
|---|---|---|
| u-blox UART default | 9600 8N1, NMEA output | verified: SAM-M8Q data sheet table 9, SAM-M10Q data sheet |
| u-blox I2C address | 0x42 (7-bit) | verified: M8 appendix C.15.4; M10 CFG-I2C-ADDRESS default 0x84 (8-bit form) |
| SAM-M8Q DDC default output | UBX + NMEA (M8 C.15.4) or NMEA only (data sheet table 9) | sources disagree; the probe CFG-PRT covers both |
| DDC registers 0xFD/0xFE/0xFF | as above | verified for M8 (section 11.5); assumed for M10 |
| UBX frame, Fletcher checksum, NAV-PVT offsets | as in `Ubx.hpp` | verified: M10 3.2, 3.4, 3.15.11; M8 32.17.17 |
| CFG-PRT, CFG-MSG, CFG-RATE layouts and bit fields | as in `Ubx.cpp` | verified from the M8 figures |
| CFG-VALSET layout and key value sizes | as in `Ubx.cpp` | verified: M10 3.10.5, 4.2, 4.5 |
| Key ids (RATE-MEAS, OUTPROT-NMEA, MSGOUT NAV_PVT) | as in `Ubx.hpp` | verified: M10 4.9 |
| Protocol versions | M8 15 to 23.01, M10 SPG 5.10 34.10; VALSET from 24 | verified: M8 section 2, M10 section 1.2 (MON-VER string "PROTVER=34.00" in its example) |
| GP-02 baud rate | 9600 8N1 | **unverified**: the data sheet only says "max support 256000bps"; 9600 is the board example's value |
| GP-02 update rate | 1 Hz (max 5 Hz) | data sheet; the driver does not change it |
| GP-02 configuration commands ($PCAS) | not used | **unverified** (not in the data sheet) |
| NMEA field layouts | GGA, RMC, GSA | verified: M10 2.7.5, 2.7.17, 2.7.13 |
| NMEA two-digit year | read as 20yy | convention, not in a data sheet |

Two example sentences in the M10 document carry wrong checksums (the RMC example in 2.7.17 and the
GSA example in 2.7.13); the tests use the GGA example of 2.7.5 and the RMC example of 2.5.3, whose
checksums are right.

Only on hardware (not checked here): a real fix on each board; the M10 NAK of CFG-PRT; M10 clock
stretching (up to 20 ms) against the 50 ms ESP32 I2C timeout; 100 kHz on the M10; the level
shifter behaviour at 5 V; the SAM-M10Q UART pin naming.

## 7. Resource use

| Class | Bus | Thread | Memory (approx.) | Bus load at defaults |
|---|---|---|---|---|
| `UbloxGnss` (I2C) | shares an I2C port; one address | 1 (`ublox-gnss`, 20 ms) | about 1 KiB (parser 512 B, read chunk, fix) | a 2-byte count read every 20 ms; about 100 bytes of NAV-PVT per navigation period |
| `UbloxGnss` (UART) | one UART (1 KiB RX buffer) | 1 (`ublox-gnss`, 20 ms) | about 1 KiB | 100 bytes per navigation period (10% of 9600 baud at 1 Hz) |
| `NmeaGnss` | one UART (1 KiB RX buffer) | 1 (`nmea-gnss`, 20 ms) | about 0.5 KiB | the receiver's NMEA stream (about 500 bytes per second at 1 Hz) |

The 20 ms period keeps the 1 KiB UART buffer far from full (it holds about 1 s at 9600 baud). The
thread stack and priority come from `platform::ScopedThreadConfig` defaults; the budget with many
threaded drivers at once must be checked on hardware (plan, threads risk). No PWM, RMT, PCNT,
LEDC or ADC resources are used.

## 8. Tests

- `NmeaTest.cpp`: checksums against the u-blox examples, GGA/RMC/GSA decoding (hemispheres,
  empty fields, NMEA 2.1 RMC without a mode field), and the reader split at every byte position
  with garbage, a bad checksum and a sentence without a checksum.
- `UbxTest.cpp`: frames against known byte sequences (MON-VER poll, CFG-RATE 1 Hz, CFG-MSG
  NAV-PVT), key value sizes, the CFG-VALSET and CFG-PRT byte layouts, NAV-PVT round trip, the
  parser split at every byte position with 0xFF fill, NMEA text, a bad checksum and an oversize
  length, and MON-VER protocol versions.
- `UbloxGnssTest.cpp`: a fake u-blox on the I2C sim (`FakeI2CBus`, DDC register model) that
  answers MON-VER and acknowledges configuration: the M10 VALSET path, the M8 CFG-MSG/CFG-RATE
  path, `SetRate`, NAK retries, the NAV-PVT timeout and re-probe, a missing receiver, and serial
  `Feed` parsing.
- `NmeaGnssTest.cpp`: the fix from RMC, GGA and GSA split into 17-byte reads, 2D, dead reckoning,
  fix loss, a bad checksum and the connection timeout.
