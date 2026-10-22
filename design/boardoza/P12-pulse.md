# P12: Pulse-protocol devices

Package P12 of the Boardoza plan. It adds three classes on the F3 pulse I/O
(`design/boardoza/F3-timed-io.md`): an IR transceiver, a 433 MHz OOK code receiver and a PIR
motion sensor configured over a one-wire serial input.

| Board | Ozdisan id | Chip | Class | Repo (`Boardoza_` prefix omitted) |
|---|---|---|---|---|
| IrDA Transceiver (a 38 kHz consumer-IR link, not IrDA SIR) | 1595255 | Everlight IRM-H638T receiver, Vishay VSMB10940 IR LED | `prosdk::IRTransceiver` | IrDA_Transceiver_Infrared_Wireless_Communication_Breakout_Board |
| CMT2210LH RF Receiver | 1564803 | CMOSTEK CMT2210LH, 433.92 MHz OOK | `prosdk::OokReceiver` | CMT2210LH_RF_Receiver_Breakout_Board |
| PYD1598 | 1473943 | Excelitas PYD 1598 DigiPyro | `prosdk::PYD1598` | none (Ozdisan only) |

Files:
- `src/prosdk/drivers/radio/IRTransceiver.hpp` and `.cpp`, `OokReceiver.hpp` and `.cpp`
- `src/prosdk/drivers/sensor/PYD1598.hpp` and `.cpp`
- Tests (host only): `test/prosdk/drivers/IRTransceiverTest.cpp`, `OokReceiverTest.cpp`,
  `PYD1598Test.cpp`
- No example (plan).

Sources (copies kept outside the repo, in the plan's `boardoza-dl/` folder):
- Everlight IRM-H6XXT/TR2 data sheet DMO-0000008 Rev. 3 and Vishay VSMB10940 data sheet (from
  the board repo); the board README, schematic and example sketches.
- CMOSTEK CMT2210/17LH data sheet v0.5 and the board schematic.
- EV1527 data sheet (Shenzhen Silvan, Chinese, section 2.3 and the oscillator table).
- Excelitas "PYD 1588 / 7656 low power DigiPyro" product specification (issued 20/03/2018,
  revised 06/04/2021): Table 4, sections 2.4 to 2.7. Excelitas' "Dual-Element, Low-Power
  DigiPyro" brochure page (PYD 1588 / PYD 1598 / PYD 2592). The PYD 1598 product specification
  itself could not be downloaded: excelitas.com answers 403 to every request for it (the
  DigiKey and search-result links were tried).

No vendor code was used. NOTICE and `licenses/` do not change.

## Wiring

All three boards connect with plain GPIOs; any free GPIO works.

- **IR transceiver.** J1: VCC, RXD, TXD, GND. TXD to the transmit GPIO, RXD to the receive GPIO.
  **Power the board from 3.3 V**: RXD swings to VCC, so at VCC = 5 V it outputs about 5 V, which
  an ESP32-S3 pin does not tolerate. TXD high turns the LED on through a 33 ohm resistor only
  (about 59 mA at 3.3 V, about 110 mA at 5 V; the VSMB10940 is rated 65 mA continuous and
  130 mA peak at 50 % duty and 100 us), so TXD must only carry carrier bursts. The class never
  holds TXD high: the transmitter idles low and sends every high level as 38 kHz bursts at 33 %
  duty. That TXD floating or low keeps the LED off was inferred from the schematic's component
  list, not traced net by net (unverified).
- **CMT2210LH.** J2: NC, VDD, DATA, GND; J1: antenna (a 17 cm wire for 433.92 MHz) and GND.
  DATA to the receive GPIO. **3.3 V only**: the board is wired in the chip's low-voltage mode,
  absolute maximum 3.6 V on VDD. DATA is a push-pull output at VDD and needs no pull resistor.
  DATA is also the chip's factory programming pin; leave it an input.
- **PYD1598.** The chip has four pins: VDD, VSS, SERIN and DIRECT LINK (DL). SERIN to the
  `PulseTransmitter` GPIO, DL to a DIO GPIO. **The chip is 1.8-3.6 V with 3.6 V absolute
  maximum on every pin**: SERIN and DL connect directly only with the sensor at 3.3 V. The
  Boardoza board has no public repo or schematic, so its pin order, any regulator and any
  components on DL are unknown (unverified); check the board before powering it.
  - **DL and the ESP32 pull-up.** prosdk DIO inputs always have the internal pull-up
    (`design/hal-esp32.md`, DIO), while the data sheet says pull resistors on DL are not
    applicable (Table 4, note 2) and asks for a high-impedance input. Whether the sensor holds
    DL low firmly enough between events is unverified. The driver checks DL right after every
    clear (motion is blocked for the blind time then, so DL must read low) and prints one
    console warning if it reads high. If that happens on hardware, the fix is a DIO input
    without pull-up, which is a HAL change (not part of this package).

## Class API

### `prosdk::IRTransceiver`

```cpp
enum class IrProtocol { kNec, kNecExtended };
struct IrFrame { IrProtocol protocol; uint16_t address; uint8_t command; bool repeat; };

IRTransceiver(int txChannel, int rxChannel);   // -1: that side is not used
bool SendNec(uint8_t address, uint8_t command);
bool SendNecRepeat();
bool SendRaw(std::span<const Pulse> pulses);    // high = carrier burst, low = silence
std::optional<IrFrame> Poll();
std::vector<Pulse> GetLastPulses() const;       // last received burst, carrier levels
```

- Carrier 38 kHz at 33 % duty (`kCarrier`, `kCarrierDuty`), fixed.
- NEC: 9 ms leader burst, 4.5 ms space, 32 bits LSB first (address, inverted address, command,
  inverted command), each a 562 us burst and a 562 us (0) or 1687 us (1) space, then a 562 us
  stop burst; a frame takes about 68 ms. Repeat code: 9 ms burst, 2.25 ms space, 562 us burst.
  A remote sends one every 108 ms (`kNecRepeatPeriod`) while the key is held; `SendNecRepeat()`
  sends one, and timing them is the caller's job.
- Decoding: a burst ends when the receiver output stays at one level for 10 ms (`kIdleTime`).
  Bursts of exactly 67 levels with valid NEC timing and a matching command inverse become frames;
  when the address inverse does not match, the frame is extended NEC with a 16-bit address. Every
  level is accepted with about +-250 us of distortion (the IRM-H638T's output pulse width is
  400-800 us for a 600 us burst). A repeat code is returned only directly after a frame or repeat
  code of the same run, with that frame's address and command and `repeat = true`.
- `Poll()` returns one frame per call; up to 16 are queued. `GetLastPulses()` returns the last
  complete burst as carrier levels (true = carrier received), decoded or not, so other remote
  formats can be inspected or replayed with `SendRaw()`.
- The receiver sees the board's own LED, so frames sent are normally received back (unverified
  on the board). The class does not filter them; compare with what was sent if needed.
- Errors: a pin in use or no free RMT channel throws (from `PulseTransmitter`/`PulseCapture`).
  Send methods return false without a transmit pin (console line) or after a transmitter error.
- Telemetry type "IR Transceiver": Address, Command (last frame), Frames (count).

### `prosdk::OokReceiver`

```cpp
struct OokCode { uint32_t value; int bits; microsecond_t pulseLength; bool repeat; };

explicit OokReceiver(int channel);
std::optional<OokCode> Poll();
std::vector<Pulse> GetRecentPulses() const;   // last 256 pulses read, oldest first
int GetChannel() const;
```

- Frame format (EV1527, PT2262 and compatible encoders): a sync (one short high, then a long
  low) and N bits, each a high and a low: 1:3 short pulses for 0, 3:1 for 1. EV1527 sends the
  sync before each 24-bit word (20-bit ID, then D0-D3); PT2262 sends 12 tri-state digits (two
  such bits each: 00 = '0', 11 = '1', 01 = 'F') and the sync after them. `value` holds the bits
  with the first received one most significant.
- Decoder: a low of at least 1.5 ms and at least 8 times the high before it is a sync gap. The
  levels between two gaps must be 8 to 32 bits; the short pulse length is measured from the frame
  (each bit is 4 short pulses) and must be 50 to 1000 us; every bit must last 2.5 to 5.5 short
  pulses with long/short at least 1.6; the sync high at most 2.5 short pulses and both gaps at
  least 8. A level of 32.767 ms or longer (the capture's idle time) ends a frame like a gap when
  it is low.
- Acceptance: the receiver outputs noise when no remote transmits, so a code is returned only
  when two identical frames arrive in a row (`repeat = false`), then once per further identical
  frame (`repeat = true`) while the key is held. Any other frame or noise in between starts over.
  The last frame of a transmission counts only when a sync follows it (PT2262 always, EV1527
  not), so a short press gives at least one code when the remote sends 3 or more frames.
- `Poll()` returns one code per call; up to 16 are queued. Call it every loop: the capture holds
  256 pulses, about 5 frames of 24 bits (about 0.25 s at a 350 us short pulse).
- No authentication: anyone with a matching remote can replay a code. Never use one as an enable
  or an emergency stop.
- Telemetry type "OOK Receiver": Code, Bits (last code), Codes (count).

### `prosdk::PYD1598`

```cpp
struct PYD1598::Config {
  int threshold = 32;                  // 0-255
  second_t blindTime{2.0};             // 0.5-8 s, 0.5 s steps
  int pulseCount = 2;                  // 1-4
  second_t windowTime{4.0};            // 2, 4, 6, 8 s
  bool lowHighPassCutoff = false;      // false 0.4 Hz, true 0.2 Hz
  bool countWithoutSignChange = false;
};

PYD1598(int serinChannel, int dlChannel);
PYD1598(int serinChannel, int dlChannel, const Config& config);
bool Configure();                       // write the word again
bool Configure(const Config& config);
const Config& GetConfig() const;
bool Poll();                            // DL high: a motion event is pending
void Clear();                           // DL low for 200 us, starts the blind time
static uint32_t EncodeConfig(const Config& config);
```

- Wake-up mode only. The sensor's motion detection unit counts band-pass filtered signal
  crossings of the threshold; when `pulseCount` crossings fall within `windowTime` it drives DL
  high and keeps it high. `Clear()` pulls DL low (at least 160 us by the data sheet; 200 us
  here), and no motion is detected for `blindTime` after that. The forced and interrupt
  readouts of the raw 14-bit ADC values are out of scope.
- The defaults are a starting point, not data sheet values; the threshold's scale against the
  ADC counts is not given in the data sheet, so tune it on the robot. Out-of-range settings throw
  `ParameterOutOfRange` (times are rounded to the nearest step).
- Configuration word (data sheet Table 5), 25 bits, bit 24 first: [24:17] threshold, [16:13]
  blind time (0.5 s + n * 0.5 s), [12:11] pulse counter (1 + n), [10:9] window time
  (2 s + n * 2 s), [8:7] operation mode = 2 (wake up), [6:5] signal source = 0 (PIR band-pass),
  [4:3] reserved = 2, [2] HPF cut-off, [1] reserved = 0, [0] count mode. The defaults give
  0x406B10.
- SERIN waveform (section 2.6, Table 4) on a `PulseTransmitter` without a carrier: 700 us low
  (a new word is accepted only after SERIN was low for more than tSLT = 650 us), then per bit
  1 us low (tSL), 1 us high (tSH, both 0.2-2 us) and the data level for 100 us (tSHD 80-150 us;
  the word must not pause for more than 150 us), then 700 us low to latch it. About 3.95 ms.
  Interrupt-masked bit-banging (F3 `WriteWaveform`, 20 us at most) cannot send it.
- DL is held low during the whole word, as the data sheet requires. A GPIO cannot be a
  `DigitalInput` and a `DigitalOutput` at once, so `Configure()` and `Clear()` free the DL
  input, drive DL low through a temporary `DigitalOutput`, then allocate the input again.
  `Configure()` blocks for about 5 ms, `Clear()` for 200 us to one FreeRTOS tick.
- There is no read back (that needs the forced readout), so `Configure()` returning true only
  means the word was sent; a missing sensor is not detected.
- The sensor's configuration is volatile: call `Configure()` again if its supply was cycled.
- Telemetry type "Motion Sensor": Motion (last Poll), Configured.

## Emergency stop and gating

- IR TX follows F3's pulse TX rule: the line is a HAL output, the e-stop drives it low and no
  train starts after it; it is not gated while the robot is merely disabled (an IR command is not
  an actuator output).
- PYD1598: SERIN is a pulse TX pin (same rule); after an e-stop `Configure()` fails. DL is driven
  low only briefly by `Clear()`/`Configure()`; after an e-stop the HAL refuses the temporary
  DIO output (it reports the error), DL is not driven and a pending event stays reported.
- Receivers are inputs and are not affected.

## Verified and unverified

Verified (from documents):
- IRM-H638T: 38 kHz, output low while the carrier is present, output pulse width 400-800 us for
  a 600 us burst, minimum burst 12 and gap 16 carrier cycles (DMO-0000008 Rev. 3). VSMB10940
  ratings (Vishay data sheet). Board supply and RXD level from the schematic and README.
- CMT2210LH: DATA is the demodulated OOK output, no registers or configuration interface, board
  in low-voltage mode at 3.3 V (data sheet v0.5, schematic).
- EV1527: data bit shapes (12 LCK + 4 LCK for 1, 4 + 12 for 0), 1 LCK = 8 oscillator clocks,
  16 LCK = 230 us to 3.32 ms (oscillator table, 47-680 kOhm at 3-13 V: a short pulse of 58 to
  830 us, inside the decoder's 50-1000 us), word order sync, C0-C19, D0-D3 (EV1527 data sheet
  section 2.3).
- PYD 1588: configuration register (Table 5), SERIN timing tSL/tSH 200-2000 ns, tSHD 80-150 us,
  tSLT > 650 us, 150 us maximum pause in a word, DL low during configuration (2.6), DL clear
  >= 160 us and wake-up behaviour (2.4, 2.7), 3.6 V maximum (Table 3).

Unverified (marked in the code):
- NEC timing: taken from the commonly documented NEC format (uPD6121 family); no NEC document
  was obtained.
- EV1527 sync length (the figure draws the long low with a break) and all PT2262 timing (a
  secondary copy of its data sheet only): the decoder does not depend on the exact sync length.
- CMT2210LH DATA polarity (high while RF is received is assumed).
- PYD 1598 = PYD 1588 electrically: the PYD 1598 document could not be downloaded; Excelitas'
  brochure gives both the same electrical data and functions, differing in window and field of
  view.
- PYD1598 board: pinout, supply and DL components (no repo or schematic), and the DL pull-up
  behaviour (Wiring).
- IR board: the TXD default state (LED off when TXD floats) and that the receiver echoes the
  board's own LED.
- Real remotes, ranges and decoding margins; the RMT timing of the 1 us SERIN pulses.
- Nothing was built or run: only per-file syntax checks (host and ESP32 for `src/`, host for
  the tests) were done.

## Resource use

| Class | RMT TX (3 shared with AddressableLED) | RMT RX (4) | Other |
|---|---|---|---|
| `IRTransceiver` | 1 (with a transmit pin) | 1 (with a receive pin) | |
| `OokReceiver` | | 1 | |
| `PYD1598` | 1 (SERIN) | | 1 DIO (DL) |

- No threads. Decoding runs in `Poll()` on the caller's thread (a few hundred pulses at most per
  call). RAM: a few KiB per object (burst buffers, a 256-pulse history in `OokReceiver`), plus
  F3's static capture slots.

## Tests

- `IRTransceiverTest.cpp` (`[irtransceiver]`): the NEC train and carrier, TX-to-RX loopback of
  frames and repeat codes, raw trains, receiver distortion of +-200 us, extended addresses, a
  wrong command inverse, a repeat code without a frame, queued frames, an out-of-range bit space,
  one-pin use and pin conflicts.
- `OokReceiverTest.cpp` (`[ookreceiver]`): EV1527 frames after noise (confirmation, held key,
  new press), PT2262 tri-state words with +-100 us jitter ending in a capture end marker, 32-bit
  and 8-bit frames at other pulse lengths, too few bits, a 40 us short pulse rejected and 60 us
  accepted (both with 2 ms sync gaps), single and broken frames, pin conflict.
- `PYD1598Test.cpp` (`[pyd1598]`): configuration words (defaults, all-maximum, all-minimum,
  rounding, every range error), the SERIN waveform decoded by a fake sensor (25 rising edges,
  timing, no carrier) with DL held low as an output during it, reconfiguration, a rejected
  setting, DL motion and clear, pin conflicts. DL uses DIO channels below 31 (the host sim HAL
  has 31 DIO channels; pulse channels may use GPIO 0-48).

## Deviations from the plan

- `IRTransceiver`: either pin may be -1 (one-sided use); `SendNecRepeat()` and
  `GetLastPulses()` added (the plan's "raw pulses" on the receive side); extended NEC addresses
  are decoded (`IrProtocol::kNecExtended`, so `IrFrame::address` is 16 bits).
- `OokReceiver`: `OokCode` has a `repeat` flag; raw pulses through `GetRecentPulses()`; frames of
  8 to 32 bits, not only 24.
- `PYD1598`: two constructors instead of one with a default `Config` (a nested struct with
  default member initializers cannot be a default argument inside its class); `Config` also has
  the HPF cut-off and count mode bits; `Configure(const Config&)`, `GetConfig()` and the static
  `EncodeConfig()` added. The register layout is from the PYD 1588 document (see above).

## Shared edits (integration)

- `design/drivers.md`: link to this file.
- `design/API-DIFFERENCES.md` section 7: rows for `IRTransceiver`, `OokReceiver`, `PYD1598`
  (prosdk additions, no WPILib counterpart).
- The integration resource budget table: the RMT TX/RX and DIO use above.
