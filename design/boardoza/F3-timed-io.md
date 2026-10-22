# F3: timed DIO and RMT pulse I/O (gaps G3, G4)

Platform package for Boardoza boards that need microsecond timing on plain GPIOs. It adds two
prosdk extensions to the HAL (WPILib has neither) and three user classes:

- **Timed DIO bit-banging (G3):** `prosdk::DigitalShift` and `prosdk::WriteWaveform` on DIO pins
  the caller already owns, with interrupts masked for short windows.
- **Pulse I/O on the RMT peripheral (G4):** `prosdk::PulseTransmitter` (pulse trains, optionally
  on a carrier) and `prosdk::PulseCapture` (pulse-width capture in the background).

Boards covered (the board drivers themselves are in P6 and P12):

| Board | Chip | Uses | Driver package |
|---|---|---|---|
| HX711 Load Cell Amplifier | Avia HX711 | `DigitalShift` (clock out, data in) | P6 |
| IrDA Transceiver (a 38 kHz consumer-IR link, not IrDA SIR) | IRM-H638T receiver, VSMB10940 LED | `PulseTransmitter` on 38 kHz, `PulseCapture` | P12 |
| CMT2210LH RF Receiver | CMOSTEK CMT2210LH, 433.92 MHz OOK | `PulseCapture` | P12 |
| PYD1598 (Ozdisan only) | Excelitas PYD 1598 | `PulseTransmitter` without a carrier (SERIN word) | P12 |

## Files

| File | What |
|---|---|
| `src/prosdk/platform/hal/BitBang.hpp` | bit-bang backend interface, `LevelSpan`, argument checks |
| `src/prosdk/platform/hal/esp32/BitBang.cpp` | ESP32-S3 backend, target only |
| `src/prosdk/platform/hal/BitBangSim.cpp` | host backend over the DIO sim; empty on the robot |
| `src/prosdk/drivers/bus/DigitalShift.hpp` | `prosdk::DigitalShift`, `prosdk::WriteWaveform` (header only) |
| `src/prosdk/platform/hal/PulseIO.hpp` | pulse backend interface, `Pulse`, limits, argument checks |
| `src/prosdk/platform/hal/esp32/PulseIO.cpp` | ESP32-S3 backend (RMT TX and RX), target only |
| `src/prosdk/platform/hal/PulseIOSim.cpp` | host backend; empty on the robot |
| `src/prosdk/simulation/PulseIOSim.hpp` | sim hooks for tests (host only) |
| `src/prosdk/drivers/bus/PulseTransmitter.hpp`, `.cpp` | `prosdk::PulseTransmitter`, `prosdk::Pulse` |
| `src/prosdk/drivers/bus/PulseCapture.hpp`, `.cpp` | `prosdk::PulseCapture` |
| `src/prosdk/platform/hal/esp32/Esp32Internal.hpp` | shared RMT TX budget (`rmt::AcquireTxChannel`), `PinMode::kPulseTx/kPulseRx` |
| `src/prosdk/platform/hal/esp32/AddressableLED.cpp` | LED outputs now take their channel from the shared budget |
| `test/prosdk/drivers/DigitalShiftTest.cpp`, `PulseIOTest.cpp` | host tests |

## Wiring

All four boards connect with plain GPIOs; any free GPIO (0-48) works.

- **HX711:** CLK (PD_SCK) to a `DigitalOutput`, DAT (DOUT) to a `DigitalInput`. DOUT is a
  push-pull output at DVDD, so **DVDD (the board's VDD pin) must be 3.3 V**. The board's VCC pin
  takes 2.7-5.5 V; which J1 pin feeds which HX711 supply pin is assumed (VCC to the regulator,
  VDD to DVDD), not traced in the schematic (unverified).
- **IR transceiver:** TXD to the `PulseTransmitter` GPIO, RXD to the `PulseCapture` GPIO. RXD
  swings to the board's VCC: **at VCC = 5 V it outputs about 5 V, which an ESP32-S3 pin does
  not tolerate. Power the board from 3.3 V** (or add a divider). TXD high turns the IR LED on
  with no current limit other than a 33 ohm resistor (about 59 mA at 3.3 V, about 110 mA at 5 V,
  over the LED's 65 mA continuous rating), so the line must only carry carrier bursts. The
  transmitter idles low, and the carrier duty default is 33 %. RXD is low while a 38 kHz
  carrier is received and high when idle.
- **CMT2210LH:** DATA to the `PulseCapture` GPIO. **3.3 V only:** the board is wired in its
  low-voltage mode, absolute maximum 3.6 V on VDD. With no transmitter in range the AGC makes
  DATA toggle at random, so a decoder must sync on a protocol gap and validate pulse widths.
- **PYD1598:** SERIN to a `PulseTransmitter` without a carrier; the 25-bit configuration word at
  about 100 us per bit is longer than `WriteWaveform` allows. Bit layout and timing are from the
  Excelitas datasheet as summarised in the plan, not checked against the document (unverified).

## Class API

### `prosdk::DigitalShift` and `prosdk::WriteWaveform`

```cpp
DigitalShift(DigitalOutput& clock, DigitalInput& data,
             nanosecond_t highTime = 1000 ns, nanosecond_t lowTime = 1000 ns);
std::optional<uint64_t> ShiftIn(int bits);   // 1-64 bits, MSB first
int GetClockChannel() const;
int GetDataChannel() const;

bool WriteWaveform(DigitalOutput& pin, std::span<const platform::hal::LevelSpan> spans);
```

- Per bit: clock high, wait `highTime`, sample data, clock low, wait `lowTime`. The clock idles
  low. This is the HX711 readout (datasheet Fig. 2): 24 bits, then 1-3 more pulses select the
  next conversion (25 total: channel A gain 128, 26: channel B gain 32, 27: channel A gain 64).
- `highTime` 1 ns to 20 us, `lowTime` 0 to 1 ms; other values throw `ParameterOutOfRange` in the
  constructor. The defaults are the HX711's typical T3 and T4 (1 us each).
- `DigitalShift` keeps only the channel numbers; the caller keeps the `DigitalOutput` and
  `DigitalInput` alive. A clock with a digital PWM generator enabled is not supported.
- `ShiftIn` returns nothing after an error (bad bit count, a pin that is not an allocated DIO or
  the clock not an output, an emergency stop) and reports it to the console.
- `WriteWaveform`: 1-64 spans of `{level, ns}` with a total of at most 20 us, all with interrupts
  masked; the pin keeps the last level. Longer waveforms belong on a `PulseTransmitter`.

### `prosdk::PulseTransmitter`

```cpp
explicit PulseTransmitter(int channel, hertz_t carrier = 0 Hz, double carrierDuty = 0.33);
bool Send(std::span<const Pulse> pulses);   // Pulse{bool level; uint32_t us}
bool WaitSent(second_t timeout);
int GetChannel() const;
hertz_t GetCarrier() const;
```

- Each pulse is 1-32767 us (the RMT symbol's 15-bit duration at 1 us per tick), at most 1024
  pulses per `Send`. `Send` returns at once; a train still being sent is waited for first, up to
  its length plus 100 ms. After a train the line returns low.
- Carrier: 0 (none) or 2 kHz to 1 MHz, duty 0.1-0.9. With a carrier, high levels are carrier
  bursts and low levels silence. The 2 kHz floor comes from the RMT carrier's 16-bit high and low
  tick counts (at most 65536 group-clock ticks each; ESP-IDF asserts and aborts beyond that):
  with an 80 MHz group clock and a 0.9 duty, about 1.1 kHz is the lowest safe value.
  Out-of-range arguments throw in the constructor; a bad train makes `Send` return false and
  report the error.
- The constructor throws when the pin is in use or all RMT TX channels are taken. After an
  emergency stop it only reports the error (`HAL_INCOMPATIBLE_STATE` is a warning status, as for
  `DigitalOutput`); the transmitter stays closed and every `Send` fails.

### `prosdk::PulseCapture`

```cpp
explicit PulseCapture(int channel, microsecond_t idleTime = 10000 us,
                      microsecond_t glitchFilter = 0 us);
int Read(std::span<Pulse> out);   // non-blocking, oldest first
int64_t GetDropped() const;
int GetChannel() const;
```

- The line is captured in bursts. A burst starts at the first edge after an idle line. A level
  that lasts `idleTime` or longer ends the burst with a `Pulse{level, 0}` marker; the line then
  counts as idle at that level until it changes. `idleTime` must be longer than any pulse inside
  a burst (an NEC leader is 9 ms, so the 10 ms default fits), 10 us to 32.767 ms.
- `glitchFilter` 0-3 us: shorter pulses are ignored by the RMT input filter (255 APB cycles at
  80 MHz is the hardware limit, `RMT_LL_MAX_FILTER_VALUE`). Longer noise is the decoder's job.
- The buffer holds 256 pulses; more are dropped and counted. Read at least every loop.
- The pin is an input with the internal pull-up.

## ESP32-S3 backend

- **Bit-bang (`esp32/BitBang.cpp`):** the pins are looked up by GPIO in the DIO handle table and
  must be allocated DIOs in the right direction. Pin writes and reads are direct GPIO register
  accesses (`gpio_ll`), waits are busy loops on the CPU cycle counter. A `portENTER_CRITICAL`
  section covers each clock-high phase of `ShiftIn` (at most 20 us) and each whole
  `WriteWaveform`; the low phases run with interrupts enabled. This is safe for the HX711
  because its PD_SCK low time has no maximum, while a high time over 60 us powers it down (Avia
  HX711 datasheet, timing table T3/T4 and Fig. 3). A 27-pulse read with the defaults masks
  interrupts 27 times for about 1 us each.
- **Pulse TX (`esp32/PulseIO.cpp`):** one RMT TX channel per transmitter, 1 MHz resolution,
  48-symbol memory block, transfer queue depth 1, the copy encoder (two pulses per RMT symbol).
  The carrier is the RMT's own (`rmt_apply_carrier`, active-high levels only). The train buffer
  stays allocated until the next `Send`.
- **Pulse RX:** one RMT RX channel per capture, 1 MHz resolution, a 96-symbol receive buffer with
  partial receive (`en_partial_rx`), `signal_range_min_ns` = glitch filter, `signal_range_max_ns`
  = idle time. The receive-done interrupt copies symbols into a lock-free ring (256 packed
  pulses) and re-arms the receiver after the last part of a burst. Its context is a static slot
  that is never freed, so a late interrupt cannot touch freed memory, and the re-arm runs under a
  spinlock that the close takes before it disables the channel, so no re-arm follows a close.

## Emergency stop

- Bit-bang: nothing is written after the e-stop. A clock high that raced the e-stop is driven low
  again at the end of the bit or waveform, and the call fails.
- Pulse TX follows the LED data-line rule. The pin is registered as a HAL output pin
  (`SetOutputPin`), so the e-stop routes it to plain GPIO and drives it low. After the e-stop no
  transmitter opens and no train starts (`HAL_INCOMPATIBLE_STATE`); a train that started in the
  race window is cut by driving the pin low again. A transmitter closed after the e-stop leaves
  the pin a low output.
- Pulse TX is not gated while the robot is merely disabled, like LED strips: an IR command or a
  sensor configuration word is not an actuator output.
- Pulse capture is an input and is not affected.

## Host simulation

- `BitBangSim.cpp` drives the DIO sim channels: every clock edge and waveform level is a
  `HALSIM_SetDIOValue`, and the data pin is read with `HALSIM_GetDIOValue`. DIO sim value
  callbacks run synchronously, so a test emulates a chip from them. No time passes.
- `PulseIOSim.cpp` applies the same argument checks and channel counts (3 TX, 4 RX), and one
  owner per GPIO among pulse channels (it does not check other HAL users of the pin). Trains are
  sent at once; there is no e-stop. Hooks in `prosdk/simulation/PulseIOSim.hpp`:
  `OnPulsesSent(gpio, fn)`, `CancelPulsesSent(id)`, `InjectPulses(gpio, line)` (processed like the
  RMT receiver: glitch filter, merged equal levels, burst end marker, idle level ignored, buffer
  overflow) and `GetPulseTxCarrier(gpio, &hz, &duty)`.

## Tests

- `DigitalShiftTest.cpp` (`[digitalshift]`): a fake HX711 on DIO callbacks (24-bit word MSB
  first, then the gain pulse), 64-bit reads, argument and pin errors, `WriteWaveform` levels and
  its 20 us limit.
- `PulseIOTest.cpp` (`[pulseio]`): carrier and sent trains, train range errors, constructor
  errors, the 3 TX and 4 RX channel limits, burst splitting, glitch filter, buffer overflow, and a
  TX-to-RX loopback.

## Verified and unverified

Verified (from documents):

- HX711 timing: T1 >= 0.1 us, T2 <= 0.1 us, T3 0.2/1/50 us, T4 >= 0.2 us with no maximum,
  power-down after PD_SCK high > 60 us, 25/26/27 pulses for gain 128 A / 32 B / 64 A (Avia
  HX711 datasheet, timing table, Fig. 2 and Fig. 3).
- RMT limits from the ESP-IDF 5.5 headers: 15-bit symbol durations, filter at most 255 APB
  cycles (`RMT_LL_MAX_FILTER_VALUE`), idle threshold at most 32767 ticks
  (`RMT_LL_MAX_IDLE_VALUE`), 48 memory words per channel, 4 TX and 4 RX channels on the S3,
  partial receive supported (`SOC_RMT_SUPPORT_RX_PINGPONG`).
- Board supplies and levels from the Boardoza schematics and chip datasheets as listed in the
  plan's research (IRM-H638T output levels, VSMB10940 ratings, CMT2210LH low-voltage mode).

Unverified (no hardware run; marked in the code):

- The real high and low times of the bit-bang loop, and that WiFi (the Driver Station link) and
  probot-core's loop tolerate the masked windows of up to 20 us.
- RMT group clock prescaling to 1 MHz (the group clock staying the undivided 80 MHz APB clock),
  the input filter clock, and carrier frequency and duty accuracy.
- That the receiver starts a burst at the first edge after idle, and the partial-receive re-arm
  with bursts longer than the 96-symbol buffer.
- That the RX pin keeps the internal pull-up after `rmt_new_rx_channel`.
- That the RMT driver does not re-route a TX pin back to the RMT after the e-stop moved it to
  plain GPIO.
- The HX711 board's power pin assignment and the PYD1598 SERIN timing (see Wiring).
- Nothing was built or run: only per-file syntax checks (host and ESP32) were done.

## Resource use

- RMT TX: 4 channels on the S3. probot-core's status LED uses one; the other 3 are shared by
  `AddressableLED` outputs and `PulseTransmitter`s through one counter in `Esp32Internal.hpp`
  (`rmt::AcquireTxChannel`). Opening either kind fails with `HAL_NO_AVAILABLE_RESOURCES` when all
  3 are taken.
- RMT RX: 4 channels, one per `PulseCapture`.
- RAM: about 5.5 KiB of static memory for the 4 capture slots (each a 384-byte receive buffer
  and a 1 KiB ring), used or not; a transmitter holds its last train (4 bytes per 2 pulses).
- CPU: bit-bang calls busy-wait for their whole length on the calling thread. Pulse I/O uses no
  CPU while sending; capture costs one interrupt per received burst part.

## Deviations from the plan

- The bit-bang API takes GPIO numbers instead of `HAL_DigitalHandle`s, because wpilibc's
  `DigitalOutput`/`DigitalInput` do not expose their handles. The backend looks the DIO up by
  channel and checks its direction.
- `OpenPulseRx` takes `glitchUs` (0-3 us, the RMT filter limit) instead of a free `minUs`.
- Pulses are limited to 32767 us each (one RMT symbol half); longer levels are split by the
  caller.
- `PulseIO.hpp` also has `WaitPulsesSent` and `GetDroppedPulses`.

## Shared edits (integration)

- `design/hal-esp32.md`: "Timed DIO" and "Pulse I/O" sections; capacities row "LED outputs +
  pulse TX <= 3" and a pulse RX row (4); pulse TX in the e-stop output list and the not-gated
  note.
- `design/API-DIFFERENCES.md` section 7: rows for `DigitalShift`, `PulseTransmitter`,
  `PulseCapture`.
- `design/drivers.md`: link to this file.
- `tools/build/header_check.py`: add `src/prosdk/simulation/PulseIOSim.hpp` (a hand-written
  header in a vendored folder).
