// Copyright (c) Probot Studio. Hand-written (not vendored); see LICENSE.
//
// Semtech SX1261/SX1262 LoRa transceiver over prosdk::SPI (design/boardoza/P11-lora.md). Written
// for the Ebyte E22-900M22S module (SX1262, 32 MHz TCXO powered from DIO3, external RF switch on
// RXEN/TXEN) on the Boardoza E22-900M22S board. LoRa only (no FSK), explicit header, payload CRC
// on, standard IQ. Clean-room from the Semtech SX1261/2 datasheet rev. 1.2 (June 2019); the
// command and register references are in SX126x.cpp.
//
// The chip has no interrupt line to the HAL: a thread polls DIO1 (TxDone, RxDone, CRC and header
// errors, timeout) every 2 ms and reads BUSY before every command. Send() and StartReceive() do
// not wait for the air time; received packets are queued for Poll().
//
// Regulatory: the 863-870 MHz and 902-928 MHz bands have power and duty-cycle limits that this
// driver does not enforce. Use GetTimeOnAir() to keep within them (P11-lora.md, "Regulatory").

#pragma once

#include <stdint.h>

#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>
#include <thread>
#include <vector>

#include "prosdk/drivers/bus/SPI.hpp"
#include "prosdk/hardware/discrete/DigitalInput.hpp"
#include "prosdk/hardware/discrete/DigitalOutput.hpp"
#include "prosdk/telemetry/TelemetryLoggable.hpp"
#include "prosdk/units/frequency.hpp"
#include "prosdk/units/time.hpp"

namespace prosdk {

/** Settings of an SX126x radio (SX126x::Config). Both ends of a link need the same values. */
struct SX126xConfig {
  /// LoRa bandwidth (datasheet table 13-48).
  enum class Bandwidth {
    k7_8kHz,
    k10_4kHz,
    k15_6kHz,
    k20_8kHz,
    k31_25kHz,
    k41_7kHz,
    k62_5kHz,
    k125kHz,
    k250kHz,
    k500kHz
  };

  /// DIO3 output voltage that supplies a TCXO (datasheet table 13-35); kNone for a crystal.
  enum class TcxoVoltage { kNone, k1_6V, k1_7V, k1_8V, k2_2V, k2_4V, k2_7V, k3_0V, k3_3V };

  /// Carrier frequency, 150 to 960 MHz (the E22-900M22S: 850 to 930 MHz). The default 868.1 MHz
  /// lies in the EU 868.0-868.6 MHz sub-band.
  prosdk::units::hertz_t frequency{868'100'000.0};
  /// Spreading factor, 5 to 12.
  int spreadingFactor = 7;
  /// Bandwidth.
  Bandwidth bandwidth = Bandwidth::k125kHz;
  /// Coding rate denominator: 5 to 8 for 4/5 to 4/8.
  int codingRate = 5;
  /// Output power of the SX1262 high-power PA, -9 to +22 dBm (conducted, before the antenna).
  /// The default 14 dBm (25 mW) is the EU 868.0-868.6 MHz limit with a 0 dBd antenna.
  int powerDbm = 14;
  /// TCXO supply on DIO3. E22-900M22S: 1.8 V (from the Boardoza library; unverified).
  TcxoVoltage tcxoVoltage = TcxoVoltage::k1_8V;
  /// LoRa sync word (registers 0x0740/0x0741): 0x1424 private network (reset value), 0x3444
  /// public (LoRaWAN).
  uint16_t syncWord = 0x1424;
  /// Preamble length in symbols, 1 to 65535.
  int preambleLength = 8;
  /// True when TXEN is wired to DIO2 instead of a GPIO: the chip then drives the TX switch
  /// (SetDIO2AsRfSwitchCtrl). RXEN still needs a GPIO.
  bool dio2AsRfSwitch = false;
  /// SPI clock (the chip allows 16 MHz, the E22 manual 10 Mbit/s).
  int spiClockHz = 8'000'000;
};

/**
 * Semtech SX1262 (SX126x) LoRa radio, for example the Ebyte E22-900M22S module.
 *
 * Call Begin() once: it resets and configures the chip (about 15 ms) and returns false when the
 * chip does not answer or a calibration fails. Then Send() a packet (up to 255 bytes) or
 * StartReceive() and read packets with Poll(). A Send() during reception interrupts it; the
 * reception resumes when the packet is sent.
 */
class SX126x : public prosdk::telemetry::TelemetryLoggable {
 public:
  using Config = SX126xConfig;

  /** A received packet. */
  struct Packet {
    /// Payload (CRC-checked).
    std::vector<uint8_t> data;
    /// Average RSSI over the packet, dBm (GetPacketStatus RssiPkt).
    double rssi = 0.0;
    /// Estimated SNR, dB (GetPacketStatus SnrPkt).
    double snr = 0.0;
  };

  /// Largest payload in bytes (the 256-byte data buffer, an 8-bit length field).
  static constexpr int kMaxPayload = 255;
  /// Received packets kept for Poll(); a newer packet drops the oldest one.
  static constexpr int kMaxQueuedPackets = 16;
  /// Period of the DIO1 polling thread.
  static constexpr prosdk::units::millisecond_t kPollPeriod{2};

  /**
   * Opens the SPI device and the control pins and starts the polling thread. Does not touch the
   * chip (Begin() does). Does not throw for a missing chip; the pin classes throw when a pin is
   * invalid or in use.
   *
   * @param bus    The SPI bus (prosdk::platform::SetSPIPins sets its pins).
   * @param csGpio The chip-select GPIO (NSS).
   * @param busy   The BUSY GPIO (input).
   * @param dio1   The DIO1 GPIO (input, the interrupt line).
   * @param reset  The NRST GPIO (output, active low), or -1 when it is not wired.
   * @param rxEn   The RXEN GPIO of the RF switch (output, high while receiving), or -1.
   * @param txEn   The TXEN GPIO of the RF switch (output, high while transmitting), or -1 (for
   *               example with Config::dio2AsRfSwitch).
   * @param config The radio settings.
   */
  SX126x(prosdk::SPI::Port bus, int csGpio, int busy, int dio1, int reset, int rxEn = -1,
         int txEn = -1, const Config& config = {});
  ~SX126x() override;

  SX126x(const SX126x&) = delete;
  SX126x& operator=(const SX126x&) = delete;

  /**
   * Resets the chip (when NRST is wired), checks that it answers in STDBY_RC, sets up the TCXO,
   * calibrates and applies the configuration. Blocks the caller for about 15 ms. Prints one
   * console line on failure. Calling it again starts over (a pending send or a reception stops).
   *
   * @return true when the chip is ready.
   */
  bool Begin();

  /**
   * Starts sending one packet without waiting for the air time; IsTxDone() turns true when it
   * has been sent (TxDone) or the send timed out on the chip (twice the time on air plus 100 ms).
   * It may wait a few ms for BUSY while the chip is still starting a reception (TCXO start-up).
   *
   * @return false when Begin() did not succeed, a send is still running, the size is not 1 to
   *         kMaxPayload, or a command failed.
   */
  bool Send(std::span<const uint8_t> data);

  /** True when no send is running (the last Send() finished or timed out, or none started). */
  bool IsTxDone() const;

  /**
   * Starts continuous reception (packets up to kMaxPayload bytes). It stays on until
   * StopReceive() or Begin(); a Send() pauses it.
   *
   * @return false when Begin() did not succeed or a command failed.
   */
  bool StartReceive();

  /** Stops reception (the chip goes to standby, the RF switch off) unless a send is running. */
  void StopReceive();

  /** The oldest received packet not yet returned, if any. */
  std::optional<Packet> Poll();

  /**
   * Time on air of a packet with this configuration (datasheet section 6.1.4, explicit header,
   * CRC on).
   *
   * @param payloadBytes Payload size, 0 to kMaxPayload.
   */
  prosdk::units::millisecond_t GetTimeOnAir(int payloadBytes) const;

  /**
   * Services DIO1 once: reads and clears the interrupt flags and handles them (the thread's
   * work; safe to call from another thread too). Returns at once while BUSY is high.
   */
  void Update();

  /**
   * True when Begin() succeeded and BUSY has not stayed high for 100 ms since it was last seen
   * low. After such a timeout commands fail at once (no new 100 ms wait) until BUSY is low again.
   */
  bool IsConnected() const;

  /** Packets sent (TxDone). */
  int GetSentCount() const;
  /** Sends that ended with the chip's timeout instead of TxDone. */
  int GetTxTimeoutCount() const;
  /** Packets received with a good CRC (queued for Poll()). */
  int GetReceivedCount() const;
  /** Packets dropped because of a payload CRC error or a header error. */
  int GetCrcErrorCount() const;
  /** Received packets dropped because the queue was full. */
  int GetDroppedCount() const;

  void LogTo(prosdk::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;

 private:
  enum class RfPath { kOff, kRx, kTx };

  bool Configure(const char** failed, uint16_t* deviceErrors);
  void Run();
  bool WaitReady();
  void BusyStuck();
  bool Command(std::span<const uint8_t> command);
  bool Read(std::span<const uint8_t> command, std::span<uint8_t> reply);
  bool WriteRegister(uint16_t address, std::span<const uint8_t> data);
  bool ReadRegister(uint16_t address, std::span<uint8_t> data);
  bool SetStandby();
  bool SetPacketParams(int payloadLength);
  bool ClearIrq(uint16_t flags);
  bool GetIrqStatus(uint16_t* flags);
  bool GetDeviceErrors(uint16_t* errors);
  bool StartRxLocked();
  void ReadPacketLocked();
  void SetRfPath(RfPath path);

  prosdk::SPI m_spi;
  prosdk::DigitalInput m_busy;
  prosdk::DigitalInput m_dio1;
  std::optional<prosdk::DigitalOutput> m_reset;
  std::optional<prosdk::DigitalOutput> m_rxEn;
  std::optional<prosdk::DigitalOutput> m_txEn;
  Config m_config;

  mutable std::mutex m_mutex;  // the chip and everything below
  std::chrono::steady_clock::time_point m_lastCommand{};
  bool m_ready = false;        // Begin() succeeded
  bool m_lastReadyOk = false;  // BUSY was not stuck high (kBusyTimeout) since it was last low
  bool m_warnedBusy = false;
  bool m_busyHigh = false;  // Update() saw BUSY high, since m_busySince
  std::chrono::steady_clock::time_point m_busySince{};
  bool m_txPending = false;
  bool m_rxWanted = false;
  std::deque<Packet> m_packets;
  double m_lastRssi = 0.0;
  double m_lastSnr = 0.0;
  int m_sent = 0;
  int m_txTimeouts = 0;
  int m_received = 0;
  int m_crcErrors = 0;
  int m_dropped = 0;

  std::atomic<bool> m_begun{false};  // m_ready, readable without the lock
  std::atomic<bool> m_stop{false};
  std::thread m_thread;
};

}  // namespace prosdk
