#ifndef RBF_GNSS_INS_DRIVER__GNSS_STREAM_PARSER_HPP_
#define RBF_GNSS_INS_DRIVER__GNSS_STREAM_PARSER_HPP_

#include <rbf_gnss_ins_driver/structs.h>

#include <array>
#include <cstdint>
#include <functional>
#include <string>

namespace rbf_gnss_ins_driver
{

/**
 * @brief Streaming GNSS parser supporting Binary + NMEA protocols.
 *
 * This parser consumes raw byte streams and extracts:
 *  - Proprietary binary GNSS/INS frames
 *  - NMEA sentences ($GGA, $RMC, etc.)
 *
 * Designed as a production-grade FSM parser:
 *  - No dynamic allocation
 *  - Byte-wise processing
 *  - Error counters & recovery
 */
class GnssStreamParser
{
public:
  /**
   * @brief Supported binary message identifiers.
   */
  enum class MessageId : std::uint16_t {
    GNSSPOS_1 = 42,
    GNSSVEL_1 = 99,
    ECEF = 241,
    HEADING = 971,
    GNSSPOS = 1429,
    GNSSVEL = 1430,
    RAWIMUX = 1461,
    INSPVAX = 1465,
    IMUDATA = 2264,
  };

  using BinaryCallback = std::function<void(const uint8_t * payload, MessageId msg_id)>;

  using NmeaCallback = std::function<void(const std::string & sentence)>;

  /**
   * @brief Runtime parser statistics for diagnostics.
   */
  struct ParserStats
  {
    uint64_t sync_errors{0};
    uint64_t header_length_errors{0};
    uint64_t payload_length_errors{0};
    uint64_t crc_errors{0};
    uint64_t messages_ok{0};
    uint64_t buffer_overflows{0};
    uint64_t nmea_overflows{0};
    uint64_t nmea_checksum_errors{0};
  };

  /**
   * @brief Construct parser with optional callbacks.
   */
  GnssStreamParser(BinaryCallback binary_cb = nullptr, NmeaCallback nmea_cb = nullptr)
  : binary_callback_(std::move(binary_cb)), nmea_callback_(std::move(nmea_cb))
  {
  }

  ~GnssStreamParser() = default;

  /**
   * @brief Feed raw data into the parser.
   *
   * @param buffer Pointer to raw byte array
   * @param size Number of bytes
   */
  void parse(const uint8_t * buffer, size_t size);
  /**
   * @brief Access parser statistics.
   */
  const ParserStats & stats() const noexcept { return stats_; }

  int64_t get_unix_time_ns();

private:
  // ============================================================================
  // Protocol constants
  // ============================================================================
  static constexpr uint8_t SYNC1 = 0xAA;
  static constexpr uint8_t SYNC2 = 0x44;
  static constexpr uint8_t SYNC3 = 0x12;

  static constexpr size_t MAX_HEADER_LEN = 32U;
  static constexpr size_t MAX_PAYLOAD_LEN = 512U;
  static constexpr size_t CRC_LEN = sizeof(uint32_t);
  static constexpr size_t MAX_FRAME_LEN = MAX_HEADER_LEN + MAX_PAYLOAD_LEN + CRC_LEN;

  enum class StreamType { UNKNOWN, BINARY, NMEA };

  // ============================================================================
  // FSM
  // ============================================================================
  enum class ParserState : uint8_t {
    WAIT_SYNC_1,
    WAIT_SYNC_2,
    WAIT_SYNC_3,
    READ_HEADER_LENGTH,
    READ_HEADER,
    READ_PAYLOAD_AND_CRC
  };

  // ============================================================================
  // NMEA parsing states
  // ============================================================================

  enum class NmeaState {
    WAIT_DOLLAR,
    READ_PREFIX_1,
    READ_PREFIX_2,
    READ_BODY,
    READ_CHECKSUM_1,
    READ_CHECKSUM_2,
    WAIT_CR,
    WAIT_LF
  };

  StreamType stream_type_{StreamType::UNKNOWN};
  ParserState state_{ParserState::WAIT_SYNC_1};
  NmeaState nmea_state_{NmeaState::WAIT_DOLLAR};

  /**
   * @brief Reset parser state and internal buffers.
   */
  void binary_reset(bool reset_stream);
  void nmea_reset(bool reset_stream);
  uint8_t nmea_prefix_[2];

  // ============================================================================
  // Buffers & State
  // ============================================================================
  std::array<uint8_t, MAX_FRAME_LEN> buffer_{};
  size_t index_{0};
  size_t expected_length_{0};
  uint8_t header_length_{0};

  std::string nmea_buffer_;
  uint8_t nmea_calc_crc_{0};
  uint8_t nmea_recv_crc_{0};

  Header header_{};
  ParserStats stats_{};

  // ============================================================================
  // Callbacks
  // ============================================================================
  BinaryCallback binary_callback_;
  NmeaCallback nmea_callback_;

  // ============================================================================
  // Internal helpers
  // ============================================================================
  void handle_byte(uint8_t byte);
  bool detect_stream_type_and_consume(uint8_t byte);
  void handle_complete_binary_frame();

  // NMEA handling helpers (optional extension)
  void nmea_parser(uint8_t byte);
  void binary_parser(uint8_t byte);
};

}  // namespace rbf_gnss_ins_driver

#endif  // RBF_GNSS_INS_DRIVER__GNSS_STREAM_PARSER_HPP_
