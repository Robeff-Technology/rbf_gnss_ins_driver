#ifndef RBF_GNSS_INS_DRIVER_BINARY_PARSER_H
#define RBF_GNSS_INS_DRIVER_BINARY_PARSER_H

#include <cstdint>
#include <functional>
#include <rbf_gnss_ins_driver/structs.h>
#include <string>
#include <vector>

namespace rbf_gnss_ins_driver {
class GnssStreamParser {
public:
  enum class MessageId : std::uint16_t {
    kGNSSPOS_1 = 42,
    kGNSSVEL_1 = 99,
    kECEF = 241,
    kHEADING = 971,
    kGNSSPOS = 1429,
    kGNSSVEL = 1430,
    kRAWIMUX = 1461,
    kINSPVAX = 1465,
    kIMUDATA = 2264,
  };

  GnssStreamParser();
  GnssStreamParser(std::function<void(const uint8_t *buffer,
                                      GnssStreamParser::MessageId msg_id)>
                       binary_callback,
                   std::function<void(const std::string &)> nmea_callback) {
    binary_callback_ = binary_callback;
    nmea_callback_ = nmea_callback;
  }
  ~GnssStreamParser();

  // Function to get Unix time in nanoseconds
  int64_t get_unix_time_ns();

  // Function to handle new data
  void parse(const uint8_t *buffer, size_t size);

private:
  enum class ParserMode {
    UNKNOWN,
    NMEA,   // Starts with '$'
    BINARY, // Starts with 0xAA
  };

  Header header_;

  void handle_byte(uint8_t byte);
  void check_binary_message(const uint8_t *buffer, size_t size);

  // Parser mode switches based on lead byte ($ or 0xAA)
  ParserMode mode_ = ParserMode::UNKNOWN;

  // NMEA buffer
  std::string nmea_buffer_;
  bool nmea_started_ = false;

  // Binary buffer
  std::vector<uint8_t> binary_buffer_;
  size_t binary_expected_length_ = 0;
  bool binary_header_found_ = false;
  

  // Callbacks
  std::function<void(const std::string &)> nmea_callback_;
  std::function<void(const uint8_t *, GnssStreamParser::MessageId)>
      binary_callback_;
};

} // namespace rbf_gnss_ins_driver

#endif // RBF_GNSS_INS_DRIVER_BINARY_PARSER_H