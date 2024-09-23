#ifndef RBF_GNSS_INS_DRIVER_BINARY_PARSER_H
#define RBF_GNSS_INS_DRIVER_BINARY_PARSER_H

#include <cstdint>
#include <functional>
#include <vector>
#include <rbf_gnss_ins_driver/structs.h>

namespace rbf_gnss_ins_driver
{
    class BinaryParser
    {
    public:
        enum class MessageId : std::uint16_t
        {
            kECEF = 241,
            kRAWIMU = 268,
            kHEADING = 971,
            kGNSSPOS = 1429,
            kGNSSVEL = 1430,
            kRAWIMUX = 1461,
            kINSPVAX = 1465,
        };

        BinaryParser();
        BinaryParser(std::function<void(const uint8_t *buffer, BinaryParser::MessageId msg_id)> callback)
        {
            callback_ = callback;
        }
        ~BinaryParser();

        // Function to handle new data
        void parse(const uint8_t *buffer, uint16_t size);

        // Function to set receive callback
        void set_receive_callback(std::function<void(const uint8_t *buffer, MessageId msg_id)> callback);

        // Function to get Unix time in nanoseconds
        int64_t get_unix_time_ns();

        Header header_;
    private:
        std::function<void(const uint8_t *, BinaryParser::MessageId)> callback_;
        enum class ParseStatus : uint8_t
        {
            SYNCH1_CONTROL,
            SYNCH2_CONTROL,
            SYNCH3_CONTROL,
            HEADER_LENGTH,
            HEADER_ADD,
            DATA_ADD,
        };
        ParseStatus status_;
        uint16_t data_index_;
        uint8_t raw_data_[512];
    };

} // namespace rbf_gnss_ins_driver

#endif // RBF_GNSS_INS_DRIVER_BINARY_PARSER_H