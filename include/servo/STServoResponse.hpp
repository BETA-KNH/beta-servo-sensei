#ifndef ST_SERVO_RESPONSE_HPP
#define ST_SERVO_RESPONSE_HPP

#include <cstdint>
#include <optional>
#include <vector>
#include "servo/STServoRegisters.hpp"

/**
 * @brief Parsed representation of a response packet sent by the servo.
 *
 * Packet layout:
 * @code
 *   [0xFF] [0xFF] [ID] [LEN] [ERROR] [DATA...] [CHECKSUM]
 *
 *   LEN      = sizeof(ERROR) + sizeof(DATA) + sizeof(CHECKSUM)
 *   CHECKSUM = ~(ID + LEN + ERROR + DATA...) & 0xFF
 * @endcode
 *
 * @par Example
 * @code
 *   auto resp = ServoResponse::parse(rawBytes);
 *   if (!resp.valid) { // bad packet
 *   }
 *   if (resp.hasError()) { // servo reported a fault
 *   }
 *   uint16_t pos = resp.asUint16();  // first two data bytes, little-endian
 * @endcode
 */

namespace STServo {

struct ServoResponse {
    bool                  valid = false;  ///< @c false if headers, length, or checksum are wrong.
    uint8_t               id    = 0;      ///< Echoed servo ID.
    uint8_t               error = 0;      ///< Raw error byte (see @ref ErrorFlag).
    std::vector<uint8_t>  data;           ///< Payload bytes, excluding framing and checksum.

    /// @brief Returns @c true if any error bit is set in the error byte.
    bool hasError() const { return error != 0; }

    /// @brief Returns @c true if a specific @ref ErrorFlag bit is set.
    /// @param f  The flag bit to test.
    bool hasFlag(ErrorFlag f) const
    {
        return (error & static_cast<uint8_t>(f)) != 0;
    }

    /// @brief Decode two consecutive data bytes as a little-endian 16-bit unsigned integer.
    /// @param offset  Byte offset into @ref data (default 0).
    /// @return        The decoded value, or @c std::nullopt if there are not enough bytes.
    std::optional<uint16_t> asUint16(std::size_t offset = 0) const
    {
        if (offset + 1 >= data.size()) return std::nullopt;
        return static_cast<uint16_t>(
            static_cast<uint16_t>(data[offset]) |
            (static_cast<uint16_t>(data[offset + 1]) << 8));
    }

    /// @brief Parse a raw byte vector received from the bus.
    ///
    /// On any structural or checksum failure a response with @c valid = @c false
    /// is returned so the caller can handle corrupted packets without throwing.
    /// @param raw  Raw bytes as received from the serial port.
    /// @return     Parsed response; always check @ref valid before use.
    static ServoResponse parse(const std::vector<uint8_t>& raw)
    {
        ServoResponse resp;

        // Minimum packet: [FF FF id len error cksum] = 6 bytes
        if (raw.size() < 6)
            return resp;

        // Headers
        if (raw[0] != HEADER || raw[1] != HEADER)
            return resp;

        // LEN field: number of bytes after [FF FF id LEN], i.e. raw.size() - 4
        const uint8_t declaredLen = raw[3];
        if (declaredLen != static_cast<uint8_t>(raw.size() - 4))
            return resp;

        // Checksum covers bytes from ID onward, excluding the checksum byte itself
        uint8_t sum = 0;
        for (std::size_t i = 2; i < raw.size() - 1; ++i)
            sum = static_cast<uint8_t>(sum + raw[i]);
        if (static_cast<uint8_t>(~sum & 0xFF) != raw.back())
            return resp;

        // All checks passed
        resp.valid = true;
        resp.id    = raw[2];
        resp.error = raw[4];
        resp.data.assign(raw.begin() + 5, raw.end() - 1); // exclude checksum

        return resp;
    }
};

} // namespace STServo

#endif // ST_SERVO_RESPONSE_HPP