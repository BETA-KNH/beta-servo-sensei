#ifndef ST_SERVO_REQUEST_HPP
#define ST_SERVO_REQUEST_HPP

#include <cstdint>
#include <utility>
#include <vector>
#include "servo/STServoRegisters.hpp"

/**
 * @brief Static packet builders for the ST bus servo binary protocol.
 *
 * Packet layout:
 * @code
 *   [0xFF] [0xFF] [ID] [LEN] [INSTRUCTION] [PARAMS...] [CHECKSUM]
 *
 *   LEN      = sizeof(INSTRUCTION) + sizeof(PARAMS) + sizeof(CHECKSUM)
 *   CHECKSUM = ~(ID + LEN + INSTRUCTION + PARAMS...) & 0xFF
 * @endcode
 *
 * Overloaded pairs:
 *   - read(uint8_t, const STServo::Reg&)                          → READ       (0x02)
 *   - read(const std::vector<uint8_t>&, const STServo::Reg&)      → SYNC_READ  (0x82)
 *   - write(uint8_t, const STServo::Reg&, const std::vector&)     → WRITE      (0x03)
 *   - write(const STServo::Reg&, const std::vector of pairs)      → SYNC_WRITE (0x83)
 */

class STServoRequest {
public:
    /// @brief Build a PING packet — queries servo presence.
    /// @param id  Target servo ID.
    /// @return    Packet bytes; the servo returns a status packet.
    static std::vector<uint8_t> ping(uint8_t id)
    {
        return buildPacket(id, STServo::Instruction::PING, {});
    }

    /// @brief Build a READ packet — request a register from a single servo.
    /// @param id   Target servo ID.
    /// @param reg  Register to read.
    /// @return     Packet bytes.
    static std::vector<uint8_t> read(uint8_t id, const STServo::Reg& reg)
    {
        return buildPacket(id, STServo::Instruction::READ,
                           { reg.address, reg.size });
    }

    /// @brief Build a SYNC_READ packet — request the same register from multiple servos.
    ///
    /// Each addressed servo replies with its own individual response packet in ID order.
    /// @param ids  List of servo IDs to query.
    /// @param reg  Register to read from each servo.
    /// @return     Packet bytes (broadcast address, instruction 0x82).
    static std::vector<uint8_t> read(const std::vector<uint8_t>& ids,
                                     const STServo::Reg& reg)
    {
        std::vector<uint8_t> params;
        params.reserve(2 + ids.size());
        params.push_back(reg.address);
        params.push_back(reg.size);
        params.insert(params.end(), ids.begin(), ids.end());
        return buildPacket(STServo::BROADCAST_ID,
                           STServo::Instruction::SYNC_READ, params);
    }

    /// @brief Build a WRITE packet — write a register on a single servo immediately.
    /// @param id    Target servo ID.
    /// @param reg   Register to write (address and expected size).
    /// @param data  Bytes to write; length should equal `reg.size`.
    /// @return      Packet bytes.
    static std::vector<uint8_t> write(uint8_t id, const STServo::Reg& reg,
                                      const std::vector<uint8_t>& data)
    {
        std::vector<uint8_t> params;
        params.reserve(1 + data.size());
        params.push_back(reg.address);
        params.insert(params.end(), data.begin(), data.end());
        return buildPacket(id, STServo::Instruction::WRITE, params);
    }

    /// @brief Build a SYNC_WRITE packet — write the same register to multiple servos atomically.
    ///
    /// No response packets are returned by the servos.
    /// @param reg      Target register (address and per-servo data size).
    /// @param targets  Pairs of `(servo_id, data)`; every data vector must be `reg.size` bytes.
    /// @return         Packet bytes (broadcast address, instruction 0x83).
    static std::vector<uint8_t>
    write(const STServo::Reg& reg,
          const std::vector<std::pair<uint8_t, std::vector<uint8_t>>>& targets)
    {
        std::vector<uint8_t> params;
        params.reserve(2 + targets.size() * (1 + reg.size));
        params.push_back(reg.address);
        params.push_back(reg.size);            // data length per servo
        for (const auto& [servo_id, data] : targets) {
            params.push_back(servo_id);
            params.insert(params.end(), data.begin(), data.end());
        }
        return buildPacket(STServo::BROADCAST_ID,
                           STServo::Instruction::SYNC_WRITE, params);
    }

    /// @brief Build a REG_WRITE packet — buffer a command until ACTION is sent.
    ///
    /// Useful for synchronising multiple servos when each receives different
    /// data. Combine with @ref action() to trigger simultaneously.
    /// @param id    Target servo ID.
    /// @param reg   Register to write.
    /// @param data  Bytes to write; length should equal `reg.size`.
    /// @return      Packet bytes.
    static std::vector<uint8_t> regWrite(uint8_t id, const STServo::Reg& reg,
                                         const std::vector<uint8_t>& data)
    {
        std::vector<uint8_t> params;
        params.reserve(1 + data.size());
        params.push_back(reg.address);
        params.insert(params.end(), data.begin(), data.end());
        return buildPacket(id, STServo::Instruction::REG_WRITE, params);
    }

    /// @brief Build an ACTION packet — trigger all buffered REG_WRITE commands.
    /// @param id  Target ID (default: broadcast so all servos act simultaneously).
    /// @return    Packet bytes.
    static std::vector<uint8_t> action(uint8_t id = STServo::BROADCAST_ID)
    {
        return buildPacket(id, STServo::Instruction::ACTION, {});
    }

    /// @brief Build a RESET packet — reset the servo's EEPROM to factory defaults.
    /// @param id  Target servo ID.
    /// @return    Packet bytes.
    static std::vector<uint8_t> reset(uint8_t id)
    {
        return buildPacket(id, STServo::Instruction::RESET, {});
    }

private:
    /// @brief Assemble a complete packet from its components.
    /// @param id      Destination servo ID.
    /// @param instr   Instruction byte.
    /// @param params  Instruction parameters (may be empty).
    /// @return        Full packet: `[FF FF id len instr params... cksum]`.
    static std::vector<uint8_t> buildPacket(uint8_t id,
                                            STServo::Instruction instr,
                                            const std::vector<uint8_t>& params)
    {
        const uint8_t len = static_cast<uint8_t>(2 + params.size());

        std::vector<uint8_t> packet;
        packet.reserve(5 + params.size());  // 2×header + id + len + instr + params + cksum
        packet.push_back(STServo::HEADER);
        packet.push_back(STServo::HEADER);
        packet.push_back(id);
        packet.push_back(len);
        packet.push_back(static_cast<uint8_t>(instr));
        packet.insert(packet.end(), params.begin(), params.end());

        // Checksum covers every byte from ID onward (excludes the two 0xFF headers)
        uint8_t sum = 0;
        for (std::size_t i = 2; i < packet.size(); ++i)
            sum = static_cast<uint8_t>(sum + packet[i]);
        packet.push_back(static_cast<uint8_t>(~sum & 0xFFu));

        return packet;
    }
};

#endif // ST_SERVO_REQUEST_HPP