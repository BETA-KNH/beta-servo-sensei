#ifndef ST_SERVO_REGISTERS_HPP
#define ST_SERVO_REGISTERS_HPP

#include <cstdint>

namespace STServo {

    static constexpr uint8_t HEADER       = 0xFF; ///< Packet start byte (two consecutive).
    static constexpr uint8_t BROADCAST_ID = 0xFE; ///< Broadcast address — all servos listen, none reply.

    /// @brief Instruction byte values used in request packets.
    enum Instruction : uint8_t {
        PING       = 0x01,  ///< Query servo presence / status.
        READ       = 0x02,  ///< Read register(s) from a single servo.
        WRITE      = 0x03,  ///< Write register(s) to a single servo immediately.
        REG_WRITE  = 0x04,  ///< Write register(s) but hold until ACTION is sent.
        ACTION     = 0x05,  ///< Trigger all pending REG_WRITE commands.
        RESET      = 0x06,  ///< Reset servo to factory defaults.
        SYNC_READ  = 0x82,  ///< Read the same register from multiple servos.
        SYNC_WRITE = 0x83,  ///< Write the same register to multiple servos atomically.
    };

    /// @brief Bitmask flags carried in the error byte of every response packet.
    ///
    /// A servo sets one or more bits to signal a fault condition.
    enum ErrorFlag : uint8_t {
        VOLTAGE     = 0x01,  ///< Input voltage out of operating range.
        ANGLE       = 0x02,  ///< Position sensor / angle error.
        OVERHEAT    = 0x04,  ///< Internal temperature exceeded limit.
        RANGE       = 0x08,  ///< Command value out of range.
        CHECKSUM    = 0x10,  ///< Received packet checksum mismatch.
        OVERLOAD    = 0x20,  ///< Sustained load exceeded torque limit.
        INSTRUCTION = 0x40,  ///< Unknown or malformed instruction received.
    };

    enum class MemoryArea {
        EEPROM,
        SRAM,
    };

    enum class Access {
        R,  // Read-only
        RW  // Read-write
    };

    /// @brief Descriptor for a single servo register.
    struct Reg {
        uint8_t    address; ///< Register address on the bus.
        uint8_t    size;    ///< Width in bytes.
        MemoryArea area;    ///< EEPROM (persistent) or SRAM (volatile).
        Access     access;  ///< Read-only or read-write.
    };

    /**
     * @brief Register table
     *
     * Each entry: `{ address, size_bytes, area, access }`.
     * Inline comments show: initial value | range | unit.
     */
    namespace Register {

        // --- EEPROM — version / identity (read-only) -------------------------
        static constexpr Reg FIRMWARE_MAJOR_VER  = { 0x00, 1, MemoryArea::EEPROM, Access::R  }; // init 3
        static constexpr Reg FIRMWARE_MINOR_VER   = { 0x01, 1, MemoryArea::EEPROM, Access::R  }; // init 7
        static constexpr Reg SERVO_MAJOR_VER      = { 0x03, 1, MemoryArea::EEPROM, Access::R  }; // init 9
        static constexpr Reg SERVO_MINOR_VER      = { 0x04, 1, MemoryArea::EEPROM, Access::R  }; // init 3

        // --- EEPROM — identity / comms ---------------------------------------
        static constexpr Reg ID              = { 0x05, 1, MemoryArea::EEPROM, Access::RW }; // init 1  | 0-253
        static constexpr Reg BAUD_RATE       = { 0x06, 1, MemoryArea::EEPROM, Access::RW }; // init 7  | 0-7   (0=1M,1=500k,2=250k,3=128k,4=115200,5=76800,6=57600,7=38400)
        static constexpr Reg RETURN_DELAY    = { 0x07, 1, MemoryArea::EEPROM, Access::RW }; // init 0  | 0-254 | 2 us
        static constexpr Reg RESPONSE_LEVEL  = { 0x08, 1, MemoryArea::EEPROM, Access::RW }; // init 1  | 0-1   (0=no response for writes,1=all)

        // --- EEPROM — motion limits ------------------------------------------
        static constexpr Reg MIN_ANGLE_LIMIT = { 0x09, 2, MemoryArea::EEPROM, Access::RW }; // init 0    | -32766..32767 | step
        static constexpr Reg MAX_ANGLE_LIMIT = { 0x0B, 2, MemoryArea::EEPROM, Access::RW }; // init 4095 | -32767..32767 | step

        // --- EEPROM — protection thresholds ----------------------------------
        static constexpr Reg MAX_TEMP_LIMIT  = { 0x0D, 1, MemoryArea::EEPROM, Access::RW }; // init 70  | 0-100  | °C
        static constexpr Reg MAX_VOLTAGE     = { 0x0E, 1, MemoryArea::EEPROM, Access::RW }; // init 80  | 0-254  | 0.1 V
        static constexpr Reg MIN_VOLTAGE     = { 0x0F, 1, MemoryArea::EEPROM, Access::RW }; // init 40  | 0-254  | 0.1 V
        static constexpr Reg MAX_TORQUE      = { 0x10, 2, MemoryArea::EEPROM, Access::RW }; // init 1000| 0-1000 | 0.1 %  (1000=100% locked torque)

        // --- EEPROM — special function / alarm -------------------------------
        static constexpr Reg PHASE              = { 0x12, 1, MemoryArea::EEPROM, Access::RW }; // init 12  | 0-254  (special byte, see datasheet)
        static constexpr Reg UNLOAD_CONDITION   = { 0x13, 1, MemoryArea::EEPROM, Access::RW }; // init 44  | 0-254  bitmask
        static constexpr Reg LED_ALARM          = { 0x14, 1, MemoryArea::EEPROM, Access::RW }; // init 47  | 0-254  bitmask

        // --- EEPROM — PID / control ------------------------------------------
        static constexpr Reg P_COEFF            = { 0x15, 1, MemoryArea::EEPROM, Access::RW }; // init 32  | 0-254
        static constexpr Reg D_COEFF            = { 0x16, 1, MemoryArea::EEPROM, Access::RW }; // init 32  | 0-254
        static constexpr Reg I_COEFF            = { 0x17, 1, MemoryArea::EEPROM, Access::RW }; // init 0   | 0-254
        static constexpr Reg MIN_STARTUP_FORCE  = { 0x18, 2, MemoryArea::EEPROM, Access::RW }; // init 16  | 0-1000 | 0.1 %
        static constexpr Reg CW_DEADZONE        = { 0x1A, 1, MemoryArea::EEPROM, Access::RW }; // init 1   | 0-32   | step
        static constexpr Reg CCW_DEADZONE       = { 0x1B, 1, MemoryArea::EEPROM, Access::RW }; // init 1   | 0-32   | step
        static constexpr Reg PROTECTION_CURRENT = { 0x1C, 2, MemoryArea::EEPROM, Access::RW }; // init 500 | 0-511  | 6.5 mA
        static constexpr Reg ANGULAR_RESOLUTION = { 0x1E, 1, MemoryArea::EEPROM, Access::RW }; // init 1   | 1-3
        static constexpr Reg POSITION_CORRECTION= { 0x1F, 2, MemoryArea::EEPROM, Access::RW }; // init 0   | -2047..2047 | step (bit15=direction)

        // --- EEPROM — operational mode / overload ----------------------------
        static constexpr Reg OPERATION_MODE        = { 0x21, 1, MemoryArea::EEPROM, Access::RW }; // init 0 | 0=servo,1=constant-speed,2=PWM,3=step
        static constexpr Reg PROTECTION_TORQUE     = { 0x22, 1, MemoryArea::EEPROM, Access::RW }; // init 20  | 0-100  | 1 %
        static constexpr Reg PROTECTION_TIME       = { 0x23, 1, MemoryArea::EEPROM, Access::RW }; // init 200 | 0-254  | 10 ms
        static constexpr Reg OVERLOAD_TORQUE       = { 0x24, 1, MemoryArea::EEPROM, Access::RW }; // init 80  | 0-100  | 1 %
        static constexpr Reg SPEED_P_COEFF         = { 0x25, 1, MemoryArea::EEPROM, Access::RW }; // init 10  | 0-100  (mode 1 speed loop P)
        static constexpr Reg OVERCURRENT_TIME      = { 0x26, 1, MemoryArea::EEPROM, Access::RW }; // init 200 | 0-254  | 10 ms  (max 2540 ms)
        static constexpr Reg VELOCITY_I_COEFF      = { 0x27, 1, MemoryArea::EEPROM, Access::RW }; // init 10  | 0-254  (mode 1 speed loop I)

        // --- SRAM area (volatile) — motion control --------------------------
        static constexpr Reg TORQUE_SWITCH       = { 0x28, 1, MemoryArea::SRAM, Access::RW }; // 0=off,1=on,128=hold current pos
        static constexpr Reg ACCELERATION        = { 0x29, 1, MemoryArea::SRAM, Access::RW }; // 0-254 | 100 step/s²
        static constexpr Reg TARGET_LOCATION     = { 0x2A, 2, MemoryArea::SRAM, Access::RW }; // -30719..30719 | step
        static constexpr Reg OPERATION_TIME      = { 0x2C, 2, MemoryArea::SRAM, Access::RW }; // 0-1000 | 0.1 %  (ST3215: unused, keep 0)
        static constexpr Reg OPERATION_SPEED     = { 0x2E, 2, MemoryArea::SRAM, Access::RW }; // 0-3400 | step/s
        static constexpr Reg TORQUE_LIMIT        = { 0x30, 2, MemoryArea::SRAM, Access::RW }; // 0-1000 | 1.0 %  (init = MAX_TORQUE value)
        static constexpr Reg LOCK_FLAG           = { 0x37, 1, MemoryArea::SRAM, Access::RW }; // 0=open EEPROM writes, 1=lock EEPROM writes

        // --- SRAM area (volatile) — read-only status -------------------------
        static constexpr Reg CURRENT_LOCATION    = { 0x38, 2, MemoryArea::SRAM, Access::R  }; // step   | current absolute position
        static constexpr Reg CURRENT_SPEED       = { 0x3A, 2, MemoryArea::SRAM, Access::R  }; // step/s | current rotation speed
        static constexpr Reg CURRENT_LOAD        = { 0x3C, 2, MemoryArea::SRAM, Access::R  }; // 0.1 %  | voltage duty cycle of drive output
        static constexpr Reg CURRENT_VOLTAGE     = { 0x3E, 1, MemoryArea::SRAM, Access::R  }; // 0.1 V  | current supply voltage
        static constexpr Reg CURRENT_TEMPERATURE = { 0x3F, 1, MemoryArea::SRAM, Access::R  }; // °C     | internal temperature
        static constexpr Reg ASYNC_WRITE_FLAG    = { 0x40, 1, MemoryArea::SRAM, Access::R  }; // flag bit for asynchronous write in progress
        static constexpr Reg SERVO_STATUS        = { 0x41, 1, MemoryArea::SRAM, Access::R  }; // bitmask: voltage/sensor/temp/angle/overload/no-phase
        static constexpr Reg MOVE_FLAG           = { 0x42, 1, MemoryArea::SRAM, Access::R  }; // 1=moving, 0=stopped
        static constexpr Reg CURRENT_CURRENT     = { 0x45, 2, MemoryArea::SRAM, Access::R  }; // 6.5 mA | max measurable = 500*6.5=3250 mA

    } // namespace Register

} // namespace STServo

#endif // ST_SERVO_REGISTERS_HPP
