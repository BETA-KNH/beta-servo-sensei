#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>

#include "servo/STServoRequest.hpp"
#include "servo/STServoResponse.hpp"
#include "serial/SerialPortConfig.hpp"
#include "serial/SerialPortDriver.hpp"
#include "servo/STServoRegisters.hpp"

// ---------------------------------------------------------------------------
// servo_cli — interactive command-line tool for ST bus servos
//
// Usage:
//   servo_cli <command> [args...]
//
// Commands:
//   ping  <id>                        Query servo presence
//   read  <id> <register>             Read named register, print value
//   write <id> <register> <value>     Write integer value to named register
//   move  <id> <position> [speed]     Move servo to position (0–4095)
//                                     speed = 0–254 step/s (0 = maximum speed)
//   reset <id>                        Factory reset servo EEPROM
//
// Registers (for read/write):
//   (see --help for full register list)
//
// Environment variables:
//   SERVO_PORT, SERVO_BAUD, SERVO_TIMEOUT_MS  (or set in servo.env / conf/servo.env)
//
// Examples:
//   servo_cli ping 1
//   servo_cli read 1 target_location
//   servo_cli write 1 id 2
//   servo_cli move 1 2048
//   servo_cli move 1 2048 1000 300
// ---------------------------------------------------------------------------

static const std::map<std::string, const STServo::Reg*> REGISTER_MAP = {
    // identity / comms
    { "firmware_major_ver",    &STServo::Register::FIRMWARE_MAJOR_VER  },
    { "firmware_minor_ver",    &STServo::Register::FIRMWARE_MINOR_VER  },
    { "servo_major_ver",        &STServo::Register::SERVO_MAJOR_VER      },
    { "servo_minor_ver",        &STServo::Register::SERVO_MINOR_VER      },
    { "id",                    &STServo::Register::ID                   },
    { "baud_rate",             &STServo::Register::BAUD_RATE            },
    { "return_delay",          &STServo::Register::RETURN_DELAY         },
    { "response_level",        &STServo::Register::RESPONSE_LEVEL       },
    // motion limits
    { "min_angle_limit",       &STServo::Register::MIN_ANGLE_LIMIT      },
    { "max_angle_limit",       &STServo::Register::MAX_ANGLE_LIMIT      },
    // protection thresholds
    { "max_temp_limit",        &STServo::Register::MAX_TEMP_LIMIT       },
    { "max_voltage",           &STServo::Register::MAX_VOLTAGE          },
    { "min_voltage",           &STServo::Register::MIN_VOLTAGE          },
    { "max_torque",            &STServo::Register::MAX_TORQUE           },
    // special / alarm
    { "phase",                 &STServo::Register::PHASE                },
    { "unload_condition",      &STServo::Register::UNLOAD_CONDITION     },
    { "led_alarm",             &STServo::Register::LED_ALARM            },
    // PID / control
    { "p_coeff",               &STServo::Register::P_COEFF              },
    { "d_coeff",               &STServo::Register::D_COEFF              },
    { "i_coeff",               &STServo::Register::I_COEFF              },
    { "min_startup_force",     &STServo::Register::MIN_STARTUP_FORCE    },
    { "cw_deadzone",           &STServo::Register::CW_DEADZONE          },
    { "ccw_deadzone",          &STServo::Register::CCW_DEADZONE         },
    { "protection_current",    &STServo::Register::PROTECTION_CURRENT   },
    { "angular_resolution",    &STServo::Register::ANGULAR_RESOLUTION   },
    { "position_correction",   &STServo::Register::POSITION_CORRECTION  },
    // operational mode / overload
    { "operation_mode",        &STServo::Register::OPERATION_MODE       },
    { "protection_torque",     &STServo::Register::PROTECTION_TORQUE    },
    { "protection_time",       &STServo::Register::PROTECTION_TIME      },
    { "overload_torque",       &STServo::Register::OVERLOAD_TORQUE      },
    { "speed_p_coeff",         &STServo::Register::SPEED_P_COEFF        },
    { "overcurrent_time",      &STServo::Register::OVERCURRENT_TIME     },
    { "velocity_i_coeff",      &STServo::Register::VELOCITY_I_COEFF     },
    // SRAM — motion control (read/write)
    { "torque_switch",         &STServo::Register::TORQUE_SWITCH        },
    { "acceleration",          &STServo::Register::ACCELERATION         },
    { "target_location",       &STServo::Register::TARGET_LOCATION      },
    { "operation_time",         &STServo::Register::OPERATION_TIME       },
    { "operation_speed",       &STServo::Register::OPERATION_SPEED      },
    { "torque_limit",          &STServo::Register::TORQUE_LIMIT         },
    { "lock_flag",              &STServo::Register::LOCK_FLAG            },
    // SRAM — status (read-only)
    { "current_location",      &STServo::Register::CURRENT_LOCATION     },
    { "current_speed",         &STServo::Register::CURRENT_SPEED        },
    { "current_load",          &STServo::Register::CURRENT_LOAD         },
    { "current_voltage",       &STServo::Register::CURRENT_VOLTAGE      },
    { "current_temperature",   &STServo::Register::CURRENT_TEMPERATURE  },
    { "async_write_flag",      &STServo::Register::ASYNC_WRITE_FLAG     },
    { "servo_status",          &STServo::Register::SERVO_STATUS         },
    { "move_flag",              &STServo::Register::MOVE_FLAG            },
    { "current_current",       &STServo::Register::CURRENT_CURRENT      },
};

// Total response bytes for a READ of `reg`: FF FF id LEN ERROR DATA... CKSUM
// LEN = 1(error) + reg.size + 1(cksum)  →  total = 4 + LEN = 6 + reg.size
static std::size_t readResponseLen(const STServo::Reg& reg)
{
    return static_cast<std::size_t>(6 + reg.size);
}

static void printUsage(const char* prog)
{
    std::printf(
        "Usage: %s <command> [args]\n"
        "\n"
        "Commands:\n"
        "  ping  <id>\n"
        "  read  <id> <register>\n"
        "  write <id> <register> <value>\n"
        "  move  <id> <position> [speed]    speed: 0-254 step/s (0 = max)\n"
        "  reset <id>\n"
        "\n"
        "Registers (EEPROM): id baud_rate return_delay response_level\n"
        "                    min_angle_limit max_angle_limit\n"
        "                    max_temp_limit max_voltage min_voltage max_torque\n"
        "                    phase unload_condition led_alarm\n"
        "                    p_coeff d_coeff i_coeff min_startup_force\n"
        "                    cw_deadzone ccw_deadzone protection_current\n"
        "                    angular_resolution position_correction\n"
        "                    operation_mode protection_torque protection_time\n"
        "                    overload_torque speed_p_coeff overcurrent_time velocity_i_coeff\n"
        "Registers (read-only): firmware_major_ver firmware_minor_ver servo_major_ver servo_minor_ver\n"
        "Registers (SRAM rw): torque_switch acceleration target_location operation_time\n"
        "                     operation_speed torque_limit lock_flag\n"
        "Registers (SRAM ro): current_location current_speed current_load\n"
        "                     current_voltage current_temperature async_write_flag\n"
        "                     servo_status move_flag current_current\n"
        "\n"
        "Env vars:  SERVO_PORT (required)  SERVO_BAUD (required)  SERVO_TIMEOUT_MS (optional, default 100)\n"
        "           Set in your shell or place them in conf/servo.env (auto-loaded).\n",
        prog);
}

// Encode an integer into a little-endian byte vector of `size` bytes
static std::vector<uint8_t> encodeLE(uint16_t value, uint8_t size)
{
    if (size == 1) return { static_cast<uint8_t>(value & 0xFF) };
    return { static_cast<uint8_t>(value & 0xFF),
             static_cast<uint8_t>((value >> 8) & 0xFF) };
}

// Send packet, read response, parse and print; returns true on success
static bool sendAndReceive(SerialPortDriver& port,
                           const std::vector<uint8_t>& packet,
                           std::size_t expectedResponseLen)
{
    if (!port.write(packet)) {
        std::fprintf(stderr, "Write error: %s\n", std::strerror(errno));
        return false;
    }

    auto raw  = port.read(expectedResponseLen);
    auto resp = STServo::ServoResponse::parse(raw);

    if (!resp.valid) {
        std::fprintf(stderr, "No valid response (timeout or framing error).\n");
        return false;
    }

    if (resp.hasError()) {
        std::fprintf(stderr, "Servo ID %d reported error: 0x%02X\n",
                     resp.id, resp.error);
        if (resp.hasFlag(STServo::ErrorFlag::VOLTAGE))     std::fputs("  [!] Voltage\n",     stderr);
        if (resp.hasFlag(STServo::ErrorFlag::ANGLE))       std::fputs("  [!] Angle\n",        stderr);
        if (resp.hasFlag(STServo::ErrorFlag::OVERHEAT))    std::fputs("  [!] Overheat\n",     stderr);
        if (resp.hasFlag(STServo::ErrorFlag::RANGE))       std::fputs("  [!] Range\n",        stderr);
        if (resp.hasFlag(STServo::ErrorFlag::CHECKSUM))    std::fputs("  [!] Checksum\n",     stderr);
        if (resp.hasFlag(STServo::ErrorFlag::OVERLOAD))    std::fputs("  [!] Overload\n",     stderr);
        if (resp.hasFlag(STServo::ErrorFlag::INSTRUCTION)) std::fputs("  [!] Instruction\n",  stderr);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

static int cmdPing(SerialPortDriver& port, uint8_t id)
{
    std::printf("Pinging servo %d...\n", id);
    bool ok = sendAndReceive(port, STServoRequest::ping(id), 6);
    if (ok) std::printf("OK — servo %d is present.\n", id);
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

static int cmdRead(SerialPortDriver& port, uint8_t id, const std::string& regName)
{
    auto it = REGISTER_MAP.find(regName);
    if (it == REGISTER_MAP.end()) {
        std::fprintf(stderr, "Unknown register '%s'.\n", regName.c_str());
        return EXIT_FAILURE;
    }
    const STServo::Reg& reg = *it->second;

    if (!port.write(STServoRequest::read(id, reg))) {
        std::fprintf(stderr, "Write error: %s\n", std::strerror(errno));
        return EXIT_FAILURE;
    }

    auto raw  = port.read(readResponseLen(reg));
    auto resp = STServo::ServoResponse::parse(raw);

    if (!resp.valid) {
        std::fprintf(stderr, "No valid response.\n");
        return EXIT_FAILURE;
    }

    if (reg.size == 1) {
        uint8_t val = resp.data.empty() ? 0 : resp.data[0];
        std::printf("%s = %u  (0x%02X)\n", regName.c_str(), val, val);
    } else {
        auto val = resp.asUint16();
        if (val) std::printf("%s = %u  (0x%04X)\n", regName.c_str(), *val, *val);
        else     std::fprintf(stderr, "Not enough data bytes in response.\n");
    }

    return resp.valid ? EXIT_SUCCESS : EXIT_FAILURE;
}

static int cmdWrite(SerialPortDriver& port, uint8_t id,
                    const std::string& regName, uint16_t value)
{
    auto it = REGISTER_MAP.find(regName);
    if (it == REGISTER_MAP.end()) {
        std::fprintf(stderr, "Unknown register '%s'.\n", regName.c_str());
        return EXIT_FAILURE;
    }
    const STServo::Reg& reg = *it->second;

    if (reg.access == STServo::Access::R) {
        std::fprintf(stderr, "Register '%s' is read-only.\n", regName.c_str());
        return EXIT_FAILURE;
    }

    bool ok = sendAndReceive(port,
                             STServoRequest::write(reg, id, encodeLE(value, reg.size)),
                             6);
    if (ok) std::printf("Wrote %u to %s on servo %d.\n", value, regName.c_str(), id);
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

static int cmdMove(SerialPortDriver& port, uint8_t id,
                   uint16_t position, uint16_t speed)
{
    // ST-series servos (e.g. ST3215) have no time mode.  The memory map at
    // 0x2A is: pos(2) | time_slot(2) | speed(2).  We write all six bytes in
    // one atomic WRITE, hardcoding the time slot to 0x0000.
    auto pos   = encodeLE(position, 2);
    auto sdata = encodeLE(speed,    2);

    std::vector<uint8_t> combined;
    combined.reserve(6);
    combined.insert(combined.end(), pos.begin(),  pos.end());
    combined.push_back(0x00); combined.push_back(0x00);  // time slot — always 0
    combined.insert(combined.end(), sdata.begin(), sdata.end());

    bool ok = sendAndReceive(port,
                             STServoRequest::write(STServo::Register::TARGET_LOCATION, id,
                                                     combined),
                             6);
    if (ok) {
        std::printf("Moving servo %d → position %u", id, position);
        if (speed > 0) std::printf("  speed=%u", speed);
        std::printf("\n");
    }
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

static int cmdReset(SerialPortDriver& port, uint8_t id)
{
    std::printf("Resetting servo %d to factory defaults...\n", id);
    bool ok = sendAndReceive(port, STServoRequest::reset(id), 6);
    if (ok) std::printf("Reset sent to servo %d.\n", id);
    return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

int main(int argc, char* argv[])
{
    if (argc < 2) { printUsage(argv[0]); return EXIT_FAILURE; }

    const std::string cmd = argv[1];

    // ---- Load config & open port -------------------------------------------
    STServo::SerialPortConfig cfg;
    try { cfg = STServo::SerialPortConfig::fromEnv(); }
    catch (const std::exception& e) {
        std::fprintf(stderr, "Config error: %s\n", e.what());
        return EXIT_FAILURE;
    }

    SerialPortDriver port(cfg);
    if (!port.isOpen()) {
        std::fprintf(stderr, "Failed to open %s: %s\n",
                     cfg.port.c_str(), std::strerror(errno));
        return EXIT_FAILURE;
    }

    // ---- Dispatch ----------------------------------------------------------
    if (cmd == "ping") {
        if (argc < 3) { std::fprintf(stderr, "Usage: servo_cli ping <id>\n"); return EXIT_FAILURE; }
        return cmdPing(port, static_cast<uint8_t>(std::atoi(argv[2])));
    }

    if (cmd == "read") {
        if (argc < 4) { std::fprintf(stderr, "Usage: servo_cli read <id> <register>\n"); return EXIT_FAILURE; }
        return cmdRead(port, static_cast<uint8_t>(std::atoi(argv[2])), argv[3]);
    }

    if (cmd == "write") {
        if (argc < 5) { std::fprintf(stderr, "Usage: servo_cli write <id> <register> <value>\n"); return EXIT_FAILURE; }
        return cmdWrite(port,
                        static_cast<uint8_t>(std::atoi(argv[2])),
                        argv[3],
                        static_cast<uint16_t>(std::atoi(argv[4])));
    }

    if (cmd == "move") {
        if (argc < 4) { std::fprintf(stderr, "Usage: servo_cli move <id> <position> [speed]\n"); return EXIT_FAILURE; }
        uint16_t position = static_cast<uint16_t>(std::atoi(argv[3]));
        uint16_t speed    = argc >= 5 ? static_cast<uint16_t>(std::atoi(argv[4])) : 0;
        return cmdMove(port, static_cast<uint8_t>(std::atoi(argv[2])),
                       position, speed);
    }

    if (cmd == "reset") {
        if (argc < 3) { std::fprintf(stderr, "Usage: servo_cli reset <id>\n"); return EXIT_FAILURE; }
        return cmdReset(port, static_cast<uint8_t>(std::atoi(argv[2])));
    }

    std::fprintf(stderr, "Unknown command '%s'\n\n", cmd.c_str());
    printUsage(argv[0]);
    return EXIT_FAILURE;
}
