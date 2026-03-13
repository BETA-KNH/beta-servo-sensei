#pragma once

#include <atomic>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <sys/types.h>
#include <vector>

namespace fake_servo_bus {

inline constexpr uint8_t INSTR_PING      = 0x01;
inline constexpr uint8_t INSTR_READ      = 0x02;
inline constexpr uint8_t INSTR_SYNC_READ = 0x82;

inline constexpr uint8_t REG_TORQUE_SWITCH = 0x28;
inline constexpr uint8_t REG_ACCELERATION  = 0x29;
inline constexpr uint8_t REG_TARGET_LOC    = 0x2A;
inline constexpr uint8_t REG_OP_SPEED      = 0x2E;
inline constexpr uint8_t REG_TORQUE_LIMIT  = 0x30;
inline constexpr uint8_t REG_P_COEFF       = 0x15;
inline constexpr uint8_t REG_D_COEFF       = 0x16;
inline constexpr uint8_t REG_I_COEFF       = 0x17;
inline constexpr uint8_t REG_SPEED_P       = 0x25;
inline constexpr uint8_t REG_VEL_I         = 0x27;
inline constexpr uint8_t REG_CURRENT_LOC   = 0x38;
inline constexpr uint8_t REG_CURRENT_SPEED = 0x3A;
inline constexpr uint8_t REG_CURRENT_LOAD  = 0x3C;
inline constexpr uint8_t REG_VOLTAGE       = 0x3E;
inline constexpr uint8_t REG_TEMPERATURE   = 0x3F;
inline constexpr uint8_t REG_SERVO_STATUS  = 0x41;
inline constexpr uint8_t REG_MOVE_FLAG     = 0x42;
inline constexpr uint8_t REG_CURRENT       = 0x45;
inline constexpr uint8_t BROADCAST_ID      = 0xFE;

struct SimServo {
    int16_t current_pos = 2047;
    int16_t target_pos = 2047;
    uint16_t op_speed = 500;
    uint8_t torque_on = 0;
    uint8_t p_coeff = 32;
    uint8_t d_coeff = 32;
    uint8_t i_coeff = 0;
    uint8_t speed_p = 10;
    uint8_t vel_i = 10;
    uint16_t torque_limit = 1000;
};

extern std::map<uint8_t, SimServo> g_servos;
extern std::mutex g_mutex;
extern std::atomic<bool> g_running;

extern std::string g_displayPty;
extern std::string g_displayPortName;
extern std::string g_yarpLogFile;

extern pid_t g_yarpdevPid;

}  // namespace fake_servo_bus
