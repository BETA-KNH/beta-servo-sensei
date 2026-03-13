#include "common.hpp"

#include <sys/types.h>

namespace fake_servo_bus {

std::map<uint8_t, SimServo> g_servos;
std::mutex g_mutex;
std::atomic<bool> g_running{true};

std::string g_displayPty;
std::string g_displayPortName;
std::string g_yarpLogFile;

pid_t g_yarpdevPid = -1;

}  // namespace fake_servo_bus
