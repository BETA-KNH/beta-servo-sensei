#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fake_servo_bus {

struct Options {
    std::vector<uint8_t> ids{1, 2, 3};
    std::string portName{"/robot/debug_servo"};
    std::string baud{"115200"};
    std::string fromFile;
    bool launchYarp{false};
    bool noUi{false};
};

bool parseArgs(int argc, char** argv, Options& options, std::string& error, bool& wantHelp);
void printUsage(const char* prog);
std::string buildIdsString(const std::vector<uint8_t>& ids);

}  // namespace fake_servo_bus
