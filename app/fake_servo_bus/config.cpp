#include "config.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>

namespace fake_servo_bus {

static std::string trim(std::string s)
{
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return {};
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

static bool parseIdsValue(const std::string& raw, std::vector<uint8_t>& ids)
{
    std::string cleaned;
    cleaned.reserve(raw.size());
    for (char c : raw) {
        cleaned.push_back((c == '(' || c == ')') ? ' ' : c);
    }

    std::istringstream ss(cleaned);
    std::vector<uint8_t> parsed;
    int v = 0;
    while (ss >> v) {
        if (v < 0 || v > 253) return false;
        parsed.push_back(static_cast<uint8_t>(v));
    }
    if (parsed.empty()) return false;

    ids = std::move(parsed);
    return true;
}

static bool loadConfigFromIni(const std::string& filePath,
                              std::vector<uint8_t>& ids,
                              std::string& portName,
                              std::string& baud,
                              bool useIds,
                              bool useName,
                              bool useBaud,
                              std::string& err)
{
    std::ifstream in(filePath);
    if (!in.is_open()) {
        err = "cannot open file: " + filePath;
        return false;
    }

    std::string line;
    bool foundIds = false;
    while (std::getline(in, line)) {
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = trim(line);
        if (line.empty()) continue;

        std::istringstream ls(line);
        std::string key;
        ls >> key;
        std::string value;
        std::getline(ls, value);
        value = trim(value);
        if (value.empty()) continue;

        if (key == "servo_ids") {
            std::vector<uint8_t> tmpIds;
            if (!parseIdsValue(value, tmpIds)) {
                err = "invalid servo_ids in " + filePath + ": " + value;
                return false;
            }
            foundIds = true;
            if (useIds) ids = std::move(tmpIds);
        } else if (key == "name") {
            if (useName) portName = value;
        } else if (key == "baud") {
            if (useBaud) baud = value;
        }
    }

    if (useIds && !foundIds) {
        err = "missing 'servo_ids' in " + filePath;
        return false;
    }
    return true;
}

void printUsage(const char* prog)
{
    std::cout
        << "Usage: " << prog << " [options]\n"
        << "\n"
        << "Debug simulator for the ST servo bus. Creates a fake serial device\n"
        << "using a PTY and keeps the simulated servos alive until you stop it.\n"
        << "\n"
        << "Options:\n"
        << "  --help, -h          Show this help and exit\n"
        << "  --from conf/file.ini Load defaults from YARP ini (servo_ids/name/baud)\n"
        << "  --ids \"1 2 3\"       Servo IDs to simulate (default: \"1 2 3\")\n"
        << "  --name /my/port     YARP port prefix (default: /robot/debug_servo)\n"
        << "  --baud 115200       Baud rate for yarpdev/manual use (default: 115200)\n"
        << "  --launch-yarp       Also start yarpdev using conf/st_servo.ini\n"
        << "  --no-ui             Disable the ncurses live-state display\n"
        << "\n"
        << "Default workflow:\n"
        << "  1. Start this simulator and note the printed PTY path.\n"
        << "  2. Run yarpdev yourself with conf/st_servo.ini, overriding port/name/baud/servo_ids.\n"
        << "  3. Connect tools such as yarpmotorgui to the YARP ports.\n"
        << "\n"
        << "Examples:\n"
        << "  " << prog << "\n"
        << "  " << prog << " --from conf/st_servo.ini\n"
        << "  " << prog << " --ids \"1 2 3 4\" --name /robot/debug_servo\n"
        << "  yarpdev --from conf/st_servo.ini --port /dev/pts/N --baud 115200 \\\n"
        << "          --name /robot/debug_servo --servo_ids \"(1 2 3)\"\n"
        << "  " << prog << " --launch-yarp --ids \"1 2 3\"\n";
}

std::string buildIdsString(const std::vector<uint8_t>& ids)
{
    std::string idsStr = "(";
    for (std::size_t i = 0; i < ids.size(); ++i) {
        idsStr += std::to_string(ids[i]);
        if (i + 1 < ids.size()) idsStr += ' ';
    }
    idsStr += ')';
    return idsStr;
}

bool parseArgs(int argc, char** argv, Options& options, std::string& error, bool& wantHelp)
{
    bool idsCliSet = false;
    bool nameCliSet = false;
    bool baudCliSet = false;
    wantHelp = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
            wantHelp = true;
            return true;
        }
        if (std::strcmp(argv[i], "--from") == 0 && i + 1 < argc) {
            options.fromFile = argv[++i];
        } else if (std::strcmp(argv[i], "--launch-yarp") == 0) {
            options.launchYarp = true;
        } else if (std::strcmp(argv[i], "--no-ui") == 0) {
            options.noUi = true;
        } else if (std::strcmp(argv[i], "--no-yarp") == 0) {
            // backward-compatible no-op
        } else if (std::strcmp(argv[i], "--ids") == 0 && i + 1 < argc) {
            options.ids.clear();
            std::istringstream ss(argv[++i]);
            int v = 0;
            while (ss >> v) options.ids.push_back(static_cast<uint8_t>(v));
            idsCliSet = true;
        } else if (std::strcmp(argv[i], "--name") == 0 && i + 1 < argc) {
            options.portName = argv[++i];
            nameCliSet = true;
        } else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            options.baud = argv[++i];
            baudCliSet = true;
        } else if (argv[i][0] == '-') {
            error = std::string("Unknown option: ") + argv[i];
            return false;
        } else {
            error = std::string("Unexpected positional argument: ") + argv[i];
            return false;
        }
    }

    if (!options.fromFile.empty()) {
        std::string iniErr;
        if (!loadConfigFromIni(options.fromFile,
                               options.ids,
                               options.portName,
                               options.baud,
                               !idsCliSet,
                               !nameCliSet,
                               !baudCliSet,
                               iniErr)) {
            error = std::string("--from ") + iniErr;
            return false;
        }
    }

    if (options.ids.empty()) {
        error = "--ids must specify at least one servo ID";
        return false;
    }

    return true;
}

}  // namespace fake_servo_bus
