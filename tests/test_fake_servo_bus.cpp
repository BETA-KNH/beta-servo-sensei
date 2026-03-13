#include <gtest/gtest.h>

#include <fstream>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include "serial/SerialPortDriver.hpp"
#include "servo/STServoRegisters.hpp"
#include "servo/STServoRequest.hpp"
#include "servo/STServoResponse.hpp"

namespace {

std::set<std::string> listPtsDevices()
{
    std::set<std::string> out;
    namespace fs = std::filesystem;
    const fs::path ptsDir{"/dev/pts"};

    if (!fs::exists(ptsDir)) return out;

    for (const auto& entry : fs::directory_iterator(ptsDir)) {
        const auto name = entry.path().filename().string();
        if (!name.empty() &&
            std::all_of(name.begin(), name.end(),
                        [](unsigned char c) { return std::isdigit(c) != 0; })) {
            out.insert(entry.path().string());
        }
    }
    return out;
}

std::optional<std::string>
waitForNewPts(const std::set<std::string>& before,
              std::chrono::milliseconds timeout)
{
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto now = listPtsDevices();
        for (const auto& dev : now) {
            if (before.find(dev) == before.end()) {
                return dev;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return std::nullopt;
}

bool processAlive(pid_t pid)
{
    return pid > 0 && (::kill(pid, 0) == 0);
}

std::optional<uint16_t> readU16(SerialPortDriver& port, uint8_t id,
                                const STServo::Reg& reg)
{
    if (!port.write(STServoRequest::read(id, reg))) return std::nullopt;
    auto resp = STServo::ServoResponse::parse(port.read(6 + reg.size));
    if (!resp.valid) return std::nullopt;
    return resp.asUint16(0);
}

bool pingOk(SerialPortDriver& port, uint8_t id)
{
    if (!port.write(STServoRequest::ping(id))) return false;
    auto resp = STServo::ServoResponse::parse(port.read(6));
    return resp.valid && resp.id == id;
}

} // namespace

TEST(FakeServoBusTest, ProtocolReadWriteAndSyncReadWork)
{
    const auto ptsBefore = listPtsDevices();

    pid_t pid = ::fork();
    ASSERT_GE(pid, 0);

    if (pid == 0) {
        ::execl(FAKE_SERVO_BUS_PATH,
                FAKE_SERVO_BUS_PATH,
                "--ids", "1 2 3",
                static_cast<char*>(nullptr));
        _exit(127);
    }

    auto newPts = waitForNewPts(ptsBefore, std::chrono::milliseconds(3000));
    ASSERT_TRUE(newPts.has_value()) << "fake_servo_bus did not create a new PTY";
    ASSERT_TRUE(processAlive(pid)) << "fake_servo_bus exited unexpectedly";

    SerialPortDriver port(*newPts, 115200, 200);
    ASSERT_TRUE(port.isOpen()) << "Could not open fake PTY: " << *newPts;

    // PING all simulated IDs.
    for (uint8_t id : {1u, 2u, 3u}) {
        ASSERT_TRUE(port.write(STServoRequest::ping(id)));
        auto resp = STServo::ServoResponse::parse(port.read(6));
        ASSERT_TRUE(resp.valid);
        EXPECT_EQ(resp.id, id);
    }

    // Default CURRENT_LOCATION should be 2047 steps.
    auto cur = readU16(port, 1, STServo::Register::CURRENT_LOCATION);
    ASSERT_TRUE(cur.has_value());
    EXPECT_EQ(*cur, static_cast<uint16_t>(2047));

    // WRITE TARGET_LOCATION, then READ it back.
    const uint16_t target = 1234;
    ASSERT_TRUE(port.write(STServoRequest::write(
        STServo::Register::TARGET_LOCATION,
        1,
        {static_cast<uint8_t>(target & 0xFF),
         static_cast<uint8_t>((target >> 8) & 0xFF)})));

    auto targetRead = readU16(port, 1, STServo::Register::TARGET_LOCATION);
    ASSERT_TRUE(targetRead.has_value());
    EXPECT_EQ(*targetRead, target);

    // SYNC_WRITE OPERATION_SPEED for two servos, then verify with READ.
    std::vector<std::pair<uint8_t, std::vector<uint8_t>>> syncTargets = {
        {1, {0x11, 0x00}}, // 17
        {2, {0x22, 0x00}}, // 34
    };
    ASSERT_TRUE(port.write(STServoRequest::write(STServo::Register::OPERATION_SPEED,
                                                 syncTargets)));

    auto s1 = readU16(port, 1, STServo::Register::OPERATION_SPEED);
    auto s2 = readU16(port, 2, STServo::Register::OPERATION_SPEED);
    ASSERT_TRUE(s1.has_value());
    ASSERT_TRUE(s2.has_value());
    EXPECT_EQ(*s1, static_cast<uint16_t>(17));
    EXPECT_EQ(*s2, static_cast<uint16_t>(34));

    // SYNC_READ CURRENT_LOCATION from all IDs.
    ASSERT_TRUE(port.write(STServoRequest::read(std::vector<uint8_t>{1, 2, 3},
                                                STServo::Register::CURRENT_LOCATION)));
    for (uint8_t expectedId : {1u, 2u, 3u}) {
        auto resp = STServo::ServoResponse::parse(port.read(8));
        ASSERT_TRUE(resp.valid);
        EXPECT_EQ(resp.id, expectedId);
        ASSERT_TRUE(resp.asUint16(0).has_value());
    }

    // Clean shutdown.
    ::kill(pid, SIGINT);
    int status = 0;
    ::waitpid(pid, &status, 0);
    EXPECT_TRUE(WIFEXITED(status) || WIFSIGNALED(status));
}

TEST(FakeServoBusTest, FromIniLoadsServoIds)
{
    const std::filesystem::path iniPath =
        std::filesystem::temp_directory_path() / "fake_servo_bus_test.ini";

    {
        std::ofstream out(iniPath);
        ASSERT_TRUE(out.is_open());
        out << "name /robot/from_ini\n";
        out << "baud 115200\n";
        out << "servo_ids (7 8)\n";
    }

    const auto ptsBefore = listPtsDevices();

    pid_t pid = ::fork();
    ASSERT_GE(pid, 0);

    if (pid == 0) {
        ::execl(FAKE_SERVO_BUS_PATH,
                FAKE_SERVO_BUS_PATH,
                "--from", iniPath.c_str(),
                static_cast<char*>(nullptr));
        _exit(127);
    }

    auto newPts = waitForNewPts(ptsBefore, std::chrono::milliseconds(3000));
    ASSERT_TRUE(newPts.has_value()) << "fake_servo_bus did not create a new PTY";
    ASSERT_TRUE(processAlive(pid)) << "fake_servo_bus exited unexpectedly";

    SerialPortDriver port(*newPts, 115200, 200);
    ASSERT_TRUE(port.isOpen()) << "Could not open fake PTY: " << *newPts;

    // IDs from the ini must answer.
    EXPECT_TRUE(pingOk(port, 7));
    EXPECT_TRUE(pingOk(port, 8));

    // Default ID (1) should not exist when servo_ids was loaded from ini.
    EXPECT_FALSE(pingOk(port, 1));

    ::kill(pid, SIGINT);
    int status = 0;
    ::waitpid(pid, &status, 0);
    std::error_code ec;
    std::filesystem::remove(iniPath, ec);
    EXPECT_TRUE(WIFEXITED(status) || WIFSIGNALED(status));
}
