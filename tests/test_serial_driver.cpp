#include <cstdlib>
#include <gtest/gtest.h>
#include "servo/STServoRequest.hpp"
#include "servo/STServoResponse.hpp"
#include "PtyHelper.hpp"
#include "serial/SerialPortConfig.hpp"
#include "serial/SerialPortDriver.hpp"
#include "servo/STServoRegisters.hpp"

// ---------------------------------------------------------------------------
// SerialPortConfig — environment variable handling
// ---------------------------------------------------------------------------

TEST(SerialPortConfig, DefaultsWhenEnvUnset)
{
    // Clear the env vars to ensure we get defaults
    ::unsetenv("SERVO_PORT");
    ::unsetenv("SERVO_BAUD");
    ::unsetenv("SERVO_TIMEOUT_MS");

    auto cfg = STServo::SerialPortConfig::fromEnv();
    EXPECT_EQ(cfg.port,      "/dev/ttyUSB0");
    EXPECT_EQ(cfg.baud,      1000000);
    EXPECT_EQ(cfg.timeoutMs, 100);
}

TEST(SerialPortConfig, ReadsPortFromEnv)
{
    ::setenv("SERVO_PORT", "/dev/ttyAMA0", 1);
    auto cfg = STServo::SerialPortConfig::fromEnv();
    EXPECT_EQ(cfg.port, "/dev/ttyAMA0");
    ::unsetenv("SERVO_PORT");
}

TEST(SerialPortConfig, ReadsBaudFromEnv)
{
    ::setenv("SERVO_BAUD", "115200", 1);
    auto cfg = STServo::SerialPortConfig::fromEnv();
    EXPECT_EQ(cfg.baud, 115200);
    ::unsetenv("SERVO_BAUD");
}

TEST(SerialPortConfig, ReadsTimeoutFromEnv)
{
    ::setenv("SERVO_TIMEOUT_MS", "250", 1);
    auto cfg = STServo::SerialPortConfig::fromEnv();
    EXPECT_EQ(cfg.timeoutMs, 250);
    ::unsetenv("SERVO_TIMEOUT_MS");
}

TEST(SerialPortConfig, AllThreeEnvVarsOverrideDefaults)
{
    ::setenv("SERVO_PORT",       "/dev/ttyS0", 1);
    ::setenv("SERVO_BAUD",       "921600",     1);
    ::setenv("SERVO_TIMEOUT_MS", "50",         1);

    auto cfg = STServo::SerialPortConfig::fromEnv();
    EXPECT_EQ(cfg.port,      "/dev/ttyS0");
    EXPECT_EQ(cfg.baud,      921600);
    EXPECT_EQ(cfg.timeoutMs, 50);

    ::unsetenv("SERVO_PORT");
    ::unsetenv("SERVO_BAUD");
    ::unsetenv("SERVO_TIMEOUT_MS");
}

TEST(SerialPortConfig, InvalidBaudThrows)
{
    ::setenv("SERVO_BAUD", "-1", 1);
    EXPECT_THROW(STServo::SerialPortConfig::fromEnv(), std::invalid_argument);
    ::unsetenv("SERVO_BAUD");
}

TEST(SerialPortConfig, NegativeTimeoutThrows)
{
    ::setenv("SERVO_TIMEOUT_MS", "-100", 1);
    EXPECT_THROW(STServo::SerialPortConfig::fromEnv(), std::invalid_argument);
    ::unsetenv("SERVO_TIMEOUT_MS");
}

// ---------------------------------------------------------------------------
// SerialPortDriver — opening behaviour
// ---------------------------------------------------------------------------

TEST(SerialPortDriver, NonExistentPortIsNotOpen)
{
    // /dev/servo_nonexistent will never exist
    SerialPortDriver drv("/dev/servo_nonexistent", 115200, 100);
    EXPECT_FALSE(drv.isOpen());
}

TEST(SerialPortDriver, UnsupportedBaudThrowsBeforeOpen)
{
    // baudToConstant() throws for unmapped baud rates
    EXPECT_THROW(
        (SerialPortDriver("/dev/servo_nonexistent", 12345, 100)),
        std::invalid_argument);
}

TEST(SerialPortDriver, WriteReturnsFalseWhenNotOpen)
{
    SerialPortDriver drv("/dev/servo_nonexistent", 115200, 100);
    ASSERT_FALSE(drv.isOpen());
    EXPECT_FALSE(drv.write({0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(SerialPortDriver, ReadReturnsEmptyWhenNotOpen)
{
    SerialPortDriver drv("/dev/servo_nonexistent", 115200, 100);
    ASSERT_FALSE(drv.isOpen());
    EXPECT_TRUE(drv.read(6).empty());
}

TEST(SerialPortDriver, PortPathStoredCorrectly)
{
    SerialPortDriver drv("/dev/servo_nonexistent", 115200, 100);
    EXPECT_EQ(drv.portPath(), "/dev/servo_nonexistent");
}

// ---------------------------------------------------------------------------
// SerialPortDriver — hardware integration (skipped when port unavailable)
// ---------------------------------------------------------------------------

TEST(SerialPortDriver, HardwarePingServo1)
{
    // Reads SERVO_PORT from the environment; skips gracefully if unavailable.
    const char* envPort = std::getenv("SERVO_PORT");
    const std::string portPath = envPort ? envPort : "/dev/ttyUSB0";

    SerialPortDriver drv(portPath, 1000000, 100);
    if (!drv.isOpen()) {
        GTEST_SKIP() << "Serial port " << portPath << " not available — skipping hardware test";
    }

    auto packet = std::vector<uint8_t>{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}; // ping(1)
    EXPECT_TRUE(drv.write(packet));

    auto resp = drv.read(6);
    // We can't assert specific bytes without a real servo, but we can confirm
    // the driver returned a non-empty buffer within the timeout window.
    EXPECT_FALSE(resp.empty()) << "No response received — is the servo connected and powered?";
}

// ---------------------------------------------------------------------------
// PTY — virtual loopback tests (no hardware required)
//
// PtyHelper creates a linked master/slave PTY pair entirely in kernel memory.
// The slave path is handed to SerialPortDriver, the master fd is used by the
// test to play the role of the servo.
// ---------------------------------------------------------------------------

TEST(PtyHelper, Opens)
{
    PtyHelper pty;
    ASSERT_TRUE(pty.isOpen());
    EXPECT_FALSE(pty.slavePath().empty());
    EXPECT_GE(pty.masterFd(), 0);
}

TEST(SerialPortDriver, PtySlaveOpensSuccessfully)
{
    PtyHelper pty;
    ASSERT_TRUE(pty.isOpen());

    SerialPortDriver drv(pty.slavePath(), 115200, 100);
    EXPECT_TRUE(drv.isOpen());
}

// Bytes written by the driver arrive intact on the master (outgoing direction)
TEST(SerialPortDriver, PtyWrittenPacketArrivesOnMaster)
{
    PtyHelper pty;
    ASSERT_TRUE(pty.isOpen());
    SerialPortDriver drv(pty.slavePath(), 115200, 100);
    ASSERT_TRUE(drv.isOpen());

    auto packet = STServoRequest::ping(0x01);
    EXPECT_TRUE(drv.write(packet));

    auto received = pty.masterRead(packet.size());
    EXPECT_EQ(received, packet);
}

// Bytes written to the master arrive intact on the driver (incoming direction)
TEST(SerialPortDriver, PtyMasterBytesReadByDriver)
{
    PtyHelper pty;
    ASSERT_TRUE(pty.isOpen());
    SerialPortDriver drv(pty.slavePath(), 115200, 100);
    ASSERT_TRUE(drv.isOpen());

    // Fake servo response: ping reply from servo 1 with no errors
    // FF FF 01 02 00 FC
    const std::vector<uint8_t> fakeReply = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
    EXPECT_TRUE(pty.masterWrite(fakeReply));

    auto received = drv.read(fakeReply.size());
    EXPECT_EQ(received, fakeReply);
}

// Full round-trip: send a command, receive a response, parse it
TEST(SerialPortDriver, PtyPingRoundTrip)
{
    PtyHelper pty;
    ASSERT_TRUE(pty.isOpen());
    SerialPortDriver drv(pty.slavePath(), 115200, 100);
    ASSERT_TRUE(drv.isOpen());

    // 1. Send PING for servo 1
    auto pingPkt = STServoRequest::ping(0x01);
    ASSERT_TRUE(drv.write(pingPkt));

    // 2. Verify the master received the exact PING packet
    auto sentBytes = pty.masterRead(pingPkt.size());
    EXPECT_EQ(sentBytes, pingPkt);

    // 3. Simulate servo replying: FF FF 01 02 00 FC
    const std::vector<uint8_t> pingReply = {0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC};
    ASSERT_TRUE(pty.masterWrite(pingReply));

    // 4. Driver reads and parser validates the response
    auto raw  = drv.read(pingReply.size());
    auto resp = STServo::ServoResponse::parse(raw);

    EXPECT_TRUE(resp.valid);
    EXPECT_EQ(resp.id, 0x01);
    EXPECT_FALSE(resp.hasError());
}

// Full round-trip: READ command → fake register data response → parsed value
TEST(SerialPortDriver, PtyReadRegisterRoundTrip)
{
    PtyHelper pty;
    ASSERT_TRUE(pty.isOpen());
    SerialPortDriver drv(pty.slavePath(), 115200, 100);
    ASSERT_TRUE(drv.isOpen());

    // 1. Send READ for TARGET_LOCATION (addr=0x2A, size=2)
    auto readPkt = STServoRequest::read(0x01, STServo::Register::TARGET_LOCATION);
    ASSERT_TRUE(drv.write(readPkt));

    // Drain it from the master (we don't assert it here, already tested above)
    pty.masterRead(readPkt.size());

    // 2. Simulate servo replying with position = 0x0518 (1304)
    //    FF FF 01 04 00 18 05 DD  — same as the protocol doc example
    const std::vector<uint8_t> dataReply = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x18, 0x05, 0xDD};
    ASSERT_TRUE(pty.masterWrite(dataReply));

    // 3. Driver reads and parser decodes the 16-bit value
    auto raw  = drv.read(dataReply.size());
    auto resp = STServo::ServoResponse::parse(raw);

    ASSERT_TRUE(resp.valid);
    EXPECT_EQ(resp.id, 0x01);
    EXPECT_FALSE(resp.hasError());
    ASSERT_TRUE(resp.asUint16().has_value());
    EXPECT_EQ(*resp.asUint16(), 0x0518u);  // 1304 decimal
}

// Error flags in the response byte are correctly propagated through the driver
TEST(SerialPortDriver, PtyErrorFlagsDetected)
{
    PtyHelper pty;
    ASSERT_TRUE(pty.isOpen());
    SerialPortDriver drv(pty.slavePath(), 115200, 100);
    ASSERT_TRUE(drv.isOpen());

    // Build a valid ping reply with OVERLOAD (0x20) and OVERHEAT (0x04) flags set
    // FF FF 01 02 <error> <cksum>
    // error = 0x24
    // cksum = ~(01 + 02 + 24) & FF = ~27 & FF = D8
    const std::vector<uint8_t> faultReply = {0xFF, 0xFF, 0x01, 0x02, 0x24, 0xD8};
    ASSERT_TRUE(pty.masterWrite(faultReply));

    auto raw  = drv.read(faultReply.size());
    auto resp = STServo::ServoResponse::parse(raw);

    ASSERT_TRUE(resp.valid);
    EXPECT_TRUE(resp.hasError());
    EXPECT_TRUE(resp.hasFlag(STServo::ErrorFlag::OVERHEAT));
    EXPECT_TRUE(resp.hasFlag(STServo::ErrorFlag::OVERLOAD));
    EXPECT_FALSE(resp.hasFlag(STServo::ErrorFlag::VOLTAGE));
    EXPECT_FALSE(resp.hasFlag(STServo::ErrorFlag::INSTRUCTION));
}
