/**
 * @file test_yarp_driver.cpp
 * @brief Unit and integration tests for STServoYarpDriver.
 *
 * All tests run without physical hardware by using PtyHelper, which creates
 * an in-kernel pseudo-terminal (PTY) pair:
 *
 *   master fd  — held by the test; acts as the simulated servo bus
 *   slave path — handed to STServoYarpDriver::open(); it sees a real tty
 *
 * Pattern
 * -------
 * 1. Pre-stage response bytes via pty_.masterWrite() BEFORE calling the
 *    driver method (they sit in the kernel slave-read buffer).
 * 2. Call the driver method under test.
 * 3. Verify outgoing command packets by pty_.masterRead() from the master.
 *
 * No YARP server / yarpserver process needed — we use driver objects directly.
 */

#include <gtest/gtest.h>

#include <atomic>
#include <cmath>
#include <string>
#include <vector>

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/os/Property.h>
#include <yarp/os/Time.h>

#include "PtyHelper.hpp"
#include "yarp/STServoYarpDriver.hpp"

// ---------------------------------------------------------------------------
// Unit-conversion constants (mirror the private ones in the driver)
// ---------------------------------------------------------------------------
static constexpr double STEPS_PER_DEG = 4095.0 / 360.0;
static constexpr double DEG_PER_STEP  = 360.0  / 4095.0;

// ---------------------------------------------------------------------------
// Protocol helpers — build valid servo response packets
// ---------------------------------------------------------------------------
namespace FakeServo {

/// Build a ping status-OK response for `id` (no error flags).
static std::vector<uint8_t> pingOk(uint8_t id)
{
    // [FF FF id 02 00 cksum]  —  LEN=02 (1 error byte + 1 cksum), ERROR=00
    uint8_t cksum = static_cast<uint8_t>(~(id + 0x02u + 0x00u) & 0xFFu);
    return {0xFF, 0xFF, id, 0x02, 0x00, cksum};
}

/// Build a read-response packet carrying an arbitrary data payload.
static std::vector<uint8_t> readOk(uint8_t id, const std::vector<uint8_t>& data)
{
    const uint8_t len = static_cast<uint8_t>(2u + data.size());
    uint8_t sum = static_cast<uint8_t>(id + len + 0x00u);
    for (auto b : data) sum = static_cast<uint8_t>(sum + b);
    std::vector<uint8_t> pkt = {0xFF, 0xFF, id, len, 0x00};
    pkt.insert(pkt.end(), data.begin(), data.end());
    pkt.push_back(static_cast<uint8_t>(~sum & 0xFFu));
    return pkt;
}

/// Build a read-response carrying a little-endian uint16.
static std::vector<uint8_t> readUint16(uint8_t id, uint16_t value)
{
    return readOk(id, {static_cast<uint8_t>(value & 0xFF),
                       static_cast<uint8_t>((value >> 8) & 0xFF)});
}

/// Build a read-response carrying a single byte.
static std::vector<uint8_t> readByte(uint8_t id, uint8_t value)
{
    return readOk(id, {value});
}

} // namespace FakeServo

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------
class DriverTest : public ::testing::Test {
protected:
    PtyHelper           pty_;
    STServoYarpDriver   drv_;

    // Ordered servo IDs used in multi-joint tests
    std::vector<int> ids2_{1, 2};

    void SetUp() override
    {
        ASSERT_TRUE(pty_.isOpen()) << "PTY unavailable — cannot run tests";
    }

    // -----------------------------------------------------------------------
    // Open helpers
    // -----------------------------------------------------------------------

    /// Build a yarp::os::Property config pointing at the PTY slave.
    yarp::os::Property makeConfig(const std::vector<int>& ids,
                                  int timeoutMs = 50) const
    {
        std::string idStr;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            if (i > 0) idStr += ' ';
            idStr += std::to_string(ids[i]);
        }
        yarp::os::Property cfg;
        cfg.fromString("(port "       + pty_.slavePath()  + ")"
                       " (baud 115200)"
                       " (timeout_ms " + std::to_string(timeoutMs) + ")"
                       " (servo_ids (" + idStr + "))");
        return cfg;
    }

    /// Pre-stage ping responses on the master and open the driver.
    /// Returns true when open() succeeds.
    bool openDriver(const std::vector<int>& ids = {1})
    {
        for (int id : ids)
            EXPECT_TRUE(pty_.masterWrite(FakeServo::pingOk(static_cast<uint8_t>(id))));
        auto cfg = makeConfig(ids);
        return drv_.open(cfg);
    }

    /// Drain exactly `n` bytes from the master without asserting their value.
    void drain(std::size_t n) { pty_.masterRead(n); }

    /// Drain all ping-command bytes left from open() for `ids`.
    /// Each ping is a 6-byte packet.
    void drainOpenPings(const std::vector<int>& ids)
    {
        drain(6u * ids.size());
    }

    /// Compute the expected WRITE packet for a 1-byte register value.
    static std::vector<uint8_t> makeWrite1(uint8_t id, const STServo::Reg& reg,
                                           uint8_t value)
    {
        return STServoRequest::write(reg, id, {value});
    }

    /// Compute the expected WRITE packet for a 2-byte LE register value.
    static std::vector<uint8_t> makeWrite2(uint8_t id, const STServo::Reg& reg,
                                           uint16_t value)
    {
        return STServoRequest::write(reg, id,
                                     {static_cast<uint8_t>(value & 0xFF),
                                      static_cast<uint8_t>((value >> 8) & 0xFF)});
    }

    /// Packet size for a single-servo WRITE of `regSize` data bytes.
    static std::size_t writePacketSize(uint8_t regSize)
    {
        // [FF FF id LEN INSTR addr data{regSize} cksum]
        // LEN = 1(INSTR) + 1(addr) + regSize + 1(cksum) = regSize + 3
        return static_cast<std::size_t>(5u + 1u + regSize + 1u);
    }

    /// Move-packet size (7-byte composite ACCELERATION→OPERATION_SPEED).
    static constexpr std::size_t MOVE_PACKET_SIZE = 14u;  // 5(hdr)+1(addr)+7(data)+1(cksum)
};

// ===========================================================================
// Open / Close
// ===========================================================================

TEST_F(DriverTest, OpenSucceedsWithValidPTY)
{
    EXPECT_TRUE(openDriver({1}));
}

TEST_F(DriverTest, OpenFailsMissingPort)
{
    yarp::os::Property cfg;
    cfg.fromString("(baud 115200) (servo_ids (1))");
    EXPECT_FALSE(drv_.open(cfg));
}

TEST_F(DriverTest, OpenFailsMissingBaud)
{
    yarp::os::Property cfg;
    cfg.fromString("(port " + pty_.slavePath() + ") (servo_ids (1))");
    EXPECT_FALSE(drv_.open(cfg));
}

TEST_F(DriverTest, OpenFailsMissingServoIds)
{
    yarp::os::Property cfg;
    cfg.fromString("(port " + pty_.slavePath() + ") (baud 115200)");
    EXPECT_FALSE(drv_.open(cfg));
}

TEST_F(DriverTest, OpenFailsNonexistentPort)
{
    yarp::os::Property cfg;
    cfg.fromString("(port /dev/servo_no_exist) (baud 115200) (servo_ids (1))");
    EXPECT_FALSE(drv_.open(cfg));
}

TEST_F(DriverTest, CloseSucceedsAfterOpen)
{
    // close() will disableAmp for each joint (TORQUE_SWITCH=0 write packet)
    // so pre-stage the open pings and be ready to absorb the close packets
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});
    EXPECT_TRUE(drv_.close());
}

// ===========================================================================
// getAxes
// ===========================================================================

TEST_F(DriverTest, GetAxesReturnsSingleJoint)
{
    ASSERT_TRUE(openDriver({1}));
    int ax = -1;
    EXPECT_TRUE(drv_.getAxes(&ax));
    EXPECT_EQ(ax, 1);
}

TEST_F(DriverTest, GetAxesReturnsCorrectMultiple)
{
    ASSERT_TRUE(openDriver({1, 2, 3, 4}));
    int ax = -1;
    EXPECT_TRUE(drv_.getAxes(&ax));
    EXPECT_EQ(ax, 4);
}

TEST_F(DriverTest, GetAxesNullPtrReturnsFalse)
{
    ASSERT_TRUE(openDriver({1}));
    EXPECT_FALSE(drv_.getAxes(nullptr));
}

// ===========================================================================
// Reference speed (pure state — no serial traffic)
// ===========================================================================

TEST_F(DriverTest, DefaultRefSpeedIs60DegS)
{
    ASSERT_TRUE(openDriver({1}));
    double sp = -1.0;
    EXPECT_TRUE(drv_.getRefSpeed(0, &sp));
    EXPECT_DOUBLE_EQ(sp, 60.0);
}

TEST_F(DriverTest, SetGetRefSpeedRoundTrip)
{
    ASSERT_TRUE(openDriver({1, 2}));
    EXPECT_TRUE(drv_.setRefSpeed(0, 45.0));
    EXPECT_TRUE(drv_.setRefSpeed(1, 120.0));
    double sp0 = 0.0, sp1 = 0.0;
    EXPECT_TRUE(drv_.getRefSpeed(0, &sp0));
    EXPECT_TRUE(drv_.getRefSpeed(1, &sp1));
    EXPECT_DOUBLE_EQ(sp0, 45.0);
    EXPECT_DOUBLE_EQ(sp1, 120.0);
}

TEST_F(DriverTest, SetRefSpeedClampNegativeToZero)
{
    ASSERT_TRUE(openDriver({1}));
    EXPECT_TRUE(drv_.setRefSpeed(0, -5.0));
    double sp = 99.0;
    drv_.getRefSpeed(0, &sp);
    EXPECT_DOUBLE_EQ(sp, 0.0);
}

TEST_F(DriverTest, SetRefSpeedsAllJoints)
{
    ASSERT_TRUE(openDriver({1, 2}));
    const double spds[2] = {30.0, 90.0};
    EXPECT_TRUE(drv_.setRefSpeeds(spds));
    double out[2] = {};
    EXPECT_TRUE(drv_.getRefSpeeds(out));
    EXPECT_DOUBLE_EQ(out[0], 30.0);
    EXPECT_DOUBLE_EQ(out[1], 90.0);
}

TEST_F(DriverTest, RefSpeedOutOfRangeJointReturnsFalse)
{
    ASSERT_TRUE(openDriver({1}));
    EXPECT_FALSE(drv_.setRefSpeed(5, 10.0));
    double sp = 0.0;
    EXPECT_FALSE(drv_.getRefSpeed(-1, &sp));
}

// ===========================================================================
// Reference acceleration (pure state)
// ===========================================================================

TEST_F(DriverTest, DefaultAccelerationIsZero)
{
    ASSERT_TRUE(openDriver({1}));
    double acc = -1.0;
    EXPECT_TRUE(drv_.getRefAcceleration(0, &acc));
    EXPECT_DOUBLE_EQ(acc, 0.0);
}

TEST_F(DriverTest, SetGetRefAccelerationRoundTrip)
{
    ASSERT_TRUE(openDriver({1, 2}));
    const double accs[2] = {200.0, 500.0};
    EXPECT_TRUE(drv_.setRefAccelerations(accs));
    double out[2] = {};
    EXPECT_TRUE(drv_.getRefAccelerations(out));
    EXPECT_DOUBLE_EQ(out[0], 200.0);
    EXPECT_DOUBLE_EQ(out[1], 500.0);
}

// ===========================================================================
// Control mode (pure state — setControlMode for IDLE tested separately below)
// ===========================================================================

TEST_F(DriverTest, DefaultControlModeIsPosition)
{
    ASSERT_TRUE(openDriver({1}));
    int mode = -1;
    EXPECT_TRUE(drv_.getControlMode(0, &mode));
    EXPECT_EQ(mode, VOCAB_CM_POSITION);
}

TEST_F(DriverTest, GetControlModesAllJoints)
{
    ASSERT_TRUE(openDriver({1, 2}));
    int modes[2] = {-1, -1};
    EXPECT_TRUE(drv_.getControlModes(modes));
    EXPECT_EQ(modes[0], VOCAB_CM_POSITION);
    EXPECT_EQ(modes[1], VOCAB_CM_POSITION);
}

// ===========================================================================
// Reference torque (pure state)
// ===========================================================================

TEST_F(DriverTest, DefaultRefTorqueIsHundredPercent)
{
    ASSERT_TRUE(openDriver({1}));
    double t = -1.0;
    EXPECT_TRUE(drv_.getRefTorque(0, &t));
    EXPECT_DOUBLE_EQ(t, 100.0);
}

// ===========================================================================
// enableAmp / disableAmp — verify TORQUE_SWITCH packets
// ===========================================================================

TEST_F(DriverTest, EnableAmpSendsTorqueSwitchOn)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    EXPECT_TRUE(drv_.enableAmp(0));

    auto expected = makeWrite1(1, STServo::Register::TORQUE_SWITCH, 1);
    auto actual   = pty_.masterRead(expected.size());
    EXPECT_EQ(actual, expected);
}

TEST_F(DriverTest, DisableAmpSendsTorqueSwitchOff)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    EXPECT_TRUE(drv_.disableAmp(0));

    auto expected = makeWrite1(1, STServo::Register::TORQUE_SWITCH, 0);
    auto actual   = pty_.masterRead(expected.size());
    EXPECT_EQ(actual, expected);
}

TEST_F(DriverTest, EnableAmpInvalidJointReturnsFalse)
{
    ASSERT_TRUE(openDriver({1}));
    EXPECT_FALSE(drv_.enableAmp(5));
}

// ===========================================================================
// stop(j) — verify OPERATION_SPEED = 0 packet
// ===========================================================================

TEST_F(DriverTest, StopSingleJointSendsZeroSpeed)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    EXPECT_TRUE(drv_.stop(0));

    auto expected = makeWrite2(1, STServo::Register::OPERATION_SPEED, 0);
    auto actual   = pty_.masterRead(expected.size());
    EXPECT_EQ(actual, expected);
}

// ===========================================================================
// positionMove — verify TARGET_LOCATION step encoding
// ===========================================================================

TEST_F(DriverTest, PositionMoveEncodesStepsCorrectly)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // Move to 90 degrees
    constexpr double DEG = 90.0;
    EXPECT_TRUE(drv_.positionMove(0, DEG));

    // Expected steps
    int16_t  steps  = static_cast<int16_t>(std::round(DEG * STEPS_PER_DEG));
    uint16_t speed  = static_cast<uint16_t>(std::round(60.0 * STEPS_PER_DEG)); // default 60 deg/s

    // Expected packet: WRITE of 7-byte composite register at 0x29
    // Payload: ACCELERATION(1) | TARGET_LOCATION(2) | OPERATION_TIME(2) | OPERATION_SPEED(2)
    static constexpr STServo::Reg REG_MOVE = {0x29, 7, STServo::MemoryArea::SRAM, STServo::Access::RW};
    std::vector<uint8_t> payload = {
        0x00,  // ACCELERATION = 0 (servo max, default refAcceleration)
        static_cast<uint8_t>(steps  & 0xFF),
        static_cast<uint8_t>((steps  >> 8) & 0xFF),
        0x00, 0x00,  // OPERATION_TIME always 0
        static_cast<uint8_t>(speed & 0xFF),
        static_cast<uint8_t>((speed >> 8) & 0xFF),
    };
    auto expected = STServoRequest::write(REG_MOVE, 1, payload);
    auto actual   = pty_.masterRead(expected.size());
    EXPECT_EQ(actual, expected);
}

TEST_F(DriverTest, PositionMoveUsesSetRefSpeed)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    drv_.setRefSpeed(0, 120.0);  // override default
    drv_.positionMove(0, 45.0);

    auto pkt = pty_.masterRead(MOVE_PACKET_SIZE);
    ASSERT_EQ(pkt.size(), MOVE_PACKET_SIZE);

    // Speed bytes are at offsets [10] and [11] in the move packet:
    //  [FF FF id LEN INSTR 0x29 acc d0 d1 d2 d3 d4 d5 cksum]
    //  0  1  2  3   4     5    6   7  8  9  10 11 12 13
    //  acc = ACCELERATION(1), d0..d1 = TARGET_LOCATION, d2..d3 = OPERATION_TIME=0, d4..d5 = OPERATION_SPEED
    uint16_t speedInPkt = static_cast<uint16_t>(pkt[11] | (pkt[12] << 8));
    uint16_t expected   = static_cast<uint16_t>(std::round(120.0 * STEPS_PER_DEG));
    EXPECT_EQ(speedInPkt, expected);
}

TEST_F(DriverTest, PositionMoveAllJointsSyncWrite)
{
    ASSERT_TRUE(openDriver({1, 2}));
    drainOpenPings({1, 2});

    const double refs[2] = {45.0, 135.0};
    EXPECT_TRUE(drv_.positionMove(refs));

    // SYNC_WRITE for 2 servos, 7-byte register:
    // params = [addr(1) + size(1) + {id(1)+data(7)} × 2] = 2 + 16 = 18 bytes
    // total  = 6 + 18 = 24 bytes:  [FF FF FE LEN 83 params CKSUM]
    constexpr std::size_t SYNC_MOVE_2_SIZE = 24u;
    auto pkt = pty_.masterRead(SYNC_MOVE_2_SIZE);
    ASSERT_EQ(pkt.size(), SYNC_MOVE_2_SIZE);

    // Verify header
    EXPECT_EQ(pkt[0], 0xFF);
    EXPECT_EQ(pkt[1], 0xFF);
    EXPECT_EQ(pkt[2], 0xFE);  // broadcast
    EXPECT_EQ(pkt[4], static_cast<uint8_t>(STServo::Instruction::SYNC_WRITE));
    EXPECT_EQ(pkt[5], static_cast<uint8_t>(0x29));  // ACCELERATION register (start of 7-byte block)
}

TEST_F(DriverTest, PositionMoveInvalidJointReturnsFalse)
{
    ASSERT_TRUE(openDriver({1}));
    EXPECT_FALSE(drv_.positionMove(-1, 45.0));
    EXPECT_FALSE(drv_.positionMove(10, 45.0));
}

// ===========================================================================
// getEncoder — inject CURRENT_LOCATION response and verify degree conversion
// ===========================================================================

TEST_F(DriverTest, GetEncoderDecodesToDegrees)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // Servo reports 2048 steps (≈ 180 deg)
    constexpr uint16_t STEPS = 2048;
    pty_.masterWrite(FakeServo::readUint16(1, STEPS));

    double enc = -999.0;
    EXPECT_TRUE(drv_.getEncoder(0, &enc));
    EXPECT_NEAR(enc, STEPS * DEG_PER_STEP, 0.01);
}

TEST_F(DriverTest, GetEncoderZeroStepsIsZeroDeg)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    pty_.masterWrite(FakeServo::readUint16(1, 0));
    double enc = -999.0;
    EXPECT_TRUE(drv_.getEncoder(0, &enc));
    EXPECT_NEAR(enc, 0.0, 1e-9);
}

TEST_F(DriverTest, GetEncodersAllJoints)
{
    ASSERT_TRUE(openDriver({1, 2}));
    drainOpenPings({1, 2});

    // Pre-stage sync-read responses (one per servo in ID order)
    constexpr uint16_t STEPS1 = 1024;
    constexpr uint16_t STEPS2 = 3000;
    pty_.masterWrite(FakeServo::readUint16(1, STEPS1));
    pty_.masterWrite(FakeServo::readUint16(2, STEPS2));

    double encs[2] = {-1.0, -1.0};
    EXPECT_TRUE(drv_.getEncoders(encs));
    EXPECT_NEAR(encs[0], STEPS1 * DEG_PER_STEP, 0.01);
    EXPECT_NEAR(encs[1], STEPS2 * DEG_PER_STEP, 0.01);
}

TEST_F(DriverTest, GetEncoderNullPtrReturnsFalse)
{
    ASSERT_TRUE(openDriver({1}));
    EXPECT_FALSE(drv_.getEncoder(0, nullptr));
}

// ===========================================================================
// getEncoderSpeed — inject CURRENT_SPEED, verify deg/s conversion
// ===========================================================================

TEST_F(DriverTest, GetEncoderSpeedConvertsToDegS)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    constexpr uint16_t RAW_SPEED = 512;
    pty_.masterWrite(FakeServo::readUint16(1, RAW_SPEED));

    double sp = -99.0;
    EXPECT_TRUE(drv_.getEncoderSpeed(0, &sp));
    EXPECT_NEAR(sp, RAW_SPEED * DEG_PER_STEP, 0.01);
}

// ===========================================================================
// checkMotionDone — inject MOVE_FLAG responses
// ===========================================================================

TEST_F(DriverTest, CheckMotionDoneWhenStopped)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    pty_.masterWrite(FakeServo::readByte(1, 0));  // 0 = stopped

    bool done = false;
    EXPECT_TRUE(drv_.checkMotionDone(0, &done));
    EXPECT_TRUE(done);
}

TEST_F(DriverTest, CheckMotionDoneWhenMoving)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    pty_.masterWrite(FakeServo::readByte(1, 1));  // 1 = moving

    bool done = true;
    EXPECT_TRUE(drv_.checkMotionDone(0, &done));
    EXPECT_FALSE(done);
}

TEST_F(DriverTest, CheckMotionDoneAllJoints)
{
    ASSERT_TRUE(openDriver({1, 2}));
    drainOpenPings({1, 2});

    pty_.masterWrite(FakeServo::readByte(1, 0));
    pty_.masterWrite(FakeServo::readByte(2, 0));

    bool done = false;
    EXPECT_TRUE(drv_.checkMotionDone(&done));
    EXPECT_TRUE(done);
}

TEST_F(DriverTest, CheckMotionDoneAnyMovingReturnsFalse)
{
    ASSERT_TRUE(openDriver({1, 2}));
    drainOpenPings({1, 2});

    pty_.masterWrite(FakeServo::readByte(1, 0));
    pty_.masterWrite(FakeServo::readByte(2, 1));  // joint 1 still moving

    bool done = true;
    EXPECT_TRUE(drv_.checkMotionDone(&done));
    EXPECT_FALSE(done);
}

// ===========================================================================
// getCurrent / getCurrents — inject CURRENT_CURRENT register
// ===========================================================================

TEST_F(DriverTest, GetCurrentConvertsToAmps)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // 100 counts × 6.5 mA = 650 mA = 0.65 A
    constexpr uint16_t COUNTS = 100;
    pty_.masterWrite(FakeServo::readUint16(1, COUNTS));

    double amps = 0.0;
    EXPECT_TRUE(drv_.getCurrent(0, &amps));
    EXPECT_NEAR(amps, COUNTS * 0.0065, 1e-6);
}

TEST_F(DriverTest, GetCurrentsAllJoints)
{
    ASSERT_TRUE(openDriver({1, 2}));
    drainOpenPings({1, 2});

    pty_.masterWrite(FakeServo::readUint16(1, 200));
    pty_.masterWrite(FakeServo::readUint16(2, 50));

    double amps[2] = {};
    EXPECT_TRUE(drv_.getCurrents(amps));
    EXPECT_NEAR(amps[0], 200 * 0.0065, 1e-6);
    EXPECT_NEAR(amps[1],  50 * 0.0065, 1e-6);
}

// ===========================================================================
// getPowerSupplyVoltage — inject CURRENT_VOLTAGE (0.1 V / count)
// ===========================================================================

TEST_F(DriverTest, GetPowerSupplyVoltage)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // 120 counts × 0.1 V = 12.0 V
    pty_.masterWrite(FakeServo::readByte(1, 120));

    double v = 0.0;
    EXPECT_TRUE(drv_.getPowerSupplyVoltage(0, &v));
    EXPECT_NEAR(v, 12.0, 1e-6);
}

// ===========================================================================
// getTorque — inject CURRENT_LOAD (0.1 % / count)
// ===========================================================================

TEST_F(DriverTest, GetTorqueProxyFromCurrentLoad)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // 500 counts × 0.1 % = 50.0 %
    pty_.masterWrite(FakeServo::readUint16(1, 500));

    double t = -1.0;
    EXPECT_TRUE(drv_.getTorque(0, &t));
    EXPECT_NEAR(t, 50.0, 0.01);
}

// ===========================================================================
// setRefTorque — verify TORQUE_LIMIT packet
// ===========================================================================

TEST_F(DriverTest, SetRefTorqueSendsTorqueLimitRegister)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // 50 % → raw = 50.0 × 10 = 500 counts
    EXPECT_TRUE(drv_.setRefTorque(0, 50.0));

    auto expected = makeWrite2(1, STServo::Register::TORQUE_LIMIT, 500);
    auto actual   = pty_.masterRead(expected.size());
    EXPECT_EQ(actual, expected);
}

TEST_F(DriverTest, SetRefTorqueClampsAbove100)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    drv_.setRefTorque(0, 150.0);

    // Should clamp to 100 % → 1000 counts
    auto expected = makeWrite2(1, STServo::Register::TORQUE_LIMIT, 1000);
    auto actual   = pty_.masterRead(expected.size());
    EXPECT_EQ(actual, expected);
}

// ===========================================================================
// setControlMode — verify OPERATION_MODE register write
// ===========================================================================

TEST_F(DriverTest, SetControlModePositionWritesMode0)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    EXPECT_TRUE(drv_.setControlMode(0, VOCAB_CM_POSITION));

    // writeOperationMode sends 3 write packets:
    //   LOCK_FLAG=0 (8 bytes), OPERATION_MODE=0 (8 bytes), LOCK_FLAG=1 (8 bytes)
    constexpr std::size_t WMODE_TOTAL = 3u * 8u;
    auto pkt = pty_.masterRead(WMODE_TOTAL);
    ASSERT_EQ(pkt.size(), WMODE_TOTAL);

    // Second packet: bytes 8-15 — check OPERATION_MODE address and value
    EXPECT_EQ(pkt[ 8 + 5], STServo::Register::OPERATION_MODE.address); // addr byte
    EXPECT_EQ(pkt[ 8 + 6], 0u);                                         // mode = 0
}

TEST_F(DriverTest, SetControlModeVelocityWritesMode1)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    EXPECT_TRUE(drv_.setControlMode(0, VOCAB_CM_VELOCITY));
    auto pkt = pty_.masterRead(24u);
    ASSERT_EQ(pkt.size(), 24u);

    EXPECT_EQ(pkt[8 + 5], STServo::Register::OPERATION_MODE.address);
    EXPECT_EQ(pkt[8 + 6], 1u);  // mode = 1 (constant-speed)
}

TEST_F(DriverTest, SetControlModeIdleDisablesTorque)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    EXPECT_TRUE(drv_.setControlMode(0, VOCAB_CM_IDLE));

    // VOCAB_CM_IDLE → writeByte(TORQUE_SWITCH, 0)
    auto expected = makeWrite1(1, STServo::Register::TORQUE_SWITCH, 0);
    auto actual   = pty_.masterRead(expected.size());
    EXPECT_EQ(actual, expected);
}

TEST_F(DriverTest, SetControlModeUpdatesCache)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    drv_.setControlMode(0, VOCAB_CM_IDLE);
    drain(8u);  // consume the TORQUE_SWITCH packet

    int mode = -1;
    drv_.getControlMode(0, &mode);
    EXPECT_EQ(mode, VOCAB_CM_IDLE);
}

TEST_F(DriverTest, SetControlModeUnsupportedReturnsFalse)
{
    ASSERT_TRUE(openDriver({1}));
    EXPECT_FALSE(drv_.setControlMode(0, 0xDEAD));
}

// ===========================================================================
// getPid — inject P/I/D register values and verify Pid struct
// ===========================================================================

TEST_F(DriverTest, GetPositionPidReadsThreeRegisters)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // Inject P=32, I=5, D=16 (one byte each, 3 individual reads)
    pty_.masterWrite(FakeServo::readByte(1, 32));  // P_COEFF
    pty_.masterWrite(FakeServo::readByte(1,  5));  // I_COEFF
    pty_.masterWrite(FakeServo::readByte(1, 16));  // D_COEFF

    yarp::dev::Pid pid{};
    EXPECT_TRUE(drv_.getPid(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &pid));
    EXPECT_DOUBLE_EQ(pid.kp, 32.0);
    EXPECT_DOUBLE_EQ(pid.ki,  5.0);
    EXPECT_DOUBLE_EQ(pid.kd, 16.0);
}

TEST_F(DriverTest, GetVelocityPidReadsTwoRegisters)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    pty_.masterWrite(FakeServo::readByte(1, 10));  // SPEED_P_COEFF
    pty_.masterWrite(FakeServo::readByte(1, 20));  // VELOCITY_I_COEFF

    yarp::dev::Pid pid{};
    EXPECT_TRUE(drv_.getPid(yarp::dev::VOCAB_PIDTYPE_VELOCITY, 0, &pid));
    EXPECT_DOUBLE_EQ(pid.kp, 10.0);
    EXPECT_DOUBLE_EQ(pid.ki, 20.0);
    EXPECT_DOUBLE_EQ(pid.kd,  0.0);  // no D term in velocity loop
}

// ===========================================================================
// setPid — verify correct register write packets
// ===========================================================================

TEST_F(DriverTest, SetPositionPidWritesCorrectRegisters)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    yarp::dev::Pid pid{};
    pid.kp = 40.0;
    pid.ki =  8.0;
    pid.kd = 20.0;
    EXPECT_TRUE(drv_.setPid(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, pid));

    // Expect 3 consecutive single-byte WRITE packets (P, I, D)
    auto expP = makeWrite1(1, STServo::Register::P_COEFF, 40);
    auto expI = makeWrite1(1, STServo::Register::I_COEFF,  8);
    auto expD = makeWrite1(1, STServo::Register::D_COEFF, 20);

    auto pP = pty_.masterRead(expP.size());
    auto pI = pty_.masterRead(expI.size());
    auto pD = pty_.masterRead(expD.size());

    EXPECT_EQ(pP, expP);
    EXPECT_EQ(pI, expI);
    EXPECT_EQ(pD, expD);
}

TEST_F(DriverTest, SetPidClampsCoefficientsTo254)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    yarp::dev::Pid pid{};
    pid.kp = 999.0;  // way above 254
    pid.ki = 0.0;
    pid.kd = 0.0;
    EXPECT_TRUE(drv_.setPid(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, pid));

    auto pP = pty_.masterRead(8u);
    ASSERT_EQ(pP.size(), 8u);
    // Data byte is at index 6
    EXPECT_EQ(pP[6], 254u);  // clamped
}

// ===========================================================================
// IMotorEncoders delegating to IEncoders
// ===========================================================================

TEST_F(DriverTest, GetMotorEncoderCountsPerRev)
{
    ASSERT_TRUE(openDriver({1}));
    double cpr = 0.0;
    EXPECT_TRUE(drv_.getMotorEncoderCountsPerRevolution(0, &cpr));
    EXPECT_DOUBLE_EQ(cpr, 4096.0);
}

TEST_F(DriverTest, GetNumberOfMotorEncoders)
{
    ASSERT_TRUE(openDriver({1, 2, 3}));
    int n = 0;
    EXPECT_TRUE(drv_.getNumberOfMotorEncoders(&n));
    EXPECT_EQ(n, 3);
}

TEST_F(DriverTest, GetMotorEncoderDelegatestoGetEncoder)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    pty_.masterWrite(FakeServo::readUint16(1, 1024));

    double enc = -1.0;
    EXPECT_TRUE(drv_.getMotorEncoder(0, &enc));
    EXPECT_NEAR(enc, 1024 * DEG_PER_STEP, 0.01);
}

// ===========================================================================
// resetEncoder / setEncoder — software offset
// ===========================================================================

TEST_F(DriverTest, ResetEncoderOffsetsByHWPosition)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // Servo is at position 1024 steps; after reset, reading that same position
    // should report 0 degrees
    pty_.masterWrite(FakeServo::readUint16(1, 1024));
    EXPECT_TRUE(drv_.resetEncoder(0));
    drain(8u);  // drain the READ packet sent for resetEncoder

    pty_.masterWrite(FakeServo::readUint16(1, 1024));
    double enc = -99.0;
    EXPECT_TRUE(drv_.getEncoder(0, &enc));
    EXPECT_NEAR(enc, 0.0, 0.01);
}

TEST_F(DriverTest, SetEncoderAssignsLogicalZero)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // HW position is 2048 steps; declare this as 90 degrees
    pty_.masterWrite(FakeServo::readUint16(1, 2048));
    EXPECT_TRUE(drv_.setEncoder(0, 90.0));
    drain(8u);

    // Reading same HW position should return 90 degrees
    pty_.masterWrite(FakeServo::readUint16(1, 2048));
    double enc = -99.0;
    EXPECT_TRUE(drv_.getEncoder(0, &enc));
    EXPECT_NEAR(enc, 90.0, 0.01);
}

// ===========================================================================
// PID enabled state
// ===========================================================================

TEST_F(DriverTest, PidEnabledByDefault)
{
    ASSERT_TRUE(openDriver({1}));
    bool enabled = false;
    EXPECT_TRUE(drv_.isPidEnabled(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &enabled));
    EXPECT_TRUE(enabled);
}

TEST_F(DriverTest, DisablePidSetsStateAndDisablesAmp)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    EXPECT_TRUE(drv_.disablePid(yarp::dev::VOCAB_PIDTYPE_POSITION, 0));
    drain(8u);  // consume TORQUE_SWITCH packet

    bool enabled = true;
    drv_.isPidEnabled(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &enabled);
    EXPECT_FALSE(enabled);
}

TEST_F(DriverTest, EnablePidRestoresState)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    drv_.disablePid(yarp::dev::VOCAB_PIDTYPE_POSITION, 0);
    drain(8u);
    drv_.enablePid(yarp::dev::VOCAB_PIDTYPE_POSITION, 0);
    drain(8u);

    bool enabled = false;
    drv_.isPidEnabled(yarp::dev::VOCAB_PIDTYPE_POSITION, 0, &enabled);
    EXPECT_TRUE(enabled);
}

// ===========================================================================
// relativeMove — reads encoder, adds delta, sends move
// ===========================================================================

TEST_F(DriverTest, RelativeMoveAddsToCurrentPosition)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // Encoder at 1024 steps → ~90 deg; move +45 deg → target ~135 deg
    constexpr uint16_t CURRENT_STEPS = 1024;
    pty_.masterWrite(FakeServo::readUint16(1, CURRENT_STEPS));

    constexpr double DELTA = 45.0;
    EXPECT_TRUE(drv_.relativeMove(0, DELTA));

    drain(8u);   // drain the getEncoder READ packet
    auto movePkt = pty_.masterRead(MOVE_PACKET_SIZE);
    ASSERT_EQ(movePkt.size(), MOVE_PACKET_SIZE);

    // Target position bytes [7],[8] in the move packet (after acc byte at [6]):
    //  [FF FF id LEN INSTR 0x29 acc tgt_lo tgt_hi time_lo time_hi spd_lo spd_hi cksum]
    //  0  1  2  3   4     5    6   7      8      9      10     11     12     13
    int16_t targetSteps = static_cast<int16_t>(movePkt[7] | (movePkt[8] << 8));
    double  targetDeg   = targetSteps * DEG_PER_STEP;
    double  expectedDeg = CURRENT_STEPS * DEG_PER_STEP + DELTA;
    EXPECT_NEAR(targetDeg, expectedDeg, 0.5);  // 0.5 deg rounding tolerance
}

// ===========================================================================
// Interface coverage: IPositionControl/IVelocityControl/IInteractionMode
// ===========================================================================

TEST_F(DriverTest, GetTargetPositionReturnsLastCommandedReference)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    constexpr double TARGET = 123.0;
    ASSERT_TRUE(drv_.positionMove(0, TARGET));
    drain(MOVE_PACKET_SIZE);

    double ref = 0.0;
    EXPECT_TRUE(drv_.getTargetPosition(0, &ref));
    EXPECT_DOUBLE_EQ(ref, TARGET);
}

TEST_F(DriverTest, GetTargetPositionsAllJointsReturnsCachedTargets)
{
    ASSERT_TRUE(openDriver({1, 2}));
    drainOpenPings({1, 2});

    const double refsCmd[2] = {10.0, 20.0};
    ASSERT_TRUE(drv_.positionMove(refsCmd));

    // SYNC_WRITE for 2 servos with 7-byte register payload: total 24 bytes.
    drain(24u);

    double refsOut[2] = {-1.0, -1.0};
    EXPECT_TRUE(drv_.getTargetPositions(refsOut));
    EXPECT_DOUBLE_EQ(refsOut[0], refsCmd[0]);
    EXPECT_DOUBLE_EQ(refsOut[1], refsCmd[1]);
}

TEST_F(DriverTest, GetRefVelocityMirrorsSetRefSpeed)
{
    ASSERT_TRUE(openDriver({1, 2}));

    ASSERT_TRUE(drv_.setRefSpeed(0, 75.0));
    ASSERT_TRUE(drv_.setRefSpeed(1, 33.0));

    double v0 = 0.0;
    double v1 = 0.0;
    EXPECT_TRUE(drv_.getRefVelocity(0, &v0));
    EXPECT_TRUE(drv_.getRefVelocity(1, &v1));
    EXPECT_DOUBLE_EQ(v0, 75.0);
    EXPECT_DOUBLE_EQ(v1, 33.0);

    double all[2] = {0.0, 0.0};
    EXPECT_TRUE(drv_.getRefVelocities(all));
    EXPECT_DOUBLE_EQ(all[0], 75.0);
    EXPECT_DOUBLE_EQ(all[1], 33.0);
}

TEST_F(DriverTest, InteractionModesAreAlwaysStiff)
{
    ASSERT_TRUE(openDriver({1, 2, 3}));

    yarp::dev::InteractionModeEnum mode = yarp::dev::VOCAB_IM_UNKNOWN;
    EXPECT_TRUE(drv_.getInteractionMode(1, &mode));
    EXPECT_EQ(mode, yarp::dev::VOCAB_IM_STIFF);

    yarp::dev::InteractionModeEnum modes[3] = {
        yarp::dev::VOCAB_IM_UNKNOWN,
        yarp::dev::VOCAB_IM_UNKNOWN,
        yarp::dev::VOCAB_IM_UNKNOWN,
    };
    EXPECT_TRUE(drv_.getInteractionModes(modes));
    EXPECT_EQ(modes[0], yarp::dev::VOCAB_IM_STIFF);
    EXPECT_EQ(modes[1], yarp::dev::VOCAB_IM_STIFF);
    EXPECT_EQ(modes[2], yarp::dev::VOCAB_IM_STIFF);
}

TEST_F(DriverTest, SetInteractionModeAcceptsOnlyStiffButReturnsTrue)
{
    ASSERT_TRUE(openDriver({1}));

    // Driver is stiff-only; non-stiff requests are ignored but should not fail.
    EXPECT_TRUE(drv_.setInteractionMode(0, yarp::dev::VOCAB_IM_COMPLIANT));

    yarp::dev::InteractionModeEnum mode = yarp::dev::VOCAB_IM_UNKNOWN;
    EXPECT_TRUE(drv_.getInteractionMode(0, &mode));
    EXPECT_EQ(mode, yarp::dev::VOCAB_IM_STIFF);
}

// ===========================================================================
// Amp status — inject SERVO_STATUS byte
// ===========================================================================

TEST_F(DriverTest, GetAmpStatusSingleJoint)
{
    ASSERT_TRUE(openDriver({1}));
    drainOpenPings({1});

    // Report OVERHEAT + OVERLOAD flags in SERVO_STATUS
    constexpr uint8_t STATUS = static_cast<uint8_t>(STServo::ErrorFlag::OVERHEAT) |
                               static_cast<uint8_t>(STServo::ErrorFlag::OVERLOAD);
    pty_.masterWrite(FakeServo::readByte(1, STATUS));

    int st = 0;
    EXPECT_TRUE(drv_.getAmpStatus(0, &st));
    EXPECT_EQ(static_cast<uint8_t>(st), STATUS);
}

// ---------------------------------------------------------------------------
// Custom main — no YARP network needed (driver is tested in-process)
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // Point YARP's time subsystem at the real wall clock so that
    // Time::now() calls in the driver (acceleration estimation, timestamps)
    // work without a full yarpserver / Network::init().
    yarp::os::Time::useSystemClock();
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
