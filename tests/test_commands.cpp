#include <gtest/gtest.h>
#include "servo/STServoRequest.hpp"
#include "servo/STServoRegisters.hpp"

// ---------------------------------------------------------------------------
// Helper: verify structural invariants every well-formed packet must satisfy
// ---------------------------------------------------------------------------
static void assertValidPacket(const std::vector<uint8_t>& pkt)
{
    ASSERT_GE(pkt.size(), 6u);           // [FF FF id len instr cksum] minimum
    EXPECT_EQ(pkt[0], 0xFF);            // header byte 1
    EXPECT_EQ(pkt[1], 0xFF);            // header byte 2
    EXPECT_EQ(pkt[3], pkt.size() - 4); // LEN = bytes after [FF FF id LEN], i.e. total - 4

    // checksum = ~(sum of bytes from ID onward, excluding checksum itself)
    uint8_t sum = 0;
    for (std::size_t i = 2; i < pkt.size() - 1; ++i)
        sum = static_cast<uint8_t>(sum + pkt[i]);
    EXPECT_EQ(pkt.back(), static_cast<uint8_t>(~sum & 0xFF));
}

// ---------------------------------------------------------------------------
// PING
// ---------------------------------------------------------------------------
TEST(Ping, KnownBytes)
{
    EXPECT_EQ(STServoRequest::ping(0x01),
        (std::vector<uint8_t>{0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB}));
}

TEST(Ping, StructurallyValid)
{
    assertValidPacket(STServoRequest::ping(0x01));
}

// ---------------------------------------------------------------------------
// READ  (single ID → READ instruction)
// ---------------------------------------------------------------------------
TEST(Read, SingleIdUsesReadInstruction)
{
    auto pkt = STServoRequest::read(0x01, STServo::Register::TARGET_LOCATION);
    EXPECT_EQ(pkt[4], static_cast<uint8_t>(STServo::Instruction::READ));
    assertValidPacket(pkt);
}

TEST(Read, SingleIdKnownBytes)
{
    // FF FF 01 04 02 2A 02 CC
    EXPECT_EQ(STServoRequest::read(0x01, STServo::Register::TARGET_LOCATION),
        (std::vector<uint8_t>{0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2A, 0x02, 0xCC}));
}

// ---------------------------------------------------------------------------
// SYNC_READ  (multiple IDs → merged overload)
// ---------------------------------------------------------------------------
TEST(Read, MultipleIdsUsesSyncReadInstruction)
{
    auto pkt = STServoRequest::read({0x01, 0x02, 0x03},
                                      STServo::Register::TARGET_LOCATION);
    EXPECT_EQ(pkt[2], 0xFE);  // broadcast ID
    EXPECT_EQ(pkt[4], static_cast<uint8_t>(STServo::Instruction::SYNC_READ));
    assertValidPacket(pkt);
}

TEST(Read, SyncReadContainsAllIds)
{
    std::vector<uint8_t> ids = {0x01, 0x02, 0x03};
    auto pkt = STServoRequest::read(ids, STServo::Register::TARGET_LOCATION);
    // layout: [FF FF FE LEN INSTR ADDR SIZE id1 id2 id3 CKSUM]
    //          0  1  2  3   4     5    6    7   8   9   10
    for (std::size_t i = 0; i < ids.size(); ++i)
        EXPECT_EQ(pkt[7 + i], ids[i]);
}

// ---------------------------------------------------------------------------
// WRITE  (single ID → WRITE instruction)
// ---------------------------------------------------------------------------
TEST(Write, SingleIdUsesWriteInstruction)
{
    auto pkt = STServoRequest::write(0x01, STServo::Register::TARGET_LOCATION,
                                       {0x00, 0x08});
    EXPECT_EQ(pkt[4], static_cast<uint8_t>(STServo::Instruction::WRITE));
    assertValidPacket(pkt);
}

TEST(Write, SingleIdKnownBytes)
{
    // FF FF 01 05 03 2A 00 08 C4
    EXPECT_EQ(STServoRequest::write(0x01, STServo::Register::TARGET_LOCATION,
                                      {0x00, 0x08}),
        (std::vector<uint8_t>{0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0x00, 0x08, 0xC4}));
}

// ---------------------------------------------------------------------------
// SYNC_WRITE  (multiple targets → merged overload)
// ---------------------------------------------------------------------------
TEST(Write, MultipleTargetsUsesSyncWriteInstruction)
{
    auto pkt = STServoRequest::write(STServo::Register::TARGET_LOCATION, {
        {0x01, {0x00, 0x08}},
        {0x02, {0x00, 0x04}},
    });
    EXPECT_EQ(pkt[2], 0xFE);  // broadcast ID
    EXPECT_EQ(pkt[4], static_cast<uint8_t>(STServo::Instruction::SYNC_WRITE));
    assertValidPacket(pkt);
}

TEST(Write, SyncWriteContainsAllTargetData)
{
    auto pkt = STServoRequest::write(STServo::Register::TARGET_LOCATION, {
        {0x01, {0x00, 0x08}},
        {0x02, {0x00, 0x04}},
    });
    // layout: [FF FF FE LEN INSTR ADDR DATA_LEN  id1 d1_lo d1_hi  id2 d2_lo d2_hi  CKSUM]
    //          0  1  2  3   4     5    6          7   8     9      10  11    12      13
    EXPECT_EQ(pkt[7],  0x01);  // servo 1 id
    EXPECT_EQ(pkt[8],  0x00);  // servo 1 data lo
    EXPECT_EQ(pkt[9],  0x08);  // servo 1 data hi
    EXPECT_EQ(pkt[10], 0x02);  // servo 2 id
    EXPECT_EQ(pkt[11], 0x00);  // servo 2 data lo
    EXPECT_EQ(pkt[12], 0x04);  // servo 2 data hi
}

// ---------------------------------------------------------------------------
// REG_WRITE
// ---------------------------------------------------------------------------
TEST(RegWrite, UsesRegWriteInstruction)
{
    auto pkt = STServoRequest::regWrite(0x01, STServo::Register::TARGET_LOCATION,
                                          {0x00, 0x08});
    EXPECT_EQ(pkt[4], static_cast<uint8_t>(STServo::Instruction::REG_WRITE));
    assertValidPacket(pkt);
}

TEST(RegWrite, DiffersFromWriteOnlyInInstructionByte)
{
    auto w  = STServoRequest::write   (0x01, STServo::Register::TARGET_LOCATION, {0x00, 0x08});
    auto rw = STServoRequest::regWrite(0x01, STServo::Register::TARGET_LOCATION, {0x00, 0x08});
    ASSERT_EQ(w.size(), rw.size());
    for (std::size_t i = 0; i < w.size(); ++i) {
        if (i == 4)              EXPECT_NE(w[i], rw[i]);  // instruction byte must differ
        else if (i == w.size()-1) EXPECT_NE(w[i], rw[i]);  // checksum must differ (depends on instruction)
        else                      EXPECT_EQ(w[i], rw[i]);  // all other bytes identical
    }
}

// ---------------------------------------------------------------------------
// ACTION
// ---------------------------------------------------------------------------
TEST(Action, DefaultsBroadcast)
{
    auto pkt = STServoRequest::action();
    EXPECT_EQ(pkt[2], 0xFE);
    EXPECT_EQ(pkt[4], static_cast<uint8_t>(STServo::Instruction::ACTION));
    assertValidPacket(pkt);
}

TEST(Action, SpecificIdOverridesBroadcast)
{
    auto pkt = STServoRequest::action(0x03);
    EXPECT_EQ(pkt[2], 0x03);
    assertValidPacket(pkt);
}

TEST(Action, KnownBytes)
{
    // FF FF FE 02 05 FA
    EXPECT_EQ(STServoRequest::action(),
        (std::vector<uint8_t>{0xFF, 0xFF, 0xFE, 0x02, 0x05, 0xFA}));
}

// ---------------------------------------------------------------------------
// RESET
// ---------------------------------------------------------------------------
TEST(Reset, UsesResetInstruction)
{
    auto pkt = STServoRequest::reset(0x05);
    EXPECT_EQ(pkt[4], static_cast<uint8_t>(STServo::Instruction::RESET));
    EXPECT_EQ(pkt[2], 0x05);
    assertValidPacket(pkt);
}

TEST(Reset, KnownBytes)
{
    // FF FF 05 02 06 F2
    EXPECT_EQ(STServoRequest::reset(0x05),
        (std::vector<uint8_t>{0xFF, 0xFF, 0x05, 0x02, 0x06, 0xF2}));
}

// ---------------------------------------------------------------------------
// WRITE — move golden bytes
// Verified reference packet from ST servo documentation:
//   ID=2  position=1000  time=0  speed=0
//   FF FF 02 09 03 2A E8 03 00 00 00 00 DC
// ---------------------------------------------------------------------------
TEST(Write, MoveGoldenBytes)
{
    // position(2) + time(2) + speed(2) written atomically to TARGET_LOCATION
    std::vector<uint8_t> combined = {
        0xE8, 0x03,  // position 1000 little-endian
        0x00, 0x00,  // time 0
        0x00, 0x00,  // speed 0
    };
    auto pkt = STServoRequest::write(0x02, STServo::Register::TARGET_LOCATION, combined);
    EXPECT_EQ(pkt, (std::vector<uint8_t>{
        0xFF, 0xFF, 0x02, 0x09, 0x03, 0x2A,
        0xE8, 0x03, 0x00, 0x00, 0x00, 0x00,
        0xDC
    }));
    assertValidPacket(pkt);
}
