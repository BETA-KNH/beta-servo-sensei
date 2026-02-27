#include <gtest/gtest.h>
#include "servo/STServoResponse.hpp"
#include "servo/STServoRegisters.hpp"

using STServo::ServoResponse;
using STServo::ErrorFlag;

// Doc example:
//   Request:  FF FF 01 04 02 38 02 BE  (READ addr=0x38, len=2 from servo 1)
//   Response: FF FF 01 04 00 18 05 DD
static const std::vector<uint8_t> DOC_RESPONSE = {0xFF, 0xFF, 0x01, 0x04, 0x00, 0x18, 0x05, 0xDD};

// ---------------------------------------------------------------------------
// Parse — structural and checksum validation
// ---------------------------------------------------------------------------

TEST(Parse, DocExampleIsValid)
{
    auto r = ServoResponse::parse(DOC_RESPONSE);
    EXPECT_TRUE(r.valid);
}

TEST(Parse, DocExampleId)
{
    auto r = ServoResponse::parse(DOC_RESPONSE);
    EXPECT_EQ(r.id, 0x01);
}

TEST(Parse, DocExampleErrorByteIsZero)
{
    auto r = ServoResponse::parse(DOC_RESPONSE);
    EXPECT_EQ(r.error, 0x00);
}

TEST(Parse, DocExampleDataBytes)
{
    auto r = ServoResponse::parse(DOC_RESPONSE);
    ASSERT_EQ(r.data.size(), 2u);
    EXPECT_EQ(r.data[0], 0x18);
    EXPECT_EQ(r.data[1], 0x05);
}

TEST(Parse, TooShortReturnsInvalid)
{
    // Only 5 bytes — below minimum of 6
    auto r = ServoResponse::parse({0xFF, 0xFF, 0x01, 0x03, 0x00});
    EXPECT_FALSE(r.valid);
}

TEST(Parse, BadHeaderReturnsInvalid)
{
    auto raw = DOC_RESPONSE;
    raw[0] = 0x00;  // corrupt first header byte
    EXPECT_FALSE(ServoResponse::parse(raw).valid);
}

TEST(Parse, ChecksumMismatchReturnsInvalid)
{
    auto raw = DOC_RESPONSE;
    raw.back() ^= 0xFF;  // flip checksum bits
    EXPECT_FALSE(ServoResponse::parse(raw).valid);
}

TEST(Parse, LenMismatchReturnsInvalid)
{
    auto raw = DOC_RESPONSE;
    raw[3] = 0x07;  // claim more bytes than are present
    EXPECT_FALSE(ServoResponse::parse(raw).valid);
}

TEST(Parse, UnknownRegisterStillParsed)
{
    // A response carrying data from an unrecognised register address is still
    // a structurally valid packet — the parser must not reject it.
    // Build: FF FF 03 05 00 AA BB CC DD <cksum>
    //   id=3, error=0, data={AA BB CC DD}
    std::vector<uint8_t> raw = {0xFF, 0xFF, 0x03, 0x06, 0x00, 0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t sum = 0;
    for (std::size_t i = 2; i < raw.size(); ++i)
        sum = static_cast<uint8_t>(sum + raw[i]);
    raw.push_back(static_cast<uint8_t>(~sum & 0xFF));

    auto r = ServoResponse::parse(raw);
    EXPECT_TRUE(r.valid);
    EXPECT_EQ(r.id, 0x03);
    ASSERT_EQ(r.data.size(), 4u);
    EXPECT_EQ(r.data[0], 0xAA);
    EXPECT_EQ(r.data[3], 0xDD);
}

// ---------------------------------------------------------------------------
// Error flags
// ---------------------------------------------------------------------------

TEST(Error, NoFlagsWhenErrorByteZero)
{
    auto r = ServoResponse::parse(DOC_RESPONSE);
    EXPECT_FALSE(r.hasError());
}

TEST(Error, HasErrorWhenAnyBitSet)
{
    // Build a minimal valid response with error = 0x01
    std::vector<uint8_t> raw = {0xFF, 0xFF, 0x01, 0x02, 0x01};
    uint8_t sum = 0;
    for (std::size_t i = 2; i < raw.size(); ++i)
        sum = static_cast<uint8_t>(sum + raw[i]);
    raw.push_back(static_cast<uint8_t>(~sum & 0xFF));

    auto r = ServoResponse::parse(raw);
    ASSERT_TRUE(r.valid);
    EXPECT_TRUE(r.hasError());
}

TEST(Error, IndividualFlagsDetected)
{
    // Test every ErrorFlag bit individually
    const std::pair<ErrorFlag, std::string> flags[] = {
        {ErrorFlag::VOLTAGE,     "VOLTAGE"},
        {ErrorFlag::ANGLE,       "ANGLE"},
        {ErrorFlag::OVERHEAT,    "OVERHEAT"},
        {ErrorFlag::RANGE,       "RANGE"},
        {ErrorFlag::CHECKSUM,    "CHECKSUM"},
        {ErrorFlag::OVERLOAD,    "OVERLOAD"},
        {ErrorFlag::INSTRUCTION, "INSTRUCTION"},
    };

    for (const auto& [flag, name] : flags) {
        // Build response with only this flag set in the error byte
        std::vector<uint8_t> raw = {0xFF, 0xFF, 0x01, 0x02, static_cast<uint8_t>(flag)};
        uint8_t sum = 0;
        for (std::size_t i = 2; i < raw.size(); ++i)
            sum = static_cast<uint8_t>(sum + raw[i]);
        raw.push_back(static_cast<uint8_t>(~sum & 0xFF));

        auto r = ServoResponse::parse(raw);
        ASSERT_TRUE(r.valid) << "Packet invalid for flag " << name;
        EXPECT_TRUE(r.hasFlag(flag)) << "Expected flag " << name << " to be set";

        // Check no other flags are reported
        for (const auto& [other, otherName] : flags) {
            if (other != flag)
                EXPECT_FALSE(r.hasFlag(other))
                    << "Flag " << otherName << " should not be set when only " << name << " is";
        }
    }
}

// ---------------------------------------------------------------------------
// Data accessors
// ---------------------------------------------------------------------------

TEST(Data, AsUint16DocExample)
{
    // data = {0x18, 0x05}  →  little-endian 0x0518 = 1304
    auto r = ServoResponse::parse(DOC_RESPONSE);
    ASSERT_TRUE(r.valid);
    auto val = r.asUint16();
    ASSERT_TRUE(val.has_value());
    EXPECT_EQ(*val, 0x0518);
}

TEST(Data, AsUint16WithOffset)
{
    // Build response with data = {0x00, 0x01, 0x02, 0x03}
    // asUint16(0) → 0x0100, asUint16(2) → 0x0302
    std::vector<uint8_t> raw = {0xFF, 0xFF, 0x02, 0x06, 0x00, 0x00, 0x01, 0x02, 0x03};
    uint8_t sum = 0;
    for (std::size_t i = 2; i < raw.size(); ++i)
        sum = static_cast<uint8_t>(sum + raw[i]);
    raw.push_back(static_cast<uint8_t>(~sum & 0xFF));

    auto r = ServoResponse::parse(raw);
    ASSERT_TRUE(r.valid);
    EXPECT_EQ(*r.asUint16(0), 0x0100u);
    EXPECT_EQ(*r.asUint16(2), 0x0302u);
}

TEST(Data, AsUint16ReturnsNulloptWhenNotEnoughBytes)
{
    // Response with only 1 data byte — can't form a uint16
    std::vector<uint8_t> raw = {0xFF, 0xFF, 0x01, 0x03, 0x00, 0x42};
    uint8_t sum = 0;
    for (std::size_t i = 2; i < raw.size(); ++i)
        sum = static_cast<uint8_t>(sum + raw[i]);
    raw.push_back(static_cast<uint8_t>(~sum & 0xFF));

    auto r = ServoResponse::parse(raw);
    ASSERT_TRUE(r.valid);
    EXPECT_FALSE(r.asUint16().has_value());
}
