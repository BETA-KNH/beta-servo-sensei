#include <cstdio>
#include "servo/STServoRequest.hpp"
#include "servo/STServoRegisters.hpp"

static void printPacket(const char* label, const std::vector<uint8_t>& pkt)
{
    std::printf("%-14s:", label);
    for (uint8_t b : pkt)
        std::printf(" %02X", b);
    std::printf("\n");
}

int main()
{
    // PING servo 1
    printPacket("ping(1)",
        STServoRequest::ping(0x01));

    // READ target position from servo 1
    printPacket("read(1,pos)",
        STServoRequest::read(0x01, STServo::Register::TARGET_LOCATION));

    // SYNC_READ target position from servos 1, 2, 3
    printPacket("read({1,2,3})",
        STServoRequest::read({0x01, 0x02, 0x03}, STServo::Register::TARGET_LOCATION));

    // WRITE target position 2048 (0x0800) to servo 1  (2-byte LE)
    printPacket("write(1,pos)",
        STServoRequest::write(0x01, STServo::Register::TARGET_LOCATION, {0x00, 0x08}));

    // SYNC_WRITE target position to servos 1 and 2
    printPacket("write(pos,2)",
        STServoRequest::write(STServo::Register::TARGET_LOCATION, {
            {0x01, {0x00, 0x08}},   // servo 1 → 2048
            {0x02, {0x00, 0x04}},   // servo 2 → 1024
        }));

    // REG_WRITE (buffered) to servo 1, then broadcast ACTION
    printPacket("regWrite(1)",
        STServoRequest::regWrite(0x01, STServo::Register::TARGET_LOCATION, {0x00, 0x08}));
    printPacket("action()",
        STServoRequest::action());

    // RESET servo 5
    printPacket("reset(5)",
        STServoRequest::reset(0x05));

    return 0;
}
