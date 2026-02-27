#include <cerrno>
#include <cstdio>
#include <cstring>
#include <vector>

#include "servo/STServoRequest.hpp"
#include "servo/STServoResponse.hpp"
#include "servo/STServoRegisters.hpp"
#include "serial/SerialPortConfig.hpp"
#include "serial/SerialPortDriver.hpp"

// ---------------------------------------------------------------------------
// test_sync_read
//
// Sends a single SYNC_READ packet requesting CURRENT_LOCATION from IDs 1 and
// 3, then reads back each servo's individual response packet in sequence.
//
// This probes whether the ST servo firmware actually honours SYNC_READ (0x82),
// which is not mentioned in the available documentation.  If both servos reply
// the instruction is usable; if neither replies it is simply unsupported.
// ---------------------------------------------------------------------------

static constexpr std::size_t RESPONSE_LEN = 6 + 2; // header(4) + error(1) + 2-byte data + cksum

int main()
{
    STServo::SerialPortConfig cfg;
    try {
        cfg = STServo::SerialPortConfig::fromEnv();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Config error: %s\n", e.what());
        return 1;
    }

    SerialPortDriver port(cfg);
    if (!port.isOpen()) {
        std::fprintf(stderr, "Failed to open %s: %s\n",
                     cfg.port.c_str(), std::strerror(errno));
        return 1;
    }

    const std::vector<uint8_t> ids = { 1, 3 };
    const auto& reg = STServo::Register::CURRENT_LOCATION;

    // Build and send the SYNC_READ packet
    auto pkt = STServoRequest::read(ids, reg);

    std::printf("Sending SYNC_READ (0x82) for CURRENT_LOCATION to IDs:");
    for (uint8_t id : ids) std::printf(" %d", id);
    std::printf("\nPacket bytes:");
    for (uint8_t b : pkt) std::printf(" %02X", b);
    std::printf("\n\n");

    if (!port.write(pkt)) {
        std::fprintf(stderr, "Write failed: %s\n", std::strerror(errno));
        return 1;
    }

    // Each servo should reply with its own individual status packet
    int responded = 0;
    for (uint8_t id : ids) {
        auto raw = port.read(RESPONSE_LEN);

        if (raw.empty()) {
            std::printf("  ID %d : no response (timeout)\n", id);
            continue;
        }

        std::printf("  ID %d raw:", id);
        for (uint8_t b : raw) std::printf(" %02X", b);
        std::printf("\n");

        auto resp = STServo::ServoResponse::parse(raw);
        if (!resp.valid) {
            std::printf("  ID %d : response received but failed to parse\n", id);
            continue;
        }

        ++responded;
        if (resp.hasError()) {
            std::printf("  ID %d : error byte 0x%02X\n", id, resp.error);
            continue;
        }

        auto pos = resp.asUint16();
        if (pos)
            std::printf("  ID %d : CURRENT_LOCATION = %u (0x%04X)\n",
                        id, *pos, *pos);
        else
            std::printf("  ID %d : valid packet but unexpected data length (%zu bytes)\n",
                        id, resp.data.size());
    }

    std::printf("\n");
    if (responded == 0)
        std::puts("Result: SYNC_READ appears UNSUPPORTED — no servo responded.");
    else if (responded < (int)ids.size())
        std::printf("Result: partial — %d / %d servo(s) responded.\n",
                    responded, (int)ids.size());
    else
        std::puts("Result: SYNC_READ appears SUPPORTED — all servos responded.");

    return (responded == (int)ids.size()) ? 0 : 1;
}
