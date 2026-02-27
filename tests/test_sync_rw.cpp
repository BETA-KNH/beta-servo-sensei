#include <cerrno>
#include <cstdio>
#include <cstring>
#include <thread>
#include <chrono>
#include <vector>
#include <optional>

#include "servo/STServoRequest.hpp"
#include "servo/STServoResponse.hpp"
#include "servo/STServoRegisters.hpp"
#include "serial/SerialPortConfig.hpp"
#include "serial/SerialPortDriver.hpp"

// ---------------------------------------------------------------------------
// test_sync_rw
//
// 1. SYNC_READ  CURRENT_LOCATION  from IDs 1 and 3
// 2. SYNC_WRITE TARGET_LOCATION   to   IDs 1 and 3  (swap their positions)
// 3. Wait for motion to complete
// 4. SYNC_READ  CURRENT_LOCATION  again to confirm arrival
// ---------------------------------------------------------------------------

static constexpr std::size_t READ_RESPONSE_LEN = 8; // FF FF id 04 err lo hi cksum
static constexpr uint16_t    MOVE_SPEED        = 800;
static constexpr int         MOVE_WAIT_MS      = 5000;

// Build the 6-byte motion payload: pos(LE,2) | time(0,2) | speed(LE,2)
static std::vector<uint8_t> movePayload(uint16_t pos, uint16_t speed)
{
    return {
        static_cast<uint8_t>(pos   & 0xFF), static_cast<uint8_t>(pos   >> 8),
        0x00, 0x00,
        static_cast<uint8_t>(speed & 0xFF), static_cast<uint8_t>(speed >> 8),
    };
}

// Read and parse responses for each id; returns position or nullopt on failure.
static std::vector<std::optional<uint16_t>>
syncRead(SerialPortDriver& port, const std::vector<uint8_t>& ids,
         const STServo::Reg& reg)
{
    std::vector<std::optional<uint16_t>> results(ids.size(), std::nullopt);

    auto pkt = STServoRequest::read(ids, reg);
    if (!port.write(pkt)) {
        std::fprintf(stderr, "  SYNC_READ write failed: %s\n", std::strerror(errno));
        return results;
    }

    for (std::size_t i = 0; i < ids.size(); ++i) {
        auto raw = port.read(READ_RESPONSE_LEN);
        if (raw.empty()) {
            std::printf("  ID %d : no response\n", ids[i]);
            continue;
        }
        auto resp = STServo::ServoResponse::parse(raw);
        if (!resp.valid || resp.hasError()) {
            std::printf("  ID %d : bad response (valid=%d error=0x%02X)\n",
                        ids[i], resp.valid, resp.error);
            continue;
        }
        results[i] = resp.asUint16();
        std::printf("  ID %d : %u (0x%04X)\n", ids[i], *results[i], *results[i]);
    }
    return results;
}

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
    const auto& posReg = STServo::Register::CURRENT_LOCATION;

    // ---- 1. Read current positions -----------------------------------------
    std::printf("--- SYNC_READ (before move) ---\n");
    auto before = syncRead(port, ids, posReg);

    if (!before[0] || !before[1]) {
        std::fprintf(stderr, "Could not read from one or both servos. Aborting.\n");
        return 1;
    }

    uint16_t pos1 = *before[0];
    uint16_t pos3 = *before[1];

    // ---- 2. SYNC_WRITE: swap positions -------------------------------------
    std::printf("\n--- SYNC_WRITE (swapping positions: ID1→%u, ID3→%u) ---\n",
                pos3, pos1);

    // Write 6-byte move payload: TARGET_LOCATION(2) + RUNNING_TIME(2) + OPERATION_SPEED(2)
    // The SYNC_WRITE reg must span all 6 bytes starting at TARGET_LOCATION.
    // We construct a synthetic Reg covering the 6-byte motion block.
    const STServo::Reg moveReg = {
        STServo::Register::TARGET_LOCATION.address,
        6,
        STServo::MemoryArea::SRAM,
        STServo::Access::RW
    };

    auto writePkt = STServoRequest::write(moveReg, {
        { 1, movePayload(pos3, MOVE_SPEED) },
        { 3, movePayload(pos1, MOVE_SPEED) },
    });

    std::printf("Packet bytes:");
    for (uint8_t b : writePkt) std::printf(" %02X", b);
    std::printf("\n");

    if (!port.write(writePkt)) {
        std::fprintf(stderr, "SYNC_WRITE failed: %s\n", std::strerror(errno));
        return 1;
    }

    // SYNC_WRITE to broadcast ID — no response packet expected.
    std::printf("Waiting %d ms for servos to reach target...\n", MOVE_WAIT_MS);
    std::this_thread::sleep_for(std::chrono::milliseconds(MOVE_WAIT_MS));

    // ---- 3. Read positions again --------------------------------------------
    std::printf("\n--- SYNC_READ (after move) ---\n");
    auto after = syncRead(port, ids, posReg);

    // ---- 4. Verdict ---------------------------------------------------------
    std::printf("\n--- Result ---\n");
    bool syncWriteOk = true;

    for (std::size_t i = 0; i < ids.size(); ++i) {
        uint16_t expected = (i == 0) ? pos3 : pos1;
        if (!after[i]) {
            std::printf("  ID %d : no post-move reading\n", ids[i]);
            syncWriteOk = false;
            continue;
        }
        int delta = static_cast<int>(*after[i]) - static_cast<int>(expected);
        if (delta < 0) delta = -delta;
        std::printf("  ID %d : expected %u, got %u  (delta %d)%s\n",
                    ids[i], expected, *after[i], delta,
                    delta <= 10 ? "  OK" : "  MISS");
        if (delta > 10) syncWriteOk = false;
    }

    std::printf("\nSYNC_READ  : SUPPORTED\n");
    std::printf("SYNC_WRITE : %s\n", syncWriteOk ? "SUPPORTED" : "UNSUPPORTED or not settled");
    return syncWriteOk ? 0 : 1;
}
