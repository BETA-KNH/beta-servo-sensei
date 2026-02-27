#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "servo/STServoRequest.hpp"
#include "servo/STServoResponse.hpp"
#include "serial/SerialPortConfig.hpp"
#include "serial/SerialPortDriver.hpp"
#include "servo/STServoRegisters.hpp"

// ---------------------------------------------------------------------------
// scan_servos
//
// Sends a PING to every servo ID in [firstId, lastId] and prints which ones
// respond.  Useful for discovering servos on a new bus or verifying wiring.
//
// Environment variables (same as servo_controller):
//   SERVO_PORT        default: /dev/ttyUSB0  (or set in servo.env / conf/servo.env)
//   SERVO_BAUD        default: 1000000
//   SERVO_TIMEOUT_MS  default: 100   (per-servo timeout; lower = faster scan)
//
// Optional positional arguments:
//   scan_servos [firstId] [lastId]   default: 1 to 253
// ---------------------------------------------------------------------------

static constexpr std::size_t PING_RESPONSE_LEN = 6;  // FF FF id 02 error cksum

int main(int argc, char* argv[])
{
    // ---- Parse optional range arguments ------------------------------------
    uint8_t firstId = 1;
    uint8_t lastId  = 253;  // 0xFE is broadcast, never a real servo ID

    if (argc >= 2) firstId = static_cast<uint8_t>(std::atoi(argv[1]));
    if (argc >= 3) lastId  = static_cast<uint8_t>(std::atoi(argv[2]));

    if (firstId > lastId) {
        std::fprintf(stderr, "Usage: scan_servos [firstId] [lastId]\n");
        return EXIT_FAILURE;
    }

    // ---- Load configuration ------------------------------------------------
    STServo::SerialPortConfig cfg;
    try {
        cfg = STServo::SerialPortConfig::fromEnv();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Config error: %s\n", e.what());
        return EXIT_FAILURE;
    }

    std::printf("Port     : %s\n",    cfg.port.c_str());
    std::printf("Baud     : %d\n",    cfg.baud);
    std::printf("Timeout  : %d ms\n", cfg.timeoutMs);
    std::printf("Scanning : IDs %d – %d\n\n", firstId, lastId);

    // ---- Open serial port --------------------------------------------------
    SerialPortDriver port(cfg);
    if (!port.isOpen()) {
        std::fprintf(stderr, "Failed to open %s: %s\n",
                     cfg.port.c_str(), std::strerror(errno));
        return EXIT_FAILURE;
    }

    // ---- Scan --------------------------------------------------------------
    std::vector<uint8_t> foundIds;

    const int total = lastId - firstId + 1;
    int scanned = 0;

    for (uint8_t id = firstId; id <= lastId; ++id) {
        // Progress indicator
        std::printf("\rScanning ID %3d / %3d ...", id, lastId);
        std::fflush(stdout);

        // Send PING
        auto ping = STServoRequest::ping(id);
        if (!port.write(ping)) {
            std::fprintf(stderr, "\nWrite error on ID %d: %s\n",
                         id, std::strerror(errno));
            break;
        }

        // Read response (times out after SERVO_TIMEOUT_MS if no servo present)
        auto raw = port.read(PING_RESPONSE_LEN);
        if (raw.empty()) {
            // No response — no servo at this ID
            ++scanned;
            continue;
        }

        auto resp = STServo::ServoResponse::parse(raw);
        if (resp.valid && resp.id == id) {
            foundIds.push_back(id);
            std::printf("\r  Found servo ID %3d%s\n",
                        id, resp.hasError() ? "  [!] errors reported" : "");
        }

        ++scanned;

        // Avoid wrapping at 255 → 0
        if (id == lastId) break;
    }

    // ---- Summary -----------------------------------------------------------
    std::printf("\r%-40s\n", "");  // clear progress line
    std::printf("Scan complete. %d ID%s checked.\n\n",
                scanned, scanned == 1 ? "" : "s");

    if (foundIds.empty()) {
        std::puts("No servos found.");
        std::puts("Check: power, wiring, baud rate, and that TX/RX are not swapped.");
    } else {
        std::printf("%zu servo%s found:\n",
                    foundIds.size(), foundIds.size() == 1 ? "" : "s");
        for (uint8_t id : foundIds)
            std::printf("  ID %d\n", id);
    }

    return foundIds.empty() ? EXIT_FAILURE : EXIT_SUCCESS;
}
