#include <cstdio>
#include <cstdlib>
#include "servo/STServoRequest.hpp"
#include "servo/STServoResponse.hpp"
#include "serial/SerialPortConfig.hpp"
#include "serial/SerialPortDriver.hpp"
#include "servo/STServoRegisters.hpp"

// A response to PING is: [FF FF id 02 error cksum] = 6 bytes
static constexpr std::size_t PING_RESPONSE_LEN = 6;

static void printBytes(const char* label, const std::vector<uint8_t>& v)
{
    std::printf("%-16s:", label);
    for (uint8_t b : v) std::printf(" %02X", b);
    std::printf("\n");
}

int main()
{
    // ---- Load configuration from environment --------------------------------
    STServo::SerialPortConfig cfg;
    try {
        cfg = STServo::SerialPortConfig::fromEnv();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Config error: %s\n", e.what());
        return EXIT_FAILURE;
    }

    std::printf("Port    : %s\n", cfg.port.c_str());
    std::printf("Baud    : %d\n", cfg.baud);
    std::printf("Timeout : %d ms\n\n", cfg.timeoutMs);

    // ---- Open serial port ---------------------------------------------------
    SerialPortDriver port(cfg);
    if (!port.isOpen()) {
        std::fprintf(stderr, "Failed to open %s: %s\n",
                     cfg.port.c_str(), std::strerror(errno));
        return EXIT_FAILURE;
    }

    // ---- Ping servo ID 1 ----------------------------------------------------
    const uint8_t servoId = 1;
    auto pingPacket = STServoRequest::ping(servoId);
    printBytes("Sending PING", pingPacket);

    if (!port.write(pingPacket)) {
        std::fprintf(stderr, "Write failed: %s\n", std::strerror(errno));
        return EXIT_FAILURE;
    }

    // ---- Read response ------------------------------------------------------
    auto raw = port.read(PING_RESPONSE_LEN);
    printBytes("Raw response", raw);

    if (raw.empty()) {
        std::fprintf(stderr, "No response — check wiring and servo ID.\n");
        return EXIT_FAILURE;
    }

    // ---- Parse response -----------------------------------------------------
    auto resp = STServo::ServoResponse::parse(raw);
    if (!resp.valid) {
        std::fprintf(stderr, "Invalid response packet (checksum or framing error).\n");
        return EXIT_FAILURE;
    }

    std::printf("\nServo ID : %d\n", resp.id);
    std::printf("Error    : 0x%02X%s\n", resp.error,
                resp.hasError() ? "  <-- fault reported" : "  (no errors)");
    if (resp.hasFlag(STServo::ErrorFlag::VOLTAGE))     std::puts("  [!] Voltage error");
    if (resp.hasFlag(STServo::ErrorFlag::ANGLE))       std::puts("  [!] Angle/sensor error");
    if (resp.hasFlag(STServo::ErrorFlag::OVERHEAT))    std::puts("  [!] Overheat error");
    if (resp.hasFlag(STServo::ErrorFlag::RANGE))       std::puts("  [!] Range error");
    if (resp.hasFlag(STServo::ErrorFlag::CHECKSUM))    std::puts("  [!] Checksum error");
    if (resp.hasFlag(STServo::ErrorFlag::OVERLOAD))    std::puts("  [!] Overload error");
    if (resp.hasFlag(STServo::ErrorFlag::INSTRUCTION)) std::puts("  [!] Instruction error");

    return EXIT_SUCCESS;
}
