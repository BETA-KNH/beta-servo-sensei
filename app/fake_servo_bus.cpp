/**
 * @file fake_servo_bus.cpp
 * @brief Debug simulator for the ST servo bus — no hardware needed.
 *
 * Creates a kernel PTY pair:
 *   master fd  — this process acts as the simulated servo bus
 *   slave path — opened by yarpdev / STServoYarpDriver as if it were /dev/ttyUSB0
 *
 * What the simulator does
 * -----------------------
 * A background thread reads raw servo packets from the master fd, parses
 * them, and replies with plausible responses:
 *   - PING         → status-OK packet for known IDs
 *   - READ / SYNC_READ → per-register simulated values
 *   - WRITE / SYNC_WRITE → applied to simulated state (no reply)
 *
 * A second thread advances the simulation each 20 ms:
 *   - current position drifts toward the commanded target at op_speed steps/s
 *
 * The yarpdev process
 * -------------------
 * After setting up the PTY, this program forks and execs yarpdev with
 * deviceBundler + controlBoard_nws_yarp + st_servo pointed at the slave PTY.
 * The YARP network ports become available at /robot/debug_servo (default).
 *
 * Then connect with e.g.:
 *   yarpmotorgui --robot /robot/debug_servo
 *
 * Usage
 * -----
 *   ./fake_servo_bus [options]
 *
 *   --ids    "1 2 3"      Servo IDs to simulate (default: "1 2 3")
 *   --name   /my/port     YARP port prefix     (default: /robot/debug_servo)
 *   --baud   115200       Baud rate passed to yarpdev (default: 115200)
 *   --no-yarp             Print the slave PTY path and exit — you run yarpdev yourself
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include "PtyHelper.hpp"

// ---------------------------------------------------------------------------
// Register addresses (subset — mirrors STServoRegisters.hpp)
// ---------------------------------------------------------------------------
static constexpr uint8_t INSTR_PING       = 0x01;
static constexpr uint8_t INSTR_READ       = 0x02;
static constexpr uint8_t INSTR_SYNC_READ  = 0x82;
// WRITE / SYNC_WRITE don't need constants — detected by elimination

static constexpr uint8_t REG_TORQUE_SWITCH = 0x28;
static constexpr uint8_t REG_ACCELERATION  = 0x29;
static constexpr uint8_t REG_TARGET_LOC    = 0x2A;  // also start of 6-byte move block
static constexpr uint8_t REG_OP_SPEED      = 0x2E;
static constexpr uint8_t REG_TORQUE_LIMIT  = 0x30;
static constexpr uint8_t REG_P_COEFF       = 0x15;
static constexpr uint8_t REG_D_COEFF       = 0x16;
static constexpr uint8_t REG_I_COEFF       = 0x17;
static constexpr uint8_t REG_SPEED_P       = 0x25;
static constexpr uint8_t REG_VEL_I         = 0x27;
static constexpr uint8_t REG_CURRENT_LOC   = 0x38;
static constexpr uint8_t REG_CURRENT_SPEED = 0x3A;
static constexpr uint8_t REG_CURRENT_LOAD  = 0x3C;
static constexpr uint8_t REG_VOLTAGE       = 0x3E;
static constexpr uint8_t REG_TEMPERATURE   = 0x3F;
static constexpr uint8_t REG_SERVO_STATUS  = 0x41;
static constexpr uint8_t REG_MOVE_FLAG     = 0x42;
static constexpr uint8_t REG_CURRENT       = 0x45;
static constexpr uint8_t BROADCAST_ID      = 0xFE;

// ---------------------------------------------------------------------------
// Per-servo simulated state
// ---------------------------------------------------------------------------
struct SimServo {
    int16_t  current_pos  = 2047;    ///< Current encoder position (steps, ~180 deg)
    int16_t  target_pos   = 2047;    ///< Commanded target position
    uint16_t op_speed     = 500;     ///< Operation speed (steps/s)
    uint8_t  torque_on    = 0;
    uint8_t  p_coeff      = 32;
    uint8_t  d_coeff      = 32;
    uint8_t  i_coeff      = 0;
    uint8_t  speed_p      = 10;
    uint8_t  vel_i        = 10;
    uint16_t torque_limit = 1000;
};

std::map<uint8_t, SimServo> g_servos;
std::mutex                  g_mutex;
std::atomic<bool>           g_running{true};

// ---------------------------------------------------------------------------
// Packet helpers
// ---------------------------------------------------------------------------
static std::vector<uint8_t> makeResponse(uint8_t id, const std::vector<uint8_t>& data)
{
    uint8_t len = static_cast<uint8_t>(2u + data.size());
    uint8_t sum = static_cast<uint8_t>(id + len);
    for (auto b : data) sum = static_cast<uint8_t>(sum + b);
    std::vector<uint8_t> pkt = {0xFF, 0xFF, id, len, 0x00};
    pkt.insert(pkt.end(), data.begin(), data.end());
    pkt.push_back(static_cast<uint8_t>(~sum & 0xFFu));
    return pkt;
}

static std::vector<uint8_t> buildReadReply(uint8_t id, uint8_t reg, uint8_t size)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    auto  it = g_servos.find(id);
    if (it == g_servos.end()) return {};   // unknown ID — no reply

    const SimServo& s = it->second;
    std::vector<uint8_t> data(size, 0);

    auto putU16 = [&](int16_t v) {
        if (size >= 1) data[0] = static_cast<uint8_t>(v & 0xFF);
        if (size >= 2) data[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };
    auto putU16u = [&](uint16_t v) { putU16(static_cast<int16_t>(v)); };

    switch (reg) {
    case REG_CURRENT_LOC:   putU16(s.current_pos);         break;
    case REG_TARGET_LOC:    putU16(s.target_pos);          break;
    case REG_CURRENT_SPEED:
        putU16u((s.torque_on && s.current_pos != s.target_pos) ? 100u : 0u);
        break;
    case REG_MOVE_FLAG:
        data[0] = (s.torque_on && std::abs(s.current_pos - s.target_pos) > 5) ? 1 : 0;
        break;
    case REG_TORQUE_SWITCH: data[0] = s.torque_on;         break;
    case REG_OP_SPEED:      putU16u(s.op_speed);           break;
    case REG_TORQUE_LIMIT:  putU16u(s.torque_limit);       break;
    case REG_P_COEFF:       data[0] = s.p_coeff;           break;
    case REG_D_COEFF:       data[0] = s.d_coeff;           break;
    case REG_I_COEFF:       data[0] = s.i_coeff;           break;
    case REG_SPEED_P:       data[0] = s.speed_p;           break;
    case REG_VEL_I:         data[0] = s.vel_i;             break;
    case REG_VOLTAGE:       data[0] = 120;                 break;  // 12.0 V
    case REG_TEMPERATURE:   data[0] = 35;                  break;  // 35 °C
    case REG_SERVO_STATUS:  data[0] = 0;                   break;  // no faults
    case REG_CURRENT_LOAD:  putU16u(0);                    break;
    case REG_CURRENT:       putU16u(0);                    break;
    default:                                               break;  // zeros
    }
    return makeResponse(id, data);
}

static void applyWrite(uint8_t id, uint8_t reg, const std::vector<uint8_t>& data)
{
    std::lock_guard<std::mutex> lk(g_mutex);
    auto it = g_servos.find(id);
    if (it == g_servos.end()) return;
    SimServo& s = it->second;

    auto u16 = [&](std::size_t off = 0) -> uint16_t {
        return static_cast<uint16_t>(
            (off < data.size() ? data[off] : 0u) |
            ((off+1 < data.size() ? data[off+1] : 0u) << 8));
    };

    switch (reg) {
    case REG_TORQUE_SWITCH: if (!data.empty()) s.torque_on = data[0]; break;
    case REG_TARGET_LOC:    // 0x2A — 2-byte single write OR 6-byte compound move block
        s.target_pos = static_cast<int16_t>(u16(0));
        if (data.size() >= 6)
            s.op_speed = u16(4);   // compound move: [pos(2) time(2) speed(2)]
        break;
    case REG_OP_SPEED:      s.op_speed  = u16(); break;
    case REG_TORQUE_LIMIT:  s.torque_limit = u16(); break;
    case REG_P_COEFF:       if (!data.empty()) s.p_coeff  = data[0]; break;
    case REG_D_COEFF:       if (!data.empty()) s.d_coeff  = data[0]; break;
    case REG_I_COEFF:       if (!data.empty()) s.i_coeff  = data[0]; break;
    case REG_SPEED_P:       if (!data.empty()) s.speed_p  = data[0]; break;
    case REG_VEL_I:         if (!data.empty()) s.vel_i    = data[0]; break;
    default: break;
    }
}

// ---------------------------------------------------------------------------
// Packet parser + dispatcher (runs on the master fd)
// ---------------------------------------------------------------------------
static void writeMaster(int fd, const std::vector<uint8_t>& pkt)
{
    const uint8_t* p = pkt.data();
    std::size_t rem = pkt.size();
    while (rem > 0) {
        ssize_t n = ::write(fd, p, rem);
        if (n <= 0) break;
        p += n; rem -= static_cast<std::size_t>(n);
    }
}

static void dispatchPacket(int masterFd,
                           uint8_t id, uint8_t instr,
                           const std::vector<uint8_t>& params)
{
    if (instr == INSTR_PING) {
        if (g_servos.count(id))
            writeMaster(masterFd, makeResponse(id, {}));
        return;
    }

    if (instr == INSTR_READ && params.size() >= 2) {
        auto reply = buildReadReply(id, params[0], params[1]);
        if (!reply.empty()) writeMaster(masterFd, reply);
        return;
    }

    if (instr == INSTR_SYNC_READ && params.size() >= 3) {
        // params: [reg_addr] [reg_size] [id1] [id2] ...
        uint8_t reg  = params[0];
        uint8_t size = params[1];
        for (std::size_t i = 2; i < params.size(); ++i) {
            auto reply = buildReadReply(params[i], reg, size);
            if (!reply.empty()) writeMaster(masterFd, reply);
        }
        return;
    }

    // WRITE (0x03): params[0]=reg_addr, params[1..]=data
    if (instr == 0x03 && !params.empty()) {
        applyWrite(id, params[0],
                   std::vector<uint8_t>(params.begin() + 1, params.end()));
        return;
    }

    // SYNC_WRITE (0x83): params[0]=reg, params[1]=per-servo-len, then [id][data]...
    if (instr == 0x83 && params.size() >= 2) {
        uint8_t reg     = params[0];
        uint8_t datalen = params[1];
        std::size_t off = 2;
        while (off + 1 + datalen <= params.size()) {
            uint8_t sid = params[off];
            std::vector<uint8_t> d(params.begin() + off + 1,
                                   params.begin() + off + 1 + datalen);
            applyWrite(sid, reg, d);
            off += 1 + datalen;
        }
        return;
    }
    // LOCK_FLAG writes, ACTION, RESET, REG_WRITE — silently ignore
}

/// Reads bytes from masterFd and parses complete packets.
static void busThread(int masterFd)
{
    std::vector<uint8_t> buf;
    buf.reserve(256);
    uint8_t tmp[128];

    // Use poll() so the thread wakes immediately when a packet arrives
    // rather than sleeping a fixed interval. This ensures the response
    // is written back before the driver's read() poll-timeout fires.
    struct pollfd pfd{masterFd, POLLIN, 0};

    while (g_running) {
        int r = ::poll(&pfd, 1, 10);  // wake within 10 ms to check g_running
        if (r <= 0) continue;         // timeout or error
        if (!(pfd.revents & POLLIN)) continue;

        ssize_t n = ::read(masterFd, tmp, sizeof(tmp));
        if (n <= 0) continue;
        buf.insert(buf.end(), tmp, tmp + n);

        // Process complete packets out of buf
        while (buf.size() >= 6) {
            // Find sync header
            while (buf.size() >= 2 &&
                   !(buf[0] == 0xFF && buf[1] == 0xFF))
                buf.erase(buf.begin());

            if (buf.size() < 6) break;

            uint8_t id  = buf[2];
            uint8_t len = buf[3];   // = 1 (instr) + N (params) + 1 (cksum)
            if (len < 2) { buf.erase(buf.begin()); continue; }

            std::size_t total = static_cast<std::size_t>(4 + len);
            if (buf.size() < total) break;   // wait for more bytes

            uint8_t instr = buf[4];
            std::vector<uint8_t> params(buf.begin() + 5,
                                        buf.begin() + 4 + len - 1);
            buf.erase(buf.begin(), buf.begin() + total);

            dispatchPacket(masterFd, id, instr, params);
        }
    }
}

/// Advances the physics simulation every 20 ms.
static void simThread()
{
    using clk = std::chrono::steady_clock;
    auto last = clk::now();

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        auto now   = clk::now();
        double dt  = std::chrono::duration<double>(now - last).count();
        last = now;

        std::lock_guard<std::mutex> lk(g_mutex);
        for (auto& [id, s] : g_servos) {
            if (!s.torque_on) continue;
            int diff = static_cast<int>(s.target_pos) -
                       static_cast<int>(s.current_pos);
            if (std::abs(diff) <= 1) { s.current_pos = s.target_pos; continue; }
            double step = s.op_speed * dt;
            int    move = std::clamp(static_cast<int>(step), 1,
                                     std::abs(diff));
            s.current_pos = static_cast<int16_t>(
                s.current_pos + (diff > 0 ? move : -move));
        }
    }
}

// ---------------------------------------------------------------------------
// Signal handling
// ---------------------------------------------------------------------------
static pid_t g_yarpdevPid = -1;

static void onSignal(int)
{
    g_running = false;
    if (g_yarpdevPid > 0) ::kill(g_yarpdevPid, SIGTERM);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
    // --- Parse arguments ---------------------------------------------------
    std::vector<uint8_t> ids     = {1, 2, 3};
    std::string          portName = "/robot/debug_servo";
    std::string          baud     = "115200";
    bool                 noYarp   = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--no-yarp") == 0) {
            noYarp = true;
        } else if (std::strcmp(argv[i], "--ids") == 0 && i + 1 < argc) {
            ids.clear();
            std::istringstream ss(argv[++i]);
            int v;
            while (ss >> v) ids.push_back(static_cast<uint8_t>(v));
        } else if (std::strcmp(argv[i], "--name") == 0 && i + 1 < argc) {
            portName = argv[++i];
        } else if (std::strcmp(argv[i], "--baud") == 0 && i + 1 < argc) {
            baud = argv[++i];
        }
    }

    // --- Populate servo table ---------------------------------------------
    for (uint8_t id : ids) g_servos[id];   // default-initialises SimServo

    // --- Open PTY ---------------------------------------------------------
    PtyHelper pty;
    if (!pty.isOpen()) {
        std::cerr << "[fake_servo_bus] ERROR: could not create PTY\n";
        return 1;
    }
    std::cout << "[fake_servo_bus] Slave PTY: " << pty.slavePath() << "\n";
    std::cout << "[fake_servo_bus] Simulating servo IDs:";
    for (auto id : ids) std::cout << " " << static_cast<int>(id);
    std::cout << "\n";

    // Master fd stays blocking — busThread uses poll() to wait for packets
    // without a fixed sleep delay, so responses arrive in microseconds.

    // --- Signals ----------------------------------------------------------
    struct sigaction sa{};
    sa.sa_handler = onSignal;
    ::sigaction(SIGINT,  &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);
    ::sigaction(SIGCHLD, &sa, nullptr);

    // --- Start background threads -----------------------------------------
    std::thread bus(busThread,  pty.masterFd());
    std::thread sim(simThread);

    if (noYarp) {
        std::cout << "[fake_servo_bus] --no-yarp: bus is live on " << pty.slavePath() << "\n";
        std::cout << "[fake_servo_bus] Run yarpdev yourself:\n"
                  << "  yarpdev --device deviceBundler \\\n"
                  << "          --wrapper_device controlBoard_nws_yarp \\\n"
                  << "          --attached_device st_servo \\\n"
                  << "          --name " << portName << " \\\n"
                  << "          --port " << pty.slavePath() << " \\\n"
                  << "          --baud " << baud << " \\\n"
                  << "          --servo_ids \"(";
        for (std::size_t i = 0; i < ids.size(); ++i)
            std::cout << static_cast<int>(ids[i]) << (i+1 < ids.size() ? " " : "");
        std::cout << ")\"\n";
        std::cout << "[fake_servo_bus] Press Ctrl+C to exit.\n";
        while (g_running) std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
        // Build servo_ids string  "(1 2 3)"
        std::string idsStr = "(";
        for (std::size_t i = 0; i < ids.size(); ++i) {
            idsStr += std::to_string(ids[i]);
            if (i + 1 < ids.size()) idsStr += ' ';
        }
        idsStr += ')';

        pid_t pid = ::fork();
        if (pid < 0) {
            std::cerr << "[fake_servo_bus] fork() failed\n";
            g_running = false;
        } else if (pid == 0) {
            // child — exec yarpdev
            ::execlp("yarpdev", "yarpdev",
                     "--device",          "deviceBundler",
                     "--wrapper_device",  "controlBoard_nws_yarp",
                     "--attached_device", "st_servo",
                     "--name",  portName.c_str(),
                     "--port",  pty.slavePath().c_str(),
                     "--baud",  baud.c_str(),
                     "--servo_ids", idsStr.c_str(),
                     "--timeout_ms", "100",
                     nullptr);
            std::cerr << "[fake_servo_bus] exec yarpdev failed: "
                      << std::strerror(errno) << "\n";
            ::_exit(1);
        } else {
            g_yarpdevPid = pid;
            std::cout << "[fake_servo_bus] yarpdev PID " << pid << " started\n";
            std::cout << "[fake_servo_bus] YARP ports on: " << portName << "\n";
            std::cout << "[fake_servo_bus] Connect with:  yarpmotorgui --robot "
                      << portName << "\n";
            std::cout << "[fake_servo_bus] Press Ctrl+C to stop.\n";

            // Print servo state every 2 seconds while running
            while (g_running) {
                std::this_thread::sleep_for(std::chrono::seconds(2));
                if (!g_running) break;
                std::cout << "[sim]";
                std::lock_guard<std::mutex> lk(g_mutex);
                for (auto& [id, s] : g_servos) {
                    double deg = s.current_pos * (360.0 / 4095.0);
                    std::printf("  ID%d: pos=%.1f° tgt=%.1f° torque=%s",
                                id, deg,
                                s.target_pos * (360.0 / 4095.0),
                                s.torque_on ? "ON" : "off");
                }
                std::cout << "\n";
            }

            // Wait for yarpdev
            int status = 0;
            ::waitpid(pid, &status, 0);
            g_yarpdevPid = -1;
        }
    }

    g_running = false;
    // Wake the bus thread (write a dummy byte to the slave)
    {
        int slaveFd = ::open(pty.slavePath().c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
        if (slaveFd >= 0) { ::write(slaveFd, "\x00", 1); ::close(slaveFd); }
    }
    bus.join();
    sim.join();
    std::cout << "[fake_servo_bus] Bye.\n";
    return 0;
}
