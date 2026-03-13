#include "sim_bus.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <poll.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "common.hpp"

namespace fake_servo_bus {

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
    auto it = g_servos.find(id);
    if (it == g_servos.end()) return {};

    const SimServo& s = it->second;
    std::vector<uint8_t> data(size, 0);

    auto putU16 = [&](int16_t v) {
        if (size >= 1) data[0] = static_cast<uint8_t>(v & 0xFF);
        if (size >= 2) data[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };
    auto putU16u = [&](uint16_t v) { putU16(static_cast<int16_t>(v)); };

    switch (reg) {
    case REG_CURRENT_LOC:   putU16(s.current_pos); break;
    case REG_TARGET_LOC:    putU16(s.target_pos); break;
    case REG_CURRENT_SPEED:
        putU16u((s.torque_on && s.current_pos != s.target_pos) ? 100u : 0u);
        break;
    case REG_MOVE_FLAG:
        data[0] = (s.torque_on && std::abs(s.current_pos - s.target_pos) > 5) ? 1 : 0;
        break;
    case REG_TORQUE_SWITCH: data[0] = s.torque_on; break;
    case REG_OP_SPEED:      putU16u(s.op_speed); break;
    case REG_TORQUE_LIMIT:  putU16u(s.torque_limit); break;
    case REG_P_COEFF:       data[0] = s.p_coeff; break;
    case REG_D_COEFF:       data[0] = s.d_coeff; break;
    case REG_I_COEFF:       data[0] = s.i_coeff; break;
    case REG_SPEED_P:       data[0] = s.speed_p; break;
    case REG_VEL_I:         data[0] = s.vel_i; break;
    case REG_VOLTAGE:       data[0] = 120; break;
    case REG_TEMPERATURE:   data[0] = 35; break;
    case REG_SERVO_STATUS:  data[0] = 0; break;
    case REG_CURRENT_LOAD:  putU16u(0); break;
    case REG_CURRENT:       putU16u(0); break;
    default: break;
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
            ((off + 1 < data.size() ? data[off + 1] : 0u) << 8));
    };

    switch (reg) {
    case REG_TORQUE_SWITCH: if (!data.empty()) s.torque_on = data[0]; break;
    case REG_TARGET_LOC:
        s.target_pos = static_cast<int16_t>(u16(0));
        if (data.size() >= 6) s.op_speed = u16(4);
        break;
    case REG_OP_SPEED:      s.op_speed = u16(); break;
    case REG_TORQUE_LIMIT:  s.torque_limit = u16(); break;
    case REG_P_COEFF:       if (!data.empty()) s.p_coeff = data[0]; break;
    case REG_D_COEFF:       if (!data.empty()) s.d_coeff = data[0]; break;
    case REG_I_COEFF:       if (!data.empty()) s.i_coeff = data[0]; break;
    case REG_SPEED_P:       if (!data.empty()) s.speed_p = data[0]; break;
    case REG_VEL_I:         if (!data.empty()) s.vel_i = data[0]; break;
    default: break;
    }
}

static void writeMaster(int fd, const std::vector<uint8_t>& pkt)
{
    const uint8_t* p = pkt.data();
    std::size_t rem = pkt.size();
    while (rem > 0) {
        ssize_t n = ::write(fd, p, rem);
        if (n <= 0) break;
        p += n;
        rem -= static_cast<std::size_t>(n);
    }
}

static void dispatchPacket(int masterFd,
                           uint8_t id,
                           uint8_t instr,
                           const std::vector<uint8_t>& params)
{
    if (instr == INSTR_PING) {
        if (g_servos.count(id)) writeMaster(masterFd, makeResponse(id, {}));
        return;
    }

    if (instr == INSTR_READ && params.size() >= 2) {
        auto reply = buildReadReply(id, params[0], params[1]);
        if (!reply.empty()) writeMaster(masterFd, reply);
        return;
    }

    if (instr == INSTR_SYNC_READ && params.size() >= 3) {
        const uint8_t reg = params[0];
        const uint8_t size = params[1];
        for (std::size_t i = 2; i < params.size(); ++i) {
            auto reply = buildReadReply(params[i], reg, size);
            if (!reply.empty()) writeMaster(masterFd, reply);
        }
        return;
    }

    if (instr == 0x03 && !params.empty()) {
        applyWrite(id, params[0], std::vector<uint8_t>(params.begin() + 1, params.end()));
        return;
    }

    if (instr == 0x83 && params.size() >= 2) {
        const uint8_t reg = params[0];
        const uint8_t datalen = params[1];
        std::size_t off = 2;
        while (off + 1 + datalen <= params.size()) {
            const uint8_t sid = params[off];
            std::vector<uint8_t> d(params.begin() + off + 1,
                                   params.begin() + off + 1 + datalen);
            applyWrite(sid, reg, d);
            off += 1 + datalen;
        }
    }
}

void busThread(int masterFd)
{
    std::vector<uint8_t> buf;
    buf.reserve(256);
    uint8_t tmp[128];
    struct pollfd pfd{masterFd, POLLIN, 0};

    while (g_running) {
        int r = ::poll(&pfd, 1, 10);
        if (r <= 0) continue;
        if (!(pfd.revents & POLLIN)) continue;

        ssize_t n = ::read(masterFd, tmp, sizeof(tmp));
        if (n <= 0) continue;
        buf.insert(buf.end(), tmp, tmp + n);

        while (buf.size() >= 6) {
            while (buf.size() >= 2 && !(buf[0] == 0xFF && buf[1] == 0xFF)) {
                buf.erase(buf.begin());
            }
            if (buf.size() < 6) break;

            const uint8_t id = buf[2];
            const uint8_t len = buf[3];
            if (len < 2) {
                buf.erase(buf.begin());
                continue;
            }

            const std::size_t total = static_cast<std::size_t>(4 + len);
            if (buf.size() < total) break;

            const uint8_t instr = buf[4];
            std::vector<uint8_t> params(buf.begin() + 5, buf.begin() + 4 + len - 1);
            buf.erase(buf.begin(), buf.begin() + total);
            dispatchPacket(masterFd, id, instr, params);
        }
    }
}

void simThread()
{
    using clk = std::chrono::steady_clock;
    auto last = clk::now();

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        const auto now = clk::now();
        const double dt = std::chrono::duration<double>(now - last).count();
        last = now;

        std::lock_guard<std::mutex> lk(g_mutex);
        for (auto& [id, s] : g_servos) {
            (void)id;
            if (!s.torque_on) continue;
            int diff = static_cast<int>(s.target_pos) - static_cast<int>(s.current_pos);
            if (std::abs(diff) <= 1) {
                s.current_pos = s.target_pos;
                continue;
            }
            const double step = s.op_speed * dt;
            const int move = std::clamp(static_cast<int>(step), 1, std::abs(diff));
            s.current_pos = static_cast<int16_t>(s.current_pos + (diff > 0 ? move : -move));
        }
    }
}

}  // namespace fake_servo_bus
