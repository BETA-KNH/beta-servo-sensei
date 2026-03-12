#ifndef PTY_HELPER_HPP
#define PTY_HELPER_HPP

#include <cerrno>
#include <cstring>
#include <string>
#include <stdexcept>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#ifdef __linux__
#  include <pty.h>     // grantpt, unlockpt, ptsname
#endif

// ---------------------------------------------------------------------------
// PtyHelper
//
// RAII wrapper that creates a linked pseudo-terminal (PTY) pair:
//
//   master fd  — held by the helper; used by tests to simulate a servo
//                (write fake response bytes here; read command bytes from here)
//
//   slave path — passed to SerialPortDriver; it sees a real tty device
//
// The PTY pair is entirely in kernel memory — no hardware or socat needed.
//
// Usage:
//   PtyHelper pty;
//   ASSERT_TRUE(pty.isOpen());
//
//   SerialPortDriver drv(pty.slavePath(), 115200, 100);
//
//   // Simulate servo: inject a fake response into the master end
//   pty.masterWrite({0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC});
//
//   // Driver reads it as if it came from real hardware
//   auto raw = drv.read(6);
// ---------------------------------------------------------------------------

class PtyHelper {
public:
    PtyHelper() : masterFd_(-1)
    {
        // Open a new PTY master
        masterFd_ = ::posix_openpt(O_RDWR | O_NOCTTY);
        if (masterFd_ < 0) return;

        if (::grantpt(masterFd_) != 0 || ::unlockpt(masterFd_) != 0) {
            ::close(masterFd_);
            masterFd_ = -1;
            return;
        }

        const char* name = ::ptsname(masterFd_);
        if (!name) {
            ::close(masterFd_);
            masterFd_ = -1;
            return;
        }
        slavePath_ = name;

        // Set master to raw mode so bytes pass through unmodified
        // (no echo, no line discipline transformations)
        struct termios tty{};
        if (::tcgetattr(masterFd_, &tty) == 0) {
            ::cfmakeraw(&tty);
            ::tcsetattr(masterFd_, TCSANOW, &tty);
        }
    }

    ~PtyHelper()
    {
        if (masterFd_ >= 0)
            ::close(masterFd_);
    }

    // Non-copyable, move-only
    PtyHelper(const PtyHelper&)            = delete;
    PtyHelper& operator=(const PtyHelper&) = delete;

    bool isOpen() const { return masterFd_ >= 0 && !slavePath_.empty(); }

    int masterFd() const { return masterFd_; }

    const std::string& slavePath() const { return slavePath_; }

    // Write bytes to the master end — the slave (SerialPortDriver) will read them.
    // Returns true if all bytes were written.
    bool masterWrite(const std::vector<uint8_t>& data) const
    {
        if (!isOpen()) return false;
        const auto* buf     = data.data();
        std::size_t remaining = data.size();
        while (remaining > 0) {
            ssize_t n = ::write(masterFd_, buf, remaining);
            if (n < 0) {
                if (errno == EINTR) continue;
                return false;
            }
            buf       += n;
            remaining -= static_cast<std::size_t>(n);
        }
        return true;
    }

    // Read exactly `n` bytes from the master end — these are bytes the slave
    // (SerialPortDriver) wrote, i.e. the outgoing command packet.  Loops
    // until all n bytes are available (handles cases where the writer issues
    // multiple small write() calls that arrive in separate kernel buffers).
    std::vector<uint8_t> masterRead(std::size_t n) const
    {
        if (!isOpen() || n == 0) return {};
        std::vector<uint8_t> buf;
        buf.reserve(n);
        std::vector<uint8_t> tmp(n);
        while (buf.size() < n) {
            ssize_t got = ::read(masterFd_, tmp.data(), n - buf.size());
            if (got <= 0) break;
            buf.insert(buf.end(), tmp.begin(), tmp.begin() + got);
        }
        return buf;
    }

private:
    int         masterFd_;
    std::string slavePath_;
};

#endif // PTY_HELPER_HPP
