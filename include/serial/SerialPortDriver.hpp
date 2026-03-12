#ifndef SERIAL_PORT_DRIVER_HPP
#define SERIAL_PORT_DRIVER_HPP

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include "serial/SerialPortConfig.hpp"

/**
 * @brief RAII wrapper around a POSIX termios serial port file descriptor.
 *
 * Configured for 8N1 raw mode with a blocking read timeout supplied at
 * construction time. Designed for half-duplex single-wire UART bus servo
 * communication (e.g. ST3215, ST3025 at up to 1 Mbps).
 *
 * Non-copyable, move-only.
 *
 * @par Example
 * @code
 *   auto cfg = STServo::SerialPortConfig::fromEnv();
 *   SerialPortDriver port(cfg);
 *   if (!port.isOpen()) { return; }
 *   port.write(STServoRequest::ping(1));
 *   auto raw  = port.read(6);
 *   auto resp = STServo::ServoResponse::parse(raw);
 * @endcode
 */

class SerialPortDriver {
public:
    /// @brief Construct from a @ref STServo::SerialPortConfig.
    /// @param cfg  Port, baud rate, and timeout configuration.
    explicit SerialPortDriver(const STServo::SerialPortConfig& cfg)
        : fd_(-1), port_(cfg.port), timeoutMs_(cfg.timeoutMs)
    {
        open(cfg.port, cfg.baud, cfg.timeoutMs);
    }

    /// @brief Construct directly from individual parameters.
    /// @param port       Path to the serial device.
    /// @param baud       Baud rate in bits per second.
    /// @param timeoutMs  Read timeout in milliseconds.
    SerialPortDriver(const std::string& port, int baud, int timeoutMs)
        : fd_(-1), port_(port), timeoutMs_(timeoutMs)
    {
        open(port, baud, timeoutMs);
    }

    ~SerialPortDriver()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    // Non-copyable
    SerialPortDriver(const SerialPortDriver&)            = delete;
    SerialPortDriver& operator=(const SerialPortDriver&) = delete;

    // Movable
    SerialPortDriver(SerialPortDriver&& other) noexcept
        : fd_(other.fd_), port_(std::move(other.port_)), timeoutMs_(other.timeoutMs_)
    {
        other.fd_ = -1;
    }

    SerialPortDriver& operator=(SerialPortDriver&& other) noexcept
    {
        if (this != &other) {
            if (fd_ >= 0) ::close(fd_);
            fd_         = other.fd_;
            port_       = std::move(other.port_);
            timeoutMs_  = other.timeoutMs_;
            other.fd_   = -1;
        }
        return *this;
    }

    /// @brief Returns @c true if the port was opened successfully.
    bool isOpen() const { return fd_ >= 0; }

    /// @brief Returns the device path this instance was opened with.
    const std::string& portPath() const { return port_; }

    /// @brief Write a packet to the serial port.
    /// @param packet  Bytes to transmit.
    /// @return @c true if all bytes were written, @c false on error.
    bool write(const std::vector<uint8_t>& packet) const
    {
        if (!isOpen()) return false;

        const auto* buf = packet.data();
        std::size_t remaining = packet.size();

        while (remaining > 0) {
            ssize_t n = ::write(fd_, buf, remaining);
            if (n < 0) {
                if (errno == EINTR) continue;  // interrupted — retry
                return false;
            }
            buf       += n;
            remaining -= static_cast<std::size_t>(n);
        }
        return true;
    }

    /// @brief Read up to @p expectedBytes bytes from the serial port.
    ///
    /// Blocks until all bytes arrive or the configured timeout elapses.
    /// Uses poll(2) for the wait so the timeout is reliable on both real
    /// serial ports and PTY pairs (VTIME on a PTY is not guaranteed by
    /// the kernel tty line discipline on all configurations).
    ///
    /// @param expectedBytes  Maximum number of bytes to read.
    /// @return               Bytes received (may be fewer than requested on timeout).
    std::vector<uint8_t> read(std::size_t expectedBytes) const
    {
        if (!isOpen() || expectedBytes == 0)
            return {};

        std::vector<uint8_t> buf(expectedBytes);
        std::size_t received = 0;

        while (received < expectedBytes) {
            // Wait for data with an explicit timeout via poll()
            struct pollfd pfd{fd_, POLLIN, 0};
            int r = ::poll(&pfd, 1, timeoutMs_);
            if (r < 0) {
                if (errno == EINTR) continue;  // signal — retry
                break;
            }
            if (r == 0) break;  // timeout expired
            if (!(pfd.revents & POLLIN)) break;  // unexpected event

            ssize_t n = ::read(fd_,
                               buf.data() + received,
                               expectedBytes - received);
            if (n < 0) {
                if (errno == EINTR) continue;  // interrupted — retry
                break;
            }
            if (n == 0) break;
            received += static_cast<std::size_t>(n);
        }

        buf.resize(received);
        return buf;
    }

private:
    int         fd_;
    std::string port_;
    int         timeoutMs_ = 50;

    // ------------------------------------------------------------------
    // Internal setup
    // ------------------------------------------------------------------

    // Map an integer baud rate to the corresponding Bxxx POSIX constant.
    // Throws std::invalid_argument for unsupported rates.
    static speed_t baudToConstant(int baud)
    {
        switch (baud) {
            case 9600:    return B9600;
            case 19200:   return B19200;
            case 38400:   return B38400;
            case 57600:   return B57600;
            case 115200:  return B115200;
            case 230400:  return B230400;
            case 460800:  return B460800;
            case 500000:  return B500000;
            case 576000:  return B576000;
            case 921600:  return B921600;
            case 1000000: return B1000000;
            case 1152000: return B1152000;
            case 1500000: return B1500000;
            case 2000000: return B2000000;
            case 2500000: return B2500000;
            case 3000000: return B3000000;
            default:
                throw std::invalid_argument(
                    "Unsupported baud rate: " + std::to_string(baud));
        }
    }

    void open(const std::string& port, int baud, int timeoutMs)
    {
        // Validate baud rate before touching the fd — throws for unknown values
        const speed_t speed = baudToConstant(baud);

        // O_NOCTTY  — don't make this the controlling terminal
        // O_NDELAY  — don't block on open even if DCD is not asserted
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) return;  // caller checks isOpen()

        // Restore blocking behaviour for reads
        ::fcntl(fd_, F_SETFL, 0);

        // ---- termios configuration ----
        struct termios tty{};
        if (::tcgetattr(fd_, &tty) != 0) {
            ::close(fd_);
            fd_ = -1;
            return;
        }

        // Raw mode: no line processing, no signals, no special characters
        ::cfmakeraw(&tty);

        // 8 data bits, no parity, 1 stop bit (8N1)
        tty.c_cflag &= ~static_cast<tcflag_t>(CSIZE);
        tty.c_cflag |=  CS8;
        tty.c_cflag &= ~static_cast<tcflag_t>(PARENB);
        tty.c_cflag &= ~static_cast<tcflag_t>(CSTOPB);

        // Disable hardware flow control
        tty.c_cflag &= ~static_cast<tcflag_t>(CRTSCTS);

        // Enable receiver, local mode (ignore modem control lines)
        tty.c_cflag |= (CLOCAL | CREAD);

        // Baud rate (already resolved above)
        ::cfsetispeed(&tty, speed);
        ::cfsetospeed(&tty, speed);

        // Non-blocking reads — poll() in read() provides the timeout.
        // VMIN=0, VTIME=0: read() returns immediately with whatever is buffered.
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;

        if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
            ::close(fd_);
            fd_ = -1;
        }
    }
};

#endif // SERIAL_PORT_DRIVER_HPP
