#ifndef SERIAL_PORT_CONFIG_HPP
#define SERIAL_PORT_CONFIG_HPP

#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

/**
 * @brief Serial port parameters needed to open the servo bus connection.
 *
 * Populated from environment variables so no recompile is needed when
 * targeting a different device or baud rate.
 *
 * Required environment variables (or entries in a servo.env file):
 *   - `SERVO_PORT`       — path to the serial device
 *   - `SERVO_BAUD`       — baud rate integer
 *
 * Optional:
 *   - `SERVO_TIMEOUT_MS` — read timeout in ms (default: 100)
 *
 * Env-file search order (first match per key wins, shell env always wins):
 *   1. `servo.env`                    (CWD)
 *   2. `conf/servo.env`               (CWD/conf — development layout)
 *   3. `/etc/servo_sensei/servo.env`  (system-wide install on embedded targets)
 */

namespace STServo {

struct SerialPortConfig {
    std::string port;      ///< Path to the serial device (e.g. `/dev/ttyUSB0`).
    int         baud;      ///< Baud rate in bits per second.
    int         timeoutMs; ///< Read timeout in milliseconds.

private:
    // Parse a KEY=VALUE env file. Lines starting with '#' or empty lines are
    // ignored. Only sets a variable if it is not already present in the
    // environment, so shell exports always win.
    static void loadEnvFile(const std::string& path)
    {
        std::ifstream f(path);
        if (!f.is_open()) return;

        std::string line;
        while (std::getline(f, line)) {
            // Strip leading whitespace
            const auto start = line.find_first_not_of(" \t");
            if (start == std::string::npos) continue;
            line = line.substr(start);

            if (line.empty() || line[0] == '#') continue;

            const auto eq = line.find('=');
            if (eq == std::string::npos) continue;

            std::string key   = line.substr(0, eq);
            std::string value = line.substr(eq + 1);

            // Only set if not already in the environment
            if (!std::getenv(key.c_str()))
                ::setenv(key.c_str(), value.c_str(), 0);
        }
    }

public:
    /// @brief Load configuration from environment variables.
    ///
    /// Env-file search order (first match per key wins; shell env always wins):
    ///   1. `servo.env`                    (CWD)
    ///   2. `conf/servo.env`               (CWD/conf — development layout)
    ///   3. `/etc/servo_sensei/servo.env`  (system-wide install on embedded targets)
    ///
    /// @throws std::runtime_error  If any required variable (`SERVO_PORT`,
    ///                             `SERVO_BAUD`) is missing.
    static SerialPortConfig fromEnv()
    {
        loadEnvFile("servo.env");
        loadEnvFile("conf/servo.env");
        loadEnvFile("/etc/servo_sensei/servo.env");

        const char* envPort    = std::getenv("SERVO_PORT");
        const char* envBaud    = std::getenv("SERVO_BAUD");
        const char* envTimeout = std::getenv("SERVO_TIMEOUT_MS");

        // Collect every missing required variable before reporting.
        std::string missing;
        if (!envPort) missing += "  SERVO_PORT\n";
        if (!envBaud) missing += "  SERVO_BAUD\n";

        if (!missing.empty()) {
            throw std::runtime_error(
                "Missing required environment variable(s):\n" + missing +
                "Set them in your shell, in conf/servo.env (development), "
                "or in /etc/servo_sensei/servo.env (embedded target)."
            );
        }

        SerialPortConfig cfg;
        cfg.port      = envPort;
        cfg.baud      = std::stoi(envBaud);
        cfg.timeoutMs = envTimeout ? std::stoi(envTimeout) : 100;

        if (cfg.baud <= 0)
            throw std::invalid_argument("SERVO_BAUD must be a positive integer");
        if (cfg.timeoutMs < 0)
            throw std::invalid_argument("SERVO_TIMEOUT_MS must be >= 0");

        return cfg;
    }
};

} // namespace STServo

#endif // SERIAL_PORT_CONFIG_HPP
