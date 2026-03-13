#include <chrono>
#include <csignal>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <cstdint>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include "PtyHelper.hpp"
#include "common.hpp"
#include "config.hpp"
#include "sim_bus.hpp"
#include "ui.hpp"

namespace fake_servo_bus {

static void onSignal(int)
{
    g_running = false;
    if (g_yarpdevPid > 0) ::kill(g_yarpdevPid, SIGTERM);
}

}  // namespace fake_servo_bus

int main(int argc, char** argv)
{
    using namespace fake_servo_bus;

    Options options;
    std::string parseError;
    bool wantHelp = false;
    if (!parseArgs(argc, argv, options, parseError, wantHelp)) {
        std::cerr << "[fake_servo_bus] ERROR: " << parseError << "\n\n";
        printUsage(argv[0]);
        return 1;
    }
    if (wantHelp) {
        printUsage(argv[0]);
        return 0;
    }

    for (uint8_t id : options.ids) g_servos[id];

    PtyHelper pty;
    if (!pty.isOpen()) {
        std::cerr << "[fake_servo_bus] ERROR: could not create PTY\n";
        return 1;
    }

    std::cout << "[fake_servo_bus] Slave PTY: " << pty.slavePath() << "\n";
    std::cout << "[fake_servo_bus] Simulating servo IDs:";
    for (auto id : options.ids) std::cout << " " << static_cast<int>(id);
    std::cout << "\n";

    g_displayPty = pty.slavePath();
    g_displayPortName = options.portName;

#ifdef HAVE_CURSES
    const bool useUi = !options.noUi && ::isatty(STDOUT_FILENO);
#else
    const bool useUi = false;
#endif

    struct sigaction sa {};
    sa.sa_handler = onSignal;
    ::sigaction(SIGINT, &sa, nullptr);
    ::sigaction(SIGTERM, &sa, nullptr);
    ::sigaction(SIGCHLD, &sa, nullptr);

    std::thread bus(busThread, pty.masterFd());
    std::thread sim(simThread);

    const std::string idsStr = buildIdsString(options.ids);

    if (options.launchYarp) {
        const pid_t pid = ::fork();
        if (pid < 0) {
            std::cerr << "[fake_servo_bus] fork() failed\n";
            g_running = false;
        } else if (pid == 0) {
            if (useUi) {
                g_yarpLogFile = "/tmp/fake_servo_bus_yarpdev.log";
                int logFd = ::open(g_yarpLogFile.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
                if (logFd >= 0) {
                    ::dup2(logFd, STDOUT_FILENO);
                    ::dup2(logFd, STDERR_FILENO);
                    ::close(logFd);
                }
            }

            ::execlp("yarpdev",
                     "yarpdev",
                     "--from",
                     (options.fromFile.empty() ? "conf/st_servo.ini" : options.fromFile.c_str()),
                     "--name",
                     options.portName.c_str(),
                     "--port",
                     pty.slavePath().c_str(),
                     "--baud",
                     options.baud.c_str(),
                     "--servo_ids",
                     idsStr.c_str(),
                     nullptr);
            std::cerr << "[fake_servo_bus] exec yarpdev failed: " << std::strerror(errno) << "\n";
            ::_exit(1);
        } else {
            g_yarpdevPid = pid;
            if (useUi) {
                g_yarpLogFile = "/tmp/fake_servo_bus_yarpdev.log";
            } else {
                std::cout << "[fake_servo_bus] yarpdev PID " << pid << " started\n";
                std::cout << "[fake_servo_bus] Loaded "
                          << (options.fromFile.empty() ? "conf/st_servo.ini" : options.fromFile)
                          << " with PTY overrides\n";
                std::cout << "[fake_servo_bus] YARP ports on: " << options.portName << "\n";
                std::cout << "[fake_servo_bus] Connect with:  yarpmotorgui --robot " << options.portName << "\n";
                std::cout << "[fake_servo_bus] Press Ctrl+C to stop.\n";
            }
        }
    } else if (!useUi) {
        std::cout << "[fake_servo_bus] Bus is live on " << pty.slavePath() << "\n";
        std::cout << "[fake_servo_bus] Run yarpdev yourself:\n"
                  << "  yarpdev --from "
                  << (options.fromFile.empty() ? "conf/st_servo.ini" : options.fromFile) << " \\\n"
                  << "          --name " << options.portName << " \\\n"
                  << "          --port " << pty.slavePath() << " \\\n"
                  << "          --baud " << options.baud << " \\\n"
                  << "          --servo_ids \"" << idsStr << "\"\n";
        std::cout << "[fake_servo_bus] After yarpdev starts, connect with: yarpmotorgui --robot "
                  << options.portName << "\n";
        std::cout << "[fake_servo_bus] Press Ctrl+C to stop the simulator.\n";
    }

#ifdef HAVE_CURSES
    if (useUi) {
        uiThread();
    } else
#endif
    {
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::milliseconds(options.launchYarp ? 2000 : 500));
            if (!g_running) break;
            if (options.launchYarp) {
                std::cout << "[sim]";
                std::lock_guard<std::mutex> lk(g_mutex);
                for (auto& [id, s] : g_servos) {
                    std::printf("  ID%d: pos=%.1f\xc2\xb0 tgt=%.1f\xc2\xb0 torque=%s",
                                static_cast<int>(id),
                                s.current_pos * (360.0 / 4095.0),
                                s.target_pos * (360.0 / 4095.0),
                                s.torque_on ? "ON" : "off");
                }
                std::cout << "\n";
            }
        }
    }

    if (options.launchYarp && g_yarpdevPid > 0) {
        int status = 0;
        ::waitpid(g_yarpdevPid, &status, 0);
        g_yarpdevPid = -1;
    }

    g_running = false;
    {
        int slaveFd = ::open(pty.slavePath().c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
        if (slaveFd >= 0) {
            ::write(slaveFd, "\x00", 1);
            ::close(slaveFd);
        }
    }

    bus.join();
    sim.join();
    std::cout << "[fake_servo_bus] Bye.\n";
    return 0;
}
