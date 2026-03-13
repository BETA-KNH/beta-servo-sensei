#include "ui.hpp"

#include <cstdlib>
#include <chrono>
#include <cstring>
#include <thread>

#include "common.hpp"

#ifdef HAVE_CURSES
#include <curses.h>
#include <ctime>
#endif

namespace fake_servo_bus {

#ifdef HAVE_CURSES
void uiThread()
{
    initscr();
    cbreak();
    noecho();
    curs_set(0);
    keypad(stdscr, FALSE);

    bool useColor = has_colors();
    if (useColor) {
        start_color();
        use_default_colors();
        init_pair(1, COLOR_GREEN, -1);
        init_pair(2, COLOR_YELLOW, -1);
        init_pair(3, COLOR_CYAN, -1);
        init_pair(4, COLOR_RED, -1);
        init_pair(5, COLOR_MAGENTA, -1);
    }

    while (g_running) {
        int screenRows = 0;
        int screenCols = 0;
        getmaxyx(stdscr, screenRows, screenCols);
        erase();

        if (useColor) attron(COLOR_PAIR(3) | A_BOLD);
        const char* title = " Fake Servo Bus Simulator";
        mvaddstr(0, 0, title);
        if (!g_displayPty.empty()) {
            std::string ptyLabel = "  PTY: " + g_displayPty;
            mvaddstr(0, static_cast<int>(std::strlen(title)), ptyLabel.c_str());
        }
        {
            const time_t now = time(nullptr);
            char tbuf[32];
            std::strftime(tbuf, sizeof(tbuf), "%H:%M:%S", std::localtime(&now));
            const int tlen = static_cast<int>(std::strlen(tbuf));
            if (screenCols > tlen) mvaddstr(0, screenCols - tlen - 1, tbuf);
        }
        if (useColor) attroff(COLOR_PAIR(3) | A_BOLD);

        if (useColor) attron(COLOR_PAIR(3));
        move(1, 0);
        for (int c = 0; c < screenCols; ++c) addch(ACS_HLINE);
        if (useColor) attroff(COLOR_PAIR(3));

        if (useColor) attron(A_BOLD);
        mvprintw(2, 0,
                 "  %3s   %-6s   %7s   %7s   %9s   %9s   %6s   %6s",
                 "ID", "Torque", "Pos", "Pos°", "Target", "Tgt°", "Speed", "Δ");
        if (useColor) attroff(A_BOLD);

        if (useColor) attron(COLOR_PAIR(3));
        move(3, 0);
        for (int c = 0; c < screenCols; ++c) addch(ACS_HLINE);
        if (useColor) attroff(COLOR_PAIR(3));

        int row = 4;
        {
            std::lock_guard<std::mutex> lk(g_mutex);
            for (auto& [id, s] : g_servos) {
                if (row >= screenRows - 2) break;

                const double pos_deg = s.current_pos * (360.0 / 4095.0);
                const double tgt_deg = s.target_pos * (360.0 / 4095.0);
                const int delta = static_cast<int>(s.target_pos) - static_cast<int>(s.current_pos);
                const bool moving = s.torque_on && std::abs(delta) > 1;

                if (useColor) {
                    if (moving) attron(COLOR_PAIR(2));
                    else if (s.torque_on) attron(COLOR_PAIR(1));
                    else attron(COLOR_PAIR(4) | A_DIM);
                }

                mvprintw(row,
                         0,
                         "  %3d   %-6s   %7d   %6.1f°   %9d   %8.1f°   %6d   %+6d",
                         static_cast<int>(id),
                         s.torque_on ? "ON" : "off",
                         static_cast<int>(s.current_pos),
                         pos_deg,
                         static_cast<int>(s.target_pos),
                         tgt_deg,
                         static_cast<int>(s.op_speed),
                         delta);

                if (useColor) attroff(COLOR_PAIR(1) | COLOR_PAIR(2) | COLOR_PAIR(4) | A_DIM);
                ++row;
            }
        }

        if (row < screenRows - 1) {
            if (useColor) attron(COLOR_PAIR(3));
            move(row, 0);
            for (int c = 0; c < screenCols; ++c) addch(ACS_HLINE);
            if (useColor) attroff(COLOR_PAIR(3));
            ++row;
        }

        if (row < screenRows) {
            if (useColor) attron(COLOR_PAIR(5));
            const pid_t ypid = g_yarpdevPid;
            if (ypid > 0) {
                mvprintw(row, 0, " yarpdev PID %d on %s", static_cast<int>(ypid), g_displayPortName.c_str());
                if (!g_yarpLogFile.empty()) {
                    int used = 22 + static_cast<int>(g_displayPortName.size());
                    if (screenCols > used + 8) mvprintw(row, used + 2, "log: %s", g_yarpLogFile.c_str());
                }
            } else {
                mvprintw(row, 0, " PTY: %s   port: %s", g_displayPty.c_str(), g_displayPortName.c_str());
            }
            const char* hint = " Ctrl+C to stop ";
            const int hlen = static_cast<int>(std::strlen(hint));
            if (screenCols > hlen) mvaddstr(row, screenCols - hlen, hint);
            if (useColor) attroff(COLOR_PAIR(5));
        }

        refresh();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    endwin();
}
#else
void uiThread() {}
#endif

}  // namespace fake_servo_bus
