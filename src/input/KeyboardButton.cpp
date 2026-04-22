#include "input/KeyboardButton.hpp"
#include "util/Logger.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

#if defined(_WIN32)
  #include <conio.h>
#else
  #include <termios.h>
  #include <unistd.h>
  #include <sys/select.h>
#endif

namespace dgd {

KeyboardButton::KeyboardButton() {}
KeyboardButton::~KeyboardButton() { stop(); }

bool KeyboardButton::start() {
    if (running_.exchange(true)) return true;
    poll_thread_ = std::thread(&KeyboardButton::pollLoop, this);
    LOG_I("KeyboardButton: SPACE = toggle authorise, Q/ESC = quit.");
    return true;
}

void KeyboardButton::stop() {
    if (!running_.exchange(false)) return;
    if (poll_thread_.joinable()) poll_thread_.join();
}

#if defined(_WIN32)

void KeyboardButton::pollLoop() {
    while (running_.load()) {
        if (_kbhit()) {
            int c = _getch();
            if (c == ' ') {
                bool now = !active_.load();
                active_.store(now);
                LOG_I("External button toggled -> %s", now ? "ACTIVE" : "INACTIVE");
            } else if (c == 'q' || c == 'Q' || c == 27 /*ESC*/) {
                shutdown_.store(true);
                running_.store(false);
                return;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
}

#else

namespace {
struct TtyRaw {
    termios saved{};
    bool applied = false;
    TtyRaw() {
        if (!isatty(STDIN_FILENO)) return;
        tcgetattr(STDIN_FILENO, &saved);
        termios raw = saved;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        applied = true;
    }
    ~TtyRaw() {
        if (applied) tcsetattr(STDIN_FILENO, TCSANOW, &saved);
    }
};
} // namespace

void KeyboardButton::pollLoop() {
    TtyRaw raw;
    while (running_.load()) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv{0, 15'000};
        int sel = select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
        if (sel > 0 && FD_ISSET(STDIN_FILENO, &fds)) {
            char c;
            ssize_t n = read(STDIN_FILENO, &c, 1);
            if (n == 1) {
                if (c == ' ') {
                    bool now = !active_.load();
                    active_.store(now);
                    LOG_I("External button toggled -> %s", now ? "ACTIVE" : "INACTIVE");
                } else if (c == 'q' || c == 'Q' || c == 27 /*ESC*/) {
                    shutdown_.store(true);
                    running_.store(false);
                    return;
                }
            }
        }
    }
}

#endif

} // namespace dgd
