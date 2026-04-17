#pragma once

#include "input/IExternalButton.hpp"

#include <atomic>
#include <thread>

namespace dgd {

// SPACE key toggles the demo authorisation. Q/ESC requests shutdown.
// Implemented non-blocking on Windows via _kbhit/_getch; on POSIX via
// a small tty raw-mode poll so developers can test the build on Linux.
class KeyboardButton final : public IExternalButton {
public:
    KeyboardButton();
    ~KeyboardButton() override;

    bool start() override;
    void stop()  override;
    bool isActive() const override { return active_.load(); }

    // Returns true once the user has requested shutdown.
    bool shutdownRequested() const { return shutdown_.load(); }

private:
    std::atomic<bool> active_   {false};
    std::atomic<bool> shutdown_ {false};
    std::atomic<bool> running_  {false};
    std::thread       poll_thread_;

    void pollLoop();
};

} // namespace dgd
