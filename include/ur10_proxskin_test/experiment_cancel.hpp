#pragma once

#include <atomic>
#include <cerrno>
#include <fcntl.h>
#include <functional>
#include <poll.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <utility>

// Own the terminal only while a menu experiment runs. Restore it before exit so
// the launch menu can resume normal line input. No Enter is needed for q.
class ExperimentCancelInput
{
public:
    ExperimentCancelInput(bool enabled, std::function<void()> cancel)
    {
        if (!enabled) return;
        fd_ = open("/dev/tty", O_RDONLY | O_NONBLOCK | O_NOCTTY);
        if (fd_ < 0) return;
        if (tcgetattr(fd_, &saved_) != 0)
        {
            close(fd_);
            fd_ = -1;
            return;
        }
        termios mode = saved_;
        mode.c_lflag &= ~(ICANON | ECHO);
        mode.c_cc[VMIN] = 0;
        mode.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &mode) != 0)
        {
            close(fd_);
            fd_ = -1;
            return;
        }
        try
        {
            worker_ = std::thread([this, cancel = std::move(cancel)] {
                while (watching_)
                {
                    pollfd input{fd_, POLLIN, 0};
                    const int ready = poll(&input, 1, 25);
                    if (ready < 0)
                    {
                        if (errno == EINTR) continue;
                        return;
                    }
                    if (ready == 0) continue;
                    if (input.revents & (POLLHUP | POLLERR | POLLNVAL)) return;
                    if (!(input.revents & POLLIN)) continue;
                    char bytes[64];
                    const ssize_t count = read(fd_, bytes, sizeof(bytes));
                    for (ssize_t i = 0; i < count; ++i)
                        if (bytes[i] == 'q' || bytes[i] == 'Q')
                        {
                            requested_ = true;
                            cancel();
                            return;
                        }
                }
            });
        }
        catch (...)
        {
            tcsetattr(fd_, TCSANOW, &saved_);
            close(fd_);
            fd_ = -1;
            throw;
        }
    }

    ~ExperimentCancelInput() { stop(); }
    ExperimentCancelInput(const ExperimentCancelInput &) = delete;
    ExperimentCancelInput &operator=(const ExperimentCancelInput &) = delete;

    bool available() const { return fd_ >= 0; }
    bool requested() const { return requested_.load(); }

    void stop()
    {
        watching_ = false;
        if (worker_.joinable()) worker_.join();
        if (fd_ >= 0)
        {
            if (requested_) tcflush(fd_, TCIFLUSH);
            tcsetattr(fd_, TCSANOW, &saved_);
            close(fd_);
            fd_ = -1;
        }
    }

private:
    int fd_ = -1;
    termios saved_{};
    std::atomic<bool> watching_{true};
    std::atomic<bool> requested_{false};
    std::thread worker_;
};
