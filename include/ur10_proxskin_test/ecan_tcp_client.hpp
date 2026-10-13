#pragma once

#include <arpa/inet.h>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "ur10_proxskin_test/ecan_frame.hpp"

// Adapted from Prox_sensor_React/src/EthernetCommunication.cpp's TCP client
// receive_loop. Keep the uint32 raw values, without Range/float conversion.
// The worker owns its socket; nonblocking I/O bounds cancellation and shutdown.
template<class Active, class Receive, class Reset>
void runEcanTcpClient(rclcpp::Node &node, const std::string &ip, int port,
                      double reconnect_seconds, double connect_timeout_seconds,
                      std::atomic<bool> &connected, Active active, Receive receive, Reset reset)
{
    const auto alive = [&]() { return rclcpp::ok() && active(); };
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, ip.c_str(), &address.sin_addr) != 1)
    {
        RCLCPP_ERROR(node.get_logger(), "Invalid eCAN server IP: %s", ip.c_str());
        return;
    }
    while (alive())
    {
        const int fd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
        if (fd < 0)
        {
            RCLCPP_ERROR(node.get_logger(), "Cannot create eCAN client socket");
            return;
        }
        bool ready = connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) == 0;
        if (!ready && errno == EINPROGRESS)
        {
            const auto deadline = std::chrono::steady_clock::now() +
                std::chrono::duration<double>(connect_timeout_seconds);
            while (alive() && std::chrono::steady_clock::now() < deadline)
            {
                pollfd socket{fd, POLLOUT, 0};
                const int result = poll(&socket, 1, 100);
                if (result < 0 && errno == EINTR) continue;
                if (result < 0) break;
                if (result == 0) continue;
                int error = 0;
                socklen_t size = sizeof(error);
                ready = getsockopt(fd, SOL_SOCKET, SO_ERROR, &error, &size) == 0 && error == 0;
                break;
            }
        }
        std::vector<uint8_t> frames;
        frames.reserve(4096);
        if (ready && alive())
        {
            reset();
            connected = true;
            RCLCPP_INFO(node.get_logger(), "Connected to eCAN server %s:%d", ip.c_str(), port);
            std::array<uint8_t, 1024> buffer{};
            while (alive())
            {
                pollfd socket{fd, POLLIN, 0};
                const int result = poll(&socket, 1, 100);
                if (result < 0 && errno == EINTR) continue;
                if (result < 0) break;
                if (result == 0) continue;
                const ssize_t count = recv(fd, buffer.data(), buffer.size(), 0);
                if (count < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) continue;
                if (count <= 0) break;
                frames.insert(frames.end(), buffer.begin(), buffer.begin() + count);
                if (frames.size() > 65536)
                {
                    frames.clear();
                    RCLCPP_WARN(node.get_logger(), "eCAN receive buffer overflow; discarded buffered data");
                    continue;
                }
                size_t consumed = 0;
                while (frames.size() - consumed >= 14)
                {
                    const auto frame = decodeEcanFrame(frames.data() + consumed, 14);
                    if (frame) receive(*frame);
                    consumed += 14;
                }
                frames.erase(frames.begin(), frames.begin() + consumed);
            }
        }
        connected = false;
        reset();
        close(fd);
        if (!alive()) break;
        RCLCPP_WARN_THROTTLE(node.get_logger(), *node.get_clock(), 5000,
            "eCAN disconnected; retrying %s:%d", ip.c_str(), port);
        const auto retry = std::chrono::steady_clock::now() + std::chrono::duration<double>(reconnect_seconds);
        while (alive() && std::chrono::steady_clock::now() < retry)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    connected = false;
}
