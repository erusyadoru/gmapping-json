#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class UdpServer {
public:
    using MessageCallback = std::function<void(const std::string& data, const std::string& sender)>;

    UdpServer(int port, size_t buffer_size = 65536)
        : port_(port)
        , buffer_size_(buffer_size)
        , socket_fd_(-1)
        , running_(false)
    {}

    ~UdpServer() {
        stop();
    }

    bool start() {
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "[UDP] Failed to create socket: " << strerror(errno) << std::endl;
            return false;
        }

        // Allow address reuse
        int opt = 1;
        setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Increase receive buffer size to 16MB to handle burst traffic
        int rcvbuf_size = 16 * 1024 * 1024;  // 16MB
        if (setsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
            std::cerr << "[UDP] Warning: Failed to set receive buffer size: " << strerror(errno) << std::endl;
        } else {
            // Verify actual buffer size (kernel may limit it)
            int actual_size;
            socklen_t size_len = sizeof(actual_size);
            getsockopt(socket_fd_, SOL_SOCKET, SO_RCVBUF, &actual_size, &size_len);
            std::cout << "[UDP] Receive buffer size: " << actual_size / 1024 / 1024 << " MB" << std::endl;
        }

        // Set receive timeout
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);

        if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "[UDP] Failed to bind to port " << port_ << ": " << strerror(errno) << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        running_ = true;
        std::cout << "[UDP] Server listening on port " << port_ << std::endl;
        return true;
    }

    void stop() {
        running_ = false;
        if (socket_fd_ >= 0) {
            close(socket_fd_);
            socket_fd_ = -1;
        }
    }

    bool isRunning() const { return running_; }

    // Blocking receive - returns false on timeout or error
    bool receive(std::string& data, std::string& sender) {
        if (socket_fd_ < 0 || !running_) {
            return false;
        }

        std::vector<char> buffer(buffer_size_);
        struct sockaddr_in sender_addr;
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t recv_len = recvfrom(socket_fd_, buffer.data(), buffer.size() - 1,
                                    0, (struct sockaddr*)&sender_addr, &sender_len);

        if (recv_len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout - normal
                return false;
            }
            std::cerr << "[UDP] Receive error: " << strerror(errno) << std::endl;
            return false;
        }

        buffer[recv_len] = '\0';
        data = std::string(buffer.data(), recv_len);

        char sender_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, INET_ADDRSTRLEN);
        sender = std::string(sender_ip) + ":" + std::to_string(ntohs(sender_addr.sin_port));

        return true;
    }

    void setMessageCallback(MessageCallback callback) {
        callback_ = callback;
    }

    // Run receive loop (blocking)
    void runLoop() {
        std::string data;
        std::string sender;

        while (running_) {
            if (receive(data, sender)) {
                if (callback_) {
                    callback_(data, sender);
                }
            }
        }
    }

    // Run in background thread
    void startAsync() {
        receive_thread_ = std::thread(&UdpServer::runLoop, this);
    }

    void join() {
        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

private:
    int port_;
    size_t buffer_size_;
    int socket_fd_;
    std::atomic<bool> running_;
    MessageCallback callback_;
    std::thread receive_thread_;
};

#endif // UDP_SERVER_H
