#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <queue>
#include <condition_variable>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <random>
#include <chrono>

/**
 * Simple WebSocket client for rosbridge
 * Supports ws:// protocol (no TLS)
 */
class WebSocketClient {
public:
    using MessageCallback = std::function<void(const std::string& message)>;

    WebSocketClient() : socket_fd_(-1), connected_(false), running_(false) {}

    ~WebSocketClient() {
        stop();
    }

    /**
     * Connect to WebSocket server
     * @param url WebSocket URL (e.g., "ws://localhost:9091")
     * @return true if connection successful
     */
    bool connect(const std::string& url) {
        // Parse URL
        if (!parseUrl(url)) {
            std::cerr << "[WebSocket] Invalid URL: " << url << std::endl;
            return false;
        }

        // Create socket
        socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (socket_fd_ < 0) {
            std::cerr << "[WebSocket] Failed to create socket: " << strerror(errno) << std::endl;
            return false;
        }

        // Set receive timeout
        struct timeval tv;
        tv.tv_sec = 5;
        tv.tv_usec = 0;
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        // Resolve hostname
        struct hostent* host = gethostbyname(host_.c_str());
        if (!host) {
            std::cerr << "[WebSocket] Failed to resolve host: " << host_ << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Connect
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port_);
        memcpy(&addr.sin_addr, host->h_addr, host->h_length);

        if (::connect(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "[WebSocket] Failed to connect to " << host_ << ":" << port_
                      << " - " << strerror(errno) << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        // Perform WebSocket handshake
        if (!performHandshake()) {
            std::cerr << "[WebSocket] Handshake failed" << std::endl;
            close(socket_fd_);
            socket_fd_ = -1;
            return false;
        }

        connected_ = true;
        std::cout << "[WebSocket] Connected to " << url << std::endl;
        return true;
    }

    /**
     * Subscribe to a ROS topic via rosbridge
     */
    bool subscribe(const std::string& topic, const std::string& type = "") {
        std::stringstream ss;
        ss << "{\"op\":\"subscribe\",\"topic\":\"" << topic << "\"";
        if (!type.empty()) {
            ss << ",\"type\":\"" << type << "\"";
        }
        ss << "}";

        if (sendText(ss.str())) {
            std::cout << "[WebSocket] Subscribed to " << topic << std::endl;
            return true;
        }
        return false;
    }

    /**
     * Send a text message
     */
    bool sendText(const std::string& message) {
        if (!connected_ || socket_fd_ < 0) {
            return false;
        }

        std::vector<uint8_t> frame;

        // Opcode: 0x81 = final frame, text
        frame.push_back(0x81);

        // Payload length with mask bit set (client must mask)
        size_t len = message.size();
        if (len <= 125) {
            frame.push_back(0x80 | static_cast<uint8_t>(len));
        } else if (len <= 65535) {
            frame.push_back(0x80 | 126);
            frame.push_back((len >> 8) & 0xFF);
            frame.push_back(len & 0xFF);
        } else {
            frame.push_back(0x80 | 127);
            for (int i = 7; i >= 0; i--) {
                frame.push_back((len >> (i * 8)) & 0xFF);
            }
        }

        // Generate random mask
        uint8_t mask[4];
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 255);
        for (int i = 0; i < 4; i++) {
            mask[i] = static_cast<uint8_t>(dis(gen));
            frame.push_back(mask[i]);
        }

        // Masked payload
        for (size_t i = 0; i < len; i++) {
            frame.push_back(message[i] ^ mask[i % 4]);
        }

        ssize_t sent = send(socket_fd_, frame.data(), frame.size(), 0);
        return sent == static_cast<ssize_t>(frame.size());
    }

    /**
     * Start receiving messages in background thread
     */
    void startAsync(MessageCallback callback) {
        callback_ = callback;
        running_ = true;
        receive_thread_ = std::thread(&WebSocketClient::receiveLoop, this);
    }

    /**
     * Blocking receive - returns false on timeout or error
     */
    bool receive(std::string& message, int timeout_ms = 1000) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        if (message_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                                  [this] { return !message_queue_.empty() || !running_; })) {
            if (!message_queue_.empty()) {
                message = std::move(message_queue_.front());
                message_queue_.pop();
                return true;
            }
        }
        return false;
    }

    void stop() {
        running_ = false;
        connected_ = false;

        if (socket_fd_ >= 0) {
            // Send close frame
            uint8_t close_frame[] = {0x88, 0x80, 0x00, 0x00, 0x00, 0x00};
            send(socket_fd_, close_frame, sizeof(close_frame), 0);
            close(socket_fd_);
            socket_fd_ = -1;
        }

        message_cv_.notify_all();

        if (receive_thread_.joinable()) {
            receive_thread_.join();
        }
    }

    bool isConnected() const { return connected_; }
    bool isRunning() const { return running_; }

private:
    bool parseUrl(const std::string& url) {
        // Expected format: ws://host:port/path
        if (url.substr(0, 5) != "ws://") {
            return false;
        }

        std::string rest = url.substr(5);

        // Find path
        size_t path_pos = rest.find('/');
        if (path_pos != std::string::npos) {
            path_ = rest.substr(path_pos);
            rest = rest.substr(0, path_pos);
        } else {
            path_ = "/";
        }

        // Find port
        size_t port_pos = rest.find(':');
        if (port_pos != std::string::npos) {
            host_ = rest.substr(0, port_pos);
            port_ = std::stoi(rest.substr(port_pos + 1));
        } else {
            host_ = rest;
            port_ = 80;
        }

        return !host_.empty();
    }

    bool performHandshake() {
        // Generate random key
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> dis(0, 255);

        uint8_t key_bytes[16];
        for (int i = 0; i < 16; i++) {
            key_bytes[i] = static_cast<uint8_t>(dis(gen));
        }
        std::string key = base64Encode(key_bytes, 16);

        // Build handshake request
        std::stringstream request;
        request << "GET " << path_ << " HTTP/1.1\r\n";
        request << "Host: " << host_ << ":" << port_ << "\r\n";
        request << "Upgrade: websocket\r\n";
        request << "Connection: Upgrade\r\n";
        request << "Sec-WebSocket-Key: " << key << "\r\n";
        request << "Sec-WebSocket-Version: 13\r\n";
        request << "\r\n";

        std::string req_str = request.str();
        if (send(socket_fd_, req_str.c_str(), req_str.size(), 0) < 0) {
            return false;
        }

        // Read response
        char buffer[1024];
        ssize_t received = recv(socket_fd_, buffer, sizeof(buffer) - 1, 0);
        if (received <= 0) {
            return false;
        }
        buffer[received] = '\0';

        // Check for upgrade confirmation
        std::string response(buffer);
        return response.find("101") != std::string::npos &&
               response.find("Upgrade") != std::string::npos;
    }

    void receiveLoop() {
        std::vector<uint8_t> recvBuffer(65536);
        std::vector<uint8_t> frameBuffer;  // Accumulates data across recv calls
        std::string messageAccumulator;    // For fragmented WebSocket messages

        std::cerr << "[WebSocket] receiveLoop started" << std::endl;

        while (running_ && connected_) {
            ssize_t received = recv(socket_fd_, recvBuffer.data(), recvBuffer.size(), 0);

            if (received <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;  // Timeout, retry
                }
                if (running_) {
                    std::cerr << "[WebSocket] Connection closed (errno=" << errno << ")" << std::endl;
                    connected_ = false;
                }
                break;
            }

            // Append received data to frame buffer
            frameBuffer.insert(frameBuffer.end(), recvBuffer.begin(), recvBuffer.begin() + received);

            // Parse complete WebSocket frames from buffer
            size_t pos = 0;
            while (pos < frameBuffer.size()) {
                // Need at least 2 bytes for header
                if (pos + 2 > frameBuffer.size()) break;

                uint8_t byte0 = frameBuffer[pos];
                uint8_t byte1 = frameBuffer[pos + 1];

                bool fin = (byte0 & 0x80) != 0;
                uint8_t opcode = byte0 & 0x0F;
                bool masked = (byte1 & 0x80) != 0;
                uint64_t payload_len = byte1 & 0x7F;

                size_t header_size = 2;

                // Extended payload length
                if (payload_len == 126) {
                    header_size += 2;
                    if (pos + header_size > frameBuffer.size()) break;
                    payload_len = (static_cast<uint64_t>(frameBuffer[pos + 2]) << 8) |
                                   static_cast<uint64_t>(frameBuffer[pos + 3]);
                } else if (payload_len == 127) {
                    header_size += 8;
                    if (pos + header_size > frameBuffer.size()) break;
                    payload_len = 0;
                    for (int i = 0; i < 8; i++) {
                        payload_len = (payload_len << 8) | frameBuffer[pos + 2 + i];
                    }
                }

                // Masking key
                if (masked) {
                    header_size += 4;
                }

                // Check if complete frame is available
                size_t total_frame_size = header_size + payload_len;
                if (pos + total_frame_size > frameBuffer.size()) {
                    // Incomplete frame, wait for more data
                    break;
                }

                // Extract mask if present
                uint8_t mask[4] = {0, 0, 0, 0};
                if (masked) {
                    size_t mask_pos = header_size - 4;
                    memcpy(mask, &frameBuffer[pos + mask_pos], 4);
                }

                // Extract and unmask payload
                std::string payload(payload_len, '\0');
                size_t payload_start = pos + header_size;
                for (uint64_t i = 0; i < payload_len; i++) {
                    payload[i] = frameBuffer[payload_start + i] ^ mask[i % 4];
                }

                pos += total_frame_size;

                // Handle frame
                switch (opcode) {
                    case 0x00:  // Continuation
                        messageAccumulator += payload;
                        if (fin) {
                            deliverMessage(messageAccumulator);
                            messageAccumulator.clear();
                        }
                        break;
                    case 0x01:  // Text
                        if (fin) {
                            deliverMessage(payload);
                        } else {
                            messageAccumulator = payload;
                        }
                        break;
                    case 0x02:  // Binary
                        if (fin) {
                            deliverMessage(payload);
                        }
                        break;
                    case 0x08:  // Close
                        connected_ = false;
                        running_ = false;
                        break;
                    case 0x09:  // Ping
                        sendPong(payload);
                        break;
                    case 0x0A:  // Pong
                        break;
                }
            }

            // Remove processed data from buffer
            if (pos > 0) {
                frameBuffer.erase(frameBuffer.begin(), frameBuffer.begin() + pos);
            }
        }
    }

    void deliverMessage(const std::string& message) {
        // Debug: show first part of each message
        static int msg_count = 0;
        if (msg_count < 20) {
            std::string preview = message.substr(0, 100);
            std::cerr << "[WS-RX] #" << msg_count << ": " << preview << "..." << std::endl;
            msg_count++;
        }

        if (callback_) {
            callback_(message);
        } else {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            message_queue_.push(message);
            message_cv_.notify_one();
        }
    }

    void sendPong(const std::string& payload) {
        std::vector<uint8_t> frame;
        frame.push_back(0x8A);  // Pong
        frame.push_back(0x80 | static_cast<uint8_t>(payload.size()));

        // Random mask
        uint8_t mask[4] = {0, 0, 0, 0};
        for (int i = 0; i < 4; i++) {
            frame.push_back(mask[i]);
        }

        for (size_t i = 0; i < payload.size(); i++) {
            frame.push_back(payload[i] ^ mask[i % 4]);
        }

        send(socket_fd_, frame.data(), frame.size(), 0);
    }

    static std::string base64Encode(const uint8_t* data, size_t len) {
        static const char* chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string result;

        for (size_t i = 0; i < len; i += 3) {
            uint32_t n = (static_cast<uint32_t>(data[i]) << 16);
            if (i + 1 < len) n |= (static_cast<uint32_t>(data[i + 1]) << 8);
            if (i + 2 < len) n |= static_cast<uint32_t>(data[i + 2]);

            result += chars[(n >> 18) & 0x3F];
            result += chars[(n >> 12) & 0x3F];
            result += (i + 1 < len) ? chars[(n >> 6) & 0x3F] : '=';
            result += (i + 2 < len) ? chars[n & 0x3F] : '=';
        }

        return result;
    }

    int socket_fd_;
    std::string host_;
    int port_;
    std::string path_;
    std::atomic<bool> connected_;
    std::atomic<bool> running_;

    MessageCallback callback_;
    std::thread receive_thread_;

    std::mutex queue_mutex_;
    std::condition_variable message_cv_;
    std::queue<std::string> message_queue_;
};

#endif // WEBSOCKET_CLIENT_H
