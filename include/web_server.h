#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <vector>

class WebServer {
public:
    struct ParticlePose {
        double x, y, theta;
        double weight;
    };

    struct RobotState {
        double x = 0, y = 0, theta = 0;
        std::vector<float> scan_ranges;
        float scan_angle_min = 0, scan_angle_max = 0;
        int map_width = 0, map_height = 0;
        double map_resolution = 0.05;
        double map_origin_x = 0, map_origin_y = 0;
        std::vector<int8_t> map_data;
        std::vector<ParticlePose> particles;
        int best_particle_idx = -1;
    };

    WebServer(int port = 8080) : port_(port), running_(false) {}

    ~WebServer() { stop(); }

    bool start() {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) return false;

        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(server_fd_);
            return false;
        }

        if (listen(server_fd_, 10) < 0) {
            close(server_fd_);
            return false;
        }

        running_ = true;
        server_thread_ = std::thread(&WebServer::serverLoop, this);
        std::cout << "[WebServer] Started on http://localhost:" << port_ << std::endl;
        return true;
    }

    void stop() {
        running_ = false;
        if (server_fd_ >= 0) {
            shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
            server_fd_ = -1;
        }
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
    }

    void updateState(const RobotState& state) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = state;
    }

private:
    void serverLoop() {
        while (running_) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);

            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(server_fd_, &readfds);

            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100ms timeout

            if (select(server_fd_ + 1, &readfds, NULL, NULL, &tv) <= 0) continue;

            int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
            if (client_fd < 0) continue;

            handleClient(client_fd);
            close(client_fd);
        }
    }

    void handleClient(int client_fd) {
        char buffer[4096];
        int n = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (n <= 0) return;
        buffer[n] = '\0';

        std::string request(buffer);

        if (request.find("GET /api/state") != std::string::npos) {
            sendJson(client_fd);
        } else if (request.find("GET /api/map") != std::string::npos) {
            sendMapPng(client_fd);
        } else {
            sendHtml(client_fd);
        }
    }

    void sendJson(int client_fd) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        std::ostringstream json;
        json << "{";
        json << "\"robot\":{\"x\":" << state_.x << ",\"y\":" << state_.y << ",\"theta\":" << state_.theta << "},";
        json << "\"map\":{\"width\":" << state_.map_width << ",\"height\":" << state_.map_height;
        json << ",\"resolution\":" << state_.map_resolution;
        json << ",\"origin_x\":" << state_.map_origin_x << ",\"origin_y\":" << state_.map_origin_y << "},";
        json << "\"scan\":{\"angle_min\":" << state_.scan_angle_min << ",\"angle_max\":" << state_.scan_angle_max;
        json << ",\"ranges\":[";
        for (size_t i = 0; i < state_.scan_ranges.size(); i += 10) { // Downsample
            if (i > 0) json << ",";
            float r = state_.scan_ranges[i];
            // Output null for infinity/nan values (not valid JSON)
            if (std::isfinite(r)) {
                json << r;
            } else {
                json << "null";
            }
        }
        json << "]},";

        // Add particles
        json << "\"particles\":[";
        for (size_t i = 0; i < state_.particles.size(); i++) {
            if (i > 0) json << ",";
            json << "{\"x\":" << state_.particles[i].x
                 << ",\"y\":" << state_.particles[i].y
                 << ",\"theta\":" << state_.particles[i].theta
                 << ",\"w\":" << state_.particles[i].weight << "}";
        }
        json << "],\"best_idx\":" << state_.best_particle_idx << "}";

        std::string body = json.str();
        std::ostringstream response;
        response << "HTTP/1.1 200 OK\r\n";
        response << "Content-Type: application/json\r\n";
        response << "Access-Control-Allow-Origin: *\r\n";
        response << "Content-Length: " << body.size() << "\r\n\r\n";
        response << body;

        std::string resp = response.str();
        send(client_fd, resp.c_str(), resp.size(), 0);
    }

    void sendMapPng(int client_fd) {
        std::lock_guard<std::mutex> lock(state_mutex_);

        // Create simple PGM in memory, then send as binary
        std::ostringstream pgm;
        pgm << "P5\n" << state_.map_width << " " << state_.map_height << "\n255\n";

        std::string header = pgm.str();
        std::vector<unsigned char> pixels(state_.map_width * state_.map_height);

        for (int y = state_.map_height - 1; y >= 0; --y) {
            for (int x = 0; x < state_.map_width; ++x) {
                int idx = y * state_.map_width + x;
                int8_t val = (idx < (int)state_.map_data.size()) ? state_.map_data[idx] : -1;
                unsigned char pixel;
                if (val < 0) pixel = 128;
                else if (val > 25) pixel = 0;
                else pixel = 255;
                pixels[(state_.map_height - 1 - y) * state_.map_width + x] = pixel;
            }
        }

        std::ostringstream response;
        response << "HTTP/1.1 200 OK\r\n";
        response << "Content-Type: image/x-portable-graymap\r\n";
        response << "Access-Control-Allow-Origin: *\r\n";
        response << "Content-Length: " << (header.size() + pixels.size()) << "\r\n\r\n";

        std::string resp = response.str();
        send(client_fd, resp.c_str(), resp.size(), 0);
        send(client_fd, header.c_str(), header.size(), 0);
        send(client_fd, pixels.data(), pixels.size(), 0);
    }

    void sendHtml(int client_fd) {
        std::string html = R"(<!DOCTYPE html>
<html>
<head>
    <title>GMapping Viewer</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { background: #222; color: #fff; font-family: monospace; }
        #container { width: 100vw; height: 100vh; position: relative; }
        #canvas { display: block; width: 100%; height: 100%; background: #333; }
        #info { position: fixed; top: 10px; left: 10px; background: rgba(0,0,0,0.8); padding: 10px; border-radius: 5px; z-index: 100; font-size: 14px; }
        #info div { margin: 4px 0; }
        #status { color: #0f0; }
    </style>
</head>
<body>
    <div id="container">
        <canvas id="canvas"></canvas>
        <div id="info">
            <div>Robot: <span id="pose">loading...</span></div>
            <div>Map: <span id="mapinfo">-</span></div>
            <div>Status: <span id="status">connecting...</span></div>
        </div>
    </div>
    <script>
        const canvas = document.getElementById('canvas');
        const ctx = canvas.getContext('2d');
        let mapCanvas = null;
        let state = null;

        // Zoom and pan state
        let zoom = 1.0;
        let panX = 0, panY = 0;
        let isDragging = false;
        let lastMouseX = 0, lastMouseY = 0;

        // Origin interpolation for smooth map transitions
        let displayOriginX = null, displayOriginY = null;
        let displayMapWidth = null, displayMapHeight = null;
        const originLerpFactor = 0.1;  // Smooth factor (0.1 = 10% per frame)

        function resize() {
            canvas.width = window.innerWidth;
            canvas.height = window.innerHeight;
        }
        window.addEventListener('resize', resize);
        resize();

        // Mouse wheel zoom
        canvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const rect = canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;

            const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
            const newZoom = Math.max(0.1, Math.min(20, zoom * zoomFactor));

            // Zoom toward mouse position
            panX = mouseX - (mouseX - panX) * (newZoom / zoom);
            panY = mouseY - (mouseY - panY) * (newZoom / zoom);
            zoom = newZoom;
        });

        // Mouse drag to pan
        canvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastMouseX = e.clientX;
            lastMouseY = e.clientY;
            canvas.style.cursor = 'grabbing';
        });

        canvas.addEventListener('mousemove', (e) => {
            if (isDragging) {
                panX += e.clientX - lastMouseX;
                panY += e.clientY - lastMouseY;
                lastMouseX = e.clientX;
                lastMouseY = e.clientY;
            }
        });

        canvas.addEventListener('mouseup', () => {
            isDragging = false;
            canvas.style.cursor = 'grab';
        });

        canvas.addEventListener('mouseleave', () => {
            isDragging = false;
            canvas.style.cursor = 'grab';
        });

        // Double click to reset view
        canvas.addEventListener('dblclick', () => {
            zoom = 1.0;
            panX = 0;
            panY = 0;
        });

        canvas.style.cursor = 'grab';

        async function fetchMap() {
            try {
                const resp = await fetch('/api/map');
                const arrayBuffer = await resp.arrayBuffer();
                const data = new Uint8Array(arrayBuffer);

                // Parse PGM header
                let idx = 0;
                let header = '';
                let newlines = 0;
                while (newlines < 3 && idx < data.length) {
                    const c = String.fromCharCode(data[idx++]);
                    header += c;
                    if (c === '\n') newlines++;
                }

                const parts = header.trim().split(/\s+/);
                if (parts[0] !== 'P5') return;
                const w = parseInt(parts[1]);
                const h = parseInt(parts[2]);

                if (w <= 0 || h <= 0) return;

                // Create off-screen canvas for map
                if (!mapCanvas || mapCanvas.width !== w || mapCanvas.height !== h) {
                    mapCanvas = document.createElement('canvas');
                    mapCanvas.width = w;
                    mapCanvas.height = h;
                }
                const mapCtx = mapCanvas.getContext('2d');
                const imgData = mapCtx.createImageData(w, h);

                // Copy pixels directly (C++ already flips Y in PGM generation)
                for (let i = 0; i < w * h && (idx + i) < data.length; i++) {
                    const v = data[idx + i];
                    imgData.data[i * 4 + 0] = v;
                    imgData.data[i * 4 + 1] = v;
                    imgData.data[i * 4 + 2] = v;
                    imgData.data[i * 4 + 3] = 255;
                }
                mapCtx.putImageData(imgData, 0, 0);
                document.getElementById('status').textContent = 'Map OK (' + w + 'x' + h + ')';
            } catch(e) {
                document.getElementById('status').textContent = 'Map error: ' + e.message;
            }
        }

        async function fetchState() {
            try {
                const resp = await fetch('/api/state');
                state = await resp.json();
                document.getElementById('pose').textContent =
                    '(' + state.robot.x.toFixed(2) + ', ' + state.robot.y.toFixed(2) + ', ' + (state.robot.theta * 180 / Math.PI).toFixed(1) + ')';
                document.getElementById('mapinfo').textContent =
                    state.map.width + 'x' + state.map.height + ' @ ' + state.map.resolution + 'm';
            } catch(e) {
                document.getElementById('status').textContent = 'State error: ' + e.message;
            }
        }

        function draw() {
            // Clear with dark background
            ctx.fillStyle = '#333';
            ctx.fillRect(0, 0, canvas.width, canvas.height);

            if (mapCanvas && state && state.map.width > 0) {
                // Initialize display origin on first frame
                if (displayOriginX === null) {
                    displayOriginX = state.map.origin_x;
                    displayOriginY = state.map.origin_y;
                    displayMapWidth = state.map.width;
                    displayMapHeight = state.map.height;
                }

                // Smoothly interpolate origin when map expands
                const targetOriginX = state.map.origin_x;
                const targetOriginY = state.map.origin_y;

                // Detect significant origin change (map expansion)
                const originDeltaX = targetOriginX - displayOriginX;
                const originDeltaY = targetOriginY - displayOriginY;
                const originChanged = Math.abs(originDeltaX) > 0.001 || Math.abs(originDeltaY) > 0.001;

                if (originChanged) {
                    // Lerp toward target origin
                    displayOriginX += originDeltaX * originLerpFactor;
                    displayOriginY += originDeltaY * originLerpFactor;
                    // Show indicator when interpolating
                    document.getElementById('status').textContent =
                        'Map expanding... origin delta: (' + originDeltaX.toFixed(3) + ', ' + originDeltaY.toFixed(3) + ')';
                }

                // Update display map size
                displayMapWidth = state.map.width;
                displayMapHeight = state.map.height;

                // Base scale to fit map in canvas
                const baseScale = Math.min(canvas.width / mapCanvas.width, canvas.height / mapCanvas.height) * 0.95;
                const scale = baseScale * zoom;

                // Center offset with pan
                const offsetX = (canvas.width - mapCanvas.width * baseScale) / 2 + panX;
                const offsetY = (canvas.height - mapCanvas.height * baseScale) / 2 + panY;

                // Apply zoom transform
                ctx.save();
                ctx.translate(canvas.width/2, canvas.height/2);
                ctx.scale(zoom, zoom);
                ctx.translate(-canvas.width/2, -canvas.height/2);
                ctx.translate(panX/zoom, panY/zoom);

                // Draw map (already Y-flipped in pixel data)
                const mapOffsetX = (canvas.width - mapCanvas.width * baseScale) / 2;
                const mapOffsetY = (canvas.height - mapCanvas.height * baseScale) / 2;
                ctx.drawImage(mapCanvas, mapOffsetX, mapOffsetY, mapCanvas.width * baseScale, mapCanvas.height * baseScale);

                // Robot position - direct world coordinates
                const robotMapX = (state.robot.x - displayOriginX) / state.map.resolution;
                const robotMapY = (state.robot.y - displayOriginY) / state.map.resolution;

                // Switch to Y-up coordinate system for robot and scan
                ctx.save();
                ctx.translate(mapOffsetX, mapOffsetY + mapCanvas.height * baseScale);
                ctx.scale(baseScale, -baseScale);  // Y-up, with scale

                // Robot position in map pixels (direct, no inversion)
                const robotX = robotMapX;
                const robotY = robotMapY;

                // Draw scan points - fully direct coordinates
                if (state.scan && state.scan.ranges && state.scan.ranges.length > 0) {
                    ctx.fillStyle = 'rgba(0, 255, 0, 0.8)';
                    const numRanges = state.scan.ranges.length;
                    const originalAngleStep = (state.scan.angle_max - state.scan.angle_min) / ((numRanges - 1) * 10);
                    const pointSize = 2 / zoom / baseScale;

                    for (let i = 0; i < numRanges; i++) {
                        const range = state.scan.ranges[i];
                        if (range > 0.1 && range < 10) {
                            // Direct: scan_angle + laser_theta(PI) + robot.theta
                            const scanAngle = state.scan.angle_min + i * 10 * originalAngleStep;
                            const angle = scanAngle + Math.PI + state.robot.theta;
                            const rangePixels = range / state.map.resolution;
                            // Direct coordinates - just cos/sin, no negation
                            const endX = robotX + Math.cos(angle) * rangePixels;
                            const endY = robotY + Math.sin(angle) * rangePixels;
                            ctx.beginPath();
                            ctx.arc(endX, endY, pointSize, 0, Math.PI * 2);
                            ctx.fill();
                        }
                    }
                }

                // Draw robot circle first (so particles appear on top)
                ctx.fillStyle = '#f00';
                ctx.beginPath();
                ctx.arc(robotX, robotY, 8 / zoom / baseScale, 0, Math.PI * 2);
                ctx.fill();

                // Draw robot direction - direct theta
                ctx.strokeStyle = '#ff0';
                ctx.lineWidth = 3 / zoom / baseScale;
                ctx.beginPath();
                ctx.moveTo(robotX, robotY);
                ctx.lineTo(robotX + Math.cos(state.robot.theta) * 20 / baseScale, robotY + Math.sin(state.robot.theta) * 20 / baseScale);
                ctx.stroke();

                // Draw particles on top of robot
                if (state.particles && state.particles.length > 0) {
                    const particleSize = 6 / zoom / baseScale;
                    for (let i = 0; i < state.particles.length; i++) {
                        const p = state.particles[i];
                        const px = (p.x - displayOriginX) / state.map.resolution;
                        const py = (p.y - displayOriginY) / state.map.resolution;

                        // Highlight best particle in cyan, others in orange
                        if (i === state.best_idx) {
                            ctx.fillStyle = '#0ff';  // Cyan for best
                            ctx.strokeStyle = '#0ff';
                        } else {
                            ctx.fillStyle = '#ffa500';  // Orange for others
                            ctx.strokeStyle = '#ffa500';
                        }

                        // Draw particle circle
                        ctx.beginPath();
                        ctx.arc(px, py, particleSize, 0, Math.PI * 2);
                        ctx.fill();

                        // Draw particle direction arrow
                        ctx.lineWidth = 2 / zoom / baseScale;
                        ctx.beginPath();
                        ctx.moveTo(px, py);
                        ctx.lineTo(px + Math.cos(p.theta) * 15 / baseScale, py + Math.sin(p.theta) * 15 / baseScale);
                        ctx.stroke();
                    }
                }

                ctx.restore();

                ctx.restore();

                // Draw zoom level indicator
                ctx.fillStyle = '#fff';
                ctx.font = '14px monospace';
                ctx.fillText('Zoom: ' + zoom.toFixed(1) + 'x (scroll to zoom, drag to pan, dblclick to reset)', 10, canvas.height - 10);
            } else {
                ctx.fillStyle = '#fff';
                ctx.font = '20px monospace';
                ctx.fillText('Waiting for map data...', 50, 100);
                if (state) {
                    ctx.fillText('Robot: (' + state.robot.x.toFixed(2) + ', ' + state.robot.y.toFixed(2) + ')', 50, 130);
                }
            }

            requestAnimationFrame(draw);
        }

        // Start fetching
        fetchState();
        fetchMap();
        setInterval(fetchState, 200);
        setInterval(fetchMap, 1000);
        draw();
        console.log('GMapping Viewer started');
    </script>
</body>
</html>)";

        std::ostringstream response;
        response << "HTTP/1.1 200 OK\r\n";
        response << "Content-Type: text/html\r\n";
        response << "Content-Length: " << html.size() << "\r\n\r\n";
        response << html;

        std::string resp = response.str();
        send(client_fd, resp.c_str(), resp.size(), 0);
    }

    int port_;
    int server_fd_ = -1;
    std::atomic<bool> running_;
    std::thread server_thread_;
    std::mutex state_mutex_;
    RobotState state_;
};

#endif // WEB_SERVER_H
