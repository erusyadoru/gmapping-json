#include <iostream>
#include <iomanip>
#include <string>
#include <cstring>
#include <signal.h>
#include <mutex>
#include <optional>
#include <getopt.h>
#include <deque>
#include <cmath>
#include <queue>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <chrono>

#include "json.hpp"

// デバッグ用タイマー
class ScopedTimer {
public:
    ScopedTimer(const char* name, bool enabled = true)
        : name_(name), enabled_(enabled), start_(std::chrono::high_resolution_clock::now()) {}

    ~ScopedTimer() {
        if (enabled_) {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count();
            std::cout << "[Timer] " << name_ << ": " << duration / 1000.0 << " ms" << std::endl;
        }
    }

    double elapsed_ms() const {
        auto end = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start_).count() / 1000.0;
    }

private:
    const char* name_;
    bool enabled_;
    std::chrono::high_resolution_clock::time_point start_;
};

// グローバルデバッグフラグ
static bool g_timing_debug = false;
#include "rosbridge_types.h"
#include "gmapping_wrapper.h"
#include "udp_server.h"
#include "websocket_client.h"
#include "web_server.h"
#include "rosbag_writer.h"

using json = nlohmann::json;

// Odom buffer size (keep ~2 seconds of odom at 20Hz = 40 samples)
constexpr size_t ODOM_BUFFER_SIZE = 100;

// Message queue size for buffering UDP packets
constexpr size_t MESSAGE_QUEUE_SIZE = 10000;

// Thread-safe message queue
class MessageQueue {
public:
    void push(const std::string& data) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (queue_.size() < MESSAGE_QUEUE_SIZE) {
            queue_.push(data);
            total_received_++;
        } else {
            dropped_++;
            if (dropped_ % 100 == 1) {
                std::cerr << "[Queue] Dropped " << dropped_ << " messages (queue full)" << std::endl;
            }
        }
        cv_.notify_one();
    }

    bool pop(std::string& data, int timeout_ms = 100) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms),
                         [this] { return !queue_.empty() || shutdown_; })) {
            if (!queue_.empty()) {
                data = std::move(queue_.front());
                queue_.pop();
                return true;
            }
        }
        return false;
    }

    void shutdown() {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        cv_.notify_all();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }

    void printStats() const {
        std::cout << "[Queue] Total received: " << total_received_
                  << ", dropped: " << dropped_
                  << ", pending: " << queue_.size() << std::endl;
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::string> queue_;
    std::atomic<size_t> total_received_{0};
    std::atomic<size_t> dropped_{0};
    bool shutdown_ = false;
};

// Global flag for shutdown
volatile sig_atomic_t g_shutdown = 0;

void signalHandler(int sig) {
    (void)sig;
    g_shutdown = 1;
    std::cout << "\n[Main] Shutdown requested..." << std::endl;
}

// Message processor class with timestamp synchronization
class SlamProcessor {
public:
    SlamProcessor(const GmappingWrapper::Config& config)
        : config_(config), gmapping_(config)
    {}

    void processMessage(const std::string& data, const std::string& sender) {
        ScopedTimer total_timer("processMessage total", g_timing_debug);
        try {
            ScopedTimer parse_timer("JSON parse", g_timing_debug);
            json j = json::parse(data);
            if (g_timing_debug) {
                std::cout << "[Timer] JSON size: " << data.size() << " bytes" << std::endl;
            }

            // Parse rosbridge message
            rosbridge::RosbridgeMessage msg;
            from_json(j, msg);

            if (msg.op == "publish") {
                processPublish(msg);
            }
        } catch (const json::parse_error& e) {
            std::cerr << "[Parser] JSON parse error from " << sender << ": " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[Parser] Error processing message: " << e.what() << std::endl;
        }
    }

    void saveMap() {
        gmapping_.saveMap();
    }

    void enableScanLogging(const std::string& path) {
        gmapping_.enableScanLogging(path);
    }

    void saveScanLog() {
        gmapping_.saveScanLog();
    }

    // Enable rosbag recording
    void enableBagRecording(const std::string& bag_path) {
        bag_writer_ = std::make_unique<RosbagWriter>();
        if (!bag_writer_->open(bag_path)) {
            bag_writer_.reset();
            std::cerr << "[Recording] Failed to open bag file: " << bag_path << std::endl;
        }
    }

    void closeBagRecording() {
        if (bag_writer_) {
            bag_writer_->close();
            bag_writer_.reset();
        }
    }

    bool isInitialized() const {
        return gmapping_.isInitialized();
    }

    // Get best particle index for logging
    int getBestParticleIndex() const {
        return gmapping_.getBestParticleIndex();
    }

    // Get state for web visualization
    WebServer::RobotState getWebState() {
        std::lock_guard<std::mutex> lock(mutex_);
        WebServer::RobotState state;

        // Robot pose
        auto pose = gmapping_.getCurrentPose();
        state.x = pose.x;
        state.y = pose.y;
        state.theta = pose.theta;

        // Last scan
        state.scan_ranges = last_scan_ranges_;
        state.scan_angle_min = last_scan_angle_min_;
        state.scan_angle_max = last_scan_angle_max_;

        // Map data
        auto map = gmapping_.getMapData();
        state.map_width = map.width;
        state.map_height = map.height;
        state.map_resolution = map.resolution;
        state.map_origin_x = map.origin_x;
        state.map_origin_y = map.origin_y;
        state.map_data = map.data;

        // Particle data
        auto particles = gmapping_.getParticlePoses();
        state.particles.reserve(particles.size());
        for (const auto& p : particles) {
            WebServer::ParticlePose pp;
            pp.x = p.x;
            pp.y = p.y;
            pp.theta = p.theta;
            pp.weight = p.weight;
            state.particles.push_back(pp);
        }
        state.best_particle_idx = gmapping_.getBestParticleIndex();

        return state;
    }

private:
    void processPublish(const rosbridge::RosbridgeMessage& msg) {
        static int topic_debug_count = 0;
        if (topic_debug_count < 10) {
            std::cout << "[Topic] Received: " << msg.topic << std::endl;
            topic_debug_count++;
        }

        if (msg.topic == "/scan" || msg.topic == "scan" ||
            msg.topic.find("scan") != std::string::npos) {
            processScan(msg.msg);
        } else if (msg.topic == "/odom" || msg.topic == "odom" ||
                   msg.topic.find("odom") != std::string::npos) {
            processOdom(msg.msg);
        }
    }

    void processScan(const json& msg) {
        ScopedTimer total_timer("processScan total", g_timing_debug);
        std::lock_guard<std::mutex> lock(mutex_);

        // Parse LaserScan message
        rosbridge::LaserScan scan;
        {
            ScopedTimer t("LaserScan from_json", g_timing_debug);
            from_json(msg, scan);
        }

        // Store for web visualization
        last_scan_ranges_ = scan.ranges;
        last_scan_angle_min_ = scan.angle_min;
        last_scan_angle_max_ = scan.angle_max;

        // Find closest odom by timestamp
        if (odom_buffer_.empty()) {
            std::cerr << "[Sync] No odom data available, skipping scan" << std::endl;
            return;
        }

        double scan_time = scan.header.stamp.toSec();

        // Find odom with closest timestamp
        auto closest_it = odom_buffer_.begin();
        double min_diff = std::abs(closest_it->header.stamp.toSec() - scan_time);

        for (auto it = odom_buffer_.begin(); it != odom_buffer_.end(); ++it) {
            double diff = std::abs(it->header.stamp.toSec() - scan_time);
            if (diff < min_diff) {
                min_diff = diff;
                closest_it = it;
            }
        }

        // Report sync status (first few times)
        if (sync_report_count_ < 15) {
            double time_diff_ms = min_diff * 1000.0;
            double odom_first = odom_buffer_.front().header.stamp.toSec();
            double odom_last = odom_buffer_.back().header.stamp.toSec();
            double closest_time = closest_it->header.stamp.toSec();
            // Use relative time from first odom for clearer display
            static double base_time = odom_first;
            std::cout << std::fixed << std::setprecision(3);
            std::cout << "[Sync] scan_t=" << (scan_time - base_time)
                      << " odom_range=[" << (odom_first - base_time) << "," << (odom_last - base_time) << "]"
                      << " closest_t=" << (closest_time - base_time)
                      << " diff=" << time_diff_ms << "ms"
                      << " (buf=" << odom_buffer_.size() << ")" << std::endl;
            sync_report_count_++;
        }

        // Check if time difference is too large (> 500ms)
        if (min_diff > 0.5) {
            std::cerr << "[Sync] WARNING: Large time diff " << (min_diff * 1000) << "ms, skipping scan" << std::endl;
            return;
        }

        // Process with synchronized odom
        bool processed;
        {
            ScopedTimer t("gmapping.processScan", g_timing_debug);
            processed = gmapping_.processScan(scan, *closest_it);
        }

        // Record to bag if enabled and scan was processed
        if (processed && bag_writer_ && bag_writer_->isOpen()) {
            auto pose = gmapping_.getCurrentPose();
            double timestamp = scan.header.stamp.toSec();

            // Write tf_static: base_link -> scan (only once)
            if (!tf_static_written_) {
                RosbagWriter::TransformStamped tf_static;
                tf_static.header.seq = 0;
                tf_static.header.stamp = RosbagWriter::Time(timestamp);
                tf_static.header.frame_id = "base_link";
                tf_static.child_frame_id = "scan";
                tf_static.transform.translation = RosbagWriter::Vector3(
                    config_.laser_x, config_.laser_y, 0.0);
                tf_static.transform.rotation = RosbagWriter::Quaternion::fromYaw(
                    config_.laser_theta);
                bag_writer_->writeTFStatic(tf_static);
                tf_static_written_ = true;
                std::cout << "[Recording] Wrote tf_static: base_link -> scan (offset: "
                          << config_.laser_x << ", " << config_.laser_y << ", "
                          << config_.laser_theta << ")" << std::endl;
            }

            // Write TF: map -> base_link
            RosbagWriter::TransformStamped tf;
            tf.header.seq = bag_seq_;
            tf.header.stamp = RosbagWriter::Time(timestamp);
            tf.header.frame_id = "map";
            tf.child_frame_id = "base_link";
            tf.transform.translation = RosbagWriter::Vector3(pose.x, pose.y, 0.0);
            tf.transform.rotation = RosbagWriter::Quaternion::fromYaw(pose.theta);
            bag_writer_->writeTF(tf);

            // Write LaserScan with frame_id = "scan"
            RosbagWriter::LaserScan bag_scan;
            bag_scan.header.seq = bag_seq_;
            bag_scan.header.stamp = RosbagWriter::Time(timestamp);
            bag_scan.header.frame_id = "scan";  // laser frame with offset from base_link
            bag_scan.angle_min = scan.angle_min;
            bag_scan.angle_max = scan.angle_max;
            bag_scan.angle_increment = scan.angle_increment;
            bag_scan.time_increment = scan.time_increment;
            bag_scan.scan_time = scan.scan_time;
            bag_scan.range_min = scan.range_min;
            bag_scan.range_max = scan.range_max;
            bag_scan.ranges = scan.ranges;
            bag_scan.intensities = scan.intensities;
            bag_writer_->writeLaserScan("/scan_estimated", bag_scan);

            bag_seq_++;
        }
    }

    void processOdom(const json& msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        // Parse Odometry message
        rosbridge::Odometry odom;
        from_json(msg, odom);

        // Debug: print odom position for first few messages
        static int odom_debug_count = 0;
        if (odom_debug_count < 5 || odom_debug_count % 400 == 0) {
            std::cout << "[OdomRx] #" << odom_debug_count
                      << " pos=(" << odom.pose.pose.position.x
                      << "," << odom.pose.pose.position.y << ")"
                      << " yaw=" << odom.pose.pose.orientation.toYaw() << std::endl;
        }
        odom_debug_count++;

        // Add to buffer
        odom_buffer_.push_back(odom);

        // Remove old entries if buffer is full
        while (odom_buffer_.size() > ODOM_BUFFER_SIZE) {
            odom_buffer_.pop_front();
        }
    }

    GmappingWrapper::Config config_;
    GmappingWrapper gmapping_;
    std::mutex mutex_;
    std::deque<rosbridge::Odometry> odom_buffer_;
    int sync_report_count_ = 0;

    // For web visualization
    std::vector<float> last_scan_ranges_;
    float last_scan_angle_min_ = 0;
    float last_scan_angle_max_ = 0;

    // For bag recording
    std::unique_ptr<RosbagWriter> bag_writer_;
    uint32_t bag_seq_ = 0;
    bool tf_static_written_ = false;
};

void printUsage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "\nOptions:\n"
              << "  -p, --port PORT          UDP port to listen on (default: 9090)\n"
              << "  -o, --output FILE        Output map file path (default: map.pgm)\n"
              << "  -r, --resolution RES     Map resolution in meters (default: 0.05)\n"
              << "  --particles N            Number of particles (default: 30)\n"
              << "  --max-range RANGE        Maximum laser range (default: 25.0)\n"
              << "  --linear-update DIST     Linear update threshold (default: 0.5)\n"
              << "  --angular-update ANG     Angular update threshold (default: 0.5)\n"
              << "  --map-size SIZE          Map size in meters (default: 100, creates -50 to 50)\n"
              << "  --save-interval SEC      Map save interval in seconds (default: 5.0)\n"
              << "  --laser-x X              Laser X offset from base_link (default: 0.0)\n"
              << "  --laser-y Y              Laser Y offset from base_link (default: 0.0)\n"
              << "  --laser-theta THETA      Laser rotation in radians (default: 0.0)\n"
              << "  --web-port PORT          Web server port for visualization (default: 8080)\n"
              << "  --lskip N                Skip N beams in scan matching (map uses all beams)\n"
              << "  --scan-log FILE          Save scan poses and data to JSON for verification\n"
              << "  --record-bag FILE        Record mapping process to ROS bag file\n"
              << "  -c, --config FILE        Load configuration from JSON file\n"
              << "  --save-config FILE       Save current configuration to JSON file and exit\n"
              << "  --timing-debug           Enable detailed timing output for debugging\n"
              << "  --single-map             Use single map mode (faster, best particle only)\n"
              << "\nWebSocket mode (connect to rosbridge directly):\n"
              << "  --ws URL                 Connect to rosbridge WebSocket (e.g., ws://localhost:9091)\n"
              << "  --scan-topic TOPIC       Scan topic name (default: /scan)\n"
              << "  --odom-topic TOPIC       Odom topic name (default: /odom)\n"
              << "  -h, --help               Show this help message\n"
              << "\nExamples:\n"
              << "  " << prog << " -p 9090 -o mymap.pgm --laser-x -0.37 --laser-theta 3.14159\n"
              << "  " << prog << " --ws ws://192.168.1.1:9091 --scan-topic /wr_scan_rear\n"
              << std::endl;
}

int main(int argc, char* argv[]) {
    // Default configuration
    int udp_port = 9090;
    int web_port = 8080;
    std::string server_mode = "udp";  // "udp" or "websocket"
    std::string rosbridge_url = "ws://localhost:9091";
    std::string scan_topic = "/scan";
    std::string odom_topic = "/odom";
    GmappingWrapper::Config config;

    // First pass: check for config file option
    for (int i = 1; i < argc; i++) {
        if ((strcmp(argv[i], "-c") == 0 || strcmp(argv[i], "--config") == 0) && i + 1 < argc) {
            try {
                config = GmappingWrapper::Config::loadFromFile(argv[i + 1]);

                // Also load server settings from the same file
                std::ifstream file(argv[i + 1]);
                if (file.is_open()) {
                    json j;
                    file >> j;
                    if (j.contains("server")) {
                        auto& s = j["server"];
                        if (s.contains("mode")) server_mode = s["mode"].get<std::string>();
                        if (s.contains("udp_port")) udp_port = s["udp_port"].get<int>();
                        if (s.contains("port")) udp_port = s["port"].get<int>();  // backward compat
                        if (s.contains("web_port")) web_port = s["web_port"].get<int>();
                    }
                    if (j.contains("rosbridge")) {
                        auto& rb = j["rosbridge"];
                        if (rb.contains("url")) rosbridge_url = rb["url"].get<std::string>();
                        if (rb.contains("scan_topic")) scan_topic = rb["scan_topic"].get<std::string>();
                        if (rb.contains("odom_topic")) odom_topic = rb["odom_topic"].get<std::string>();
                    }
                }
            } catch (const std::exception& e) {
                std::cerr << "Error loading config file: " << e.what() << std::endl;
                return 1;
            }
            break;
        }
    }

    // Command line options
    static struct option long_options[] = {
        {"port",           required_argument, 0, 'p'},
        {"output",         required_argument, 0, 'o'},
        {"resolution",     required_argument, 0, 'r'},
        {"particles",      required_argument, 0, 0},
        {"max-range",      required_argument, 0, 0},
        {"linear-update",  required_argument, 0, 0},
        {"angular-update", required_argument, 0, 0},
        {"map-size",       required_argument, 0, 0},
        {"save-interval",  required_argument, 0, 0},
        {"laser-x",        required_argument, 0, 0},
        {"laser-y",        required_argument, 0, 0},
        {"laser-theta",    required_argument, 0, 0},
        {"web-port",       required_argument, 0, 0},
        {"srr",            required_argument, 0, 0},
        {"srt",            required_argument, 0, 0},
        {"str",            required_argument, 0, 0},
        {"stt",            required_argument, 0, 0},
        {"resample-threshold", required_argument, 0, 0},
        {"lskip",          required_argument, 0, 0},
        {"scan-log",       required_argument, 0, 0},
        {"record-bag",     required_argument, 0, 0},
        {"config",         required_argument, 0, 'c'},
        {"save-config",    required_argument, 0, 0},
        {"timing-debug",   no_argument,       0, 0},
        {"single-map",     no_argument,       0, 0},
        {"ws",             required_argument, 0, 0},  // WebSocket URL
        {"scan-topic",     required_argument, 0, 0},
        {"odom-topic",     required_argument, 0, 0},
        {"help",           no_argument,       0, 'h'},
        {0, 0, 0, 0}
    };

    std::string scan_log_path;
    std::string record_bag_path;
    std::string config_path;
    std::string save_config_path;

    int opt;
    int option_index = 0;
    while ((opt = getopt_long(argc, argv, "p:o:r:c:h", long_options, &option_index)) != -1) {
        switch (opt) {
            case 0: {
                std::string name = long_options[option_index].name;
                if (name == "particles") {
                    config.particles = std::stoi(optarg);
                } else if (name == "max-range") {
                    config.maxRange = std::stod(optarg);
                    config.maxUrange = config.maxRange;
                } else if (name == "linear-update") {
                    config.linearUpdate = std::stod(optarg);
                } else if (name == "angular-update") {
                    config.angularUpdate = std::stod(optarg);
                } else if (name == "map-size") {
                    double size = std::stod(optarg);
                    config.xmin = -size / 2;
                    config.ymin = -size / 2;
                    config.xmax = size / 2;
                    config.ymax = size / 2;
                } else if (name == "save-interval") {
                    config.mapSaveInterval = std::stod(optarg);
                } else if (name == "laser-x") {
                    config.laser_x = std::stod(optarg);
                } else if (name == "laser-y") {
                    config.laser_y = std::stod(optarg);
                } else if (name == "laser-theta") {
                    config.laser_theta = std::stod(optarg);
                } else if (name == "web-port") {
                    web_port = std::stoi(optarg);
                } else if (name == "srr") {
                    config.srr = std::stod(optarg);
                } else if (name == "srt") {
                    config.srt = std::stod(optarg);
                } else if (name == "str") {
                    config.str = std::stod(optarg);
                } else if (name == "stt") {
                    config.stt = std::stod(optarg);
                } else if (name == "resample-threshold") {
                    config.resampleThreshold = std::stod(optarg);
                } else if (name == "lskip") {
                    config.lskip = std::stod(optarg);
                } else if (name == "scan-log") {
                    scan_log_path = optarg;
                } else if (name == "record-bag") {
                    record_bag_path = optarg;
                } else if (name == "save-config") {
                    save_config_path = optarg;
                } else if (name == "timing-debug") {
                    g_timing_debug = true;
                } else if (name == "single-map") {
                    config.singleMapMode = true;
                } else if (name == "ws") {
                    server_mode = "websocket";
                    rosbridge_url = optarg;
                } else if (name == "scan-topic") {
                    scan_topic = optarg;
                } else if (name == "odom-topic") {
                    odom_topic = optarg;
                }
                break;
            }
            case 'c':
                config_path = optarg;
                break;
            case 'p':
                udp_port = std::stoi(optarg);
                break;
            case 'o':
                config.mapOutputPath = optarg;
                break;
            case 'r':
                config.resolution = std::stod(optarg);
                break;
            case 'h':
                printUsage(argv[0]);
                return 0;
            default:
                printUsage(argv[0]);
                return 1;
        }
    }

    // Handle save-config option
    if (!save_config_path.empty()) {
        try {
            config.saveToFile(save_config_path);
            std::cout << "Configuration saved to " << save_config_path << std::endl;
            return 0;
        } catch (const std::exception& e) {
            std::cerr << "Error saving config file: " << e.what() << std::endl;
            return 1;
        }
    }

    // Setup signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "=== GMapping JSON Server ===" << std::endl;
    std::cout << "Mode: " << server_mode << std::endl;
    if (server_mode == "websocket") {
        std::cout << "Rosbridge URL: " << rosbridge_url << std::endl;
        std::cout << "Scan topic: " << scan_topic << std::endl;
        std::cout << "Odom topic: " << odom_topic << std::endl;
    } else {
        std::cout << "UDP Port: " << udp_port << std::endl;
    }
    std::cout << "Output: " << config.mapOutputPath << std::endl;
    std::cout << "Resolution: " << config.resolution << " m/cell" << std::endl;
    std::cout << "Map size: [" << config.xmin << ", " << config.ymin
              << "] to [" << config.xmax << ", " << config.ymax << "]" << std::endl;
    std::cout << "Particles: " << config.particles << std::endl;
    std::cout << "Laser offset: (" << config.laser_x << ", " << config.laser_y
              << ", " << config.laser_theta << " rad)" << std::endl;
    if (config.lskip > 0) {
        std::cout << "Likelihood skip: " << config.lskip << " (scan matching uses 1/" << (config.lskip + 1) << " beams)" << std::endl;
    }
    std::cout << "Web viewer: http://localhost:" << web_port << std::endl;
    std::cout << std::endl;

    // Create processor
    SlamProcessor processor(config);

    // Enable scan logging if specified
    if (!scan_log_path.empty()) {
        processor.enableScanLogging(scan_log_path);
    }

    // Enable bag recording if specified
    if (!record_bag_path.empty()) {
        processor.enableBagRecording(record_bag_path);
        std::cout << "Recording to bag: " << record_bag_path << std::endl;
    }

    // Create and start web server for visualization
    WebServer webServer(web_port);
    if (!webServer.start()) {
        std::cerr << "[Main] Failed to start web server on port " << web_port << std::endl;
    }

    // Create message queue for decoupling receive from processing
    MessageQueue messageQueue;

    // Server instances (only one will be used)
    std::unique_ptr<UdpServer> udpServer;
    std::unique_ptr<WebSocketClient> wsClient;
    std::thread receiveThread;

    if (server_mode == "websocket") {
        // WebSocket mode - connect to rosbridge
        wsClient = std::make_unique<WebSocketClient>();

        if (!wsClient->connect(rosbridge_url)) {
            std::cerr << "[Main] Failed to connect to rosbridge: " << rosbridge_url << std::endl;
            return 1;
        }

        // Subscribe to topics
        std::cout << "[Main] Subscribing to scan: " << scan_topic << std::endl;
        bool scan_ok = wsClient->subscribe(scan_topic, "sensor_msgs/LaserScan");
        std::cout << "[Main] Scan subscribe result: " << (scan_ok ? "OK" : "FAILED") << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds(200));  // Delay between subscriptions

        std::cout << "[Main] Subscribing to odom: " << odom_topic << std::endl;
        bool odom_ok = wsClient->subscribe(odom_topic, "nav_msgs/Odometry");
        std::cout << "[Main] Odom subscribe result: " << (odom_ok ? "OK" : "FAILED") << std::endl;

        std::cout << "Connected to rosbridge, waiting for messages..." << std::endl;
        std::cout << "Press Ctrl+C to save map and exit" << std::endl;
        std::cout << std::endl;

        // Start receive thread for WebSocket
        receiveThread = std::thread([&wsClient, &messageQueue]() {
            wsClient->startAsync([&messageQueue](const std::string& data) {
                messageQueue.push(data);
            });
            // Wait until shutdown
            while (!g_shutdown && wsClient->isConnected()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

    } else {
        // UDP mode (default)
        udpServer = std::make_unique<UdpServer>(udp_port);

        if (!udpServer->start()) {
            return 1;
        }

        std::cout << "Waiting for rosbridge messages via UDP..." << std::endl;
        std::cout << "Expected topics: /scan (sensor_msgs/LaserScan), /odom (nav_msgs/Odometry)" << std::endl;
        std::cout << "Press Ctrl+C to save map and exit" << std::endl;
        std::cout << std::endl;

        // Start receive thread for UDP
        receiveThread = std::thread([&udpServer, &messageQueue]() {
            std::string data;
            std::string sender;
            while (!g_shutdown && udpServer->isRunning()) {
                if (udpServer->receive(data, sender)) {
                    messageQueue.push(data);
                }
            }
        });
    }

    // Main processing loop - slower, but doesn't block receiving
    std::string data;
    int stats_counter = 0;
    int web_update_counter = 0;

    // For tracking pose changes
    double prev_x = 0, prev_y = 0, prev_theta = 0;
    int prev_best_idx = -1;
    bool first_log = true;

    while (!g_shutdown) {
        if (messageQueue.pop(data, 100)) {
            processor.processMessage(data, "");
        }
        // Update web server state periodically (every 10 iterations ~1 second)
        if (++web_update_counter % 10 == 0) {
            auto state = processor.getWebState();
            webServer.updateState(state);

            // Log robot position with best particle index
            int best_idx = processor.getBestParticleIndex();
            double dx = state.x - prev_x;
            double dy = state.y - prev_y;
            double dtheta = state.theta - prev_theta;
            double dist = std::sqrt(dx*dx + dy*dy);

            // Log if significant movement or particle change
            if (first_log || dist > 0.01 || std::abs(dtheta) > 0.01 || best_idx != prev_best_idx) {
                std::cout << "[WebLog] best_idx=" << best_idx
                          << " pose=(" << state.x << ", " << state.y << ", " << state.theta << ")";
                if (!first_log) {
                    std::cout << " delta=(" << dx << ", " << dy << ", " << dtheta << ") dist=" << dist;
                    if (best_idx != prev_best_idx) {
                        std::cout << " ***PARTICLE_CHANGED*** from " << prev_best_idx;
                    }
                    if (dist > 0.5) {
                        std::cout << " ***BIG_JUMP***";
                    }
                }
                std::cout << std::endl;

                // Log all particle positions
                std::cout << "[Particles] ";
                for (size_t i = 0; i < state.particles.size(); i++) {
                    const auto& p = state.particles[i];
                    if (i == static_cast<size_t>(best_idx)) std::cout << "*";
                    std::cout << "P" << i << "(" << p.x << "," << p.y << "," << p.theta << ") ";
                }
                std::cout << std::endl;

                // Calculate particle spread statistics
                if (!state.particles.empty()) {
                    double sum_x = 0, sum_y = 0, sum_theta = 0;
                    double min_x = state.particles[0].x, max_x = state.particles[0].x;
                    double min_y = state.particles[0].y, max_y = state.particles[0].y;
                    for (const auto& p : state.particles) {
                        sum_x += p.x;
                        sum_y += p.y;
                        sum_theta += p.theta;
                        min_x = std::min(min_x, p.x);
                        max_x = std::max(max_x, p.x);
                        min_y = std::min(min_y, p.y);
                        max_y = std::max(max_y, p.y);
                    }
                    double mean_x = sum_x / state.particles.size();
                    double mean_y = sum_y / state.particles.size();
                    double var_x = 0, var_y = 0;
                    for (const auto& p : state.particles) {
                        var_x += (p.x - mean_x) * (p.x - mean_x);
                        var_y += (p.y - mean_y) * (p.y - mean_y);
                    }
                    double std_x = std::sqrt(var_x / state.particles.size());
                    double std_y = std::sqrt(var_y / state.particles.size());
                    std::cout << "[ParticleSpread] mean=(" << mean_x << "," << mean_y
                              << ") std=(" << std_x << "," << std_y
                              << ") range_x=" << (max_x - min_x)
                              << " range_y=" << (max_y - min_y) << std::endl;
                }

                first_log = false;
            }

            prev_x = state.x;
            prev_y = state.y;
            prev_theta = state.theta;
            prev_best_idx = best_idx;
        }
        // Print queue stats periodically
        if (++stats_counter % 500 == 0 && messageQueue.size() > 0) {
            messageQueue.printStats();
        }
    }

    // Shutdown
    messageQueue.shutdown();

    // Process remaining messages in queue
    std::cout << "\n[Main] Processing remaining " << messageQueue.size() << " messages..." << std::endl;
    while (messageQueue.pop(data, 10)) {
        processor.processMessage(data, "");
    }

    // Save final map
    std::cout << "\n[Main] Saving final map..." << std::endl;
    processor.saveMap();

    // Save scan log if enabled
    if (!scan_log_path.empty()) {
        processor.saveScanLog();
    }

    // Close bag recording
    processor.closeBagRecording();

    messageQueue.printStats();

    // Stop the appropriate server/client
    if (wsClient) {
        wsClient->stop();
    }
    if (udpServer) {
        udpServer->stop();
    }
    webServer.stop();
    receiveThread.join();
    std::cout << "[Main] Shutdown complete" << std::endl;

    return 0;
}
