#include "gmapping_wrapper.h"
#include <gmapping/scanmatcher/smmap.h>
#include "json.hpp"
#include <chrono>

using json = nlohmann::json;

// Timing debug flag (weak reference, defined in main.cpp)
bool g_timing_debug __attribute__((weak)) = false;

// Simple timer for this file
class Timer {
public:
    Timer() : start_(std::chrono::high_resolution_clock::now()) {}
    double elapsed_ms() const {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(now - start_).count() / 1000.0;
    }
private:
    std::chrono::high_resolution_clock::time_point start_;
};

// Config loading/saving implementation
GmappingWrapper::Config GmappingWrapper::Config::loadFromFile(const std::string& filepath) {
    Config config;

    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open config file: " + filepath);
    }

    json j;
    try {
        file >> j;
    } catch (const json::parse_error& e) {
        throw std::runtime_error("JSON parse error in " + filepath + ": " + e.what());
    }

    // Map parameters
    if (j.contains("map")) {
        auto& m = j["map"];
        if (m.contains("xmin")) config.xmin = m["xmin"].get<double>();
        if (m.contains("ymin")) config.ymin = m["ymin"].get<double>();
        if (m.contains("xmax")) config.xmax = m["xmax"].get<double>();
        if (m.contains("ymax")) config.ymax = m["ymax"].get<double>();
        if (m.contains("resolution")) config.resolution = m["resolution"].get<double>();
        if (m.contains("size")) {
            double size = m["size"].get<double>();
            config.xmin = -size / 2;
            config.ymin = -size / 2;
            config.xmax = size / 2;
            config.ymax = size / 2;
        }
    }

    // Particle filter parameters
    if (j.contains("particle_filter")) {
        auto& pf = j["particle_filter"];
        if (pf.contains("particles")) config.particles = pf["particles"].get<int>();
        if (pf.contains("resample_threshold")) config.resampleThreshold = pf["resample_threshold"].get<double>();
        if (pf.contains("single_map_mode")) config.singleMapMode = pf["single_map_mode"].get<bool>();
    }

    // Motion model parameters
    if (j.contains("motion_model")) {
        auto& mm = j["motion_model"];
        if (mm.contains("srr")) config.srr = mm["srr"].get<double>();
        if (mm.contains("srt")) config.srt = mm["srt"].get<double>();
        if (mm.contains("str")) config.str = mm["str"].get<double>();
        if (mm.contains("stt")) config.stt = mm["stt"].get<double>();
    }

    // Scan matcher parameters
    if (j.contains("scan_matcher")) {
        auto& sm = j["scan_matcher"];
        if (sm.contains("max_range")) config.maxRange = sm["max_range"].get<double>();
        if (sm.contains("max_urange")) config.maxUrange = sm["max_urange"].get<double>();
        if (sm.contains("sigma")) config.sigma = sm["sigma"].get<double>();
        if (sm.contains("kernel_size")) config.kernelSize = sm["kernel_size"].get<int>();
        if (sm.contains("lstep")) config.lstep = sm["lstep"].get<double>();
        if (sm.contains("astep")) config.astep = sm["astep"].get<double>();
        if (sm.contains("iterations")) config.iterations = sm["iterations"].get<int>();
        if (sm.contains("lsigma")) config.lsigma = sm["lsigma"].get<double>();
        if (sm.contains("lskip")) config.lskip = sm["lskip"].get<double>();
    }

    // Update thresholds
    if (j.contains("update")) {
        auto& u = j["update"];
        if (u.contains("linear")) config.linearUpdate = u["linear"].get<double>();
        if (u.contains("angular")) config.angularUpdate = u["angular"].get<double>();
    }

    // Laser pose
    if (j.contains("laser")) {
        auto& l = j["laser"];
        if (l.contains("x")) config.laser_x = l["x"].get<double>();
        if (l.contains("y")) config.laser_y = l["y"].get<double>();
        if (l.contains("theta")) config.laser_theta = l["theta"].get<double>();
    }

    // Output
    if (j.contains("output")) {
        auto& o = j["output"];
        if (o.contains("map_path")) config.mapOutputPath = o["map_path"].get<std::string>();
        if (o.contains("save_interval")) config.mapSaveInterval = o["save_interval"].get<double>();
    }

    std::cout << "[Config] Loaded configuration from " << filepath << std::endl;
    std::cout << "[Config]   Map: " << config.xmin << " to " << config.xmax << ", resolution=" << config.resolution << std::endl;
    std::cout << "[Config]   Particles: " << config.particles << ", resample_threshold=" << config.resampleThreshold << std::endl;
    std::cout << "[Config]   Motion model: srr=" << config.srr << ", srt=" << config.srt << ", str=" << config.str << ", stt=" << config.stt << std::endl;
    std::cout << "[Config]   Laser: x=" << config.laser_x << ", y=" << config.laser_y << ", theta=" << config.laser_theta << std::endl;

    return config;
}

void GmappingWrapper::Config::saveToFile(const std::string& filepath) const {
    json j;

    // Map parameters
    j["map"] = {
        {"xmin", xmin},
        {"ymin", ymin},
        {"xmax", xmax},
        {"ymax", ymax},
        {"resolution", resolution}
    };

    // Particle filter parameters
    j["particle_filter"] = {
        {"particles", particles},
        {"resample_threshold", resampleThreshold},
        {"single_map_mode", singleMapMode}
    };

    // Motion model parameters
    j["motion_model"] = {
        {"srr", srr},
        {"srt", srt},
        {"str", str},
        {"stt", stt}
    };

    // Scan matcher parameters
    j["scan_matcher"] = {
        {"max_range", maxRange},
        {"max_urange", maxUrange},
        {"sigma", sigma},
        {"kernel_size", kernelSize},
        {"lstep", lstep},
        {"astep", astep},
        {"iterations", iterations},
        {"lsigma", lsigma},
        {"lskip", lskip}
    };

    // Update thresholds
    j["update"] = {
        {"linear", linearUpdate},
        {"angular", angularUpdate}
    };

    // Laser pose
    j["laser"] = {
        {"x", laser_x},
        {"y", laser_y},
        {"theta", laser_theta}
    };

    // Output
    j["output"] = {
        {"map_path", mapOutputPath},
        {"save_interval", mapSaveInterval}
    };

    std::ofstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open config file for writing: " + filepath);
    }

    file << j.dump(2);
    std::cout << "[Config] Saved to " << filepath << std::endl;
}

GmappingWrapper::GmappingWrapper()
    : GmappingWrapper(Config())
{}

GmappingWrapper::GmappingWrapper(const Config& config)
    : config_(config)
    , processor_(new GMapping::GridSlamProcessor())
    , laser_sensor_(nullptr)
    , odom_sensor_(nullptr)
{
    // Create odometry sensor
    odom_sensor_ = new GMapping::OdometrySensor("odom");
}

GmappingWrapper::~GmappingWrapper() {
    if (laser_sensor_) delete laser_sensor_;
    if (odom_sensor_) delete odom_sensor_;
}

bool GmappingWrapper::initializeLaser(float angle_min, float angle_max,
                                       float angle_increment, float range_max,
                                       unsigned int num_beams) {
    if (laser_initialized_) {
        return true;
    }

    // Use actual beam count from scan data, recalculate angle_increment
    // This ensures RangeSensor beam count matches actual scan data
    float actual_angle_increment = (angle_max - angle_min) / (num_beams - 1);

    std::cout << "[GMapping] Initializing laser with " << num_beams << " beams" << std::endl;
    std::cout << "[GMapping] Angle range: [" << angle_min << ", " << angle_max << "]" << std::endl;
    std::cout << "[GMapping] Original angle_increment: " << angle_increment
              << ", Recalculated: " << actual_angle_increment << std::endl;

    angle_increment = actual_angle_increment;

    // Create range sensor - use the full constructor to set m_pose properly
    GMapping::OrientedPoint laser_pose(config_.laser_x, config_.laser_y, config_.laser_theta);

    laser_sensor_ = new GMapping::RangeSensor(
        "FLASER",
        num_beams,
        angle_increment,
        laser_pose,
        0.0,
        range_max
    );

    // Override beam angles to use actual angle_min to angle_max range
    // The default constructor uses symmetric angles around 0, but we need angle_min to angle_max
    for (unsigned int i = 0; i < num_beams; i++) {
        double angle = angle_min + i * angle_increment;
        GMapping::RangeSensor::Beam& beam = laser_sensor_->beams()[i];
        beam.pose.theta = angle;
        beam.s = sin(angle);
        beam.c = cos(angle);
    }

    std::cout << "[GMapping] First beam angle: " << laser_sensor_->beams()[0].pose.theta << std::endl;
    std::cout << "[GMapping] Last beam angle: " << laser_sensor_->beams()[num_beams-1].pose.theta << std::endl;
    std::cout << "[GMapping] Laser pose: (" << config_.laser_x << ", " << config_.laser_y << ", " << config_.laser_theta << ")" << std::endl;

    // Set sensor map
    GMapping::SensorMap smap;
    smap.insert(std::make_pair(laser_sensor_->getName(), laser_sensor_));
    smap.insert(std::make_pair(odom_sensor_->getName(), odom_sensor_));
    processor_->setSensorMap(smap);

    // Initialize processor
    processor_->init(
        config_.particles,
        config_.xmin, config_.ymin,
        config_.xmax, config_.ymax,
        config_.resolution,
        GMapping::OrientedPoint(0, 0, 0)
    );

    // Set matching parameters
    processor_->setMatchingParameters(
        config_.maxUrange,
        config_.maxRange,
        config_.sigma,
        config_.kernelSize,
        config_.lstep,
        config_.astep,
        config_.iterations,
        config_.lsigma,
        1.0,  // likelihoodGain
        static_cast<unsigned int>(config_.lskip)
    );

    std::cout << "[GMapping] ScanMatcher params: kernelSize=" << config_.kernelSize
              << ", iterations=" << config_.iterations
              << ", lskip=" << config_.lskip << std::endl;

    // Set motion model parameters
    processor_->setMotionModelParameters(
        config_.srr,
        config_.srt,
        config_.str,
        config_.stt
    );

    // Set update distances
    processor_->setUpdateDistances(
        config_.linearUpdate,
        config_.angularUpdate,
        config_.resampleThreshold
    );

    // Disable temporal update (like ROS gmapping default)
    processor_->setUpdatePeriod(-1.0);

    // Additional settings
    processor_->setgenerateMap(true);
    processor_->setlaserMaxRange(range_max);
    processor_->setusableRange(config_.maxUrange);
    processor_->setsingleMapMode(config_.singleMapMode);

    if (config_.singleMapMode) {
        std::cout << "[GMapping] Single-map mode enabled (faster, uses only best particle's map)" << std::endl;
    }

    laser_initialized_ = true;
    initialized_ = true;

    std::cout << "[GMapping] Initialization complete" << std::endl;
    return true;
}

GMapping::OrientedPoint GmappingWrapper::toOrientedPoint(const rosbridge::Pose& pose) {
    double yaw = pose.orientation.toYaw();
    return GMapping::OrientedPoint(pose.position.x, pose.position.y, yaw);
}

GMapping::RangeReading* GmappingWrapper::createRangeReading(const rosbridge::LaserScan& scan) {
    double timestamp = scan.header.stamp.toSec();

    // Create range reading with simple constructor
    GMapping::RangeReading* reading = new GMapping::RangeReading(laser_sensor_, timestamp);

    // Resize and copy range data
    reading->resize(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        float r = scan.ranges[i];
        // Handle invalid ranges
        if (std::isnan(r) || std::isinf(r) || r < scan.range_min || r > scan.range_max) {
            (*reading)[i] = scan.range_max;  // Use max range for invalid readings
        } else {
            (*reading)[i] = static_cast<double>(r);
        }
    }

    return reading;
}

bool GmappingWrapper::processScan(const rosbridge::LaserScan& scan, const rosbridge::Odometry& odom) {
    Timer total_timer;

    // Initialize laser on first scan if not done
    if (!laser_initialized_) {
        if (!initializeLaser(scan.angle_min, scan.angle_max,
                             scan.angle_increment, scan.range_max,
                             static_cast<unsigned int>(scan.ranges.size()))) {
            std::cerr << "[GMapping] Failed to initialize laser" << std::endl;
            return false;
        }
    }

    // Get odometry pose
    GMapping::OrientedPoint odom_pose = toOrientedPoint(odom.pose.pose);

    // Debug: Print odom pose being used
    if (scan_count_ < 10 || scan_count_ % 20 == 0) {
        std::cout << "[Debug] Scan " << (scan_count_ + 1) << " odom_pose: ("
                  << odom_pose.x << ", " << odom_pose.y << ", " << odom_pose.theta << ")" << std::endl;
    }

    // Create range reading
    Timer reading_timer;
    GMapping::RangeReading* reading = createRangeReading(scan);
    reading->setPose(odom_pose);
    double reading_ms = reading_timer.elapsed_ms();

    // Process scan (this is the core SLAM processing)
    Timer slam_timer;
    bool processed = processor_->processScan(*reading);
    double slam_ms = slam_timer.elapsed_ms();

    if (processed) {
        scan_count_++;

        // Get best particle pose
        GMapping::OrientedPoint pose = getCurrentPose();
        double total_ms = total_timer.elapsed_ms();

        // Print timing info
        if (g_timing_debug) {
            std::cout << "[Timer] GMapping breakdown - reading: " << reading_ms
                      << "ms, SLAM: " << slam_ms << "ms, total: " << total_ms << "ms" << std::endl;
        }

        std::cout << "[GMapping] Scan " << scan_count_
                  << " processed. Pose: (" << pose.x << ", " << pose.y
                  << ", " << pose.theta << ") [" << slam_ms << "ms]" << std::endl;

        // Log scan step data if enabled
        if (scan_logging_enabled_) {
            ScanStep step;
            step.step = scan_count_;
            step.pose_x = pose.x;
            step.pose_y = pose.y;
            step.pose_theta = pose.theta;
            step.odom_x = odom_pose.x;
            step.odom_y = odom_pose.y;
            step.odom_theta = odom_pose.theta;
            step.angle_min = scan.angle_min;
            step.angle_max = scan.angle_max;
            step.ranges = scan.ranges;
            scan_steps_.push_back(step);
        }

        // Save map periodically
        double current_time = scan.header.stamp.toSec();
        if (config_.mapSaveInterval > 0 &&
            (current_time - last_map_save_time_) >= config_.mapSaveInterval) {
            saveMap();
            last_map_save_time_ = current_time;
        }
    }

    delete reading;
    return processed;
}

GMapping::OrientedPoint GmappingWrapper::getCurrentPose() const {
    if (!initialized_) {
        return GMapping::OrientedPoint(0, 0, 0);
    }

    const GMapping::GridSlamProcessor::ParticleVector& particles = processor_->getParticles();
    int best_idx = processor_->getBestParticleIndex();

    if (best_idx >= 0 && best_idx < static_cast<int>(particles.size())) {
        return particles[best_idx].pose;
    }

    return GMapping::OrientedPoint(0, 0, 0);
}

int GmappingWrapper::getBestParticleIndex() const {
    if (!initialized_) {
        return -1;
    }
    return processor_->getBestParticleIndex();
}

std::vector<GmappingWrapper::ParticlePose> GmappingWrapper::getParticlePoses() const {
    std::vector<ParticlePose> poses;
    if (!initialized_) {
        return poses;
    }

    const GMapping::GridSlamProcessor::ParticleVector& particles = processor_->getParticles();
    poses.reserve(particles.size());

    for (const auto& p : particles) {
        ParticlePose pose;
        pose.x = p.pose.x;
        pose.y = p.pose.y;
        pose.theta = p.pose.theta;
        pose.weight = p.weight;
        poses.push_back(pose);
    }

    return poses;
}

GmappingWrapper::MapData GmappingWrapper::getMapData() const {
    MapData map_data;
    map_data.width = 0;
    map_data.height = 0;
    map_data.resolution = config_.resolution;
    map_data.origin_x = 0;
    map_data.origin_y = 0;

    if (!initialized_) {
        return map_data;
    }

    const GMapping::GridSlamProcessor::ParticleVector& particles = processor_->getParticles();
    int best_idx = processor_->getBestParticleIndex();

    if (best_idx < 0 || best_idx >= static_cast<int>(particles.size())) {
        return map_data;
    }

    const GMapping::ScanMatcherMap& smap = particles[best_idx].map;

    // Get map dimensions
    GMapping::Point center = smap.getCenter();
    GMapping::Point world_size(smap.getMapSizeX() * smap.getResolution(),
                               smap.getMapSizeY() * smap.getResolution());

    map_data.width = smap.getMapSizeX();
    map_data.height = smap.getMapSizeY();
    map_data.origin_x = center.x - world_size.x / 2.0;
    map_data.origin_y = center.y - world_size.y / 2.0;
    map_data.resolution = smap.getResolution();

    // Extract map data
    map_data.data.resize(map_data.width * map_data.height, -1);

    // Debug: track occupancy value distribution
    int unknown_count = 0, free_count = 0, occupied_count = 0;
    double max_occ_val = 0, min_occ_positive = 1.0;

    for (int y = 0; y < map_data.height; ++y) {
        for (int x = 0; x < map_data.width; ++x) {
            GMapping::IntPoint p(x, y);
            // Use current occupancy (max_occ disabled for speed test)
            double occ = smap.cell(p);

            int idx = y * map_data.width + x;

            if (occ < 0) {
                map_data.data[idx] = -1;  // Unknown
                unknown_count++;
            } else {
                // Use max occupancy (preserves once-occupied cells)
                map_data.data[idx] = static_cast<int8_t>(occ * 100.0);
                if (occ > 0.25) {  // Match ROS gmapping default threshold
                    occupied_count++;
                } else {
                    free_count++;
                }
                if (occ > max_occ_val) max_occ_val = occ;
                if (occ > 0 && occ < min_occ_positive) min_occ_positive = occ;
            }
        }
    }

    return map_data;
}

bool GmappingWrapper::saveMap(const std::string& filename) {
    std::string output_file = filename.empty() ? config_.mapOutputPath : filename;

    MapData map = getMapData();

    if (map.width == 0 || map.height == 0) {
        std::cerr << "[GMapping] Cannot save empty map" << std::endl;
        return false;
    }

    // Save PGM file
    std::ofstream pgm(output_file, std::ios::binary);
    if (!pgm.is_open()) {
        std::cerr << "[GMapping] Failed to open " << output_file << " for writing" << std::endl;
        return false;
    }

    pgm << "P5\n";
    pgm << map.width << " " << map.height << "\n";
    pgm << "255\n";

    for (int y = map.height - 1; y >= 0; --y) {  // Flip Y for image
        for (int x = 0; x < map.width; ++x) {
            int idx = y * map.width + x;
            int8_t val = map.data[idx];
            unsigned char pixel;

            if (val < 0) {
                pixel = 128;  // Unknown = gray
            } else if (val > 25) {  // ROS gmapping default: occ_thresh = 0.25
                pixel = 0;    // Occupied = black
            } else {
                pixel = 255;  // Free = white
            }

            pgm.write(reinterpret_cast<char*>(&pixel), 1);
        }
    }

    pgm.close();

    // Save YAML metadata file
    std::string yaml_file = output_file.substr(0, output_file.rfind('.')) + ".yaml";
    std::ofstream yaml(yaml_file);
    if (yaml.is_open()) {
        yaml << "image: " << output_file << "\n";
        yaml << "resolution: " << map.resolution << "\n";
        yaml << "origin: [" << map.origin_x << ", " << map.origin_y << ", 0.0]\n";
        yaml << "negate: 0\n";
        yaml << "occupied_thresh: 0.65\n";
        yaml << "free_thresh: 0.196\n";
        yaml.close();
    }

    std::cout << "[GMapping] Map saved to " << output_file
              << " (" << map.width << "x" << map.height << ")" << std::endl;

    return true;
}

void GmappingWrapper::enableScanLogging(const std::string& output_path) {
    scan_logging_enabled_ = true;
    scan_log_path_ = output_path;
    std::cout << "[GMapping] Scan logging enabled, will save to: " << output_path << std::endl;
}

void GmappingWrapper::saveScanLog() const {
    if (!scan_logging_enabled_ || scan_steps_.empty()) {
        std::cout << "[GMapping] No scan log data to save" << std::endl;
        return;
    }

    json j;
    j["steps"] = json::array();

    for (const auto& step : scan_steps_) {
        json step_json;
        step_json["step"] = step.step;
        step_json["pose"] = {
            {"x", step.pose_x},
            {"y", step.pose_y},
            {"theta", step.pose_theta}
        };
        step_json["odom"] = {
            {"x", step.odom_x},
            {"y", step.odom_y},
            {"theta", step.odom_theta}
        };
        step_json["angle_min"] = step.angle_min;
        step_json["angle_max"] = step.angle_max;
        step_json["ranges"] = step.ranges;
        j["steps"].push_back(step_json);
    }

    // Add map metadata
    MapData map = getMapData();
    j["map"] = {
        {"width", map.width},
        {"height", map.height},
        {"resolution", map.resolution},
        {"origin_x", map.origin_x},
        {"origin_y", map.origin_y}
    };

    std::ofstream file(scan_log_path_);
    if (file.is_open()) {
        file << j.dump(2);
        file.close();
        std::cout << "[GMapping] Scan log saved to " << scan_log_path_
                  << " (" << scan_steps_.size() << " steps)" << std::endl;
    } else {
        std::cerr << "[GMapping] Failed to save scan log to " << scan_log_path_ << std::endl;
    }
}
