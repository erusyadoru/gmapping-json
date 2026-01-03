#ifndef GMAPPING_WRAPPER_H
#define GMAPPING_WRAPPER_H

#include <memory>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <cmath>

#include <gmapping/gridfastslam/gridslamprocessor.h>
#include <gmapping/sensor/sensor_range/rangesensor.h>
#include <gmapping/sensor/sensor_range/rangereading.h>
#include <gmapping/sensor/sensor_odometry/odometrysensor.h>
#include <gmapping/sensor/sensor_odometry/odometryreading.h>

#include "rosbridge_types.h"

class GmappingWrapper {
public:
    struct Config {
        // Map parameters
        double xmin = -10.0;
        double ymin = -10.0;
        double xmax = 10.0;
        double ymax = 10.0;
        double resolution = 0.05;  // 5cm per cell

        // Particle filter parameters
        int particles = 10;
        double resampleThreshold = 0.5;
        bool singleMapMode = false;  // Use single shared map (faster but less robust)

        // Motion model parameters (odometry error)
        double srr = 0.1;  // translation error from translation
        double srt = 0.2;  // translation error from rotation
        double str = 0.1;  // rotation error from translation
        double stt = 0.2;  // rotation error from rotation

        // Scan matcher parameters
        double maxRange = 10.0;
        double maxUrange = 10.0;
        double sigma = 0.05;
        int kernelSize = 1;
        double lstep = 0.05;
        double astep = 0.05;
        int iterations = 5;
        double lsigma = 0.075;
        double lskip = 0;  // Use all beams

        // Update thresholds
        double linearUpdate = 0.2;   // meters
        double angularUpdate = 0.2;  // radians

        // Laser pose relative to robot base
        double laser_x = 0.0;
        double laser_y = 0.0;
        double laser_theta = 0.0;

        // Output
        std::string mapOutputPath = "map.pgm";
        double mapSaveInterval = 5.0;  // save every 5 seconds

        Config() = default;

        // Load configuration from JSON file
        static Config loadFromFile(const std::string& filepath);

        // Save current configuration to JSON file
        void saveToFile(const std::string& filepath) const;
    };

    GmappingWrapper();
    explicit GmappingWrapper(const Config& config);
    ~GmappingWrapper();

    // Initialize with laser scan parameters
    bool initializeLaser(float angle_min, float angle_max,
                         float angle_increment, float range_max,
                         unsigned int num_beams);

    // Process incoming data
    bool processScan(const rosbridge::LaserScan& scan, const rosbridge::Odometry& odom);

    // Get current robot pose
    GMapping::OrientedPoint getCurrentPose() const;

    // Get best particle index
    int getBestParticleIndex() const;

    // Get all particle poses
    struct ParticlePose {
        double x, y, theta;
        double weight;
    };
    std::vector<ParticlePose> getParticlePoses() const;

    // Save map to PGM file
    bool saveMap(const std::string& filename = "");

    // Get map data
    struct MapData {
        std::vector<int8_t> data;  // -1: unknown, 0: free, 100: occupied
        int width;
        int height;
        double resolution;
        double origin_x;
        double origin_y;
    };
    MapData getMapData() const;

    // Scan step data for verification
    struct ScanStep {
        int step;
        double pose_x, pose_y, pose_theta;  // Best particle pose
        double odom_x, odom_y, odom_theta;  // Odometry pose
        float angle_min, angle_max;
        std::vector<float> ranges;
    };
    void enableScanLogging(const std::string& output_path);
    void saveScanLog() const;

    // Check if initialized
    bool isInitialized() const { return initialized_; }

private:
    Config config_;
    std::unique_ptr<GMapping::GridSlamProcessor> processor_;
    GMapping::RangeSensor* laser_sensor_;
    GMapping::OdometrySensor* odom_sensor_;

    bool initialized_ = false;
    bool laser_initialized_ = false;
    int scan_count_ = 0;
    double last_map_save_time_ = 0.0;

    GMapping::OrientedPoint last_odom_pose_;
    bool first_scan_ = true;

    // Scan logging
    bool scan_logging_enabled_ = false;
    std::string scan_log_path_;
    std::vector<ScanStep> scan_steps_;

    // Convert rosbridge types to gmapping types
    GMapping::RangeReading* createRangeReading(const rosbridge::LaserScan& scan);
    GMapping::OrientedPoint toOrientedPoint(const rosbridge::Pose& pose);
};

#endif // GMAPPING_WRAPPER_H
