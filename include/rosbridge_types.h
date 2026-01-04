#ifndef ROSBRIDGE_TYPES_H
#define ROSBRIDGE_TYPES_H

#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include "json.hpp"

using json = nlohmann::json;

namespace rosbridge {

// ROS Header
struct Header {
    struct Stamp {
        int32_t sec = 0;
        uint32_t nanosec = 0;

        double toSec() const {
            return static_cast<double>(sec) + static_cast<double>(nanosec) * 1e-9;
        }
    } stamp;
    std::string frame_id;
};

// geometry_msgs/Point
struct Point {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// geometry_msgs/Quaternion
struct Quaternion {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;

    // Convert to yaw angle (2D rotation)
    double toYaw() const {
        // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
};

// geometry_msgs/Pose
struct Pose {
    Point position;
    Quaternion orientation;
};

// geometry_msgs/PoseWithCovariance
struct PoseWithCovariance {
    Pose pose;
    std::vector<double> covariance; // 36 elements
};

// geometry_msgs/Twist
struct Twist {
    Point linear;
    Point angular;
};

// geometry_msgs/TwistWithCovariance
struct TwistWithCovariance {
    Twist twist;
    std::vector<double> covariance;
};

// sensor_msgs/LaserScan
struct LaserScan {
    Header header;
    float angle_min = 0.0f;
    float angle_max = 0.0f;
    float angle_increment = 0.0f;
    float time_increment = 0.0f;
    float scan_time = 0.0f;
    float range_min = 0.0f;
    float range_max = 0.0f;
    std::vector<float> ranges;
    std::vector<float> intensities;
};

// nav_msgs/Odometry
struct Odometry {
    Header header;
    std::string child_frame_id;
    PoseWithCovariance pose;
    TwistWithCovariance twist;
};

// rosbridge message wrapper
struct RosbridgeMessage {
    std::string op;
    std::string topic;
    json msg;
};

// Helper to check if field exists and is not null
inline bool has_value(const json& j, const std::string& key) {
    return j.contains(key) && !j[key].is_null();
}

// Helper to parse float array with null handling (null -> infinity)
inline std::vector<float> parse_float_array(const json& j) {
    std::vector<float> result;
    if (!j.is_array()) return result;
    result.reserve(j.size());
    for (const auto& val : j) {
        if (val.is_null()) {
            result.push_back(std::numeric_limits<float>::infinity());
        } else if (val.is_number()) {
            result.push_back(val.get<float>());
        } else {
            result.push_back(std::numeric_limits<float>::infinity());
        }
    }
    return result;
}

// JSON parsing functions
inline void from_json(const json& j, Header::Stamp& s) {
    // Support both ROS1 (secs/nsecs) and ROS2 (sec/nanosec) formats
    if (has_value(j, "sec")) s.sec = j["sec"].get<int32_t>();
    if (has_value(j, "secs")) s.sec = j["secs"].get<int32_t>();
    if (has_value(j, "nanosec")) s.nanosec = j["nanosec"].get<uint32_t>();
    if (has_value(j, "nsec")) s.nanosec = j["nsec"].get<uint32_t>();
    if (has_value(j, "nsecs")) s.nanosec = j["nsecs"].get<uint32_t>();
}

inline void from_json(const json& j, Header& h) {
    if (has_value(j, "stamp")) j["stamp"].get_to(h.stamp);
    if (has_value(j, "frame_id")) h.frame_id = j["frame_id"].get<std::string>();
}

inline void from_json(const json& j, Point& p) {
    if (has_value(j, "x")) p.x = j["x"].get<double>();
    if (has_value(j, "y")) p.y = j["y"].get<double>();
    if (has_value(j, "z")) p.z = j["z"].get<double>();
}

inline void from_json(const json& j, Quaternion& q) {
    if (has_value(j, "x")) q.x = j["x"].get<double>();
    if (has_value(j, "y")) q.y = j["y"].get<double>();
    if (has_value(j, "z")) q.z = j["z"].get<double>();
    if (has_value(j, "w")) q.w = j["w"].get<double>();
}

inline void from_json(const json& j, Pose& p) {
    if (has_value(j, "position")) j["position"].get_to(p.position);
    if (has_value(j, "orientation")) j["orientation"].get_to(p.orientation);
}

inline void from_json(const json& j, PoseWithCovariance& p) {
    if (has_value(j, "pose")) j["pose"].get_to(p.pose);
    if (has_value(j, "covariance")) p.covariance = j["covariance"].get<std::vector<double>>();
}

inline void from_json(const json& j, Twist& t) {
    if (has_value(j, "linear")) j["linear"].get_to(t.linear);
    if (has_value(j, "angular")) j["angular"].get_to(t.angular);
}

inline void from_json(const json& j, TwistWithCovariance& t) {
    if (has_value(j, "twist")) j["twist"].get_to(t.twist);
    if (has_value(j, "covariance")) t.covariance = j["covariance"].get<std::vector<double>>();
}

inline void from_json(const json& j, LaserScan& scan) {
    if (has_value(j, "header")) j["header"].get_to(scan.header);
    if (has_value(j, "angle_min")) scan.angle_min = j["angle_min"].get<float>();
    if (has_value(j, "angle_max")) scan.angle_max = j["angle_max"].get<float>();
    if (has_value(j, "angle_increment")) scan.angle_increment = j["angle_increment"].get<float>();
    if (has_value(j, "time_increment")) scan.time_increment = j["time_increment"].get<float>();
    if (has_value(j, "scan_time")) scan.scan_time = j["scan_time"].get<float>();
    if (has_value(j, "range_min")) scan.range_min = j["range_min"].get<float>();
    if (has_value(j, "range_max")) scan.range_max = j["range_max"].get<float>();
    // Use custom parser for arrays that may contain null values
    if (j.contains("ranges") && j["ranges"].is_array()) {
        scan.ranges = parse_float_array(j["ranges"]);
    }
    if (j.contains("intensities") && j["intensities"].is_array()) {
        scan.intensities = parse_float_array(j["intensities"]);
    }
}

inline void from_json(const json& j, Odometry& odom) {
    if (has_value(j, "header")) j["header"].get_to(odom.header);
    if (has_value(j, "child_frame_id")) odom.child_frame_id = j["child_frame_id"].get<std::string>();
    if (has_value(j, "pose")) j["pose"].get_to(odom.pose);
    if (has_value(j, "twist")) j["twist"].get_to(odom.twist);
}

inline void from_json(const json& j, RosbridgeMessage& msg) {
    if (has_value(j, "op")) msg.op = j["op"].get<std::string>();
    if (has_value(j, "topic")) msg.topic = j["topic"].get<std::string>();
    if (j.contains("msg")) msg.msg = j["msg"];  // msg can be any JSON value
}

} // namespace rosbridge

#endif // ROSBRIDGE_TYPES_H
