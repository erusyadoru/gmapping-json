#ifndef ROSBAG_WRITER_H
#define ROSBAG_WRITER_H

#include <string>
#include <vector>
#include <fstream>
#include <cstdint>
#include <map>
#include <memory>
#include <cmath>

// Simple ROSBAG writer without ROS dependencies
// Supports writing TF and LaserScan messages

class RosbagWriter {
public:
    // ROS Time structure
    struct Time {
        uint32_t sec;
        uint32_t nsec;

        Time() : sec(0), nsec(0) {}
        Time(double t) {
            sec = static_cast<uint32_t>(t);
            nsec = static_cast<uint32_t>((t - sec) * 1e9);
        }
        double toSec() const { return sec + nsec * 1e-9; }
    };

    // Header structure
    struct Header {
        uint32_t seq;
        Time stamp;
        std::string frame_id;
    };

    // Vector3
    struct Vector3 {
        double x, y, z;
        Vector3() : x(0), y(0), z(0) {}
        Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    // Quaternion
    struct Quaternion {
        double x, y, z, w;
        Quaternion() : x(0), y(0), z(0), w(1) {}
        Quaternion(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}

        // Create from yaw angle
        static Quaternion fromYaw(double yaw) {
            Quaternion q;
            q.x = 0;
            q.y = 0;
            q.z = std::sin(yaw / 2.0);
            q.w = std::cos(yaw / 2.0);
            return q;
        }
    };

    // Transform structure
    struct Transform {
        Vector3 translation;
        Quaternion rotation;
    };

    // TransformStamped (for TF)
    struct TransformStamped {
        Header header;
        std::string child_frame_id;
        Transform transform;
    };

    // LaserScan message
    struct LaserScan {
        Header header;
        float angle_min;
        float angle_max;
        float angle_increment;
        float time_increment;
        float scan_time;
        float range_min;
        float range_max;
        std::vector<float> ranges;
        std::vector<float> intensities;
    };

    RosbagWriter();
    ~RosbagWriter();

    // Open bag file for writing
    bool open(const std::string& filename);

    // Close the bag file
    void close();

    // Write TF message
    void writeTF(const TransformStamped& transform);

    // Write TF static message (for static transforms like base_link -> laser)
    void writeTFStatic(const TransformStamped& transform);

    // Write LaserScan message
    void writeLaserScan(const std::string& topic, const LaserScan& scan);

    // Check if bag is open
    bool isOpen() const { return file_.is_open(); }

private:
    // Connection info for a topic
    struct ConnectionInfo {
        uint32_t id;
        std::string topic;
        std::string datatype;
        std::string md5sum;
        std::string message_definition;
    };

    // Chunk info for index
    struct ChunkInfo {
        uint64_t chunk_pos;
        Time start_time;
        Time end_time;
        uint32_t connection_count;
        std::map<uint32_t, uint32_t> connection_counts;
    };

    // Index entry
    struct IndexEntry {
        Time time;
        uint64_t offset;
    };

    // Serialize primitives
    void writeUint8(uint8_t val);
    void writeUint32(uint32_t val);
    void writeUint64(uint64_t val);
    void writeInt32(int32_t val);
    void writeFloat32(float val);
    void writeFloat64(double val);
    void writeString(const std::string& str);
    void writeTime(const Time& t);
    void writeHeader(const Header& h);

    // Write to buffer (for message serialization)
    void bufferWriteUint8(std::vector<uint8_t>& buf, uint8_t val);
    void bufferWriteUint32(std::vector<uint8_t>& buf, uint32_t val);
    void bufferWriteInt32(std::vector<uint8_t>& buf, int32_t val);
    void bufferWriteFloat32(std::vector<uint8_t>& buf, float val);
    void bufferWriteFloat64(std::vector<uint8_t>& buf, double val);
    void bufferWriteString(std::vector<uint8_t>& buf, const std::string& str);
    void bufferWriteTime(std::vector<uint8_t>& buf, const Time& t);

    // Bag format writing
    void writeBagHeader();
    void updateBagHeader(uint64_t index_pos, uint32_t conn_count, uint32_t chunk_count);
    void writeConnectionRecord(const ConnectionInfo& conn, bool to_file);
    void writeMessageRecord(uint32_t conn_id, const Time& time, const std::vector<uint8_t>& data);

    // Serialize messages
    std::vector<uint8_t> serializeTFMessage(const TransformStamped& tf);
    std::vector<uint8_t> serializeLaserScan(const LaserScan& scan);

    // Get or create connection for topic
    uint32_t getOrCreateConnection(const std::string& topic, const std::string& datatype,
                                    const std::string& md5sum, const std::string& msg_def);

    std::ofstream file_;
    std::string filename_;
    uint32_t next_conn_id_;
    std::map<std::string, uint32_t> topic_to_conn_;
    std::vector<ConnectionInfo> connections_;

    // For building index
    std::vector<std::pair<uint32_t, IndexEntry>> index_entries_;
    Time start_time_;
    Time end_time_;
    bool has_start_time_;

    // Current chunk position
    uint64_t chunk_start_pos_;

    // Buffer for chunk data (all connection + message records)
    std::vector<uint8_t> chunk_buffer_;
};

#endif // ROSBAG_WRITER_H
