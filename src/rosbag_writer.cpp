#include "rosbag_writer.h"
#include <cstring>
#include <iostream>
#include <algorithm>
#include <cmath>

// MD5 sums and message definitions for ROS messages
namespace {

// tf2_msgs/TFMessage MD5 and definition
const char* TF_MESSAGE_MD5 = "94810edda583a504dfda3829e70d7eec";
const char* TF_MESSAGE_DATATYPE = "tf2_msgs/TFMessage";
const char* TF_MESSAGE_DEFINITION = R"(geometry_msgs/TransformStamped[] transforms

================================================================================
MSG: geometry_msgs/TransformStamped
std_msgs/Header header
string child_frame_id
geometry_msgs/Transform transform

================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id

================================================================================
MSG: geometry_msgs/Transform
geometry_msgs/Vector3 translation
geometry_msgs/Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w)";

// sensor_msgs/LaserScan MD5 and definition
const char* LASER_SCAN_MD5 = "90c7ef2dc6895d81024acba2ac42f369";
const char* LASER_SCAN_DATATYPE = "sensor_msgs/LaserScan";
const char* LASER_SCAN_DEFINITION = R"(std_msgs/Header header
float32 angle_min
float32 angle_max
float32 angle_increment
float32 time_increment
float32 scan_time
float32 range_min
float32 range_max
float32[] ranges
float32[] intensities

================================================================================
MSG: std_msgs/Header
uint32 seq
time stamp
string frame_id)";

} // anonymous namespace

RosbagWriter::RosbagWriter()
    : next_conn_id_(0)
    , has_start_time_(false)
    , chunk_start_pos_(0) {
}

RosbagWriter::~RosbagWriter() {
    if (file_.is_open()) {
        close();
    }
}

bool RosbagWriter::open(const std::string& filename) {
    filename_ = filename;
    file_.open(filename, std::ios::binary | std::ios::in | std::ios::out | std::ios::trunc);

    if (!file_.is_open()) {
        std::cerr << "[RosbagWriter] Failed to open " << filename << std::endl;
        return false;
    }

    // Write bag header (with placeholder values)
    writeBagHeader();

    std::cout << "[RosbagWriter] Opened " << filename << " for writing" << std::endl;
    return true;
}

// Helper to add a header field with length prefix
static void addHeaderField(std::vector<uint8_t>& buf, const std::string& name, const uint8_t* value, size_t value_len) {
    uint32_t field_len = name.size() + 1 + value_len; // name + '=' + value
    for (int i = 0; i < 4; i++) {
        buf.push_back((field_len >> (i * 8)) & 0xFF);
    }
    buf.insert(buf.end(), name.begin(), name.end());
    buf.push_back('=');
    buf.insert(buf.end(), value, value + value_len);
}

// Helper to write uint32 to buffer
static void writeUint32ToBuffer(std::vector<uint8_t>& buf, uint32_t val) {
    for (int i = 0; i < 4; i++) {
        buf.push_back((val >> (i * 8)) & 0xFF);
    }
}

// Helper to write uint64 to buffer
static void writeUint64ToBuffer(std::vector<uint8_t>& buf, uint64_t val) {
    for (int i = 0; i < 8; i++) {
        buf.push_back((val >> (i * 8)) & 0xFF);
    }
}

void RosbagWriter::close() {
    if (!file_.is_open()) return;

    if (chunk_buffer_.empty()) {
        // No messages written
        file_.close();
        std::cout << "[RosbagWriter] Closed bag file with 0 messages" << std::endl;
        return;
    }

    // Write the chunk containing all buffered data
    uint64_t chunk_pos = file_.tellp();

    // CHUNK header
    std::vector<uint8_t> chunk_header;
    uint8_t chunk_op = 0x05;  // CHUNK
    addHeaderField(chunk_header, "op", &chunk_op, 1);

    // compression = none
    std::string compression = "none";
    addHeaderField(chunk_header, "compression", reinterpret_cast<const uint8_t*>(compression.data()), compression.size());

    // size = uncompressed size of chunk data
    uint32_t chunk_data_size = chunk_buffer_.size();
    addHeaderField(chunk_header, "size", reinterpret_cast<uint8_t*>(&chunk_data_size), 4);

    // Write chunk header
    uint32_t chunk_header_len = chunk_header.size();
    file_.write(reinterpret_cast<char*>(&chunk_header_len), 4);
    file_.write(reinterpret_cast<char*>(chunk_header.data()), chunk_header.size());

    // Write chunk data length and data
    file_.write(reinterpret_cast<char*>(&chunk_data_size), 4);
    file_.write(reinterpret_cast<char*>(chunk_buffer_.data()), chunk_buffer_.size());

    // Write INDEX_DATA for each connection
    for (const auto& conn : connections_) {
        // Collect index entries for this connection
        std::vector<IndexEntry> conn_entries;
        for (const auto& entry : index_entries_) {
            if (entry.first == conn.id) {
                conn_entries.push_back(entry.second);
            }
        }

        if (conn_entries.empty()) continue;

        // INDEX_DATA header
        std::vector<uint8_t> index_header;
        uint8_t index_op = 0x04;  // INDEX_DATA
        addHeaderField(index_header, "op", &index_op, 1);

        uint32_t conn_id = conn.id;
        addHeaderField(index_header, "conn", reinterpret_cast<uint8_t*>(&conn_id), 4);

        uint32_t ver = 1;
        addHeaderField(index_header, "ver", reinterpret_cast<uint8_t*>(&ver), 4);

        uint32_t count = conn_entries.size();
        addHeaderField(index_header, "count", reinterpret_cast<uint8_t*>(&count), 4);

        // Write header
        uint32_t index_header_len = index_header.size();
        file_.write(reinterpret_cast<char*>(&index_header_len), 4);
        file_.write(reinterpret_cast<char*>(index_header.data()), index_header.size());

        // INDEX_DATA data: array of (time, offset) pairs
        std::vector<uint8_t> index_data;
        for (const auto& entry : conn_entries) {
            writeUint32ToBuffer(index_data, entry.time.sec);
            writeUint32ToBuffer(index_data, entry.time.nsec);
            writeUint32ToBuffer(index_data, static_cast<uint32_t>(entry.offset));  // offset within chunk
        }

        uint32_t index_data_len = index_data.size();
        file_.write(reinterpret_cast<char*>(&index_data_len), 4);
        file_.write(reinterpret_cast<char*>(index_data.data()), index_data.size());
    }

    // Record where connections start (for index_pos)
    uint64_t index_pos = file_.tellp();

    // Write CONNECTION records at file level
    for (const auto& conn : connections_) {
        writeConnectionRecord(conn, true);  // write to file, not buffer
    }

    // Write CHUNK_INFO record
    std::vector<uint8_t> chunk_info_header;
    uint8_t chunk_info_op = 0x06;  // CHUNK_INFO
    addHeaderField(chunk_info_header, "op", &chunk_info_op, 1);

    uint32_t ver = 1;
    addHeaderField(chunk_info_header, "ver", reinterpret_cast<uint8_t*>(&ver), 4);

    addHeaderField(chunk_info_header, "chunk_pos", reinterpret_cast<uint8_t*>(&chunk_pos), 8);

    addHeaderField(chunk_info_header, "start_time", reinterpret_cast<uint8_t*>(&start_time_), 8);
    addHeaderField(chunk_info_header, "end_time", reinterpret_cast<uint8_t*>(&end_time_), 8);

    uint32_t chunk_conn_count = connections_.size();
    addHeaderField(chunk_info_header, "count", reinterpret_cast<uint8_t*>(&chunk_conn_count), 4);

    // Write header
    uint32_t chunk_info_header_len = chunk_info_header.size();
    file_.write(reinterpret_cast<char*>(&chunk_info_header_len), 4);
    file_.write(reinterpret_cast<char*>(chunk_info_header.data()), chunk_info_header.size());

    // CHUNK_INFO data: array of (conn_id, count) pairs
    std::vector<uint8_t> chunk_info_data;
    for (const auto& conn : connections_) {
        uint32_t msg_count = 0;
        for (const auto& entry : index_entries_) {
            if (entry.first == conn.id) msg_count++;
        }
        writeUint32ToBuffer(chunk_info_data, conn.id);
        writeUint32ToBuffer(chunk_info_data, msg_count);
    }

    uint32_t chunk_info_data_len = chunk_info_data.size();
    file_.write(reinterpret_cast<char*>(&chunk_info_data_len), 4);
    file_.write(reinterpret_cast<char*>(chunk_info_data.data()), chunk_info_data.size());

    // Go back and update the bag header
    updateBagHeader(index_pos, connections_.size(), 1);

    file_.close();
    std::cout << "[RosbagWriter] Closed bag file with " << index_entries_.size() << " messages" << std::endl;
}

void RosbagWriter::writeBagHeader() {
    // Write magic string
    const char* magic = "#ROSBAG V2.0\n";
    file_.write(magic, 13);

    // Write bag header record
    std::vector<uint8_t> header_fields;

    // op field (must be first for parsing)
    uint8_t op = 0x03; // bag header
    addHeaderField(header_fields, "op", &op, 1);

    // index_pos field (placeholder, will be updated on close)
    uint64_t index_pos = 0;
    addHeaderField(header_fields, "index_pos", reinterpret_cast<uint8_t*>(&index_pos), 8);

    // conn_count field (placeholder)
    uint32_t conn_count = 0;
    addHeaderField(header_fields, "conn_count", reinterpret_cast<uint8_t*>(&conn_count), 4);

    // chunk_count field (placeholder)
    uint32_t chunk_count = 0;
    addHeaderField(header_fields, "chunk_count", reinterpret_cast<uint8_t*>(&chunk_count), 4);

    // Write header length
    uint32_t header_len = header_fields.size();
    file_.write(reinterpret_cast<char*>(&header_len), 4);

    // Write header
    file_.write(reinterpret_cast<char*>(header_fields.data()), header_fields.size());

    // Calculate padding to make total bag header record 4096 bytes
    uint32_t padding_size = 4096 - 13 - 4 - header_len - 4;

    // Write data length (padding size)
    file_.write(reinterpret_cast<char*>(&padding_size), 4);

    // Write padding (spaces)
    std::vector<char> padding(padding_size, ' ');
    file_.write(padding.data(), padding_size);
}

void RosbagWriter::updateBagHeader(uint64_t index_pos, uint32_t conn_count, uint32_t chunk_count) {
    // Seek back to the header and rewrite with correct values
    file_.seekp(13);  // After magic

    std::vector<uint8_t> header_fields;

    uint8_t op = 0x03;
    addHeaderField(header_fields, "op", &op, 1);
    addHeaderField(header_fields, "index_pos", reinterpret_cast<uint8_t*>(&index_pos), 8);
    addHeaderField(header_fields, "conn_count", reinterpret_cast<uint8_t*>(&conn_count), 4);
    addHeaderField(header_fields, "chunk_count", reinterpret_cast<uint8_t*>(&chunk_count), 4);

    uint32_t header_len = header_fields.size();
    file_.write(reinterpret_cast<char*>(&header_len), 4);
    file_.write(reinterpret_cast<char*>(header_fields.data()), header_fields.size());

    // Seek back to end
    file_.seekp(0, std::ios::end);
}

void RosbagWriter::writeConnectionRecord(const ConnectionInfo& conn, bool to_file) {
    std::vector<uint8_t> header_fields;

    uint8_t op = 0x07; // connection op
    addHeaderField(header_fields, "op", &op, 1);

    uint32_t conn_id = conn.id;
    addHeaderField(header_fields, "conn", reinterpret_cast<uint8_t*>(&conn_id), 4);

    addHeaderField(header_fields, "topic", reinterpret_cast<const uint8_t*>(conn.topic.data()), conn.topic.size());

    // Connection data fields
    std::vector<uint8_t> data_fields;
    addHeaderField(data_fields, "topic", reinterpret_cast<const uint8_t*>(conn.topic.data()), conn.topic.size());
    addHeaderField(data_fields, "type", reinterpret_cast<const uint8_t*>(conn.datatype.data()), conn.datatype.size());
    addHeaderField(data_fields, "md5sum", reinterpret_cast<const uint8_t*>(conn.md5sum.data()), conn.md5sum.size());
    addHeaderField(data_fields, "message_definition", reinterpret_cast<const uint8_t*>(conn.message_definition.data()), conn.message_definition.size());

    if (to_file) {
        uint32_t header_len = header_fields.size();
        file_.write(reinterpret_cast<char*>(&header_len), 4);
        file_.write(reinterpret_cast<char*>(header_fields.data()), header_fields.size());

        uint32_t data_len = data_fields.size();
        file_.write(reinterpret_cast<char*>(&data_len), 4);
        file_.write(reinterpret_cast<char*>(data_fields.data()), data_fields.size());
    } else {
        // Write to chunk buffer
        writeUint32ToBuffer(chunk_buffer_, header_fields.size());
        chunk_buffer_.insert(chunk_buffer_.end(), header_fields.begin(), header_fields.end());
        writeUint32ToBuffer(chunk_buffer_, data_fields.size());
        chunk_buffer_.insert(chunk_buffer_.end(), data_fields.begin(), data_fields.end());
    }
}

void RosbagWriter::writeMessageRecord(uint32_t conn_id, const Time& time,
                                       const std::vector<uint8_t>& data) {
    // Record offset within chunk (before writing this message)
    uint64_t offset_in_chunk = chunk_buffer_.size();

    std::vector<uint8_t> header_fields;

    uint8_t op = 0x02; // message data op
    addHeaderField(header_fields, "op", &op, 1);
    addHeaderField(header_fields, "conn", reinterpret_cast<uint8_t*>(&conn_id), 4);

    // time field
    uint8_t time_buf[8];
    for (int i = 0; i < 4; i++) {
        time_buf[i] = (time.sec >> (i * 8)) & 0xFF;
    }
    for (int i = 0; i < 4; i++) {
        time_buf[4 + i] = (time.nsec >> (i * 8)) & 0xFF;
    }
    addHeaderField(header_fields, "time", time_buf, 8);

    // Write to chunk buffer
    writeUint32ToBuffer(chunk_buffer_, header_fields.size());
    chunk_buffer_.insert(chunk_buffer_.end(), header_fields.begin(), header_fields.end());

    uint32_t data_len = data.size();
    writeUint32ToBuffer(chunk_buffer_, data_len);
    chunk_buffer_.insert(chunk_buffer_.end(), data.begin(), data.end());

    // Update times
    if (!has_start_time_) {
        start_time_ = time;
        has_start_time_ = true;
    }
    end_time_ = time;

    // Add to index (offset is within the chunk)
    IndexEntry entry;
    entry.time = time;
    entry.offset = offset_in_chunk;
    index_entries_.push_back({conn_id, entry});
}

uint32_t RosbagWriter::getOrCreateConnection(const std::string& topic,
                                              const std::string& datatype,
                                              const std::string& md5sum,
                                              const std::string& msg_def) {
    auto it = topic_to_conn_.find(topic);
    if (it != topic_to_conn_.end()) {
        return it->second;
    }

    // Create new connection
    ConnectionInfo conn;
    conn.id = next_conn_id_++;
    conn.topic = topic;
    conn.datatype = datatype;
    conn.md5sum = md5sum;
    conn.message_definition = msg_def;

    topic_to_conn_[topic] = conn.id;
    connections_.push_back(conn);

    // Write connection record to chunk buffer
    writeConnectionRecord(conn, false);

    return conn.id;
}

// Buffer write helpers
void RosbagWriter::bufferWriteUint8(std::vector<uint8_t>& buf, uint8_t val) {
    buf.push_back(val);
}

void RosbagWriter::bufferWriteUint32(std::vector<uint8_t>& buf, uint32_t val) {
    for (int i = 0; i < 4; i++) {
        buf.push_back((val >> (i * 8)) & 0xFF);
    }
}

void RosbagWriter::bufferWriteInt32(std::vector<uint8_t>& buf, int32_t val) {
    bufferWriteUint32(buf, static_cast<uint32_t>(val));
}

void RosbagWriter::bufferWriteFloat32(std::vector<uint8_t>& buf, float val) {
    uint32_t* ptr = reinterpret_cast<uint32_t*>(&val);
    bufferWriteUint32(buf, *ptr);
}

void RosbagWriter::bufferWriteFloat64(std::vector<uint8_t>& buf, double val) {
    uint64_t* ptr = reinterpret_cast<uint64_t*>(&val);
    for (int i = 0; i < 8; i++) {
        buf.push_back((*ptr >> (i * 8)) & 0xFF);
    }
}

void RosbagWriter::bufferWriteString(std::vector<uint8_t>& buf, const std::string& str) {
    bufferWriteUint32(buf, str.size());
    buf.insert(buf.end(), str.begin(), str.end());
}

void RosbagWriter::bufferWriteTime(std::vector<uint8_t>& buf, const Time& t) {
    bufferWriteUint32(buf, t.sec);
    bufferWriteUint32(buf, t.nsec);
}

std::vector<uint8_t> RosbagWriter::serializeTFMessage(const TransformStamped& tf) {
    std::vector<uint8_t> buf;

    // tf2_msgs/TFMessage is an array of TransformStamped
    bufferWriteUint32(buf, 1);

    // TransformStamped:
    bufferWriteUint32(buf, tf.header.seq);
    bufferWriteTime(buf, tf.header.stamp);
    bufferWriteString(buf, tf.header.frame_id);
    bufferWriteString(buf, tf.child_frame_id);

    // Transform
    bufferWriteFloat64(buf, tf.transform.translation.x);
    bufferWriteFloat64(buf, tf.transform.translation.y);
    bufferWriteFloat64(buf, tf.transform.translation.z);
    bufferWriteFloat64(buf, tf.transform.rotation.x);
    bufferWriteFloat64(buf, tf.transform.rotation.y);
    bufferWriteFloat64(buf, tf.transform.rotation.z);
    bufferWriteFloat64(buf, tf.transform.rotation.w);

    return buf;
}

std::vector<uint8_t> RosbagWriter::serializeLaserScan(const LaserScan& scan) {
    std::vector<uint8_t> buf;

    bufferWriteUint32(buf, scan.header.seq);
    bufferWriteTime(buf, scan.header.stamp);
    bufferWriteString(buf, scan.header.frame_id);

    bufferWriteFloat32(buf, scan.angle_min);
    bufferWriteFloat32(buf, scan.angle_max);
    bufferWriteFloat32(buf, scan.angle_increment);
    bufferWriteFloat32(buf, scan.time_increment);
    bufferWriteFloat32(buf, scan.scan_time);
    bufferWriteFloat32(buf, scan.range_min);
    bufferWriteFloat32(buf, scan.range_max);

    bufferWriteUint32(buf, scan.ranges.size());
    for (float r : scan.ranges) {
        bufferWriteFloat32(buf, r);
    }

    bufferWriteUint32(buf, scan.intensities.size());
    for (float i : scan.intensities) {
        bufferWriteFloat32(buf, i);
    }

    return buf;
}

void RosbagWriter::writeTF(const TransformStamped& transform) {
    if (!file_.is_open()) return;

    uint32_t conn_id = getOrCreateConnection("/tf", TF_MESSAGE_DATATYPE,
                                              TF_MESSAGE_MD5, TF_MESSAGE_DEFINITION);

    std::vector<uint8_t> data = serializeTFMessage(transform);
    writeMessageRecord(conn_id, transform.header.stamp, data);
}

void RosbagWriter::writeTFStatic(const TransformStamped& transform) {
    if (!file_.is_open()) return;

    uint32_t conn_id = getOrCreateConnection("/tf_static", TF_MESSAGE_DATATYPE,
                                              TF_MESSAGE_MD5, TF_MESSAGE_DEFINITION);

    std::vector<uint8_t> data = serializeTFMessage(transform);
    writeMessageRecord(conn_id, transform.header.stamp, data);
}

void RosbagWriter::writeLaserScan(const std::string& topic, const LaserScan& scan) {
    if (!file_.is_open()) return;

    uint32_t conn_id = getOrCreateConnection(topic, LASER_SCAN_DATATYPE,
                                              LASER_SCAN_MD5, LASER_SCAN_DEFINITION);

    std::vector<uint8_t> data = serializeLaserScan(scan);
    writeMessageRecord(conn_id, scan.header.stamp, data);
}
