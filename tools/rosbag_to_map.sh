#!/bin/bash
#
# rosbag_to_map.sh - ROSBAGからマップを生成するスクリプト
#
# Usage: ./rosbag_to_map.sh [options] <input_bag>
#
# このスクリプトはDocker内で実行されます。
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# デフォルト値
INPUT_BAG=""
OUTPUT_MAP="map.pgm"
OUTPUT_BAG="mapping_output.bag"
SPEED="2.0"
CONFIG_FILE=""
SCAN_TOPIC=""
ODOM_TOPIC=""

# ヘルプ表示
show_help() {
    echo "Usage: $0 [options] <input_bag>"
    echo ""
    echo "ROSBAGファイルからGMappingでマップを生成します。"
    echo ""
    echo "Arguments:"
    echo "  input_bag              入力ROSBAGファイル（必須）"
    echo ""
    echo "Options:"
    echo "  -o, --output MAP       出力マップファイル (default: map.pgm)"
    echo "  -r, --record BAG       記録用ROSBAG (default: mapping_output.bag)"
    echo "  -s, --speed SPEED      再生速度 (default: 2.0)"
    echo "  -c, --config FILE      設定ファイル (default: config/default.json)"
    echo "  --scan-topic TOPIC     scanトピック名 (default: 自動検出)"
    echo "  --odom-topic TOPIC     odomトピック名 (default: 自動検出)"
    echo "  -h, --help             このヘルプを表示"
    echo ""
    echo "Examples:"
    echo "  # 基本的な使用"
    echo "  $0 robot_data.bag"
    echo ""
    echo "  # すべてのオプションを指定"
    echo "  $0 -o mymap.pgm -r record.bag -s 3.0 -c myconfig.json \\"
    echo "     --scan-topic /lidar/scan --odom-topic /robot/odom robot_data.bag"
    echo ""
    echo "Config file (config/default.json) settings:"
    echo "  laser.x, laser.y, laser.theta  : レーザーオフセット"
    echo "  map.resolution                 : マップ解像度"
    echo "  particle_filter.particles      : パーティクル数"
    exit 0
}

# 引数解析
while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            OUTPUT_MAP="$2"
            shift 2
            ;;
        -r|--record)
            OUTPUT_BAG="$2"
            shift 2
            ;;
        -s|--speed)
            SPEED="$2"
            shift 2
            ;;
        -c|--config)
            CONFIG_FILE="$2"
            shift 2
            ;;
        --scan-topic)
            SCAN_TOPIC="$2"
            shift 2
            ;;
        --odom-topic)
            ODOM_TOPIC="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            ;;
        -*)
            echo "Unknown option: $1"
            echo "Use -h for help"
            exit 1
            ;;
        *)
            INPUT_BAG="$1"
            shift
            ;;
    esac
done

# 引数チェック
if [ -z "$INPUT_BAG" ]; then
    echo "Error: Input bag file is required"
    echo "Use -h for help"
    exit 1
fi

# 入力ファイル存在確認
if [ ! -f "$INPUT_BAG" ]; then
    echo "Error: Input bag file not found: $INPUT_BAG"
    exit 1
fi

# 絶対パスに変換
INPUT_BAG="$(realpath "$INPUT_BAG")"
OUTPUT_MAP="$(realpath -m "$OUTPUT_MAP")"
OUTPUT_BAG="$(realpath -m "$OUTPUT_BAG")"

# 設定ファイルの処理
if [ -n "$CONFIG_FILE" ]; then
    if [ ! -f "$CONFIG_FILE" ]; then
        echo "Error: Config file not found: $CONFIG_FILE"
        exit 1
    fi
    CONFIG_FILE="$(realpath "$CONFIG_FILE")"
    CONFIG_IN_CONTAINER="/config/$(basename "$CONFIG_FILE")"
    CONFIG_MOUNT="-v $(dirname "$CONFIG_FILE"):/config"
else
    CONFIG_IN_CONTAINER="/workspace/config/default.json"
    CONFIG_MOUNT=""
fi

# 入力と出力が同じファイルでないことを確認
if [ "$INPUT_BAG" = "$OUTPUT_BAG" ]; then
    echo "Error: Output bag must be different from input bag!"
    exit 1
fi

echo "=== ROSBAGからマップを生成 ==="
echo "入力: $INPUT_BAG"
echo "マップ出力: $OUTPUT_MAP"
echo "ROSBAG出力: $OUTPUT_BAG"
echo "再生速度: ${SPEED}x"
echo "設定ファイル: ${CONFIG_FILE:-config/default.json}"
[ -n "$SCAN_TOPIC" ] && echo "Scanトピック: $SCAN_TOPIC"
[ -n "$ODOM_TOPIC" ] && echo "Odomトピック: $ODOM_TOPIC"
echo ""

# Dockerイメージをビルド（必要な場合）
IMAGE_NAME="gmapping-json"
cd "$PROJECT_DIR"

if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
    echo "Dockerイメージをビルドしています..."
    docker build -t "$IMAGE_NAME" .
fi

# bag_to_udp.pyを生成
BAG_PLAYER="$PROJECT_DIR/tools/bag_player.py"
cat > "$BAG_PLAYER" << 'PYTHON_SCRIPT'
#!/usr/bin/env python3
import rosbag
import socket
import json
import time
import math
import sys
import argparse

def play_bag(bag_path, speed=2.0, scan_topic=None, odom_topic=None):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"Opening bag: {bag_path}")
    bag = rosbag.Bag(bag_path)

    # トピック情報取得
    topics = bag.get_type_and_topic_info().topics

    # トピック自動検出または指定されたトピックを使用
    if scan_topic:
        if scan_topic not in topics:
            print(f"Error: Specified scan topic '{scan_topic}' not found in bag")
            print(f"Available topics: {list(topics.keys())}")
            return False
        print(f"Using specified scan topic: {scan_topic}")
    else:
        for topic in topics:
            if 'scan' in topic.lower():
                scan_topic = topic
                print(f"Scan topic detected: {topic}")
                break

    if odom_topic:
        if odom_topic not in topics:
            print(f"Error: Specified odom topic '{odom_topic}' not found in bag")
            print(f"Available topics: {list(topics.keys())}")
            return False
        print(f"Using specified odom topic: {odom_topic}")
    else:
        for topic in topics:
            if 'odom' in topic.lower():
                odom_topic = topic
                print(f"Odom topic detected: {topic}")
                break

    if not scan_topic or not odom_topic:
        print(f"Error: Required topics not found. scan={scan_topic}, odom={odom_topic}")
        print(f"Available topics: {list(topics.keys())}")
        return False

    messages = []
    for topic, msg, t in bag.read_messages(topics=[scan_topic, odom_topic]):
        messages.append((t.to_sec(), topic, msg))

    messages.sort(key=lambda x: x[0])
    print(f"Loaded {len(messages)} messages")

    start_time = None
    playback_start = time.time()
    scan_count = 0
    odom_count = 0

    for t, topic, msg in messages:
        if start_time is None:
            start_time = t

        elapsed = (t - start_time) / speed
        while time.time() - playback_start < elapsed:
            time.sleep(0.001)

        if topic == odom_topic:
            odom_json = {
                "op": "publish",
                "topic": "/odom",
                "msg": {
                    "header": {
                        "stamp": {"secs": msg.header.stamp.secs, "nsecs": msg.header.stamp.nsecs},
                        "frame_id": msg.header.frame_id
                    },
                    "pose": {"pose": {
                        "position": {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y, "z": msg.pose.pose.position.z},
                        "orientation": {"x": msg.pose.pose.orientation.x, "y": msg.pose.pose.orientation.y, "z": msg.pose.pose.orientation.z, "w": msg.pose.pose.orientation.w}
                    }}
                }
            }
            sock.sendto(json.dumps(odom_json).encode(), ("127.0.0.1", 9090))
            odom_count += 1

        elif topic == scan_topic:
            ranges = []
            for r in msg.ranges:
                if math.isnan(r) or math.isinf(r) or r > msg.range_max:
                    ranges.append(float(msg.range_max))
                elif r < msg.range_min:
                    ranges.append(float(msg.range_min))
                else:
                    ranges.append(float(r))

            scan_json = {
                "op": "publish",
                "topic": "/scan",
                "msg": {
                    "header": {
                        "stamp": {"secs": msg.header.stamp.secs, "nsecs": msg.header.stamp.nsecs},
                        "frame_id": msg.header.frame_id
                    },
                    "angle_min": float(msg.angle_min),
                    "angle_max": float(msg.angle_max),
                    "angle_increment": float(msg.angle_increment),
                    "time_increment": float(msg.time_increment),
                    "scan_time": float(msg.scan_time),
                    "range_min": float(msg.range_min),
                    "range_max": float(msg.range_max),
                    "ranges": ranges,
                    "intensities": []
                }
            }
            sock.sendto(json.dumps(scan_json).encode(), ("127.0.0.1", 9090))
            scan_count += 1

            if scan_count % 50 == 0:
                print(f"Progress: {scan_count} scans, {odom_count} odoms, elapsed={time.time()-playback_start:.1f}s")

    print(f"Done! Sent {scan_count} scans, {odom_count} odoms")
    bag.close()
    return True

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Play ROS bag to UDP')
    parser.add_argument('bag_path', help='Path to bag file')
    parser.add_argument('--speed', type=float, default=2.0, help='Playback speed')
    parser.add_argument('--scan-topic', help='Scan topic name')
    parser.add_argument('--odom-topic', help='Odom topic name')
    args = parser.parse_args()

    success = play_bag(args.bag_path, args.speed, args.scan_topic, args.odom_topic)
    sys.exit(0 if success else 1)
PYTHON_SCRIPT

chmod +x "$BAG_PLAYER"

# Dockerコンテナ内で実行するスクリプト
INNER_SCRIPT="$PROJECT_DIR/tools/run_mapping_inner.sh"
cat > "$INNER_SCRIPT" << 'INNER_BASH'
#!/bin/bash
set -e

INPUT_BAG="$1"
OUTPUT_MAP="$2"
OUTPUT_BAG="$3"
SPEED="$4"
CONFIG_FILE="$5"
SCAN_TOPIC="$6"
ODOM_TOPIC="$7"

# ビルドディレクトリ（マウントの影響を受けない場所）
BUILD_DIR="/opt/gmapping_build"

# 初回はDocker内でビルド
if [ ! -f "$BUILD_DIR/gmapping_json_server" ]; then
    echo "Docker内でビルド中..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake /workspace && make -j$(nproc)
fi

SERVER_BIN="$BUILD_DIR/gmapping_json_server"

cd /data

# gmapping_json_serverをバックグラウンドで起動
echo "GMappingサーバーを起動..."
echo "設定ファイル: $CONFIG_FILE"
"$SERVER_BIN" \
    -c "$CONFIG_FILE" \
    -o "$OUTPUT_MAP" \
    --record-bag "$OUTPUT_BAG" &
SERVER_PID=$!

sleep 2

# サーバーが起動しているか確認
if ! kill -0 $SERVER_PID 2>/dev/null; then
    echo "Error: GMappingサーバーの起動に失敗しました"
    exit 1
fi

echo "ROSBAGを再生中..."

# bag_player.pyのオプション構築
PLAYER_OPTS="--speed $SPEED"
[ -n "$SCAN_TOPIC" ] && PLAYER_OPTS="$PLAYER_OPTS --scan-topic $SCAN_TOPIC"
[ -n "$ODOM_TOPIC" ] && PLAYER_OPTS="$PLAYER_OPTS --odom-topic $ODOM_TOPIC"

python3 /workspace/tools/bag_player.py "$INPUT_BAG" $PLAYER_OPTS || true

# 少し待ってから終了
echo "処理完了を待っています..."
sleep 3

# サーバーを正常終了
echo "GMappingサーバーを停止してマップを保存..."
kill -SIGINT $SERVER_PID 2>/dev/null || true
wait $SERVER_PID 2>/dev/null || true

echo "完了!"
INNER_BASH

chmod +x "$INNER_SCRIPT"

# Dockerコンテナを実行
echo "Docker内でマッピングを実行..."
docker run --rm \
    --network host \
    -v "$PROJECT_DIR:/workspace" \
    -v "$(dirname "$INPUT_BAG"):/data" \
    $CONFIG_MOUNT \
    "$IMAGE_NAME" \
    /workspace/tools/run_mapping_inner.sh \
    "/data/$(basename "$INPUT_BAG")" \
    "/data/$(basename "$OUTPUT_MAP")" \
    "/data/$(basename "$OUTPUT_BAG")" \
    "$SPEED" \
    "$CONFIG_IN_CONTAINER" \
    "$SCAN_TOPIC" \
    "$ODOM_TOPIC"

# 出力ファイルを目的の場所に移動（必要な場合）
INPUT_DIR="$(dirname "$INPUT_BAG")"
if [ "$INPUT_DIR" != "$(dirname "$OUTPUT_MAP")" ]; then
    if [ -f "$INPUT_DIR/$(basename "$OUTPUT_MAP")" ]; then
        mv "$INPUT_DIR/$(basename "$OUTPUT_MAP")" "$OUTPUT_MAP"
        mv "$INPUT_DIR/$(basename "${OUTPUT_MAP%.pgm}.yaml")" "$(dirname "$OUTPUT_MAP")/$(basename "${OUTPUT_MAP%.pgm}.yaml")" 2>/dev/null || true
    fi
fi
if [ "$INPUT_DIR" != "$(dirname "$OUTPUT_BAG")" ]; then
    if [ -f "$INPUT_DIR/$(basename "$OUTPUT_BAG")" ]; then
        mv "$INPUT_DIR/$(basename "$OUTPUT_BAG")" "$OUTPUT_BAG"
    fi
fi

# 結果確認
echo ""
echo "=== 結果 ==="
if [ -f "$OUTPUT_MAP" ]; then
    echo "マップ: $OUTPUT_MAP ($(du -h "$OUTPUT_MAP" | cut -f1))"
elif [ -f "$INPUT_DIR/$(basename "$OUTPUT_MAP")" ]; then
    echo "マップ: $INPUT_DIR/$(basename "$OUTPUT_MAP") ($(du -h "$INPUT_DIR/$(basename "$OUTPUT_MAP")" | cut -f1))"
fi

YAML_FILE="${OUTPUT_MAP%.pgm}.yaml"
if [ -f "$YAML_FILE" ]; then
    echo "メタデータ: $YAML_FILE"
fi

if [ -f "$OUTPUT_BAG" ]; then
    echo "記録ROSBAG: $OUTPUT_BAG ($(du -h "$OUTPUT_BAG" | cut -f1))"
elif [ -f "$INPUT_DIR/$(basename "$OUTPUT_BAG")" ]; then
    echo "記録ROSBAG: $INPUT_DIR/$(basename "$OUTPUT_BAG") ($(du -h "$INPUT_DIR/$(basename "$OUTPUT_BAG")" | cut -f1))"
fi

echo ""
echo "完了!"
