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
