#!/bin/bash
#
# rosbridge-relay コンテナを起動
#
# Usage: ./run.sh [rosbridge_url] [options]
#
# Examples:
#   ./run.sh ws://192.168.1.100:9090
#   ./run.sh ws://robot:9090 --topics /scan /odom
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# イメージがなければビルド
if ! docker image inspect rosbridge-relay &>/dev/null; then
    echo "Building image..."
    "$SCRIPT_DIR/build.sh"
fi

# デフォルト値
ROSBRIDGE_URL="${1:-ws://localhost:9090}"
shift 2>/dev/null || true

echo "=== Rosbridge Relay ==="
echo "Rosbridge: $ROSBRIDGE_URL"
echo "UDP target: host (localhost:9090)"
echo ""

# Linuxの場合はhost.docker.internalが使えないので--network hostを使用
if [[ "$(uname)" == "Linux" ]]; then
    docker run --rm -it \
        --network host \
        --name rosbridge-relay \
        rosbridge-relay \
        --rosbridge "$ROSBRIDGE_URL" \
        --udp-host 127.0.0.1 \
        --udp-port 9090 \
        "$@"
else
    # Mac/Windowsの場合
    docker run --rm -it \
        --name rosbridge-relay \
        rosbridge-relay \
        --rosbridge "$ROSBRIDGE_URL" \
        --udp-host host.docker.internal \
        --udp-port 9090 \
        "$@"
fi
