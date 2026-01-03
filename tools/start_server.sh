#!/bin/bash
#
# start_server.sh - GMappingサーバーを起動するスクリプト
#
# Usage: ./start_server.sh [options]
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
BUILD_DIR="$PROJECT_DIR/build"

# ビルド確認
if [ ! -f "$BUILD_DIR/gmapping_json_server" ]; then
    echo "ビルドが必要です..."
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    cmake .. && make -j$(nproc)
fi

# 引数をそのまま渡して実行
exec "$BUILD_DIR/gmapping_json_server" "$@"
