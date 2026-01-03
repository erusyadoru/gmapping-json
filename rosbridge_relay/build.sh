#!/bin/bash
#
# Dockerイメージをビルド
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

docker build -t rosbridge-relay "$SCRIPT_DIR"

echo "Done! Image: rosbridge-relay"
