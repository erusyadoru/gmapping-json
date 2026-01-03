#!/bin/bash
set -e

# ROS環境をセットアップ
source /opt/ros/noetic/setup.bash

# roscoreをバックグラウンドで起動
roscore &
ROSCORE_PID=$!

# roscoreが起動するまで待機
sleep 2

echo "[rosbridge_server] roscore started (PID: $ROSCORE_PID)"
echo "[rosbridge_server] Starting rosbridge_server..."

# 引数で渡されたコマンドを実行
exec "$@"
