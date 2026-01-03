FROM ros:noetic-ros-base

# rosbridge_serverをインストール
RUN apt-get update && apt-get install -y \
    ros-noetic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# エントリポイントスクリプト
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 9091

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "rosbridge_server", "rosbridge_websocket.launch", "port:=9091"]
