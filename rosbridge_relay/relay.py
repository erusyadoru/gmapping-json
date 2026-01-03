#!/usr/bin/env python3
"""
rosbridge_relay.py - rosbridgeからトピックを受信してUDPで転送

rosbridgeに接続し、指定されたトピックをsubscribeして
JSON形式のままUDPでgmapping_json_serverに転送する
"""

import asyncio
import json
import socket
import argparse
import signal
import sys
import time
from websockets import connect
from websockets.exceptions import ConnectionClosed

def sanitize_nulls(obj, default_num=0.0):
    """再帰的にnull値を適切なデフォルト値に置換"""
    if obj is None:
        return default_num
    elif isinstance(obj, dict):
        return {k: sanitize_nulls(v, default_num) for k, v in obj.items()}
    elif isinstance(obj, list):
        return [sanitize_nulls(item, default_num) for item in obj]
    else:
        return obj


class RosbridgeRelay:
    def __init__(self, rosbridge_url, udp_host, udp_port, topics, throttle_hz=None):
        self.rosbridge_url = rosbridge_url
        self.udp_host = udp_host
        self.udp_port = udp_port
        self.topics = topics
        self.throttle_hz = throttle_hz
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.running = True
        self.msg_count = {topic: 0 for topic in topics}
        self.subscribed = {topic: False for topic in topics}
        self.last_send_time = {topic: 0 for topic in topics}

    async def subscribe(self, ws, topic):
        """トピックをsubscribe（型を指定）"""
        # トピック名から型を推測
        if 'scan' in topic.lower():
            msg_type = 'sensor_msgs/LaserScan'
        elif 'odom' in topic.lower():
            msg_type = 'nav_msgs/Odometry'
        else:
            msg_type = None

        subscribe_msg = {
            "op": "subscribe",
            "topic": topic
        }
        if msg_type:
            subscribe_msg["type"] = msg_type

        await ws.send(json.dumps(subscribe_msg))
        self.subscribed[topic] = True
        print(f"[Relay] Subscribed to {topic} (type: {msg_type})")

    async def run(self):
        """メインループ"""
        while self.running:
            try:
                print(f"[Relay] Connecting to {self.rosbridge_url}...")
                async with connect(self.rosbridge_url) as ws:
                    print(f"[Relay] Connected!")

                    # すべてのトピックをsubscribe
                    for topic in self.topics:
                        await self.subscribe(ws, topic)

                    # メッセージ受信ループ
                    while self.running:
                        try:
                            message = await asyncio.wait_for(ws.recv(), timeout=1.0)
                            self.handle_message(message)
                        except asyncio.TimeoutError:
                            continue
                        except ConnectionClosed:
                            print("[Relay] Connection closed, reconnecting...")
                            break

            except Exception as e:
                print(f"[Relay] Connection error: {e}, retrying in 3s...")
                await asyncio.sleep(3)

    def handle_message(self, message):
        """受信したメッセージをUDPで転送"""
        try:
            data = json.loads(message)

            if data.get("op") == "publish":
                topic = data.get("topic", "")

                # スロットリング（指定されている場合）
                if self.throttle_hz and topic in self.last_send_time:
                    now = time.time()
                    min_interval = 1.0 / self.throttle_hz
                    if now - self.last_send_time[topic] < min_interval:
                        return  # スキップ
                    self.last_send_time[topic] = now

                # null値をデフォルト値に置換
                data = sanitize_nulls(data)

                # トピック名をgmappingが期待する形式に変換
                # /wr_scan_rear -> /scan
                if "scan" in topic.lower():
                    data["topic"] = "/scan"
                elif "odom" in topic.lower():
                    data["topic"] = "/odom"

                # UDPで送信
                msg_bytes = json.dumps(data).encode('utf-8')
                self.sock.sendto(msg_bytes, (self.udp_host, self.udp_port))

                # カウント
                if topic in self.msg_count:
                    self.msg_count[topic] += 1
                    if self.msg_count[topic] % 100 == 0:
                        print(f"[Relay] {topic}: {self.msg_count[topic]} msgs forwarded")

        except json.JSONDecodeError as e:
            print(f"[Relay] JSON decode error: {e}")
        except Exception as e:
            print(f"[Relay] Error handling message: {e}")

    def stop(self):
        """停止"""
        self.running = False
        print("[Relay] Stopping...")
        for topic, count in self.msg_count.items():
            print(f"[Relay] Total {topic}: {count} msgs")


def main():
    parser = argparse.ArgumentParser(description='Rosbridge to UDP relay')
    parser.add_argument('--rosbridge', '-r', default='ws://localhost:9090',
                        help='Rosbridge WebSocket URL (default: ws://localhost:9090)')
    parser.add_argument('--udp-host', '-H', default='127.0.0.1',
                        help='UDP destination host (default: 127.0.0.1)')
    parser.add_argument('--udp-port', '-p', type=int, default=9090,
                        help='UDP destination port (default: 9090)')
    parser.add_argument('--topics', '-t', nargs='+',
                        default=['/wr_scan_rear', '/odom'],
                        help='Topics to subscribe (default: /wr_scan_rear /odom)')
    parser.add_argument('--throttle', type=float, default=None,
                        help='Max message rate in Hz (default: no limit)')
    args = parser.parse_args()

    print("=== Rosbridge to UDP Relay ===")
    print(f"Rosbridge: {args.rosbridge}")
    print(f"UDP target: {args.udp_host}:{args.udp_port}")
    print(f"Topics: {args.topics}")
    print(f"Throttle: {args.throttle} Hz" if args.throttle else "Throttle: disabled")
    print()

    relay = RosbridgeRelay(
        args.rosbridge,
        args.udp_host,
        args.udp_port,
        args.topics,
        args.throttle
    )

    # シグナルハンドラ
    def signal_handler(sig, frame):
        relay.stop()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 実行
    asyncio.run(relay.run())


if __name__ == '__main__':
    main()
