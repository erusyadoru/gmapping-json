# GMapping JSON Server

ROSに依存せず、UDP経由でJSON形式のセンサーデータを受信してgmappingを実行するスタンドアロンSLAMサーバーです。

## 概要

```
[Robot] --UDP/JSON--> [gmapping_json_server] --> map.pgm
         (scan/odom)
```

- ロボットからLaserScanとOdometryデータをrosbridge形式のJSONでUDP送信
- gmapping_json_serverがそれを受信してSLAM処理
- PGM形式の占有グリッドマップを出力

## ビルド

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 使い方

### サーバー起動

```bash
./build/gmapping_json_server -p 9090 -o map.pgm
```

### オプション

| オプション | 説明 | デフォルト |
|-----------|------|-----------|
| `-p, --port` | UDPリッスンポート | 9090 |
| `-o, --output` | 出力マップファイル | map.pgm |
| `-r, --resolution` | マップ解像度 (m/cell) | 0.05 |
| `--particles` | パーティクル数 | 30 |
| `--max-range` | レーザー最大距離 | 25.0 |
| `--linear-update` | 線形更新閾値 (m) | 0.5 |
| `--angular-update` | 角度更新閾値 (rad) | 0.5 |
| `--map-size` | マップサイズ (m) | 100 |
| `--save-interval` | マップ保存間隔 (s) | 5.0 |

### テスト

シミュレーションデータでテスト:

```bash
# Terminal 1: サーバー起動
./build/gmapping_json_server -p 9090

# Terminal 2: テストデータ送信
python3 tools/test_sender.py --host 127.0.0.1 --port 9090
```

## JSON形式

### LaserScan (rosbridge形式)

```json
{
  "op": "publish",
  "topic": "/scan",
  "msg": {
    "header": {
      "stamp": {"sec": 123, "nanosec": 456789000},
      "frame_id": "laser"
    },
    "angle_min": -1.5707963,
    "angle_max": 1.5707963,
    "angle_increment": 0.00872664,
    "time_increment": 0.0,
    "scan_time": 0.1,
    "range_min": 0.1,
    "range_max": 30.0,
    "ranges": [1.0, 1.1, 1.2, ...],
    "intensities": []
  }
}
```

### Odometry (rosbridge形式)

```json
{
  "op": "publish",
  "topic": "/odom",
  "msg": {
    "header": {
      "stamp": {"sec": 123, "nanosec": 456789000},
      "frame_id": "odom"
    },
    "child_frame_id": "base_link",
    "pose": {
      "pose": {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
      },
      "covariance": [0.0, ...]
    },
    "twist": {
      "twist": {
        "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
      },
      "covariance": [0.0, ...]
    }
  }
}
```

## ROSからの使用

ROS環境でトピックをUDPに転送する場合:

```bash
# gmapping_json_serverを別マシンで起動
./gmapping_json_server -p 9090

# ROSマシンでブリッジ起動
python3 tools/ros_to_udp_bridge.py --host <server_ip> --port 9090
```

## 出力ファイル

- `map.pgm`: 占有グリッドマップ画像
  - 白 (255): フリースペース
  - 黒 (0): 障害物
  - グレー (128): 不明
- `map.yaml`: マップメタデータ (ROS map_server互換)

## アーキテクチャ

```
┌─────────────────────────────────────────────────┐
│                  main.cpp                        │
│  ┌──────────────┐  ┌──────────────────────────┐ │
│  │  UdpServer   │  │    SlamProcessor         │ │
│  │  (UDP受信)    │──▶│  (メッセージ処理)         │ │
│  └──────────────┘  └──────────────────────────┘ │
│                           │                      │
│                           ▼                      │
│               ┌──────────────────────┐          │
│               │  GmappingWrapper     │          │
│               │  (gmapping API)      │          │
│               └──────────────────────┘          │
│                           │                      │
│                           ▼                      │
│               ┌──────────────────────┐          │
│               │  openslam_gmapping   │          │
│               │  (SLAM Core)         │          │
│               └──────────────────────┘          │
└─────────────────────────────────────────────────┘
```

## ライセンス

openslam_gmappingのライセンスに従います。
