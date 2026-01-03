---
name: rosbag-to-map
description: ROSBAGファイルからGMappingでマップを生成する。rosbagからmap作成、SLAMマッピング、センサーデータからマップ生成時に使用。
---

# /rosbag-to-map

ROSBAGファイルからGMappingを使用してマップを生成するスキル。

## 概要

入力のROSBAGファイル（/scan, /odomトピックを含む）を読み込み、GMappingでSLAM処理を行い、マップ画像（.pgm）とマッピング過程を記録したROSBAGを出力する。

## 使用方法

```
/rosbag-to-map [options] <input_bag>
```

### オプション

| オプション | 説明 | デフォルト |
|------------|------|------------|
| `-o, --output MAP` | 出力マップファイル | map.pgm |
| `-r, --record BAG` | 記録用ROSBAG | mapping_output.bag |
| `-s, --speed SPEED` | 再生速度倍率 | 2.0 |
| `-c, --config FILE` | 設定ファイル | config/default.json |
| `--scan-topic TOPIC` | scanトピック名 | 自動検出 |
| `--odom-topic TOPIC` | odomトピック名 | 自動検出 |

## 実行コマンド

```bash
cd /home/industryalpha/gmapping-json
./tools/rosbag_to_map.sh [options] "$input_bag"
```

## 実行例

```bash
# 基本的な使用
./tools/rosbag_to_map.sh /path/to/robot_data.bag

# トピック名を明示的に指定
./tools/rosbag_to_map.sh --scan-topic /lidar/scan --odom-topic /robot/odom robot_data.bag

# カスタム設定ファイルを使用
./tools/rosbag_to_map.sh -c my_config.json -o mymap.pgm robot_data.bag

# すべてのオプションを指定
./tools/rosbag_to_map.sh -o mymap.pgm -r record.bag -s 3.0 -c myconfig.json \
   --scan-topic /lidar/scan --odom-topic /robot/odom robot_data.bag
```

## 設定ファイル (config/default.json)

```json
{
  "laser": {
    "x": -0.37,      // レーザーのX方向オフセット (m)
    "y": 0.0,        // レーザーのY方向オフセット (m)
    "theta": 3.14159 // レーザーの回転 (rad)
  },
  "map": {
    "size": 100,       // マップサイズ (m)
    "resolution": 0.01 // 解像度 (m/pixel)
  },
  "particle_filter": {
    "particles": 30    // パーティクル数
  }
}
```

## 出力

- **マップ画像**: `output_map` (.pgm形式)
- **メタデータ**: `output_map.yaml` (解像度、原点情報)
- **記録ROSBAG**: `output_bag` (/tf, /scan_estimated トピック)

## 要件

- 入力ROSBAGに `/scan` と `/odom` トピックが含まれていること
- トピック名は部分一致で自動検出（例: `/localization/scan` も検出可能）
- `--scan-topic` / `--odom-topic` で明示的に指定も可能

## 注意事項

- 出力ROSBAGには入力とは**異なるファイル名**を指定すること（上書き防止）
- マッピング中は http://localhost:8080 でリアルタイム可視化が可能
- 処理時間は再生速度に依存（speed=2.0 で実時間の半分）
