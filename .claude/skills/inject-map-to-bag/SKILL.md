---
name: inject-map-to-bag
description: マップをROSBAGに付与する。map.pgmとmap.yamlをrosbagに追加、/mapトピックをbagに含める時に使用。
---

# /inject-map-to-bag

マップ（.pgm/.yaml）をROSBAGに付与するスキル。

## 概要

既存のROSBAGにマップデータ（/mapトピック）を追加します。map_serverでマップを配信しながら入力bagを再生し、全トピックを新しいbagに記録します。

## 使用方法

```
/inject-map-to-bag <map_yaml> <input_bag> <output_bag>
```

### 引数

| 引数 | 説明 |
|------|------|
| `map_yaml` | マップのYAMLファイル（.pgmと同じディレクトリに配置） |
| `input_bag` | 入力ROSBAGファイル |
| `output_bag` | 出力ROSBAGファイル（マップ付き） |

## 実行コマンド

```bash
cd /home/industryalpha/gmapping-json
./tools/inject_map_to_bag.sh <map_yaml> <input_bag> <output_bag>
```

## 実行例

```bash
# 基本的な使用
./tools/inject_map_to_bag.sh map.yaml input.bag output_with_map.bag

# 新しく作成したマップを既存のbagに付与
./tools/inject_map_to_bag.sh new_map.yaml 2025-12-18-10-12-28.bag data_with_map.bag
```

## 処理の流れ

1. Docker内でROSマスターを起動
2. `map_server`でマップを `/map` トピックとして配信
3. `rosbag record -a` で全トピックを記録開始
4. `rosbag play` で入力bagを再生（required=true）
5. 再生完了後、自動的に記録を停止

## 出力

- **出力ROSBAG**: 入力bagの全トピック + `/map` トピック

## 追加されるトピック

| トピック | 型 | 説明 |
|----------|-----|------|
| `/map` | `nav_msgs/OccupancyGrid` | マップデータ |
| `/map_metadata` | `nav_msgs/MapMetaData` | マップメタデータ |

## 要件

- `map.yaml` と `map.pgm` が同じディレクトリに存在すること
- YAMLファイル内の `image:` フィールドがPGMファイル名を指していること

## 注意事項

- 出力bagは入力bagとは異なるファイル名を指定すること
- 処理にはDockerが必要（ROS環境はコンテナ内で実行）
- 大きなbagファイルの処理には時間がかかる場合があります
