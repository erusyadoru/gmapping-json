#!/bin/bash
#
# inject_map_to_bag.sh - マップをROSBAGに付与するスクリプト
#
# Usage: ./inject_map_to_bag.sh [options] <map_yaml> <input_bag> <output_bag>
#

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# デフォルト値
MAP_YAML=""
INPUT_BAG=""
OUTPUT_BAG=""

# ヘルプ表示
show_help() {
    echo "Usage: $0 <map_yaml> <input_bag> <output_bag>"
    echo ""
    echo "マップをROSBAGに付与します。"
    echo ""
    echo "Arguments:"
    echo "  map_yaml    マップのYAMLファイル（.pgmと同じディレクトリにあること）"
    echo "  input_bag   入力ROSBAGファイル"
    echo "  output_bag  出力ROSBAGファイル（マップ付き）"
    echo ""
    echo "Options:"
    echo "  -h, --help  このヘルプを表示"
    echo ""
    echo "Examples:"
    echo "  $0 map.yaml input.bag output_with_map.bag"
    echo ""
    echo "Note:"
    echo "  - map.yamlとmap.pgmは同じディレクトリに配置してください"
    echo "  - 出力bagには/mapトピックが追加されます"
    exit 0
}

# 引数解析
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            ;;
        -*)
            echo "Unknown option: $1"
            echo "Use -h for help"
            exit 1
            ;;
        *)
            if [ -z "$MAP_YAML" ]; then
                MAP_YAML="$1"
            elif [ -z "$INPUT_BAG" ]; then
                INPUT_BAG="$1"
            elif [ -z "$OUTPUT_BAG" ]; then
                OUTPUT_BAG="$1"
            fi
            shift
            ;;
    esac
done

# 引数チェック
if [ -z "$MAP_YAML" ] || [ -z "$INPUT_BAG" ] || [ -z "$OUTPUT_BAG" ]; then
    echo "Error: All arguments are required"
    echo "Usage: $0 <map_yaml> <input_bag> <output_bag>"
    exit 1
fi

# 入力ファイル存在確認
if [ ! -f "$MAP_YAML" ]; then
    echo "Error: Map YAML file not found: $MAP_YAML"
    exit 1
fi

if [ ! -f "$INPUT_BAG" ]; then
    echo "Error: Input bag file not found: $INPUT_BAG"
    exit 1
fi

# PGMファイルの確認（YAMLと同じディレクトリ）
MAP_DIR="$(dirname "$(realpath "$MAP_YAML")")"
MAP_YAML_BASENAME="$(basename "$MAP_YAML")"
MAP_YAML_NAME="${MAP_YAML_BASENAME%.yaml}"

# YAMLからPGMファイル名を取得
PGM_PATH=$(grep "^image:" "$MAP_YAML" | sed 's/image: *//' | tr -d '\r')
if [ -z "$PGM_PATH" ]; then
    echo "Error: Could not find 'image:' field in $MAP_YAML"
    exit 1
fi

# PGMファイル名を取得（絶対パスの場合はbasenameを取る）
PGM_BASENAME="$(basename "$PGM_PATH")"

# PGMファイルの存在確認（YAMLと同じディレクトリで探す）
if [ -f "$MAP_DIR/$PGM_BASENAME" ]; then
    PGM_FILE="$MAP_DIR/$PGM_BASENAME"
elif [ -f "$PGM_PATH" ]; then
    # 絶対パスで存在する場合
    PGM_FILE="$PGM_PATH"
else
    echo "Error: PGM file not found: $MAP_DIR/$PGM_BASENAME or $PGM_PATH"
    exit 1
fi

# 絶対パスに変換
MAP_YAML="$(realpath "$MAP_YAML")"
INPUT_BAG="$(realpath "$INPUT_BAG")"
OUTPUT_BAG="$(realpath -m "$OUTPUT_BAG")"
OUTPUT_DIR="$(dirname "$OUTPUT_BAG")"
OUTPUT_BASENAME="$(basename "$OUTPUT_BAG" .bag)"

# 入力と出力が同じファイルでないことを確認
if [ "$INPUT_BAG" = "$OUTPUT_BAG" ]; then
    echo "Error: Output bag must be different from input bag!"
    exit 1
fi

echo "=== マップをROSBAGに付与 ==="
echo "マップYAML: $MAP_YAML"
echo "マップPGM: $PGM_FILE"
echo "入力BAG: $INPUT_BAG"
echo "出力BAG: $OUTPUT_BAG"
echo ""

# ROSイメージ（map_serverが使えるもの）
ROS_IMAGE="industryalphainc/ia-amr-ros:base"

# Dockerイメージ確認
if ! docker image inspect "$ROS_IMAGE" &>/dev/null; then
    echo "Dockerイメージをプルしています: $ROS_IMAGE"
    docker pull "$ROS_IMAGE"
fi

# 作業ディレクトリ作成
WORK_DIR=$(mktemp -d)
trap "rm -rf $WORK_DIR" EXIT

# launchファイルをコピー
cp "$SCRIPT_DIR/inject_map.launch" "$WORK_DIR/"

# マップファイルをワークディレクトリにコピー（Docker内でのパスを統一するため）
cp "$PGM_FILE" "$WORK_DIR/map.pgm"

# YAMLファイルを作成（Docker内でのパスに合わせる）
cat > "$WORK_DIR/map.yaml" << EOF
image: /work/map.pgm
resolution: $(grep "^resolution:" "$MAP_YAML" | sed 's/resolution: *//')
origin: $(grep "^origin:" "$MAP_YAML" | sed 's/origin: *//')
negate: $(grep "^negate:" "$MAP_YAML" | sed 's/negate: *//')
occupied_thresh: $(grep "^occupied_thresh:" "$MAP_YAML" | sed 's/occupied_thresh: *//')
free_thresh: $(grep "^free_thresh:" "$MAP_YAML" | sed 's/free_thresh: *//')
EOF

# Dockerコンテナ内で実行するスクリプト
cat > "$WORK_DIR/run_inject.sh" << 'INNER_SCRIPT'
#!/bin/bash
set -e

MAP_YAML="$1"
INPUT_BAG="$2"
OUTPUT_NAME="$3"

cd /output

echo "ROSマスターを起動..."
source /opt/ros/noetic/setup.bash 2>/dev/null || source /opt/ros/melodic/setup.bash

# roscoreをバックグラウンドで起動
roscore &
ROSCORE_PID=$!
sleep 3

echo "Launchファイルを実行..."
# roslaunchを実行
timeout 300 roslaunch /work/inject_map.launch \
    map_file:="$MAP_YAML" \
    input_bag:="$INPUT_BAG" \
    output_bag:="/output/$OUTPUT_NAME" \
    --wait || true

# roscoreを停止
kill $ROSCORE_PID 2>/dev/null || true

echo "完了!"
INNER_SCRIPT

chmod +x "$WORK_DIR/run_inject.sh"

# Dockerコンテナを実行
echo "Docker内でマップ付与を実行..."
docker run --rm \
    -v "$WORK_DIR:/work" \
    -v "$(dirname "$INPUT_BAG"):/input:ro" \
    -v "$OUTPUT_DIR:/output" \
    "$ROS_IMAGE" \
    /work/run_inject.sh \
    "/work/map.yaml" \
    "/input/$(basename "$INPUT_BAG")" \
    "$OUTPUT_BASENAME"

# 出力ファイルの確認
if [ -f "$OUTPUT_DIR/${OUTPUT_BASENAME}.bag" ]; then
    # ファイル名が期待通りでない場合はリネーム
    if [ "$OUTPUT_DIR/${OUTPUT_BASENAME}.bag" != "$OUTPUT_BAG" ]; then
        mv "$OUTPUT_DIR/${OUTPUT_BASENAME}.bag" "$OUTPUT_BAG"
    fi
fi

# 結果確認
echo ""
echo "=== 結果 ==="
if [ -f "$OUTPUT_BAG" ]; then
    echo "出力BAG: $OUTPUT_BAG ($(du -h "$OUTPUT_BAG" | cut -f1))"
    echo ""
    echo "トピック一覧:"
    docker run --rm -v "$OUTPUT_DIR:/data:ro" "$ROS_IMAGE" \
        bash -c "source /opt/ros/*/setup.bash && rosbag info /data/$(basename "$OUTPUT_BAG") 2>/dev/null | grep -E '(topics:|msgs)'" || true
else
    echo "Error: 出力ファイルが作成されませんでした"
    exit 1
fi

echo ""
echo "完了!"
