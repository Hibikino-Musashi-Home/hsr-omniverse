#!/bin/bash
# 指定された InteriorAgent シーンで HSR を起動
#
# 使い方:
#   ./interior_agent/run.sh kujiale_0003

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ "$#" -ne 1 ]; then
    echo "使い方: $0 <scene_name>"
    echo ""
    echo "例:"
    echo "  $0 kujiale_0003"
    echo ""
    echo "利用可能なシーン一覧:"
    echo "  $SCRIPT_DIR/list_scenes.sh"
    exit 1
fi

SCENE_NAME="$1"

if [ -f "$REPO_ROOT/.env" ]; then
    set -a
    # shellcheck disable=SC1091
    source "$REPO_ROOT/.env"
    set +a
fi
INTERIOR_AGENT_PATH="${INTERIOR_AGENT_PATH:-$HOME/datasets/InteriorAgent}"

SCENE_USDA="$INTERIOR_AGENT_PATH/$SCENE_NAME/${SCENE_NAME}.usda"
if [ ! -f "$SCENE_USDA" ]; then
    echo "エラー: シーンが見つかりません: $SCENE_USDA"
    echo ""
    echo "まずダウンロードしてください:"
    echo "  ./interior_agent/download.sh $SCENE_NAME"
    exit 1
fi

xhost +local:root > /dev/null 2>&1

cd "$REPO_ROOT"
COMPOSE_FILES="-f docker-compose-ros2.yml -f docker-compose.dev.yml"

if ! docker compose $COMPOSE_FILES ps --status running --services 2>/dev/null | grep -q "isaacsim"; then
    echo "エラー: isaacsim コンテナが起動していません。"
    echo ""
    echo "別のターミナルで以下を実行してください:"
    echo "  cd $REPO_ROOT"
    echo "  docker compose $COMPOSE_FILES up"
    exit 1
fi

echo "シーン: $SCENE_NAME"
echo "USD: /data/InteriorAgent/$SCENE_NAME/${SCENE_NAME}.usda (コンテナ内パス)"
echo ""
echo "Isaac Sim を起動中..."
echo ""

docker compose $COMPOSE_FILES exec \
    -e INTERIOR_AGENT_SCENE="$SCENE_NAME" \
    isaacsim \
    /ros_entrypoint.sh /isaac-sim/python.sh /app/sample-interior.py
