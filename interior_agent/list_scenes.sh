#!/bin/bash
# ローカルにダウンロード済みの InteriorAgent シーンを一覧表示

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$REPO_ROOT/.env" ]; then
    set -a
    # shellcheck disable=SC1091
    source "$REPO_ROOT/.env"
    set +a
fi

INTERIOR_AGENT_PATH="${INTERIOR_AGENT_PATH:-$HOME/datasets/InteriorAgent}"

if [ ! -d "$INTERIOR_AGENT_PATH" ]; then
    echo "InteriorAgent ディレクトリが存在しません: $INTERIOR_AGENT_PATH"
    echo "まず download.sh でシーンを取得してください:"
    echo "  ./interior_agent/download.sh kujiale_0003"
    exit 1
fi

echo "利用可能なシーン ($INTERIOR_AGENT_PATH):"
echo ""

FOUND=0
for SCENE_DIR in "$INTERIOR_AGENT_PATH"/kujiale_*/; do
    [ ! -d "$SCENE_DIR" ] && continue

    SCENE_NAME=$(basename "$SCENE_DIR")
    SIZE=$(du -sh "$SCENE_DIR" 2>/dev/null | cut -f1)

    USDA_FILE="$SCENE_DIR/${SCENE_NAME}.usda"
    if [ -f "$USDA_FILE" ]; then
        echo "  - $SCENE_NAME ($SIZE)"
    else
        echo "  - $SCENE_NAME ($SIZE) [不完全: .usda が見つかりません]"
    fi
    FOUND=$((FOUND + 1))
done

echo ""
if [ "$FOUND" -eq 0 ]; then
    echo "シーンがまだダウンロードされていません。"
    echo "  ./interior_agent/download.sh <scene_name>"
else
    echo "合計 $FOUND シーン"
    echo ""
    echo "起動するには:"
    echo "  ./interior_agent/run.sh <scene_name>"
fi
