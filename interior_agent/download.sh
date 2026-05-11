#!/bin/bash
# Spatialverse InteriorAgent から指定されたシーンをダウンロード
#
# 使い方:
#   ./interior_agent/download.sh kujiale_0003
#   ./interior_agent/download.sh kujiale_0003 kujiale_0007 kujiale_0012

set -e

# .env を読み込む(リポジトリルートにある想定)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

if [ -f "$REPO_ROOT/.env" ]; then
    set -a
    # shellcheck disable=SC1091
    source "$REPO_ROOT/.env"
    set +a
fi

# 保存先(環境変数で上書き可能、デフォルトは ~/datasets/InteriorAgent)
INTERIOR_AGENT_PATH="${INTERIOR_AGENT_PATH:-$HOME/datasets/InteriorAgent}"

# 引数チェック
if [ "$#" -eq 0 ]; then
    echo "使い方: $0 <scene_name> [scene_name ...]"
    echo ""
    echo "例:"
    echo "  $0 kujiale_0003"
    echo "  $0 kujiale_0003 kujiale_0007"
    echo ""
    echo "シーン名一覧は以下で確認:"
    echo "  https://huggingface.co/datasets/spatialverse/InteriorAgent/tree/main"
    exit 1
fi

# hf コマンドの存在確認
if ! command -v hf &> /dev/null; then
    echo "エラー: hf コマンドが見つかりません。"
    echo ""
    echo "Hugging Face CLI のインストール:"
    echo "  pip install -U \"huggingface_hub[cli]\""
    exit 1
fi

# 保存先ディレクトリの作成
mkdir -p "$INTERIOR_AGENT_PATH"

echo "保存先: $INTERIOR_AGENT_PATH"
echo ""

# 各シーンをダウンロード
for SCENE_NAME in "$@"; do
    SCENE_DIR="$INTERIOR_AGENT_PATH/$SCENE_NAME"

    if [ -d "$SCENE_DIR" ] && [ -n "$(ls -A "$SCENE_DIR" 2>/dev/null)" ]; then
        echo "[$SCENE_NAME] 既に存在します (スキップ): $SCENE_DIR"
        continue
    fi

    echo "[$SCENE_NAME] ダウンロード開始..."
    hf download spatialverse/InteriorAgent \
        --repo-type dataset \
        --include "$SCENE_NAME/**" \
        --local-dir "$INTERIOR_AGENT_PATH"

    if [ -d "$SCENE_DIR" ]; then
        SIZE=$(du -sh "$SCENE_DIR" | cut -f1)
        echo "[$SCENE_NAME] 完了 ($SIZE)"
    else
        echo "[$SCENE_NAME] 失敗 (ディレクトリが作成されませんでした)"
    fi
    echo ""
done

echo "ダウンロード処理完了"
