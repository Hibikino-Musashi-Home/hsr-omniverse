#!/usr/bin/env python3
"""configs/textures/ のプレースホルダ画像を生成する。

リポジトリに同梱する小さな (合計 100KB 程度) サンプルテクスチャを作る。
実本番テクスチャを使うときは、各 .jpg をそのまま上書きするか、compose の
mount を ${HOME}/datasets/SceneTextures などに切り替える。

実行: python3 configs/textures/generate_textures.py
"""
from __future__ import annotations

from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


# このスクリプトと同じディレクトリに出力する
OUT_DIR = Path(__file__).parent
OUT_DIR.mkdir(exist_ok=True)

# DejaVu フォントが無くてもフォールバックする
def _load_font(size: int):
    for path in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans-Bold.ttf",
    ):
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


FONT_LARGE = _load_font(36)
FONT_SMALL = _load_font(18)


def make_wall(filename: str, color: tuple, direction: str) -> None:
    """ラベル付きの単色壁プレースホルダ。"""
    img = Image.new("RGB", (512, 256), color)
    d = ImageDraw.Draw(img)

    # 方角ラベル (中央上)
    d.text((256, 90), direction, fill=(255, 255, 255), font=FONT_LARGE, anchor="mm")
    # SAMPLE 表示
    d.text((256, 140), "PLACEHOLDER", fill=(255, 255, 255), font=FONT_SMALL, anchor="mm")
    # ファイル名表示
    d.text((256, 170), filename, fill=(255, 255, 255), font=FONT_SMALL, anchor="mm")
    # 説明
    d.text((256, 210),
           "Override by replacing this file or changing the compose mount.",
           fill=(255, 255, 255), font=FONT_SMALL, anchor="mm")

    # 枠線
    d.rectangle((0, 0, 511, 255), outline=(255, 255, 255), width=4)

    out = OUT_DIR / filename
    img.save(out, "JPEG", quality=80)
    print(f"  {filename}: color={color} dir='{direction}' size={out.stat().st_size} bytes")


def make_floor() -> None:
    """タイル模様の床プレースホルダ。"""
    size = 256
    img = Image.new("RGB", (size, size), (180, 180, 180))
    d = ImageDraw.Draw(img)

    # タイル境界
    tile = 64
    for i in range(0, size + 1, tile):
        d.line((i, 0, i, size), fill=(100, 100, 100), width=2)
        d.line((0, i, size, i), fill=(100, 100, 100), width=2)

    # 中央ラベル
    d.text((size // 2, size // 2 - 14), "FLOOR", fill=(60, 60, 60),
           font=FONT_SMALL, anchor="mm")
    d.text((size // 2, size // 2 + 8), "PLACEHOLDER", fill=(60, 60, 60),
           font=FONT_SMALL, anchor="mm")

    out = OUT_DIR / "floor.jpg"
    img.save(out, "JPEG", quality=80)
    print(f"  floor.jpg: tile pattern size={out.stat().st_size} bytes")


def main() -> None:
    print(f"Generating placeholder textures into {OUT_DIR} ...")
    # 方角ごとに視認しやすい色に
    make_wall("wall_1.jpg", (60, 100, 180), "NORTH  (+Y)")     # 青
    make_wall("wall_2.jpg", (180, 80, 80), "SOUTH  (-Y)")      # 赤
    make_wall("wall_3.jpg", (80, 160, 100), "EAST  (+X)")      # 緑
    make_wall("wall_4.jpg", (200, 180, 60), "WEST  (-X)")      # 黄
    make_floor()
    total = sum(p.stat().st_size for p in OUT_DIR.glob("*.jpg"))
    print(f"Done. Total size: {total / 1024:.1f} KB")


if __name__ == "__main__":
    main()
