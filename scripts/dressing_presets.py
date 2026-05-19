#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
dressing_presets: scene_dressing 用のテクスチャ・照明プリセット集

construct_environment.apply_lab_dressing(preset="...", lighting="...") から
名前指定で呼び出される。

新しいテーマを追加する手順:
    1. /data/<NewTextures>/ にテクスチャ画像 5 枚を置く
       (床 + 北南東西の 4 壁背景画像)
    2. このファイルの DRESSING_PRESETS にエントリを追加
    3. apply_lab_dressing(preset="new_theme") で呼ぶ

新しい照明モードを追加する手順:
    1. このファイルの LIGHTING_PRESETS にエントリを追加
    2. apply_lab_dressing(lighting="new_mode") で呼ぶ

両方の辞書のキーは EnvBoxConfig (scene_dressing/__init__.py) のフィールド名と
対応している。未指定のフィールドは EnvBoxConfig のデフォルトが使われる。
"""
from __future__ import annotations


# ============================================================
# テクスチャプリセット
# ============================================================
#
# キー: 床と 4 壁背景の画像パス、必要に応じて room_size などサイズも指定可能。
# 値が未指定のフィールドは EnvBoxConfig のデフォルトを使う。

DRESSING_PRESETS = {
    "lab": {
        "floor_texture": "/data/LabTextures/lab_floor.jpg",
        "backdrop_textures": (
            "/data/LabTextures/lab_wall_1.jpg",  # 北 (+Y)
            "/data/LabTextures/lab_wall_2.jpg",  # 南 (-Y)
            "/data/LabTextures/lab_wall_3.jpg",  # 東 (+X)
            "/data/LabTextures/lab_wall_4.jpg",  # 西 (-X)
        ),
        # オプション項目 (未指定なら EnvBoxConfig のデフォルト):
        # "floor_tile": 4.0,
        # "room_size": 15.0,
        # "room_height": 4.0,
    },

    # 床のみ (背景なし) のサンプル
    "floor_only": {
        "floor_texture": "/data/LabTextures/lab_floor.jpg",
        "backdrop_textures": None,
    },

    # 背景のみ (床なし) のサンプル
    "backdrop_only": {
        "floor_texture": None,
        "backdrop_textures": (
            "/data/LabTextures/lab_wall_1.jpg",
            "/data/LabTextures/lab_wall_2.jpg",
            "/data/LabTextures/lab_wall_3.jpg",
            "/data/LabTextures/lab_wall_4.jpg",
        ),
    },

    # ---- 新しいテーマを足すときの例 (コメントアウト) ----
    # "office": {
    #     "floor_texture": "/data/OfficeTextures/carpet.jpg",
    #     "backdrop_textures": (
    #         "/data/OfficeTextures/office_n.jpg",
    #         "/data/OfficeTextures/office_s.jpg",
    #         "/data/OfficeTextures/office_e.jpg",
    #         "/data/OfficeTextures/office_w.jpg",
    #     ),
    #     "floor_tile": 6.0,
    # },
}


# ============================================================
# 照明プリセット
# ============================================================
#
# キーは EnvBoxConfig の照明関連フィールド + 拡張キー:
#   - dome_intensity / ceiling_intensity / ceiling_count /
#     ceiling_height / ceiling_color_temp / enable_lighting (EnvBoxConfig)
#   - default_lights_intensity (拡張): launch_isaacsim.py が作る
#       /World/Light_1, /World/Light_2 の強度を連動制御する。
#       これが無いとシーンに残っている既存ライト (各 5e4) がドミネートして
#       プリセット切り替えで見た目があまり変わらない。
#
# プリセット間で見た目をはっきり変えるため、強度差は桁レベルで広く取っている。

LIGHTING_PRESETS = {
    # 標準: 壁で囲まれた部屋がしっかり明るく見える
    "default": {
        "dome_intensity": 3000.0,
        "ceiling_intensity": 1.5e5,
        "ceiling_count": 4,
        "ceiling_height": 3.7,
        "ceiling_color_temp": 6500.0,  # 昼白色
        "default_lights_intensity": 5e4,  # launch_isaacsim.py 既存値を維持
    },

    # より明るい (撮影用など): 桁を 1 つ上げる
    "bright": {
        "dome_intensity": 10000.0,
        "ceiling_intensity": 6e5,          # default の 4 倍
        "ceiling_count": 5,                # 四隅 + 中央
        "ceiling_color_temp": 6500.0,
        "default_lights_intensity": 2e5,   # 既存も 4 倍
    },

    # スタジオ級 (オーバーキル気味、ハッキリ明るい)
    "studio": {
        "dome_intensity": 20000.0,
        "ceiling_intensity": 1.5e6,        # default の 10 倍
        "ceiling_count": 5,
        "ceiling_color_temp": 5600.0,      # デイライト
        "default_lights_intensity": 5e5,
    },

    # 薄暗い (夜の部屋イメージ): 桁を 1 つ下げる
    "dim": {
        "dome_intensity": 300.0,
        "ceiling_intensity": 1.5e4,        # default の 1/10
        "ceiling_count": 4,
        "ceiling_color_temp": 6500.0,
        "default_lights_intensity": 5e3,   # 既存も 1/10
    },

    # 電球色 (温かみのある雰囲気、明るさは default 同等)
    "warm": {
        "dome_intensity": 3000.0,
        "ceiling_intensity": 1.5e5,
        "ceiling_count": 4,
        "ceiling_color_temp": 3000.0,      # 電球色
        "default_lights_intensity": 5e4,
    },

    # 照明追加なし (既存ライトも消灯 = ほぼ真っ暗、デバッグ用)
    "off": {
        "enable_lighting": False,
        "default_lights_intensity": 0.0,
    },
}


# デフォルト名 (apply_lab_dressing で引数省略時に使われる)
DEFAULT_PRESET: str = "lab"
DEFAULT_LIGHTING: str = "default"


def list_presets() -> dict:
    """利用可能なプリセット一覧を返す (デバッグ・ドキュメント用)。"""
    return {
        "dressing": list(DRESSING_PRESETS.keys()),
        "lighting": list(LIGHTING_PRESETS.keys()),
    }
