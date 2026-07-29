#!/usr/bin/env python3
"""
arena_cameras: アリーナを4方向 (北/南/東/西) から見下ろす競技録画カメラ。

TidyUp 競技で「どこに何が入ったか」を人が目視確認できるように、
部屋の外側の高い位置に固定カメラを 4 台置き、シミュレータ内部で
直接 mp4 に録画する (ROS トピックは使わない)。

使い方:
    make up TIME=600     # 競技時間 600 秒 (シミュレータ内時間)。
                         # 時間が来ると動画を保存してシミュレータが自動終了する。
    make up              # TIME 無し = カメラも録画も無し (通常の開発モード)。

出力:
    recordings/YYYYMMDD_HHMMSS/arena.mp4
    (コンテナ内 /recordings がホストのリポジトリ内 recordings/ に bind mount)

    4 画面を 2x2 に並べた 1 本の動画:
        ┌─────────┬─────────┐
        │  NORTH  │  EAST   │
        ├─────────┼─────────┤
        │  SOUTH  │  WEST   │
        └─────────┴─────────┘
    各画面の左下にカメラ名、全体の左上に「TASK 経過時間 / 競技時間」を焼き込む。

動画は 1/FPS シミュレータ秒 = 1 フレームで書き出すので、描画が実時間より
遅くても、動画の再生時間 = シミュレータ内の競技時間になる。

設定 (configs/placement.yaml の arena_cameras: セクション、無ければ既定値):
    arena_cameras:
      distance: 6.5        # 部屋の中心からの水平距離 (m)
      height: 4.5          # カメラの高さ (m)
      resolution: [640, 480]
      fps: 20
"""
from __future__ import annotations

import datetime
import os
from typing import Any, Dict

import cv2
import numpy as np
import yaml

import omni.replicator.core as rep
import omni.usd
from omni.isaac.core.utils.prims import create_prim
from pxr import Gf, UsdGeom

DEFAULT_CONFIG_PATH = "/app/configs/placement.yaml"
CAMERA_NAMES = ("north", "south", "east", "west")


def log(message: str) -> None:
    print(f"[arena_cameras] {message}", flush=True)


def _load_config() -> Dict[str, Any]:
    """placement.yaml の arena_cameras: セクションを読む (無ければ空)。"""
    candidates = [
        os.environ.get("PLACEMENT_CONFIG", DEFAULT_CONFIG_PATH),
        os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "configs", "placement.yaml",
        ),
    ]
    for path in candidates:
        if os.path.exists(path):
            with open(path) as f:
                cfg = yaml.safe_load(f) or {}
            return cfg.get("arena_cameras") or {}
    return {}


def _make_camera_prim(path: str, eye, target) -> None:
    """指定位置から target を見るカメラ prim を作る。

    USD のカメラは「-Z 方向を向き +Y が上」の約束なので、
    LookAt 行列 (world→カメラ) の逆行列がそのままカメラの姿勢になる。
    """
    create_prim(path, "Camera")
    st = omni.usd.get_context().get_stage()
    prim = st.GetPrimAtPath(path)

    cam = UsdGeom.Camera(prim)
    cam.CreateFocalLengthAttr(15.0)          # 広角ぎみ (約70度) で部屋全体を収める
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, 100.0))

    view = Gf.Matrix4d()
    view.SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(view.GetInverse())


def _format_time(sec: float) -> str:
    sec = max(0, int(sec))
    return f"{sec // 60:02d}:{sec % 60:02d}"


class ArenaRecorder:
    """4方向カメラの録画係。

    setup() でカメラを作り、メインループから step(sim_time) を毎回呼ぶ。
    競技時間に達すると step() が True を返すので、呼び出し側は録画を
    close() してシミュレータを終了する。
    """

    def __init__(self, task_time_sec: float,
                 center_x: float = 0.0, center_y: float = 0.0,
                 out_root: str = "/recordings"):
        cfg = _load_config()
        self.task_time = float(task_time_sec)
        # 注視点: 既定は床の中心 (呼び出し側から渡される)。config の center:
        # [x, y] で特定の部屋に寄せられる (デモで pick&place の部屋を狙う用)。
        _c = cfg.get("center")
        if _c and len(_c) >= 2:
            self.center = (float(_c[0]), float(_c[1]))
        else:
            self.center = (center_x, center_y)
        self.out_root = out_root
        self.distance = float(cfg.get("distance", 6.5))
        self.height = float(cfg.get("height", 4.5))
        resolution = cfg.get("resolution") or [640, 480]
        self.width, self.height_px = int(resolution[0]), int(resolution[1])
        self.fps = float(cfg.get("fps", 20))

        self.annotators = {}   # name -> rgb annotator
        self.writer = None     # 2x2 合成した 1 本の動画の VideoWriter
        self.frame_count = 0
        self.out_dir = None
        self._t0 = None        # 競技開始のシミュレータ時刻 (最初の step で決まる)
        self._next_capture = 0.0

    # ------------------------------------------------------------
    def setup(self) -> None:
        cx, cy = self.center
        target = (cx, cy, 0.3)  # 部屋の中心の少し上を見る
        positions = {
            "north": (cx, cy + self.distance, self.height),
            "south": (cx, cy - self.distance, self.height),
            "east": (cx + self.distance, cy, self.height),
            "west": (cx - self.distance, cy, self.height),
        }
        for name, eye in positions.items():
            cam_path = f"/World/ArenaCameras/Camera_{name}"
            _make_camera_prim(cam_path, eye, target)
            render_product = rep.create.render_product(
                cam_path, (self.width, self.height_px))
            annot = rep.AnnotatorRegistry.get_annotator("rgb")
            annot.attach([render_product])
            self.annotators[name] = annot
            log(f"camera {name}: eye={eye}")

        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.out_dir = os.path.join(self.out_root, stamp)
        os.makedirs(self.out_dir, exist_ok=True)
        log(f"録画開始: 競技時間 {_format_time(self.task_time)} "
            f"-> {self.out_dir}/arena.mp4 (4画面を2x2に合成)")

    # ------------------------------------------------------------
    def step(self, sim_time: float) -> bool:
        """毎ループ呼ぶ。競技時間に達したら True を返す。"""
        if self._t0 is None:
            self._t0 = sim_time
        elapsed = sim_time - self._t0

        # 1/fps シミュレータ秒ごとに 1 フレーム取り込む
        if elapsed >= self._next_capture:
            self._capture(elapsed)
            self._next_capture += 1.0 / self.fps

        return elapsed >= self.task_time

    def _get_tile(self, name: str) -> np.ndarray:
        """1 カメラ分の画像 (BGR) を返す。まだ描画が無ければ黒画面。"""
        data = self.annotators[name].get_data()
        if data is None or getattr(data, "size", 0) == 0:
            img = np.zeros((self.height_px, self.width, 3), dtype=np.uint8)
        else:
            img = cv2.cvtColor(
                np.asarray(data)[:, :, :3], cv2.COLOR_RGB2BGR)
        # 各画面の左下にカメラ名 (黒フチ + 白文字)
        label = name.upper()
        pos = (10, self.height_px - 12)
        cv2.putText(img, label, pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, label, pos, cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2, cv2.LINE_AA)
        return img

    def _capture(self, elapsed: float) -> None:
        # 4 画面を 2x2 に合成: 上段 north|east, 下段 south|west
        top = np.hstack([self._get_tile("north"), self._get_tile("east")])
        bottom = np.hstack([self._get_tile("south"), self._get_tile("west")])
        grid = np.vstack([top, bottom])

        # 全体の左上にタスク時間 (黒フチ + 白文字で読みやすく)
        label = f"TASK {_format_time(elapsed)} / {_format_time(self.task_time)}"
        cv2.putText(grid, label, (10, 38), cv2.FONT_HERSHEY_SIMPLEX,
                    1.1, (0, 0, 0), 5, cv2.LINE_AA)
        cv2.putText(grid, label, (10, 38), cv2.FONT_HERSHEY_SIMPLEX,
                    1.1, (255, 255, 255), 2, cv2.LINE_AA)

        if self.writer is None:
            path = os.path.join(self.out_dir, "arena.mp4")
            self.writer = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc(*"mp4v"),
                self.fps, (grid.shape[1], grid.shape[0]))
        self.writer.write(grid)
        self.frame_count += 1

    # ------------------------------------------------------------
    def close(self) -> None:
        if self.writer is not None:
            self.writer.release()
            log(f"saved arena.mp4 ({self.frame_count} frames, "
                f"{self.width * 2}x{self.height_px * 2})")
        else:
            log("WARNING: フレームが 1 枚も取れませんでした")
        log(f"録画完了: {self.out_dir}")
