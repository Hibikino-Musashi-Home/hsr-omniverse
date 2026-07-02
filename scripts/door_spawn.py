#!/usr/bin/env python3
# Copyright (c) 2025
# All rights reserved.
"""
door_spawn: placement.yaml の doors: セクションに従って「ドアノブ付きのドア」を配置する。

なぜ作るか:
  ロボカップの部屋にはドアが無いが、実機タスクでは「ドアノブを掴んでドアを開ける」
  動作を試したい。そこでシミュ上に、物理でちゃんと開閉するドアと、掴んで回せる
  ドアノブを置けるようにする。人 (people_spawn.py) や家具 (furniture_spawn.py) と
  同じ「専用モジュール + placement.yaml のセクション」方式にそろえてある。

作りかた (物理の中身):
  部品は 3 つ。すべて PhysX の剛体/関節でつなぐ (メッシュを手で組むだけなので、
  外部 USD ファイルは不要 = オフラインでも必ず出る)。
    - frame  (枠)   : 動かない柱 2 本 + 上の梁。キネマティック剛体(質量∞で動かないが、
                      ヒンジ関節の相手として確実に機能する正式な剛体)。
    - panel  (ドア板): 剛体。frame に「縦軸まわりのヒンジ(回転関節)」で付き、実際に開閉する。
    - knob   (レバーハンドル): 剛体。軸受け(スピンドル)+横棒(レバー) を表裏 (板の両面)
                      それぞれに持ち、1 本のスピンドルが板を貫通してつながる実物と
                      同じ構造 (= 1 つの剛体。片面を回すと両面いっしょに回る)。panel に
                      「ドア法線まわりの回転関節」で付き、レバーを押し下げると回る。
  ヒンジ軸は鉛直 (Z) なので重力ではドアは勝手に動かず、押した/引いたときだけ開く。
  ハンドル軸は水平 (ドアの法線方向)。レバーは横棒なので重力で垂れないよう「戻りバネ」を
  付けてあり、離すと水平に戻る (実物のレバーハンドルと同じ挙動)。
  どちら側からでもレバーを押し下げてドアを開けられる (両面ハンドル)。
  ※ ラッチ(掛け金)は DoorLatchController が再現する。ノブを回していないときはヒンジの
    可動範囲を(0,0)に施錠して「押しても引いても開かない」。ノブを THRESH 以上回すと解錠され、
    「握ったまま押す/引く」とその力で扉が開く(実物のドアと同じ操作)。扉が閉位置に戻ると
    カチッと再施錠。launch_isaacsim.py がメインループで step() を毎フレーム呼ぶ。
  ※ ドアは「一方向にだけ開く片開きドア」。ヒンジの可動範囲を片側だけに制限してあるので、
    片側からは押して開く専用・反対側からは引いて開く専用になる (実の室内ドアと同じ)。
  ※ 自動で閉じるバネ(ドアクローザー)は付けていない。ヒンジ軸は鉛直なので勝手には閉じず、
    開けたら開いたまま止まる。閉じるのはロボットが押し込んだときだけで、そのとき閉位置
    (0°=戸口の真ん中) の「硬いリミット」でピタッと止まって閉まる (反対側へ突き抜けない)。
    ※リミットは UsdPhysics の lower/upper だけ(硬い制約)。PhysxLimitAPI の stiffness は
      入れない(入れると柔らかいばねに化けて押し込むと突き抜けるため。_harden_limit 参照)。

配置のしかた (configs/tasks/<task>/placement.yaml):
  doors:
    - name: entrance_door     # シーン内の名前 (重複しても自動で連番にする)
      x: 0.4                  # 出入り口の位置 (world 座標 = map 座標, m)
      y: 2.5
      yaw: 0                  # 向き (度)。0 のときドア面は Y 方向に伸び、法線は X。
      #                         壁が Y 方向に走る所 (西/東の壁) は yaw:0。
      #                         壁が X 方向に走る所 (南/北の壁) は yaw:90。
      # width: 0.8            # ドア板の幅 (m, 省略時 0.8)
      # height: 2.0           # ドア板の高さ (m, 省略時 2.0)
      # thickness: 0.04       # ドア板の厚み (m, 省略時 0.04)
      # panel_mass: 6.0       # ドア板の重さ (kg, 省略時 6.0)。軽いほど押して開けやすい。
      # knob_mass: 0.2        # レバーハンドルの重さ (kg, 省略時 0.2)
      # open_limit: 120       # ドアが開ける最大角度 (度, 省略時 120)
      # hinge_side: left      # ちょうつがい(ヒンジ)を付ける側。left=-Y側 / right=+Y側 (省略時 left)

公開:
  - spawn_doors(assets_root, kit, config_path=None, initial_states=None): ドアを配置し数を返す。
    initial_states に list を渡すと、reset_world 用にドア板/ノブの初期姿勢を追記する。
"""
from __future__ import annotations

import math
import os
from typing import Any, Dict, List, Optional

import yaml

import omni.usd
from omni.physx.scripts import utils as physx_utils
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics


DEFAULT_CONFIG_PATH: str = "/app/configs/placement.yaml"
CONFIG_PATH: str = os.environ.get("DOOR_CONFIG", DEFAULT_CONFIG_PATH)

# ドアをまとめて置く親 prim (GUI で見やすいようまとめる。/World/People と同じ思想)。
DOOR_ROOT = "/World/Doors"

# 既定パラメータ (m, kg, 度)。placement.yaml で 1 つずつ上書きできる。
_DEF = {
    "width": 0.8,        # ドア板の幅
    "height": 2.0,       # ドア板の高さ
    "thickness": 0.04,   # ドア板の厚み
    "gap": 0.01,         # ドア板と柱のすき間 (擦れ防止)
    "post_w": 0.06,      # 柱・梁の断面
    "post_depth": 0.12,  # 柱・梁の奥行き (法線方向)
    # ハンドルは「レバーハンドル」(丸ノブではなく、横棒を押し下げて開ける形式)。
    "stem_r": 0.011,       # 軸受け(スピンドル)の半径
    # 軸受けの出っぱり長さ (板面から前へ)。これが長いほどレバーとドア面の隙間が広がり、
    # グリッパの指が裏に回り込んで掴みやすくなる。0.045 では隙間 ~3.4cm で狭く「もちにくい」
    # → 0.10 に伸ばして隙間を ~9cm 確保する。
    "stem_len": 0.10,
    "lever_len": 0.13,     # レバー(横棒)の長さ。少し長くして掴みしろを増やす
    "lever_thick": 0.022,  # レバーの厚み (前後=X)
    "lever_h": 0.024,      # レバーの高さ (上下=Z)
    "handle_inset": 0.16,  # ラッチ側の端からハンドル軸までの距離
    "handle_height": 1.0,  # ハンドル軸の高さ (床から)
    # レバーは軸から横に出るので放っておくと重力で垂れる。実物と同じく水平へ戻る
    # 「戻りバネ」を関節に付ける (ロボットが押し下げると回り、離すと水平に戻る)。
    # ゆるめ(5.0)にして軽い力でひねれるようにする(大きいほど固くて回しにくい)。
    "spring_stiffness": 5.0,   # 戻りバネの強さ (大きいほど固い)
    "spring_damping": 1.0,     # 戻りバネの減衰
    # ドア板の重さ (kg)。重いほど「慣性」が増え、押すのをやめても勢いで動き続ける
    # (軽すぎると押している間しか動かない)。台車の駆動力は 100 N·m に制限済み(hsr.py)
    # なので吹き飛ばない。12 はまだ重い体感だったので 9 に軽減。
    # placement.yaml の doors: に panel_mass: を書けば上書きできる(再ビルド不要)。
    "panel_mass": 9.0,
    "knob_mass": 0.2,      # ハンドル(レバー)の重さ
    "open_limit": 120.0,
    # ラッチばね(ポップ): ノブを回して解錠された瞬間、ばねが扉を数度「カチャッ」と
    # 押し開ける(実物のラッチばね/戸当たりパッキンの反発の再現)。
    # ※実行時に Drive のゲインを変えても PhysX に反映されない(ヘッドレステストで実証)ため、
    #   ばねは起動時から有効にしておき、施錠中は硬いリミット(0,0)が押さえ込む方式にする。
    # ばねの目標角。ポップの初動トルクは stiffness×target で決まるので、target を
    # 大きくすると「閉位置から押し出す力」が強くなり(ロボットの手が扉の前にあっても
    # 押しのけて開ける)、逆に大きく開いたときの引き戻し(=重さ)は軽くなる。
    # 18° ではロボットの手に負けて開けなかった(実ログ opened=0.4°)ため 30° に増強。
    "pop_target_deg": 30.0,
    # ばね強さ。強いほどポップも引き戻し(開くほどの重さ)も強くなる。
    # 押す重さは target を上げる方が有利なので、ここは 5.0 のままにする。
    "pop_stiffness": 5.0,
    # ばね減衰。大きいほど「開きだす勢い」が滑らかに抑えられる(押し出す力=stiffness×target
    # は変わらないので、ロボットの手をどかす力は保ったまま初速だけ緩む)。
    # 質量を 12→9 に軽くした分だけ勢いが増すため、0.5→1.5 に上げて少し緩める。
    "pop_damping": 1.5,
    # ヒンジ摩擦 (N·m)。実ドアの蝶番の渋さ。惰性で滑った扉を「少し動いて止まる」ようにし、
    # ばねで開いた扉もその場に保持する。大きいほど常時の抵抗=「重さ」が増す。
    # 0.6 もまだ重い体感だったので 0.45 に軽減 (ハンドル位置換算 ~1.2N)。
    "hinge_friction": 0.45,
}

# 見た目の色 (RGB 0..1)。物理には関係ないが区別しやすいように付ける。
# RoboCup 会場の白いドアに合わせて白系。レバーは金属(サテンニッケル)っぽいグレー。
_COL_FRAME = (0.96, 0.96, 0.96)   # 枠 = 白
_COL_PANEL = (0.93, 0.93, 0.93)   # 板 = 白 (ややグレー寄りで陰影が見える)
_COL_KNOB = (0.20, 0.21, 0.23)    # レバー = 濃いガンメタル (白いドアと明度差をつけ視認性UP)


def log(message: str) -> None:
    print(f"[door] {message}")


# ============================================================
# YAML 読み込み (furniture_spawn.py と同じフォールバック)
# ============================================================
def load_config(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        repo_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        fallback = os.path.join(repo_root, "configs", "placement.yaml")
        if os.path.exists(fallback):
            path = fallback
        else:
            log(f"WARNING: config not found: {path} (nor {fallback})")
            return {}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return data


# ============================================================
# 幾何プリミティブ (メッシュを手で組む。外部 USD 不要)
# ============================================================
def _box_mesh(stage: Usd.Stage, path: str, dx: float, dy: float, dz: float,
              center, color) -> Usd.Prim:
    """原点中心・実寸 (m) の直方体メッシュを作る。スケール op は使わない。

    剛体+関節にスケールが混ざると、関節の取り付け位置(localPos)の解釈が曖昧になり
    バグの温床になる。そこで頂点そのものを実寸で置き、スケールは一切かけない。
    """
    hx, hy, hz = dx / 2.0, dy / 2.0, dz / 2.0
    mesh = UsdGeom.Mesh.Define(stage, path)
    pts = [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
           (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in pts])
    # 面 (外向き CCW)。collider は凸包を使うので厳密な向きは不要だが、見た目のため。
    faces = [0, 3, 2, 1,   4, 5, 6, 7,   0, 1, 5, 4,
             2, 3, 7, 6,   1, 2, 6, 5,   0, 4, 7, 3]
    mesh.CreateFaceVertexCountsAttr([4, 4, 4, 4, 4, 4])
    mesh.CreateFaceVertexIndicesAttr(faces)
    mesh.CreateExtentAttr([Gf.Vec3f(-hx, -hy, -hz), Gf.Vec3f(hx, hy, hz)])
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    # 原点からの位置 (組み立てローカル座標)。回転は無し。
    xf = UsdGeom.Xformable(mesh)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    return mesh.GetPrim()


def _set_mass(prim: Usd.Prim, mass_kg: float) -> None:
    """剛体 prim に質量を入れる (未指定だと PhysX が体積から推定してブレるため)。"""
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr(float(mass_kg))


def _set_damping(prim: Usd.Prim, ang: float, lin: float = 0.0) -> None:
    """剛体に減衰を入れて、押した後だらだら揺れ続けないようにする。"""
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateAngularDampingAttr(float(ang))
    rb.CreateLinearDampingAttr(float(lin))


def _harden_dynamic_body(prim: Usd.Prim) -> None:
    """高速な衝突で物理が破綻しない(貫通→めり込み補正が暴走→吹き飛ぶ)よう安全策を入れる。

    ドア板・ハンドルは薄い/軽いので、ロボットが勢いよくぶつかると 1 ステップの間に
    すり抜け(トンネリング)て深くめり込み、それを 1 ステップで戻そうとする補正力が
    過大になって速度が発散する ("物理が崩壊する") ことがある。
      - CCD (連続衝突判定) を有効にしてすり抜け自体を防ぐ。※ただしシーン側の CCD も
        launch_isaacsim.py で有効化して初めて効く (片方だけでは PhysX に無視される)。
      - めり込み解消の速度に上限を付け、一気に弾き飛ばされないようにする。薄板ではこの
        排出速度がヒンジで「回転」に化けてドアが暴れて開くので、小さめ(1.0)に絞る。
      - ソルバの反復回数を増やし、関節(ヒンジ/ハンドル)がつながった状態でも
        衝突の力を安定して解けるようにする。
    """
    rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateEnableCCDAttr(True)
    rb.CreateMaxDepenetrationVelocityAttr(1.0)
    rb.CreateSolverPositionIterationCountAttr(16)
    # 速度反復は 4 まで。新しい TGS ソルバでは 4 超で挙動が変わり警告が出るため。
    rb.CreateSolverVelocityIterationCountAttr(4)
    # スリープ(省電力の凍結)を無効にする。眠った剛体は関節ドライブ(ラッチばね)では
    # 起こせず、ばねが効き始めた直後に凍結して「扉が全く開かない」実測不具合になった
    # (ヘッドレステストで -0.41° で静止を確認)。ドアは常時アクティブでも計算負荷は微小。
    rb.CreateSleepThresholdAttr(0.0)


def _harden_limit(joint) -> None:
    """回転関節の可動範囲(リミット)を「硬い壁」にし、跳ね返りだけ消す。

    重要: UsdPhysics の lower/upper リミットだけなら、PhysX はそれを「絶対に越えられ
    ない硬い不等式制約」として解く。ところが PhysxLimitAPI に stiffness>0 を入れると、
    同じリミットが「柔らかいばね」に化けて “越えられる” ようになる (復元トルク =
    stiffness × 越えた量)。すると押し続ける力(台車の駆動)でドアが閉位置(0°)を突き抜けて
    反対側へ開いてしまう(実際に起きていた B2 不具合)。
    そこで stiffness/damping は入れず、反発(restitution)を 0 にするだけにする。これで
    リミットは硬いまま(押しても越えられない)で、突き当たりの跳ね返りだけが消える。
    """
    limit_api = PhysxSchema.PhysxLimitAPI.Apply(joint.GetPrim(), "angular")
    limit_api.CreateRestitutionAttr(0.0)


def _revolute(stage: Usd.Stage, path: str, body0: Optional[str], body1: str,
              axis, local_pos0, local_pos1,
              lower: Optional[float] = None,
              upper: Optional[float] = None):
    """回転関節を 1 つ作り、その UsdPhysics.RevoluteJoint を返す。

    body0=None のとき localPos0 はワールド座標になる。ここでは body0 に「枠」や「板」
    を渡し、それぞれのローカル座標で軸位置を指定する (組み立てをどこへ置いても不変)。
    lower/upper (度) を渡すと可動範囲を制限する。渡さなければ自由回転。
    """
    j = UsdPhysics.RevoluteJoint.Define(stage, path)
    if body0 is not None:
        j.CreateBody0Rel().SetTargets([body0])
    j.CreateBody1Rel().SetTargets([body1])
    j.CreateAxisAttr(axis)
    j.CreateLocalPos0Attr().Set(Gf.Vec3f(*local_pos0))
    j.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
    j.CreateLocalPos1Attr().Set(Gf.Vec3f(*local_pos1))
    j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    if lower is not None and upper is not None:
        j.CreateLowerLimitAttr(float(lower))
        j.CreateUpperLimitAttr(float(upper))
    return j


def _add_return_spring(joint, stiffness: float, damping: float,
                       target_deg: float = 0.0) -> None:
    """回転関節に「戻りバネ」(角度ドライブ)を付ける。

    レバーハンドルは軸から横棒が出ているため、放っておくと重力で垂れ下がる。
    実物のハンドルと同じく、目標角(既定0度=水平)へ戻ろうとするバネを入れる。
    ロボットが押し下げると回り、離すと水平へ戻る。stiffness を 0 にすると自由回転。
    """
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(float(target_deg))
    drive.CreateStiffnessAttr(float(stiffness))
    drive.CreateDampingAttr(float(damping))


def _author_hinge_joint(stage: Usd.Stage, cfg: Dict[str, Any],
                        with_spring: bool, locked: bool,
                        enabled: bool = True) -> None:
    """ヒンジ関節を作成する (スポーン時のみ。実行時の作り直しはしない)。

    ドアは「ばね有(A)」「ばね無(B)」の 2 本のヒンジをスポーン時に作っておき、
    実行時は physics:jointEnabled のホット切り替えで片方だけを有効にする(実証済み)。
    ※関節プリムの実行時削除/再作成は PhysX のシーン再パースを誘発し、ロボットの
      制御ハンドル(物理ビュー)を無効化して腕が操作不能になる(実測)ため行わない。
      - with_spring=True : ラッチばね(ポップ)装填。解錠されるとカチャッと開く。
      - with_spring=False: ばね無し。扉は減衰以外フリーで「ちょんと押すだけで
        さらに開き、押した所に留まる」(ポップ完了後用)。
      - locked=True: 施錠(リミット0,0)。False: 全可動範囲。
      - enabled: physics:jointEnabled の初期値。
    """
    if locked:
        lo, hi = 0.0, 0.0
    elif cfg["open_dir"] > 0:
        lo, hi = 0.0, cfg["open_limit_deg"]
    else:
        lo, hi = -cfg["open_limit_deg"], 0.0
    j = _revolute(
        stage, cfg["joint_path"],
        body0=cfg["frame_path"], body1=cfg["panel_path"],
        axis=UsdPhysics.Tokens.z,
        local_pos0=(0.0, cfg["hinge_y"], cfg["panel_h"] / 2.0),  # 枠ローカル
        local_pos1=(0.0, cfg["hinge_y"], 0.0),                   # 板ローカル(中心は H/2)
        lower=lo, upper=hi,
    )
    # 全開/全閉の突き当たりは「硬い壁」(restitution=0)。閉位置 0° は押し込んでも
    # 反対側へ突き抜けない(柔らかいばねリミットは貫通する。実測)。
    _harden_limit(j)
    # ラッチばね(ポップ)。無効時はゲイン 0 で作る。
    drv = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular")
    drv.CreateTypeAttr("force")
    drv.CreateTargetPositionAttr(cfg["open_dir"] * cfg["pop_target_deg"])
    drv.CreateStiffnessAttr(float(cfg["pop_k"]) if with_spring else 0.0)
    drv.CreateDampingAttr(float(cfg["pop_c"]) if with_spring else 0.0)
    # ヒンジ摩擦(蝶番の渋さ)。※maximal関節では効きが当てにならない(実測)ため、
    # 「止まる」役は剛体側の角減衰が担う。ここは付けておくだけ(害はない)。
    PhysxSchema.PhysxJointAPI.Apply(
        j.GetPrim()).CreateJointFrictionAttr(float(cfg["friction"]))
    # 有効/無効の初期値 (実行時は physics:jointEnabled のホット切替で遷移する)
    UsdPhysics.Joint(j.GetPrim()).CreateJointEnabledAttr(bool(enabled))


# ============================================================
# 1 エントリの正規化
# ============================================================
def _parse_item(item: Any, idx: int) -> Optional[Dict[str, Any]]:
    if not isinstance(item, dict):
        log(f"WARNING: doors[{idx}] は辞書ではありません: {item!r}。スキップ。")
        return None
    out: Dict[str, Any] = {"name": str(item.get("name", f"door_{idx}"))}
    try:
        out["x"] = float(item.get("x", 0.0))
        out["y"] = float(item.get("y", 0.0))
        out["yaw_deg"] = float(item.get("yaw", 0.0))
        for k in ("width", "height", "thickness", "panel_mass", "knob_mass",
                  "open_limit"):
            out[k] = float(item.get(k, _DEF[k]))
    except (TypeError, ValueError):
        log(f'WARNING: door "{out["name"]}" の数値が読めません: {item!r}。'
            f"スキップ (小数点は '.' で書いてください)。")
        return None
    out["hinge_side"] = str(item.get("hinge_side", "left")).lower()
    return out


# ============================================================
# 1 枚ぶんの組み立て
# ============================================================
def _spawn_one(stage: Usd.Stage, item: Dict[str, Any],
               initial_states: Optional[List[Dict[str, Any]]],
               door_handles: Optional[List[Dict[str, Any]]] = None) -> bool:
    name = item["name"]
    root_path = f"{DOOR_ROOT}/{name}"

    W = item["width"]
    H = item["height"]
    T = item["thickness"]
    gap = _DEF["gap"]
    pw = _DEF["post_w"]
    pd = _DEF["post_depth"]

    # ヒンジ側の符号。left = -Y 側にヒンジ / right = +Y 側にヒンジ。
    hinge_sign = 1.0 if item["hinge_side"] == "right" else -1.0

    # 組み立てローカル座標系:
    #   原点 = 出入り口の床の中心。X = ドアが開く向き(法線), Y = 幅方向, Z = 上。
    #   ドア板は中央 (0,0,H/2) を中心に、幅 (W-2gap)・高さ H・厚み T。
    panel_w = W - 2.0 * gap

    # --- 親 Xform (ここに world への配置 = 平行移動 + Z回転 をかける) ---
    root = UsdGeom.Xform.Define(stage, root_path)
    rxf = UsdGeom.Xformable(root)
    rxf.ClearXformOpOrder()
    rxf.AddTranslateOp().Set(Gf.Vec3d(item["x"], item["y"], 0.0))
    rxf.AddRotateZOp().Set(item["yaw_deg"])

    # --- 枠 (frame): 柱 2 本 + 上の梁。「キネマティック剛体」にする ---
    # なぜ剛体にするか: ヒンジ(回転関節)の相手(body0)にするため。ただの Xform や
    # 静的 collider だけの prim を関節の相手にすると、PhysX が正しくアンカーできず
    # 「板が枠に固定されない/落ちる」ことがある。キネマティック剛体は力を受けても
    # 動かない(質量∞)ので、見た目は静的な枠のまま、関節の相手として確実に機能する。
    # 子メッシュに付けた collider は、この親剛体に 1 つの剛体としてまとめられる。
    frame_path = f"{root_path}/frame"
    frame_prim = UsdGeom.Xform.Define(stage, frame_path).GetPrim()
    y_post = W / 2.0 + gap + pw / 2.0            # 柱の中心 Y (両側)
    posts = [
        (f"{frame_path}/post_left", (0.0, -y_post, H / 2.0), (pd, pw, H)),
        (f"{frame_path}/post_right", (0.0, y_post, H / 2.0), (pd, pw, H)),
        (f"{frame_path}/lintel", (0.0, 0.0, H + pw / 2.0),
         (pd, W + 2.0 * (gap + pw), pw)),
    ]
    for p_path, center, dims in posts:
        p = _box_mesh(stage, p_path, dims[0], dims[1], dims[2], center, _COL_FRAME)
        physx_utils.setCollider(p, approximationShape="none")
    # 枠グループ全体をキネマティック剛体にする (kinematic=True で動かない剛体になる)。
    frame_rb = UsdPhysics.RigidBodyAPI.Apply(frame_prim)
    frame_rb.CreateRigidBodyEnabledAttr(True)
    frame_rb.CreateKinematicEnabledAttr(True)

    # --- ドア板 (panel): 剛体。中心 (0,0,H/2) ---
    panel_path = f"{root_path}/panel"
    panel = _box_mesh(stage, panel_path, T, panel_w, H,
                      (0.0, 0.0, H / 2.0), _COL_PANEL)
    physx_utils.setRigidBody(panel, "convexHull", False)
    _set_mass(panel, item["panel_mass"])
    # 角減衰 = 「押すのをやめたら少し滑って止まる」役。ヒンジの jointFriction は
    # 実行時に作り直した関節では効かない(実測: 切断後に毎秒1.4°のクリープ)ため、
    # 止まる役はこの剛体側の減衰が担う(剛体は作り直さないので確実に効く)。
    # 0.35 → ちょんと押す(0.3rad/s)と ~50° 滑ってから止まる = 軽いが止まる。
    # (2.0 は押した瞬間に止まり「慣性なし」、0.05 は 20 秒も流れ続けた。実測)
    _set_damping(panel, ang=0.35, lin=0.0)
    _harden_dynamic_body(panel)

    # ヒンジ (frame ↔ panel): 鉛直軸 Z。ヒンジ側の端 (y = hinge_sign*(W/2-gap)) に置く。
    hinge_y = hinge_sign * (W / 2.0 - gap)
    #   下限/上限: ドアは「ハンドル側の面 (+X, 出入り口に近づいて来る側)」から
    #   押して開く向きに開く。ヒンジ支点でのトルクを計算すると
    #   (力=+X方向, 腕=ラッチ端-ヒンジ端の相対Y) τ_z = -(latch_y - hinge_y) * Fx
    #   となり、hinge_side="left" (hinge_sign<0, latch は +Y 側) では τ_z が負
    #   → 開角は負の側 (lo=-open_limit, hi=0)。hinge_side="right" はその逆。
    # スポーン時は「施錠状態(リミット0,0)+ばね装填済み」で作る。以降の状態遷移
    # (解錠/ばね切離し/再施錠) は DoorLatchController が行う。
    hinge_cfg = {
        "joint_path": f"{root_path}/hinge_joint",       # A = ばね有 (施錠/ポップ担当)
        "joint_path_b": f"{root_path}/hinge_joint_b",   # B = ばね無 (ポップ後のフリー担当)
        "frame_path": frame_path,
        "panel_path": panel_path,
        "hinge_y": float(hinge_y),
        "panel_h": float(H),
        "open_dir": float(hinge_sign),
        "open_limit_deg": float(item["open_limit"]),
        "pop_target_deg": float(_DEF["pop_target_deg"]),
        "pop_k": float(_DEF["pop_stiffness"]),
        "pop_c": float(_DEF["pop_damping"]),
        "friction": float(_DEF["hinge_friction"]),
    }
    # A: ばね装填+施錠+有効 で開始。B: ばね無し+全範囲+無効 で待機。
    # 実行時は jointEnabled の切替だけで A↔B を遷移する(削除/再作成はしない)。
    _author_hinge_joint(stage, hinge_cfg, with_spring=True, locked=True, enabled=True)
    _cfg_b = dict(hinge_cfg)
    _cfg_b["joint_path"] = hinge_cfg["joint_path_b"]
    _author_hinge_joint(stage, _cfg_b, with_spring=False, locked=False, enabled=False)

    # --- ハンドル = レバーハンドル (knob): 剛体。ラッチ側 (ヒンジと反対側) の板面に付く ---
    # 丸ノブではなく、軸受け(スピンドル)+横棒(レバー) の形。押し下げて開ける実物と同じ。
    # 実物のドアハンドルと同じく、1 本のスピンドルが板を貫通して表裏両面のレバーを
    # つないでいる構造にする (= 表裏あわせて 1 つの剛体。片側を回すと両方一緒に回る)。
    # これでロボットがどちら側からでもレバーを押し下げて開けられる。
    latch_sign = -hinge_sign                  # ハンドルはヒンジと反対の端
    handle_y = latch_sign * (W / 2.0 - gap - _DEF["handle_inset"])  # 軸の Y 位置
    handle_z = _DEF["handle_height"]                                # 軸の高さ
    face_x = T / 2.0                          # ドアの前面 (+X 側)。裏面は -face_x。
    stem_len = _DEF["stem_len"]
    lever_len = _DEF["lever_len"]
    # レバーはヒンジ側 (ドアの中央寄り) へ向けて伸ばす。実物のレバーハンドルは
    # 握り手が自由端の外へ飛び出さず、ヒンジ側 (板の中央方向) へ弧を描くように
    # 付いている (そうしないとドアを閉めたとき枠にぶつかる)。以前は逆(自由端側)に
    # 伸びており「取り付け向きが逆」に見えていた。
    lever_dir = hinge_sign

    knob_path = f"{root_path}/knob"
    knob_prim = UsdGeom.Xform.Define(stage, knob_path).GetPrim()
    # 表(+X)・裏(-X) それぞれに 軸受け(スピンドル)+レバー を作る (符号だけ反転)。
    # スピンドルは箱メッシュで作る(円柱プリミティブはコライダ生成の軸解釈が曖昧で、
    # 見えない縦向きの当たり判定ができ「何もない所に引っかかる」原因になり得るため。
    # メッシュなら見た目とコライダが正確に一致する)。
    for side_name, side_sign in (("front", 1.0), ("back", -1.0)):
        stem_center = (side_sign * (face_x + stem_len / 2.0), handle_y, handle_z)
        stem = _box_mesh(stage, f"{knob_path}/stem_{side_name}",
                         stem_len, 2.0 * _DEF["stem_r"], 2.0 * _DEF["stem_r"],
                         stem_center, _COL_KNOB)
        physx_utils.setCollider(stem, approximationShape="convexHull")
        # レバー(横棒): 軸の先端から水平(Y)に伸びる棒。押し下げると法線(X)まわりに回る。
        lever_center = (side_sign * (face_x + stem_len),
                        handle_y + lever_dir * lever_len / 2.0,
                        handle_z)
        lever = _box_mesh(stage, f"{knob_path}/lever_{side_name}",
                          _DEF["lever_thick"], lever_len, _DEF["lever_h"],
                          lever_center, _COL_KNOB)
        physx_utils.setCollider(lever, approximationShape="convexHull")
    # 表裏 4 部品 (stem x2 + lever x2) をまとめて 1 つの剛体にする
    # (子 collider が親剛体にまとまる。回転軸はドア厚みの中心 X=0 を通る)。
    knob_rb = UsdPhysics.RigidBodyAPI.Apply(knob_prim)
    knob_rb.CreateRigidBodyEnabledAttr(True)
    _set_mass(knob_prim, item["knob_mass"])
    _set_damping(knob_prim, ang=0.5, lin=0.5)
    _harden_dynamic_body(knob_prim)

    # ハンドル関節 (panel ↔ handle): ドア法線 X まわりに回転。回転軸はドア厚みの中心
    # (X=0) を通る一本のスピンドルなので、関節のアンカーもそこに置く。
    # レバーは重力で垂れるので「戻りバネ」で水平へ戻す (実物のハンドルと同じ)。
    knob_joint = _revolute(
        stage, f"{root_path}/knob_joint",
        body0=panel_path, body1=knob_path,
        axis=UsdPhysics.Tokens.x,
        local_pos0=(0.0, handle_y, handle_z - H / 2.0),  # 板中心 (0,0,H/2) からの相対
        local_pos1=(0.0, handle_y, handle_z),            # ハンドル群ローカル(=組み立て座標)
        lower=None, upper=None,
    )
    _add_return_spring(knob_joint, _DEF["spring_stiffness"], _DEF["spring_damping"])

    # --- reset_world 用: 板とノブの初期ワールド姿勢を記録 (drop_object と同じ方式) ---
    if initial_states is not None:
        for body_path in (panel_path, knob_path):
            try:
                bp = stage.GetPrimAtPath(body_path)
                m = UsdGeom.Xformable(bp).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default())
                t = m.ExtractTranslation()
                q = m.GetOrthonormalized().ExtractRotationQuat()
                qi = q.GetImaginary()
                initial_states.append({
                    "body_path": body_path,
                    "p": (t[0], t[1], t[2]),
                    "q": (q.GetReal(), qi[0], qi[1], qi[2]),  # (w, x, y, z)
                })
            except Exception as ex:
                log(f'NOTE: door "{name}" の {body_path} の初期姿勢記録に失敗: {ex!r}')

    # ラッチ連動コントローラ用の記述子を渡す(「ノブを回すとラッチが外れて開けられる」に使う)。
    # hinge_cfg はコントローラが関節を作り直す(状態遷移)ときにそのまま使う。
    if door_handles is not None:
        door_handles.append({
            "name": name,
            "panel_path": panel_path,
            "knob_path": knob_path,
            "hinge_cfg": hinge_cfg,
        })

    log(f'spawned "{name}" at ({item["x"]:.2f}, {item["y"]:.2f}) '
        f'yaw={item["yaw_deg"]:.0f}deg W={W:.2f} H={H:.2f} '
        f'hinge={item["hinge_side"]}')
    return True


# ============================================================
# 公開関数
# ============================================================
def spawn_doors(assets_root: str, kit: Any,
                config_path: str | None = None,
                initial_states: Optional[List[Dict[str, Any]]] = None,
                door_handles: Optional[List[Dict[str, Any]]] = None) -> int:
    """placement.yaml の doors: セクションに従ってドアを配置する。

    戻り値は配置できたドアの数。doors: が無い/空のときは 0 (何も置かない) なので、
    doors: を書いていないタスクでも安全に no-op になる (launch から無条件に呼んでよい)。
    assets_root は他ローダーと引数をそろえるためだけに受け取る (この関数では未使用)。
    """
    path = config_path or CONFIG_PATH
    cfg = load_config(path)

    section = cfg.get("doors")
    if isinstance(section, dict):
        items = section.get("list") or []
    else:
        items = section or []

    if not items:
        log("doors が空です。ドアは配置しません (既存タスクに影響なし)。")
        return 0

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(DOOR_ROOT).IsValid():
        UsdGeom.Xform.Define(stage, DOOR_ROOT)

    placed = 0
    used_names: set = set()
    for idx, raw in enumerate(items):
        parsed = _parse_item(raw, idx)
        if parsed is None:
            continue
        base = parsed["name"]
        uniq = base
        n = 1
        while uniq in used_names:
            uniq = f"{base}_{n}"
            n += 1
        parsed["name"] = uniq
        used_names.add(uniq)

        # 1 枚の失敗で起動全体を巻き込まないよう、ここで受け止めて次へ進む。
        try:
            if _spawn_one(stage, parsed, initial_states, door_handles):
                placed += 1
                kit.update()
        except Exception as ex:
            log(f'WARNING: door "{parsed["name"]}" の配置中に例外: {ex!r}。スキップして続行します。')

    log(f"placed {placed}/{len(items)} doors from {path}")
    return placed


# ============================================================
# ラッチ(掛け金): 「ノブを回すと開けられる」実物のドアの仕組み
# ============================================================
# 実物のドアはこう動く:
#   1. ノブを回していないとき、ラッチ(掛け金)が枠に掛かっていて、押しても引いても開かない。
#   2. ノブを回すとラッチが引っ込み、ヒンジが自由になる。
#   3. 人はノブを「握ったまま」押す/引く → その力で扉がついてくる(勝手には開かない)。
#   4. 扉が閉位置に戻るとカチッと再ラッチ。
# これをそのまま再現する。実装は「ヒンジ関節の可動範囲(硬いリミット)を実行時に
# 施錠(0,0)/解錠(全範囲) で切り替える」だけ。omni.physx は実行中の関節リミット変更を
# ライブ反映できる(GUI のプロパティ変更がシミュ中に効くのと同じ経路)。
#
# 安全設計: 速度・姿勢・力は一切書かない(以前試した角速度書き込みはヒンジ拘束のソルバに
# 消されて動かず、テレポートはロボットの手にめり込んで吹き飛んだ)。リミットの付け替えは
# 拘束の定義を変えるだけでエネルギーを注入しないので、ロボットが触れていても壊れない。
def _quat_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def _quat_mul(a, b):
    """クォータニオン (x,y,z,w) のハミルトン積 (hsr.py の _q_mul と同じ順序)。"""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def _rel_angle_about(qA, qB, axis):
    """qA を基準にした qB の、A ローカル軸まわりの符号つき最短角 (rad, -pi..pi)。"""
    x, y, z, w = _quat_mul(_quat_conj(qA), qB)
    if w < 0.0:                       # w>=0 半球にそろえて (-pi, pi] に収める
        x, y, z, w = -x, -y, -z, -w
    s = x if axis == 'x' else (y if axis == 'y' else z)
    return 2.0 * math.atan2(s, w)


class DoorLatchController:
    """実物のドアのラッチ(掛け金)を再現するコントローラ。

    - レバー中立: 施錠(ヒンジのリミットを 0,0 に固定) → 押しても引いても開かない。
    - レバーを THRESH 以上ひねっている間: 解錠(リミットを全範囲に戻す) → ロボットは
      ノブを「握ったまま」押す/引くだけで、その力で扉が自然に開く(実世界と同じ)。
    - レバーが戻り、かつ扉が閉位置(CLOSE_EPS 以内)のとき: カチッと再施錠。
      (開いたままレバーを離しても施錠されない = 実物と同じ。閉めた瞬間に掛かる)

    使い方: spawn_doors(..., door_handles=lst) で得た記述子 lst を渡して生成し、
    メインループで毎フレーム step() を呼ぶ。reset の直後に notify_reset() を呼ぶ。
    """
    THRESH = math.radians(10.0)    # 解錠に必要なレバー角(レバーの揺れより上)
    RELATCH = math.radians(5.0)    # 再施錠にはこれ未満まで戻す(ヒステリシス)
    CLOSE_EPS = math.radians(1.5)  # 「閉まっている」とみなす扉角(施錠でここから0へ引き込む)
    # ポップ終了(=ばね切離し)の判定。角度ではなく「静止」で切る:
    # ばね自身の減衰が目標角付近で扉を自然に減速させるので、動きが止まりかけた
    # 瞬間に切れば惰性がほぼ残らず「少し開いて止まる」になる。
    # (加速中に角度で切ると、勢いが残ったまま完全フリーになり全開まで滑走する。実測)
    SETTLE_EPS = 0.00015           # 「静止」とみなす1フレームの角度変化 (rad ≈ 0.5°/s)
    SETTLE_MIN_AGE = 30            # 解錠直後の「まだ動き出してない」を静止と誤認しない猶予
    # 「開き待ち」ガードの猶予(フレーム数)。ロボットがノブを「握っている間」は腕が扉を
    # 固定していてばねは押せないので、タイマーは進めない(握っている限り待つ)。
    # 離した瞬間からカウント。引く側(扉がロボットに向かって開く側)では、ロボットが
    # 後退して進路を空けるまで扉は開けないため、その時間の余裕を見て 10 秒にする
    # (4 秒では後退する前に切れて即再施錠され「開かない」ことがあった。実ログ)。
    POP_TIMEOUT = 600
    # 再施錠のデバウンス(フレーム数)。「レバー戻り+扉閉」の条件がこの時間連続して
    # 成立したときだけ施錠する(一瞬の成立でガチャッと施錠されるのを防ぐ)。
    RELATCH_HOLD = 60
    # 診断ログ。既定 OFF。環境変数 DOOR_LATCH_DEBUG=1 のときだけ 0.5 秒ごとに
    # レバー角/扉角/施錠状態を出す(常時ONにはしない)。有効化例:
    #   DOOR_LATCH_DEBUG=1 ROS_DOMAIN_ID=26 make ros2 up TASK=door_test
    DEBUG = bool(os.environ.get("DOOR_LATCH_DEBUG"))

    def __init__(self, dc, descriptors):
        self.dc = dc
        self._dbg = 0
        self.doors = [{
            "name": d["name"], "panel_path": d["panel_path"],
            "knob_path": d["knob_path"],
            "joint_path": d["hinge_cfg"]["joint_path"],      # A = ばね有
            "joint_path_b": d["hinge_cfg"]["joint_path_b"],  # B = ばね無
            "open_dir": float(d["hinge_cfg"]["open_dir"]),
            "lim_deg": float(d["hinge_cfg"]["open_limit_deg"]),
            "ph": None, "kh": None, "q_closed": None,
            # スポーン時に施錠(0,0)+ばね装填済みで作られている前提で locked=True から開始。
            "locked": True,
            "popping": False, "pop_left": 0,   # ラッチばね(ポップ)の作動状態
            "relatch_cnt": 0,                  # 再施錠デバウンスのカウンタ
        } for d in descriptors]

    # --- ヒンジリミットの付け替え(解錠の実体。実行時変更が効くと実証済み) ---
    def _set_limits(self, d, lo_deg, hi_deg):
        stage_ = omni.usd.get_context().get_stage()
        j = UsdPhysics.RevoluteJoint.Get(stage_, d["joint_path"])
        if not j:
            raise RuntimeError(f'hinge joint not found: {d["joint_path"]}')
        j.GetLowerLimitAttr().Set(float(lo_deg))
        j.GetUpperLimitAttr().Set(float(hi_deg))

    # --- ヒンジ A/B の有効・無効切り替え(施錠/ばね切離しの実体) ---
    # Drive ゲインの実行時書き換えは PhysX に反映されない(実測)。また関節プリムの
    # 実行時削除/再作成は PhysX の再パースを誘発し、ロボットの制御ハンドルを無効化して
    # 腕が操作不能になる(実測)。そこでスポーン時に「ばね有 A / ばね無 B」の 2 本を
    # 作っておき、physics:jointEnabled のホット切替(実証済み)だけで遷移する。
    def _set_joint_enabled(self, path, on):
        stage_ = omni.usd.get_context().get_stage()
        prim = stage_.GetPrimAtPath(path)
        attr = prim.GetAttribute("physics:jointEnabled")
        if not attr:
            attr = UsdPhysics.Joint(prim).CreateJointEnabledAttr(bool(on))
        attr.Set(bool(on))

    def _lock(self, d):
        # 施錠 = A(ばね装填済み)を施錠リミットにして有効化、B を無効化。
        # ばね(2N·m程度)は硬いリミットが押さえ込むので扉は開かない(実測 0.00°)。
        d["popping"] = False
        d["relatch_cnt"] = 0
        self._set_limits(d, 0.0, 0.0)                       # A を施錠リミットへ
        self._set_joint_enabled(d["joint_path"], True)      # A 有効 (ばね有)
        self._set_joint_enabled(d["joint_path_b"], False)   # B 無効
        d["locked"] = True

    def _unlock(self, d):
        # 解錠 = リミットを全範囲へ(ホット変更、実証済み)。施錠時に装填したばねが
        # その瞬間から扉を押し、減衰とヒンジ摩擦で pop 目標角にカチャッと開く(実測)。
        if d["open_dir"] > 0:               # right ヒンジ: 0 〜 +lim
            self._set_limits(d, 0.0, d["lim_deg"])
        else:                               # left ヒンジ: -lim 〜 0
            self._set_limits(d, -d["lim_deg"], 0.0)
        d["locked"] = False
        # popping = 「開き待ち」ガード。解錠直後は扉がまだ閉位置(0°)にあるため、この
        # ガードが無いと即座に再施錠されてしまう。ばねが開いて静止するか、
        # 塞がれてタイムアウトするまで再施錠を保留する。
        d["popping"] = True
        d["pop_left"] = self.POP_TIMEOUT
        d["pop_age"] = 0
        d["prev_opened"] = 0.0

    def notify_reset(self):
        # _reset_objects() が扉を閉位置へ戻した直後に呼ぶ。施錠状態に戻す。
        for d in self.doors:
            try:
                self._lock(d)
            except Exception as ex:
                print(f'[door] latch reset {d["name"]}: {ex!r}', flush=True)

    def step(self):
        self._dbg += 1
        for d in self.doors:
            try:
                self._one(d)
            except Exception as ex:
                # 持続的なエラーで毎フレーム(60Hz)ログが溢れないよう、
                # ドアごとに 5 秒(300フレーム)に 1 回だけ出す。
                if self._dbg - d.get("err_at", -999) >= 300:
                    d["err_at"] = self._dbg
                    print(f'[door] latch ctrl {d["name"]}: {ex!r}', flush=True)

    def _one(self, d):
        dc = self.dc
        if not d["ph"]:                    # ハンドルは遅延解決(無効な間は毎フレーム再試行)
            d["ph"] = dc.get_rigid_body(d["panel_path"])
        if not d["kh"]:
            d["kh"] = dc.get_rigid_body(d["knob_path"])
        if not d["ph"] or not d["kh"]:
            return
        # スポーン時に施錠済み(リミット0,0で作成)なので初回の施錠処理は不要。
        pp = dc.get_rigid_body_pose(d["ph"])
        kp = dc.get_rigid_body_pose(d["kh"])
        qp = (pp.r.x, pp.r.y, pp.r.z, pp.r.w)   # .r は (x,y,z,w)
        qk = (kp.r.x, kp.r.y, kp.r.z, kp.r.w)
        if d["q_closed"] is None:
            d["q_closed"] = qp              # スポーン姿勢=閉(鉛直ヒンジで漂流しない)
        theta = _rel_angle_about(qp, qk, 'x')             # レバー角(法線 X まわり)
        phi = _rel_angle_about(d["q_closed"], qp, 'z')    # ヒンジ角(世界 Z まわり)
        opened = max(0.0, d["open_dir"] * phi)            # >=0 = どれだけ開いたか
        lever = abs(theta)                                # 左右どちらのひねりも見る
        # --- ラッチ状態機械 (実物と同じ) ---
        #   施錠中にノブが THRESH 以上回った → 解錠 (握ったまま押す/引くと開く)。
        #   解錠中にノブが戻り(RELATCH 未満) かつ 扉が閉位置(CLOSE_EPS 以内) → 再施錠。
        #   開いたままノブを離しても施錠しない(実物どおり。閉めた瞬間にカチッと掛かる)。
        if d["locked"] and lever > self.THRESH:
            self._unlock(d)
            print('[door] %s: ラッチ解除 (lever=%.1fdeg) → ばねで少し開く'
                  % (d["name"], math.degrees(lever)), flush=True)
        elif not d["locked"]:
            if d["popping"]:
                # ラッチばね作動中: 目標まで開いたら(または離されてから2秒開かなければ)
                # ばねを切って扉を自由にする。この間は再施錠しない(ばねに開く猶予を与える)。
                # ノブを握っている間(レバーが倒れている間)はタイマーを進めない:
                # 腕が扉を固定していてばねは押せないので、離した瞬間から数える。
                # ★毎フレーム起こす: 眠った剛体は関節ドライブでは起きず、ばねが
                #   効かない(実測)。スリープ無効化済みだが二重の保険で起こしておく。
                dc.wake_up_rigid_body(d["ph"])
                if lever > self.RELATCH:
                    d["pop_left"] = self.POP_TIMEOUT
                else:
                    d["pop_left"] -= 1
                # 静止検知: ばねの減衰で扉が目標角付近に静止しかけたらばねを切る
                # (惰性ほぼゼロで切れるので「少し開いて止まる」になる)。
                d["pop_age"] = d.get("pop_age", 0) + 1
                _moving = abs(opened - d.get("prev_opened", opened)) > self.SETTLE_EPS
                d["prev_opened"] = opened
                _settled = ((not _moving) and d["pop_age"] >= self.SETTLE_MIN_AGE
                            and opened > self.CLOSE_EPS)
                if _settled or d["pop_left"] <= 0:
                    # ポップ終了: ばね有ヒンジ A を無効化し、ばね無ヒンジ B に交代。
                    # 以降の扉は減衰以外フリー = ちょんと押すだけでさらに開き、
                    # 押した所に留まる(ばねを残すと目標角へ引き戻す見えない壁になる)。
                    self._set_joint_enabled(d["joint_path"], False)
                    self._set_joint_enabled(d["joint_path_b"], True)
                    d["popping"] = False
                    if opened < math.radians(2.0):
                        # 開けなかった = 扉の進路が塞がれている。引く側(扉がロボットへ
                        # 向かって開く側)ではロボット自身が進路上に居ると開けない。
                        print('[door] %s: ポップ失敗 (opened=%.1fdeg) — 扉の進路が塞がれて'
                              'います。ノブを回した後、ロボットを後退させてください'
                              ' (扉は押し側の反対=引く側から見て手前に開きます)'
                              % (d["name"], math.degrees(opened)), flush=True)
                    else:
                        print('[door] %s: ポップ完了 (opened=%.1fdeg) → 扉フリー'
                              % (d["name"], math.degrees(opened)), flush=True)
            elif lever < self.RELATCH and opened < self.CLOSE_EPS:
                # 再施錠は条件が RELATCH_HOLD フレーム連続で成立したときだけ
                # (一瞬の成立でガチャッと施錠されて「開かない」のを防ぐデバウンス)。
                d["relatch_cnt"] += 1
                if d["relatch_cnt"] >= self.RELATCH_HOLD:
                    self._lock(d)
                    print('[door] %s: カチッと施錠 (扉が閉位置に戻った)' % d["name"],
                          flush=True)
            else:
                d["relatch_cnt"] = 0
        if self.DEBUG and (self._dbg % 30 == 0):
            print('[door-dbg] %s lever=%.1fdeg hinge=%.1fdeg locked=%s'
                  % (d["name"], math.degrees(theta), math.degrees(phi),
                     d["locked"]), flush=True)
