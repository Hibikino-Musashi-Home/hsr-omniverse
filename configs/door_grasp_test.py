#!/usr/bin/env python3
"""「ノブを握ったまま扉を開ける」= 実行時に手↔ノブを関節でつなぐ方式の検証。

  G1: 実行時に D6 関節(並進ロック・回転自由)を 手(キネマティック)↔ノブ に生成できるか
  G2: 解錠後、手を引く側(+X)へ後退させると扉がついてきて開くか (期待: 手に追従して開く)
  G3: 途中で関節を削除(手を離す)しても物理が壊れず、扉がその場に留まるか
実行: docker compose -f env_docker/docker-compose-ros2.yml run --rm --no-deps isaacsim \
        /isaac-sim/python.sh /app/configs/door_grasp_test.py
"""
from isaacsim.simulation_app import SimulationApp

kit = SimulationApp({"headless": True})

import math
import sys

sys.path.insert(0, "/app")

import omni.timeline
import omni.usd
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.dynamic_control import _dynamic_control
from pxr import Gf, UsdPhysics

import door_spawn

sim = SimulationContext(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

import yaml

cfg = {"doors": [{"name": "d1", "x": 0.0, "y": 0.0, "yaw": 0}]}
cfg_path = "/tmp/door_grasp_cfg.yaml"
with open(cfg_path, "w") as f:
    yaml.safe_dump(cfg, f)

handles = []
n = door_spawn.spawn_doors("", kit, config_path=cfg_path,
                           initial_states=[], door_handles=handles)
print(f"[TEST] spawned {n}", flush=True)

# 「手」役のキネマティック剛体 (ロボットの位置制御された手の代役)。
# 引く側(+X)のレバー棒の中央に置く (握り点)。レバー中心 ≈ (0.13, 0.165, 1.0)。
HAND = "/World/TestHand"
create_prim(HAND, "Cube", translation=[0.13, 0.165, 1.0], scale=[0.02, 0.02, 0.02])
hand_prim = stage.GetPrimAtPath(HAND)
rb = UsdPhysics.RigidBodyAPI.Apply(hand_prim)
rb.CreateRigidBodyEnabledAttr(True)
rb.CreateKinematicEnabledAttr(True)

sim.initialize_physics()
omni.timeline.get_timeline_interface().play()

dc = _dynamic_control.acquire_dynamic_control_interface()
ctrl = door_spawn.DoorLatchController(dc, handles)
D = ctrl.doors[0]


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def q_rot(q, v):
    r = q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_conj(q))
    return (r[0], r[1], r[2])


def angle():
    if not D["ph"] or D["q_closed"] is None:
        return float("nan")
    pp = dc.get_rigid_body_pose(D["ph"])
    q = (pp.r.x, pp.r.y, pp.r.z, pp.r.w)
    return math.degrees(door_spawn._rel_angle_about(D["q_closed"], q, 'z'))


def run(steps):
    for _ in range(steps):
        sim.step(render=False)
        ctrl.step()


# 安定化
run(60)
hand = dc.get_rigid_body(HAND)
knob = dc.get_rigid_body("/World/Doors/d1/knob")
hp = dc.get_rigid_body_pose(hand)
kp = dc.get_rigid_body_pose(knob)
print(f"[TEST] settle: door={angle():.2f}deg locked={D['locked']}", flush=True)

# --- G1: 実行時に D6 関節を生成 (並進ロック・回転自由) ---
# ★アンカーは「手の位置=実際の握り点」に置く。ノブ剛体の原点は組み立ての都合で
#   扉の足元中央にあるため、そこにアンカーすると床を握ったことになる(前回の失敗)。
#   frame0 = 手そのもの(原点・向きとも)。frame1 = ノブローカルで見た手の位置/向き。
qh = (hp.r.x, hp.r.y, hp.r.z, hp.r.w)
qk = (kp.r.x, kp.r.y, kp.r.z, kp.r.w)
# アンカー = レバーの回転軸上(スピンドル中点、組み立て座標 (0, 0.23, 1.0))。
# hsr.py 本実装と同じ(棒の上に置くとひねりでアンカーが動き腕と衝突するため)。
AW = (0.0, 0.23, 1.0)
lp0 = q_rot(q_conj(qh), (AW[0] - hp.p.x, AW[1] - hp.p.y, AW[2] - hp.p.z))
lr0 = q_mul(q_conj(qh), qk)                   # frame の軸 = ノブ軸
lp1 = q_rot(q_conj(qk), (AW[0] - kp.p.x, AW[1] - kp.p.y, AW[2] - kp.p.z))
GJ = "/World/TestGraspJoint"
gj = UsdPhysics.Joint.Define(stage, GJ)
gj.CreateBody0Rel().SetTargets([HAND])
gj.CreateBody1Rel().SetTargets(["/World/Doors/d1/knob"])
gj.CreateLocalPos0Attr().Set(Gf.Vec3f(*lp0))
gj.CreateLocalRot0Attr().Set(Gf.Quatf(lr0[3], lr0[0], lr0[1], lr0[2]))
gj.CreateLocalPos1Attr().Set(Gf.Vec3f(*lp1))
gj.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
# 破断しきい値は付けない(解錠時のヒンジ再構築の衝撃で誤破断する。実測)
for ax in ("rotX", "rotY", "rotZ"):           # 回転3軸は自由 (low>high = free)
    la = UsdPhysics.LimitAPI.Apply(gj.GetPrim(), ax)
    la.CreateLowAttr(1.0)
    la.CreateHighAttr(-1.0)
# レバーの棒(ノブローカル Y)に沿ったスライドを許す「リング握り」。
# ノブはヒンジ中心の弧を描くため、直線的に引く手との横ズレをこのスライドが吸収する
# (人間が握ったまま手をハンドル上で滑らせるのと同じ)。棒の端(±6cm)で止まる。
la_y = UsdPhysics.LimitAPI.Apply(gj.GetPrim(), "transY")
la_y.CreateLowAttr(-0.06)
la_y.CreateHighAttr(0.06)
run(30)
a_g1 = angle()
print(f"[TEST] G1 関節生成後: door={a_g1:.2f}deg (爆発せず安定なら OK)", flush=True)

# --- G2: 解錠して手を「弧を描いて」引く → 扉がついてくるか ---
# ノブはヒンジ中心の弧を描くので、直線後退では幾何的にすぐ頭打ちになる(実ロボットも同じ)。
# HSR は全方位台車なので、後退(+X)しながら横(-Y)へも流して弧に沿わせる。
ctrl._unlock(D)
trace = []
tf = _dynamic_control.Transform()
for i in range(240):
    hp = dc.get_rigid_body_pose(hand)
    tf.p = (hp.p.x + 0.0022, hp.p.y - 0.0013, hp.p.z)   # 弧に沿う斜め後退
    tf.r = (hp.r.x, hp.r.y, hp.r.z, hp.r.w)
    dc.set_rigid_body_pose(hand, tf)
    sim.step(render=False)
    ctrl.step()
    if (i + 1) % 30 == 0:
        trace.append(round(angle(), 2))
print(f"[TEST] G2 弧引き中の扉角度(0.5s毎)={trace}", flush=True)

# --- G3: 手を離す (関節削除) → 壊れず留まるか ---
stage.RemovePrim(GJ)
run(120)
a_g3 = angle()
pp = dc.get_rigid_body_pose(D["ph"])
sane = abs(pp.p.x) < 5 and abs(pp.p.y) < 5 and 0 < pp.p.z < 3
print(f"[TEST] G3 関節削除2秒後: door={a_g3:.2f}deg panel_pos=({pp.p.x:.2f},{pp.p.y:.2f},{pp.p.z:.2f})",
      flush=True)

print("========== 判定 ==========", flush=True)
print(f"  G1 実行時関節生成: {'OK' if abs(a_g1) < 3.0 else 'NG'} ({a_g1:.2f}deg)", flush=True)
g2ok = min(trace) <= -20.0
print(f"  G2 握ったまま引いて開く: {'OK' if g2ok else 'NG'} (min={min(trace)}deg)", flush=True)
print(f"  G3 離しても安定: {'OK' if sane else 'NG'} ({a_g3:.2f}deg)", flush=True)
print("===========================", flush=True)

kit.close()
