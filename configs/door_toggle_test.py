#!/usr/bin/env python3
"""jointEnabled ホット切り替え検証: 事前生成した2本のヒンジ(ばね有A/無B)を
実行時に enable/disable で切り替えられるか (=削除/再作成を廃止できるか)。

  T1: A(ばね有・施錠)有効 + B(ばね無・全範囲)無効 → 施錠保持
  T2: A のリミットをホット解錠 → ばねポップで ~30° に静止
  T3: A無効/B有効 に切替 → 掴んで -60° へ引いて離す → 留まる(ばね切replace実証)
  T4: 扉を 0° 付近へ戻す → A施錠リミット+A有効/B無効 → 施錠
  T5: A を再解錠 → 2回目のポップ
実行: docker compose -f env_docker/docker-compose-ros2.yml run --rm --no-deps isaacsim \
        /isaac-sim/python.sh /app/configs/door_toggle_test.py
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
from pxr import Gf, UsdGeom, UsdPhysics

import door_spawn

sim = SimulationContext(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()

import yaml

cfg_yaml = {"doors": [{"name": "d1", "x": 0.0, "y": 0.0, "yaw": 0}]}
p = "/tmp/door_toggle_cfg.yaml"
with open(p, "w") as f:
    yaml.safe_dump(cfg_yaml, f)

handles = []
n = door_spawn.spawn_doors("", kit, config_path=p,
                           initial_states=[], door_handles=handles)
cfg = handles[0]["hinge_cfg"]
print(f"[TEST] spawned {n}", flush=True)

# ヒンジB (ばね無し・全範囲・無効) をパース前に作っておく
cfg_b = dict(cfg)
cfg_b["joint_path"] = cfg["joint_path"] + "_b"
door_spawn._author_hinge_joint(stage, cfg_b, with_spring=False, locked=False)
jA = UsdPhysics.RevoluteJoint.Get(stage, cfg["joint_path"])
jB = UsdPhysics.RevoluteJoint.Get(stage, cfg_b["joint_path"])
jB.GetPrim().GetAttribute("physics:jointEnabled").Set(False) if \
    jB.GetPrim().GetAttribute("physics:jointEnabled") else \
    UsdPhysics.Joint(jB.GetPrim()).CreateJointEnabledAttr(False)
print("[TEST] hinge B authored (springless, disabled)", flush=True)

# 「手」(キネマティック) — ドラッグ用
HAND = "/World/TestHand"
create_prim(HAND, "Cube", translation=[0.5, 0.5, 1.0], scale=[0.02, 0.02, 0.02])
hprim = stage.GetPrimAtPath(HAND)
rb = UsdPhysics.RigidBodyAPI.Apply(hprim)
rb.CreateRigidBodyEnabledAttr(True)
rb.CreateKinematicEnabledAttr(True)

sim.initialize_physics()
omni.timeline.get_timeline_interface().play()
dc = _dynamic_control.acquire_dynamic_control_interface()

ph = None
q0 = None
HINGE_W = (0.0, -0.39, 0.0)   # ヒンジのワールド位置 (left, W=0.8)


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
    pp = dc.get_rigid_body_pose(ph)
    q = (pp.r.x, pp.r.y, pp.r.z, pp.r.w)
    return math.degrees(door_spawn._rel_angle_about(q0, q, 'z'))


def run(steps):
    for _ in range(steps):
        sim.step(render=False)


def set_enabled(j, on):
    j.GetPrim().GetAttribute("physics:jointEnabled").Set(bool(on))


def knob_axis_anchor():
    """ノブのスピンドル(stem)中点のワールド位置。"""
    xc = UsdGeom.XformCache()
    pts = []
    for s in ("stem_front", "stem_back"):
        pr = stage.GetPrimAtPath(f"/World/Doors/d1/knob/{s}")
        t = xc.GetLocalToWorldTransform(pr).ExtractTranslation()
        pts.append((t[0], t[1], t[2]))
    return tuple(sum(c) / len(pts) for c in zip(*pts))


GJ = "/World/TestGraspJoint"


def grab():
    hp = dc.get_rigid_body_pose(hand)
    kp = dc.get_rigid_body_pose(knob)
    qh = (hp.r.x, hp.r.y, hp.r.z, hp.r.w)
    qk = (kp.r.x, kp.r.y, kp.r.z, kp.r.w)
    aw = knob_axis_anchor()
    lp0 = q_rot(q_conj(qh), (aw[0]-hp.p.x, aw[1]-hp.p.y, aw[2]-hp.p.z))
    lr0 = q_mul(q_conj(qh), qk)
    lp1 = q_rot(q_conj(qk), (aw[0]-kp.p.x, aw[1]-kp.p.y, aw[2]-kp.p.z))
    gj = UsdPhysics.Joint.Define(stage, GJ)
    gj.CreateBody0Rel().SetTargets([HAND])
    gj.CreateBody1Rel().SetTargets(["/World/Doors/d1/knob"])
    gj.CreateLocalPos0Attr().Set(Gf.Vec3f(*lp0))
    gj.CreateLocalRot0Attr().Set(Gf.Quatf(lr0[3], lr0[0], lr0[1], lr0[2]))
    gj.CreateLocalPos1Attr().Set(Gf.Vec3f(*lp1))
    gj.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
    for ax in ("rotX", "rotY", "rotZ"):
        la = UsdPhysics.LimitAPI.Apply(gj.GetPrim(), ax)
        la.CreateLowAttr(1.0)
        la.CreateHighAttr(-1.0)
    la = UsdPhysics.LimitAPI.Apply(gj.GetPrim(), "transY")
    la.CreateLowAttr(-0.06)
    la.CreateHighAttr(0.06)


def drag(direction, steps):
    """ヒンジ弧の接線方向に手を動かす。direction=+1 で開く向き、-1 で閉じる向き。"""
    tf = _dynamic_control.Transform()
    for _ in range(steps):
        hp = dc.get_rigid_body_pose(hand)
        r = (hp.p.x - HINGE_W[0], hp.p.y - HINGE_W[1])
        norm = math.sqrt(r[0]*r[0] + r[1]*r[1]) or 1.0
        # open_dir=-1(左ヒンジ): 開く=φ減少=接線 (r_y, -r_x)/|r|
        tx, ty = direction * r[1] / norm, -direction * r[0] / norm
        tf.p = (hp.p.x + 0.0025*tx, hp.p.y + 0.0025*ty, hp.p.z)
        tf.r = (hp.r.x, hp.r.y, hp.r.z, hp.r.w)
        dc.set_rigid_body_pose(hand, tf)
        sim.step(render=False)


run(60)
ph = dc.get_rigid_body("/World/Doors/d1/panel")
knob = dc.get_rigid_body("/World/Doors/d1/knob")
hand = dc.get_rigid_body(HAND)
pp = dc.get_rigid_body_pose(ph)
q0 = (pp.r.x, pp.r.y, pp.r.z, pp.r.w)
run(60)
a1 = angle()
print(f"[TEST] T1 施錠保持(A有効/B無効): {a1:.2f}deg", flush=True)

# T2: A をホット解錠 → ポップ
jA.GetLowerLimitAttr().Set(-cfg["open_limit_deg"])
jA.GetUpperLimitAttr().Set(0.0)
run(180)
a2 = angle()
print(f"[TEST] T2 解錠ポップ: {a2:.2f}deg (期待 ~-30)", flush=True)

# T3: A無効/B有効 → 掴んで -60 へ引いて離す → 留まるか
set_enabled(jA, False)
set_enabled(jB, True)
run(30)
# 手をアンカー位置へテレポートしてから掴む
aw = knob_axis_anchor()
tf = _dynamic_control.Transform()
tf.p = aw
tf.r = (0.0, 0.0, 0.0, 1.0)
dc.set_rigid_body_pose(hand, tf)
run(5)
grab()
run(10)
drag(+1, 240)          # さらに開く方向へ
stage.RemovePrim(GJ)   # 離す
run(120)
a3 = angle()
print(f"[TEST] T3 引いて離した後: {a3:.2f}deg (期待: -50前後に留まる=ばね無し)", flush=True)

# T4: 扉を閉位置へ戻して A施錠/B無効 → 施錠保持
grab()
run(10)
drag(-1, 300)          # 閉じる方向へ戻す
stage.RemovePrim(GJ)
run(30)
a4pre = angle()
jA.GetLowerLimitAttr().Set(0.0)
jA.GetUpperLimitAttr().Set(0.0)
set_enabled(jA, True)
set_enabled(jB, False)
run(60)
a4 = angle()
print(f"[TEST] T4 再施錠: 戻し後={a4pre:.2f}deg → 施錠後={a4:.2f}deg (期待 ~0)", flush=True)

# T5: 2回目の解錠ポップ
jA.GetLowerLimitAttr().Set(-cfg["open_limit_deg"])
jA.GetUpperLimitAttr().Set(0.0)
run(180)
a5 = angle()
print(f"[TEST] T5 2回目ポップ: {a5:.2f}deg (期待 ~-30)", flush=True)

print("========== 判定 ==========", flush=True)
print(f"  T1 施錠保持: {'OK' if abs(a1) < 1 else 'NG'} ({a1:.2f})", flush=True)
print(f"  T2 ポップ: {'OK' if a2 <= -20 else 'NG'} ({a2:.2f})", flush=True)
print(f"  T3 切替後フリー保持: {'OK' if a3 <= -40 else 'NG'} ({a3:.2f})", flush=True)
print(f"  T4 再施錠: {'OK' if abs(a4) < 2 else 'NG'} ({a4:.2f})", flush=True)
print(f"  T5 2回目ポップ: {'OK' if a5 <= -20 else 'NG'} ({a5:.2f})", flush=True)
print("===========================", flush=True)

kit.close()
