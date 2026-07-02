#!/usr/bin/env python3
"""door_spawn 最終検証: 本番経路 (DoorLatchController) でフルサイクルを確認。

  F1: スポーン=施錠+ばね装填済み → 施錠保持 (期待: 0°)
  F2: 解錠(コントローラの _unlock) → ばねがカチャッと開く (期待: ~12°で切断→摩擦保持)
  F3: 切断後にドリフトしないか (期待: その場保持)
  F4: 手で閉じたと仮定して再施錠→再解錠で 2 回目のポップも効くか (ばね再装填の検証)
実行: docker compose -f env_docker/docker-compose-ros2.yml run --rm --no-deps isaacsim \
        /isaac-sim/python.sh /app/configs/door_pop_test.py
"""
from isaacsim.simulation_app import SimulationApp

kit = SimulationApp({"headless": True})

import math
import sys

sys.path.insert(0, "/app")

import omni.timeline
import omni.usd
from omni.isaac.core import SimulationContext
from omni.isaac.dynamic_control import _dynamic_control

import door_spawn

sim = SimulationContext(stage_units_in_meters=1.0)

import yaml

cfg = {"doors": [{"name": "d1", "x": 0.0, "y": 0.0, "yaw": 0}]}
cfg_path = "/tmp/door_pop_test_cfg.yaml"
with open(cfg_path, "w") as f:
    yaml.safe_dump(cfg, f)

handles = []
n = door_spawn.spawn_doors("", kit, config_path=cfg_path,
                           initial_states=[], door_handles=handles)
print(f"[TEST] spawned {n}", flush=True)

sim.initialize_physics()
omni.timeline.get_timeline_interface().play()

dc = _dynamic_control.acquire_dynamic_control_interface()
ctrl = door_spawn.DoorLatchController(dc, handles)
D = ctrl.doors[0]


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


# F1: 安定化 + 施錠保持
run(90)
a1 = angle()
print(f"[TEST] F1 施錠保持: angle={a1:.2f}deg locked={D['locked']}", flush=True)

# F2: 解錠 (レバーが回された時と同じ処理)。レバー角0なので離した直後扱い=2秒以内に切断。
ctrl._unlock(D)
trace = []
for i in range(240):
    sim.step(render=False)
    ctrl.step()
    if (i + 1) % 30 == 0:
        trace.append(round(angle(), 2))
print(f"[TEST] F2 解錠後trace(0.5s毎)={trace} popping={D['popping']}", flush=True)
a2 = angle()

# F3: さらに3秒静観 → ドリフトしないか
run(180)
a3 = angle()
print(f"[TEST] F3 3秒後: angle={a3:.2f}deg (drift={abs(a3-a2):.2f})", flush=True)

# F4: 手で閉じたと仮定 → 再施錠 → 再解錠で 2 回目のポップ
#     (実際は扉が0°付近に戻った時に state machine が施錠する。ここでは直接呼ぶ)
#     まず扉を閉位置へ戻す: reset と同じテレポート(接触なしなので安全)
tf = _dynamic_control.Transform()
pp0 = None
for s in door_spawn.spawn_doors.__defaults__ or []:
    pass
# 初期姿勢は q_closed/spawn 位置。板の初期ワールド位置を組み立て情報から再構築するのは
# 冗長なので、リミットを(0,0)へ戻す施錠の引き込み(1.5°以内想定)は使わず、
# ここでは「開いた扉を摩擦に逆らって手で閉じた」代わりに施錠→ばね再装填だけ検証する。
ctrl._lock(D)
run(60)
a4 = angle()   # 施錠(0,0) は現在角から 0 へ硬い拘束で引き込む
print(f"[TEST] F4a 施錠で閉位置へ引き込み: angle={a4:.2f}deg locked={D['locked']}", flush=True)
ctrl._unlock(D)
trace2 = []
for i in range(240):
    sim.step(render=False)
    ctrl.step()
    if (i + 1) % 30 == 0:
        trace2.append(round(angle(), 2))
print(f"[TEST] F4b 2回目の解錠trace={trace2}", flush=True)

print("========== 判定 ==========", flush=True)
print(f"  F1 施錠保持: {'OK' if abs(a1) < 1.0 else 'NG'} ({a1:.2f}deg)", flush=True)
f2ok = min(trace) <= -8.0
print(f"  F2 ポップで開く: {'OK' if f2ok else 'NG'} (min={min(trace)}deg)", flush=True)
print(f"  F3 切断後保持: {'OK' if abs(a3-a2) < 3.0 else 'NG'} (drift={abs(a3-a2):.2f}deg)",
      flush=True)
f4ok = min(trace2) <= -8.0
print(f"  F4 2回目のポップ: {'OK' if f4ok else 'NG'} (min={min(trace2)}deg)", flush=True)
print("===========================", flush=True)

kit.close()
