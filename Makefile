# ============================================================================
# carrobo-isaac Makefile (ロボット実習用 Isaac Sim 環境)
#
# 基本の使い方:
#   make build      イメージをビルド (初回のみ。30分〜1時間程度)
#   make up         シミュレータ一式を起動 (初回はシェーダー生成で10〜20分)
#   make down       停止・コンテナ削除
#   make ros        ros2 コンテナに入る (ros2 topic list などを叩く場所)
#   make isaacsim   isaacsim コンテナに入る
#   make logs       全サービスのログを表示
#   make ps         コンテナの状態を表示
#
# 競技モード:
#   make up TIME=600   競技時間 600 秒 (シミュレータ内時間)。4方向の観戦カメラで
#                      録画し、時間が来たら recordings/ に mp4 を保存して自動終了。
#
# 開発モード (シミュレータの Python を手動実行したいとき):
#   make dev up     コンテナだけバックグラウンド起動 (シミュレータは自動起動しない)
#   make dev run    同じ端末でシミュレータを実行 (ログ・エラーがここに出る)
#   make dev down   停止
# ============================================================================

# -p carrobo-isaac: compose のプロジェクト名。これを付けないとフォルダ名 (env_docker) が
# プロジェクト名になり、hsr-omniverse のイメージ (env_docker-isaacsim 等) と衝突・上書きしてしまう。
COMPOSE := docker compose -p carrobo-isaac -f env_docker/docker-compose.yml

# --- dev modifier -----------------------------------------------------------
# 'dev' を付けると docker-compose.dev.yml を重ねて適用する。
# isaacsim は 'sleep infinity' で待機起動し、Python は自動実行されない。
# その後 'make dev run' で手動実行する (ログがその端末だけに出る)。
DEV := $(if $(filter dev,$(MAKECMDGOALS)),1,)
ifeq ($(DEV),1)
  COMPOSE := $(COMPOSE) -f docker-compose.dev.yml
  # dev では scripts/ をディレクトリごと live マウントするので、そちらを実行する
  # (編集がコンテナ再起動なしで反映される)。
  LAUNCH_PY := /app/scripts/launch_isaacsim.py
  # dev では up をバックグラウンド(-d)にし、端末をすぐ返す。
  UP_FLAGS := -d
else
  # 通常起動はフォアグラウンド (全コンテナのログを表示)。
  LAUNCH_PY := /app/launch_isaacsim.py
  UP_FLAGS :=
  # 競技モード (TIME=秒) のときは、シミュレータが自動終了したら
  # 他のコンテナ (ros2 等) も一緒に止めて make up 自体を終わらせる。
  ifneq ($(TIME),)
    UP_FLAGS += --abort-on-container-exit
  endif
endif

# --- Targets ----------------------------------------------------------------
.PHONY: help dev build up down logs ps ros isaacsim run
.DEFAULT_GOAL := help

RVIZ ?= true
USE_RVIZ := $(if $(filter 1 true TRUE yes YES on ON,$(RVIZ)),true,false)
BASE_DIRECT_DRIVE ?= 1
BASE_TRAJ_P_GAIN ?= 2.0
BASE_TRAJ_D_GAIN ?= 0.5
BASE_TRAJ_I_GAIN ?= 4.0
BASE_TRAJ_I_LINEAR_LIMIT ?= 0.08
BASE_TRAJ_I_ANGULAR_LIMIT ?= 0.15
BASE_CMD_TAU ?= 0.10
BASE_WHEEL_ACCEL_LIMIT ?= 41.7
BASE_STEER_ACCEL_LIMIT ?= 5.0
BASE_LINEAR_ACCEL_LIMIT ?= 0.35
BASE_ANGULAR_ACCEL_LIMIT ?= 0.8
BASE_WHEEL_DRIVE_DAMPING ?= 10.0
BASE_WHEEL_DRIVE_MAX_FORCE ?= 10.0
BASE_STEER_DRIVE_DAMPING ?= 5.0
BASE_STEER_DRIVE_MAX_FORCE ?= 5.0
BASE_DIRECT_JOINT_DAMPING ?= 1.0
BASE_DIRECT_JOINT_MAX_FORCE ?= 1.0
BASE_DIRECT_ROOT_DAMPING ?= 2.0
BASE_BRAKE ?= 1
BASE_BRAKE_ENGAGE_LINEAR ?= 0.05
BASE_BRAKE_ENGAGE_ANGULAR ?= 0.10
BASE_BRAKE_CREEP_SPEED ?= 0.05
BASE_BRAKE_CREEP_ANGULAR ?= 0.15
BASE_BRAKE_K ?= 200000
BASE_BRAKE_C ?= 20000
BASE_BRAKE_MAX_FORCE ?= 20000
BASE_BRAKE_ANGULAR_K ?= 50000
BASE_BRAKE_ANGULAR_C ?= 5000
BASE_BRAKE_MAX_TORQUE ?= 5000
BASE_SETPOINT_MAX_LAG ?= 0.02
BASE_SETPOINT_MAX_LAG_ANGULAR ?= 0.05
BASE_GOAL_VELOCITY_TOLERANCE ?= 0.10
BASE_JOINT_BRAKE ?= 1
BASE_DIAG ?= 0
CAMERA_FRAME_SKIP ?= 2
RENDER_EVERY_N_STEPS ?= 4

help:
	@echo "Usage: make <action> [dev] [TIME=<秒>]"
	@echo ""
	@echo "Actions:"
	@echo "  build     Build images"
	@echo "  up        Start the stack (Isaac Sim + ROS2)"
	@echo "  down      Stop and remove containers"
	@echo "  logs      Tail logs"
	@echo "  ps        Container status"
	@echo "  ros       Open bash in ros2 container"
	@echo "  isaacsim  Open bash in isaacsim container"
	@echo "  run       (dev) Run launch_isaacsim.py manually in isaacsim"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make up"
	@echo "  make up RVIZ=0    # RViz2 を起動しない"
	@echo "  make up BASE_TRAJ_P_GAIN=1.0   # whole_body台車FBを弱める"
	@echo "  make up BASE_DIRECT_DRIVE=0    # 物理車輪駆動へ戻す"
	@echo "  make up TIME=600   # 競技モード: 600秒(シミュ内時間)で自動終了。"
	@echo "                     # 4方向の観戦カメラで録画し recordings/ に mp4 保存"
	@echo ""
	@echo "  # Dev mode (開発用):"
	@echo "  make dev up     # start containers in background, isaacsim idle"
	@echo "  make dev run    # same terminal: run the sim, logs shown here"
	@echo "  make dev down   # stop when finished"

# Modifier "target" — accept as no-op so it can sit on the command line
dev:
	@:

build:
	$(COMPOSE) build

# TIME=<秒> を付けると競技モード (環境変数 TASK_TIME で isaacsim コンテナに渡る)。
up:
	TASK_TIME=$(TIME) GRASP_ATTACH=$(GRASP) USE_RVIZ=$(USE_RVIZ) BASE_DIRECT_DRIVE=$(BASE_DIRECT_DRIVE) BASE_TRAJ_P_GAIN=$(BASE_TRAJ_P_GAIN) BASE_TRAJ_D_GAIN=$(BASE_TRAJ_D_GAIN) BASE_TRAJ_I_GAIN=$(BASE_TRAJ_I_GAIN) BASE_TRAJ_I_LINEAR_LIMIT=$(BASE_TRAJ_I_LINEAR_LIMIT) BASE_TRAJ_I_ANGULAR_LIMIT=$(BASE_TRAJ_I_ANGULAR_LIMIT) BASE_CMD_TAU=$(BASE_CMD_TAU) BASE_WHEEL_ACCEL_LIMIT=$(BASE_WHEEL_ACCEL_LIMIT) BASE_STEER_ACCEL_LIMIT=$(BASE_STEER_ACCEL_LIMIT) BASE_LINEAR_ACCEL_LIMIT=$(BASE_LINEAR_ACCEL_LIMIT) BASE_ANGULAR_ACCEL_LIMIT=$(BASE_ANGULAR_ACCEL_LIMIT) BASE_WHEEL_DRIVE_DAMPING=$(BASE_WHEEL_DRIVE_DAMPING) BASE_WHEEL_DRIVE_MAX_FORCE=$(BASE_WHEEL_DRIVE_MAX_FORCE) BASE_STEER_DRIVE_DAMPING=$(BASE_STEER_DRIVE_DAMPING) BASE_STEER_DRIVE_MAX_FORCE=$(BASE_STEER_DRIVE_MAX_FORCE) BASE_DIRECT_JOINT_DAMPING=$(BASE_DIRECT_JOINT_DAMPING) BASE_DIRECT_JOINT_MAX_FORCE=$(BASE_DIRECT_JOINT_MAX_FORCE) BASE_DIRECT_ROOT_DAMPING=$(BASE_DIRECT_ROOT_DAMPING) BASE_BRAKE=$(BASE_BRAKE) BASE_BRAKE_ENGAGE_LINEAR=$(BASE_BRAKE_ENGAGE_LINEAR) BASE_BRAKE_ENGAGE_ANGULAR=$(BASE_BRAKE_ENGAGE_ANGULAR) BASE_BRAKE_CREEP_SPEED=$(BASE_BRAKE_CREEP_SPEED) BASE_BRAKE_CREEP_ANGULAR=$(BASE_BRAKE_CREEP_ANGULAR) BASE_BRAKE_K=$(BASE_BRAKE_K) BASE_BRAKE_C=$(BASE_BRAKE_C) BASE_BRAKE_MAX_FORCE=$(BASE_BRAKE_MAX_FORCE) BASE_BRAKE_ANGULAR_K=$(BASE_BRAKE_ANGULAR_K) BASE_BRAKE_ANGULAR_C=$(BASE_BRAKE_ANGULAR_C) BASE_BRAKE_MAX_TORQUE=$(BASE_BRAKE_MAX_TORQUE) BASE_SETPOINT_MAX_LAG=$(BASE_SETPOINT_MAX_LAG) BASE_SETPOINT_MAX_LAG_ANGULAR=$(BASE_SETPOINT_MAX_LAG_ANGULAR) BASE_GOAL_VELOCITY_TOLERANCE=$(BASE_GOAL_VELOCITY_TOLERANCE) BASE_JOINT_BRAKE=$(BASE_JOINT_BRAKE) BASE_DIAG=$(BASE_DIAG) CAMERA_FRAME_SKIP=$(CAMERA_FRAME_SKIP) RENDER_EVERY_N_STEPS=$(RENDER_EVERY_N_STEPS) $(COMPOSE) up $(UP_FLAGS)

down:
	$(COMPOSE) down

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) ps -a

# /ros_entrypoint.sh 経由で bash を起動する。これで ROS 環境
# (/opt/ros/humble + /ws の hsrb_interface 等) が source 済みの状態で入れる。
ros:
	$(COMPOSE) exec ros2 /ros_entrypoint.sh bash

isaacsim:
	$(COMPOSE) exec isaacsim bash

run:
	$(COMPOSE) exec -e TASK_TIME=$(TIME) -e BASE_DIRECT_DRIVE=$(BASE_DIRECT_DRIVE) -e BASE_TRAJ_P_GAIN=$(BASE_TRAJ_P_GAIN) -e BASE_TRAJ_D_GAIN=$(BASE_TRAJ_D_GAIN) -e BASE_TRAJ_I_GAIN=$(BASE_TRAJ_I_GAIN) -e BASE_TRAJ_I_LINEAR_LIMIT=$(BASE_TRAJ_I_LINEAR_LIMIT) -e BASE_TRAJ_I_ANGULAR_LIMIT=$(BASE_TRAJ_I_ANGULAR_LIMIT) -e BASE_CMD_TAU=$(BASE_CMD_TAU) -e BASE_WHEEL_ACCEL_LIMIT=$(BASE_WHEEL_ACCEL_LIMIT) -e BASE_STEER_ACCEL_LIMIT=$(BASE_STEER_ACCEL_LIMIT) -e BASE_LINEAR_ACCEL_LIMIT=$(BASE_LINEAR_ACCEL_LIMIT) -e BASE_ANGULAR_ACCEL_LIMIT=$(BASE_ANGULAR_ACCEL_LIMIT) -e BASE_WHEEL_DRIVE_DAMPING=$(BASE_WHEEL_DRIVE_DAMPING) -e BASE_WHEEL_DRIVE_MAX_FORCE=$(BASE_WHEEL_DRIVE_MAX_FORCE) -e BASE_STEER_DRIVE_DAMPING=$(BASE_STEER_DRIVE_DAMPING) -e BASE_STEER_DRIVE_MAX_FORCE=$(BASE_STEER_DRIVE_MAX_FORCE) -e BASE_DIRECT_JOINT_DAMPING=$(BASE_DIRECT_JOINT_DAMPING) -e BASE_DIRECT_JOINT_MAX_FORCE=$(BASE_DIRECT_JOINT_MAX_FORCE) -e BASE_DIRECT_ROOT_DAMPING=$(BASE_DIRECT_ROOT_DAMPING) -e BASE_BRAKE=$(BASE_BRAKE) -e BASE_BRAKE_ENGAGE_LINEAR=$(BASE_BRAKE_ENGAGE_LINEAR) -e BASE_BRAKE_ENGAGE_ANGULAR=$(BASE_BRAKE_ENGAGE_ANGULAR) -e BASE_BRAKE_CREEP_SPEED=$(BASE_BRAKE_CREEP_SPEED) -e BASE_BRAKE_CREEP_ANGULAR=$(BASE_BRAKE_CREEP_ANGULAR) -e BASE_BRAKE_K=$(BASE_BRAKE_K) -e BASE_BRAKE_C=$(BASE_BRAKE_C) -e -e BASE_BRAKE_MAX_FORCE=$(BASE_BRAKE_MAX_FORCE) -e BASE_BRAKE_ANGULAR_K=$(BASE_BRAKE_ANGULAR_K) -e BASE_BRAKE_ANGULAR_C=$(BASE_BRAKE_ANGULAR_C) -e -e BASE_BRAKE_MAX_TORQUE=$(BASE_BRAKE_MAX_TORQUE) -e BASE_SETPOINT_MAX_LAG=$(BASE_SETPOINT_MAX_LAG) -e BASE_SETPOINT_MAX_LAG_ANGULAR=$(BASE_SETPOINT_MAX_LAG_ANGULAR) -e BASE_GOAL_VELOCITY_TOLERANCE=$(BASE_GOAL_VELOCITY_TOLERANCE) -e BASE_JOINT_BRAKE=$(BASE_JOINT_BRAKE) -e -e BASE_DIAG=$(BASE_DIAG) BASE_BRAKE_K=$(BASE_BRAKE_K) BASE_BRAKE_C=$(BASE_BRAKE_C) BASE_BRAKE_MAX_FORCE=$(BASE_BRAKE_MAX_FORCE) BASE_BRAKE_ANGULAR_K=$(BASE_BRAKE_ANGULAR_K) BASE_BRAKE_ANGULAR_C=$(BASE_BRAKE_ANGULAR_C) BASE_BRAKE_MAX_TORQUE=$(BASE_BRAKE_MAX_TORQUE) BASE_SETPOINT_MAX_LAG=$(BASE_SETPOINT_MAX_LAG) BASE_SETPOINT_MAX_LAG_ANGULAR=$(BASE_SETPOINT_MAX_LAG_ANGULAR) BASE_GOAL_VELOCITY_TOLERANCE=$(BASE_GOAL_VELOCITY_TOLERANCE) BASE_JOINT_BRAKE=$(BASE_JOINT_BRAKE) BASE_DIAG=$(BASE_DIAG) -e CAMERA_FRAME_SKIP=$(CAMERA_FRAME_SKIP) -e RENDER_EVERY_N_STEPS=$(RENDER_EVERY_N_STEPS) isaacsim /ros_entrypoint.sh /isaac-sim/python.sh $(LAUNCH_PY)
