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
	TASK_TIME=$(TIME) $(COMPOSE) up $(UP_FLAGS)

down:
	$(COMPOSE) down

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) ps -a

ros:
	$(COMPOSE) exec ros2 bash

isaacsim:
	$(COMPOSE) exec isaacsim bash

run:
	$(COMPOSE) exec -e TASK_TIME=$(TIME) isaacsim /ros_entrypoint.sh /isaac-sim/python.sh $(LAUNCH_PY)
