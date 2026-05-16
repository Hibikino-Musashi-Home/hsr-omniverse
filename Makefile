# ============================================================================
# HSR-Omniverse Makefile
#
# Usage:
#   make {ros1|ros2} <action> [pc]
#
#   ros1, ros2    Select ROS version (default: ros2 when omitted)
#   build         Build images
#   up            Start the stack
#   down          Stop and remove containers
#   logs          Tail logs of all services
#   ps            Container status
#   ros           Open bash in ros container (ros-noetic / ros2)
#   isaacsim      Open bash in isaacsim container
#   pc            CycloneDDS cross-PC mode (ros2 only, runtime only)
#
# Examples:
#   make ros2 build
#   make ros2 up
#   make ros2 up pc                  # PC mode, ROS_DOMAIN_ID=55 default
#   ROS_DOMAIN_ID=49 make ros2 up pc # override Domain ID
#   make ros1 build
#   make ros1 up
# ============================================================================

ROS_DOMAIN_ID_PC ?= 55

# --- ROS version selection (from goals) -------------------------------------
ROS := 2
ifneq (,$(filter ros1,$(MAKECMDGOALS)))
  ROS := 1
endif
ifneq (,$(filter ros2,$(MAKECMDGOALS)))
  ROS := 2
endif

ifeq ($(ROS),1)
  COMPOSE_FILE := env_docker/docker-compose.yml
  ROS_SERVICE  := ros-noetic
else
  COMPOSE_FILE := env_docker/docker-compose-ros2.yml
  ROS_SERVICE  := ros2
endif

COMPOSE := docker compose -f $(COMPOSE_FILE)

# --- pc modifier ------------------------------------------------------------
PC := $(if $(filter pc,$(MAKECMDGOALS)),1,)
ifeq ($(PC),1)
  ifneq ($(ROS),2)
    $(error 'pc' modifier requires ros2 (got ros$(ROS)). CycloneDDS PC mode is ROS2-only.)
  endif
  ENV_PC := CYCLONEDDS_URI=file:///cyclonedds.pc.xml ROS_DOMAIN_ID=$${ROS_DOMAIN_ID:-$(ROS_DOMAIN_ID_PC)}
else
  ENV_PC :=
endif

# --- Targets ----------------------------------------------------------------
.PHONY: help ros1 ros2 pc build up down logs ps ros isaacsim
.DEFAULT_GOAL := help

help:
	@echo "Usage: make {ros1|ros2} <action> [pc]"
	@echo ""
	@echo "Modifiers (pick one ROS version; pc is optional, ros2 only):"
	@echo "  ros1            ROS1 Noetic (compose: env_docker/docker-compose.yml)"
	@echo "  ros2            ROS2 Humble (compose: env_docker/docker-compose-ros2.yml) [default]"
	@echo "  pc              CycloneDDS cross-PC mode, ROS_DOMAIN_ID=$(ROS_DOMAIN_ID_PC) (override via env)"
	@echo ""
	@echo "Actions:"
	@echo "  build           Build images"
	@echo "  up              Start the stack"
	@echo "  down            Stop and remove containers"
	@echo "  logs            Tail logs"
	@echo "  ps              Container status"
	@echo "  ros             Open bash in ros container ($(ROS_SERVICE))"
	@echo "  isaacsim        Open bash in isaacsim container"
	@echo ""
	@echo "Examples:"
	@echo "  make ros2 build"
	@echo "  make ros2 up"
	@echo "  make ros2 up pc"
	@echo "  ROS_DOMAIN_ID=49 make ros2 up pc"
	@echo "  make ros1 build"
	@echo "  make ros1 up"
	@echo ""
	@echo "Resolved: ROS=$(ROS), PC=$(if $(PC),on,off), compose=$(COMPOSE_FILE)"

# Modifier "targets" — accept as no-ops so they can sit on the command line
ros1 ros2 pc:
	@:

build:
	$(ENV_PC) $(COMPOSE) build

up:
	$(ENV_PC) $(COMPOSE) up

down:
	$(COMPOSE) down

logs:
	$(COMPOSE) logs -f

ps:
	$(COMPOSE) ps -a

ros:
	$(COMPOSE) exec $(ROS_SERVICE) bash

isaacsim:
	$(COMPOSE) exec isaacsim bash
