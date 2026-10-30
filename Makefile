# ApplePic — developer entry points. Native ROS 2 Jazzy + gz Harmonic by default.
# Set USE_DOCKER=1 to run the same targets inside the Docker image (see docker/Dockerfile).
SHELL := /bin/bash
.ONESHELL:
ROS_DISTRO ?= jazzy
USE_DOCKER ?= 0
IMAGE ?= applepic:jazzy
SIM_SECONDS ?= 20
E2E_MIN_RATIO ?= 0.8
E2E_SIM_LIMIT ?= 1200
E2E_RTF ?= 3
E2E_SEED ?=
# Optional world config for e2e, e.g. src/applepic_gazebo/config/orchard_hard.yaml.
ORCHARD_CONFIG ?=
# Localization for e2e/demo: truth (ground-truth pose) or gps (GPS + wheel odometry EKF).
LOCALIZATION ?= truth
GPS_NOISE ?= 0.02
GPS_BIAS ?= 0.0
DEMO_RTF ?= 1

ifeq ($(USE_DOCKER),1)
RUN = docker run --rm --net=host -v $(CURDIR):/ws -w /ws $(IMAGE) bash -c
else
RUN = bash -c
endif

SETUP = source /opt/ros/$(ROS_DISTRO)/setup.bash && { [ -f install/setup.bash ] && source install/setup.bash || true; }

.PHONY: build test e2e demo sim-headless world docker-image clean

build:
	$(RUN) 'source /opt/ros/$(ROS_DISTRO)/setup.bash && colcon build --symlink-install'

test: build
	$(RUN) '$(SETUP) && colcon test --return-code-on-test-failure --event-handlers console_direct- && colcon test-result --verbose'

# M7 end-to-end scenario (~5 min wall at RTF 3): metrics in artifacts/e2e_metrics.json.
# E2E_SEED=<n> regenerates the apples with that seed (world in artifacts/, not committed).
e2e: build
	$(RUN) 'ORCHARD_CONFIG=$(ORCHARD_CONFIG) LOCALIZATION=$(LOCALIZATION) GPS_NOISE=$(GPS_NOISE) GPS_BIAS=$(GPS_BIAS) scripts/e2e_mission.sh $(E2E_MIN_RATIO) $(E2E_SIM_LIMIT) $(E2E_RTF) 10 1800 $(E2E_SEED)'

# M8 demo: full mission with the Gazebo GUI + RViz (needs a display, native only).
demo: build
	$(SETUP) && ros2 launch applepic_bringup demo.launch.py rtf:=$(DEMO_RTF) localization:=$(LOCALIZATION) gps_noise:=$(GPS_NOISE) gps_bias:=$(GPS_BIAS)

world:
	$(RUN) '$(SETUP) && ros2 run applepic_gazebo generate_world --config src/applepic_gazebo/config/orchard.yaml --output src/applepic_gazebo/worlds/orchard.sdf'

sim-headless: build
	$(RUN) '$(SETUP) && timeout $(SIM_SECONDS) ros2 launch applepic_bringup sim.launch.py headless:=true || [ $$? -eq 124 ]'

docker-image:
	docker build -t $(IMAGE) -f docker/Dockerfile .

clean:
	rm -rf build install log
