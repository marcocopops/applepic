# ApplePic — developer entry points. Native ROS 2 Jazzy + gz Harmonic by default.
# Set USE_DOCKER=1 to run the same targets inside the Docker image (see docker/Dockerfile).
SHELL := /bin/bash
.ONESHELL:
ROS_DISTRO ?= jazzy
USE_DOCKER ?= 0
IMAGE ?= applepic:jazzy
SIM_SECONDS ?= 20

ifeq ($(USE_DOCKER),1)
RUN = docker run --rm --net=host -v $(CURDIR):/ws -w /ws $(IMAGE) bash -c
else
RUN = bash -c
endif

SETUP = source /opt/ros/$(ROS_DISTRO)/setup.bash && { [ -f install/setup.bash ] && source install/setup.bash || true; }

.PHONY: build test sim-headless world docker-image clean

build:
	$(RUN) 'source /opt/ros/$(ROS_DISTRO)/setup.bash && colcon build --symlink-install'

test: build
	$(RUN) '$(SETUP) && colcon test --return-code-on-test-failure --event-handlers console_direct- && colcon test-result --verbose'

world:
	$(RUN) '$(SETUP) && ros2 run applepic_gazebo generate_world --config src/applepic_gazebo/config/orchard.yaml --output src/applepic_gazebo/worlds/orchard.sdf'

sim-headless: build
	$(RUN) '$(SETUP) && timeout $(SIM_SECONDS) ros2 launch applepic_bringup sim.launch.py headless:=true || [ $$? -eq 124 ]'

docker-image:
	docker build -t $(IMAGE) -f docker/Dockerfile .

clean:
	rm -rf build install log
