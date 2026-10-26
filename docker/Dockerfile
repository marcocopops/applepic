# Optional containerised environment (the default workflow is native, see docs/DECISIONS.md).
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz ros-jazzy-xacro ros-jazzy-robot-state-publisher \
      ros-jazzy-cv-bridge python3-opencv python3-yaml python3-pytest \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
