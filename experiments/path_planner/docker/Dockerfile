# Planner-only bench environment for experiments/path_planner.
# ROS 2 Jazzy (Ubuntu 24.04) + Nav2 map_server / planner_server (NavFn) / lifecycle_manager.
# No Gazebo, no AMCL, no controller: only global planning on a static map.
#
#   docker build -t amr-path-planner-bench:jazzy experiments/path_planner/docker
#
# The repository is mounted at run time (see experiments/path_planner/README.md).
FROM ros:jazzy-ros-base

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-nav2-planner \
        ros-jazzy-nav2-navfn-planner \
        ros-jazzy-nav2-smac-planner \
        ros-jazzy-nav2-map-server \
        ros-jazzy-nav2-lifecycle-manager \
        ros-jazzy-nav2-msgs \
        python3-pytest \
        python3-yaml \
    && rm -rf /var/lib/apt/lists/*

ENV BENCH_ENV=docker:amr-path-planner-bench:jazzy
WORKDIR /repo
