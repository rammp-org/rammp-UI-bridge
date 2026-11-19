# syntax=docker/dockerfile:1

# Container image for the rammp_ui_bridge node (+ its C++
# rammp_ui_bridge_streaming sibling package).
#
# Copied from RAMMP-docker's templates/module.Dockerfile (see that repo for
# the generic, heavily-commented version this is based on) and
# rammp-orchestrator's Dockerfile (same base-image/overlay pattern).
#
# The base image already contains ROS 2 Humble, Cyclone DDS, and the RAMMP
# interface contract (including rammp_ui_bridge_interfaces and
# rammp_orchestrator_interfaces), so this copies ONLY this repo.

# ── Pick and pin the base ────────────────────────────────────────────────────
# rammp-base, not rammp-cuda: the bridge has no GPU workload -- it only
# relays UE WebSocket/RMSS traffic and ROS2 topics/services. Pin an explicit
# VERSION, not the floating :humble tag, so this image records exactly which
# robot-interface version it was built against.
#
# This image is arm64/Jetson only -- build it on the Jetson, or on a native
# arm64 CI runner.
#
# 2.0.0-jp6 carries rammp_ui_bridge_interfaces, but only as of the
# rammp-interfaces-ros2 contract it compiled in at the time (v2.0.0), which
# predates the UI-control services this node actually calls
# (SetEngineOverlayVisible/UpdateUserInputPanel were added in interfaces
# v2.1.0). 2.2.0-jp6 is the first rammp-base release built against
# rammp-interfaces-ros2 v2.1.0+ (matches rammp-orchestrator's own pin) --
# the earliest base tag that actually has everything this bridge needs.
FROM ghcr.io/rammp-org/rammp-base:2.2.0-jp6

# ── Build this repo's packages into their own overlay ───────────────────────
# /module_ws is a separate workspace layered on top of the base's /ros2_ws.
# The entrypoint sources it automatically. Building here rather than into
# /ros2_ws means this build does not recompile the robot interfaces (much
# faster) and cannot accidentally ship a modified copy of them.
WORKDIR /module_ws

COPY . /module_ws/src/rammp_ui_bridge/

# colcon finds every package.xml under src/, so both rammp_ui_bridge
# (ament_python) and rammp_ui_bridge_streaming (ament_cmake, C++) are built.
# The base ships rosdep's sources but no cache, so rosdep update first (bare
# rosdep install fails with "rosdep installation has not been initialized
# yet" otherwise). ramms-tools (rammp_ui_bridge/setup.py's pinned git
# dependency) has no rosdep key -- `colcon build`'s ament_python step does
# NOT pip-install a package's install_requires (confirmed the hard way: CI's
# first real build produced an image that couldn't import ramms_tools at
# all), so it needs its own explicit pip install, pinned to the exact same
# tag setup.py declares (per package.xml's own comment on this). Plain
# `pip install git+<url>@<tag>`, not the PEP 508 `name @ git+<url>` form --
# the latter also failed CI, since pip cross-checks the requested name
# ("ramms-tools") against this package's own build metadata name and
# rejects the install on any mismatch.
#
# Also upgrade setuptools/wheel first AND build with --no-build-isolation:
# ramms-tools has no setup.py/setup.cfg, only a pyproject.toml with PEP 621
# `[project]` metadata and `[tool.setuptools.packages.find]`. Ubuntu/Debian's
# python3-pip's "isolated" build environment for a pyproject.toml-only
# package does not fully isolate -- it still resolves the system
# `/usr/lib/python3/dist-packages/setuptools` (jammy's 59.6.0, which
# predates PEP 621 support) ahead of whatever newer setuptools pip installs
# into its own ephemeral build env, so metadata generation silently reads no
# name/version/packages at all. Confirmed the hard way twice: CI's build
# kept "succeeding" while silently producing a near-empty `UNKNOWN-0.0.0`
# wheel (1.8 KB, vs. the real ~119 KB package) even after upgrading
# setuptools system-wide, because that upgrade only helps once isolation is
# turned off and the (now-correct) real environment's setuptools is used
# directly instead of pip's broken ephemeral one.
# Keeps /module_ws/build (unlike rammp-orchestrator's Dockerfile, which
# removes it): rammp_ui_bridge_streaming's gtest binary isn't installed by
# `colcon build` (ament_add_gtest only places it under build/), and CI's own
# docker-build-then-test-inside-the-image step needs it there.
RUN . /opt/ros/humble/setup.sh \
    && . /ros2_ws/install/setup.sh \
    && apt-get update \
    && apt-get install -y --no-install-recommends git \
    && rosdep update --rosdistro humble \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir --upgrade 'setuptools>=68,<80' wheel \
    && python3 -m pip install --no-cache-dir --no-build-isolation \
         'git+https://github.com/rammp-org/ramms-tools.git@v0.2.1' \
    && colcon build \
    && rm -rf /module_ws/log

# ros2 launch: what every engine developer actually runs today, per
# README.md/docs/MANUAL_TESTING.md, and the default when this image is run
# standalone (no sheppy). sheppy's own rammp-alternative.yaml overrides this
# CMD with the node's installed executable directly (ros2 launch rejects the
# --ros-args --params-file sheppy appends) -- the launch file's
# LaunchConfiguration names are the same as the node's own parameter names,
# so the same params work either way.
CMD ["ros2", "launch", "rammp_ui_bridge", "ui_bridge.launch.py"]

# `ros2 launch` on Humble has no SIGTERM handler: as PID 1 it ignores the
# SIGTERM `docker stop` (and `sheppy woof`) sends and is SIGKILLed after the
# grace period, leaving the node no chance to close its UE WebSocket/RMSS
# connections cleanly. On SIGINT it shuts its nodes down properly, so make
# that the stop signal (same fix already proven in
# rammp-kitchen-capability-engines's Dockerfiles).
STOPSIGNAL SIGINT

# ── Running on the robot ────────────────────────────────────────────────────
#   docker run --rm --network host --ipc host \
#     ghcr.io/rammp-org/rammp-ui-bridge:<version>
#
#   --network host    required for DDS discovery. On a bridge network Cyclone
#                     binds the bridge interface and nodes in other containers
#                     never discover each other.
#   --ipc host        lets Cyclone use shared memory between containers.
