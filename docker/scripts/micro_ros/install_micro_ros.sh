#!/bin/bash
# =============================================================
# micro-ROS Agent Installation Script
# =============================================================
# Installs and builds micro_ros_setup and the micro-ROS agent
# inside the Docker container.
#
# This script is called during Docker image build.
# =============================================================

set -e

echo "=========================================="
echo "  Installing micro-ROS Agent"
echo "=========================================="

# Source ROS2
source /opt/ros/${ROS_DISTRO}/setup.bash

# Create a workspace for micro-ROS
MICRO_ROS_WS=/home/${USERNAME}/micro_ros_ws
mkdir -p ${MICRO_ROS_WS}/src
cd ${MICRO_ROS_WS}

# Clone micro_ros_setup
git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies
sudo apt-get update
rosdep update && rosdep install --from-paths src --ignore-src -y

# Build micro_ros_setup
colcon build --packages-select micro_ros_setup
source install/setup.bash

# Create and build the agent workspace
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

# Final build
colcon build

echo "=========================================="
echo "  micro-ROS Agent installed successfully"
echo "=========================================="
