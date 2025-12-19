#!/bin/bash
# This script sets up the ROS2 environment
# IMPORTANT: Use "source ./e.sh" or ". ./e.sh" instead of "./e.sh"
# to make the environment changes persist in your current shell
source /opt/ros/humble/setup.sh
# Source profile for camera if exists
if [ -f ~/.profile ]; then
  source ~/.profile
  echo ✅ Camera environment sourced
fi
# Source local setup if exists
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
if [ -f "$SCRIPT_DIR/install/local_setup.sh" ]; then
  source "$SCRIPT_DIR/install/local_setup.sh"
  echo ✅ Local setup sourced
fi
