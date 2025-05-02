#!/usr/bin/env bash
set -e

# Source the ROS 2 setup
source /opt/ros/humble/setup.bash
# Source your workspace install
source /app/install/setup.bash

# Execute the given command
exec "$@"
