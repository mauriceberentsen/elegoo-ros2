# Use a minimal ROS 2 Humble image (Ubuntu 22.04-based).
FROM ros:humble-ros-core

# We'll need bash for sourcing, so set the default shell and avoid interactive installs
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Install colcon and any dependencies your package needs
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create a directory for your workspace
WORKDIR /app

# Copy your package *source* into /app/src
# (using .dockerignore to exclude build/install/log)
COPY . /app/src

# Source ROS 2, build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Optional: Entry point that always sources the environment
# so `docker run <image>` has ROS 2 ready
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default to camera_servo node, but can be overridden with environment variables
# Example: docker run -e ROS_PKG=robot_driver -e ROS_EXEC=robot_driver_node myimage
ENV ROS_PKG=camera_servo
ENV ROS_EXEC=camera_servo_node
CMD ros2 run $ROS_PKG $ROS_EXEC
