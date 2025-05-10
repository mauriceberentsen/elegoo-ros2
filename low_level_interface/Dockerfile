# Gebruik een minimaal ROS 2 Humble-image (gebaseerd op Ubuntu 22.04).
FROM ros:humble-ros-core
# We hebben bash nodig voor sourcing, dus stel de standaardshell in en vermijd interactieve installaties
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
# Installeer colcon en eventuele afhankelijkheden die je pakket nodig heeft
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*
# Maak een directory voor je werkruimte
WORKDIR /app
# Kopieer de _source_ van je pakket naar /app/src
# (gebruik .dockerignore om build/install/log uit te sluiten)
COPY . /app/src
# Sourcen van ROS 2, bouw de werkruimte
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install
# Optioneel: Ingangspunt dat altijd de omgeving sourced
# zodat `docker run <image>` ROS 2 klaar heeft
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
# Standaard naar camera_servo node, maar kan worden overschreven met omgevingsvariabelen
# Voorbeeld: docker run -e ROS_PKG=robot_driver -e ROS_EXEC=robot_driver_node myimage
ENV ROS_PKG=camera_servo
ENV ROS_EXEC=camera_servo_node
CMD ros2 run $ROS_PKG $ROS_EXEC
