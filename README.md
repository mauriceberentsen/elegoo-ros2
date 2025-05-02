# Elegoo Robot ROS2 Interface

This package provides a low-level interface to control the Elegoo robot using ROS2 (Robot Operating System 2).

## Packages

- **camera_servo**: Controls the camera servos (pan and tilt)
- **robot_driver**: Controls the robot's motors and movement

## Quick Start with Docker

1. Build the Docker image:

```bash
docker build -t elegoo_ros_interface ./low_level_interface
```

2. Run the Docker container with the desired node:

```bash
# To run the camera servo node
docker run -e ROS_PKG=camera_servo -e ROS_EXEC=camera_servo_node elegoo_ros_interface

# To run the robot driver node
docker run -e ROS_PKG=robot_driver -e ROS_EXEC=robot_driver_node elegoo_ros_interface
```

## Robot Driver Usage

### Controlling with Twist messages

```bash
# To drive forward at half speed
docker exec -it <container_id> ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# To turn in place (counter-clockwise)
docker exec -it <container_id> ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# To stop
docker exec -it <container_id> ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Direct Motor Control

```bash
# Using N=1: Drive both motors forward at speed 100
docker exec -it <container_id> ros2 topic pub /motor_control std_msgs/msg/Int32MultiArray "{data: [1, 0, 100, 1]}"

# Using N=4: Set left motor speed to 80, right motor to 200
docker exec -it <container_id> ros2 topic pub /motor_control std_msgs/msg/Int32MultiArray "{data: [4, 80, 200]}"
```

### Auto Movement Commands

```bash
# Using N=2: Move forward at speed 150 for 3 seconds
docker exec -it <container_id> ros2 topic pub /auto_command std_msgs/msg/String '{data: "2:3:150:3000"}'

# Using N=3: Move backward at speed 130 (continuous)
docker exec -it <container_id> ros2 topic pub /auto_command std_msgs/msg/String '{data: "3:4:130"}'
```

### Emergency Stop

```bash
# Send stop command (N=100)
docker exec -it <container_id> ros2 topic pub /stop std_msgs/msg/Empty "{}"

# Send program mode command (N=110)
docker exec -it <container_id> ros2 topic pub /program_mode std_msgs/msg/Empty "{}"
```

## Camera Servo Usage

```bash
# Set tilt to 90 degrees and pan to 45 degrees
docker exec -it <container_id> ros2 topic pub /camera_servo_angles geometry_msgs/msg/Vector3 "{x: 90.0, y: 45.0, z: 0.0}"
```

## Protocol Documentation

See the `protocol.md` file for the full command protocol documentation.

## Development

### Building Locally with Docker

```bash
# Build using docker
docker build -t elegoo_ros_interface ./low_level_interface

# Run with interactive shell
docker run -it --rm elegoo_ros_interface bash
```

### Extending the Interface

To add support for additional protocol commands:

1. Identify the command in protocol.md
2. Add the appropriate subscriber in the relevant node
3. Implement the callback function to convert ROS messages to protocol commands
4. Add any necessary helper functions 