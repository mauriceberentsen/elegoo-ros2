#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import socket
import json
import time


class ServoCommand:
    """Handles the creation of servo commands."""
    @staticmethod
    def create_command(servo_id: int, angle: int) -> dict:
        return {
            "N": 5,
            "D1": servo_id,
            "D2": angle
        }


class RobotCommunicator:
    """Handles communication with the robot."""
    def __init__(self, robot_ip: str, robot_port: int, timeout: float = 2.0):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.timeout = timeout

    def send_command(self, command_dict: dict, logger):
        """
        Opens a TCP connection to the robot, sends a single JSON command, then closes.
        """
        data_str = json.dumps(command_dict)

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(self.timeout)
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(data_str.encode('utf-8'))

                response = s.recv(128)
                if response:
                    logger.info(f"Robot response: {response.decode('utf-8', errors='ignore')}")
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            logger.error(f"Socket error sending command {command_dict}: {e}")


class CameraServoNode(Node):
    """ROS2 Node for controlling camera servos."""
    def __init__(self, robot_communicator: RobotCommunicator):
        super().__init__('camera_servo_node')
        self.robot_communicator = robot_communicator

        self.subscription = self.create_subscription(
            Vector3,
            'camera_servo_angles',
            self.servo_cmd_callback,
            10
        )
        self.get_logger().info("CameraServoNode initialized. Listening for servo angles...")

    def servo_cmd_callback(self, msg: Vector3):
        """
        Callback whenever we receive a Vector3:
          msg.x => tilt angle for servo #1 (0..180)
          msg.y => pan angle for servo #2 (0..180)
        """
        tilt_angle = self._clamp_angle(int(msg.x))
        pan_angle = self._clamp_angle(int(msg.y))

        self.get_logger().info(f"Received angles: tilt={tilt_angle}, pan={pan_angle}")

        # Create and send commands for both servos
        servo1_cmd = ServoCommand.create_command(servo_id=1, angle=tilt_angle)
        servo2_cmd = ServoCommand.create_command(servo_id=2, angle=pan_angle)

        self.robot_communicator.send_command(servo1_cmd, self.get_logger())
        time.sleep(0.1)
        self.robot_communicator.send_command(servo2_cmd, self.get_logger())

    @staticmethod
    def _clamp_angle(angle: int) -> int:
        """Clamps the angle to the range 0..180."""
        return max(0, min(180, angle))


def main(args=None):
    rclpy.init(args=args)

    # Dependency injection for RobotCommunicator
    robot_communicator = RobotCommunicator(robot_ip='192.168.4.1', robot_port=100)
    node = CameraServoNode(robot_communicator)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
