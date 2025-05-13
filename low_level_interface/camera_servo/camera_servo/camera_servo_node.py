#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import socket
import json
import time


class ServoCommand:
    """ 
    @class ServoCommand
    @brief Handles the creation of servo commands.
    """
    @staticmethod
    def create_command(servo_id: int, angle: int) -> dict:
        """
        @brief Creates a command dictionary for a servo.
        @param servo_id The ID of the servo (e.g., 1 or 2).
        @param angle The angle to set the servo to (0-180 degrees).
        @return A dictionary representing the servo command.
        """
        return {
            "N": 5,
            "D1": servo_id,
            "D2": angle
        }


class RobotCommunicator:
    """ 
    @class RobotCommunicator
    @brief Handles communication with the robot over a TCP connection.
    """
    def __init__(self, robot_ip: str, robot_port: int, timeout: float = 2.0):
        """
        @brief Initializes the RobotCommunicator.
        @param robot_ip The IP address of the robot.
        @param robot_port The port number of the robot.
        @param timeout The timeout for the TCP connection in seconds.
        """
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.timeout = timeout

    def send_command(self, command_dict: dict, logger):
        """
        @brief Sends a command to the robot via TCP.
        @param command_dict The command dictionary to send.
        @param logger The logger instance for logging messages.
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
    """ 
    @class CameraServoNode
    @brief ROS2 Node for controlling camera servos.
    """
    def __init__(self, robot_communicator: RobotCommunicator):
        """
        @brief Initializes the CameraServoNode.
        @param robot_communicator An instance of RobotCommunicator for sending commands.
        """
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
        @brief Callback function for processing received servo angles.
        @param msg A Vector3 message containing the servo angles:
                   - msg.x: tilt angle for servo #1 (0-180 degrees).
                   - msg.y: pan angle for servo #2 (0-180 degrees).
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
        """
        @brief Clamps the angle to the range 0-180 degrees.
        @param angle The angle to clamp.
        @return The clamped angle.
        """
        return max(0, min(180, angle))


def main(args=None):
    """
    @brief Main function to initialize and run the CameraServoNode.
    @param args Command-line arguments (optional).
    """
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
