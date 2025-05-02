#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3

import socket
import json

class CameraServoNode(Node):
    def __init__(self):
        super().__init__('camera_servo_node')
        # Declare or get parameters for robot IP/port if you wish
        self.robot_ip = '192.168.4.1'
        self.robot_port = 100

        # Create subscriber to a topic that publishes two angles: tilt (x), pan (y)
        self.subscription = self.create_subscription(
            Vector3,
            'camera_servo_angles',
            self.servo_cmd_callback,
            10
        )
        self.get_logger().info("CameraServoNode initialized. Listening for servo angles...")

    def servo_cmd_callback(self, msg: Vector3):
        """
        Callback whenever we receive a Vector2:
          msg.x => tilt angle for servo #1 (0..180)
          msg.y => pan angle for servo #2 (0..180)
        """
        tilt_angle = int(msg.x)
        pan_angle  = int(msg.y)

        # Clamp angles to 0..180 for safety
        tilt_angle = max(0, min(180, tilt_angle))
        pan_angle  = max(0, min(180, pan_angle))

        self.get_logger().info(f"Received angles: tilt={tilt_angle}, pan={pan_angle}")

        # First command: "N=5" for servo #1 (up/down)
        #   D1=1 => servo #1, D2 => angle
        servo1_cmd = {
            "N": 5,
            "D1": 1,
            "D2": tilt_angle
        }
        # Second command: "N=5" for servo #2 (left/right)
        #   D1=2 => servo #2, D2 => angle
        servo2_cmd = {
            "N": 5,
            "D1": 2,
            "D2": pan_angle
        }

        # Send each JSON command over TCP
        self.send_command_to_robot(servo1_cmd)
        time.sleep(0.1)
        self.send_command_to_robot(servo2_cmd)

    def send_command_to_robot(self, command_dict: dict):
        """
        Opens a TCP connection to the robot, sends a single JSON command, then closes.
        """
        # Convert dict to JSON string
        data_str = json.dumps(command_dict)

        # Connect to the robot
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2.0)  # 2s timeout, adjust as needed
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(data_str.encode('utf-8'))
                
                # Optionally read any response (often the code doesn't send much back)
                # We can read until no more data or up to some newline
                # Here we just do a small recv attempt
                response = s.recv(128)
                if response:
                    self.get_logger().info(f"Robot response: {response.decode('utf-8', errors='ignore')}")
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            self.get_logger().error(f"Socket error sending command {command_dict}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
