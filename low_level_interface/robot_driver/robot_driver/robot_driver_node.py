#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, String, Empty
import socket
import json
import time

class RobotDriverNode(Node):
    def __init__(self):
        super().__init__('robot_driver_node')
        
        # Parameters for robot connection
        self.declare_parameter('robot_ip', '192.168.4.1')
        self.declare_parameter('robot_port', 100)
        self.robot_ip = self.get_parameter('robot_ip').value
        self.robot_port = self.get_parameter('robot_port').value
        
        # Max speed parameters
        self.declare_parameter('max_linear_speed', 250)  # Max speed in protocol is 250
        self.declare_parameter('max_angular_speed', 250)
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        
        # Create subscribers
        # 1. For standard ROS2 Twist messages (from teleop or navigation)
        self.twist_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.twist_callback,
            10
        )
        
        # 2. For direct motor control (useful for testing or direct control)
        self.motor_subscription = self.create_subscription(
            Int32MultiArray,
            'motor_control',
            self.motor_control_callback,
            10
        )
        
        # 3. For auto movement commands (N=2 and N=3)
        self.auto_subscription = self.create_subscription(
            String,
            'auto_command',
            self.auto_command_callback,
            10
        )
        
        # 4. For emergency stop (N=100)
        self.stop_subscription = self.create_subscription(
            Empty,
            'stop',
            self.stop_callback,
            10
        )
        
        # 5. For stop and program mode (N=110)
        self.program_subscription = self.create_subscription(
            Empty,
            'program_mode',
            self.program_mode_callback,
            10
        )
        
        self.get_logger().info("RobotDriverNode initialized. Listening for motor commands...")

    def twist_callback(self, msg: Twist):
        """
        Callback for Twist messages. Converts ROS2 Twist messages to protocol commands.
        Twist.linear.x: Forward/backward speed (-1.0 to 1.0)
        Twist.angular.z: Angular velocity / turning (-1.0 to 1.0)
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to motor speeds using differential drive kinematics
        left_speed = int((linear_x - angular_z * 0.5) * self.max_linear_speed)
        right_speed = int((linear_x + angular_z * 0.5) * self.max_linear_speed)
        
        # Clamp speeds to valid range
        left_speed = max(-250, min(250, left_speed))
        right_speed = max(-250, min(250, right_speed))
        
        # If speeds are very close to zero, stop motors
        if abs(left_speed) < 10:
            left_speed = 0
        if abs(right_speed) < 10:
            right_speed = 0
        
        # Use N=4 for independent motor control
        left_speed_abs = abs(left_speed)
        right_speed_abs = abs(right_speed)
        
        # Determine direction for each motor (0=stop, 1=forward, 2=backward)
        left_dir = 0 if left_speed == 0 else (1 if left_speed > 0 else 2)
        right_dir = 0 if right_speed == 0 else (1 if right_speed > 0 else 2)
        
        # If both motors should stop, send N=1 with direction=0
        if left_speed == 0 and right_speed == 0:
            cmd = {
                "N": 1,
                "D1": 0,  # Both motors
                "D2": 0,  # Speed
                "D3": 0   # Stop
            }
            self.send_command_to_robot(cmd)
        else:
            # For each motor individually, use N=1 with D1 set to which motor
            # First left motor
            left_cmd = {
                "N": 1,
                "D1": 1,  # Left motor
                "D2": left_speed_abs,  # Speed
                "D3": left_dir  # Direction
            }
            self.send_command_to_robot(left_cmd)
            
            # Small delay between commands
            time.sleep(0.05)
            
            # Then right motor
            right_cmd = {
                "N": 1,
                "D1": 2,  # Right motor
                "D2": right_speed_abs,  # Speed
                "D3": right_dir  # Direction
            }
            self.send_command_to_robot(right_cmd)

    def motor_control_callback(self, msg: Int32MultiArray):
        """
        Callback for direct motor control.
        Expects a message with:
            data[0]: Command type (1 for N=1 simple, 4 for N=4 differential)
            data[1]: First parameter (motor selection for N=1, left speed for N=4)
            data[2]: Second parameter (speed for N=1, right speed for N=4)
            data[3]: Third parameter (direction for N=1, not used for N=4)
        """
        if len(msg.data) < 2:
            self.get_logger().error("Motor control message too short")
            return
            
        cmd_type = msg.data[0]
        
        if cmd_type == 1 and len(msg.data) >= 4:
            # N=1: Simple motor control
            motor_select = msg.data[1]  # 0=both, 1=left, 2=right
            speed = msg.data[2]         # 0-250
            direction = msg.data[3]     # 0=stop, 1=forward, 2=backward
            
            # Validate parameters
            motor_select = max(0, min(2, motor_select))
            speed = max(0, min(250, speed))
            direction = max(0, min(2, direction))
            
            cmd = {
                "N": 1,
                "D1": motor_select,
                "D2": speed,
                "D3": direction
            }
            self.send_command_to_robot(cmd)
            
        elif cmd_type == 4 and len(msg.data) >= 3:
            # N=4: Speed mode motor control
            left_speed = msg.data[1]    # 0-255
            right_speed = msg.data[2]   # 0-255
            
            # Validate parameters
            left_speed = max(0, min(255, left_speed))
            right_speed = max(0, min(255, right_speed))
            
            cmd = {
                "N": 4,
                "D1": left_speed,
                "D2": right_speed
            }
            self.send_command_to_robot(cmd)
            
        elif cmd_type == 2 and len(msg.data) >= 4:
            # N=2: Auto movement with time limit
            direction = msg.data[1]     # 1=left forward, 2=right forward, 3=forward, 4=backward
            speed = msg.data[2]         # 0-255
            duration_ms = msg.data[3]   # Time in ms
            
            # Validate parameters
            direction = max(1, min(4, direction))
            speed = max(0, min(255, speed))
            duration_ms = max(0, min(10000, duration_ms))  # Cap at 10 seconds for safety
            
            cmd = {
                "N": 2,
                "D1": direction,
                "D2": speed,
                "T": duration_ms
            }
            self.send_command_to_robot(cmd)
            
        elif cmd_type == 3 and len(msg.data) >= 3:
            # N=3: Auto movement without time limit
            direction = msg.data[1]     # 1=left forward, 2=right forward, 3=forward, 4=backward
            speed = msg.data[2]         # 0-255
            
            # Validate parameters
            direction = max(1, min(4, direction))
            speed = max(0, min(255, speed))
            
            cmd = {
                "N": 3,
                "D1": direction,
                "D2": speed
            }
            self.send_command_to_robot(cmd)
            
        elif cmd_type == 100:
            # N=100: Stop all functions -> Standby mode
            self.send_stop_command()
            
        elif cmd_type == 110:
            # N=110: Stop all functions -> Programming mode
            self.send_program_mode_command()
            
        else:
            self.get_logger().error(f"Unknown motor command type: {cmd_type}")
    
    def auto_command_callback(self, msg: String):
        """
        Process auto movement commands as a string with format:
        "command_type:direction:speed[:duration]"
        Where:
            command_type: 2 for timed, 3 for continuous
            direction: 1=left forward, 2=right forward, 3=forward, 4=backward
            speed: 0-255
            duration: time in ms (only for command_type=2)
        
        Example: "2:3:150:3000" - Move forward at speed 150 for 3 seconds
                 "3:4:130" - Move backward at speed 130 continuously
        """
        try:
            parts = msg.data.strip().split(':')
            if len(parts) < 3:
                self.get_logger().error(f"Invalid auto command format: {msg.data}")
                return
                
            cmd_type = int(parts[0])
            direction = int(parts[1])
            speed = int(parts[2])
            
            # Validate parameters
            direction = max(1, min(4, direction))
            speed = max(0, min(255, speed))
            
            if cmd_type == 2:
                if len(parts) < 4:
                    self.get_logger().error("Missing duration for timed auto command")
                    return
                    
                duration_ms = int(parts[3])
                duration_ms = max(0, min(10000, duration_ms))  # Cap at 10 seconds for safety
                
                cmd = {
                    "N": 2,
                    "D1": direction,
                    "D2": speed,
                    "T": duration_ms
                }
                self.send_command_to_robot(cmd)
                
            elif cmd_type == 3:
                cmd = {
                    "N": 3,
                    "D1": direction,
                    "D2": speed
                }
                self.send_command_to_robot(cmd)
                
            else:
                self.get_logger().error(f"Unknown auto command type: {cmd_type}")
                
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parsing auto command: {e}")
    
    def stop_callback(self, msg: Empty):
        """
        Stop all functions and go to standby mode (N=100)
        """
        self.send_stop_command()
    
    def program_mode_callback(self, msg: Empty):
        """
        Stop all functions and go to programming mode (N=110)
        """
        self.send_program_mode_command()
    
    def send_stop_command(self):
        """
        Send N=100 command to stop all functions and go to standby mode
        """
        cmd = {"N": 100}
        self.get_logger().info("Sending STOP command (N=100)")
        self.send_command_to_robot(cmd)
    
    def send_program_mode_command(self):
        """
        Send N=110 command to stop all functions and go to programming mode
        """
        cmd = {"N": 110}
        self.get_logger().info("Sending PROGRAM MODE command (N=110)")
        self.send_command_to_robot(cmd)

    def send_command_to_robot(self, command_dict: dict):
        """
        Opens a TCP connection to the robot, sends a single JSON command, then closes.
        """
        # Convert dict to JSON string
        data_str = json.dumps(command_dict)
        
        # Log the command being sent
        self.get_logger().info(f"Sending command: {data_str}")

        # Connect to the robot
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2.0)  # 2s timeout, adjust as needed
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(data_str.encode('utf-8'))
                
                # Optionally read any response
                response = s.recv(128)
                if response:
                    self.get_logger().info(f"Robot response: {response.decode('utf-8', errors='ignore')}")
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            self.get_logger().error(f"Socket error sending command {command_dict}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure motors are stopped before shutting down
        try:
            stop_cmd = {"N": 100}  # Use full stop command instead of just motors
            node.send_command_to_robot(stop_cmd)
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
