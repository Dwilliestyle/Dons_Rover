#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import serial
import json
import socket
import time
from datetime import datetime


class OLEDDisplay(Node):
    def __init__(self):
        super().__init__('oled_display')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('cmd_timeout', 0.5)  # seconds
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        
        # Initialize serial connection to ESP32
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'Connected to ESP32 on {serial_port} for OLED control')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
        
        # Display data
        self.ip_address = self.get_ip_address()
        self.battery_voltage = None
        self.robot_status = "Stopped"
        self.last_cmd_time = time.time()
        
        # Create subscriptions
        self.battery_sub = self.create_subscription(
            Float32,
            'battery_voltage',
            self.battery_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create timer to update display
        timer_period = 1.0 / update_rate
        self.display_timer = self.create_timer(timer_period, self.update_display)
        self.status_timer = self.create_timer(0.1, self.check_status)
        
        self.get_logger().info('OLED display node started')
    
    def get_ip_address(self):
        """Get the robot's IP address"""
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "No Network"
    
    def battery_callback(self, msg):
        """Receive battery voltage updates"""
        self.battery_voltage = msg.data
    
    def cmd_vel_callback(self, msg):
        """Monitor cmd_vel to determine robot status"""
        self.last_cmd_time = time.time()
        
        linear = abs(msg.linear.x)
        angular = abs(msg.angular.z)
        
        # Determine robot status
        if linear < 0.01 and angular < 0.01:
            self.robot_status = "Stopped"
        elif angular > 0.1:
            self.robot_status = "Turning"
        else:
            self.robot_status = "Moving"
    
    def check_status(self):
        """Check if robot has timed out (no recent commands)"""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            if self.robot_status != "Stopped":
                self.robot_status = "Stopped"
    
    def send_oled_command(self, line_num, text):
        """Send OLED update command to ESP32"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        
        try:
            command = {
                "T": 3,
                "lineNum": line_num,
                "Text": str(text)
            }
            cmd_str = json.dumps(command) + '\n'
            self.serial_conn.write(cmd_str.encode('utf-8'))
            self.get_logger().debug(f'Updated OLED line {line_num}: {text}')
        except Exception as e:
            self.get_logger().error(f'Failed to send OLED command: {e}')
    
    def update_display(self):
        """Update all OLED display lines"""
        try:
            # Line 0: IP Address
            self.send_oled_command(0, f"IP: {self.ip_address}")
            
            # Line 1: Current time
            current_time = datetime.now().strftime('%H:%M:%S')
            self.send_oled_command(1, f"Time: {current_time}")
            
            # Line 2: Robot status
            self.send_oled_command(2, f"Status: {self.robot_status}")
            
            # Line 3: Battery voltage with warning indicators
            if self.battery_voltage is not None:
                if self.battery_voltage < 9.5:
                    # Critical - show with emphasis
                    self.send_oled_command(3, f"BATT!: {self.battery_voltage:.2f}V!")
                elif self.battery_voltage < 10.0:
                    # Warning - show with single exclamation
                    self.send_oled_command(3, f"Batt!: {self.battery_voltage:.2f}V")
                else:
                    # Normal
                    self.send_oled_command(3, f"Battery: {self.battery_voltage:.2f}V")
            else:
                self.send_oled_command(3, "Battery: --.--V")
                
        except Exception as e:
            self.get_logger().error(f'OLED update error: {e}')
    
    def __del__(self):
        """Cleanup serial connection on shutdown"""
        if hasattr(self, 'serial_conn') and self.serial_conn:
            try:
                self.serial_conn.close()
                self.get_logger().info('OLED serial connection closed')
            except:
                pass


def main(args=None):
    rclpy.init(args=args)
    oled_display = OLEDDisplay()
    
    try:
        rclpy.spin(oled_display)
    except KeyboardInterrupt:
        pass
    finally:
        oled_display.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()