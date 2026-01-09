#!/usr/bin/env python3
"""
Keyboard teleop node for UGV Rover
Provides incremental velocity control via keyboard for precise movement during mapping
"""

import os
import select
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Default velocity limits (can be overridden by parameters)
DEFAULT_MAX_LIN_VEL = 0.5   # m/s
DEFAULT_MAX_ANG_VEL = 2.0   # rad/s

# Step sizes for incremental control
LIN_VEL_STEP_SIZE = 0.02    # 2 cm/s per keypress (smaller steps for SLAM)
ANG_VEL_STEP_SIZE = 0.1     # 0.1 rad/s per keypress

msg = """
Control Your UGV Rover!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity

space key, s : force stop

CTRL-C to quit

Current limits: Linear = {:.2f} m/s, Angular = {:.2f} rad/s
"""

e = """
Communications Failed
"""


def get_key(settings):
    """Get a single keypress from the terminal"""
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    """Display current velocity targets"""
    print('currently:\tlinear velocity {0:.3f}\t angular velocity {1:.3f} '.format(
        target_linear_velocity,
        target_angular_velocity))


def make_simple_profile(output_vel, input_vel, slop):
    """
    Create a velocity profile with smooth acceleration/deceleration
    This prevents jerky movements that are bad for odometry and SLAM
    """
    if input_vel > output_vel:
        output_vel = min(input_vel, output_vel + slop)
    elif input_vel < output_vel:
        output_vel = max(input_vel, output_vel - slop)
    else:
        output_vel = input_vel
    return output_vel


def constrain(input_vel, low_bound, high_bound):
    """Clamp velocity to safe limits"""
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    return input_vel


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Declare parameters
        self.declare_parameter('max_linear_speed', DEFAULT_MAX_LIN_VEL)
        self.declare_parameter('max_angular_speed', DEFAULT_MAX_ANG_VEL)
        self.declare_parameter('linear_step_size', LIN_VEL_STEP_SIZE)
        self.declare_parameter('angular_step_size', ANG_VEL_STEP_SIZE)
        
        # Get parameters
        self.max_linear_vel = self.get_parameter('max_linear_speed').get_parameter_value().double_value
        self.max_angular_vel = self.get_parameter('max_angular_speed').get_parameter_value().double_value
        self.linear_step = self.get_parameter('linear_step_size').get_parameter_value().double_value
        self.angular_step = self.get_parameter('angular_step_size').get_parameter_value().double_value
        
        # Log parameters
        self.get_logger().info('Keyboard Teleop Parameters:')
        self.get_logger().info(f'  Max linear speed: {self.max_linear_vel} m/s')
        self.get_logger().info(f'  Max angular speed: {self.max_angular_vel} rad/s')
        self.get_logger().info(f'  Linear step size: {self.linear_step} m/s')
        self.get_logger().info(f'  Angular step size: {self.angular_step} rad/s')
        
        # Create publisher
        qos = QoSProfile(depth=10)
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', qos)
        
        # Velocity state
        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        
        self.get_logger().info("Keyboard teleop ready!")

    def check_linear_limit_velocity(self, velocity):
        """Constrain linear velocity to safe limits"""
        return constrain(velocity, -self.max_linear_vel, self.max_linear_vel)

    def check_angular_limit_velocity(self, velocity):
        """Constrain angular velocity to safe limits"""
        return constrain(velocity, -self.max_angular_vel, self.max_angular_vel)

    def run(self):
        """Main control loop"""
        settings = None
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        status = 0

        try:
            print(msg.format(self.max_linear_vel, self.max_angular_vel))
            while rclpy.ok():
                key = get_key(settings)
                
                if key == 'w':
                    self.target_linear_velocity = self.check_linear_limit_velocity(
                        self.target_linear_velocity + self.linear_step)
                    status = status + 1
                    print_vels(self.target_linear_velocity, self.target_angular_velocity)
                    
                elif key == 'x':
                    self.target_linear_velocity = self.check_linear_limit_velocity(
                        self.target_linear_velocity - self.linear_step)
                    status = status + 1
                    print_vels(self.target_linear_velocity, self.target_angular_velocity)
                    
                elif key == 'a':
                    self.target_angular_velocity = self.check_angular_limit_velocity(
                        self.target_angular_velocity + self.angular_step)
                    status = status + 1
                    print_vels(self.target_linear_velocity, self.target_angular_velocity)
                    
                elif key == 'd':
                    self.target_angular_velocity = self.check_angular_limit_velocity(
                        self.target_angular_velocity - self.angular_step)
                    status = status + 1
                    print_vels(self.target_linear_velocity, self.target_angular_velocity)
                    
                elif key == ' ' or key == 's':
                    self.target_linear_velocity = 0.0
                    self.control_linear_velocity = 0.0
                    self.target_angular_velocity = 0.0
                    self.control_angular_velocity = 0.0
                    print_vels(self.target_linear_velocity, self.target_angular_velocity)
                    
                elif key == '\x03':  # CTRL-C
                    break

                if status == 20:
                    print(msg.format(self.max_linear_vel, self.max_angular_vel))
                    status = 0

                # Apply smooth acceleration profile
                self.control_linear_velocity = make_simple_profile(
                    self.control_linear_velocity,
                    self.target_linear_velocity,
                    (self.linear_step / 2.0))

                self.control_angular_velocity = make_simple_profile(
                    self.control_angular_velocity,
                    self.target_angular_velocity,
                    (self.angular_step / 2.0))

                # Create and publish Twist message
                twist = Twist()
                twist.linear.x = self.control_linear_velocity
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = self.control_angular_velocity

                self.pub_cmd_vel.publish(twist)

        except Exception as e:
            self.get_logger().error(f'Exception: {e}')

        finally:
            # Send stop command
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)

            if os.name != 'nt':
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()