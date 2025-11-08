import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio
import math

class DiffDriveNode(Node):
    def __init__(self):
        super().__init__('diff_drive_node')

        # ROS 2 Parameters
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('wheel_base', 0.16)     # meters
        self.declare_parameter('left_motor_in1', 17)
        self.declare_parameter('left_motor_in2', 18)
        self.declare_parameter('left_motor_en', 12)
        self.declare_parameter('right_motor_in1', 22)
        self.declare_parameter('right_motor_in2', 23)
        self.declare_parameter('right_motor_en', 13)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.left_motor_in1 = self.get_parameter('left_motor_in1').value
        self.left_motor_in2 = self.get_parameter('left_motor_in2').value
        self.left_motor_en = self.get_parameter('left_motor_en').value
        self.right_motor_in1 = self.get_parameter('right_motor_in1').value
        self.right_motor_in2 = self.get_parameter('right_motor_in2').value
        self.right_motor_en = self.get_parameter('right_motor_en').value

        # Setup pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpiod.")
            rclpy.shutdown()
            return

        # Set GPIO modes
        self.pi.set_mode(self.left_motor_in1, pigpio.OUTPUT)
        self.pi.set_mode(self.left_motor_in2, pigpio.OUTPUT)
        self.pi.set_mode(self.left_motor_en, pigpio.OUTPUT)
        self.pi.set_mode(self.right_motor_in1, pigpio.OUTPUT)
        self.pi.set_mode(self.right_motor_in2, pigpio.OUTPUT)
        self.pi.set_mode(self.right_motor_en, pigpio.OUTPUT)

        # Initialize motors to be stopped
        self.stop_motors()

        # Subscription to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        self.subscription

    def cmd_vel_callback(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Differential drive kinematics
        left_wheel_velocity = (linear_x - (angular_z * self.wheel_base / 2.0)) / self.wheel_radius
        right_wheel_velocity = (linear_x + (angular_z * self.wheel_base / 2.0)) / self.wheel_radius

        # This is a simplified conversion, assuming max wheel velocity is around 2 rad/s which maps to 255 PWM
        max_rad_s = 2.0
        left_pwm = int((left_wheel_velocity / max_rad_s) * 255)
        right_pwm = int((right_wheel_velocity / max_rad_s) * 255)

        self.set_motor_speed(self.left_motor_in1, self.left_motor_in2, self.left_motor_en, left_pwm)
        self.set_motor_speed(self.right_motor_in1, self.right_motor_in2, self.right_motor_en, right_pwm)

    def set_motor_speed(self, in1, in2, en, pwm):
        pwm = max(min(pwm, 255), -255)

        if pwm > 0:
            self.pi.write(in1, 1)
            self.pi.write(in2, 0)
            self.pi.set_PWM_dutycycle(en, pwm)
        elif pwm < 0:
            self.pi.write(in1, 0)
            self.pi.write(in2, 1)
            self.pi.set_PWM_dutycycle(en, abs(pwm))
        else:
            self.pi.write(in1, 0)
            self.pi.write(in2, 0)
            self.pi.set_PWM_dutycycle(en, 0)

    def stop_motors(self):
        self.set_motor_speed(self.left_motor_in1, self.left_motor_in2, self.left_motor_en, 0)
        self.set_motor_speed(self.right_motor_in1, self.right_motor_in2, self.right_motor_en, 0)

    def on_shutdown(self):
        self.get_logger().info("Shutting down diff_drive_node, stopping motors.")
        self.stop_motors()
        self.pi.stop()

def main(args=None):
    rclpy.init(args=args)
    diff_drive_node = DiffDriveNode()
    try:
        rclpy.spin(diff_drive_node)
    except KeyboardInterrupt:
        pass
    finally:
        diff_drive_node.on_shutdown()
        diff_drive_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
