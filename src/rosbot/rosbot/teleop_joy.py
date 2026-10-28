

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class teleopJoy(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.pub = self.create_publisher(Twist,'cmd_vel',10)
        self.sub = self.create_subscription(Joy,'ros_robot_controller/joy',self.update,10)
    def update(self, joystick):
        twist = Twist()
        twist.linear.x = joystick.axes[1] * 0.5
        twist.angular.z = joystick.axes[0] * 1.0
        self.pub.publish(twist)

def main():
    print('Hi from Joystick bot')
    rclpy.init()
    teleop_Joy = teleopJoy()
    rclpy.spin(teleop_Joy)
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
