import rclpy
from rclpy.node import Node
from ros_robot_controller.ros_robot_controller_sdk import Board
from rosbot_msgs.msg import ForwardPositions, GripperControl
from rosbot_msgs.srv import InverseKinematics


class ArmControl(Node):
    def __init__(self):
        super().__init__('arm_control')
        self.board = Board()
        self.board.enable_reception()
        # TODO: Subscribe to a forward control topic
        self.forward_control_sub = self.create_subscription(ForwardPositions, 'forward_positions_topic', self.forward_positions_callback, 10)
        # TODO: Subscribe to a gripper control topic
        self.gripper_control_sub = self.create_subscription(GripperControl, 'gripper_control_topic', self.gripper_control_callback, 10)
    
    duration = 1.0 # Time to complete motion

    

    def forward_positions_callback(self, msg):

        self.board.bus_servo_set_position(self.duration, list(zip(msg.id, msg.position)) )
        for i in [1, 2, 3, 4, 5, 10]:
            print(i, ":\t", self.board.bus_servo_read_position(i))



    def gripper_control_callback(self, msg):
        if msg.pulse < 650 and msg.pulse > 0
            self.board.bus_servo_set_position(self.duration, [ 10, msg.pulse] )

    # TODO: Write an arm_control node to receive joint angles
    # and send messages to the Board.

    # See arm_test for example of board functionality.


def main():
    rclpy.init()
    node = ArmControl()
    rclpy.spin(node)
    rclpy.shutdown()
    

if __name__=="__main__":
    main()
