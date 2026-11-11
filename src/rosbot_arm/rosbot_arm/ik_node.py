import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time
from time import sleep


from rosbot_msgs.msg import InverseMessage, ForwardPositions, GripperControl
from rosbot_msgs.action import ArmAction
import numpy as np

##  Joint Links ##
# l0 refers to the height of the arm base wrt base_link
l0 = 0.05 + 0.0654868 + 0.0338648 + 0.0772047
l1 = 0.130
l2 = 0.130
l3 = 0.055
tool_link = 0.117

lengths = [l0, l1, l2, l3+tool_link, 0]


arm_ids = [1, 2, 3, 4, 5]

# Only joint 2 is limited to protect the screen used to check at end
min_angles = [-120, -90, -120, -120, -120]
max_angles = [120, 66, 120, 120, 120]
min_pulses = [0, 125, 0, 0, 0]
max_pulses = [1000, 775, 1000, 1000, 1000]
angle_range = [-120, 120]
pulse_range = [0,1000]

# All joints operate in the range -120 to 120 with a pulse width range of 0 to 1000

# Stow Positions
arm_stow_msg = InverseMessage()
arm_stow_msg.x = 0.1
arm_stow_msg.y = 0.0
arm_stow_msg.z = 0.2
arm_stow_msg.roll = 0.0
arm_stow_msg.pitch = -60.0

# Gripper Close
gripper_close_msg = GripperControl()
gripper_close_msg.pulse = 500

# Gripper Open
gripper_open_msg = GripperControl()
gripper_open_msg.pulse = 100

# TODO: Write helper scripts to convert from one to the other.



def angle_to_pulse(angles):
    # TODO
    pulses = [np.interp(angles[i], angle_range, pulse_range).astype(int) for i in range(5)]
    return pulses

def pulse_to_angle(pulses): # Nothing uses this, idk why its here
    # TODO
    angles = [np.interp(pulses[i], pulse_range, angle_range).astype(int) for i in range(5)]
    return angles

class IKNode(Node):
    def __init__(self):
        super().__init__('ik_node')
        group = ReentrantCallbackGroup()
        self.get_logger().info('Inverse Kinematics Running')
        self.inverse_kinematics_sub = self.create_subscription(InverseMessage, 'inverse_kinematics_topic', self.inverse_kinematics_callback, 10, callback_group = group)
        self.forward_pub = self.create_publisher(ForwardPositions, 'forward_positions_topic', 10)
        self.gripper_pub = self.create_publisher(GripperControl, 'gripper_control_topic', 10)
        # TODO: make a action msg
        self.arm_serv = ActionServer(self, 
                                 ArmAction, 
                                 'arm_action_topic', 
                                 self.action_callback, 
                                 goal_callback=self.goal_callback,
                                 cancel_callback=self.cancel_callback,
                                 callback_group=group)
        self.get_logger().info('Server Running')

    def inverse_kinematics_callback(self, msg):
        try:
            print("---------- Inverse Kinematics Callback -----------")

            roll_rad = np.radians(msg.roll)
            pitch_rad = np.radians(msg.pitch)
            # angles 1 and 5
            angles = [ 0,0,0,0, roll_rad]
            angles[0] = np.arctan2(msg.y,msg.x)

            # angles 2 and 3
            r = np.hypot(msg.x, msg.y)
            r3 = r - (l3+tool_link)*np.cos(pitch_rad)
            z3 = (msg.z-l0) - (l3+tool_link)*np.sin(pitch_rad)
            le3 = np.hypot(r3, z3)

            angles[2] = -2*np.arctan(np.sqrt( ((l1+l2)**2 - le3**2) / (le3**2 - (l1-l2)**2) ))
            angles[1] =  np.arctan2(z3, r3) - np.arctan2(l2*np.sin(angles[2]), l1 + l2*np.cos(angles[2])) # - np.pi/2 moved offset to after joint 4

            # angle 4
            angles[3] = pitch_rad - angles[1] - angles[2]
            angles[1] =  angles[1] - np.pi/2 # subtract offset after being used for joint 4 ^

            forward_msg = ForwardPositions()
            forward_msg.id = [t for t in arm_ids]
            angles_deg = [np.degrees(t) for t in angles]
            forward_msg.position = [int(t) for t in angle_to_pulse(angles_deg)] # truncated
        

            print("-"*50)
            # Should error out if pulses out of bounds and prevent sending to arm_control
            in_bounds = True
            for i in range(5):
                if forward_msg.position[i] > max_pulses[i] or forward_msg.position[i] < min_pulses[i]:
                    self.get_logger().error(f"Joint {forward_msg.id[i]} is out of bounds") # Someone please change this to print what the index and pulse, IDK how
                    in_bounds = False
            if in_bounds:
                self.get_logger().info(f"Forwarded")
                self.forward_pub.publish(forward_msg)
        except:
            self.get_logger().error(f"Error")

    # Copied from demo code
    def goal_callback(self, goal_request):
        # OPTIONAL but good practice: Accept or reject a client request to begin an action
        return GoalResponse.ACCEPT
        x = goal_request.x
        y = goal_request.y
        z = goal_request.z
        self.get_logger().info(f'Received goal request, ({x},{y})')
        if (x > 11 or y > 11 or x < 0 or y < 0):
            self.get_logger().warn('Target (x, y) should be between 0 and 11')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def action_callback(self, goal_msg):
        
        arm_goal = InverseMessage()
        arm_goal.x = goal_msg.request.x
        arm_goal.y = goal_msg.request.y
        arm_goal.z = goal_msg.request.z
        arm_goal.roll = 0.0
        arm_goal.pitch = -90.0
        self.get_logger().info(f'Received goal request, ({arm_goal.x},{arm_goal.y},{arm_goal.z})')


        if goal_msg.request.pick:
            self.gripper_pub.publish(gripper_open_msg)
            sleep(2)
            self.inverse_kinematics_callback(arm_goal)
            sleep(2)
            self.gripper_pub.publish(gripper_close_msg)
            sleep(2)
            self.inverse_kinematics_callback(arm_stow_msg)
            sleep(2)
        else:
            self.inverse_kinematics_callback(arm_goal)
            sleep(2)
            self.gripper_pub.publish(gripper_open_msg)
            sleep(2)
            self.inverse_kinematics_callback(arm_stow_msg)
            sleep(2)

        result_msg = ArmAction.Result()
        result_msg.status = True

        return result_msg

        

    # TODO: Write inverse kinematics node
    

def main():
    rclpy.init()
    ik_node = IKNode()
    executor = MultiThreadedExecutor()
    rclpy.spin(ik_node, executor = executor)
    ik_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
