import rclpy
from rclpy.node import Node

from rosbot_msgs.msg import InverseMessage, ForwardPositions
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
        self.get_logger().info('Inverse Kinematics Running')
        self.inverse_kinematics_sub = self.create_subscription(InverseMessage, 'inverse_kinematics_topic', self.inverse_kinematics_callback, 10)
        self.forward_pub = self.create_publisher(ForwardPositions, 'forward_positions_topic', 10)
    

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
                self.get_logger().info('Success')
                self.forward_pub.publish(forward_msg)
        except:
            self.get_logger().error(f"Error")
            
        

    # TODO: Write inverse kinematics node
    

def main():
    rclpy.init()
    ik_node = IKNode()
    rclpy.spin(ik_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
