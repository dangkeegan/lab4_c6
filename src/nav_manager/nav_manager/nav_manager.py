#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from enum import Enum
from nav2_msgs.action import NavigateToPose
from rosbot_msgs.action import ArmAction

# An enum allows you to use words instead of arbitrary
# numbers to represent your states
class State(Enum):
    INIT = 1
    NAVIGATE = 2
    PICK = 3
    PLACE = 4
    # You can add other states to represent other behaviors
    DONE = 5 # Temporarily added NEXT as a state

class NavManager(Node):
    def __init__(self):
        super().__init__('nav_manager')
        self.state = State.INIT # Initialize internal variable to represent state
        self.get_logger().info('Nav Manager Running')
        # The timer calls the timer_callback function 10 times a second.
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Add subscribers, publishers, action clients, etc. in __init__
        self.navigation_action_client = ActionClient(self, NavigateToPose,'/navigate_to_pose')
        self.arm_action_client = ActionClient(self, ArmAction, 'arm_action_topic')
        # Can also declare some variables as parameters that can be called
        # from a launch file or the command line
        self.declare_parameter('goal_position', [0.0, 0.0, 0.0])
        self.declare_parameter('block_position', [0.2, 0.0, 0.02])

        self.goal_pos = self.get_parameter('goal_position').value
        self.block_pos = self.get_parameter('block_position').value

        # Keep track of whether bot is currently busy
        self.BUSY = False

        # Can use this finished variable to end the node once it is done
        self.finished = False

    # The timer callback handles all logic of what to do when
    def timer_callback(self):
        # Set state to BUSY while something is happening
        #self.get_logger().info(f'State: {self.state}')
        if self.BUSY:
            # If currently BUSY, do nothing
            return

        # Could just initialize your state to NAVIGATE
        # but this demonstrates state switching
        elif self.state is State.INIT:
            self.state = State.PICK
            self.BUSY = False

        elif self.state is State.NAVIGATE:
            self.send_nav_goal()
            self.BUSY = True

        elif self.state is State.PICK:
            self.send_arm_goal(True)
            self.BUSY = True
        
        elif self.state is State.PLACE:
            self.send_arm_goal(False)
            self.BUSY = True
        

        elif self.state is State.DONE:
            # Mark as finished and quit code

            self.finished = True
            

        # Include any other states in here that you wish
        # Timer callback should just handle switching between
        # other functionality

    def result_callback(self,result):
        self.BUSY = False

    def send_arm_goal(self, pick):
        self.get_logger().info(f'State: {self.state}')
        goal_msg = ArmAction.Goal()
        goal_msg.x = self.block_pos[0]
        goal_msg.y = self.block_pos[1]
        goal_msg.z = self.block_pos[2]
        goal_msg.pick = pick
        self.get_logger().info('Waiting for Server')
        self.arm_action_client.wait_for_server()
        future = self.arm_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

        self.get_logger().info(f'Sent Pick {goal_msg.pick} request')
        



        if self.state == State.PICK:
            self.state = State.NAVIGATE
        elif self.state == State.PLACE:
            self.state = State.DONE
        return future

    

    def goal_response_callback(self, future):
        goal_handle = future.result() # Accepted or rejected?
        # If rejected, the whole thing fails
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by server, stopping')
            self.done_future.set_result(False)
            return

        # Set up future to wait for the results
        result_future = goal_handle.get_result_async()
        # What do we do when we get the results? result_callback 
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        self.BUSY = False

    def send_nav_goal(self):
        self.get_logger().info(f'State: {self.state}')
        # Send goal to navigation action server in this function
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x =  self.goal_pos[0]
        goal_msg.pose.pose.position.y =  self.goal_pos[1]
        goal_msg.pose.pose.orientation.w = self.goal_pos[2]

        self.get_logger().info('Waiting for Server')
        self.navigation_action_client.wait_for_server()
        future = self.navigation_action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        self.state = State.PLACE
        
        return future
        # After navigation to target, set the state to whatever comes next
        # Manager may not be done after sending nav goal 
        
    def feedback_callback(self):
        return None

def main():
    rclpy.init()
    node = NavManager()
    while rclpy.ok() and not node.finished:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()