
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rosbot_msgs.msg import BlockDetection, InverseMessage


class FinalSequencer(Node):

    def __init__(self):
        super().__init__('final_sequencer')

        self.target_frame = self.declare_parameter(
            'target_frame',
            'depth_cam_color_optical_frame'
        ).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.latest_detection = None
        self.detection_event = threading.Event()

        self.detection_sub = self.create_subscription(
            BlockDetection,
            '/block_detector/block_detections',
            self.detection_callback,
            10
        )

        # NEW: Publisher for inverse kinematics
        self.inverse_pub = self.create_publisher(
            InverseMessage,
            'inverse_kinematics_topic',
            10
        )

    def detection_callback(self, msg):
        self.latest_detection = msg
        self.detection_event.set()

    def wait_for_detection(self, timeout_sec=5.0):
        """Wait for the next detection message and return it, or None on timeout."""
        self.detection_event.clear()
        self.latest_detection = None

        if not self.detection_event.wait(timeout=timeout_sec):
            self.get_logger().warning('Timed out waiting for block detection')
            return None

        msg = self.latest_detection

        self.get_logger().info(
            f'Detected block: color={msg.color}, '
            f'position=({msg.position.x:.3f}, '
            f'{msg.position.y:.3f}, {msg.position.z:.3f})'
        )

        return msg

    # NEW: Wait specifically for a blue block
    def wait_for_blue_detection(self, timeout_sec=10.0):
        """Ignore other colors until a blue block is detected."""
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            detection = self.wait_for_detection(remaining)

            if detection is None:
                return None

            if str(detection.color).strip().lower() == 'blue':
                return detection

        return None

    def get_camera_transform(self, base_frame='base_footprint'):
        """Get the camera transform once, when needed."""
        try:
            t = self.tf_buffer.lookup_transform(
                base_frame,
                self.target_frame,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warning(f'Could not get camera transform: {ex}')
            return None

        T = quat_to_T(t.transform.rotation, t.transform.translation)
        self.get_logger().info(f'Camera transform:\n{T}')
        return T

    # NEW: Convert a detection from camera coordinates to arm coordinates
    def detection_to_arm_coordinates(self, detection):
        """Convert camera coordinates into base_link coordinates."""

        # Inverse kinematics expects coordinates relative to base_link
        T = self.get_camera_transform(base_frame='base_link')

        if T is None:
            return None

        # Homogeneous camera coordinates
        camera_point = np.array([
            detection.position.x,
            detection.position.y,
            detection.position.z,
            1.0
        ])

        # Convert camera frame to base_link frame
        arm_point = T @ camera_point
        xyz = arm_point[:3]

        self.get_logger().info(
            f'Block in base_link: '
            f'x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}'
        )

        return xyz

    # NEW: Move arm to a specified XYZ coordinate
    def move_arm(self, xyz, roll=0.0, pitch=-60.0, wait_sec=2.0):
        """Publish an inverse kinematics command."""

        msg = InverseMessage()

        msg.x = float(xyz[0])
        msg.y = float(xyz[1])
        msg.z = float(xyz[2])
        msg.roll = float(roll)
        msg.pitch = float(pitch)

        # Wait for inverse kinematics subscriber
        timeout = time.monotonic() + 3.0

        while self.inverse_pub.get_subscription_count() < 1:
            if time.monotonic() >= timeout:
                self.get_logger().warning(
                    'No inverse kinematics subscriber found'
                )
                return False
            time.sleep(0.05)

        self.inverse_pub.publish(msg)

        self.get_logger().info(
            f'Commanding arm to: xyz={xyz}, '
            f'roll={roll}, pitch={pitch}'
        )

        # Allow time for the arm to move
        time.sleep(wait_sec)
        return True


def quat_to_T(q, t):
    """Convert a quaternion and translation to a 4x4 transform matrix."""
    n = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    if n == 0:
        raise ValueError('Invalid zero-length quaternion')

    qx, qy, qz, qw = q.x/n, q.y/n, q.z/n, q.w/n

    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),
         2*(qx*qz + qy*qw), t.x],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),
         2*(qy*qz - qx*qw), t.y],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw),
         1 - 2*(qx*qx + qy*qy), t.z],
        [0, 0, 0, 1]
    ])


def main():
    rclpy.init()
    node = FinalSequencer()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Action 1: Move robot to initial position
        print('Action 1: Move to initial position')

        if not node.move_arm(
            xyz=[0.15, 0.0, 0.25],
            roll=0.0,
            pitch=-70.0
        ):
            return

        time.sleep(1)

        if not node.move_arm(
            xyz=[0.15, 0.0, 0.25],
            roll=90.0,
            pitch=-70.0
        ):
            return

        time.sleep(1)

        if not node.move_arm(
            xyz=[0.15, 0.0, 0.25],
            roll=0.0,
            pitch=-70.0
        ):
            return

        time.sleep(1)

        if not node.move_arm(
            xyz=[0.2, -0.2, 0.1],
            roll=-90.0,
            pitch=-40.0
        ):
            return

        time.sleep(1)


        if not node.move_arm(
            xyz=[0.2, 0.1, 0.25],
            roll=90.0,
            pitch=-70.0
        ):
            return

        time.sleep(1)

        if not node.move_arm(
            xyz=[0.15, 0.0, 0.25],
            roll=0.0,
            pitch=-70.0
        ):
            return


        # Example: Move to any specified base_link coordinate
        # node.move_arm([0.15, 0.0, 0.25], roll=0.0, pitch=-60.0)

        # Action 2: Detect a blue block
        print('Action 2: Waiting for blue block')

        detection1 = node.wait_for_blue_detection(timeout_sec=10.0)
        if detection1 is None:
            return

        # Action 3: Convert block coordinates to base_link
        print('Action 3: Converting block coordinates')

        block_xyz = node.detection_to_arm_coordinates(detection1)
        if block_xyz is None:
            return

        # Action 4: Move arm to detected block
        print('Action 4: Moving to blue block')

        # Optional: approach 5 cm above the block instead
        # block_xyz[2] += 0.05

        # node.move_arm(
        #     xyz=block_xyz,
        #     roll=0.0,
        #     pitch=-90.0
        # )

    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
