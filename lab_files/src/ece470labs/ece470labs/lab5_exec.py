#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import copy
import numpy as np
from math import pi
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from .blob_search import *
from .lab_func import *
from rclpy.qos import qos_profile_sensor_data

class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3 home position
home = np.radians([120, -90, 90, -90, -90, 0])
# Position for UR3 not blocking the camera
go_away = [270*pi/180.0, -90*pi/180.0, 90*pi/180.0, -90*pi/180.0, -90*pi/180.0, 135*pi/180.0]

# World-frame distances are in meters; lab_invk takes yaw in degrees.
STACK_XY = (0.300, 0.050)
TABLE_Z = 0.0
CUBE_HEIGHT = 0.031
TRAVEL_Z = 0.180
GRIPPER_YAW_DEG = 0.0
SUCTION_THRESHOLD = 2.0  # Same Analog Input 0 threshold used in Lab 2.

class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.io_state_sub = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_state_callback, 10)

        
        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')
        
        # === Vision-related members ===
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, qos_profile_sensor_data)

        self.image_count = 0
        # Store detected yellow block centers as world-frame (x, y) coordinates.
        self.yellow_blocks = []


        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.current_JointAngles = JointAngles()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ] # shoulder_pan_joint is the base rotation joint

    def joint_state_callback(self, msg):
        self.current_joint_state = msg  # Currently only used to check if messages have arrived
        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = msg.position[index_outofOrder]
            index_inOrder = index_inOrder + 1 


    def io_state_callback(self, msg):
        for state in msg.analog_in_states:
            if state.pin == 0:
                self.analog_in_0_value = state.state   


    def image_callback(self, msg: Image):
        try:
            raw_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge error: {e}")
            return

        # If you need to flip like in lab5:
        # cv_image = cv2.flip(raw_image, 1)
        cv_image = raw_image  # if no flip is needed

        # Optional overlay / debugging lines:
        # cv2.line(cv_image, (0,50), (640,50), (0,0,0), 5)

        # Use your existing blob_search to detect block centers in world frame
        self.yellow_blocks = blob_search(cv_image, "yellow")
        self.image_count += 1



    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state
        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def move_arm(self, target):
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received!")
            return False

        V_MAX = 1#2.09    # rad/s
        A_MAX = 0.8#2.79   # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(abs(self.current_JointAngles.position[i] - target[i]))


        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)
        if max_delta > 2 * d_acc:
            # trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # triangular velocity profile
            t_total = 2 * (max_delta / A_MAX) ** 0.5

        duration = max(MIN_DURATION, min(t_total, MAX_DURATION))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Start immediately when the controller receives it
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0

        # Anchor point: current measured joint state at t = 0
        p0 = JointTrajectoryPoint()
        p0.positions = self.current_JointAngles.position
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # end at rest, 2 point trajectory
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #end at rest.
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Moving to position: {np.degrees(target)}')

        # Wait for movement completion
        start_time = time.time()
        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(abs(self.current_JointAngles.position[i] - target[i]))
            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True
        return False


    def move_block_vision(self, start_xy, target_xyz):
        """Pick a table-level cube and place its top at target_xyz in world meters.

        Return False for a missing block so main can continue with the snapshot.
        Raise RuntimeError for motion or I/O failures to stop further transfers.
        """
        # ========================= Student's code starts here =========================
        start_x, start_y = start_xy
        target_x, target_y, target_z = target_xyz
        pick_z = TABLE_Z + CUBE_HEIGHT
        travel_z = max(TRAVEL_Z, target_z + 0.080)

        # lab_invk already calls lab_fk to apply the controller angle offsets.
        pick_above = lab_invk(start_x, start_y, travel_z, GRIPPER_YAW_DEG)
        pick = lab_invk(start_x, start_y, pick_z, GRIPPER_YAW_DEG)
        place_above = lab_invk(target_x, target_y, travel_z, GRIPPER_YAW_DEG)
        place = lab_invk(target_x, target_y, target_z, GRIPPER_YAW_DEG)
        if not np.isfinite([pick_above, pick, place_above, place]).all():
            raise RuntimeError("Pick or place position is outside the IK workspace")

        def move_to(joints, description):
            if not self.move_arm(joints):
                raise RuntimeError(f"Could not {description}")

        def suction(state):
            response = self.set_io(0, state)
            if response is None or not response.success:
                raise RuntimeError(f"Could not set suction to {state}")

        def settle(seconds):
            deadline = time.monotonic() + seconds
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                raise RuntimeError("ROS shut down during the block transfer")

        suction(0.0)
        move_to(pick_above, "move above the pickup position")
        move_to(pick, "reach the pickup position")
        suction(1.0)

        # Require new feedback after suction starts; keep processing callbacks.
        self.analog_in_0_value = None
        settle(2.0)
        if self.analog_in_0_value is None:
            suction(0.0)
            raise RuntimeError("No gripper feedback received after enabling suction")
        if self.analog_in_0_value <= SUCTION_THRESHOLD:
            self.get_logger().error(
                f"No block attached at ({start_x:.3f}, {start_y:.3f}); skipping it"
            )
            suction(0.0)
            move_to(pick_above, "retract after the missed pickup")
            return False

        move_to(pick_above, "lift the block above the pickup position")
        if self.analog_in_0_value <= SUCTION_THRESHOLD:
            self.get_logger().error(
                f"Block lost while lifting at ({start_x:.3f}, {start_y:.3f}); skipping it"
            )
            suction(0.0)
            return False

        move_to(place_above, "move above the stack")
        move_to(place, "lower the block onto the stack")
        suction(0.0)
        settle(0.5)
        move_to(place_above, "retract above the stack")
        return True
        # ========================= Student's code ends here =========================




def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()

    try:
        # Wait for joint state updates
        while rclpy.ok() and node.current_joint_state is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not rclpy.ok():
            return

        # Move to go_away (your move_arm uses rclpy.spin_once internally)
        if not node.move_arm(go_away):
            raise RuntimeError("Could not move the robot out of the camera view")

        # Let vision settle for 5 seconds
        image_count_before_settle = node.image_count
        settle_start = time.monotonic()
        while rclpy.ok() and (time.monotonic() - settle_start < 5.0):
            rclpy.spin_once(node, timeout_sec=0.1)
        if not rclpy.ok():
            return
        if node.image_count == image_count_before_settle:
            raise RuntimeError("No camera image received during the five-second wait")

        # ========================= Student's code starts here =========================
        # Freeze the yellow block positions before the arm obscures the camera.
        block_snapshot = [tuple(xy) for xy in node.yellow_blocks]

        node.get_logger().info(f"Snapshot yellow blocks: {block_snapshot}")
        placed_count = 0
        for start_xy in block_snapshot:
            # z is the suction-cup contact height at the top of the placed cube.
            target_z = TABLE_Z + (placed_count + 1) * CUBE_HEIGHT
            target_xyz = (STACK_XY[0], STACK_XY[1], target_z)
            if node.move_block_vision(start_xy, target_xyz):
                placed_count += 1
                node.get_logger().info(
                    f"Placed yellow block {placed_count} at {target_xyz}"
                )

        # ========================= Student's code ends here ===========================

        if not node.move_arm(go_away):
            raise RuntimeError("Could not return to the camera-clear position")
        node.get_logger().info(
            f"Task completed: stacked {placed_count} of {len(block_snapshot)} "
            "yellow blocks"
        )

    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(f"Task stopped: {exc}")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
