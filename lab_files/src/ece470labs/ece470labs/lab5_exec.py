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
YELLOW_STACK_XY = (0.300, 0.050)
GREEN_STACK_XY = (0.300, 0.150)
TABLE_Z = 0.0
CUBE_HEIGHT = 0.029
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
        # Store detected block centers as world-frame (x, y) coordinates by color.
        self.yellow_blocks = []
        self.green_blocks = []


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
        self.green_blocks = blob_search(cv_image, "green")
        self.image_count += 1



    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state
        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("Gripper command failed")
        return response


    def move_arm(self, target):
        if self.current_joint_state is None:
            raise RuntimeError("No joint state received")

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
        raise RuntimeError("Arm did not reach the commanded position")


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

        # Like Lab 2's mid, these high waypoints provide clearance before transfer
        # and after release. Their (x, y) positions follow the detected block/stack.
        pick_above = lab_invk(start_x, start_y, travel_z, GRIPPER_YAW_DEG)
        pick = lab_invk(start_x, start_y, pick_z, GRIPPER_YAW_DEG)
        place_above = lab_invk(target_x, target_y, travel_z, GRIPPER_YAW_DEG)
        place = lab_invk(target_x, target_y, target_z, GRIPPER_YAW_DEG)
        if not np.isfinite([pick_above, pick, place_above, place]).all():
            raise RuntimeError("Pick or place position is outside the IK workspace")

        self.set_io(0, 0.0)
        self.move_arm(pick_above)
        self.move_arm(pick)
        self.set_io(0, 1.0)

        # Require new feedback after suction starts; keep processing callbacks.
        self.analog_in_0_value = None
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not rclpy.ok():
            raise RuntimeError("ROS shut down during the block transfer")

        if self.analog_in_0_value is None:
            self.set_io(0, 0.0)
            raise RuntimeError("No gripper feedback received after enabling suction")

        if self.analog_in_0_value <= SUCTION_THRESHOLD:
            self.get_logger().error(
                f"No block attached at ({start_x:.3f}, {start_y:.3f}); skipping it"
            )
            self.set_io(0, 0.0)
            self.move_arm(pick_above)
            return False

        self.move_arm(pick_above)
        if self.analog_in_0_value <= SUCTION_THRESHOLD:
            self.get_logger().error(
                f"Block lost while lifting at ({start_x:.3f}, {start_y:.3f}); skipping it"
            )
            self.set_io(0, 0.0)
            return False

        self.move_arm(place_above)
        self.move_arm(place)
        self.set_io(0, 0.0)
        time.sleep(0.5)
        self.move_arm(place_above)
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
        node.move_arm(go_away)

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
        # Freeze both colors before either pass obscures the camera.
        yellow_snapshot = list(node.yellow_blocks)
        green_snapshot = list(node.green_blocks)

        placed_count = 0
        for start_xy in yellow_snapshot:
            # z is the suction-cup contact height at the top of the placed cube.
            target_z = TABLE_Z + (placed_count + 1) * CUBE_HEIGHT
            target_xyz = (YELLOW_STACK_XY[0], YELLOW_STACK_XY[1], target_z)
            if node.move_block_vision(start_xy, target_xyz):
                placed_count += 1

        placed_count = 0
        for start_xy in green_snapshot:
            target_z = TABLE_Z + (placed_count + 1) * CUBE_HEIGHT
            target_xyz = (GREEN_STACK_XY[0], GREEN_STACK_XY[1], target_z)
            if node.move_block_vision(start_xy, target_xyz):
                placed_count += 1

        # ========================= Student's code ends here ===========================

        node.move_arm(go_away)
        node.get_logger().info("Task completed")

    except KeyboardInterrupt:
        pass
    except RuntimeError as exc:
        node.get_logger().error(f"Task stopped: {exc}")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# def main(args=None):
#     input("Check if the UR3e is in 'Remote' Mode?\n\
#     Check if the UR3e is initialized and in 'Normal' state.\n\
#     Have you run the ROS2 launch statement?\n\
#     If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
#     \n\
#     Press <Enter> to Continue.")
#     rclpy.init(args=args)
#     node = UR3e()

#     try:
#         # Wait for joint state updates
#         while rclpy.ok() and node.current_joint_state is None:
#             rclpy.spin_once(node, timeout_sec=0.1)

#         # Move to go_away (your move_arm uses rclpy.spin_once internally)
#         node.move_arm(go_away)

#         # Let vision settle for 5 seconds
#         settle_start = time.time()
#         while rclpy.ok() and (time.time() - settle_start < 5.0):
#             rclpy.spin_once(node, timeout_sec=0.1)

#         # Snapshot once 
#         green_snapshot = list(node.green_blocks)
#         node.get_logger().info(f"Snapshot green blocks: {green_snapshot}")

#         # Execute task once based on snapshot
#         # ========================= Student's code starts here =========================

#         """
#         TODO: Use the vision-detected block positions to move blocks around. 
#         Hints: use the found green_blocks, yellow_blocks to move the blocks correspondingly. You will
#         need to call move_block_vision(start_xy, target_xyz) where start_xy is the world coordinate of 
#         the block you want to move, and target_xyz is the world coordinate of where you want to place it 
#         (you can set z based on your needs).
#         """

#         # ========================= Student's code ends here ===========================

#         node.move_arm(go_away)
#         node.get_logger().info("Task Completed. Press Ctrl+C to exit.")

#         while rclpy.ok():
#             rclpy.spin_once(node, timeout_sec=0.1)

#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

if __name__ == '__main__':
    main()
