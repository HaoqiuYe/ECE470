#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import numpy as np
from math import pi
import sys
import argparse
class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3e home position
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location ``
# Q11 = [359.70*pi/180.0, -60.07*pi/180.0, 125.60*pi/180.0, -155.29*pi/180.0, -88.32*pi/180.0, 109.78*pi/180.0]

Q11 = [152.12*pi/180.0, -40.62*pi/180.0, 83.26*pi/180.0, -134.22*pi/180.0, -89.36*pi/180.0, 138.90*pi/180.0]
Q21 = [151.13*pi/180.0, -46.33*pi/180.0, 82.19*pi/180.0, -125.81*pi/180.0, -87.27*pi/180.0, 135.00*pi/180.0]
Q31 = [151.13*pi/180.0, -49.48*pi/180.0, 77.03*pi/180.0, -116.00*pi/180.0, -87.75*pi/180.0, 135.00*pi/180.0]

Q12 = [142.21*pi/180.0, -63.55*pi/180.0, 138.88*pi/180.0, -168.58*pi/180.0, -91.93*pi/180.0, 26.26*pi/180.0]
Q22 = [142.21*pi/180.0, -75.54*pi/180.0, 137.07*pi/180.0, -152.34*pi/180.0, -90.68*pi/180.0, 33.86*pi/180.0]
Q32 = [142.91*pi/180.0, -83*pi/180.0, 132.05*pi/180.0, -139.53*pi/180.0, -91.58*pi/180.0, 34.04*pi/180.0]

Q13 = [178.45*pi/180.0, -60.82*pi/180.0, 130.09*pi/180.0, -162.20*pi/180.0, -92.30*pi/180.0, 25.24*pi/180.0]
Q23= [178.46*pi/180.0, -69.46*pi/180.0, 126.84*pi/180.0, -148.90*pi/180.0, -92.80*pi/180.0, 20.98*pi/180.0]
Q33 = [177.42*pi/180.0, -76.43*pi/180.0, 121.73*pi/180.0, -133.82*pi/180.0, -90.73*pi/180.0, 20.44*pi/180.0]

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############
class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message and corresponding callback function
        # ROS2 gripper input topic: /io_and_status_controller/io_states
        self.gripper_input_sub = self.create_subscription(IOStates, "/io_and_status_controller/io_states", self.io_state_callback, 10)

        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

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
    ############## Your Code Start Here ##############
        """
        TODO: define a ROS topic callback funtion that 
        receives and stores the state of  the suction cup
        Whenever /io_and_status_controller/io_states 
        publishes this info, this callback function is
        called.
        """

        self.analog_in_0_value = msg.analog_in_states[0].state

    ############### Your Code End Here ###############

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

    def move_block(self, start_tower, start_height, end_tower, end_height):
        """Move one block between two tower positions.

        Tower and height numbers are one-based and index the Q waypoint table.
        """
        if start_tower not in (1, 2, 3) or end_tower not in (1, 2, 3):
            raise ValueError("tower numbers must be 1, 2, or 3")
        if start_height not in (1, 2, 3) or end_height not in (1, 2, 3):
            raise ValueError("tower heights must be 1, 2, or 3")
        if start_tower == end_tower:
            raise ValueError("start and end towers must be different")

        mid = np.radians([147.94, -100.79, 110.47, -101.88, -92.86, 20.48])

        if not self.move_arm(Q[start_height - 1][start_tower - 1]):
            self.get_logger().error("Could not reach the block pickup position")
            return False

        suction_on = True
        try:
            response = self.set_io(0, 1.0)
            if response is None or not getattr(response, "success", True):
                self.get_logger().error("Could not turn suction on")
                return False

            # Allow the suction cup to grip before lifting the block.
            time.sleep(0.75)

            # Move to the intermediate position after picking up the block.
            if not self.move_arm(mid):
                self.get_logger().error("Could not reach the intermediate position")
                return False

            # Move to the block drop position.
            if not self.move_arm(Q[end_height - 1][end_tower - 1]):
                self.get_logger().error("Could not reach the block drop position")
                return False

            # Release the block.
            response = self.set_io(0, 0.0)
            if response is None or not getattr(response, "success", True):
                self.get_logger().error("Could not turn suction off")
                return False

            suction_on = False

            # Return to the intermediate position after dropping the block.
            if not self.move_arm(mid):
                self.get_logger().error("Could not return to the intermediate position")
                return False

            return True

        except Exception as exc:
            self.get_logger().error(f"Block move failed: {exc}")
            return False

        finally:
            # Do not leave the gripper on after a failed move.
            if suction_on:
                try:
                    self.set_io(0, 0.0)
                except Exception as exc:
                    self.get_logger().error(f"Could not release suction: {exc}")


    # def move_block(self, start_tower, start_height, end_tower, end_height):
    #     """Move one block between two tower positions.

    #     Tower and height numbers are one-based and index the Q waypoint table.
    #     """
    #     if start_tower not in (1, 2, 3) or end_tower not in (1, 2, 3):
    #         raise ValueError("tower numbers must be 1, 2, or 3")
    #     if start_height not in (1, 2, 3) or end_height not in (1, 2, 3):
    #         raise ValueError("tower heights must be 1, 2, or 3")
    #     if start_tower == end_tower:
    #         raise ValueError("start and end towers must be different")

    #     mid = np.radians([147.94, -100.79, 110.47, -101.88, -92.86, 20.48])

    #     if not self.move_arm(Q[start_height - 1][start_tower - 1]):
    #         self.get_logger().error("Could not reach the block pickup position")
    #         return False

    #     suction_on = True
    #     try:
    #         response = self.set_io(0, 1.0)
    #         if response is None or not getattr(response, "success", True):
    #             self.get_logger().error("Could not turn suction on")
    #             return False

    #         # Allow the suction cup to grip before lifting the block.
    #         time.sleep(0.75)

    #         if not self.move_arm(mid):
    #             self.get_logger().error("Could not reach the intermediate position")
    #             return False
    #         if not self.move_arm(Q[end_height - 1][end_tower - 1]):
    #             self.get_logger().error("Could not reach the block drop position")
    #             return False

    #         response = self.set_io(0, 0.0)
    #         if response is None or not getattr(response, "success", True):
    #             self.get_logger().error("Could not turn suction off")
    #             return False

    #         suction_on = False
    #         return True
    #     except Exception as exc:
    #         self.get_logger().error(f"Block move failed: {exc}")
    #         return False
    #     finally:
    #         # Do not leave the gripper on after a failed move.
    #         if suction_on:
    #             try:
    #                 self.set_io(0, 0.0)
    #             except Exception as exc:
    #                 self.get_logger().error(f"Could not release suction: {exc}")


    def build_tower(self, start_tower, end_tower, num_blocks=3):
        """Move a complete Tower of Hanoi stack from start to end.

        The initial state contains ``num_blocks`` stacked on ``start_tower``;
        the final state contains all of them on ``end_tower``.  The waypoint
        table supports one through three blocks.
        """
        if start_tower not in (1, 2, 3) or end_tower not in (1, 2, 3):
            raise ValueError("tower numbers must be 1, 2, or 3")
        if start_tower == end_tower:
            raise ValueError("start and end towers must be different")
        if not isinstance(num_blocks, int) or not 1 <= num_blocks <= 3:
            raise ValueError("num_blocks must be an integer from 1 through 3")

        auxiliary_tower = 6 - start_tower - end_tower
        tower_heights = [0, 0, 0]
        tower_heights[start_tower - 1] = num_blocks

        def solve(num_to_move, source, destination, auxiliary):
            if num_to_move == 0:
                return True

            # Move the smaller stack out of the way first.
            if not solve(num_to_move - 1, source, auxiliary, destination):
                return False

            source_index = source - 1
            destination_index = destination - 1
            source_height = tower_heights[source_index]
            destination_height = tower_heights[destination_index] + 1

            if source_height < 1 or destination_height > 3:
                self.get_logger().error("Invalid Tower of Hanoi state")
                return False

            # The next disk is now exposed at the top of source.
            if not self.move_block(
                source, source_height, destination, destination_height
            ):
                return False

            tower_heights[source_index] -= 1
            tower_heights[destination_index] += 1

            # Put the smaller stack on top of the disk just moved.
            return solve(num_to_move - 1, auxiliary, destination, source)

        solved = solve(num_blocks, start_tower, end_tower, auxiliary_tower)
        solved = solved and tower_heights[start_tower - 1] == 0
        solved = solved and tower_heights[end_tower - 1] == num_blocks
        return solved


def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    # Wait for initial state updates
    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.05)
        node.get_logger().info("Waiting for initial state updates...")
        time.sleep(0.5)

    try:
        parser = argparse.ArgumentParser()
        parser.add_argument(
            "--start", type=int, choices=(1, 2, 3), required=True,
            help="Starting tower position"
        )
        parser.add_argument(
            "--end", type=int, choices=(1, 2, 3), required=True,
            help="Destination tower position"
        )

        cli_args = parser.parse_args()
        print(f"Start: {cli_args.start}, End: {cli_args.end}")

        if not node.move_arm(home):
            node.get_logger().error("Could not move to the home position")
        elif node.build_tower(cli_args.start, cli_args.end):
            node.get_logger().info(
                f"Successfully moved the tower from {cli_args.start} "
                f"to {cli_args.end}"
            )
        else:
            node.get_logger().error("Tower of Hanoi execution failed")

    except ValueError as exc:
        node.get_logger().error(f"Invalid tower input: {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
