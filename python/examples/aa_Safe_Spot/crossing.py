# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Command the robot to go to an offset position using a trajectory command."""

import logging
import math
import sys
import time
from math import radians

import bosdyn.client
import bosdyn.client.util
from bosdyn import geometry
from bosdyn.api import (arm_command_pb2, basic_command_pb2, full_body_command_pb2, geometry_pb2,
                        gripper_command_pb2, mobility_command_pb2, payload_estimation_pb2,
                        robot_command_pb2, robot_command_service_pb2_grpc, synchronized_command_pb2,
                        trajectory_pb2)
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.basic_command_pb2 import RobotCommandFeedbackStatus
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import (BODY_FRAME_NAME, ODOM_FRAME_NAME, VISION_FRAME_NAME,
                                         get_se2_a_tform_b)
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_for_trajectory_cmd, block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2 # for locomotion hint
from helpers import mobility_params, make_robot_command, stow_arm, relative_move_cmd
_LOGGER = logging.getLogger(__name__)

def main():
    import argparse
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--dx', default=0, type=float,
                        help='Position offset in body frame (meters forward).')
    parser.add_argument('--dy', default=0, type=float,
                        help='Position offset in body frame (meters left).')
    parser.add_argument('--dyaw', default=0, type=float,
                        help='Position offset in body frame (degrees ccw).')
    parser.add_argument('--frame', choices=[VISION_FRAME_NAME, ODOM_FRAME_NAME],
                        default=ODOM_FRAME_NAME, help='Send the command in this frame.')
    parser.add_argument('--stairs', action='store_true', help='Move the robot in stairs mode.')
    parser.add_argument('--locomo', default=1, type=int, help='Select locomotion hint.')
    parser.add_argument('--mode', default='s', type=str, choices= ['s', 'd'],
                        help='Choose either admissive or domimant')
    
    options = parser.parse_args()
    bosdyn.client.util.setup_logging(options.verbose)

    # Create robot object.
    sdk = bosdyn.client.create_standard_sdk('RobotCommandMaster')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)

    # Check that an estop is connected with the robot so that the robot commands can be executed.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    # Create the lease client.
    lease_client = robot.ensure_client(LeaseClient.default_service_name)

    # Setup clients for the robot state and robot command services.
    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    robot_command_client = robot.ensure_client(RobotCommandClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Power on the robot and stand it up.
        robot.time_sync.wait_for_sync()
        robot.power_on()
        blocking_stand(robot_command_client)

        try:
            relative_move(options.mode, options.dx, options.dy, math.radians(options.dyaw), options.frame,
                                 robot_command_client, robot_state_client)
                    
        finally:
            # Close the gripper 
            robot_command_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
            # Stow the arm before sitting down 
            stow_arm(robot_command_client)
            # Send a Stop at the end, regardless of what happened.
            robot_command_client.robot_command(RobotCommandBuilder.stop_command())

def relative_move(mode, dx, dy, dyaw, frame_name, robot_command_client, robot_state_client, stairs=False):
#     transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

#     # Build the transform for where we want the robot to be relative to where the body currently is.
#     body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
#     # We do not want to command this goal in body frame because the body will move, thus shifting
#     # our goal. Instead, we transform this offset to get the goal position in the output frame
#     # (which will be either odom or vision).
#     frame_name = ODOM_FRAME_NAME
#     out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
#     out_tform_goal = out_tform_body * body_tform_goal

# #--- Set mobility param for each mode
#     speed_limit = 0.5
#     obs_padding = 0.5

#     if mode == 's':
#         mobility_param= mobility_params(body_height= -0.1, abs_mac_vel = speed_limit, 
#                                         footprint_R_body=geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.1), 
#                                         swing_height= spot_command_pb2.SwingHeight.SWING_HEIGHT_LOW,
#                                         locomotion_hint=spot_command_pb2.HINT_AUTO, obs_padding=obs_padding)
        
#         sh0 = 0.0
#         sh1 = radians(-170) # 0 -> extended
#         el0 = radians(160) -0.1 
#         el1 = 0.0
#         wr0 = radians(15)
#         wr1 = 0.0

#         traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
#             sh0, sh1, el0, el1, wr0, wr1)
#         arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])  

#         robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
#             goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
#             frame_name=frame_name, params=mobility_param)
        
#         arm_command = make_robot_command(arm_joint_traj)
#         #arm_command = RobotCommandBuilder.arm_ready_command()
        
#     if mode =='d':
#         mobility_param= mobility_params(body_height= 0.1, abs_mac_vel = speed_limit, 
#                                         footprint_R_body=geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=-0.1), 
#                                         swing_height= spot_command_pb2.SwingHeight.SWING_HEIGHT_MEDIUM,
#                                         locomotion_hint=spot_command_pb2.HINT_TROT, obs_padding = obs_padding)
#         sh0 = 0.0
#         sh1 = radians(-150) # 0 -> extended
#         el0 = radians(130) + 0.1
#         el1 = 0.0
#         wr0 = radians(20)
#         wr1 = 0.0  

#         traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
#             sh0, sh1, el0, el1, wr0, wr1)
#         arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])  

#         #---- Try adding gripper open command 
#         #gripper_open_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.3)

#         #---Arm command 
#         arm_command = make_robot_command(arm_joint_traj)
#         #arm_command = RobotCommandBuilder.build_synchro_command(gripper_open_command, arm_command)

#     if mode =='n': # neutral
#         mobility_param= mobility_params(body_height= 0.0,
#                                     locomotion_hint=spot_command_pb2.HINT_TROT)
#         sh0 = 0.0
#         sh1 = radians(-170) # 0 -> extended
#         el0 = radians(160) 
#         el1 = 0.0
#         wr0 = radians(15)
#         wr1 = 0.0

#         traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
#             sh0, sh1, el0, el1, wr0, wr1)
#         arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])

#         #---Arm command 
#         arm_command = make_robot_command(arm_joint_traj)

    # robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
    # goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
    # frame_name=frame_name, params=mobility_param, build_on_command=arm_command)
    
    robot_cmd = relative_move_cmd(mode, dx, dy, dyaw, frame_name, robot_state_client, speed_limit = 0.5, speed_limit_ang = 0.1, obs_padding= 0.5)
    max_duration = 30.0
    time.sleep(0.1) # time before it starts walking 
    start_time = time.time()
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=start_time + max_duration)
    
    return robot_cmd, cmd_id, start_time
    
    while True:
        feedback = robot_command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
    #     # velocity = robot_state_client.get_robot_state().kinematic_state.velocity_of_body_in_odom
    #     # print("velocity", velocity)
    #     if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
    # #         print("Failed to reach the goal")
    #         return False
    #     traj_feedback = mobility_feedback.se2_trajectory_feedback
    #     if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
    #             traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
    #         print("Arrived at the goal.")
    #         return True
        time.sleep(0.5)
        print("loop here")
        print("cmd_id", cmd_id)

        return mobility_feedback


if __name__ == "__main__":
    if not main():
        sys.exit(1)
