# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use Spot's arm.
"""
from __future__ import print_function

import argparse
import sys
import time
from math import radians

import bosdyn.api.gripper_command_pb2
import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api import arm_command_pb2, robot_command_pb2, synchronized_command_pb2, trajectory_pb2, geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, BODY_FRAME_NAME, get_se2_a_tform_b, get_vision_tform_body
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.util import seconds_to_duration
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_for_trajectory_cmd, block_until_arm_arrives, blocking_stand)
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2 # for locomotion hint
from bosdyn import geometry

from fronting_straight import relative_move

def mobility_params(body_height=0.0, footprint_R_body=geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.0),
                    locomotion_hint=spot_command_pb2.HINT_AUTO, obs_padding = 0.5, 
                    swing_height= spot_command_pb2.SwingHeight.SWING_HEIGHT_MEDIUM,
                    abs_mac_vel=0.5, abs_max_angvel=0.2, stair_hint=False,
                    external_force_params=None, stairs_mode=None):
    """Helper to create Mobility params for spot mobility commands. This function is designed
    to help get started issuing commands, but lots of options are not exposed via this
    interface. See spot.robot_command_pb2 for more details. If unset, good defaults will be
    chosen by the robot.

    Args:
        body_height: Height, meters, relative to a nominal stand height.
        footprint_R_body(EulerZXY): The orientation of the body frame with respect to the
                                    footprint frame (gravity aligned framed with yaw computed
                                    from the stance feet)
        locomotion_hint: Locomotion hint to use for the command.
        obs_padding: padding (distance [m]) for obstacle avoidance
        swing_height: Height of the swing {SWING_HEIGHT_LOW, SWING_HEIGHT_MEIDUM, SWING_HEIGHT_HIGH} 
        abs_mac_vel: Absolute maximum velocity for a trajectory. Don't set larger than 2.0
        stair_hint: Boolean to specify if stair mode should be used. Deprecated in favor of stairs_mode
                                    and ignored if stairs_mode set.
        external_force_params(spot.BodyExternalForceParams): Robot body external force
                                                                parameters.
        stairs_mode: StairsMode enum specifying stairs mode as On, Auto, or Off.

    Returns:
        spot.MobilityParams, params for spot mobility commands.
    """
    # Simplified body control params
    position = geometry_pb2.Vec3(z=body_height)
    rotation = footprint_R_body.to_quaternion()
    pose = geometry_pb2.SE3Pose(position=position, rotation=rotation)
    point = trajectory_pb2.SE3TrajectoryPoint(pose=pose)
    traj = trajectory_pb2.SE3Trajectory(points=[point])
    body_control = spot_command_pb2.BodyControlParams(base_offset_rt_footprint=traj)
    speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=abs_mac_vel, y=abs_mac_vel), angular= abs_max_angvel),
                                    min_vel=SE2Velocity(linear=Vec2(x=-abs_mac_vel, y=-abs_mac_vel), angular= -abs_max_angvel))
    obstacle_params = spot_command_pb2.ObstacleParams(obstacle_avoidance_padding=obs_padding)

    return spot_command_pb2.MobilityParams(
        vel_limit = speed_limit, body_control=body_control, locomotion_hint=locomotion_hint, stair_hint=stair_hint, 
        obstacle_params = obstacle_params, swing_height= swing_height, 
        external_force_params=external_force_params, stairs_mode=stairs_mode)

#Function to create arm command from joint angles 
def make_robot_command(arm_joint_traj):
    """ Helper function to create a RobotCommand from an ArmJointTrajectory.
        The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
        filled out to follow the passed in trajectory. """

    joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)
    arm_command = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
    sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
    arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
    return RobotCommandBuilder.build_synchro_command(arm_sync_robot_cmd)

def stow_arm(robot_command_client):
    stow_cmd= RobotCommandBuilder.arm_stow_command(build_on_command=None)
    stow_command_id = robot_command_client.robot_command(stow_cmd)

    block_until_arm_arrives(robot_command_client, stow_command_id, 3.0)

def middle_stop(robot_command_client):
    footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.2, pitch=0.0)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(cmd)

    time.sleep(1) 

    footprint_R_body = bosdyn.geometry.EulerZXY(yaw=-0.4, roll=0.2, pitch=0.0)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(cmd)

    time.sleep(1)

    footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0, roll=0, pitch=0)
    cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    robot_command_client.robot_command(cmd)
    time.sleep(0.5)

def make_arm_joint_traj(mode):
    if mode == 's':
        sh0 = 0.0
        sh1 = radians(-170) # 0 -> extended
        el0 = radians(160) -0.1 
        el1 = 0.0
        wr0 = radians(15)
        wr1 = 0.0

    if mode == 'd':
        sh0 = 0.0
        sh1 = radians(-150) # 0 -> extended
        el0 = radians(130) + 0.1
        el1 = 0.0
        wr0 = radians(20)
        wr1 = 0.0  

    if mode == 'n':
        sh0 = 0.0
        sh1 = radians(-170) # 0 -> extended
        el0 = radians(160) 
        el1 = 0.0
        wr0 = radians(15)
        wr1 = 0.0

    traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
        sh0, sh1, el0, el1, wr0, wr1)
    arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point]) 

    return arm_joint_traj

def relative_move_cmd(mode, dx, dy, dyaw, frame_name, robot_state_client, speed_limit = 0.5, speed_limit_ang = 0.1, obs_padding= 0.5):
    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

    # Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    frame_name = ODOM_FRAME_NAME
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

    if mode == 's':
        mobility_param= mobility_params(body_height= -0.1, footprint_R_body=geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.1),
                                        abs_mac_vel = speed_limit, abs_max_angvel=speed_limit_ang, swing_height= spot_command_pb2.SwingHeight.SWING_HEIGHT_LOW,
                                        locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_CRAWL, obs_padding=obs_padding)
        arm_joint_traj = make_arm_joint_traj('s')

        #---Arm command 
        arm_command = make_robot_command(arm_joint_traj)
        
    if mode =='d':
        mobility_param= mobility_params(body_height= 0.1, footprint_R_body=geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=-0.1),
                                        abs_mac_vel = speed_limit, abs_max_angvel=speed_limit_ang, swing_height= spot_command_pb2.SwingHeight.SWING_HEIGHT_MEDIUM,
                                        locomotion_hint=spot_command_pb2.HINT_SPEED_SELECT_TROT, obs_padding=obs_padding)
        
        arm_joint_traj = make_arm_joint_traj('d')

        #---Arm command  
        arm_command = make_robot_command(arm_joint_traj)

    if mode =='n':
        mobility_param= mobility_params(body_height= 0.0,
                                    locomotion_hint=spot_command_pb2.HINT_TROT)
        
        arm_joint_traj = make_arm_joint_traj('n')

        #---Arm command  
        arm_command = make_robot_command(arm_joint_traj)

    robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
    goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
    frame_name=frame_name, params=mobility_param, build_on_command=arm_command)
    
    return robot_cmd

def arm_look_direction(command_client, dy):

    unstow = RobotCommandBuilder.arm_ready_command()
    unstow_command_id = command_client.robot_command(unstow)

    block_until_arm_arrives(command_client, unstow_command_id, 3.0)

    # Convert the location from the moving base frame to the world frame.
    # robot_state = robot_state_client.get_robot_state()
    # odom_T_flat_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
    #                                     ODOM_FRAME_NAME, GRAV_ALIGNED_BODY_FRAME_NAME)
    x= 0.8
    y = 0
    z = 0.6
    qw =  0.966
    qx = 0
    qy = 0
    qz = 0.259

    if dy > 0: # Look left 
        gaze_command = RobotCommandBuilder.arm_gaze_command(3, 1, 0.5, BODY_FRAME_NAME)
        pose_command = RobotCommandBuilder.arm_pose_command(x= x, y = y, z = z, qw = qw, 
                                                            qx = qx, qy = qy, qz = qz, frame_name = BODY_FRAME_NAME)
        
    if dy < 0: # Look right
        gaze_command = RobotCommandBuilder.arm_gaze_command(3,
                                                            -1,
                                                            0.5, BODY_FRAME_NAME)
        pose_command = RobotCommandBuilder.arm_pose_command(x= x, y = y, z = z, qw = qw, 
                                                            qx = qx, qy = qy, qz = - qz, frame_name = BODY_FRAME_NAME)
        
    #robot.logger.info("Requesting gaze.")
    gaze_command_id = command_client.robot_command(gaze_command)
    block_until_arm_arrives(command_client, gaze_command_id, 4.0)