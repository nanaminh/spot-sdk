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
from bosdyn.util import seconds_to_timestamp

_LOGGER = logging.getLogger(__name__)

"""
How to use:
Give argument(s) to define the trajectory  (tested for dx only)
By default, the robot walks dx*2 distance and makes a middle stop. 
"""

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
    parser.add_argument('--mode', default='n', type=str, choices= ['a', 'd', 'n'],
                        help='Choose either admissive or domimant')
    parser.add_argument('--neck', default=0, type=int, 
                        help='Choose if you want to move the neck while walking')
    
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
            relative_move(options.mode, options.dx, options.dy/2, math.radians(options.dyaw), options.frame,
                                robot_command_client, robot_state_client, 1)
            middle_stop(robot_command_client)
            relative_move(options.mode, options.dx, options.dy/2, math.radians(options.dyaw), options.frame,
                                robot_command_client, robot_state_client, 0)
        finally:
            # Close the gripper 
            robot_command_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
            # Stow the arm before sitting down 
            stow_arm(robot_command_client)
            # Send a Stop at the end, regardless of what happened.
            robot_command_client.robot_command(RobotCommandBuilder.stop_command())

def mobility_params(body_height=0.1, footprint_R_body=geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.0),
                    locomotion_hint=spot_command_pb2.HINT_AUTO, obs_padding = 0.2, 
                    swing_height= spot_command_pb2.SwingHeight.SWING_HEIGHT_MEDIUM,
                    abs_mac_vel = 0.5, stair_hint=False,
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
    speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=abs_mac_vel, y=abs_mac_vel)),
                                    min_vel=SE2Velocity(linear=Vec2(x=-abs_mac_vel, y=-abs_mac_vel)))
    obstacle_params = spot_command_pb2.ObstacleParams(obstacle_avoidance_padding=obs_padding)

    return spot_command_pb2.MobilityParams(
        vel_limit = speed_limit, body_control=body_control, locomotion_hint=locomotion_hint, stair_hint=stair_hint, 
        obstacle_params = obstacle_params, swing_height= swing_height, 
        external_force_params=external_force_params, stairs_mode=stairs_mode)

#Function to create arm command from joint angles 
def make_robot_command(arm_joint_traj, build_on = None):
    """ Helper function to create a RobotCommand from an ArmJointTrajectory.
        The returned command will be a SynchronizedCommand with an ArmJointMoveCommand
        filled out to follow the passed in trajectory. """

    joint_move_command = arm_command_pb2.ArmJointMoveCommand.Request(trajectory=arm_joint_traj)
    arm_command = arm_command_pb2.ArmCommand.Request(arm_joint_move_command=joint_move_command)
    sync_arm = synchronized_command_pb2.SynchronizedCommand.Request(arm_command=arm_command)
    arm_sync_robot_cmd = robot_command_pb2.RobotCommand(synchronized_command=sync_arm)
    return RobotCommandBuilder.build_synchro_command(build_on, arm_sync_robot_cmd)

def stow_arm(robot_command_client):
    stow_cmd= RobotCommandBuilder.arm_stow_command(build_on_command=None)
    stow_command_id = robot_command_client.robot_command(stow_cmd)

    #robot.logger.info("Stow command issued.")
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

def walk_looking_around(t, nominal_pose):
    """
    Function to create a arm trajectory to look around while walking 
    """

    # Nominal oscillation period in seconds
    T = 5.0
    w = 2 * math.pi / T

    joint_positions = nominal_pose
    # Have a few of our joints oscillate
    #joint_positions[0] = nominal_pose[0] + 0.8 * math.cos(w * t)
    #joint_positions[2] = nominal_pose[2] + 0.2 * math.sin(2 * w * t)
    joint_positions[4] = nominal_pose[4] + 0.5 * math.sin(2 * w * t)
    #joint_positions[5] = nominal_pose[5] + 1.0 * math.cos(4 * w * t)

    # Take the derivative of our position trajectory to get our velocities
    joint_velocities = [0, 0, 0, 0, 0, 0]
    #joint_velocities[0] = -0.8 * w * math.sin(w * t)
    #joint_velocities[2] = 0.2 * 2 * w * math.cos(2 * w * t)
    joint_velocities[4] = 0.5 * 2 * w * math.cos(2 * w * t)
    #joint_velocities[5] = -1.0 * 4 * w * math.sin(4 * w * t)

    # Return the joint positions and velocities at time t in our trajectory
    return joint_positions, joint_velocities

def relative_move(mode, dx, dy, dyaw, frame_name, robot_command_client, robot_state_client, neck, stairs=False):
    start_time = time.time()
    ref_time = seconds_to_timestamp(start_time)

    transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot
    
#--- Build the transform for where we want the robot to be relative to where the body currently is.
    body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
    # We do not want to command this goal in body frame because the body will move, thus shifting
    # our goal. Instead, we transform this offset to get the goal position in the output frame
    # (which will be either odom or vision).
    frame_name = ODOM_FRAME_NAME
    out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
    out_tform_goal = out_tform_body * body_tform_goal

#--- Set maximum velocity 
     

#--- Set mobility param for each mode
    if mode == 'a':
        mobility_param= mobility_params(body_height= -0.1, swing_height= spot_command_pb2.SwingHeight.SWING_HEIGHT_LOW,
                                        locomotion_hint=spot_command_pb2.HINT_CRAWL)
        sh0 = 0.0
        sh1 = -2.5
        el0 = 2.6
        el1 = 0.0
        wr0 = -0.25
        wr1 = 0.0

        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1)
        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])  
        
        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
            frame_name=frame_name, params=mobility_param)
        arm_command = make_robot_command(arm_joint_traj, build_on= robot_cmd)

    if mode =='d':
        mobility_param= mobility_params(body_height= 0.2,
                                    locomotion_hint=spot_command_pb2.HINT_TROT)
        sh0 = 0.0
        sh1 = -1.9 
        el0 = 2.6
        el1 = 0.0
        wr0 = -0.6
        wr1 = 0.0

        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1)
        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])  

        #---- Try adding gripper open command 
        gripper_open_command = RobotCommandBuilder.claw_gripper_open_fraction_command(0.3)

        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
            frame_name=frame_name, params=mobility_param, build_on_command=gripper_open_command)
        arm_command = make_robot_command(arm_joint_traj, build_on= robot_cmd)

    if mode =='n': # neutral
        mobility_param= mobility_params(body_height= 0.0,
                                    locomotion_hint=spot_command_pb2.HINT_TROT)
        sh0 = 0.0
        sh1 = -2.2 
        el0 = 2.6
        el1 = 0.0
        wr0 = -0.4
        wr1 = 0.0

        traj_point = RobotCommandBuilder.create_arm_joint_trajectory_point(
            sh0, sh1, el0, el1, wr0, wr1)
        arm_joint_traj = arm_command_pb2.ArmJointTrajectory(points=[traj_point])

        robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
            frame_name=frame_name, params=mobility_param)
        arm_command = make_robot_command(arm_joint_traj, build_on= robot_cmd)

    if neck == 1:
        mobility_param= mobility_params(body_height= 0.0,
                        locomotion_hint=spot_command_pb2.HINT_TROT)
        
        nominal_pose = [0, -2.5, 2.6, 0, -0.25, 0] #admissive

        TRAJ_APPROACH_TIME = 1.0

        pos, vel = walk_looking_around(0, nominal_pose)
        neck_cmd = RobotCommandBuilder.arm_joint_move_helper(
            joint_positions=[pos], joint_velocities=[vel], times=[TRAJ_APPROACH_TIME],
            ref_time=ref_time, max_acc=10000, max_vel=10000)
        robot_command_client.robot_command(neck_cmd)

        dt = 0.2
        segment_start_time = 0
        N = round(dx*2) + 1
        
        #---- While the robot has not reached the goal, move the joint
        while time.time() - start_time < 20:
            times = []
            positions = []
            velocities = []
            for i in range(N):
                t = segment_start_time + i * dt
                pos, vel = walk_looking_around(t, nominal_pose)
                positions.append(pos)
                velocities.append(vel)
                times.append(t)

            arm_command = RobotCommandBuilder.arm_joint_move_helper(joint_positions=positions,
                                                        joint_velocities=velocities,
                                                        times=times, ref_time=ref_time,
                                                        max_acc=10000, max_vel=10000)

            # Wait until a bit before the previous trajectory is going to expire,
            # and send our new trajectory
            sleep_time = start_time + segment_start_time - (dt * 3 / 4) - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

            # Send the request
            #cmd_id = robot_command_client.robot_command(arm_cmd)
            # We will start our next segment at the same place as the last point in our
            # trajectory, so we get good continuity
            segment_start_time = segment_start_time + dt * (N - 1)

#---Arm command 
    #arm_command = RobotCommandBuilder.arm_pose_command(x=0.5, y=0, z=0.4,qw=1, qx=0, qy=0, qz=0, frame_name=BODY_FRAME_NAME, build_on_command= gripper_open_command )
    #gripper_close_cmd = 
    # Make a RobotCommand
    
    robot_command_client.robot_command(arm_command)

    end_time = 20.0
    time.sleep(1) # time before it starts walking 
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=time.time() + end_time)

    # Wait until the robot has reached the goal.


    while True:
        feedback = robot_command_client.robot_command_feedback(cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback

        #-----
        if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
            print("Failed to reach the goal")
            return False
        traj_feedback = mobility_feedback.se2_trajectory_feedback
        if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
                traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
            print("Arrived at the goal.")
            return True
        time.sleep(1)

    return True


if __name__ == "__main__":
    if not main():
        sys.exit(1)
