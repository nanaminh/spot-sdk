# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show sending a mobility and arm command at the same time"""
import argparse
import math
import sys
import time

import numpy as np

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
from bosdyn.api import basic_command_pb2, robot_command_pb2, geometry_pb2, trajectory_pb2
from bosdyn.api.geometry_pb2 import SE2Velocity, SE2VelocityLimit, Vec2
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2
from bosdyn.client.frame_helpers import VISION_FRAME_NAME, ODOM_FRAME_NAME, BODY_FRAME_NAME, get_vision_tform_body, get_odom_tform_body, get_se2_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client import math_helpers
from bosdyn.client.robot_state import RobotStateClient
from bosdyn.util import seconds_to_duration

# # User-set params
# duration of the whole move [s]
#_SECONDS_FULL = 4
# # length of the square the robot walks [m]
# _L_ROBOT_SQUARE = 0.1
# # length of the square the robot walks [m]
# _L_ARM_CIRCLE = 0.4
# # how many points in the circle the hand will make. >4.
# _N_POINTS = 8
# # shift the circle that the robot draws in z [m]
# _VERTICAL_SHIFT = 0  # Change the position of the circle that arm draws


def admissive_walk(config):
    """Move spot with api """
    # See hello_spot.py for an explanation of these lines.
    bosdyn.client.util.setup_logging(config.verbose)

    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')
    robot = sdk.create_robot(config.hostname)
    bosdyn.client.util.authenticate(robot)
    robot.time_sync.wait_for_sync()

    #assert robot.has_arm(), "Robot requires an arm to run this example."

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)

    robot_state_client = robot.ensure_client(RobotStateClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info("Powering on robot... This may take a several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot powered on.")

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        robot.logger.info("Commanding robot to stand...")
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info("Robot standing.")
        
        #Deply arm
        unstow = RobotCommandBuilder.arm_ready_command()

        # Issue the command via the RobotCommandClient
        unstow_command_id = command_client.robot_command(unstow)
        robot.logger.info("Unstow command issued.")
        time.sleep(2)

    #--- Set obstacle avoidance parameter
        obstacle_padding_param = spot_command_pb2.ObstacleParams(obstacle_avoidance_padding=0.2)

    #--- First walk 
        # Get robot pose in vision frame from robot state (we want to send commands in vision
        # frame relative to where the robot stands now)
        robot_state = robot_state_client.get_robot_state()
        vision_T_body = get_odom_tform_body(robot_state.kinematic_state.transforms_snapshot)

        # Initialize a robot command message, which we will build out below
        command = robot_command_pb2.RobotCommand()

        # # Build a body se2trajectory by first assembling points
        # # Pull the point in the square relative to the robot and scale according to param
        dx =  1
        dy =  0
        dyaw = 0 #[degree]

        # # Transform desired position into vision frame
        # x_ewrt_vision, y_ewrt_vision, z_ewrt_vision = vision_T_body.transform_point(x, y, 0)

        # # Add a new point to the robot command's arm cartesian command se3 trajectory
        # # This will be an se2 trajectory point
        # point = command.synchronized_command.mobility_command.se2_trajectory_request.trajectory.points.add(
        # ) #add() ?? ,but using pose 

        # # Populate this point with the desired position, angle, and duration information
        # point.pose.position.x = x_ewrt_vision
        # point.pose.position.y = y_ewrt_vision

        # point.pose.angle = vision_T_body.rot.to_yaw()

                # # Get robot new state
        # robot_state = robot_state_client.get_robot_state()
        # vision_T_body = get_odom_tform_body(robot_state.kinematic_state.transforms_snapshot)        
        # # Transform desired position into vision frame
        # x_ewrt_vision, y_ewrt_vision, z_ewrt_vision = vision_T_body.transform_point(x, y, 0)
        
        transforms = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

        # Build the transform for where we want the robot to be relative to where the body currently is.
        body_tform_goal = math_helpers.SE2Pose(x=dx, y=dy, angle=dyaw)
        # We do not want to command this goal in body frame because the body will move, thus shifting
        # our goal. Instead, we transform this offset to get the goal position in the output frame
        # (which will be either odom or vision).
        # Commands will be sent in the visual odometry ("vision") frame
        frame_name = ODOM_FRAME_NAME
        out_tform_body = get_se2_a_tform_b(transforms, frame_name, BODY_FRAME_NAME)
        out_tform_goal = out_tform_body * body_tform_goal

        # Add a new point to the robot command's arm cartesian command se3 trajectory
        # This will be an se2 trajectory point
        point = command.synchronized_command.mobility_command.se2_trajectory_request.trajectory.points.add(
        ) #add() ?? ,but using pose 

        # Populate this point with the desired position, angle, and duration information
        point.pose.position.x = out_tform_goal.x
        point.pose.position.y = out_tform_goal.y

        point.pose.angle = out_tform_goal.angle

        distance = np.linalg.norm((dx,dy))
        traj_time = distance*8  #time to complete the trajectory
        duration = seconds_to_duration(traj_time)
        point.time_since_reference.CopyFrom(duration)
        
        # Commands will be sent in the visual odometry ("vision") frame
        frame_name = ODOM_FRAME_NAME
        # set the frame for the body trajectory
        command.synchronized_command.mobility_command.se2_trajectory_request.se2_frame_name = frame_name

        #Create mobility_params to specify walking style
        mobility_params = spot_command_pb2.MobilityParams(locomotion_hint=spot_command_pb2.HINT_TROT, obstacle_params= obstacle_padding_param)

        command.synchronized_command.mobility_command.params.CopyFrom(
            RobotCommandBuilder._to_any(mobility_params))

    #--- Add arm command  (This is dangerous, don't run)
        # unstow = RobotCommandBuilder.arm_ready_command()
        # time.sleep(0.5)

        # # Issue the command via the RobotCommandClient
        # unstow_command_id = command_client.robot_command(unstow)
        # robot.logger.info("Unstow command issued.")
        # command.synchronized_command.arm_command.arm_cartesian_command.root_frame_name = frame_name
        
        # odom_T_body = get_odom_tform_body(robot_state.kinematic_state.transforms_snapshot)
        # seconds_arm = 2
        # N_points = 2

        # for i in range(N_points + 1):
        #     # Get coordinates relative to the robot's body
        #     x = 0.75
        #     y = 0.3 *(i%2*(-1))
        #     z = 0.5

        #     # Using the transform we got earlier, transform the points into the vision frame
        #     x_ewrt_vision, y_ewrt_vision, z_ewrt_vision = x, y, z#odom_T_body.transform_point(x, y, z)

        #     # Add a new point to the robot command's arm cartesian command se3 trajectory
        #     # This will be an se3 trajectory point
        #     point = command.synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points.add(
        #     )
            
        #     # Populate this point with the desired position, rotation, and duration information
        #     point.pose.position.x = x_ewrt_vision
        #     point.pose.position.y = y_ewrt_vision
        #     point.pose.position.z = z_ewrt_vision

        #     point.pose.rotation.x = 1 #vision_T_body.rot.x
        #     point.pose.rotation.y = 0#vision_T_body.rot.y
        #     point.pose.rotation.z = 0#vision_T_body.rot.z
        #     point.pose.rotation.w = 0 #vision_T_body.rot.w

        #     traj_time = (i + 1) * seconds_arm
        #     duration = seconds_to_duration(traj_time)
        #     point.time_since_reference.CopyFrom(duration)


        # Send the command using the command client
        # The SE2TrajectoryRequest requires an end_time, which is set
        # during the command client call
        
        # Add arm command 
        x = 0.75  # a reasonable position in front of the robot
        y1 = 0  # centered
        y2 = 0  # 0.4 meters to the robot's left
        y3 = 0  # 0.4 meters to the robot's right
        z = 0.5  # at the body's height

        # Use the same rotation as the robot's body.
        #rotation = math_helpers.Quat()
        # Use rotation same as the arm 
        qw = 1
        qx = 0
        qy = 0
        qz = 0
        rotation = geometry_pb2.Quaternion(w=qw, x=qx, y=qy, z=qz)

        # Define times (in seconds) for each point in the trajectory.
        t_first_point = 0  # first point starts at t = 0 for the trajectory.
        t_second_point = 3.0
        t_third_point = 6.0

        # Build the points in the trajectory.
        hand_pose1 = math_helpers.SE3Pose(x=x, y=y1, z=z, rot=rotation)
        hand_pose2 = math_helpers.SE3Pose(x=x, y=y2, z=z, rot=rotation)
        hand_pose3 = math_helpers.SE3Pose(x=x, y=y3, z=z, rot=rotation)

        # Build the points by combining the pose and times into protos.
        traj_point1 = trajectory_pb2.SE3TrajectoryPoint(
            pose=hand_pose1.to_proto(), time_since_reference=seconds_to_duration(t_first_point))
        traj_point2 = trajectory_pb2.SE3TrajectoryPoint(
            pose=hand_pose2.to_proto(), time_since_reference=seconds_to_duration(t_second_point))
        traj_point3 = trajectory_pb2.SE3TrajectoryPoint(
            pose=hand_pose3.to_proto(), time_since_reference=seconds_to_duration(t_third_point))

        # Build the trajectory proto by combining the points.
        command.synchronized_command.arm_command.arm_cartesian_command.root_frame_name = BODY_FRAME_NAME
        point = command.synchronized_command.arm_command.arm_cartesian_command.pose_trajectory_in_task.points.add()
        point = [traj_point1, traj_point2, traj_point3]

        #(pose_trajectory_in_task=hand_traj, root_frame_name=BODY_FRAME_NAME)

        robot.logger.info("Sending the 1st trajectory commands.")
        command_client.robot_command(command, end_time_secs=time.time() + traj_time)
        time.sleep(max(traj_time, 8))
        
    #--- Middle stop to look around 
        footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.4, roll=0.2, pitch=0.0)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        command_client.robot_command(cmd)

        time.sleep(1) 

        footprint_R_body = bosdyn.geometry.EulerZXY(yaw=-0.4, roll=0.2, pitch=0.0)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        command_client.robot_command(cmd)

        time.sleep(1)

        footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0, roll=0, pitch=0)
        cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
        command_client.robot_command(cmd)
        time.sleep(0.5)

    #-------Second command 
        command2 = robot_command_pb2.RobotCommand()

        dx2 =  0.5
        dy2 =  0.0
        dyaw2 = 0 #[degree]

        # # Get robot new state
        # robot_state = robot_state_client.get_robot_state()
        # vision_T_body = get_odom_tform_body(robot_state.kinematic_state.transforms_snapshot)        
        # # Transform desired position into vision frame
        # x_ewrt_vision, y_ewrt_vision, z_ewrt_vision = vision_T_body.transform_point(x, y, 0)
        
        transforms2 = robot_state_client.get_robot_state().kinematic_state.transforms_snapshot

        # Build the transform for where we want the robot to be relative to where the body currently is.
        body_tform_goal = math_helpers.SE2Pose(x=dx2, y=dy2, angle=dyaw2)
        # We do not want to command this goal in body frame because the body will move, thus shifting
        # our goal. Instead, we transform this offset to get the goal position in the output frame
        # (which will be either odom or vision).
        # Commands will be sent in the visual odometry ("vision") frame
        #frame_name = ODOM_FRAME_NAME #Already defined in the top
        out_tform_body = get_se2_a_tform_b(transforms2, frame_name, BODY_FRAME_NAME)
        out_tform_goal = out_tform_body * body_tform_goal

        # Add a new point to the robot command's arm cartesian command se3 trajectory
        # This will be an se2 trajectory point
        point = command2.synchronized_command.mobility_command.se2_trajectory_request.trajectory.points.add(
        ) #add() ?? ,but using pose 

        # Populate this point with the desired position, angle, and duration information
        point.pose.position.x = out_tform_goal.x
        point.pose.position.y = out_tform_goal.y

        point.pose.angle = out_tform_goal.angle

        distance = np.linalg.norm((dx2,dy2))
        traj_time = distance*8  #time to complete the trajectory
        duration = seconds_to_duration(traj_time)
        point.time_since_reference.CopyFrom(duration)
        
        # set the frame for the body trajectory
        command2.synchronized_command.mobility_command.se2_trajectory_request.se2_frame_name = frame_name

        #Create mobility_params to specify walking style
        mobility_params = spot_command_pb2.MobilityParams(locomotion_hint=spot_command_pb2.HINT_HOP) 
        command2.synchronized_command.mobility_command.params.CopyFrom(
            RobotCommandBuilder._to_any(mobility_params))

        # Constrain the robot not to turn, forcing it to strafe laterally.
        speed_limit = SE2VelocityLimit(max_vel=SE2Velocity(linear=Vec2(x=2, y=2), angular=0),
                                       min_vel=SE2Velocity(linear=Vec2(x=-2, y=-2), angular=0))
        mobility_params = spot_command_pb2.MobilityParams(vel_limit=speed_limit)

        command2.synchronized_command.mobility_command.params.CopyFrom(
            RobotCommandBuilder._to_any(mobility_params))

        # Send the command using the command client
        # The SE2TrajectoryRequest requires an end_time, which is set
        # during the command client call
        robot.logger.info("Sending the 2nd trajectory command.")
        command_client.robot_command(command2, end_time_secs=time.time() + traj_time)
        time.sleep(traj_time)

        #command_client.robot_command(command2, end_time_secs=time.time() + _SECONDS_FULL)
        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), "Robot power off failed."
        robot.logger.info("Robot safely powered off.")


def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    options = parser.parse_args(argv)
    try:
        admissive_walk(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.exception("Threw an exception")
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
