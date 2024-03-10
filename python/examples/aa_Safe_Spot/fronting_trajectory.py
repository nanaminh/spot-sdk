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
import numpy as np
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
from bosdyn.client.image import ImageClient
from bosdyn.api.spot import robot_command_pb2 as spot_command_pb2 # for locomotion hint
from helpers import mobility_params, make_robot_command, middle_stop, stow_arm, make_arm_joint_traj, relative_move_cmd
from PC_detection import remove_ground 
from crossing import relative_move


_LOGGER = logging.getLogger(__name__)


def main():
    import argparse
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    parser.add_argument('--dx', default=1, type=float,
                        help='Position offset in body frame (meters forward).')
    parser.add_argument('--dy', default=0, type=float,
                        help='Position offset in body frame (meters left).')
    parser.add_argument('--dyaw', default=0, type=float,
                        help='Position offset in body frame (degrees ccw).')
    parser.add_argument('--frame', choices=[VISION_FRAME_NAME, ODOM_FRAME_NAME],
                        default=ODOM_FRAME_NAME, help='Send the command in this frame.')
    parser.add_argument('--avoiding_dir', choices=['l', 'r'],
                        default='l', help='Choose ro which direction the robot avoids human.')    
    parser.add_argument('--stairs', action='store_true', help='Move the robot in stairs mode.')
    parser.add_argument('--locomo', default=1, type=int, help='Select locomotion hint.')
    parser.add_argument('--mode', default='n', type=str, choices= ['s', 'd', 'n'],
                        help='Choose either admissive, domimant, or neutral')
    
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
    robot_image_client = robot.ensure_client(ImageClient.default_service_name)

    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Power on the robot and stand it up.
        robot.time_sync.wait_for_sync()
        robot.power_on()
        blocking_stand(robot_command_client)

        try:
            relative_move_traj(options.mode, options.dx, 0, math.radians(options.dyaw), options.frame, options.avoiding_dir,
                            robot_command_client, robot_state_client, robot_image_client) 
            #arm_look_direction(robot_command_client, options.dy)
            robot.logger.info("Robot looking a direction")
            # relative_move(options.mode, options.dx, options.dy, math.radians(options.dyaw), options.frame,
            #                 robot_command_client, robot_state_client)
            #relative_move(options.mode, options.dx, 0, math.radians(options.dyaw), options.frame, robot_command_client, robot_state_client)
        finally:
            # Close the gripper 
            robot_command_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
            # Stow the arm before sitting down 
            stow_arm(robot_command_client)
            robot.logger.info("Stow command issued.")
            # Send a Stop at the end, regardless of what happened.
            robot_command_client.robot_command(RobotCommandBuilder.stop_command())

def relative_move_traj(mode, dx, dy, dyaw, frame_name, avoiding_direction, robot_command_client, robot_state_client, robot_image_client, stairs=False):
    
    robot_cmd = relative_move_cmd(mode, dx, dy, dyaw, frame_name, robot_state_client, speed_limit = 0.5, speed_limit_ang = 0.1, obs_padding= 0.5)

    max_duration = 30.0
    time.sleep(0.5) # time before it starts walking 
    start_time = time.time()
    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                end_time_secs=start_time + max_duration)
    
    return robot_cmd, cmd_id, start_time

    sources = ['frontleft_depth', 'frontright_depth']
    
    not_avoided = True
    while True:
        feedback = robot_command_client.robot_command_feedback(cmd_id)
        # print("cmd_id", cmd_id)
        mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback

        # Capture and save images to disk
        image_responses = robot_image_client.get_image_from_sources(sources)

        #Image responses are in the same order as the requests.
        #Convert to opencv images.
        #image_response[0]-> depth from left camera, image_response[1]-> depth from right camera

        if len(image_responses) < 1:
            print('Error: failed to get images.')
            #return False
        
        #--- Coefficient of groundplane
        # [left, right]
        if mode == 'n':
            coeff = [[-0.10336304, -0.60649112, 0.7770675459889381], [0.24249883, -0.35360388, 0.5720567587076608]] 
        if mode == 's':
            coeff = [[-0.30305215, -0.49609337, 0.5819447951272856], [-0.08364196, -0.6345696, 0.7833940487540593]]
        if mode == 'd':
            coeff = [[-0.18850846, -0.29102495, 0.6098670007953881], [ 0.16862343, -0.2674438, 0.599150423229101 ]]
            
       # PCs = np.empty([0,3]) #PCs... Poiint cloud for both left and right camera. PCs[0] is left
        for i, image_response in enumerate(image_responses):
            PC = bosdyn.client.image.depth_image_to_pointcloud(image_response, min_dist=0.5, max_dist=2.5)
            
            #print(image_responses)
            #print("max, min", np.max(PC, axis= 0), np.min(PC, axis = 0))
            PC_noground= remove_ground(coeff[i], PC, 0.15, body_height=body_height)

        # Todo: Ideally use a proper transformation for getting center point cloud
            if i == 0:
                PC_center = PC_noground[PC_noground[:, 1]>0.7, : ]
            if i == 1:
                PC_center = PC_noground[PC_noground[:, 1]<-0.7, : ]
            
            print("#PC", len(PC_center))

            if len(PC_center)> 20:
                if not_avoided:
                    print("Object detected!!")
                    if avoiding_direction == 'l':
                        relative_move(mode, 2.1, 0.7, 0.2, frame_name, robot_command_client, robot_state_client, robot_image_client)
                    if avoiding_direction == 'r':
                        relative_move(mode, 2.1, -0.7, -0.2, frame_name, robot_command_client, robot_state_client, robot_image_client)

                    not_avoided = False
                    time.sleep(3)
                    # robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
                    #     goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
                    #     frame_name=frame_name, params=mobility_param, build_on_command=arm_command)

                    #Continue with the previous command after avoiding
                    cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
                                                 end_time_secs=start_time + max_duration)  
                    feedback = robot_command_client.robot_command_feedback(cmd_id)
                    mobility_feedback = feedback.feedback.synchronized_feedback.mobility_command_feedback
                    
        print("here")
        time.sleep(0.1)
        return True

            # else:
            #     sprint("--- Continuing straight ---")
            #     robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            #             goal_x=6, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
            #             frame_name=frame_name, params=mobility_param, build_on_command=arm_command)
            #     cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
            #                         end_time_secs=time.time() + end_time)  

                    #return True

            # robot_cmd = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            # goal_x=out_tform_goal.x, goal_y=out_tform_goal.y, goal_heading=out_tform_goal.angle,
            # frame_name=frame_name, params=mobility_param, build_on_command=arm_command)

            # end_time = 30.0
            # time.sleep(2) # time before it starts walking 
            # cmd_id = robot_command_client.robot_command(lease=None, command=robot_cmd,
            #                                     end_time_secs=time.time() + end_time)       
        


        # # velocity = robot_state_client.get_robot_state().kinematic_state.velocity_of_body_in_odom
        # # print("velocity", velocity)
        # if mobility_feedback.status != RobotCommandFeedbackStatus.STATUS_PROCESSING:
        #     print("Failed to reach the goal")
        #     return False
        #traj_feedback = mobility_feedback.se2_trajectory_feedback
        # if (traj_feedback.status == traj_feedback.STATUS_AT_GOAL and
        #         traj_feedback.body_movement_status == traj_feedback.BODY_STATUS_SETTLED):
        # #     print("Arrived at the goal.")
        #     return True



if __name__ == "__main__":
    if not main():
        sys.exit(1)
