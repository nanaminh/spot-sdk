import numpy as np
import sys
import math

from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from google.protobuf import wrappers_pb2
from bosdyn.api import arm_command_pb2

def walk_looking_around(duration, nominal_pose):
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
    joint_positions[5] = nominal_pose[5] + 1.0 * math.cos(4 * w * t)

    # Take the derivative of our position trajectory to get our velocities
    joint_velocities = [0, 0, 0, 0, 0, 0]
    #joint_velocities[0] = -0.8 * w * math.sin(w * t)
    #joint_velocities[2] = 0.2 * 2 * w * math.cos(2 * w * t)
    joint_velocities[4] = 0.5 * 2 * w * math.cos(2 * w * t)
    joint_velocities[5] = -1.0 * 4 * w * math.sin(4 * w * t)

    # Return the joint positions and velocities at time t in our trajectory
    return joint_positions, joint_velocities



def main():

if __name__ == "__main__":
    if not main():
        sys.exit(1)