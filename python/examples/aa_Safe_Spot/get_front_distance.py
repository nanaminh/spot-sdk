# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Example demonstrating capture of both visual and depth images and then overlaying them."""

import argparse
import sys

import cv2
import numpy as np

import bosdyn.client
import bosdyn.client.util
from bosdyn.client.image import ImageClient
import matplotlib.pyplot as plt
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand

def main(argv):
    # Parse args
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    # parser.add_argument('--to-depth',
    #                     help='Convert to the depth frame. Default is convert to visual.',
    #                     action='store_true')
    # parser.add_argument('--camera', help='Camera to acquire image from.', default='frontleft',\
    #                     choices=['frontleft', 'frontright', 'left', 'right', 'back',
    #                     ])
    parser.add_argument('--auto-rotate', help='rotate right and front images to be upright',
                        action='store_true', default=True)
    options = parser.parse_args(argv)



    # if options.to_depth:
    #     sources = [options.camera + '_depth', options.camera + '_visual_in_depth_frame'] # Source: which camera, in visual in depth frame
    # else:
    #     sources = [options.camera + '_depth_in_visual_frame', options.camera + '_fisheye_image']
    sources = ['frontleft_depth']#, 'frontright_depth']

    # Create robot object with an image client.
    sdk = bosdyn.client.create_standard_sdk('image_depth_plus_visual')
    robot = sdk.create_robot(options.hostname)
    bosdyn.client.util.authenticate(robot)
    image_client = robot.ensure_client(ImageClient.default_service_name)

    #Make robot stand fitst 
    # command_client = robot.ensure_client(RobotCommandClient.default_service_name)
    # blocking_stand(command_client, timeout_sec=10)
    # #Make robot stand first 
    # footprint_R_body = bosdyn.geometry.EulerZXY(yaw=0.0, roll=0.0, pitch=0.1)
    # cmd = RobotCommandBuilder.synchro_stand_command(footprint_R_body=footprint_R_body)
    # command_client.robot_command(cmd)

    # Capture and save images to disk
    image_responses = image_client.get_image_from_sources(sources)

    #Image responses are in the same order as the requests.
    #Convert to opencv images.
    #image_response[0]-> depth from left camera, image_response[1]-> depth from right camera

    if len(image_responses) < 1:
        print('Error: failed to get images.')
        return False
    
    PCs = np.empty([0,3])
    for image_response in image_responses:
        PC = bosdyn.client.image.depth_image_to_pointcloud(image_response, min_dist=0.5, max_dist=2.0)
        print(PC)
        PCs = np.vstack((PCs, PC))
    print("PCs", PCs)
    print("max, min", np.max(PCs, axis= 0), np.min(PCs, axis = 0))


    file= open("left_ground_pitch-0.1.txt", "w+") 
    for line in PCs:
        content = str(line) + "\n"
        file.write(content)
    file.close()

    
    # Visualize the point clouds
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(xs= PCs[:, 0], ys=PCs[:, 1], zs=PCs[:, 2])

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    #plt.scatter(x= PCs[:, 0], y=PCs[:, 1]) #, zs=PCs[:, 2])

    plt.show()

    # # Depth is a raw bytestream
    # cv_depth = np.frombuffer(image_responses[0].shot.image.data, dtype=np.uint16)
    # cv_depth = cv_depth.reshape(image_responses[0].shot.image.rows,
    #                             image_responses[0].shot.image.cols)

    # # # Visual is a JPEG
    # cv_visual = cv2.imdecode(np.frombuffer(image_responses[1].shot.image.data, dtype=np.uint8), -1)

    # # Convert the visual image from a single channel to RGB so we can add color
    # visual_rgb = cv_visual if len(cv_visual.shape) == 3 else cv2.cvtColor(
    #     cv_visual, cv2.COLOR_GRAY2RGB)

    # # Map depth ranges to color

    # # # cv2.applyColorMap() only supports 8-bit; convert from 16-bit to 8-bit and do scaling
    # min_val = np.min(cv_depth)
    # max_val = np.max(cv_depth)
    # depth_range = max_val - min_val
    # depth8 = (255.0 / depth_range * (cv_depth - min_val)).astype('uint8')
    # depth8_rgb = cv2.cvtColor(depth8, cv2.COLOR_GRAY2RGB)
    # depth_color = cv2.applyColorMap(depth8_rgb, cv2.COLORMAP_JET)

    # # # Add the two images together.
    # # out = cv2.addWeighted(visual_rgb, 0.5, depth_color, 0.5, 0)
    # out = depth_color

    # if options.auto_rotate:
    #     if image_responses[0].source.name[0:5] == "front":
    #         out = cv2.rotate(out, cv2.ROTATE_90_CLOCKWISE)

    #     elif image_responses[0].source.name[0:5] == "right":
    #         out = cv2.rotate(out, cv2.ROTATE_180)

    # # # Write the image out.
    # filename = "depth.jpg"
    # cv2.imwrite(filename, out)

    return True


if __name__ == "__main__":
    if not main(sys.argv[1:]):
        sys.exit(1)
