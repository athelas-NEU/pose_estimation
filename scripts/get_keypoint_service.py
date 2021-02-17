#!/usr/bin/env python3

import rospy

from pose_estimation_model import PoseEstimation

def get_keypoint_server():
    rospy.init_node('get_keypoint_server')
    p = PoseEstimation()
    print("Ready for locations.")
    rospy.spin()

if __name__ == "__main__":
    get_keypoint_server()
