#!/usr/bin/env python3
import rospy
from pose_estimation.msg import Coordinate

def keypoint_pub():
    p = PoseEstimation()
    rospy.init_node('keypoints', anonymous=True)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        p.capture()
        r.sleep()


if __name__ == '__main__':
    try:
        keypoint_pub()
    except rospy.ROSInterruptException: pass
