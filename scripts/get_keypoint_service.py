#!/usr/bin/env python3

import rospy

from pose_estimation_model import PoseEstimation

# To terminate all nodes:
# rosnode kill -a

def shutdown():
  p.terminate()
  print("shutting down")

def get_keypoint_server():
  rospy.init_node('get_keypoint_server')
  rospy.on_shutdown(shutdown)
  global p
  p = PoseEstimation()
  print("Ready for locations.")
  rospy.spin()

if __name__ == "__main__":
  get_keypoint_server()
