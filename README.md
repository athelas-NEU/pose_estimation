# pose_estimation

Pose estimation using the [trt_pose](https://github.com/NVIDIA-AI-IOT/trt_pose) library.

## Setup
```
cd ~/athelas_ws/src
git clone https://github.com/athelas-NEU/pose_estimation.git
cd ..
catkin_make
```

## get_keypoint Service

The get_keypoint ROS service returns the offset from the center of the frame of the keypoint requested.

The service interface is defined as:
```
string location
---
int32 x
int32 y
```
The magnitude of the offset is defined in pixel coordinates. The sign of the offet is:
- positive x: go right
- negative x: go left
- negative y: go down
- positive y: go up

To start the service run:
```
cd ~/catkin_ws
rosrun pose_estimation get_keypoint_service.py
```
