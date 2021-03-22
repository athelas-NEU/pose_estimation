import json
import trt_pose.coco
import torch
from torch2trt import TRTModule
import cv2
import torchvision.transforms as transforms
import PIL.Image
from trt_pose.parse_objects import ParseObjects
import matplotlib.pyplot as plt

from jetcam.csi_camera import CSICamera
from jetcam.utils import bgr8_to_jpeg

import time

import rospy
from keypoint_coordinates import KeypointCoordinates
from main_node.srv import GetKeypoint, GetKeypointResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class PoseEstimation(object):

    OPTIMIZED_MODEL = '/home/athelas/athelas_ws/src/pose_estimation/data/resnet18_baseline_att_224x224_A_epoch_249_trt.pth'
    WIDTH = 224
    HEIGHT = 224
    HUMAN_POSE = '/home/athelas/athelas_ws/src/pose_estimation/data/human_pose.json'

    def __init__(self, display_widget=None):
        self.display_widget = display_widget
        
        with open(self.HUMAN_POSE, 'r') as f:
            human_pose = json.load(f)

        topology = trt_pose.coco.coco_category_to_topology(human_pose)

        self.model_trt = TRTModule()
        self.model_trt.load_state_dict(torch.load(self.OPTIMIZED_MODEL))

        self.parse_objects = ParseObjects(topology)
        self.keypoint_coordinates = KeypointCoordinates(human_pose["keypoints"])

        self.camera = CSICamera(width=self.WIDTH, height=self.HEIGHT, capture_fps=30)
        self.camera.running = True
        
        if self.display_widget is None:
            self.display = plt.imshow(self.camera.value)
            plt.ion()
            plt.show()

        # ROS stuff
        s = rospy.Service('get_keypoint', GetKeypoint, self.__handle_get_keypoint)
        self.image_pub = rospy.Publisher("image", Image)
        self.bridge = CvBridge()

    def __handle_get_keypoint(self, req):
        keypoints = self.capture()
        if req.location in keypoints:
            coord = keypoints[req.location]
        else:
            coord = [-999, -999]
        return GetKeypointResponse(coord[0], coord[1])

    def __preprocess(self, image):
        mean = torch.Tensor([0.485, 0.456, 0.406]).cuda()
        std = torch.Tensor([0.229, 0.224, 0.225]).cuda()
        device = torch.device('cuda')
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = PIL.Image.fromarray(image)
        image = transforms.functional.to_tensor(image).to(device)
        image.sub_(mean[:, None, None]).div_(std[:, None, None])
        return image[None, ...]

    def __execute(self, change):
        image = change['new']
        data = self.__preprocess(image)
        cmap, paf = self.model_trt(data)
        cmap, paf = cmap.detach().cpu(), paf.detach().cpu()
        counts, objects, peaks = self.parse_objects(cmap, paf)
        keypoints = self.keypoint_coordinates(image, counts, objects, peaks)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
        if self.display_widget:
            self.display_widget.value = bgr8_to_jpeg(image[:, ::-1, :])
        else:
            self.display.set_data(image[:,:,::-1])
            plt.pause(0.000001)
        return keypoints

    def start_stream(self):
        self.camera.observe(self.__execute, names='value')

    def stop_stream(self):
        self.camera.unobserve_all()
    
    def capture(self):
        return self.__execute({'new': self.camera.value})
    
    def terminate(self):
        self.camera.close()


