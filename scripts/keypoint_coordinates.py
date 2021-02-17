import cv2
import rospy

from pose_estimation.msg import Coordinate


class KeypointCoordinates(object):
    
    def __init__(self, keypoint_names):
        self.keypoint_names = keypoint_names
    
    def __find_forehead(self, keypoints):
        if "nose" in keypoints:
            if "right_eye" in keypoints:
                diff = (keypoints["nose"][1] - keypoints["right_eye"][1])
                y = keypoints["right_eye"][1] - diff
            elif "left_eye" in keypoints:
                diff = keypoints["nose"][1] - keypoints["left_eye"][1]
                y = keypoints["left_eye"][1] - diff
            else:
                print("Can't find forehead")
                return
            keypoints["forehead"] = [keypoints["nose"][0], y]
        print("Can't find forehead")

    def __find_chest(self, keypoints):
        keypoints["chest"] = keypoints["neck"]

    def __find_hand(self, keypoints):
        targets = ["right_wrist", "left_wrist", "right_elbow", "left_elbow"]
        for target in targets:
            if target in keypoints:
                keypoints["hand"] = keypoints[target]
                return
        print("Can't find hand")

        
    def __call__(self, image, object_counts, objects, normalized_peaks):
        keypoints = {}
        height = image.shape[0]
        width = image.shape[1]

        count = int(object_counts[0])
        for i in range(count):
            color = (0, 255, 0)
            obj = objects[0][i]
            C = obj.shape[0]
            for j in range(C):
                k = int(obj[j])
                if k >= 0:
                    peak = normalized_peaks[0][j][k]
                    x = round(float(peak[1]) * width)
                    y = round(float(peak[0]) * height)
                    cv2.circle(image, (x, y), 3, color, 2)
                    keypoints[self.keypoint_names[j]] = [x, y]
        
        __find_chest()
        __find_forehead()
        __find_hand()
        return keypoints
