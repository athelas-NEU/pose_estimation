import cv2

class KeypointCoordinates(object):
    
    def __init__(self, keypoint_names):
        self.keypoint_names = keypoint_names
    
    def __find_forehead(self, keypoints, image):
        if "nose" in keypoints:
            if "right_eye" in keypoints:
                diff = (keypoints["nose"][1] - keypoints["right_eye"][1])
                y = max(keypoints["right_eye"][1] - diff, 0)
            elif "left_eye" in keypoints:
                diff = keypoints["nose"][1] - keypoints["left_eye"][1]
                y = max(keypoints["left_eye"][1] - diff, 0)
            else:
                y = 0
            keypoints["forehead"] = [keypoints["nose"][0], y]
        elif "neck" in keypoints:
            keypoints["forehead"] = [keypoints["neck"][0], 0]
        else:
            print("Can't find forehead")
            return
        color = (255, 0, 0)
        cv2.circle(image, (keypoints["forehead"][0], keypoints["forehead"][1]), 3, color, 2)
        return
        

    def __find_chest(self, keypoints, image):
        print("find chest")
        if "neck" in keypoints:
            keypoints["chest"] = keypoints["neck"]
            color = (255, 0, 0)
            cv2.circle(image, (keypoints["chest"][0], keypoints["chest"][1]), 3, color, 2)
            return
        print("Cannot find chest")

    def __find_hand(self, keypoints, image):
        color = (255, 0, 0)
        targets = ["right_wrist", "left_wrist", "right_elbow", "left_elbow", "right_shoulder", "left_shoulder"]
        for target in targets:
            if target in keypoints:
                if target in ["right_shoulder", "left_shoulder"]:
                    keypoints["hand"] = [keypoints[target][0], 224]
                else:
                    keypoints["hand"] = keypoints[target]
                cv2.circle(image, (keypoints["hand"][0], keypoints["hand"][1]), 3, color, 2)
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
        
        self.__find_chest(keypoints, image)
        self.__find_forehead(keypoints, image)
        self.__find_hand(keypoints, image)

        # Make (0, 0) the center
        for name, coord in keypoints.items():
            keypoints[name] = [coord[0] - 112, coord[1] - 112]
        return keypoints
