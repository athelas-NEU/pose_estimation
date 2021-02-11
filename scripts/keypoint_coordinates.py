import cv2


class KeypointCoordinates(object):
    
    def __init__(self, keypoint_names):
        self.keypoint_names = keypoint_names
        
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
        
        return keypoints
