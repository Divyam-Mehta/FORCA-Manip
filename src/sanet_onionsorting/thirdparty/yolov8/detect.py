#!/usr/bin/env python
'''
Author: Zhizheng Wang (zw45499@uga.edu)
Owner: THINC Lab @ CS UGA

@brief: This is a basic script for testing. Just gets the service results and prints it out.

Please make sure you provide credit if you are using this code.

'''

import sys
import os
import torch
from ultralytics import YOLO
import cv2
import numpy as np
import time
from ament_index_python.packages import get_package_share_directory
from thirdparty.yolov8.utils import letterbox

class YOLOV8:
    def __init__(self, weightsfile='best_realonions_yolov8.pt', conf_thres=0.25):
        #base_path = os.getcwd()
        # self.weights = os.path.join(base_path, 'weights', weightsfile)
        # self.source = os.path.join(base_path, 'inference/images', 'onion_test.JPG')
        # self.output = os.path.join(base_path, 'inference/output')  

        path = get_package_share_directory('sanet_onionsorting')
        self.weights = path + '/thirdparty/yolov8/weights/'+ weightsfile
        self.source = path + '/thirdparty/yolov8/inference/images/onion_test.JPG'
        self.output = path + '/thirdparty/yolov8/inference/output'
        self.img_size = 640
        self.conf_thres = conf_thres
        self.iou_thres = 0.3
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.view_img = False
        self.save_txt = True
        self.save_img = True
        self.classes = None
        self.agnostic_nms = False
        self.augment = False
        self.bounding_boxes = []
        self.model = YOLO(self.weights)
        self.class_name = self.model.names
        print("YOLOv8 initialized with weights:", self.weights)

    def detect(self, Image=None):
        out, source, weights, view_img, save_txt, save_img, imgsz = \
            self.output, self.source, self.weights, self.view_img, self.save_txt, self.save_img, self.img_size

        # Initialize
        self.bounding_boxes = []
        
        # Load source image if none is passed to detect function (JK)
        if Image is None:
            Image = cv2.imread(source)
            # JK: Double check that an Image is produced
            if Image is None:
                print("Path to image is incorrect")
                return None

        # Run inference
        t0 = time.time()

        # Added this to handle static images, gets numpy array error for static images (no height attribute) (JK)
        if isinstance(Image, np.ndarray):
            img = Image.astype('float32')
        else:
            img = np.frombuffer(Image.data, dtype=np.uint8).reshape(Image.height, Image.width, -1).astype('float32')

        if img is None:
            print("Error: Could not load image.")
            return None

        # Inference
        results = self.model.predict(source=img, conf=self.conf_thres, iou=self.iou_thres)

        for result in results:
            for box in result.boxes:
                xyxy = box.xyxy.cpu().numpy().tolist()  # Convert tensor to list
                prob = box.conf.cpu().item()            # Convert tensor to float
                cls = int(box.cls.cpu().item()) 
                
                #label = self.class_name[cls]

                # Combine xyxy, prob, and label into a single list
                self.bounding_boxes.append([*xyxy, prob, cls])
        # save view/save img
        if view_img or save_img:
            img_with_boxes = results[0].plot()
            
            if save_img:
                save_path = os.path.join(out, 'detected_onion_test.jpg')
                cv2.imwrite(save_path, img_with_boxes)
                
            if view_img:
                cv2.imshow('Detection', img_with_boxes)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration
                
        # save result_output.txt
        if save_txt:
            if not os.path.exists(out):
                os.makedirs(out)
            with open(os.path.join(out, 'result_output.txt'), 'w') as f:
                for bounding_box in self.bounding_boxes:
                    f.write(f'{bounding_box}\n')

        print(f"Detection completed. {len(self.bounding_boxes)} bounding boxes found.")
        return self.bounding_boxes

# Used for testing (JK)
'''path = get_package_share_directory('sanet_onionsorting')
image_path = path + "/thirdparty/yolov8/inference/images/onion_test.JPG"
image = cv2.imread(image_path)
detect = YOLOV8(conf_thres = 0.4)
detect.detect()'''

# if __name__ == '__main__':
#     yolo = YOLOV8()
#     yolo.detect()