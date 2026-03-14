#!/usr/bin/env python3
# coding: utf8
'''
This file was modified from the original in order to get it working
with ROS by Prasanth Suresh(ps32611@uga.edu).
Please make sure you provide credit if you are using this code.
'''
import sys
# from tensorflow import keras
from ament_index_python.packages import get_package_share_directory
path = get_package_share_directory('sanet_onionsorting')
sys.path.append(path + '/thirdparty/')
from faster_rcnn.keras_retinanet import models
from faster_rcnn.keras_retinanet.utils.image import preprocess_image, resize_image
from faster_rcnn.keras_retinanet.utils.visualization import draw_box, draw_caption
from faster_rcnn.keras_retinanet.utils.colors import label_color
# from keras_retinanet.utils.gpu import setup_gpu

# import miscellaneous modules
# import matplotlib.pyplot as plt
import cv2
import os
import numpy as np
import time
from copy import copy

# set tf backend to allow memory to grow, instead of claiming everything
import tensorflow as tf

#setup_gpu(gpu)
os.environ['CUDA_VISIBLE_DEVICES'] = "0"
os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true'

################################################################################
########## Annotate images (add bounding boxes and labels) #####################
################################################################################

# Use to depict input test image

class FasterRCNN():
    def __init__(self, weightsfile = 'converted_model.h5', conf_thres = 0.85):

        path = get_package_share_directory('sanet_onionsorting')
        self.model_path = path + '/thirdparty/faster_rcnn/examples/'+ weightsfile
        self.output = path + '/thirdparty/faster_rcnn/output/'
        self.model = models.load_model(self.model_path, backbone_name='resnet50') # load retinanet model
        self.bounding_boxes = []
        self.labels_to_names = {0: 'b', 1: 'u'} ##unused-->  , 2: 'g', 3: 'c', 4: 'bi', 5: 'h'}  # load label to names mapping for visualization purposes
        self.conf_thres = conf_thres

    def detect(self, Image = None):

        self.bounding_boxes = []    # Just making sure it's empty before we start

        # load image
        image = np.frombuffer(Image.data, dtype=np.uint8).reshape(Image.height, Image.width, -1).astype('float32')     # Added by Prasanth Suresh

        # copy to draw on
        draw = copy(image)

        # preprocess image for network
        image = preprocess_image(image)
        image, scale = resize_image(image,min_side=1080,max_side=1920)

        
        # process image
        start = time.time()
        boxes, scores, labels = self.model.predict_on_batch(np.expand_dims(image, axis=0))
        print("processing time: ", time.time() - start)
        

        # correct for image scale
        boxes /= scale

        prev_labels = []
        for box, score, label in zip(boxes[0], scores[0], labels[0]):
            # scores are sorted so we can break
            # print (label,score,box)

            if score < self.conf_thres:
                break
            
            self.bounding_boxes.append([box, score, label])

            prev_labels.append(label)
            
            color = label_color(label)

            b = box.astype(int)
            draw_box(draw, b, color=color)

            caption = "{} {:.3f}".format(self.labels_to_names[label], score)
            draw_caption(draw, b, caption)
 
        out_dir = self.output + 'output.jpg'
        draw = cv2.cvtColor(draw, cv2.COLOR_RGB2BGR)
        cv2.imwrite(out_dir, draw)
        return self.bounding_boxes
