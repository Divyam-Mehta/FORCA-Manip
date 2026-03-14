#!/usr/bin/env python3
# coding: utf8
'''
Author: Prasanth Suresh (ps32611@uga.edu)
Owner: THINC Lab @ CS UGA

Please make sure you provide credit if you are using this code.

'''
import copy
import rclpy
import numpy as np
from time import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from sanet_onionsorting.srv import Yolo
from thirdparty.yolov8.detect import YOLOV8

same_flag = 0
rgb_mem = None
depth_mem = None
weights = None
camera = None
y = None
detector = None
deepsort = True

class_dict = {'bo': 0, 'uo': 1, 'gl': 2, 'cb': 3, 'bn': 4, 'hd': 5}

def grabrgb(msg):

    global rgb_mem
    if msg is not None:
        rgb_mem = copy.copy(msg)
        return
    else:
        return

def getpred(msg, response):
    global y, rgb_mem, depth_mem, deepsort, camera
    # print("Entered getpred func")
    start_time = time()
    centxs = []
    centys = []
    colors = []
    trackid = []
    if rgb_mem is not None:
        # thisimage = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, -1).astype('float32')
        # print("\nThis image shape: \n",np.shape(thisimage))
        # print('output:   ',output)

        # print("Time for rgb to bgr: ", t1-t0)
        if camera == "realsense":
            rgb = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, 3)
            bgr = rgb[..., ::-1]  # Reverse the last axis to convert RGB to BGR
            rgb_mem.data = bgr.tobytes()

        output = y.detect(Image=rgb_mem)
        # print("OUTPUT :::::: ", output)
        if output is not None:
            if len(output) > 0:
                for *xyxy, conf, cls in reversed(output):
                    xyxy = xyxy[0]
                    tlx, tly, brx, bry = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
                    centx, centy = (tlx+brx)/2, (tly+bry)/2
                    if (int(cls) == 0 or int(cls) == 1):
                        print("\ntlx, tly, brx, bry, cls: ", tlx, tly, brx, bry, cls)
                        print(f"\nCentroid: {centx}, {centy}")
                        centxs.append(centx)
                        centys.append(centy)
                        colors.append(int(cls))
                        trackid.append(0)
                    else:   pass
            else:
                pass
        else:
            print("\nNo output from yolo received yet\n")

        rgb_mem = None

        print("\nTime taken by yolo is: ", time() - start_time)
        print("##"*30)
        if len(centxs) > 0:
            response.centx, response.centy, response.color  = centxs, centys, colors
            return response
        else:
            print("\nNo onions detected in frame\n")
            response.centx, response.centy, response.color  = [-1.0], [-1.0], [-1.0]
            return response

    else:
        print("\nNo RGB image received yet\n")
        response.centx, response.centy, response.color  = [], [], []
        return response

def main():
    global y, weights, camera, detector, deepsort

    rclpy.init()
    # try:
    t0 = time()
    node = Node("yolo_v8_service")
    node.get_logger().info("Yolo_V8 service started")

    node.declare_parameters(
        namespace = '',
        parameters =[
            ('choice', 'real'),
            ('camera_name', 'oak')
        ]
    )

    choice = node.get_parameter('choice').get_parameter_value().string_value
    camera_name = node.get_parameter('camera_name').get_parameter_value().string_value

    if (choice == "real"):

        weights = "best_realkinect.pt"
        print(f"{weights} weights selected with real {camera_name} camera")

        if (camera_name == "kinect"):
            node.subscription = node.create_subscription(Image, "/kinect2/hd/image_color_rect", grabrgb, 10)

        elif (camera_name == "realsense"):
            node.subscription = node.create_subscription(Image, "/camera/color/image_raw", grabrgb, 10)

        elif (camera_name == 'test'):
            node.subscription = node.create_subscription(Image, "/kinect2/color/image_raw", grabrgb, 10)

        elif (camera_name == 'oak'):
            node.subscription = node.create_subscription(Image, "/oak/rgb/image_raw", grabrgb, 10)

        else:
            raise ValueError("Wrong camera name")
        
    elif (choice == "gazebo"):

        weights = "best_gazebokinect.pt"
        print(f"{weights} weights selected with gazebo")

        if (camera_name == "kinect"):
            node.subscription = node.create_subscription(Image, "/kinect_V2/rgb/image_raw", grabrgb, 10)

        elif (camera_name == "realsense"):
            node.subscription = node.create_subscription(Image, "/camera/color/image_raw", grabrgb, 10)

        else:
            raise ValueError("Wrong camera name")

    else:
        print(f"Unknown choice: {choice}. Please choose between real and gazebo.")
        
    camera = camera_name
    y = YOLOV8(conf_thres = 0.75,)
    node.service = node.create_service(Yolo, "/get_predictions", getpred)
    print("Time taken for init: ", time() - t0)

    rclpy.spin(node)
    rclpy.shutdown

    # except Exception as e:
    #     print('Yolo_V8 Service Failed: %r' %(e,))

    # except KeyboardInterrupt:
    #     print("  Keyboard Interrupt, shutting down")
    
if __name__ == '__main__':    
    main()