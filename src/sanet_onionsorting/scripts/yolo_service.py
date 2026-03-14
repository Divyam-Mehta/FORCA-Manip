#!/usr/bin/env python3
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
from thirdparty.yolov5.detect import YOLO

same_flag = 0
rgb_mem = None
depth_mem = None
weights = None
camera = None
y = None

def grabrgb(msg):

    global rgb_mem
    if msg is not None:
        rgb_mem = copy.copy(msg)
    else:
        return

def getpred(msg, response):
    global y, rgb_mem, depth_mem
    # print("Entered getpred func")
    # start_time = time()
    centxs = []
    centys = []
    colors = []
    if rgb_mem is not None: 
        # thisimage = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, -1).astype('float32')
        # print("\nThis image shape: \n",np.shape(thisimage))
        # print('output:   ',output)

        # t0 = time()
        if (camera == "realsense"):
            # print("Switching to bgr!")
            rgb = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, 3)
            bgr = rgb[..., ::-1]  # Reverse the last axis to convert RGB to BGR
            rgb_mem.data = bgr.tobytes()
        # t1 = time()

        # print("Time for rgb to bgr: ", t1-t0)

        output = y.detect(rgb_mem)

        # t2 = time()

        # print("Time for yolo: ", t2-t1)

        if output is not None:
            if len(output) > 0:   
                for det in output:
                    for *xyxy, conf, cls in det:
                        ''' 
                        NOTE: Useful link: https://miro.medium.com/max/597/1*85uPFWLrdVejJkWeie7cGw.png
                        Kinect image resolution is (1920,1080)
                        But numpy image shape is (1080,1920) becasue np takes image in the order height x width.
                        '''
                        tlx, tly, brx, bry = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
                        centx, centy = (tlx+brx)/2, (tly+bry)/2
                        if (int(cls) == 0 or int(cls) == 1):
                            print("\ntlx, tly, brx, bry, cls: ", tlx, tly, brx, bry, cls)
                            print(f"\nCentroid: {centx}, {centy}")
                            centxs.append(centx)
                            centys.append(centy)
                            colors.append(cls)
                        else:   pass
            else: pass   
        else: print("\nNo output from yolo received yet\n")
        rgb_mem = None

        # print("Time for adding cents: ", time()-t2)

        # print("\nTime taken by yolo is: ", time() - start_time)
        if len(centxs) > 0:
            # print(f"\nFound {len(centxs)} onions\n")
            response.centx, response.centy, response.color  = centxs, centys, colors
            return response
        else:
            print("\nNo onions detected in frame\n")
            response.centx, response.centy, response.color  = [-1.0], [-1.0], [-1.0]
            return response
    else:
        print("\nNo RGB image received yet\n")
        response.centx, response.centy, response.color = [], [], []
        return response


def main(args=None):
    global y, weights, camera

    rclpy.init()
    try:
        t0 = time()
        node = Node("yolo_service")
        node.get_logger().info("Yolo service started")

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
        y = YOLO(weights, conf_thres = 0.75)
        node.service = node.create_service(Yolo, "/get_predictions", getpred)
        print("Time taken for init: ", time() - t0)

        rclpy.spin(node)
        rclpy.shutdown()

    except Exception as e:
        print('Yolo Service Failed: %r' %(e,))

    except KeyboardInterrupt:
        print("  Keyboard Interrupt, shutting down")

if __name__ == '__main__':    
    main()