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
from sanet_onionsorting.srv import Frcnn
from rclpy.node import Node
from sensor_msgs.msg import Image
from thirdparty.faster_rcnn.detect import FasterRCNN

'''test with cam, need weights?'''

same_flag = 0
rgb_mem = None
depth_mem = None
weights = None
camera = None

def grabrgb(msg):

    global rgb_mem
    if msg is not None:
        rgb_mem = copy.copy(msg)
    else:
        return

def getpred(msg, response):
    global weights, rgb_mem, depth_mem, camera
    # print("Entered getpred func")
    start_time = time()
    centxs = []
    centys = []
    colors = []
    f = FasterRCNN(weights, conf_thres = 0.95)            
    if rgb_mem is not None: 
        # thisimage = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, -1).astype('float32')
        # print("\nThis image shape: \n",np.shape(thisimage))
        # print('output:   ',output)

        if (camera == "realsense"):
            rgb = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, 3)
            bgr = rgb[..., ::-1]  # Reverse the last axis to convert RGB to BGR
            rgb_mem.data = bgr.tobytes()

        output = f.detect(rgb_mem)
        if output is not None:
            if len(output) > 0:   
                for det in output:
                    [box,score,label] = det
                    # print(f"Detection: {det}" )
                    # for [boxes, scores, labels] in det:
                    ''' 
                    NOTE: Useful link: https://miro.medium.com/max/597/1*85uPFWLrdVejJkWeie7cGw.png
                    Kinect image resolution is (1920,1080)
                    But numpy image shape is (1080,1920) becasue np takes image in the order height x width.
                    '''
                    tlx, tly, brx, bry = box[0], box[1], box[2], box[3]
                    centx, centy = (tlx+brx)/2, (tly+bry)/2
                    if (int(label) == 0 or int(label) == 1):
                        # print("\ntlx, tly, brx, bry, label: ",tlx, tly, brx, bry, int(label))
                        # print(f"\nCentroid: {centx}, {centy}")
                        centxs.append(centx)
                        centys.append(centy)
                        colors.append(label)
                    else:   pass
            else: pass   
        else: print("\nNo output from classifier received yet\n")
        rgb_mem = None
        print("\nTime taken by classifier is: ", time() - start_time)
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
    global weights, camera

    rclpy.init(args=args)
    #try:
    node = Node("frcnn_service")
    node.get_logger().info("frcnn service started")

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
        weights = "converted_model.h5"
        # for kinect v2
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
        # for kinect v2
        # rospy.Subscriber("/kinect2/hd/points", Image, grabdepth)
    elif (choice == "gazebo"):
        weights = "best_gazebokinect.pt"
        # for kinect gazebo
        print(f"{weights} weights selected with gazebo")
        if (camera_name == "kinect"):
            node.subscription = node.create_subscription(Image, "/kinect_V2/rgb/image_raw", grabrgb, 10)

        elif (camera_name == "realsense"):
            node.subscription = node.create_subscription(Image, "/camera/color/image_raw", grabrgb, 10)

        else:
            raise ValueError("Wrong camera name")
        # for kinect gazebo
        # rospy.Subscriber("/kinect_V2/depth/points", Image, grabdepth)
    else:
        print(f"Unknown choice: {choice}. Please choose between real and gazebo.")

    node.service = node.create_service(Frcnn, "/get_predictions", getpred)

    rclpy.spin(node)
    rclpy.shutdown

    '''except Exception as e:
        print('frcnn Service Failed: %r' %(e,))

    except KeyboardInterrupt:
        print("  Keyboard Interrupt, shutting down")'''

if __name__ == '__main__':    
    main()