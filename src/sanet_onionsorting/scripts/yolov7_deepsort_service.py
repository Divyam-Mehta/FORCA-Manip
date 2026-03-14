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
from sanet_onionsorting.srv import YoloDeepsort
from thirdparty.yolov7_deepsort.detection_helpers import Detector
from thirdparty.yolov7_deepsort.bridge_wrapper import YOLOv7_DeepSORT
# from yolov7_deepsort.tracking_helpers import *

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

        if (camera == "realsense"):
            # print("Switching to bgr!")
            rgb = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, 3)
            bgr = rgb[..., ::-1]  # Reverse the last axis to convert RGB to BGR
            rgb_mem.data = bgr.tobytes()

        # if deepsort:
            #if (camera == "realsense"):
            #rgb = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, 3)
            #bgr = rgb[..., ::-1]  # Reverse the last axis to convert RGB to BGR
            #rgb_mem.data = bgr.tobytes()

        output = y.track_consecutive_frames(frame=rgb_mem, verbose = 0) # verbose levels: 0,1,2

        # print("Time for yolo: ", t2-t1)
        
        # cv2.imshow("Output Video", image)   
        # cv2.waitKey(0) 
        # cv2.destroyAllWindows()

        if output is not None:
            if len(output) > 0:
                if deepsort:
                    for (_, det) in output.items():  # output is a dictionary.
                        print(det)
                        [xyxy, cls, track_id] = det   # Each det is a list.
                        ''' 
                        NOTE: Useful link: https://miro.medium.com/max/597/1*85uPFWLrdVejJkWeie7cGw.png
                        Kinect image resolution is (1920,1080)
                        But numpy image shape is (1080,1920) because np takes image in the order height x width.
                        '''
                        tlx, tly, brx, bry = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
                        centx, centy = (tlx+brx)/2, (tly+bry)/2
                        if (int(class_dict[cls]) == 0 or int(class_dict[cls]) == 1):
                            # print("\ntlx, tly, brx, bry, cls: ",tlx, tly, brx, bry, int(class_dict[cls]))
                            # print(f"\nCentroid: {centx}, {centy}")
                            centxs.append(centx)
                            centys.append(centy)
                            colors.append(int(class_dict[cls]))
                            trackid.append(track_id)
                        else:   pass
                else:
                    for *xyxy, conf, cls in reversed(output):
                        tlx, tly, brx, bry = xyxy[0], xyxy[1], xyxy[2], xyxy[3]
                        centx, centy = (tlx+brx)/2, (tly+bry)/2
                        if (int(cls) == 0 or int(cls) == 1):
                            # print("\ntlx, tly, brx, bry, cls: ",tlx, tly, brx, bry, int(class_dict[cls]))
                            # print(f"\nCentroid: {centx}, {centy}")
                            centxs.append(centx)
                            centys.append(centy)
                            colors.append(int(cls))
                            trackid.append(0)
                        else:   pass
            else: pass   
        else: print("\nNo output from yolo received yet\n")
        rgb_mem = None

        # print("Time for adding cents: ", time()-t2)

        print("\nTime taken by yolo is: ", time() - start_time)
        if len(centxs) > 0:
            response.centx, response.centy, response.color, response.trackid  = centxs, centys, colors, trackid
            return response
        
        else:
            print("\nNo onions detected in frame\n")
            response.centx, response.centy, response.color, response.trackid  = [-1.0], [-1.0], [-1.0], [-1.0]
            return response
        
    else:
        print("\nNo RGB image received yet\n")
        response.centx, response.centy, response.color, response.trackid = [], [], [], []
        return response


def main(args=None):
    global y, weights, camera, detector, deepsort

    rclpy.init(args=args)
    try:
        t0 = time()
        node = Node("yolo_v7_service")
        node.get_logger().info("Yolo_v7_deepsort service started")

        node.declare_parameters(
            namespace = '',
            parameters =[
                ('choice', 'real'),
                ('camera_name', 'oak'),
                ('deepsort', False)
            ]
        )

        choice = node.get_parameter('choice').get_parameter_value().string_value
        camera_name = node.get_parameter('camera_name').get_parameter_value().string_value
        deepsort = node.get_parameter('deepsort').get_parameter_value().bool_value

        if (choice == "real"):

            weights = "best_realonionsonly_040104.pt"
            print(f"{weights} weights selected with real {camera_name} camera")

            if (camera_name == "kinect"):
                node.subscription = node.create_subscription(Image, "/kinect2/hd/image_color_rect", grabrgb, 10)

            elif (camera_name == "realsense"):
                node.subscription = node.create_subscription(Image, "/camera/color/image_raw", grabrgb, 10)

            elif (camera_name == "test"):
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
        
        from ament_index_python.packages import get_package_share_directory
        path = get_package_share_directory('sanet_onionsorting')
        camera = camera_name
        detector = Detector(conf_thres = 0.75, weightsfile = weights, classes = None) # it'll detect ONLY [person,horses,sports ball] = [0,17,32]. class = None means detect all classes. List info at: "data/coco.yaml"
        detector.load_model() # pass the path to the trained weight file
        y = YOLOv7_DeepSORT(reID_model_path= path + "/thirdparty/yolov7_deepsort/deep_sort/model_weights/mars-small128.pb", detector=detector, deepsort = deepsort)
        node.service = node.create_service(YoloDeepsort, "/get_predictions", getpred)
        print("Time taken for init: ", time() - t0)

        rclpy.spin(node)
        rclpy.shutdown

    except Exception as e:
        print('Yolo_V7 Service Failed: %r' %(e,))

    except KeyboardInterrupt:
        print("  Keyboard Interrupt, shutting down")

if __name__ == '__main__':    
    main()