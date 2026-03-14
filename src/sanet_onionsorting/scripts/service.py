#!/usr/bin/env python3
import rclpy
import copy
import numpy as np
from time import time
from rclpy.node import Node
from sensor_msgs.msg import Image

from sanet_onionsorting.srv import Frcnn
from sanet_onionsorting.srv import Yolo
from sanet_onionsorting.srv import YoloDeepsort

class_dict = {'bo': 0, 'uo': 1, 'gl': 2, 'cb': 3, 'bn': 4, 'hd': 5}

class service(Node):
                                                                                
    def __init__(self): 

        super().__init__("service")

        t0 = time()
        weights = None
        detector = None
        self.y = None
        self.rgb_mem = None
        
        self.declare_parameters(
            namespace = '',
            parameters =[
                ('choice', 'real'),
                ('camera_name', 'oak'),
                ('deepsort', False),
                ('version', 'yolov7')
            ]
        )

        self.choice = self.get_parameter('choice').get_parameter_value().string_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.deepsort = self.get_parameter('deepsort').get_parameter_value().bool_value
        self.version = self.get_parameter('version').get_parameter_value().string_value

        self.get_logger().info('Chosen Service Started: ' + str(self.version))

        if (self.choice == "real"):

            print(f"Real {self.camera_name} camera selected")

            if (self.camera_name == "kinect"):
                self.subscription = self.create_subscription(Image, "/kinect2/hd/image_color_rect", self.grabrgb, 10)

            elif (self.camera_name == "realsense"):
                self.subscription = self.create_subscription(Image, "/camera/camera/color/image_raw", self.grabrgb, 10)

            elif (self.camera_name == 'oak'):
                self.subscription = self.create_subscription(Image, "/oak/rgb/image_raw", self.grabrgb, 10)

            elif (self.camera_name == 'test'):
                self.subscription = self.create_subscription(Image, "/kinect2/color/image_raw", self.grabrgb, 10)

            else:
                raise ValueError("Wrong camera name")
            
        elif (self.choice == "gazebo"):

            weights = "best_gazebokinect.pt"
            print(f"{weights} weights selected with gazebo")

            if (self.camera_name == "kinect"):
                self.subscription = self.create_subscription(Image, "/kinect_V2/rgb/image_raw", self.grabrgb, 10)

            elif (self.camera_name == "realsense"):
                self.subscription = self.create_subscription(Image, "/camera/color/image_raw", self.grabrgb, 10)

            else:
                raise ValueError("Wrong camera name")
            
        else:
            print(f"Unknown choice: {self.choice}. Please choose between real and gazebo.")
            
        if self.version != 'frcnn' and self.version != 'yolov5' and self.version != 'yolov7' and self.version != 'yolov8':
            self.get_logger().warn("Version provided is incorrect, defaulting to yolov8")
            self.version = 'yolov8'

        if self.version == 'frcnn':
            from thirdparty.faster_rcnn.detect import FasterRCNN
            weights = "converted_model.h5"
            self.y = FasterRCNN(weights, conf_thres = 0.95)  
            self.service = self.create_service(Frcnn, "/get_predictions", self.getpred_frcnn)
            print("Time taken for init: ", time() - t0)

        elif self.version == 'yolov5':
            from thirdparty.yolov5.detect import YOLO
            weights = "best_realkinect.pt"
            self.y = YOLO(weights, conf_thres = 0.75)
            self.service = self.create_service(Yolo, "/get_predictions", self.getpred_v5)
            print("Time taken for init: ", time() - t0)

        elif self.version == 'yolov7':
            from thirdparty.yolov7_deepsort.detection_helpers import Detector
            from thirdparty.yolov7_deepsort.bridge_wrapper import YOLOv7_DeepSORT
            from ament_index_python.packages import get_package_share_directory
            path = get_package_share_directory('sanet_onionsorting')
            
            weights = "best_realonionsonly_040104.pt"
            detector = Detector(conf_thres = 0.75, weightsfile = weights, classes = None) # it'll detect ONLY [person,horses,sports ball] = [0,17,32]. class = None means detect all classes. List info at: "data/coco.yaml"
            detector.load_model() # pass the path to the trained weight file
            self.y = YOLOv7_DeepSORT(reID_model_path= path + "/thirdparty/yolov7_deepsort/deep_sort/model_weights/mars-small128.pb", detector=detector, deepsort = self.deepsort)
            self.service = self.create_service(YoloDeepsort, "/get_predictions", self.getpred_v7)
            print("Time taken for init: ", time() - t0)

        elif self.version == 'yolov8':
            from thirdparty.yolov8.detect import YOLOV8
            weights = "best_realkinect.pt"
            self.y = YOLOV8(conf_thres = 0.75,)
            self.service = self.create_service(Yolo, "/get_predictions", self.getpred_v8)
            print("Time taken for init: ", time() - t0)

    def grabrgb(self, msg):

        if msg is not None:
            self.rgb_mem = copy.copy(msg)
            return
        else:
            return
        
    def getpred_frcnn(self, msg, response):
        start_time = time()
        centxs = []
        centys = []
        colors = []        
        if rgb_mem is not None: 
            if (self.camera_name == "realsense"):
                rgb = np.frombuffer(self.rgb_mem.data, dtype=np.uint8).reshape(self.rgb_mem.height, self.rgb_mem.width, 3)
                bgr = rgb[..., ::-1]
                self.rgb_mem.data = bgr.tobytes()

            output = self.y.detect(self.rgb_mem)
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
                response.centx, response.centy, response.color  = centxs, centys, colors
                return response
            else:
                self.get_logger().info("No Onion Detected")
                response.centx, response.centy, response.color  = [-1.0], [-1.0], [-1.0]
                return response
        else:
            self.get_logger().info("No RGB image received yet")
            response.centx, response.centy, response.color = [], [], []
            return response

    def getpred_v5(self, msg, response):
        start_time = time()
        centxs = []
        centys = []
        colors = []
        if self.rgb_mem is not None: 
            if (self.camera_name == "realsense"):
                rgb = np.frombuffer(self.rgb_mem.data, dtype=np.uint8).reshape(self.rgb_mem.height, self.rgb_mem.width, 3)
                bgr = rgb[..., ::-1]
                self.rgb_mem.data = bgr.tobytes()

            output = self.y.detect(self.rgb_mem)

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
            else: print("\nNo output from classifier received yet\n")
            rgb_mem = None
            print("\nTime taken by classifier is: ", time() - start_time)
            if len(centxs) > 0:
                response.centx, response.centy, response.color  = centxs, centys, colors
                return response
            else:
                self.get_logger().info("No Onion Detected")
                response.centx, response.centy, response.color  = [-1.0], [-1.0], [-1.0]
                return response
        else:
            self.get_logger().info("No RGB image received yet")
            response.centx, response.centy, response.color = [], [], []
            return response
        
    def getpred_v7(self, msg, response):
        start_time = time()
        centxs = []
        centys = []
        colors = []
        trackid = []
        if self.rgb_mem is not None: 
            if (self.camera_name == "realsense"):
                rgb = np.frombuffer(self.rgb_mem.data, dtype=np.uint8).reshape(self.rgb_mem.height, self.rgb_mem.width, 3)
                bgr = rgb[..., ::-1]  # Reverse the last axis to convert RGB to BGR
                self.rgb_mem.data = bgr.tobytes()

            output = self.y.track_consecutive_frames(frame=self.rgb_mem, verbose = 0) # verbose levels: 0,1,2

            if output is not None:
                if len(output) > 0:
                    if self.deepsort:
                        for (_, det) in output.items():  # output is a dictionary.
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
                print("\nNo output from yolo received yet\n")

            self.rgb_mem = None

            print("\nTime taken by yolo is: ", time() - start_time)
            print("##"*30)
            if len(centxs) > 0:
                response.centx, response.centy, response.color, response.trackid  = centxs, centys, colors, trackid
                return response
            else:
                self.get_logger().info("No Onion Detected")
                response.centx, response.centy, response.color, response.trackid  = [-1.0], [-1.0], [-1.0], [-1.0]
                return response

        else:
            self.get_logger().info("No RGB image received yet")
            response.centx, response.centy, response.color, response.trackid  = [], [], [], []
            return response
        
    def getpred_v8(self, msg, response):
        start_time = time()
        centxs = []
        centys = []
        colors = []
        trackid = []

        if self.rgb_mem is not None:
            # thisimage = np.frombuffer(rgb_mem.data, dtype=np.uint8).reshape(rgb_mem.height, rgb_mem.width, -1).astype('float32')
            # print("\nThis image shape: \n",np.shape(thisimage))
            # print('output:   ',output)

            # print("Time for rgb to bgr: ", t1-t0)
            if self.camera_name == "realsense":
                rgb = np.frombuffer(self.rgb_mem.data, dtype=np.uint8).reshape(self.rgb_mem.height, self.rgb_mem.width, 3)
                bgr = rgb[..., ::-1]  # Reverse the last axis to convert RGB to BGR
                self.rgb_mem.data = bgr.tobytes()

            output = self.y.detect(Image=self.rgb_mem)
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

            self.rgb_mem = None

            print("\nTime taken by yolo is: ", time() - start_time)
            print("##"*30)
            if len(centxs) > 0:
                response.centx, response.centy, response.color = centxs, centys, colors
                return response
            else:
                self.get_logger().info("No Onion Detected")
                response.centx, response.centy, response.color = [-1.0], [-1.0], [-1.0]
                return response

        else:
            self.get_logger().info("No RGB image received yet")
            response.centx, response.centy, response.color = [], [], []
            return response
         
def main(args=None):

    rclpy.init(args=args)
    try:
        node = service()
        rclpy.spin(node)
        rclpy.shutdown()

    except Exception as e:
        print('Combined Service Failed: %r' %(e,))

    except KeyboardInterrupt:
        print("  Keyboard Interrupt, shutting down")

if __name__ == "__main__":
    main()