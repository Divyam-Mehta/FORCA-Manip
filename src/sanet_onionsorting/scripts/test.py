#!/usr/bin/env python3
import sys
import copy
import rclpy
import cv2
from sanet_onionsorting.srv import Yolo
from rclpy.node import Node
from sensor_msgs.msg import Image
from thirdparty.yolov5.detect import YOLO

class Test(Node):

    def __init__(self):
        super().__init__('test')
        weights = "best_realkinect.pt"
        from ament_index_python import get_package_share_directory
        path = get_package_share_directory('sanet_onionsorting')
        image_path = path + "/thirdparty/yolov5/inference/images/bus.jpg"
        image = cv2.imread(image_path)
        y = YOLO(weights, conf_thres = 0.75)
        y.detect(image)
        if y is not None:
            self.get_logger().info("Success, check output")
        else:
            self.get_logger().warn("Failure, try again")
        #rgb_mem = Image()
        #image = 'path_to_image'
        #gb_mem = copy.copy(image)
        #output = y.detect(rgb_mem)
        #self.service = self.create_service(Yolo, "/get_predictions", getpred)

def main(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

'''import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback(image_msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(image_msg, '32FC1')
        print("Image received and converted successfully.")
    except CvBridgeError as e:
        print(f"CV Bridge Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Node('test_cv_bridge_node')
    subscription = node.create_subscription(Image, '/kinect2/color/image_raw', callback, 10)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()'''

if __name__ == '__main__':
    main()
