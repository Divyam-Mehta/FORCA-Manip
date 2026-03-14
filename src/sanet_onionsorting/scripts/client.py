#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
from sanet_onionsorting.srv import Frcnn
from sanet_onionsorting.srv import Yolo
from sanet_onionsorting.srv import YoloDeepsort

class client(Node):
                                                                                
    def __init__(self): 
        super().__init__("client")

        self.declare_parameters(
            namespace = '',
            parameters =[
                ('version', 'yolov8')
            ]
        )

        version = self.get_parameter('version').get_parameter_value().string_value
        self.get_logger().info('Chosen Service: ' + str(version))

        if version != 'frcnn' and version != 'yolov5' and version != 'yolov7' and version != 'yolov8':
            self.get_logger().warn("Version provided is incorrect, defaulting to yolov8")
            version = 'yolov8'

        if version == 'frcnn':
            self.client = self.create_client(Frcnn, '/get_predictions')
            self.request = Frcnn.Request()
        
        elif version == 'yolov5' or version == 'yolov8':
            self.client = self.create_client(Yolo, '/get_predictions')
            self.request = Yolo.Request()

        elif version == 'yolov7':
            self.client = self.create_client(YoloDeepsort, '/get_predictions')
            self.request = YoloDeepsort.Request()

        while not self.client.wait_for_service(4.0):
            self.get_logger().info("Waiting for Service ...")

        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        future = self.client.call_async(self.request)
        future.add_done_callback(partial(self.service_callback))
        
    def service_callback(self, future):
        try:
            response = future.result()
            if len(response.centx) > 0: 
                if response.centx[0] != -1.0:
                    self.get_logger().info("Centroid of leftmost onion:  x = " + str(response.centx[0]) + '  y = ' + str(response.centy[0]))
                    # self.get_logger().info("Centroid of each onion:  x = " + str(response.centx) + '  y = ' + str(response.centy))
                
        except Exception as e:
            self.get_logger().warn("Service failed: %r" % (e,))

def main(args=None):

    rclpy.init(args=args) 
    try:
        node = client()
        rclpy.spin(node)   
        rclpy.shutdown()

    except Exception as e:
        print('Combined Client Failed: %r' %(e,))

    except KeyboardInterrupt:
        print("  Keyboard Interrupt, shutting down")    

if __name__ == "__main__":
    main()