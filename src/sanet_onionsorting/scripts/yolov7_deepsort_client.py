#!/usr/bin/env python3
'''
Author: Prasanth Suresh (ps32611@uga.edu)
Owner: THINC Lab @ CS UGA

@brief: This is a basic script for testing. Just gets the service results and prints it out.

Please make sure you provide credit if you are using this code.

'''
import rclpy
from rclpy.node import Node
from functools import partial
from sanet_onionsorting.srv import YoloDeepsort

class yolo_deepsort_client(Node):
                                                                                
    def __init__(self): 
        super().__init__("yolo_v7_client")
        self.client = self.create_client(YoloDeepsort, '/get_predictions')
        while not self.client.wait_for_service(4.0):
            self.get_logger().info("Waiting for YOLO_V7 service ...")
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        request = YoloDeepsort.Request()
        future = self.client.call_async(request)
        future.add_done_callback(partial(self.service_callback))

    def service_callback(self, future):
        try:
            response = future.result()
            if len(response.centx) > 0:
                if response.centx[0] == -1.0:
                    print("No Onion Detected")
                else:
                    print("Centroid of leftmost onion: ", ' x =',response.centx[0],',', 'y =',response.centy[0])
                    # print("Centroid of each onion: ", response.centx, response.centy)
            else:
                print('No RGB Image received yet, make sure camera is publishing ...')

        except Exception as e:
            self.get_logger().warn("YOLO_V7 Service failed: %r" % (e,))

def main(args=None):

    rclpy.init(args=args)
    try:
        node = yolo_deepsort_client()
        rclpy.spin(node)
        rclpy.shutdown
    
    except Exception as e:
        print('Yolo_V7 Client Failed: %r' %(e,))

    except KeyboardInterrupt:
        print("  Keyboard Interrupt, shutting down")

if __name__ == "__main__":
    main()