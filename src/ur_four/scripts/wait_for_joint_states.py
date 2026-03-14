#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class WaitForJointStates(Node):
    def __init__(self):
        super().__init__('wait_for_joint_states')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10)
        self.received = False

    def callback(self, msg):
        if msg.position:  # Check if actual values exist
            self.get_logger().info('Received joint_states. Exiting...')
            self.received = True
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = WaitForJointStates()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
