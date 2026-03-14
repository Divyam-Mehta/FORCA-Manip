#!/usr/bin/env python3

import time
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sanet_onionsorting.msg import OBlobs
from message_filters import Subscriber, ApproximateTimeSynchronizer

class GazeboPublisher(Node):


    def __init__(self): 
            
        super().__init__("gazebo_pub")
        self.publisher = self.create_publisher(OBlobs, 'object_location', 10)
        self.onion_gazebo_publisher = self.create_publisher(String, 'onion_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.current_onion = None


    def timer_callback(self):
        
        # Define list of topics and empty list of subscriptions
        topics = self.get_topic_names_and_types()
        subscriptions = []
        
        # Iterate through the list of topics
        for topic_name, topic_type in topics:
            
            # Find the Odometry info topics 
            if topic_name.startswith('/model'):
                
                # Create a Subscriber for each onion's Odometry info and append the Subscribers to the list
                subscription = Subscriber(self, Odometry, topic_name)
                subscriptions.append(subscription)
        
        # If there are subscriptions that need to be created
        if subscriptions:
            
            # Synchronize the subscriptions and register the callback
            self.subs = ApproximateTimeSynchronizer(subscriptions, 10, 0.5)
            self.subs.registerCallback(self.sub_callback)
            
            # Stop the timer once the subscriptions have been registered
            self.timer.cancel()


    def sub_callback(self, *msgs):
        
        # Define OBlobs message
        onions = OBlobs()
        self.current_onion = String()
        
        # Iterate through the messages
        for i in msgs:
            
            # Append the x, y, and z positions of each onion to the OBlobs message
            onions.x.append(i.pose.pose.position.x)
            onions.y.append(i.pose.pose.position.y)
            onions.z.append(i.pose.pose.position.z)
            
            # Append the correct color depending on whether or not it is a good or bad onion
            if i.child_frame_id.startswith('good_onion'):
                onions.color.append(1.0)
            elif i.child_frame_id.startswith('bad_onion'):
                onions.color.append(0.0)
            elif i.child_frame_id.startswith('bad_real_onion'):
                onions.color.append(0.0)
        
        # Publish the OBlobs message after appending each onion's location
        self.publisher.publish(onions)
        
        # Find the current onion for gripping actions
        for index in range(0, len(msgs)):
            
            # Logic for determining if the onion has been sorted into the bin or placed on the conveyor
            if not (msgs[index].pose.pose.position.z < 0.2 or (msgs[index].pose.pose.position.y > 0.25 and 0.74 < msgs[index].pose.pose.position.z < 0.78)):
                current_frame = msgs[index].child_frame_id
                self.current_onion.data = current_frame.split('/')[0]
                self.get_logger().info(str(self.current_onion))
                break
            
        # Publish the current onion, this is used for the detach/attach operations in Gazebo
        self.onion_gazebo_publisher.publish(self.current_onion)


def main(args=None):
    
    rclpy.init(args=args)
    try:
        node = GazeboPublisher()
        rclpy.spin(node)
        rclpy.shutdown
    
    except Exception as e:
        print('Gazebo Object Location Publisher Failed: %r' %(e,))
    
    except KeyboardInterrupt:
        print("  Keyboard Interrupt, shutting down")


if __name__ == "__main__":
    main()