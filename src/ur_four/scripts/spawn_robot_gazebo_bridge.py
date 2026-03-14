#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from std_msgs.msg import String

class GazeboBridgeNode(Node):
    def __init__(self):
        super().__init__('gazebo_bridge_node')

        # Declare and initialize parameters
        self.declare_parameter('robot_name', 'ur')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_description = ''

        # Create a subscriber for the robot description topic
        self.create_subscription(String, '/robot_description', self.robot_description_callback, 1)

        # Create a service client to call SpawnEntity
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

        self.get_logger().info('Gazebo Bridge Node has been initialized.')

    def robot_description_callback(self, msg):
        # Update robot description whenever new message is received
        self.get_logger().info('Received message on /robot_description topic.')
        self.robot_description = msg.data

        if not self.robot_description:
            self.get_logger().warn('Received empty robot description.')
        else:
            self.get_logger().info('Robot description received, attempting to spawn robot.')
            self.spawn_robot()

    def spawn_robot(self):
        # Wait for the spawn service to be available
        self.get_logger().info('Waiting for spawn_entity service to be available...')
        if not self.spawn_client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error('Spawn service not available, please make sure Gazebo is running with the spawn_entity service.')
            return

        # Check if robot description is still empty
        if not self.robot_description:
            self.get_logger().error('Cannot spawn robot. Robot description is empty.')
            return

        # Log robot description size
        self.get_logger().info(f'Robot description length: {len(self.robot_description)} characters.')

        request = SpawnEntity.Request()
        request.name = self.robot_name
        request.xml = self.robot_description
        request.robot_namespace = ''
        request.reference_frame = 'world'

        # Log the request details
        self.get_logger().info(f'Spawning robot with name: {request.name}, namespace: {request.robot_namespace}, reference frame: {request.reference_frame}')

        # Call the spawn service
        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.spawn_response_callback)

    def spawn_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Successfully spawned {self.robot_name} in Gazebo.')
            else:
                self.get_logger().error(f'Failed to spawn {self.robot_name}: {response.status_message}')
        except Exception as e:
            self.get_logger().error(f'Exception while spawning entity: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridgeNode()
    try:
        node.get_logger().info('Spinning the node to wait for robot description...')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Gazebo Bridge Node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()