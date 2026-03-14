#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from copy import deepcopy
import numpy as np
import random
from ament_index_python.packages import get_package_share_directory

print("="*50)
path = get_package_share_directory("ur_two")
print(path)
print("="*50)

sdf_good_onion = """<?xml version="1.0"?> 
<sdf version='1.6'>
  <model name='onion'>
    <link name='base_link'>
      <pose frame=''>0 0 0 0 0 0</pose> <!-- make r = 1.5708 to make them upright -->
      <inertial>
        <pose frame=''>0 0 0 0 0 0</pose>
        <mass>0.005</mass>	<!-- From 0.14kg to 0.005kg -->
        <inertia>
          <ixx>0.08333</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.08333</iyy>
          <iyz>0</iyz>
          <izz>0.07780</izz>
        </inertia>
      </inertial>
      <collision name='base_link_fixed_joint_lump__box_collision_collision'>
        <pose frame=''>0 0 0 0 0 0</pose>
        <geometry>
        <!-- <mesh>
            <scale>1 1 1</scale>
            <uri>/home/divyam/dual_ur_ws/install/ur_two/share/ur_two/meshes/custom_onion.dae</uri>
          </mesh> -->
        <sphere>
               <radius>.05</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name='base_link_visual'>
        <pose frame=''>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>/home/divyam/dual_ur_ws/install/ur_two/share/ur_two/meshes/custom_onion.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>__default__</uri>
            <name>__default__</name>
          </script>
        </material>
      </visual>
      <gravity>1</gravity>
      <velocity_decay/>
      <self_collide>0</self_collide>
    </link>
      <plugin name="model_push" filename="libmodel_push.so"/>
  </model>
</sdf>
"""


sdf_bad_onion = """<?xml version="1.0"?> 
<sdf version='1.6'>
  <model name='onion'>
    <link name='base_link'>
      <pose frame=''>0 0 0 0 0 0</pose> <!-- make r = 1.5708 to make them upright -->
      <inertial>
        <pose frame=''>0 0 0 0 0 0</pose>
        <mass>0.005</mass>	<!-- From 0.14kg to 0.005kg -->
        <inertia>
          <ixx>0.08333</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.08333</iyy>
          <iyz>0</iyz>
          <izz>0.07780</izz>
        </inertia>
      </inertial>
      <collision name='base_link_fixed_joint_lump__box_collision_collision'>
        <pose frame=''>0 0 0 0 0 0</pose>
        <geometry>
        <!-- <mesh>
            <scale>1 1 1</scale>
            <uri>/home/divyam/dual_ur_ws/install/ur_two/share/ur_two/meshes/custom_onion_bad_real_blemish.dae</uri>
          </mesh> -->
        <sphere>
               <radius>.05</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name='base_link_visual'>
        <pose frame=''>0 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <scale>1 1 1</scale>
            <uri>/home/divyam/dual_ur_ws/install/ur_two/share/ur_two/meshes/custom_onion_bad_real_blemish.dae</uri>
          </mesh>
        </geometry>
        <material>
          <script>
            <uri>__default__</uri>
            <name>__default__</name>
          </script>
        </material>
      </visual>
      <gravity>1</gravity>
      <velocity_decay/>
      <self_collide>0</self_collide>
    </link>
      <plugin name="model_push" filename="libmodel_push.so"/>
  </model>
</sdf>
"""


def quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]


def create_good_onion_request(modelname, px, py, pz, rr, rp, ry, radius):
    """Create a SpawnModelRequest with the parameters of the spherical onion given.
    modelname: name of the model for gazebo
    px py pz: position of the onion (and its collision onion)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    radius: radius of the onion"""
    onion = deepcopy(sdf_good_onion)
    # Replace radius of model
    radius_str = str(round(radius, 3))
    onion = onion.replace('RADIUS', radius_str)
    # Replace modelname
    onion = onion.replace('MODELNAME', str(modelname))

    req = SpawnEntity.Request()
    req.name = modelname
    req.xml = onion
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


def create_bad_onion_request(modelname, px, py, pz, rr, rp, ry, radius):
    """Create a SpawnModelRequest with the parameters of the spherical onion given.
    modelname: name of the model for gazebo
    px py pz: position of the onion (and its collision onion)
    rr rp ry: rotation (roll, pitch, yaw) of the model
    radius: radius of the onion"""
    onion = deepcopy(sdf_bad_onion)
    # Replace radius of model
    radius_str = str(round(radius, 3))
    onion = onion.replace('RADIUS', radius_str)
    # Replace modelname
    onion = onion.replace('MODELNAME', str(modelname))

    req = SpawnEntity.Request()
    req.name = modelname
    req.xml = onion
    req.initial_pose.position.x = px
    req.initial_pose.position.y = py
    req.initial_pose.position.z = pz

    q = quaternion_from_euler(rr, rp, ry)
    req.initial_pose.orientation.x = q[0]
    req.initial_pose.orientation.y = q[1]
    req.initial_pose.orientation.z = q[2]
    req.initial_pose.orientation.w = q[3]

    return req


class OnionSpawner(Node):

    def __init__(self):
        super().__init__('onion_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')
        self.get_logger().info('Connected to service!')
        self.spawn_onion()

    def spawn_onion(self):
        onion_x = 0.75
        onion_y = -0.25
        onion_z = 1.0
        onion_radius = 0.2
        self.get_logger().info('Spawning bad onion')
        req = create_bad_onion_request('bad_onion', onion_x, onion_y, onion_z, 0.0, 0.0, 0.0, onion_radius)
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Successfully spawned bad onion')
        else:
            self.get_logger().error('Failed to spawn bad onion')


def main(args=None):
    rclpy.init(args=args)
    node = OnionSpawner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()