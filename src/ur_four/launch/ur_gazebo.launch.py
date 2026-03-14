# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    rviz_config_file = LaunchConfiguration("rviz_config_file", default=PathJoinSubstitution(
        [FindPackageShare("ur_four"), "config", "view_four_robot.rviz"])
        )
    
    launch_rviz = LaunchConfiguration("launch_rviz", default=False)

    # Path to your URDF file
    urdf_path = os.path.join(get_package_share_directory('ur_four'), 'urdf', 'gripper_pose_down.urdf')

    # Load the URDF as a parameter
    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ur_four'),
            'config',
            'four_ur_controllers.yaml',
        ]
    )

    # Define QoS overrides
    qos_overrides = {
        # "qos_overrides./clock.subscription.durability": "volatile",
        # "qos_overrides./clock.subscription.history": "keep_last",
        # "qos_overrides./clock.subscription.depth": 100,
        # "qos_overrides./clock.subscription.reliability": "reliable",
    }

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, qos_overrides]  # Added qos_overrides here
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'UR_Robots', '-allow_renaming', 'true'],
        parameters=[qos_overrides]    # Added qos_overrides here
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    joint_trajectory_controller_spawner_ur1 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller_ur1',
            '--param-file',
            robot_controllers,
            ],
    )

    joint_trajectory_controller_spawner_ur2 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller_ur2',
            '--param-file',
            robot_controllers,
            ],
    )

    joint_trajectory_controller_spawner_ur3 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller_ur3',
            '--param-file',
            robot_controllers,
            ],
    )

    joint_trajectory_controller_spawner_ur4 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller_ur4',
            '--param-file',
            robot_controllers,
            ],
    )

    ur_four = os.path.join(get_package_share_directory('ur_four'))


    conveyor_spawner= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "Conveyor",
            '-file', os.path.join(ur_four, "urdf", "conveyor.urdf"),
        ],
        output='screen',
    )

    bin1_spawner= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "Bin_1",
            '-file', os.path.join(ur_four, "urdf", "bin1.urdf"),
        ],
        output='screen',
    )

    bin2_spawner= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "Bin_2",
            '-file', os.path.join(ur_four, "urdf", "bin2.urdf"),
        ],
        output='screen',
    )


    camera1_spawner= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "Camera_1",
            '-file', os.path.join(ur_four, "urdf", "camera1.urdf"),
        ],
        output='screen',
    )

    camera2_spawner= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "Camera_2",
            '-file', os.path.join(ur_four, "urdf", "camera2.urdf"),
        ],
        output='screen',
    )

    camera3_spawner= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "Camera_3",
            '-file', os.path.join(ur_four, "urdf", "camera3.urdf"),
        ],
        output='screen',
    )

    camera4_spawner= Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "Camera_4",
            '-file', os.path.join(ur_four, "urdf", "camera4.urdf"),
        ],
        output='screen',
    )



    camera_d435_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'D435_Camera',
            '-file', os.path.join(get_package_share_directory('realsense2_description'), 'urdf', 'test_d435_camera.urdf')
        ]
    )




    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        # parameters=[robot_description],
    )


    static_tf_ur1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ur1",
        arguments=[
            "0.25", "-0.20", "0.35",
            "0.0", "0.0", "-1.57", 
            "world", "UR1_support_pillar",
        ],
    )

    static_tf_ur2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ur2",
        arguments=[
            "1.25", "-0.70", "0.35", 
            "0.0", "0.0", "1.57",
            "world", "UR2_support_pillar",
        ],
    )

    static_tf_ur3 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ur3",
        arguments=[
            "0.25", "0.80", "0.35",
            "0.0", "0.0", "-1.57",
            "world", "UR3_support_pillar",
        ],
    )

    static_tf_ur4 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_ur4",
        arguments=[
            "1.25", "0.30", "0.35", 
            "0.0", "0.0", "1.57",
            "world", "UR4_support_pillar",
        ],
    )


    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]',
                   '/world/my_world/create@ros_gz_interfaces/msg/EntityFactory[gz.msgs.EntityFactory]'
                #    '/world/empty/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V]'
        ],
        output='screen',
        # parameters=[qos_overrides]
    )

    return LaunchDescription([
        static_tf_ur1,
        static_tf_ur2,
        static_tf_ur3,
        static_tf_ur4,
        bridge,
        # Launch gazebo environment
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            # launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]),
            launch_arguments=[('gz_args', [' -r -v 4 /home/divyam/four_ur_ws/src/ur_four/worlds/test.sdf'])]),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[joint_trajectory_controller_spawner_ur1],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_trajectory_controller_spawner_ur1,
                on_exit=[joint_trajectory_controller_spawner_ur2],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_trajectory_controller_spawner_ur2,
                on_exit=[joint_trajectory_controller_spawner_ur3],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_trajectory_controller_spawner_ur3,
                on_exit=[joint_trajectory_controller_spawner_ur4],
            )
        ),
        rviz_node,
        node_robot_state_publisher,
        gz_spawn_entity,
        bin1_spawner,
        bin2_spawner,
        camera1_spawner,
        camera2_spawner,
        camera3_spawner,
        camera4_spawner,
        # camera_d435_spawner,
        conveyor_spawner,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ])
