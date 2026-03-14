import random
from math import sqrt
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, ExecuteProcess

from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import re

def launch_setup(context, *args, **kwargs):
    
    # Define the launch arguments and convert the arguments to integers so that they can be used in the code
    good_onions_launch = LaunchConfiguration("good_onions")
    bad_onions_launch = LaunchConfiguration("bad_onions")
    bad_real_onions_launch = LaunchConfiguration("bad_real_onions")
    good_onions_string = good_onions_launch.perform(context)
    bad_onions_string = bad_onions_launch.perform(context)
    bad_real_onions_string = bad_real_onions_launch.perform(context)
    good_onions = int(good_onions_string)
    bad_onions = int(bad_onions_string)
    bad_real_onions = int(bad_real_onions_string)
    
    # Define empty lists and variables
    to_start = []                             # List of executables for the launch
    onions = []                               # List of onions to be spawned
    locations = []                            # List of random locations to spawn the onions
    min_x = 0.65                              # Minimum x value of bounding box for spawning onions
    max_x = 0.85                              # Maximum x value of bounding box for spawning onions
    min_y = -1.4                              # Minimum y value of bounding box for spawning onions
    max_y = 1.4                               # Maximum y value of bounding box for spawning onions
    z = 0.81                                  # Height to spawn onions
    min_dist = 0.1                            # Minimum distance between onions
    
    # Generate unique names for each onion and append them to the list of onions
    for i in range(good_onions):
        if i == 0:
            onions.append('good_onion')
        else:
            onions.append(f'good_onion_{i-1}')
            
    for i in range(bad_onions):
        if i == 0:
            onions.append('bad_onion')
        else:
            onions.append(f'bad_onion_{i-1}')
            
    for i in range(bad_real_onions):
        if i == 0:
            onions.append('bad_real_onion')
        else:
            onions.append(f'bad_real_onion_{i-1}')
    
    # While loop that repeats until the proper number of onion locations have been generated with the minimum distance
    while len(locations) != (good_onions + bad_onions + bad_real_onions):
        
        # Get a random location
        random_x = random.uniform(min_x, max_x)
        random_y = random.uniform(min_y, max_y)
        
        # If there is nothing in the list, append the first location
        if len(locations) == 0:
            locations.append([random_x, random_y])
        
        # If there is value(s) in the list, make sure the current location is the minimum distance away from each list location
        else:
            
            # Iterate for each list item
            for location in locations:
                
                # Get the distance of the current location to the current list item
                distance = sqrt((random_x - location[0])**2 + (random_y - location[1])**2)
                
                # Check if the current location is far enough away from the current list item
                if distance >= min_dist:
                    check = True
                else:
                    check = False
                    break
                
            # If the current location is far enough away from all the list items, add it to the list
            if check:
                locations.append([random_x, random_y])
    
    # Define Node for publishing onion locations and append it to the Launch list
    # gazebo_publisher = Node(
    #     package="ur_four",
    #     executable="gazebo_publisher.py",
    #     name='gazebo_publisher'
    # )
    
    # to_start.append(gazebo_publisher)
    
    # Spawner loop for each onion
    for onion in onions:
        
        # Get a unique location from the generated list of random locations
        index = onions.index(onion)
        location = locations[index]
        
        # For the current onion, obtain the proper name for the URDF (Shaves off "_0" from "good_onion_0" if necessary)
        if onion != 'good_onion' and onion != 'bad_onion' and onion != 'bad_real_onion':
            # onion_urdf = onion[:-2]
            onion_urdf = re.sub(r'_\d+$', '', onion)
        else:
            onion_urdf = onion
        
        # Define the onion spawner node based on the URDF and random location
        onion_spawner = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'{onion}_spawner',
            parameters=[
                        {'file': get_package_share_directory('ur_four') + f'/urdf/{onion_urdf}.urdf'},
                        {'x': location[0]},
                        {'y': location[1]},
                        {'z': z},
                        {'allow_renaming': True},
                        ],
            output='screen'
        )
        
        # Create a bridge from Gazebo to ROS2 for the onion Odometry info
        onion_odom_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f'{onion}_odom_bridge',
            arguments=[
                {
                    f"/model/{onion}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                }
            ],
            output="screen",
        )
        
        # Create a bridge from ROS2 to Gazebo for the onion detach topic
        ur1_onion_detach_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f'{onion}_detach_bridge',
            arguments=[
                {
                    f"/detach_ur1_{onion}@std_msgs/msg/Empty]gz.msgs.Empty",
                }
            ],
            output="screen",
        )

        ur2_onion_detach_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f'{onion}_detach_bridge',
            arguments=[
                {
                    f"/detach_ur2_{onion}@std_msgs/msg/Empty]gz.msgs.Empty",
                }
            ],
            output="screen",
        )
        
        # Create a bridge from ROS2 to Gazebo for the onion attach topic
        ur1_onion_attach_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f'{onion}_attach_bridge',
            arguments=[
                {
                    f"/attach_ur1_{onion}@std_msgs/msg/Empty]gz.msgs.Empty",
                }
            ],
            output="screen",
        )

        ur2_onion_attach_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f'{onion}_attach_bridge',
            arguments=[
                {
                    f"/attach_ur2_{onion}@std_msgs/msg/Empty]gz.msgs.Empty",
                }
            ],
            output="screen",
        )
        
        # The onions are initially attached when spawned, so detach them after a short delay 

        # We now use a bash loop to publish the detach commands 5 times using --once.
        ur1_onion_detach = TimerAction(
            period=5.0,
            actions=[ExecuteProcess(
                cmd=["bash", "-c", f"for i in {{1..5}}; do ros2 topic pub --once /detach_ur1_{onion} std_msgs/msg/Empty '{{}}'; sleep 0.1; done"],
                output="screen"
            )]
        )
        ur2_onion_detach = TimerAction(
            period=10.0,
            actions=[ExecuteProcess(
                cmd=["bash", "-c", f"for i in {{1..5}}; do ros2 topic pub --once /detach_ur2_{onion} std_msgs/msg/Empty '{{}}'; sleep 0.1; done"],
                output="screen"
            )]
        )




        # detach_ur1_command = ExecuteProcess(
        #     cmd=[
        #         PathJoinSubstitution([
        #             FindPackageShare('ur_four'),
        #             'scripts',
        #             'gazebo_publisher.py'
        #         ]),
        #         f'/detach_ur1_{onion}'
        #     ],
        #     output="screen",
        # )

        # detach_ur2_command = ExecuteProcess(
        #     cmd=[
        #         PathJoinSubstitution([
        #             FindPackageShare('ur_four'),
        #             'scripts',
        #             'gazebo_publisher.py'
        #         ]),
        #         f'/detach_ur2_{onion}'
        #     ],
        #     output="screen",
        # )




        # Append the current Nodes to the launch list
        to_start.append(onion_spawner)
        to_start.append(onion_odom_bridge)
        # to_start.append(ur1_onion_detach_bridge)
        # to_start.append(ur2_onion_detach_bridge)
        # to_start.append(ur1_onion_attach_bridge)
        # to_start.append(ur2_onion_attach_bridge)
        # to_start.append(ur1_onion_detach)
        # to_start.append(ur2_onion_detach)

        # to_start.append(detach_ur1_command)
        # to_start.append(detach_ur2_command)

    return to_start


def generate_launch_description():
    launch_arguments = []
    launch_arguments.append(DeclareLaunchArgument("good_onions", default_value='13'))
    launch_arguments.append(DeclareLaunchArgument("bad_onions", default_value='13'))
    launch_arguments.append(DeclareLaunchArgument("bad_real_onions", default_value='14'))
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])