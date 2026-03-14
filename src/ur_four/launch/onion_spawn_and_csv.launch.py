import os
import random
import csv
import re
from math import sqrt
from datetime import datetime

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction

from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def write_records(csv_path: str, records):
    csv_path = os.path.expanduser(csv_path)
    csv_dir = os.path.dirname(csv_path) or "."
    os.makedirs(csv_dir, exist_ok=True)

    base, ext = os.path.splitext(csv_path)
    csv_path = base + ".csv" if ext.lower() != ".csv" else csv_path
    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["onion_id", "x_location", "y_location", "z_location"])
        for r in records:
            writer.writerow([r["onion_id"], r["x_location"], r["y_location"], r["z_location"]])
    print(f"Wrote CSV file to: {csv_path}")


def launch_setup(context, *args, **kwargs):
    # Read launch args
    good_onions = int(LaunchConfiguration("good_onions").perform(context))
    bad_onions = int(LaunchConfiguration("bad_onions").perform(context))
    bad_real_onions = int(LaunchConfiguration("bad_real_onions").perform(context))
    csv_path = LaunchConfiguration("csv_path").perform(context)

    # Prepare
    to_start = []
    onions = []
    locations = []
    min_x = 0.65
    max_x = 0.85
    min_y = -1.4
    max_y = 1.4
    z = 0.81
    min_dist = 0.1

    # Build onion ID list
    for i in range(good_onions):
        onions.append('good_onion' if i == 0 else f'good_onion_{i-1}')
    for i in range(bad_onions):
        onions.append('bad_onion' if i == 0 else f'bad_onion_{i-1}')
    for i in range(bad_real_onions):
        onions.append('bad_real_onion' if i == 0 else f'bad_real_onion_{i-1}')

    target_count = good_onions + bad_onions + bad_real_onions

    # Generate non-overlapping random locations
    while len(locations) != target_count:
        random_x = random.uniform(min_x, max_x)
        random_y = random.uniform(min_y, max_y)

        if not locations:
            locations.append([random_x, random_y])
            continue

        ok = True
        for loc in locations:
            distance = sqrt((random_x - loc[0]) ** 2 + (random_y - loc[1]) ** 2)
            if distance < min_dist:
                ok = False
                break
        if ok:
            locations.append([random_x, random_y])

    # Collect rows for CSV
    records = []

    # Spawn each onion + odom bridge
    for idx, onion in enumerate(onions):
        location = locations[idx]

        # Normalize URDF filename (strip trailing _<num>)
        if onion not in ('good_onion', 'bad_onion', 'bad_real_onion'):
            onion_urdf = re.sub(r'_\d+$', '', onion)
        else:
            onion_urdf = onion

        # Spawner node
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

        # Odom bridge (Gazebo -> ROS 2)
        onion_odom_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f'{onion}_odom_bridge',
            arguments=[
                f"/model/{onion}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            ],
            output="screen",
        )

        to_start.append(onion_spawner)
        to_start.append(onion_odom_bridge)

        # Add record
        records.append({
            "onion_id": onion,
            "x_location": float(location[0]),
            "y_location": float(location[1]),
            "z_location": float(z),
        })

    # Write CSV
    write_records(csv_path, records)

    return to_start


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument("good_onions", default_value='14'),
        DeclareLaunchArgument("bad_onions", default_value='18'),
        DeclareLaunchArgument("bad_real_onions", default_value='18'),
        DeclareLaunchArgument(
            "csv_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("ur_four"),
                "spawn_onions.csv"
            ]),
            description="Where to write onion spawn CSV inside ur_four"
        ),
    ]
    return LaunchDescription(launch_arguments + [OpaqueFunction(function=launch_setup)])
