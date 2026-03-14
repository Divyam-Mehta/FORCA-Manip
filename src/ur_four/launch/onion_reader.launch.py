# spawn_onions_from_csv.launch.py
import os
import csv
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def _read_records(csv_path: str):
    csv_path = os.path.expanduser(csv_path)
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"CSV file not found: {csv_path}")

    records = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        # Expecting headers: onion_id, x_location, y_location, z_location
        for row in reader:
            if not row.get("onion_id"):
                continue
            try:
                x = float(row["x_location"])
                y = float(row["y_location"])
                z = float(row["z_location"])
            except (TypeError, ValueError) as e:
                print(f"[spawn_from_csv] Skipping row with bad coords: {row} ({e})")
                continue

            records.append(
                {
                    "onion_id": row["onion_id"].strip(),
                    "x": x,
                    "y": y,
                    "z": z,
                }
            )
    print(f"[spawn_from_csv] Loaded {len(records)} records from {csv_path}")
    return records


def _onion_type_from_id(onion_id: str) -> str:
    """
    Map an onion_id to its URDF base name:
      'good_onion' or 'good_onion_<n>'     -> 'good_onion'
      'bad_onion' or 'bad_onion_<n>'       -> 'bad_onion'
      'bad_real_onion' or 'bad_real_onion_<n>' -> 'bad_real_onion'
    """
    # Strip numeric suffix
    base = re.sub(r'_\d+$', '', onion_id)
    # Only allow known types
    if base in ("good_onion", "bad_onion", "bad_real_onion"):
        return base
    raise ValueError(f"Unknown onion type for id '{onion_id}' (base='{base}')")


def launch_setup(context, *args, **kwargs):
    csv_path = LaunchConfiguration("csv_path").perform(context)
    allow_renaming = LaunchConfiguration("allow_renaming").perform(context).lower() in ("1", "true", "yes")
    pkg_share = get_package_share_directory("ur_four")

    records = _read_records(csv_path)

    to_start = []
    for rec in records:
        onion_id = rec["onion_id"]
        x, y, z = rec["x"], rec["y"], rec["z"]

        try:
            urdf_base = _onion_type_from_id(onion_id)
        except ValueError as e:
            print(f"[spawn_from_csv] Skipping record: {e}")
            continue

        urdf_path = os.path.join(pkg_share, "urdf", f"{urdf_base}.urdf")
        if not os.path.exists(urdf_path):
            print(f"[spawn_from_csv] URDF not found for {onion_id}: {urdf_path} — skipping")
            continue

        # Spawner node
        spawner = Node(
            package="ros_gz_sim",
            executable="create",
            name=f"{onion_id}_spawner",
            parameters=[
                {"file": urdf_path},
                {"x": x},
                {"y": y},
                {"z": z},
                {"allow_renaming": allow_renaming},
            ],
            output="screen",
        )

        # Odom bridge (Gazebo -> ROS 2)
        odom_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=f"{onion_id}_odom_bridge",
            arguments=[
                f"/model/{onion_id}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            ],
            output="screen",
        )

        to_start.append(spawner)
        to_start.append(odom_bridge)

    if not to_start:
        print("[spawn_from_csv] No valid onions to spawn (check CSV contents).")

    return to_start


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "csv_path",
                default_value=PathJoinSubstitution([
                FindPackageShare("ur_four"),
                "50_onions_config_3.csv"
            ]),
                description="Path to CSV file with onion_id,x_location,y_location,z_location",
            ),
            DeclareLaunchArgument(
                "allow_renaming",
                default_value="true",
                description="Pass allow_renaming to ros_gz_sim create",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
