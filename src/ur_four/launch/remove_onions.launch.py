import os, csv
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def _read_onion_ids(csv_path: str):
    csv_path = os.path.expanduser(csv_path)
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"[remove_from_csv] CSV not found: {csv_path}")
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        return [(row.get("onion_id") or "").strip() for row in reader if (row.get("onion_id") or "").strip()]

def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world_name").perform(context)
    csv_path = LaunchConfiguration("csv_path").perform(context)
    reqtype = LaunchConfiguration("reqtype").perform(context)
    reptype = LaunchConfiguration("reptype").perform(context)
    timeout_ms = LaunchConfiguration("timeout_ms").perform(context)

    onion_ids = _read_onion_ids(csv_path)

    procs = []
    for oid in onion_ids:
        # NOTE: --req must be ONE arg; keep the semicolon between fields.
        procs.append(
            ExecuteProcess(
                cmd=[
                    "gz", "service",
                    "-s", f"/world/{world}/remove",
                    "--reqtype", reqtype,
                    "--reptype", reptype,
                    "--timeout", timeout_ms,
                    "--req", f'type: MODEL; name: "{oid}"',
                ],
                name=f"remove_{oid}",
                output="screen",
            )
        )
    return procs

def generate_launch_description():
    default_csv = PathJoinSubstitution([FindPackageShare("ur_four"), "data", "onion_spawns.csv"])
    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="empty", description="GZ world name"),
        DeclareLaunchArgument("csv_path", default_value=PathJoinSubstitution([FindPackageShare("ur_four"), "onion_spawns.csv"]), description="CSV with onion_id column"),
        DeclareLaunchArgument("reqtype", default_value="gz.msgs.Entity"),
        DeclareLaunchArgument("reptype", default_value="gz.msgs.Boolean"),
        DeclareLaunchArgument("timeout_ms", default_value="5000"),
        OpaqueFunction(function=launch_setup),
    ])
