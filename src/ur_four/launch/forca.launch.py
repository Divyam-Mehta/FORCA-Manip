import os

from pathlib import Path

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

    
def generate_launch_description():

    ur_four = get_package_share_directory('ur_four')

    # mt_orca_path = os.path.join(get_package_share_directory('ur_four').replace('/share/', '/lib/'), 'mt_orca_test')

    # print("="*50)
    # print(mt_orca_path)
    # print("="*50)

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_four_moveit_config")
        # .robot_description(file_path=os.path.join(ur_four, "urdf", "dual_ur.urdf.xacro"))
        .robot_description_semantic(Path("srdf") / "ur.srdf", {"name": "ur3e"})
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .moveit_cpp(file_path="config/moveit_cpp.yaml")
        # .planning_pipelines(default_planning_pipeline="pilz_industrial_motion_planner")   #  Use PILZ
        .to_moveit_configs()
    )

   
    # MoveItCpp executable
    demo_node_UR = Node(
        # name="hello_moveitcpp_ur2",
        package="ur_four",
        # executable="rvo2_psort",
        # executable="rvo2_test",
        executable="multi_thread_psort_orca",
        # executable="dec_rrt_baseline",
        output="screen",
        parameters=[moveit_config.to_dict(),
                    {
                        "use_sim_time": True,
                        "publish_robot_description_semantic": "true",
                    }
        ],
    ) 


    return launch.LaunchDescription([
        demo_node_UR,
    ])
