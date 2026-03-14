import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('sanet_onionsorting'),
        'config',
        'test_params.yaml'
    )

    Yolo_Service = Node(
        package = "sanet_onionsorting",
        executable = "service.py",
        name = "service",
        parameters = [config]
    )

    Yolo_Client = Node(
        package = "sanet_onionsorting",
        executable = "client.py",
        name = "client",
        parameters = [config]
    )

    ld.add_action(Yolo_Service)
    ld.add_action(Yolo_Client)

    return ld