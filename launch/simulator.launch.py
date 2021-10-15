import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('gl_depth_sim'),
        'config',
#        'laser_scanner_config.yaml'
        'depth_camera_config.yaml'
    )

    urdf_fp = os.path.join(
        get_package_share_directory('gl_depth_sim'),
        'config',
        'gl_test_urdf.urdf'
    )

    node = Node(
        package='gl_depth_sim',
        executable='gl_depth_sim_simulator',
        name='simulator',
        output='screen',
        parameters=[{"config_file_path": config},
                    {"urdf_file_path": urdf_fp}]
    )

    ld.add_action(node)

    return ld
