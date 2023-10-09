# launch file for USB camera with apriltag detector
# requires files in config directory
#    <pkg>/config/camera_params.yaml
#    <pkg>/config/camera_info.yaml
#    <pkg>/config/apriltags.yaml
#
#  and an rviz configuration file stored in <pkg>/rviz/robot.rviz
# usb_cam is installed from ros-iron-usb-cam
# apriltags is installed from ros-iron-apriltags

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription

from launch_ros.actions import Node

def generate_launch_description():
    robot_path = get_package_share_directory('robot')
    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[robot_path+"/config/params_1.yaml"]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(robot_path, 
            'rviz', 'robot.rviz')],
    )

    apriltags = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/image_raw'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[robot_path+"/config/apriltags.yaml"]
    )

    return LaunchDescription([
        camera,
        apriltags,
        rviz
    ])

