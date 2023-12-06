#launch file for USB camera with apriltag detector
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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

from launch_ros.actions import Node

def generate_launch_description():
    robot_path = get_package_share_directory('robot')
    image_display_path = get_package_share_directory('image_transport_tutorials')
    
        
    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        parameters=[robot_path+"/config/params_1.yaml"],
        remappings=[
            ('image_raw', '/camera'),
            ('camera_info', '/camera_info'),
        ],
    )

    image_display = Node(
        package='image_transport_tutorials',
        executable='my_subscriber',
    )

    apriltags = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        remappings=[
            ('image_rect', '/camera'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[robot_path+"/config/apriltags.yaml"]
    )

    executive = Node(
      package = 'robot',
      executable = 'apriltag_executive.py',
    )
       
    arduino = Node(
            package = 'robot',
            executable = 'arduino.py',
            )
   
    return LaunchDescription([
        executive,
        camera,
        arduino,
        image_display,
        apriltags,
    ])

