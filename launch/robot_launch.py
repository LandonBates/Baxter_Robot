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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

from launch_ros.actions import Node

def generate_launch_description():
    robot_path = get_package_share_directory('robot')
    
    sdf_file = os.path.join(robot_path, 'models', 'baxter', 'baxter.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    #camera = Node(
    #    package='usb_cam',
    #    executable='usb_cam_node_exe',
    #    parameters=[robot_path+"/config/params_1.yaml"]
    #)

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
            ('image_rect', '/camera'),
            ('camera_info', '/camera_info'),
        ],
        parameters=[robot_path+"/config/apriltags.yaml"]
    )

    # Get the parser plugin convert sdf to urdf using robot_description topic
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[sdf_file],
        output=['screen']
    )
        
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            'camera@sensor_msgs/msg/Image[gz.msgs.Image',
            'camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output=['screen']
    )

    return LaunchDescription([
        #camera,
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=robot_path),
        apriltags,
        bridge,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])

