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
from launch_ros.actions import SetParameter

from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    robot_path = get_package_share_directory('robot')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
         launch_arguments={'gz_args': '-r '+os.path.join(robot_path,'worlds/world.sdf')}.items(),
    )
        
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
        
    sdf_file = os.path.join(robot_path, 'models', 'baxter', 'baxter.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
   
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/pan_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/tilt_cmd@std_msgs/msg/Float64]gz.msgs.Double',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/empty/model/baxter/model/Baxter/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',

        ],
        remappings=[
            ('/world/empty/model/baxter/model/Baxter/joint_state', 'joint_states'),
        ],

        output=['screen']
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
    

    return LaunchDescription([
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=robot_path),
        SetParameter(name='use_sim_time', value=True),
        robot_state_publisher,
        gz_sim,
        bridge,
        rviz,
    ])

