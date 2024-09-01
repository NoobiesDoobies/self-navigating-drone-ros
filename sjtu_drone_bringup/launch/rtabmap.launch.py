import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node


import xacro


def generate_launch_description():


    rtabmap = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')]),
                launch_arguments={
                    'rtabmap_args': "--delete_db_on_start", 
                    'rgb_topic': "/simple_drone/front/image_raw",
                    'depth_topic': "/simple_drone/front/depth/image_raw",
                    'camera_info_topic': "/simple_drone/front/camera_info",
                    'frame_id': "simple_drone/base_link",
                    'approx_sync': "false",
                    'wait_imu_to_init': "false",
                    'imu_topic': "/simple_drone/imu/out",
                    'qos': "1",
                    'rviz': "false",
                    }.items()
            )
    # Launch!
    return LaunchDescription([
        rtabmap
    ])
