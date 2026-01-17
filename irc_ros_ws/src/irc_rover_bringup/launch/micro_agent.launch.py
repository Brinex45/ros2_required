from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import xacro
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    _micro_ros_agent_0 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial',
            '--dev', '/dev/ttyACM0'
        ],
        output='screen'
    )

    _micro_ros_agent_1 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent', 'serial',
            '--dev', '/dev/ttyACM1'
        ],
        output='screen'
    )

    return LaunchDescription([
        _micro_ros_agent_0,
        _micro_ros_agent_1
    ])
