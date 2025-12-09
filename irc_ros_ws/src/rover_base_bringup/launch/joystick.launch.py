# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription

from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params = os.path.join(get_package_share_directory('rover_base_bringup'),'config','joystick.yaml')
    
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )
    
    # ps4_data_node = Node(
    #         package='ps4',
    #         executable='ps4_data_node',
    #         name='ps4_data_node',
    #         parameters=[joy_params],
    #      )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel','/unstamped_vel')]
         )
    
    twist_to_stamped_node = Node(
            package='rover_base_description',
            executable='twist_to_stamped',
            name='twist_to_stamped',
         )

    nodes = [
        joy_node,
        teleop_node,
        twist_to_stamped_node,
    ]

    return LaunchDescription(nodes)