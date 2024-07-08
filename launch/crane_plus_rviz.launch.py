# Copyright 2024 RT Corporation
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


import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('urdf_seminar'), 'urdf', 'crane_plus.urdf.xacro')
    xacro_data = xacro.process_file(xacro_path)
    urdf_data = xacro_data.toprettyxml(indent='  ')

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               output='both',
               parameters=[{'robot_description': urdf_data}]
        )
    
    jsp = Node(package='joint_state_publisher_gui',
               executable='joint_state_publisher_gui',
               output='screen'
        )

    rviz_config = get_package_share_directory(
        'urdf_seminar') + '/config/crane_plus.rviz'

    rviz_node = Node(package='rviz2',
                     executable='rviz2',
                     name='rviz2',
                     output='log',
                     arguments=['-d', rviz_config]
        )
    
    return LaunchDescription([
        rsp,
        jsp,
        rviz_node,
    ])