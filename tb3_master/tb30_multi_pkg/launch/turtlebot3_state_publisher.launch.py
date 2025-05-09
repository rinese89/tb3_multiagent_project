#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ROBOT_NAMESPACE = os.environ['ROBOT_NAMESPACE']

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = '/opt/ros/humble/share/turtlebot3_description/urdf/turtlebot3_waffle_pi_tb3_0.urdf'

    urdf = os.path.join(    
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        urdf_file_name)

    print("urdf_file_name : {}".format(urdf_file_name))

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rsp_params = {'robot_description': robot_desc}

    return LaunchDescription([
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[rsp_params, {'use_sim_time': use_sim_time}],
            #remappings=[
            #    ('/tf', 'tf'),
            #    ('/tf_static', 'tf_static')]
        )
    ])
