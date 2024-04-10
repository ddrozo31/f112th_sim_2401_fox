# Copyright 2022 Open Source Robotics Foundation, Inc.
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

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )


    robot_name = '00_bicycle_robot.urdf.xacro'
    pkg_name = 'f112th_sim_2401_fox'

    pkg_path = os.path.join(
        get_package_share_directory(pkg_name))

    xacro_file = os.path.join(pkg_path,
                              'description',
                              robot_name)

    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': True}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'tricycle'],
                        output='screen')

    #load_joint_state_broadcaster = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #        'joint_state_broadcaster'],
    #   output='screen'
    #)

    #load_tricycle_controller = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', #'bicycle_drive_controller'],
    #    output='screen'
    #)

        # Launch the Diff_Controller
    bicycle_drive_spawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['bicycle_drive_controller'])
        
        # Launch the Joint_Broadcaster
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['joint_state_broadcaster'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    #delay_joint_state_after_spawn_entity = RegisterEventHandler(
    #        event_handler=OnProcessExit(
    #            target_action=spawn_entity,
    #            on_exit=[load_joint_state_broadcaster],
    #       )
    #    )
    #delay_tricycle_after_joint_state= RegisterEventHandler(
    #        event_handler=OnProcessExit(
    #            target_action=load_joint_state_broadcaster,
    #            on_exit=[load_tricycle_controller],
    #        )
    #    )
    
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(pkg_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    twist_mux_params = os.path.join(get_package_share_directory(pkg_name),'config','twist_mux.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params,{'use_sim_time': True}],
                    remappings=[('/cmd_vel_out','/bicycle_controller/cmd_vel')]
    )

    return LaunchDescription([
        gazebo,
        rviz,
        node_robot_state_publisher,
        spawn_entity,
        joystick,
        twist_mux_node,
        bicycle_drive_spawner,
        joint_broad_spawner
    ])

#delay_joint_state_after_spawn_entity,
#delay_tricycle_after_joint_state,