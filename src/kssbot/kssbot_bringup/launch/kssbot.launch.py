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
from launch.actions import RegisterEventHandler, LogInfo, DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():
    #launch argument Setting
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    #URDF Setting
    xacro_file= PathJoinSubstitution(
        [
            FindPackageShare("kssbot_description"),
            "urdf", "kssbot.urdf.xacro",
        ]
    )

    robot_description_content = Command(
        [
            'xacro ', xacro_file, 
            ' use_ros2_control:=', use_ros2_control, 
            ' sim_mode:=', use_sim_time,
        ]
    )
    
    robot_description = {"robot_description": robot_description_content, 'use_sim_time': use_sim_time}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("kssbot_bringup"),
            "config", "kssbot_sim.controllers.yaml",
        ]
    )

    #RVIZ Setting
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("kssbot_description"), 
            "config", "kssbot.rviz"
        ]
    )

    #for launching python launch file 
    joystick_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('kss_joystick'),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    #gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    #launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'kssbot'],
                        output='screen')


    #for setting nodes
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        # remappings=[
        #     ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["kssbot_diff_drive_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_pub_node,
            on_start=[diff_drive_controller_spawner],
        )
    )


    # Nodes for gazebo simulation
    launch_simultaion = GroupAction(
        actions = [
            joystick_node,

            robot_state_pub_node,

            gazebo,
            spawn_entity,  

            # spawn_entity on_exit-> robot_state_pub_node on_start -> 2 controllers and rviz2

            rviz_node,
            diff_drive_controller_spawner,
            joint_state_broadcaster_spawner,
            # delay_rviz_after_joint_state_broadcaster_spawner,
            # delay_diff_drive_controller_spawner_after_joint_state_broadcaster_spawner,  
        ],
            condition = IfCondition(use_sim_time),
    )
    
    # Nodes for realbot
    launch_kssbot = GroupAction(
        actions = [
            joystick_node,

            control_node,

            robot_state_pub_node,
            delayed_diff_drive_spawner,
            delayed_joint_state_broadcaster_spawner,
            #delay_rviz_after_joint_state_broadcaster_spawner,
        ],
           condition = UnlessCondition(use_sim_time),
    )

    return LaunchDescription([

        DeclareLaunchArgument( 'use_sim_time',
            default_value='false', description='Use sim time if true'),

        DeclareLaunchArgument('use_ros2_control',
            default_value='true',description='Use ros2_control if true'),

        #if use_sim_time = true
        launch_simultaion,

        #if use_sim_time = false
        launch_kssbot,

    ])