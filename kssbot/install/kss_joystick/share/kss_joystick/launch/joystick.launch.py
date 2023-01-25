from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import DeclareLaunchArgument



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    joy_params = PathJoinSubstitution(
        [
            FindPackageShare("kss_joystick"),
            "config",
            "joystick.yaml",
        ]
    )

    joy_node = Node(
            package='kss_joystick',
            executable='joystick_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/diff_cont/cmd_vel_unstamped')]
         )

    twist_stamper = Node(
            package='twist_stamper',
            executable='twist_stamper',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel_in','/diff_cont/cmd_vel_unstamped'),
                        ('/cmd_vel_out','/diff_cont/cmd_vel')]
         )

    nodes = [
        joy_node,
        teleop_node,
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
            nodes,
        # twist_stamper       
    ])

