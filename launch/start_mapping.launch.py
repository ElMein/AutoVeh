from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_gazebo'),
                    'launch',
                    'turtlebot3_world.launch.py'
                ])
            ]),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turtlebot3_cartographer'),
                    'launch',
                    'cartographer.launch.py'
                ])
            ]),
            launch_arguments={'use_sim_tim': 'True'}.items()
        ),
        Node(
            package='my_robot_controller',
            executable='mapping',
            name='control'
        )
    ])
