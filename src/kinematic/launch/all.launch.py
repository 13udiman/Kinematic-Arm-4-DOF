from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg, 'urdf', 'arm.urdf.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([

        Node(
            package='kinematic',
            executable='kinematic'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2'
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[{'robot_description': robot_description},
                        os.path.join(get_package_share_directory('kinematic'), 'config', 'controllers.yaml')],
            output="screen"
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_trajectory_controller"],
        ),

        Node(
            package='joy',
            executable='joy_node'
        ),

        Node(
            package='joystick',
            executable='controller_node'
        )
    ])