from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Declare commands
    refactor_plotjuggler_node = Node(
        package='laser_uav_managers',
        executable='refactor_plotjuggler_config.sh',
        name='refactor_plotjuggler_config',
        output='screen',
    )

    # Declare nodes
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        arguments=['-l /tmp/eval_control_layout.xml'],
        prefix=["bash -c 'sleep 2; $0 $@'"])

    return LaunchDescription([refactor_plotjuggler_node, plotjuggler_node])
