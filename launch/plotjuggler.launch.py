from launch import LaunchDescription

from launch.substitutions import PathJoinSubstitution

from launch.actions import ExecuteProcess

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare commands
    refactor_plotjuggler_config_cmd = ExecuteProcess(
        cmd=[PathJoinSubstitution([FindPackageShare('laser_uav_managers'),
                                  'scripts', 'refactor_plotjuggler_config.sh'])],
        output='screen')

    # Declare nodes
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        output='screen',
        arguments=['-l /tmp/eval_control_layout.xml'],
        prefix=["bash -c 'sleep 2; $0 $@'"])

    return LaunchDescription([refactor_plotjuggler_config_cmd, plotjuggler_node])
