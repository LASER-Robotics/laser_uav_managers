import os

import lifecycle_msgs.msg
import yaml
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import EmitEvent
from launch.actions import OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import PythonExpression


def generate_launch_description():
    uav = os.environ['UAV_NAME']
    uav_type = os.environ['UAV_TYPE']
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value=PythonExpression(
            ['"', os.getenv('REAL_UAV', 'true'), '" == "false"'])),
        DeclareLaunchArgument('odometry_topic',
                              default_value='/' + uav + '/estimation_manager/estimation'),
        DeclareLaunchArgument('reference_frame', default_value=uav + '/odometry'),
        DeclareLaunchArgument('cloud_topic',
                              default_value='/' + uav + '/fast_lio/cloud_registered',
                              description='Point cloud transformed by ROG-Map into '
                                          'the odometry frame using TF.'),
        DeclareLaunchArgument('super_config', default_value='click_smooth_ros2.yaml'),
    ]
    for name, package, subdirectory, filename in [
        ('control_manager_file', 'laser_uav_managers', 'params', 'control_manager.yaml'),
        ('planning_manager_file', 'laser_uav_managers', 'params', 'planning_manager.yaml'),
        ('uav_parameters_file', 'laser_uav_managers', 'params/laser_uavs', uav_type + '.yaml'),
        ('agile_planner_file', 'laser_uav_planners', 'params/agile_planner', uav_type + '.yaml'),
    ]:
        arguments.append(DeclareLaunchArgument(name, default_value=PathJoinSubstitution(
            [FindPackageShare(package), subdirectory, filename])))

    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup, args=[uav])])


def launch_setup(context, uav):
    params_file = LaunchConfiguration('planning_manager_file').perform(context)
    try:
        with open(params_file, 'r') as stream:
            planner = yaml.safe_load(stream)['/**/**']['ros__parameters']['planner']
    except (OSError, KeyError, TypeError, yaml.YAMLError) as error:
        raise RuntimeError(f'Cannot read planner from {params_file}: {error}') from error
    if planner not in ('minco', 'pmm', 'super'):
        raise ValueError(f'Invalid planner {planner!r} in {params_file}; use minco, pmm or super')

    planning = LifecycleNode(
        package='laser_uav_managers', executable='planning_manager',
        name='planning_manager', namespace=uav, output='screen',
        parameters=[LaunchConfiguration(key) for key in [
            'control_manager_file', 'uav_parameters_file', 'agile_planner_file',
            'planning_manager_file']] + [{
                'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('odometry_in', LaunchConfiguration('odometry_topic')),
            ('control_state_in', '/' + uav + '/control_manager/state'),
            ('reference_horizon_out', '/' + uav + '/planning_manager/reference_horizon'),
            ('diagnostics_out', '/' + uav + '/planning_manager/diagnostics'),
            ('goto_in', '/' + uav + '/planning_manager/goto'),
            ('goto_relative_in', '/' + uav + '/planning_manager/goto_relative'),
            ('trajectory_path_in', '/' + uav + '/planning_manager/trajectory_path'),
            ('cancel', '/' + uav + '/planning_manager/cancel'),
            ('super_goal_out', '/' + uav + '/super_planner/goal'),
            ('super_trajectory_in', '/' + uav + '/super_planner/trajectory'),
        ])
    configure = RegisterEventHandler(OnProcessStart(target_action=planning, on_start=[
        EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(planning),
                  transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))]))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=planning, start_state='configuring', goal_state='inactive',
        entities=[EmitEvent(event=ChangeState(lifecycle_node_matcher=matches_action(planning),
                            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))]))
    super_planner = Node(
        condition=IfCondition('true' if planner == 'super' else 'false'),
        package='super_planner', executable='fsm_node', name='super_planner', namespace=uav,
        output='screen', parameters=[{
            'config_name': LaunchConfiguration('super_config'),
            'reference_frame_id': LaunchConfiguration('reference_frame'),
            'use_sim_time': LaunchConfiguration('use_sim_time')}], remappings=[
                ('/goal_pose', '/' + uav + '/super_planner/goal'),
                ('/planning_cmd/poly_traj', '/' + uav + '/super_planner/trajectory'),
                ('/lidar_slam/odom', LaunchConfiguration('odometry_topic')),
                ('/cloud_registered', LaunchConfiguration('cloud_topic'))])
    return [configure, activate, planning, super_planner]
