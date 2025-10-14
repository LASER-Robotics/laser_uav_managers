from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch.actions import RegisterEventHandler, EmitEvent

from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare

from launch.events import matches_action
from launch.event_handlers.on_process_start import OnProcessStart
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

import lifecycle_msgs.msg

import os

def generate_launch_description():
    uav_name = os.environ['UAV_NAME']
    uav_type = os.environ['UAV_TYPE']
    estimation_source = os.environ['UAV_ESTIMATION_SOURCE']
    
    defaults_uavs = ["x500", "lr7pro"]

    if uav_name == "":
        print("The uav name dont set up in yours enviroment variables")
        return

    if uav_type == "" or (not uav_type in defaults_uavs):
        print("The uav type dont set up in yours enviroment variables")
        return

    estimation_topic = ""
    if estimation_source == "GNSS":
        estimation_topic = '/' + uav_name + '/px4_api/odometry'
    elif estimation_source == "VIO":
        estimation_topic = '/' + uav_name + '/vins_republisher/odometry'
    elif estimation_source == "LIO":
        estimation_topic = '/' + uav_name + '/fast_lio/odometry'
    elif estimation_source == "EstimationManager":
        estimation_topic = '/' + uav_name + '/estimation_manager/estimation'
    else:
        print("The uav estimation source dont exist")
        return

#Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            'nmpc_controller_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_uav_controllers'), 'params', 'nmpc_controller', uav_type + '.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'agile_planner_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_uav_planners'), 'params', 'agile_planner', uav_type + '.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            'control_manager_file',
            default_value=PathJoinSubstitution([FindPackageShare('laser_uav_managers'),
                                                'params', 'control_manager.yaml']),
            description='Full path to the file with the all parameters.'
        )
    )

#Initialize arguments
    nmpc_controller_file = LaunchConfiguration('nmpc_controller_file')
    agile_planner_file = LaunchConfiguration('agile_planner_file')
    control_manager_file = LaunchConfiguration('control_manager_file')

    control_manager_lifecycle_node = LifecycleNode(
        package='laser_uav_managers',
        executable='control_manager',
        name='control_manager',
        namespace=uav_name,
        output='screen',
        parameters=[control_manager_file, agile_planner_file, nmpc_controller_file, {'use_sim_time': True}],
        remappings=[
            ('/' + uav_name + '/odometry_in', estimation_topic),
            ('/' + uav_name + '/motor_speed_estimation_in', '/' + uav_name + '/px4_api/motor_speed_estimated'),
            ('/' + uav_name + '/imu_in', '/' + uav_name + '/px4_api/imu'),
            ('/' + uav_name + '/motor_speed_reference_out', '/' + uav_name + '/control_manager/motor_speed_reference'),
            ('/' + uav_name + '/diagnostics_out', '/' + uav_name + '/control_manager/diagnostics'),
            ('/' + uav_name + '/attitude_rates_thrust_out', '/' + uav_name + '/control_manager/attitude_rates_thrust'),
            ('/' + uav_name + '/goto_in', '/' + uav_name + '/control_manager/goto'),
            ('/' + uav_name + '/trajectory_path_in', '/' + uav_name + '/control_manager/trajectory_path'),
            ('/' + uav_name + '/planner_view_out', '/' + uav_name + '/control_manager/planner_view'),
            ('/' + uav_name + '/takeoff', '/' + uav_name + '/control_manager/takeoff'),
            ('/' + uav_name + '/land', '/' + uav_name + '/control_manager/land'),
            ('/' + uav_name + '/api_diagnostics_in', '/' + uav_name + '/px4_api/diagnostics'),
        ]
    )

    event_handlers = []

    event_handlers.append(
#Right after the node starts, make it take the 'configure' transition.
        RegisterEventHandler(
            OnProcessStart(
                target_action=control_manager_lifecycle_node,
                on_start=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(control_manager_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                    )),
                ],
            )
        ),
    )

    event_handlers.append(
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=control_manager_lifecycle_node,
                start_state='configuring',
                goal_state='inactive',
                entities=[
                    EmitEvent(event=ChangeState(
                        lifecycle_node_matcher=matches_action(control_manager_lifecycle_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )),
                ],
            )
        ),
    )

    ld = LaunchDescription()

#Declare the arguments
    for argument in declared_arguments:
        ld.add_action(argument)

#Add client node
    ld.add_action(control_manager_lifecycle_node)

#Add event handlers
    for event_handler in event_handlers:
        ld.add_action(event_handler)

    return ld
