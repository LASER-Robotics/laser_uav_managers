from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
import lifecycle_msgs.msg
import os

def generate_launch_description():
    uav_name = os.environ['UAV_NAME']
    uav_type = os.environ['UAV_TYPE']
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('laser_uav_managers'),
            'params',
            'estimation_manager.yaml'
        ]),
        description='Path to the manager parameters file.'
    )
    
    params_uav_file_arg = DeclareLaunchArgument(
        'uav_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('laser_uav_managers'),
            'params', 'laser_uavs', uav_type + '.yaml'
        ]),
        description='Path to the drones parameters file.'
    )
    
    params_ekf_file_arg = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('laser_uav_estimators'),
            'params', 'state_estimator.yaml'
        ]),
        description='Path to the estimator parameters file.'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=PythonExpression(['"', os.getenv('REAL_UAV', "true"), '" == "false"']),
        description='Defines if simulation time (from the /clock topic) should be used.'
    )

    estimation_manager_node = LifecycleNode(
        package='laser_uav_managers',
        executable='estimation_manager_main',
        name='estimation_manager',
        namespace=EnvironmentVariable('UAV_NAME', default_value='uav'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            LaunchConfiguration('uav_params_file'),
            LaunchConfiguration('ekf_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry_in', 'px4_api/odometry'),
            ('odometry_fast_lio_in', 'fast_lio/odometry'),
            ('odometry_openvins_in', 'vins_republisher/odometry'),
            ('imu_in', 'px4_api/imu'),
            ('control_in', 'control_manager/diagnostics'),
            ('odometry_out', 'estimation_manager/estimation'),
            ('odometry_predict', 'estimation_manager/estimation_predict'),
            ('set_odometry', 'set_odometry'),
            ('diagnostics', 'estimation_manager/diagnostics'),
        ]
    )

    configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=estimation_manager_node,
            on_start=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(estimation_manager_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
                )),
            ],
        )
    )

    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=estimation_manager_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(estimation_manager_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    
    return LaunchDescription([
        params_file_arg,
        params_uav_file_arg,
        params_ekf_file_arg,
        use_sim_time_arg,
        estimation_manager_node,
        configure_event_handler,
        activate_event_handler,
    ])
