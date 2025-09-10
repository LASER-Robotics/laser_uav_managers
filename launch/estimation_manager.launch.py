"""
Launch file for the EstimationManager node with lifecycle automation and topic remapping.
@author Wagner Dantas Garcia | Laser UAV Team <wagnergarcia@eng.ci.ufpb.br>
@date September 1, 2025

This launch file is responsible for:
1. Declaring configurable launch arguments, including use_sim_time.
2. Launching the 'estimation_manager' as a lifecycle node (LifecycleNode).
3. Passing the parameter configuration file to the node.
4. Remapping the node's input and output topics.
5. Automating the lifecycle transitions (configure, activate) so that the node
   is ready for use immediately after launch.
"""

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

def generate_launch_description():
    """Generates the complete launch configuration."""

    # --- 1. ARGUMENT DECLARATION ---

    # Argument for the parameters file
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('laser_uav_managers'),
            'params',
            'estimation_manager.yaml'
        ]),
        description='Path to the estimator parameters file.'
    )

    # Argument to control simulation time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Defines if simulation time (from the /clock topic) should be used.'
    )


    # --- 2. NODE DEFINITION ---

    # Definition of the lifecycle node
    estimation_manager_node = LifecycleNode(
        package='laser_uav_managers',
        executable='estimation_manager_main',
        name='estimation_manager',
        namespace=EnvironmentVariable('UAV_NAME', default_value='uav'),
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            # Uses the value from the 'use_sim_time' argument
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry_in', 'px4_api/odometry'),
            ('odometry_fast_lio_in', 'fast_lio/odometry'),
            ('odometry_openvins_in', 'vins_republisher/odom'),
            ('imu_in', 'px4_api/imu'),
            ('control_in', 'control_manager/diagnostics'),
            ('odometry_out', 'estimation_manager/estimation'),
            ('odometry_predict', 'estimation_manager/estimation_predict'),
            ('set_odometry', 'set_odometry'),
            ('diagnostics', 'estimation_manager/diagnostics'),
        ]
    )


    # --- 3. LIFECYCLE EVENT HANDLERS (no changes) ---

    # Event handler to trigger the 'configure' transition upon node startup
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

    # Event handler to trigger the 'activate' transition after successful configuration
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

    # --- 4. RETURN THE LAUNCH DESCRIPTION ---
    
    return LaunchDescription([
        # Add the new arguments to the description
        params_file_arg,
        use_sim_time_arg,

        # Remaining actions
        estimation_manager_node,
        configure_event_handler,
        activate_event_handler,
    ])