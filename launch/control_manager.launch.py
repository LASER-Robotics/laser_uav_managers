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

def generate_launch_description():
    #Declare arguments
    declared_arguments = []

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'control_manager_file',
    #         default_value=PathJoinSubstitution([FindPackageShare('laser_hexapod_managers'),
    #                                             'params', 'control_manager.yaml']),
    #         description='Full path to the file with the all parameters.'
    #     )
    # )

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'development_mode',
    #         default_value='False',
    #         description='Flag for enable development mode.'
    #     )
    # )

#Initialize arguments
    # control_manager_file = LaunchConfiguration('control_manager_file')

    control_manager_lifecycle_node = LifecycleNode(
        package='laser_uav_managers',
        executable='control_manager',
        name='control_manager',
        namespace='',
        output='screen',
        parameters=[],
        # prefix=['gdb -ex "run" -ex "info locals" -ex "list"  -ex "print x0" -ex "print lastU" -ex "print *optPtr" -ex "print nlmpc_solver_" --args'],
        # prefix=['valgrind --leak-check=full'],
        remappings=[
            ('/odometry_in', '/uav1/odometry'),
            # ('/odometry_in', '/odom'),
            # ('/goto_in', '/uav1/goto'),
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
