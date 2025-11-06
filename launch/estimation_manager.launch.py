from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.actions import OpaqueFunction # Necessário
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
import lifecycle_msgs.msg
import os
import yaml # Necessário
from ament_index_python.packages import get_package_share_directory # Necessário

# Esta função só é chamada no "Momento 2" (Execução)
def load_ekf_path(context, *args, **kwargs):
    manager_params_file_path = LaunchConfiguration('params_file').perform(context)
    
    estimator_config_file = 'state_estimator.yaml'
    
    try:
        with open(manager_params_file_path, 'r') as f:
            config_data = yaml.safe_load(f)
        
        estimator_name = config_data['/**/**']['ros__parameters']['initial_odometry_source']
        
        if(estimator_name == 'fast_lio_odom'):
            estimator_config_file = 'fast_lio_state_estimator.yaml'
        elif(estimator_name == 'openvins_odom'):
            estimator_config_file = 'openvins_state_estimator.yaml'
        elif(estimator_name == 'px4_api_odom'):
            estimator_config_file = 'px4_api_state_estimator.yaml'
        else:
            estimator_config_file = 'state_estimator.yaml'

        print(f"Info: Usando o arquivo de parâmetros do estimador: {estimator_config_file}")
            
    except (IOError, KeyError, TypeError) as e:
        print(f"Alerta: Não foi possível ler 'initial_odometry_source' de {manager_params_file_path}. Usando 'state_estimator.yaml' como padrão. Erro: {e}")
        estimator_config_file = 'state_estimator.yaml'

    try:
        estimators_pkg_share = get_package_share_directory('laser_uav_estimators')
    except Exception as e:
        print(f"Erro fatal: Pacote 'laser_uav_estimators' não encontrado. {e}")
        return []

    ekf_params_path = os.path.join(
        estimators_pkg_share, 'params', estimator_config_file
    )
    
    context.launch_configurations['ekf_params_file'] = ekf_params_path
    
    return []

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
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=PythonExpression(['"', os.getenv('REAL_UAV', "true"), '" == "false"']),
        description='Defines if simulation time (from the /clock topic) should be used.'
    )

    set_ekf_path_action = OpaqueFunction(function=load_ekf_path)

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
        use_sim_time_arg,
        
        set_ekf_path_action, 
        
        estimation_manager_node,
        configure_event_handler,
        activate_event_handler,
    ])