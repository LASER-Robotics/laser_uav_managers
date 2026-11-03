import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import LifecycleNode, Node
from launch_ros.utilities import evaluate_parameters


@pytest.fixture
def planning_launch(tmp_path, monkeypatch):
    monkeypatch.setenv('UAV_NAME', 'uav1')
    monkeypatch.setenv('UAV_TYPE', 'x500')
    monkeypatch.setenv('REAL_UAV', 'false')
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'logs'))
    source = Path(__file__).resolve().parents[1] / 'launch' / 'planning_manager.launch.py'
    spec = importlib.util.spec_from_file_location('planning_launch', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    params_file = tmp_path / 'planning_manager.yaml'
    context = LaunchContext()
    context.launch_configurations['planning_manager_file'] = str(params_file)
    context.launch_configurations['planner'] = 'ignored CLI value'
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            assert action.name != 'planner'
            action.execute(context)
    setup = next(a for a in description.entities if isinstance(a, OpaqueFunction))
    return params_file, context, setup


@pytest.mark.parametrize('planner', ['minco', 'pmm', 'super'])
def test_yaml_selects_planner_and_matching_super_process(planning_launch, planner):
    params_file, context, setup = planning_launch
    params_file.write_text(yaml.safe_dump({
        '/**/**': {'ros__parameters': {'planner': planner}},
    }))
    actions = setup.execute(context)
    assert not any(isinstance(a, IncludeLaunchDescription) for a in actions)
    nodes = [a for a in actions if isinstance(a, LifecycleNode)]
    assert len(nodes) == 1
    node = nodes[0]
    assert perform_substitutions(context, normalize_to_list_of_substitutions(
        node.node_executable)) == 'planning_manager'
    parameters = evaluate_parameters(context, node._Node__parameters)
    assert params_file in parameters
    assert all('planner' not in p for p in parameters if isinstance(p, dict))
    assert parameters[-1]['use_sim_time'] is True
    super_node = next(a for a in actions if type(a) is Node)
    assert super_node.condition.evaluate(context) == (planner == 'super')


@pytest.mark.parametrize('overrides,cloud,odometry', [
    ({}, '/uav1/fast_lio/cloud_registered', '/uav1/estimation_manager/estimation'),
    ({'cloud_topic': '/custom/cloud', 'odometry_topic': '/custom/odom'},
     '/custom/cloud', '/custom/odom'),
])
def test_super_inputs_match_laser_topics_and_allow_overrides(
        planning_launch, overrides, cloud, odometry):
    params_file, context, setup = planning_launch
    params_file.write_text(yaml.safe_dump({
        '/**/**': {'ros__parameters': {'planner': 'super'}},
    }))
    context.launch_configurations.update(overrides)
    super_node = next(a for a in setup.execute(context) if type(a) is Node)
    remappings = {
        perform_substitutions(context, source): perform_substitutions(context, target)
        for source, target in super_node._Node__remappings
    }
    assert remappings['/cloud_registered'] == cloud
    assert remappings['/lidar_slam/odom'] == odometry


@pytest.mark.parametrize('content,error', [
    ('{}', RuntimeError),
    ('/**/**:\n  ros__parameters:\n    planner: unsupported\n', ValueError),
    ('/**/**: [', RuntimeError),
])
def test_invalid_planner_file_fails_before_starting_nodes(planning_launch, content, error):
    params_file, context, setup = planning_launch
    params_file.write_text(content)
    with pytest.raises(error, match=str(params_file)):
        setup.execute(context)


def test_control_launch_maps_reference_path(planning_launch):
    _, context, _ = planning_launch
    source = Path(__file__).resolve().parents[1] / 'launch' / 'control_manager.launch.py'
    spec = importlib.util.spec_from_file_location('control_launch', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    for action in description.entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    node = next(a for a in description.entities if isinstance(a, LifecycleNode))
    remappings = {
        perform_substitutions(context, source): perform_substitutions(context, target)
        for source, target in node._Node__remappings
    }
    assert remappings['/uav1/reference_path_out'] == '/uav1/control_manager/reference_path'
    assert '/uav1/reference_horizon_out' not in remappings


def test_estimation_launch_maps_odometry_path(planning_launch):
    _, context, _ = planning_launch
    source = Path(__file__).resolve().parents[1] / 'launch' / 'estimation_manager.launch.py'
    spec = importlib.util.spec_from_file_location('estimation_launch', source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    node = next(a for a in description.entities if isinstance(a, LifecycleNode))
    remappings = {
        perform_substitutions(context, source): perform_substitutions(context, target)
        for source, target in node._Node__remappings
    }
    assert remappings['odometry_path_out'] == 'estimation_manager/path'
