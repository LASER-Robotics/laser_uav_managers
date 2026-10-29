# LASER UAV Managers

`laser_uav_managers` runs the flight-management nodes of the LASER UAV System (LUS). It connects state estimation, navigation planning and the control loops that send commands to the hardware API.

| Node | Role |
| --- | --- |
| `control_manager` | Takeoff, hold, landing, safety checks, mass calibration and NMPC/INDI tracking. |
| `planning_manager` | Navigation goals, waypoint sequencing and MINCO, PMM or SUPER trajectory generation. |
| `estimation_manager` | Odometry-source selection, measurement buffering and state estimation. |

Each node has its own launch file. The launches configure and activate the lifecycle nodes automatically.

## Run

Build the package and its dependencies from the workspace root:

```bash
colcon build --packages-up-to laser_uav_managers laser_uav_tui
source install/setup.bash
```

Set the vehicle environment in each terminal, then start the required nodes separately:

```bash
export UAV_NAME=uav1
export UAV_TYPE=x500
export REAL_UAV=false
```

```bash
ros2 launch laser_uav_managers estimation_manager.launch.py
```

```bash
ros2 launch laser_uav_managers control_manager.launch.py
```

```bash
ros2 launch laser_uav_managers planning_manager.launch.py
```

Start the hardware API and sensor or simulation sources as well. `REAL_UAV=false` selects simulation time; the nodes need `/clock`. Use the calibrated YAML for the selected `UAV_TYPE`.

## Interfaces

The paths below use `uav1` and the supplied launch remappings. Replace `uav1` with your `UAV_NAME`.

### Control manager

| Direction | Topic or service | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/uav1/estimation_manager/estimation` | `nav_msgs/msg/Odometry` | State feedback. |
| Input | `/uav1/hw_api/imu` | `sensor_msgs/msg/Imu` | IMU feedback for the inner loop. |
| Input | `/uav1/hw_api/motor_speed_estimated` | `laser_msgs/msg/MotorSpeedStamped` | Timestamped motor-speed feedback. |
| Input | `/uav1/hw_api/diagnostics` | `laser_msgs/msg/ApiPx4Diagnostics` | Armed, offboard and preflight state. |
| Input | `/uav1/planning_manager/reference_horizon` | `laser_msgs/msg/ReferenceHorizon` | Timed navigation reference. |
| Input | `/uav1/planning_manager/diagnostics` | `laser_msgs/msg/UavPlanningDiagnostics` | Mission progress and outcome. |
| Output | `/uav1/control_manager/motor_speed_reference` | `laser_msgs/msg/MotorSpeed` | Individual motor commands. |
| Output | `/uav1/control_manager/attitude_rates_thrust` | `laser_msgs/msg/AttitudeRatesAndThrust` | Body-rate and total-thrust commands. |
| Output | `/uav1/control_manager/diagnostics` | `laser_msgs/msg/UavControlDiagnostics` | Tracking metrics, control timing, reference and predicted states. |
| Output | `/uav1/control_manager/state` | `laser_msgs/msg/UavControlState` | Readiness, commanded state, flight ID, horizon dimensions and calibrated mass. |
| Output | `/uav1/control_manager/reference_path` | `nav_msgs/msg/Path` | Reference path for visualization. |
| Service | `/uav1/control_manager/takeoff` | `std_srvs/srv/Trigger` | Starts the local takeoff maneuver. |
| Service | `/uav1/control_manager/land` | `std_srvs/srv/Trigger` | Starts the local landing maneuver. |

The selected control mode determines which command output is used. Motor-speed publishers must match the `MotorSpeedStamped` input type; the PX4 hardware API in this checkout publishes an unstamped `MotorSpeed`, so that feedback connection needs a matching publisher.

### Planning manager

| Direction | Topic or service | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/uav1/estimation_manager/estimation` | `nav_msgs/msg/Odometry` | Vehicle state and coordinate frame. |
| Input | `/uav1/control_manager/state` | `laser_msgs/msg/UavControlState` | Tracking readiness and commanded start state. |
| Input | `/uav1/planning_manager/goto` | `laser_msgs/msg/PoseWithHeading` | Absolute position and heading goal. |
| Input | `/uav1/planning_manager/goto_relative` | `laser_msgs/msg/PoseWithHeading` | Relative navigation goal. |
| Input | `/uav1/planning_manager/trajectory_path` | `laser_msgs/msg/TrajectoryPath` | Waypoints, requested speed and stop behavior. |
| Output | `/uav1/planning_manager/reference_horizon` | `laser_msgs/msg/ReferenceHorizon` | Trajectory window for control. |
| Output | `/uav1/planning_manager/diagnostics` | `laser_msgs/msg/UavPlanningDiagnostics` | Mission state, goal, waypoint count and planning timing. |
| Service | `/uav1/planning_manager/cancel` | `std_srvs/srv/Trigger` | Cancels navigation and requests a hold. |
| Output, SUPER only | `/uav1/super_planner/goal` | `geometry_msgs/msg/PoseStamped` | Goal passed to SUPER. |
| Input, SUPER only | `/uav1/super_planner/trajectory` | `mars_quadrotor_msgs/msg/PolynomialTrajectory` | SUPER trajectory and heartbeat/emergency information. |

Navigation goals go to `planning_manager`. Takeoff and landing remain control-manager services.

### Estimation manager

| Direction | Topic or service | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/uav1/hw_api/odometry` | `nav_msgs/msg/Odometry` | Autopilot odometry. |
| Input | `/uav1/fast_lio/odometry_high_freq` | `nav_msgs/msg/Odometry` | FAST-LIO odometry. |
| Input | `/uav1/vins_republisher/odometry` | `nav_msgs/msg/Odometry` | OpenVINS odometry after frame conversion. |
| Input | `/uav1/control_manager/diagnostics` | `laser_msgs/msg/UavControlDiagnostics` | Control inputs for model prediction. |
| Input | `/uav1/px4_api/garmin` | `sensor_msgs/msg/Range` | Optional height measurement. |
| Output | `/uav1/estimation_manager/estimation` | `nav_msgs/msg/Odometry` | Estimated state for control. |
| Output | `/uav1/estimation_manager/diagnostics` | `laser_msgs/msg/EstimationManagerDiagnostics` | Initialization, source status and buffer/timeout diagnostics. |
| Service | `/uav1/estimation_manager/set_odometry` | `laser_msgs/srv/SetString` | Selects an odometry source through the request's `data` field. |

The range input still uses `/px4_api/garmin` in the supplied launch; the hardware API publishes `/hw_api/garmin`. Remap the input when using that range output. Estimation predicts from control inputs in this implementation; it does not subscribe to an IMU topic directly.

## Configuration

### Control

[params/control_manager.yaml](params/control_manager.yaml) sets the loop rates and local flight behavior.

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `rate.external_loop_control` | `100.0` | Outer-loop target rate in Hz. |
| `rate.internal_loop_control` | `250.0` | INDI inner-loop target rate in Hz. |
| `rate.diagnostics` | `100.0` | Diagnostics rate in Hz. |
| `takeoff.height`, `takeoff.speed` | `1.5`, `1.0` | Takeoff height in metres and speed in m/s. |
| `land.speed` | `0.3` | Descent speed in m/s. |
| `land.threshold_detect`, `land.increment_rampdown` | `0.70`, `0.15` | Landing detection and thrust ramp-down settings. |
| `reference.timeout` | `0.50` | Maximum reference-publication age in seconds. |
| `goal.position_tolerance`, `goal.heading_tolerance` | `0.45`, `3.15` | Goal tolerances in metres and radians. |
| `filter_params.butterworth.*` | Coefficient arrays in the YAML | Gyroscope and motor-feedback filters. |
| `safe_area.enabled` | `false` | Enables position bounds. |
| `safe_area.constraints.{x,y,z}` | Bounds in the YAML | Minimum and maximum coordinates in metres. |
| `debug.publish_reference_path` | `true` | Publishes the reference path. |
| `benchmark_metrics.enabled` | `false` | Enables mission metrics in CSV. |
| `benchmark_metrics.csv_path` | `/tmp/laser_uav_control_metrics.csv` | Metrics output file. |

Controller gains and the prediction horizon come from `laser_uav_controllers/params/nmpc_controller/<UAV_TYPE>.yaml`. The hardware API's `control_input_mode` must match `nmpc_controller.nmpc_mode`.

### Planning

Choose `planner` in [params/planning_manager.yaml](params/planning_manager.yaml) before startup. It is read-only while the node runs, and the launch uses the same value to decide whether to start SUPER.

| Planner | Use | Mission support |
| --- | --- | --- |
| `minco` (default) | Smooth minimum-snap waypoint trajectories using MINCO-S4. | Single goal, fly-through waypoints or stops at waypoints. |
| `pmm` | Minimum-time trajectories for agile flight. | Single goal, fly-through waypoints or stops at waypoints. |
| `super` | Obstacle-aware planning and replanning with ROG-Map. | Single goal or waypoints with stops. |

MINCO and PMM do not check for collisions. SUPER requires `rog_map` from the separate `laser_uav_mappings` collection, along with the LASER `super_planner` and `mars_quadrotor_msgs` packages.

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `rate.reference_horizon` | `100.0` | Reference-window publication rate in Hz. |
| `control_state_timeout` | `0.5` | Maximum control-state age in seconds. |
| `planning_timeout` | `5.0` | Trajectory-generation timeout in seconds. |
| `waypoint_hold_time` | `3.0` | Hold duration at stop waypoints in seconds. |
| `minco.max_velocity` | `2.0` | MINCO velocity limit in m/s. |
| `minco.max_acceleration` | `2.0` | MINCO acceleration limit in m/s². |
| `minco.sampling_step` | `0.01` | MINCO output sampling interval in seconds. |
| `super.timeout` | `0.5` | SUPER update timeout in seconds. |
| `super.sampling_step` | `0.01` | Polynomial sampling interval in seconds. |

MINCO minimizes integrated squared snap for the selected waypoints and segment times, then stretches segment times to satisfy dynamic limits. It preserves the commanded initial velocity, with zero initial acceleration and jerk. Its formulation is described in [Wang et al., T-RO 2022](https://arxiv.org/abs/2103.00190).

MINCO and PMM accept a per-mission speed override; MINCO rejects speeds above `minco.max_velocity`. SUPER uses its startup velocity limits and requires `speed=0` in waypoint requests. A new valid goal or path replaces the current mission, including during generation or a waypoint hold. Only the latest pending request is retained.

### Estimation

[params/estimation_manager.yaml](params/estimation_manager.yaml) controls source selection and timing. The launch selects an MEKF tuning file from `laser_uav_estimators` using `initial_odometry_source`.

| Parameter | Meaning |
| --- | --- |
| `frequency` | Estimation rate in Hz; supplied value is `100.0`. |
| `initial_odometry_source` | Startup source: `px4_api_odom`, `openvins_odom` or `fast_lio_odom`. |
| `odometry_source_names` | Names of the available odometry sources. |
| `odometry_switch_distance_threshold` | Position-difference threshold in metres when switching sources. |
| `odometry_switch_angle_threshold` | Attitude-difference threshold in radians. |
| `odometry_switch_velocity_linear_threshold` | Linear-velocity difference threshold in m/s. |
| `odometry_switch_velocity_angular_threshold` | Angular-velocity difference threshold in rad/s. |
| `<source>_tolerance` | Measurement timing tolerance in seconds. Prefixes include `px4_odom`, `openvins_odom`, `fast_lio_odom`, `garmin` and `control`. |
| `<source>_timeout` | Time without a measurement before that source times out, in seconds. |
| `<source>_covariance` | Configured covariance value for the sensor inputs that provide this parameter. |
| `estimation_verbosity`, `ekf_verbosity` | Manager and filter logging levels. |

For OpenVINS, check the tuning filename: the estimator repository contains `mekf_openvins_state_estimator .yaml` with a space, while the launch requests the name without that space.

### Vehicle parameters

The files in [params/laser_uavs/](params/laser_uavs/) hold the calibration shared by control and planning, and the physical model used by estimation.

| Parameter under `multirotor_parameters` | Meaning |
| --- | --- |
| `mass` | Vehicle mass in kg. |
| `inertia` | Diagonal inertia terms in kg·m². |
| `n_motors` | Number of motors. |
| `G1`, `G2` | Static and motor-inertia allocation matrices, stored as flat arrays. Preserve the motor order used by the vehicle. |
| `c_thrust` | Rotor thrust coefficient. |
| `drag` | Model drag coefficients; supplied files mark these as unused. |
| `quadratic_motor_model.a`, `.b` | Thrust-to-normalized-command calibration. |
| `thrust_min`, `thrust_max` | Per-motor thrust bounds in newtons. |
| `total_thrust_max` | Total thrust limit in newtons. |
| `omega_max` | Body angular-rate limits in rad/s. |

## Launch overrides

Control accepts `control_manager_file`, `nmpc_controller_file`, `uav_parameters_file`, `odometry_topic`, `reference_horizon_topic`, `planning_diagnostics_topic` and `use_sim_time`.

Planning accepts `planning_manager_file`, `control_manager_file`, `uav_parameters_file`, `pmm_planner_file`, `odometry_topic`, `reference_frame`, `cloud_topic`, `super_config` and `use_sim_time`. Give both managers the same vehicle file and odometry source when overriding them.

For SUPER, `cloud_topic` defaults to `/uav1/fast_lio/cloud_registered`. ROG-Map needs TF from the cloud frame to the actual odometry frame at the cloud timestamp. `reference_frame` names that coordinate system; changing its name does not transform coordinates. Custom SUPER YAML files must preserve the topic names expected by the launch remappings or supply corresponding remaps.

Estimation accepts `params_file`, `uav_params_file` and `use_sim_time`. Use `ros2 launch <package> <launch_file> --show-args` to inspect launch defaults.

## Reference timing and holds

All managers must use the same ROS clock and coordinate frame. A `ReferenceHorizon` carries its publication stamp, an absolute trajectory start time and strictly increasing sample offsets. Control interpolates the samples onto its `N + 1` prediction states using linear interpolation and quaternion slerp. It requires coverage of the full horizon and does not extrapolate.

The first valid odometry initializes the commanded reference. After that, odometry is feedback: local maneuvers and safety holds start from the last commanded reference. Missing, stale or incomplete windows cause a hold; a new mission ID is required to resume after timeout or cancellation. Takeoff and landing reject navigation windows. Flight IDs prevent a previous flight's trajectory from being reused.

Only a stationary terminal reference can remain valid beyond its sample window, and it still needs fresh publications. Publication stamps may lead the local clock by one outer-loop period to accommodate independent simulation-clock delivery.

MINCO and PMM compute in a worker while planning continues to publish the previous trajectory. The configured control rates are targets; the control timers share a single-threaded executor.

## Metrics and checks

When enabled, the benchmark records one completed mission after the measured UAV reaches its goal. Abort, timeout and landing cancel the run. The CSV has 13 columns and includes the waypoint count.

The package includes checks for reference timing, interpolation, mission replacement, cancellation, planner limits and launch selection. Run them from the workspace root:

```bash
colcon test --packages-select laser_uav_managers \
  --ctest-args -R '^test_reference_horizons$' --output-on-failure
```

Software checks do not establish SITL behavior, flight performance or timer deadlines under load. Use simulation and flight validation for those measurements.
