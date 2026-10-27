# LASER UAV Managers

ROS 2 package containing the high-level management nodes for the LASER UAV System (LUS).

##  About the Package

The nodes contained here are responsible for:
-   Managing local takeoff, hold and landing transitions.
-   Tracking references from external planners and publishing attitude/thrust or motor commands.
-   Orchestrating the transition between different flight modes.
-   Managing the estimation of all variables used for control loop feedback.

##  Provided Nodes

Below is a list of the main ROS 2 nodes provided by this package.

### 1. `control_manager` and `planning_manager`

`ControlManagerNode` owns takeoff, landing, hold, safety, mass calibration,
NMPC/INDI and hardware commands. `PlanningManagerNode` owns navigation goals,
waypoint sequencing and the selected PMM or SUPER planner. Both are lifecycle
nodes, launched as separate processes.

The first valid odometry initializes the commanded reference once. Subsequent
odometry is feedback only. Local maneuvers and safety hold begin at the last
commanded reference; they never capture a new target from live odometry.
The artificial `<uav>/cm_odometry_debug -> <uav>/fcu` TF remains available.

| Owner | Interface | Type / purpose |
| --- | --- | --- |
| Control | `odometry_in`, `imu_in`, `motor_speed_estimation_in` | Existing feedback inputs |
| Control | `api_diagnostics_in` | Existing armed/offboard gate |
| Control | `reference_horizon_in` | `laser_msgs/ReferenceHorizon`; timed trajectory window |
| Control | `planning_diagnostics_in` | Native mission events and CSV metadata |
| Control | `control_state_out` | `laser_msgs/UavControlState`; readiness, flight ID, N/dt, commanded state, calibrated mass |
| Control | `diagnostics_out` | Existing `laser_msgs/UavControlDiagnostics` |
| Control | `reference_horizon_out` | `nav_msgs/Path`; debug output only |
| Control | `attitude_rates_thrust_out`, `motor_speed_reference_out` | Existing hardware commands |
| Control | `takeoff`, `land` | `std_srvs/Trigger`; local maneuvers |
| Planning | `goto_in`, `goto_relative_in` | `laser_msgs/PoseWithHeading` |
| Planning | `trajectory_path_in` | `laser_msgs/TrajectoryPath` |
| Planning | `cancel` | `std_srvs/Trigger`; abort navigation and request commanded hold |
| Planning | `reference_horizon_out` | Native trajectory windows |
| Planning | `diagnostics_out` | `laser_msgs/UavPlanningDiagnostics`; mission progress |
| Planning | `super_goal_out`, `super_trajectory_in` | Private SUPER goal and polynomial interfaces |

The launch places navigation commands at `/<uav>/planning_manager/goto`,
`goto_relative`, `trajectory_path` and `cancel`. Control services remain at
`/<uav>/control_manager/takeoff` and `land`. There are no aliases to the old
control-manager goal topics, no input-format selector, and no polynomial
subscriber in control.

**Trajectory timing**

`ReferenceHorizon` preserves the complete `ReferenceState`, including attitude,
velocities, motor thrust and usage flags. `header.stamp` is the publication time;
`start_time + time_from_start[i]` identifies each sample. Offsets may be irregular
and must increase strictly. All nodes use the same ROS clock and coordinate frame.

Only control interpolates those samples onto `now + i * dt`, for `N + 1` states.
It uses linear interpolation and quaternion slerp, validates full future coverage,
and never extrapolates. PMM returns raw samples and relative times; its API has no
controller dt, execution clock or hover state. Planning slices that raw trajectory
into windows. SUPER polynomials are evaluated inside the existing planning node
at `super.sampling_step`, independently of the controller grid. No separate
`reference_horizon` or `super_trajectory` helper class/files are required.

PMM optimization runs in a worker capturing its inputs by value. The planning
timer continues to publish the existing segment while the next one is generated.
No planner worker or trajectory mutex runs inside control. Timers still share the
existing single-threaded control executor: configured 100 Hz outer / 250 Hz INDI
rates are targets, not a real-time guarantee.

Missing, expired or incomplete references latch the last commanded pose. A new
mission ID is required to resume after timeout/abort. Takeoff/landing reject
navigation windows. Flight IDs prevent an earlier flight from resuming after a
new takeoff. A `terminal` window must contain a fixed pose and zero velocities;
only that stationary endpoint may outlive its sample window and receive timeout.

**Setup and supported missions**

Build the three native repositories on `dev/separate_planning_and_control`:

```bash
colcon build --packages-select laser_msgs laser_uav_planners laser_uav_managers \
  --allow-overriding laser_msgs laser_uav_planners laser_uav_managers
source install/setup.bash
ros2 launch laser_uav_managers planning_manager.launch.py planner:=pmm
```

The `laser_uav_tui` branch with the same name publishes navigation goals to the
new planning-manager topics; rebuild that package when using the TUI.

`planner` is read-only and chosen before startup. PMM does not start or resolve
the optional `super_planner` executable. Select SUPER explicitly:

```bash
ros2 launch laser_uav_managers planning_manager.launch.py planner:=super \
  odometry_topic:=/uav1/estimation_manager/estimation \
  reference_frame:=uav1/odometry cloud_topic:=/cloud_registered
```

`UAV_NAME` and `UAV_TYPE` retain their LASER conventions. The launch shares the
physical YAML and odometry source across the two managers. The PMM receives the
calibrated mass before generating a segment. Controller gains and ACADOS remain
in `laser_uav_controllers`, without another solver copy in planning.

SUPER requires the corresponding branch of its repository: its ROS interface
publishes the configured `reference_frame_id` and propagates `EMER_STOP` in the
heartbeat. A cloud supplied by `cloud_topic` must already be transformed into
the **actual odometry frame**. Setting `reference_frame` names that existing
coordinate system; it does not transform the cloud or the odometry. The sensor
pipeline owns that transformation, outside both managers. Vendor YAML topic
names are remapped by the launch; a custom SUPER config must retain those
remapping source names or provide matching integration remaps.

PMM supports one goal, a fly-through waypoint list, or a list with a configured
hold at each waypoint. SUPER supports one goal or a list with stops; multi-goal
fly-through requests are rejected. Mission submission requires hover; finish or
cancel an active mission before submitting another. Backend switching and
in-motion replacement of a mission are outside this implementation.
SUPER uses its startup velocity limits; its waypoint requests require `speed=0`.
Only PMM accepts a per-mission speed override.

The `AgilePlanner` C++ API intentionally changed. Experimental
`carrara_uav_managers` callers of its old constructor, `getTrajectory(N, time)`
and `isHover()` have not been migrated; the package selection above builds the
native LASER stack only. No compatibility wrappers were added.

**Benchmark and validation**

The benchmark consumes native mission events instead of a SUPER goal topic or
heartbeat silence. A completed mission is recorded once after the measured UAV
reaches the goal. Abort, timeout and landing cancel the run. The CSV keeps its
13-column schema and now records the actual waypoint count. Its default path is
`/tmp/laser_uav_control_metrics.csv`.

`test_reference_horizons` covers commanded references under odometry drift,
local-maneuver priority, native time interpolation, preserved attitude/thrust,
invalid/stale/frame/flight input, missing coverage, terminal hold, abort,
original NMPC execution, artificial TF, mission CSV, PMM worker cancellation,
PMM-to-control windows and SUPER polynomials/yaw/heartbeat/emergency stop.
It also checks that backend selection cannot change after startup.

```bash
colcon test --packages-select laser_uav_managers \
  --ctest-args -R '^test_reference_horizons$' --output-on-failure
```

The [initial audit](docs/planning_manager_audit.md) records the earlier proposal;
this README describes the implemented contract, including the subsequent user
choices: no legacy inputs, interpolation in control, setup-only backend, and
no additional trajectory helper files. Passing these software checks does not
establish SITL or real-flight behavior or timer deadlines under flight load.

Validation of this round (2026-10-27): the three native C++ packages, SUPER and
the TUI built in isolated `/tmp/laser_clean_*` directories. All 29 GTests passed.
Changed manager C++ files passed cppcheck/uncrustify; manager launches and CMake
files passed their linters. Python syntax, YAML and package manifests parsed.
The TUI still has existing lint failures outside the changed topic lines.
The ROS transport reported blocked UDP sockets in this environment; publication
checks in the tests exercised a shared process, not interprocess DDS or SITL.

### 2. `estimation_manager`

-   **Description:** This node is responsible for the **state estimation** of the UAV. It instantiates the estimator classes and orchestrates the **fusion of multiple odometry sources**.
  
-   **Subscribed Topics:**
    -   `/$UAV_NAME/px4_api/odometry`: Odometry data from the flight controller (PX4).
    -   `/$UAV_NAME/fast_lio/odometry`: LiDAR odometry from Fast-LIO.
    -   `/$UAV_NAME/vins_republisher/odometry`: Visual odometry from OpenVINS (republished).
    -   `/$UAV_NAME/px4_api/imu`: High-frequency IMU data used for state propagation.
    -   `/$UAV_NAME/control_manager/diagnostics`: Control inputs (force and torque) from the controller, used for the **EKF prediction step**.

-   **Published Topics:**
    -   `/$UAV_NAME/estimation_manager/estimation`: The final **fused odometry estimate** (Position, Orientation, Velocity).
    -   `/$UAV_NAME/estimation_manager/diagnostics`: Diagnostic information, including active sensor status, buffer health, and timeout warnings.

-   **Services:**
    -   `/$UAV_NAME/set_odometry`: Service to manually switch the active odometry source (e.g., send "fast_lio_odom" to switch sources).

-   **Configurable Parameters:**
    ```yaml
    # Do not change these parameters
    frequency: 100.0 # Hz

    # Automatic switching thresholds
    odometry_switch_distance_threshold: 0.5 # meters
    odometry_switch_angle_threshold: 0.1 # radians
    odometry_switch_velocity_linear_threshold: 0.2 # m/s
    odometry_switch_velocity_angular_threshold: 0.2 # rad/s

    initial_odometry_source: "px4_api_odom"
    odometry_source_names: ["px4_api_odom", "openvins_odom", "fast_lio_odom"]
    
    ekf_verbosity: "DEBUG" # Options: SILENT, ERROR, WARNING, INFO, DEBUG, VERBOSE

    # Sensor Tolerances and Timeouts
    px4_odom_tolerance: 0.015
    px4_odom_timeout: 0.5
    px4_odom_covariance: 0.5

    openvins_odom_tolerance: 0.0075
    openvins_odom_timeout: 1.0
    openvins_odom_covariance: 0.5

    fast_lio_odom_tolerance: 0.15
    fast_lio_odom_timeout: 1.0
    fast_lio_odom_covariance: 0.5

    imu_tolerance: 0.015
    imu_timeout: 0.5
    imu_covariance: 0.5
    
    control_tolerance: 0.015
    control_timeout: 0.5
    ```

### Configurable Parameters for Both Nodes
The following parameters define physical properties and control allocation matrices shared by the managers.

```yaml
multirotor_parameters:
  # Quadrotor Mass
  mass: 2.0 # Kg

  # Diagonal of Inertia Matrix
  inertia: [0.02631, 0.02700, 0.0441]

  # Thrust constant {Ct}
  c_thrust: 27.6e-6 # Thrust coefficient

  # Drag coefficients in x, y, and z
  drag: [0.0, 0.0, 0.0] # Currently unused

  # Number of motors
  n_motors: 4

  # Effectiveness Matrix
  G1: [  1.0,    1.0,    1.0,    1.0,
        -0.18,   0.18,   0.18,  -0.18,
        -0.185,  0.185, -0.185,  0.185,
        -0.59,  -0.59,   0.59,   0.59 ]

  # Motor inertia gain matrix
  G2: [  0.0,      0.0,      0.0,      0.0,
         0.0,      0.0,      0.0,      0.0,
         0.0,      0.0,      0.0,      0.0,
        -1.5e-5,  -1.5e-5,   1.5e-5,   1.5e-5 ]

  # Used to model the thrust vs. throttle curve to normalize inputs for PX4
  quadratic_motor_model:
    a: 0.240572
    b: -0.135153

  # Thrust Constraints
  thrust_min: 0.0 # N
  thrust_max: 15.7 # N
  total_thrust_max: 62.8 # N
