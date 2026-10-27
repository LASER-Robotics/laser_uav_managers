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

### 1. `control_manager`

This lifecycle node tracks external navigation references with the existing
`laser_uav_controllers::NmpcController` and optional `IndiController`. It no
longer instantiates `laser_uav_planners::AgilePlanner` or computes PMM routes.

The package, namespace, component, executable and hardware interfaces retain
their LASER names: `laser_uav_managers`, `laser_uav_managers::ControlManagerNode`
and `control_manager`.

**Inputs and outputs**

| Interface | Type / purpose |
| --- | --- |
| `odometry_in` | `nav_msgs/Odometry`; measured state for control feedback |
| `api_diagnostics_in` | `laser_msgs/ApiPx4Diagnostics`; armed/offboard gate |
| `imu_in`, `motor_speed_estimation_in` | Existing INDI feedback inputs |
| `polynomial_trajectory_in` | `mars_quadrotor_msgs/PolynomialTrajectory`; external navigation |
| `attitude_rates_thrust_out`, `motor_speed_reference_out` | Existing hardware commands |
| `diagnostics_out` | `laser_msgs/UavControlDiagnostics` |
| `reference_horizon_out` | `nav_msgs/Path`; visualization of the sampled reference |
| `takeoff`, `land` | `std_srvs/Trigger`; local flight transitions |

`control_manager.launch.py` retains `control_manager_file`,
`uav_parameters_file` and `nmpc_controller_file`. Physical parameters, gains and
the ACADOS solver remain in the existing LASER packages. `agile_planner_file`
and the `goto`, `goto_relative`, and `trajectory_path` subscribers have been
removed from control; their navigation ownership is part of the proposed
[planning manager](docs/planning_manager_audit.md), not implemented in this branch.

```bash
ros2 launch laser_uav_managers control_manager.launch.py \
  uav_parameters_file:=/path/to/uav.yaml \
  odometry_topic:=/uav1/estimation_manager/estimation \
  polynomial_trajectory_topic:=/planning_cmd/poly_traj
```

`odometry_topic` is explicit so deployments using FAST-LIO directly can retain
that source. The default remains the original LUS estimate topic. The launch
keeps the original IMU and motor-speed remappings for INDI.

**Reference and timing contract**

- The first valid odometry initializes the commanded reference once. Later
  measurements update feedback, not the reference origin.
- Takeoff and landing start from the last commanded reference. Their quintic
  profiles use elapsed ROS time and configured speed. Hold keeps its target
  fixed. Landing, rampdown and safety hold take priority over navigation.
- After takeoff and mass calibration, external position/yaw polynomials are
  sampled at `now + i * dt`, for `N + 1` states. The polynomial start times must
  use the same ROS clock as the manager. No position replanning occurs here.
- A polynomial received during takeoff is retained for subsequent navigation;
  landing discards external navigation. Only a heartbeat matching the stored
  polynomial start time refreshes its deadline.
- An expired input holds the last commanded state when `hold_on_stale` is
  enabled. A finished trajectory holds its endpoint, including when the
  vehicle has not reached it yet. Missing yaw keeps the commanded heading.
- The existing single-threaded executor/default callback group serializes
  callbacks. No trajectory mutex or planner worker runs inside the controller.
  The configured rates remain 100 Hz outer control and 250 Hz INDI; these are
  targets, not a measured guarantee.

The artificial `<uav>/cm_odometry_debug -> <uav>/fcu` TF and benchmark CSV are
preserved. The benchmark observes `/goal_pose`; it never changes the reference.
Its portable default path is `/tmp/laser_uav_control_metrics.csv`, configurable
through `benchmark_metrics.csv_path`. CSV output retains the 13-column schema.
`reference_horizon_topic` configures the visualization output; `nav_msgs/Path`
is not a command interface because it contains no velocity or control flags.

**Validation**

`test_reference_horizons` contains 19 migrated regression tests covering fixed
references under changing odometry, external polynomial/yaw sampling, timeout,
emergency hold, landing priority, SUPER heartbeat matching, the original NMPC
solve path, artificial TF, and CSV completion.

```bash
colcon build --packages-select laser_uav_managers --allow-overriding laser_uav_managers
colcon test --packages-select laser_uav_managers \
  --ctest-args -R '^test_reference_horizons$' --output-on-failure
```

The controller executable/component remains the original LASER entry point.
The planning manager is an audit/proposal only; PMM navigation is intentionally
unavailable until its separate node is implemented. External polynomial
publishers, including SUPER, can already drive navigation after takeoff.

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
