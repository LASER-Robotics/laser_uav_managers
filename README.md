# LASER UAV Managers

ROS 2 package containing the high-level management nodes for the Laser UAV System (LUS).

##  About the Package

The nodes contained here are responsible for:
-   Implementing state machines for missions (e.g., Takeoff -> Navigate to Waypoint -> Land).
-   Translating high-level intentions (like "go to point A") into commands that the controllers can understand (like attitude and thrust setpoints).
-   Orchestrating the transition between different flight modes.
-   Managing the estimation of all variables used for control loop feedback.

##  Provided Nodes

Below is a list of the main ROS 2 nodes provided by this package.

### 1. `control_manager`
-   **Description:** This node instantiates the controller classes, organizing the main control loop and managing the UAV's state machines.
  
-   **Subscribed Topics:**
    -   `/$UAV_NAME/px4_api/motor_speed_estimated`: Used to feed the INDI controller, which requires estimated motor angular velocities.
    -   `/$UAV_NAME/px4_api/diagnostics`: To check flight controller information, such as whether the drone is armed and if the controller is in offboard mode.
    -   `/$UAV_NAME/estimation_manager/estimation`: To receive the drone's current state vector (position, orientation, velocities).
    -   `/$UAV_NAME/control_manager/trajectory_path`: To receive a list of waypoints for a mission trajectory.
    -   `/$UAV_NAME/control_manager/goto`: To receive a single waypoint for a go-to command.
      
-   **Published Topics:**
    -   `/$UAV_NAME/control_manager/diagnostics`: Publishes comprehensive status information about the control manager, including the latest internal planner request, takeoff completion status, current waypoint, motor-level control inputs, and state machine flags.
    -   `/$UAV_NAME/control_manager/attitude_rates_thrust`: Publishes references for angular rates and a normalized total thrust value to the low-level controllers.
    -   `/$UAV_NAME/control_manager/motor_speed_reference`: Publishes references for the angular rates of each motor.

-   **Services:**
    -   `/control_manager/takeoff` (`laser_msgs/srv/SetString`): Service to trigger the takeoff routine.
    -   `/control_manager/land` (`laser_msgs/srv/SetString`): Service to trigger the landing routine.
      
-   **Configurable Parameters:**
    ```yaml
    # Do not change these parameters
    rate: # Hz
      external_loop_control: 100.0 
      internal_loop_control: 250.0 
      diagnostics: 100.0

    takeoff:
      height: 1.5 # m
      speed: 1.0 # m/s

    # If enabled, this parameter prioritizes reaching the required speed; if disabled, waypoint arrival precision is prioritized.
    agile_fly: true

    filter_params: # These parameters are for the low-pass filters used on IMU data and motor angular velocity estimation before entering the INDI controller
      butterworth:
        gyro_a: [1.0000, -1.5782, 0.6528]
        gyro_b: [0.018650, 0.037301, 0.018650]

        motor_a: [1.0000, -1.5782, 0.6528]
        motor_b: [0.018650, 0.037301, 0.018650]
    ```

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
