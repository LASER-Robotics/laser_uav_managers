# LASER UAV Managers

ROS 2 package containing the high-level management nodes for the Laser UAV System (LUS).

##  About the Package

The nodes contained here are responsible for:
-   Implementing state machines for missions (e.g., Takeoff -> Navigate to Waypoint -> Land).
-   Translating high-level intentions (like "go to point A") into commands that the controllers can understand (like attitude and thrust setpoints).
-   Orchestrating the transition between different flight modes.

##  Provided Nodes

Below is a list of the main ROS 2 nodes provided by this package.

### 1. `control_manager`
-   **Description:** This node instantiates the controller classes, organizing the main control loop and managing the UAV's state machines.
-   **Subscribed Topics:**
    -   `/uav1/any/odometry`: To receive the drone's current state vector (position, orientation, velocities).
    -   `/uav1/control_manager/trajectory_path`: To receive a list of waypoints for a mission trajectory.
    -   `/uav1/control_manager/goto`: To receive a single waypoint for a go-to command.
-   **Published Topics:**
    -   `/uav1/control_manager/attitude_rates_thrust`: Publishes refences for angular rates and a normalized total thrust value to the low-level controllers.
    -   `/uav1/control_manager/diagnostics`: Publishes important information from the controller, such as the latest waypoint from the planner, the most recent motor-level control inputs, and state machine flags.
-   **Configurable Parameters:**
    ```yaml
    control_manager:
      ros__parameters:
        rate: # Hz
          loop_control: 100.0
          diagnostic: 100.0

        takeoff:
          height: 1.5 # m
          speed: 1.0 # m/s
      
        agile_fly: false
    ```

### 2. `estimation_manager`
-   **Description:** This node is responsible for the state estimation of the UAV. It instantiates the `StateEstimator` class (EKF) and orchestrates the fusion of multiple odometry sources (PX4, OpenVINS, Fast-LIO) with IMU data. It handles the synchronization of sensor data, manages the switching between different odometry sources based on quality and consistency, and publishes the final fused state estimate used by the controllers.
-   **Subscribed Topics:**
    -   `/$UAV_NAME/px4_api/odometry` (`nav_msgs/msg/Odometry`):
        Odometry data from the flight controller (PX4).
    -   `/$UAV_NAME/fast_lio/odometry` (`nav_msgs/msg/Odometry`):
        LiDAR odometry from Fast-LIO.
    -   `/$UAV_NAME/vins_republisher/odometry` (`nav_msgs/msg/Odometry`):
        Visual odometry from OpenVINS (republished).
    -   `/$UAV_NAME/px4_api/imu` (`sensor_msgs/msg/Imu`):
        High-frequency IMU data used for state propagation.
    -   `/$UAV_NAME/control_manager/diagnostics` (`laser_msgs/msg/UavControlDiagnostics`):
        Control inputs (force and torque) from the controller, used for the prediction step of the EKF.

-   **Published Topics:**
    -   `/$UAV_NAME/estimation_manager/estimation` (`nav_msgs/msg/Odometry`):
        The final fused odometry estimate (Position, Orientation, Velocity).
    -   `/$UAV_NAME/estimation_manager/diagnostics` (`laser_msgs/msg/EstimationManagerDiagnostics`):
        Diagnostic information including active sensor status, buffer health, and timeout warnings.

-   **Services:**
    -   `~/set_odometry` (`laser_msgs/srv/SetString`):
        Service to manually switch the active odometry source (e.g., send "fast_lio_odom" to switch source).

-   **Configurable Parameters:**
    ```yaml
    estimation_manager:
      ros__parameters:
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