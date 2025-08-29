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
    -   `/uav1/control_manager/attitude_rates_thrust`: Publishes setpoints for angular rates and a normalized total thrust value to the low-level controllers.
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
