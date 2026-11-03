# Long-trajectory MINCO validation

## Change and scope

The MINCO backend in the planning manager inserts collinear internal knots, retimes violating pieces and their neighbors, checks continuous derivative bounds, and samples the current prediction window from retained polynomials. It does not split the mission into independent trajectories or force stops at internal knots. Its default sampling interval remains 0.01 s.

The SUPER optimizer is unchanged. The simulation-clock watchdog fix is already supplied by the stable base and benefits all three manager backends. It is separate from this MINCO generation change.

## Review

Reviewed the complete diff against stable, callers, parameter validation, lifecycle and replacement behavior, dependency exports, generated-plan storage, polynomial sampling, terminal references and failure handling. The recursive Bernstein bounds remain conservative when subdivision stops. Generation has finite piece and retiming limits; prediction windows have a sample limit. Unsupported or oversized requests fail without relaxing the dynamic limits.

No blocking functional defect was found. Added regression cases for rejection of excessive polynomial pieces, accepting a subsequent valid mission, and rejecting oversized prediction windows. Removed unrelated controller formatting so the control-manager source matches stable. No new dependency or parameter is introduced.

## Automated verification

Build: `cmake --build /tmp/lus_minco_fix_build -j 2` using the ROS Humble/LUS environment configured for that directory.

Behavioral suites: `ctest --test-dir /tmp/lus_minco_fix_build -R 'test_reference_horizons|test_estimation_path|test_planning_launch' --output-on-failure` with the matching built libraries on LD_LIBRARY_PATH and a local, isolated ROS domain. Results: 69 reference-horizon tests, 8 estimation-path tests and 10 launch tests passed.

The tests cover five-lap generation at acceleration limits of 2 and 30 m/s², waypoint passage, position/velocity/acceleration/jerk continuity, continuous derivative bounds, moving initial references, yaw wrapping, mission replacement, rolling and terminal windows, resource-limit failures, and simulation pause/resume handling for PMM/MINCO/SUPER.

Cppcheck, CMake lint and Python docstring checks passed. The full repository lint suite is not green: the same four Python lint errors and C++ formatting divergences exist in the local stable snapshot. The updated MINCO implementation/header and reference-horizon test pass the C++ formatter. XML schema validation could not fetch the ROS package schema in this environment. These checks are recorded separately from behavioral passes; no blanket all-checks-passed claim is made.

## SITL verification — 2026-11-03

Managers revision tested in flight: `a49a508`. Subsequent merge preparation changes only tests, documentation and controller whitespace. The MINCO implementation used in flight is unchanged.

Used an adaptation of `laser_uav_simulation/tmux/one_drone_px4/session.yml`, including normal API/autostart, estimator and controller. Vehicle: PX4 x500, Gazebo Classic empty world, ground-truth sensor profile. A fresh installed manager overlay was used for both runs. Physics timestep, controller gains, timeouts and control rates were unchanged; the GUI was omitted. ROS domain 77, localhost-only DDS, PX4 UXRCE_DDS_PTCFG=1 and Gazebo master port 11477 isolated the experiment.

Initial hover reference: `(0, 0, 1.5)` m, heading zero. Request: first waypoint `(0, 0, 3)`, followed by five repetitions of `(45, 0, 3)`, `(45, 45, 3)`, `(-15, 45, 3)`, `(-15, 0, 3)`, `(0, 0, 3)`. All 26 headings zero; `speed=2.0`, `stop_on_waypoints=false`. MINCO: max_velocity=2.0 m/s, max_acceleration=2.0 m/s², sampling_step=0.01 s.

| Backend | Generation | Mission duration (simulated) | Tracking RMS | Maximum error | Emergency samples |
| --- | --- | --- | --- | --- | --- |
| MINCO | 24.376 ms | 589.78 s | 0.207375 m | 0.308124 m | 0 |
| PMM | 47.680 ms | 750.68 s | 0.197152 m | 0.375699 m | 0 |

Both completed all five laps, landed and disarmed using normal services. Task sessions and owned processes were stopped. Tracking errors compare the controller reference and estimated position in the same odometry frame; they do not measure ground-truth accuracy. One run per backend is evidence for this scenario, not a statistical stability guarantee.

Runtime artifacts on the test machine (temporary, not part of the repository):

- `/tmp/lus_minco_merged_sitl_20261103`: session, configs, environment, manifest with library hashes, logs, telemetry.csv, events.jsonl, summary.json, cleanup.json, tracking.png, comparison.png and comparison.json.
- `/tmp/lus_pmm_merged_sitl_20261103`: equivalent PMM flight artifacts and cleanup confirmation; the same installed manager overlay was used.
- `/tmp/lus_minco_merge_ready_20261103`: review, build/check records and proposed PR description.

## Limits

No SUPER SITL flight or physical trial was performed. Gazebo pauses were not deliberately injected during these flights; paused-clock behavior was tested in the automated suite. MINCO and PMM have no collision corridor. PMM's fly-through reference left the rectangle and reached altitude 4.208 m during the initial transition; MINCO's reference reached 3.015 m. Successful tracking does not imply containment in a prescribed volume.
