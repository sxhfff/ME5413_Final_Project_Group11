# FAST-LIO + 2D Mapping Pipeline

## Goal

Use FAST-LIO for LiDAR-inertial odometry, then convert the FAST-LIO point cloud into a 2D LaserScan so `gmapping` can generate `/map` for later navigation with `move_base`.

## Architecture

Pipeline:

1. `FAST-LIO`
   - Input: `/mid/points`, `/imu/data`
   - Output: `/cloud_registered`, `/Odometry`, `/path`
2. `pointcloud_to_laserscan`
   - Input: `/cloud_registered`
   - Output: `/scan`
3. `gmapping`
   - Input: `/scan`
   - Output: `/map`, `map -> odom`

## TF design

Recommended tree:

- `map -> odom` from `gmapping`
- `odom -> base_link` from robot localization / wheel odometry
- robot frames from `robot_state_publisher`

For visualization in this repo, `fast_lio_slam.launch` also bridges:

- `camera_init -> odom`

This makes FAST-LIO global cloud and robot model visible in one TF tree.

Important:

- `gmapping` only needs a valid `odom -> base_link` tree and a scan frame reachable from `base_link`
- `FAST-LIO` itself does **not** create a 2D occupancy map

## Install requirement

This workspace currently needs:

```bash
sudo apt update
sudo apt install -y ros-noetic-pointcloud-to-laserscan
```

## New launch files

- `launch/fastlio_gmapping_2d.launch`
- `launch/include/gmapping_fastlio_2d.launch`
- `launch/include/pointcloud_to_scan.launch`

## Start commands

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source /home/yushi/ros_ws/devel/setup.bash
roslaunch me5413_world world.launch gui:=true headless:=false
```

Terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source /home/yushi/ros_ws/devel/setup.bash
roslaunch me5413_world fastlio_gmapping_2d.launch
```

If `/cloud_registered` does not produce a stable scan, try:

```bash
roslaunch me5413_world fastlio_gmapping_2d.launch projection_cloud_topic:=/cloud_registered_body
```

or

```bash
roslaunch me5413_world fastlio_gmapping_2d.launch projection_cloud_topic:=/mid/points scan_target_frame:=base_link
```

## Parameter rationale

### pointcloud_to_laserscan

- `min_height = -0.10`
- `max_height = 0.30`

This slices a near-horizontal band suitable for indoor mapping and suppresses ceiling/floor points.

- `angle_min/max = [-pi, pi]`

Use full 360-degree projection.

- `range_min = 0.35`
- `range_max = 15.0`

Reasonable for indoor Jackal-scale mapping.

### gmapping

- `particles = 40`
  Good default for indoor corridors/rooms without excessive CPU.
- `linearUpdate = 0.10`
- `angularUpdate = 0.05`
  Frequent map updates for slow indoor motion.
- `map_update_interval = 1.0`
  More responsive map refresh than the previous 2-second default.
- `minimumScore = 50`
  Lower than conservative laser-only tuning because projected scans from point clouds may be noisier.
- `delta = 0.05`
  5 cm map resolution, appropriate for move_base.

## RViz

Use:

- `rviz/fastlio_gmapping_2d.rviz`

Key displays:

- `/map`
- `/scan`
- `TF`
- `RobotModel`
- `/Odometry`
- `/path`

How to validate:

1. `Fixed Frame` must be `map`
2. `/scan` should update continuously as the robot moves
3. `/map` should expand and stabilize over explored space
4. `TF` should show a continuous `map -> odom -> base_link` chain

## Common errors

### 1. No `/map`

Possible causes:

- `pointcloud_to_laserscan` package not installed
- `/scan` is empty
- `gmapping` cannot resolve TF between scan frame and `base_link`

Check:

```bash
rostopic echo -n 1 /scan/header
rostopic echo -n 1 /map/header
rosnode info /slam_gmapping
```

### 2. Scan exists but map barely updates

Possible causes:

- height slice too thin or too high
- using `/cloud_registered` causes overly globalized scan geometry

Try:

- widening `min_height` / `max_height`
- switching `projection_cloud_topic` to `/cloud_registered_body`
- lowering `minimumScore`

### 3. Robot rotates and map drifts badly

Possible causes:

- FAST-LIO drift
- poor 2D scan projection from sparse 3D cloud
- gmapping over-trusting noisy projected scans

Try:

- slow down angular speed
- increase `particles`
- lower `linearUpdate` / `angularUpdate`
- project a thinner height band

### 4. RobotModel missing in RViz

Check:

- `robot_description` exists
- `base_link` is present in TF
- `Fixed Frame = map`

### 5. TF tree disconnected

Check that all of these exist:

- `map -> odom`
- `odom -> base_link`
- `base_link -> sensor frames`

If `odom -> base_link` is missing, navigation and gmapping will not behave correctly.

## Recommendation for navigation

For final `move_base` navigation:

1. run this launch to generate a 2D map
2. save the map with `map_saver`
3. switch to the normal navigation stack using:
   - `map_server`
   - `amcl`
   - `move_base`

This is the correct production workflow.
