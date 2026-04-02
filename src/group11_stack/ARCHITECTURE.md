# Current Useful Architecture

## 1. Shared mapping pipeline

This part is already usable and should remain shared by the team.

- World and robot simulation:
  - `me5413_world/launch/world.launch`
- Shared mapping entry:
  - `me5413_world/launch/fastlio_gmapping_2d.launch`
- Optional stair variant:
  - `me5413_world/launch/fastlio_gmapping_2d_stairs.launch`
- Saved maps:
  - `me5413_world/maps/my_map.yaml`
  - `me5413_world/maps/my_map_cropped_nav.yaml`

### Effective shared data flow

1. Gazebo world publishes simulated robot sensors
2. FAST-LIO consumes:
   - `/mid/points`
   - `/imu/data`
3. `pointcloud_to_laserscan` produces:
   - `/scan`
4. `slam_gmapping` produces:
   - `/map`
   - `map -> odom`
5. Saved map is reused later by navigation

## 2. Existing navigation base

This is already present in the workspace and should be treated as shared infrastructure.

- Navigation entry:
  - `me5413_world/launch/navigation.launch`
- It currently provides:
  - `map_server`
  - `amcl`
  - `move_base`
  - `rviz`

### Important boundary

`navigation.launch` is the shared navigation base.
`group11_stack` should not replace it.

## 3. Existing code that should not be the B main task node

- Deprecated helper:
  - `me5413_world/src/goal_publisher_node.cpp`

This file publishes `/move_base_simple/goal` and was not designed to be the full floor-2 task coordinator.

## 4. B-side package role

`group11_stack` is now the task layer for teammate B.

It should:

- receive `least_box_type` from teammate A
- choose the floor-2 target room
- own the floor-2 state machine
- send goals to `move_base`
- decide success / failure / completion

It should not:

- rebuild the mapping pipeline
- replace map saving
- replace `map_server`, `amcl`, or `move_base`
- own low-level dynamic obstacle tuning

## 5. New B-side package structure

- `group11_stack/package.xml`
- `group11_stack/CMakeLists.txt`
- `group11_stack/config/floor2_task.yaml`
- `group11_stack/launch/floor2_task.launch`
- `group11_stack/scripts/floor2_task_node.py`

## 6. B-side runtime architecture

### Inputs

- `/group11/least_box_type`:
  expected type `std_msgs/Int32`

### Internal decision layers

1. `WAIT_RESULT`
2. `GO_TO_RAMP`
3. `ENTER_FLOOR2`
4. `GO_TO_TARGET_ROOM`
5. `FINAL_PARK`
6. `COMPLETED` or `FAILED`

### Outputs

- sends `move_base` action goals
- publishes task status:
  - `/group11/floor2_task/status`

## 7. Shared-map assumption for B

B-side logic assumes the team continues to use a shared saved map from `me5413_world/maps/`.

Recommended current navigation map:

- `me5413_world/maps/my_map_cropped_nav.yaml`

This keeps B-side focused on mission logic instead of rebuilding mapping.
