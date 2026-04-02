# group11_stack

`group11_stack` is the B-side task package for the floor-2 mission.

It is intentionally a task-layer package, not a replacement for the team's shared mapping or navigation stack.

## Current architecture

### Shared team infrastructure

These parts already exist and should continue to be shared:

- Simulation:
  - `me5413_world/launch/world.launch`
- Shared mapping:
  - `me5413_world/launch/fastlio_gmapping_2d.launch`
- Shared navigation base:
  - `me5413_world/launch/navigation.launch`
- Shared saved map:
  - `me5413_world/maps/my_map_cropped_nav.yaml`

### B-side responsibility

This package is responsible for:

- receiving `least_box_type` from teammate A
- choosing the correct floor-2 room
- sequencing the floor-2 mission as a state machine
- sending goals to `move_base`
- handling retries and fallback entry selection
- publishing task status for integration/debugging

This package does not own:

- building the map
- saving the map
- `map_server`
- `amcl`
- `move_base` parameter tuning

## Package structure

- [package.xml](/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/group11_stack/package.xml)
  ROS package dependencies
- [CMakeLists.txt](/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/group11_stack/CMakeLists.txt)
  install/build rules
- [config/floor2_task.yaml](/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/group11_stack/config/floor2_task.yaml)
  mission config, room mapping, goal coordinates, retry policy
- [scripts/floor2_task_node.py](/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/group11_stack/scripts/floor2_task_node.py)
  B-side task state machine
- [launch/floor2_task.launch](/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/group11_stack/launch/floor2_task.launch)
  launch entry for the B-side node
- [ARCHITECTURE.md](/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/group11_stack/ARCHITECTURE.md)
  shorter architecture note

## Runtime flow

The task node runs this mission flow:

1. `WAIT_RESULT`
   waits for teammate A to publish `least_box_type`
2. `GO_TO_RAMP`
   sends the ramp goal
3. `ENTER_FLOOR2`
   chooses a preferred floor-2 entry and can fall back to the second entry
4. `GO_TO_TARGET_ROOM`
   maps `least_box_type` to a room and sends the room goal
5. `FINAL_PARK`
   pauses briefly and marks the mission complete
6. `FAILED`
   enters a terminal failure state with an explicit reason

## Interfaces

### Input

- Topic: `/group11/least_box_type`
- Type: `std_msgs/Int32`

Example:

```bash
rostopic pub -1 /group11/least_box_type std_msgs/Int32 "data: 3"
```

### Output

- Topic: `/group11/floor2_task/status`
- Type: `std_msgs/String`

Status strings include:

- current state
- `least_box_type`
- `target_room`
- active goal
- retry/fallback information
- failure reason

## Configuration

The main runtime config is:

- [floor2_task.yaml](/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/group11_stack/config/floor2_task.yaml)

The key sections are:

- `task`
  global behavior such as timeout and retry policy
- `box_type_to_room`
  mapping from A-side result to target room
- `navigation_goals`
  actual map-frame poses for ramp, entries, and room parking goals
- `room_entry_priority`
  preferred entry order for each room

### Important placeholders still to replace

Before final use, you must replace placeholder values in:

- `box_type_to_room`
- `navigation_goals`
- `room_entry_priority`

Right now the file is structurally correct but still contains example room names and example coordinates.

## Retry and fallback behavior

The node now supports:

- fallback to the second floor-2 entry if the preferred entry fails
- retrying the target-room goal before declaring mission failure

These are controlled by:

- `task/fallback_to_other_entry`
- `task/room_goal_retry_limit`

## How to run

### 1. Start shared navigation infrastructure

Use the team-shared navigation stack, not this package:

```bash
roslaunch me5413_world navigation.launch map_file:=/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/me5413_world/maps/my_map_cropped_nav.yaml
```

### 2. Start the B-side task node

```bash
roslaunch group11_stack floor2_task.launch
```

### 3. Publish a test A-side result

```bash
rostopic pub -1 /group11/least_box_type std_msgs/Int32 "data: 3"
```

## Dry-run mode

For task-logic debugging without real navigation execution:

```bash
roslaunch group11_stack floor2_task.launch dry_run:=true
```

In dry-run mode, the node still transitions through the full state machine, but it does not wait for real `move_base` success.

## What is already done

- package created and buildable
- floor-2 state machine in place
- A-to-B input interface reserved
- room mapping config entry created
- move_base action client integrated
- fallback entry and retry logic added

## What still needs real project data

- true room mapping for each box type
- true floor-2 room parking poses
- true ramp and entry poses
- final integration test with teammate A
