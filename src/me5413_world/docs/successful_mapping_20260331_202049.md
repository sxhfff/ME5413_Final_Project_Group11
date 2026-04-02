# Successful Mapping Record

Timestamp: `2026-03-31 20:20:49`

## Saved map files

- `/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/me5413_world/maps/fastlio_gmapping_2d_map_20260331_202049.pgm`
- `/home/yushi/ros_ws/src/ME5413_Final_Project_Group11/src/me5413_world/maps/fastlio_gmapping_2d_map_20260331_202049.yaml`

## Map metadata

- Resolution: `0.05 m/pix`
- Size: `2400 x 2400`

## Successful mapping pipeline

World:

- `roslaunch me5413_world world.launch gui:=true headless:=false`

2D mapping:

- `roslaunch me5413_world fastlio_gmapping_2d.launch`

## Effective topic chain

- FAST-LIO input: `/mid/points`, `/imu/data`
- 2D projection source: `/mid/points`
- Projected scan: `/scan`
- GMapping output: `/map`

## Effective TF / frames

- Scan frame: `base_link`
- Map frame: `map`
- Odom frame: `odom`

## Notes

- This successful 2D mapping run used the optimized launch where:
  - `projection_cloud_topic` defaults to `/mid/points`
  - `min_height = 0.05`
  - `max_height = 0.55`
  - `range_max = 12.0`
  - `particles = 60`
  - `minimumScore = 30`
  - `linearUpdate = 0.08`
  - `angularUpdate = 0.04`
  - `map_update_interval = 0.5`

- Recommended next step for navigation:
  1. Use the saved `.yaml` map with `map_server`
  2. Run `amcl`
  3. Run `move_base`
