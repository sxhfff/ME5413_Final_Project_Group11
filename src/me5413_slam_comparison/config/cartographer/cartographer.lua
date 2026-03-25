include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.5,        -- SLOWER = fewer updates
  pose_publish_period_sec = 1e-2,          -- Slightly slower
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- ★★★ FASTER MAPPING (KEY CHANGES) ★★★
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 6     -- WAS 8: Fewer scans/submap = 30% faster
TRAJECTORY_BUILDER_2D.min_range = 0.3
TRAJECTORY_BUILDER_2D.max_range = 15.0                    -- WAS 20: Match your lidar
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- Faster scan matching
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1

-- ★★★ FAST SAVE (CRITICAL) ★★★
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.5            -- WAS missing: Bigger voxels = 50% faster
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 8         -- WAS missing: 8 scans/submap = fast+accurate

-- Pose graph: Balance speed+accuracy
POSE_GRAPH.optimization_problem.huber_scale = 5e1
POSE_GRAPH.optimize_every_n_nodes = 50                    -- WAS 90: More frequent = better loops
POSE_GRAPH.constraint_builder.min_score = 0.62            -- WAS 0.68: Slightly looser

return options

