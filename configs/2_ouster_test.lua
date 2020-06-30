-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "body_aligned_imu_link",
  published_frame = "body_aligned_imu_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

------------------------
-- Trajectory Builder --
------------------------
MAX_3D_RANGE = 100.
TRAJECTORY_BUILDER_3D.min_range = 1.
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.min_range = 1.
voxel_filter_size = 0.2

--TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.11 						--default 0.15
--TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.) 	--default .1
--TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1e-1		--default 1e-1
--TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1			--default 1e-1

--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 1.	 													--default 1.
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 6.														--default 6.
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 3.																--default 5.
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 30.																	--default 4e2
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw = false															--default false
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = fales 		--default false
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations	= 12,					--default 12
--TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 4,									--default 1

--TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5																			--default 0.5
--TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters= 0.1																		--default 0.1
--TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = 0.4																			--default 0.004

--TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10																						--default 0.10
--TRAJECTORY_BUILDER_3D.submaps.high_resolution_max_range = 20.																		--default 20.
--TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.45																							--default 0.45
--TRAJECTORY_BUILDER_3D.submaps.num_range_data = 160																							--default 160
--TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = .55													--default 0.55
--TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability =.45													--default 0.49
--TRAJECTORY_BUILDER_3D.submaps.num_free_space_voxels = 2																					--default 2

-----------------
-- Map Builder --
-----------------
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 7 

----------------
-- Pose Graph --
----------------
POSE_GRAPH.optimize_every_n_nodes = 0

POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d = false
POSE_GRAPH.optimization_problem.fix_z_in_3d = false
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10 --Gene set to 5

POSE_GRAPH.constraint_builder.min_score = 0.55 --0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03 --Gene set to 0.3 and/or 0.6?????
--POSE_GRAPH.constraint_builder.max_constraint_distance = 30 --15
POSE_GRAPH.constraint_builder.ceres_scan_matcher.ceres_solver_options.num_threads = 7
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5 --10.
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 3 --5.
--POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(22.5)
--POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 100.
--POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10
--POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.only_optimize_yaw = false
--POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 4

return options
