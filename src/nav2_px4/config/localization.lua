include "map_builder.lua"
include "trajectory_builder.lua"

-- 기본 설정
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  use_odometry = true,
  use_nav_sat = true,
  use_landmarks = false, -- 안 씀
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  publish_to_tf = true,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 0.05,
  trajectory_publish_period_sec = 0.1,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 2D Trajectory 설정
MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 해상도 설정 (GridResolution). 0.1 = 10cm 단위, 여기서는 5cm
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- Pure Localization 모드 관련 설정
  -- ◆ [강력한 효과]전역 매칭(루프 클로저) 최소 점수
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.675
  -- ◆ [강력한 효과]로컬 매칭(일반 스캔 매칭) 최소 점수
POSE_GRAPH.constraint_builder.min_score = 0.75

POSE_GRAPH.global_constraint_search_after_n_seconds = 0
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 5,
}
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- ◆ [GLOBAL]
-- 초기 위치에 대한 설정. 아래 두 값은 초기 위치가 크게 벗어날 가능성이 높으면 큰 값을 지정
  -- [강력한 효과]global Fast Correlative 매칭에서 x-y 평면상 탐색 범위 (m), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 0.01
  -- [강력한 효과]global Fast Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 고정. 작을수록 좋음
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(1.5)
  -- [매우 강력한 효과] global 전역 매칭(큰 오프셋 수정 등) 시 스캔을 추출하여 매칭 시도할 확률 (0 ~ 1). 연산량 tradeoff가 존재. 0.0036-0.004 사이. 0.0001 단위로 조절
POSE_GRAPH.global_sampling_ratio = 0.00498 -- 정반대 일 떄

-- ◆ [LOCAL]
-- [강력한 효과]real time 변수 설정
  -- 실시간 Local Correlative 매칭에서 x-y 평면상 탐색 범위 (m)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.01
-- [강력한 효과]실시간 Local Correlative 매칭에서 회전(각도) 탐색 범위 (라디안), 얼마나 허용할 지
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.5)

TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 200.0
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 200.0

-- LiDAR 관련
TRAJECTORY_BUILDER_2D.min_range = 0.1
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 5.0
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05

-- Ceres 기반 Scan Matcher 설정, Lidar 데이터로 이전 서브맵과의 비교를 수행, pose&orientation 파악
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 400.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 200.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 200.0

--[드리프트 심할 때 키우세요] IMU 설정
  -- 급격한 steering이 있을 경우에는 time_constant와 rotation_weight 증가 고려
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 30.0

MAP_BUILDER.num_background_threads = 4

-- ◆ 기타 posegraph 관련
  --[강력한 효과]n개의 노드(스캔)이 쌓일 때마다 전역 최적화(Loop Closure 등) 실행. 적을수록 빠르게 최적화가 일어남. 1개가 적절
POSE_GRAPH.optimize_every_n_nodes = 1

-- 전역 매칭을 위한 Submap 간 최대 거리
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.0

-- [강력한 효과]Loop clousre 관련 변수
POSE_GRAPH.constraint_builder.loop_closure_translation_weight = 500000.0
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight = 500000.0

POSE_GRAPH.constraint_builder.sampling_ratio = 0.0001

return options
