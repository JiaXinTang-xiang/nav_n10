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
  map_frame = "map",  -- 地图坐标系
  tracking_frame = "imu_link",  -- 用于追踪的传感器坐标系，通常是 IMU 坐标系
  published_frame = "laser_frame",  -- 发布的坐标系，通常是激光雷达的坐标系
  odom_frame = "odom",  -- 里程计坐标系
  provide_odom_frame = true,  -- 是否提供 odom 坐标系，通常设为 true
  publish_frame_projected_to_2d = false,  -- 是否将坐标系投影到 2D 平面，设置为 false 代表使用 3D 坐标系
  use_pose_extrapolator = true,  -- 是否使用位姿外推器，以处理无效位姿的情况
  use_odometry = false,  -- 是否使用外部里程计数据（这里设置为 false，表示不使用）
  use_nav_sat = false,  -- 是否使用卫星定位（如 GPS）数据，通常设置为 false
  use_landmarks = false,  -- 是否使用地标数据进行定位和建图，设置为 false
  num_laser_scans = 1,  -- 激光雷达数据的数量，1 表示单激光雷达
  num_multi_echo_laser_scans = 0,  -- 多回波激光雷达数据的数量
  num_subdivisions_per_laser_scan = 1,  -- 每个激光扫描的子分段数，通常为 1
  num_point_clouds = 0,  -- 点云数据的数量
  lookup_transform_timeout_sec = 0.2,  -- 坐标变换的超时时间
  submap_publish_period_sec = 0.3,  -- 子地图发布的周期（秒）
  pose_publish_period_sec = 5e-3,  -- 位姿发布周期（秒）
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期（秒）
  rangefinder_sampling_ratio = 1.,  -- 激光雷达数据的采样比例
  odometry_sampling_ratio = 1.,  -- 里程计数据的采样比例
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定框架下的位姿采样比例
  imu_sampling_ratio = 1.,  -- IMU 数据的采样比例
  landmarks_sampling_ratio = 1.,  -- 地标数据的采样比例
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- 每个子地图包含的激光数据数量
TRAJECTORY_BUILDER_2D.min_range = 0.3  -- 激光雷达的最小有效测量范围
TRAJECTORY_BUILDER_2D.max_range = 12.  -- 激光雷达的最大有效测量范围
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 1.  -- 缺失数据时的最大射线长度
TRAJECTORY_BUILDER_2D.use_imu_data = true  -- 启用 IMU 数据进行融合
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 启用在线相关扫描匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1  -- 线性搜索窗口（单位：米）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.  -- 位移变化代价权重
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1  -- 旋转变化代价权重

POSE_GRAPH.optimization_problem.huber_scale = 1e2  -- Hubner 损失函数的尺度
POSE_GRAPH.optimize_every_n_nodes = 35  -- 每隔 35 个节点进行优化
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 子图约束的最小分数

return options

