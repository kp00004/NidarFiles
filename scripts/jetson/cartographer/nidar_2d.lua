-- Cartographer 2D SLAM for the RPLIDAR A2 (lidar_node, /scan, frame "laser").
-- LiDAR only: no odometry, no IMU -- works hand-held or on the drone.
-- Based on Cartographer's own hand-held LiDAR example (revo_lds.lua), which
-- start_jetson.sh copies next to this file from the installed cartographer_ros
-- so every option name matches the installed version.

include "revo_lds.lua"

-- our LaserScan frame; Cartographer publishes TF map -> odom -> laser
options.tracking_frame = "laser"
options.published_frame = "laser"
options.provide_odom_frame = true
options.use_odometry = false

-- RPLIDAR A2: 0.15 .. 12 m. min_range is replaced at start by
-- NIDAR_LIDAR_MIN_RANGE_M (default 0.4): returns closer than that are the
-- person holding the LiDAR, or the drone's own frame/props, not the room.
TRAJECTORY_BUILDER_2D.min_range = 0.4
TRAJECTORY_BUILDER_2D.max_range = 12.
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

return options
