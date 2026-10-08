-- LSN10 Cartographer pure-localization configuration.
-- Load a mapping pbstream as a frozen trajectory, then keep only a small
-- number of localization submaps for the live trajectory.

include "lsn10.lua"

-- 30 Hz is sufficient for navigation and avoids flooding reliable /tf over
-- WiFi (the shared mapping config publishes poses at 200 Hz).
options.pose_publish_period_sec = 1. / 30.

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- Keep pure localization responsive on the Jetson. The frozen map contains
-- many nodes, so the mapping defaults can otherwise saturate every CPU core
-- while building constraints and starve odometry/TF publication.
MAP_BUILDER.num_background_threads = 2
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03
POSE_GRAPH.constraint_builder.log_matches = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 2

-- Localization must stay near the explicitly supplied initial pose. Avoid a
-- full-map global match when the environment contains repeated geometry.
POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9

return options
