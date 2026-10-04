-- Pure-localization configuration for the accepted RPLIDAR A1 2D map.
-- Reuse the production scan, wheel-odometry, and TF settings from mapping.

include "rplidar_a1_2d.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}
POSE_GRAPH.optimize_every_n_nodes = 20

-- The accepted H-shaped map has repeated geometry. Keep scan matching local to
-- the explicitly supplied initial pose instead of falling back to ambiguous
-- full-map matches after the normal ten-second connection timeout.
POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9

return options
