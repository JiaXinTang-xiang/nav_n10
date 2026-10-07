-- LSN10 Cartographer pure-localization configuration.
-- Load a mapping pbstream as a frozen trajectory, then keep only a small
-- number of localization submaps for the live trajectory.

include "lsn10.lua"

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

POSE_GRAPH.optimize_every_n_nodes = 20

-- Localization must stay near the explicitly supplied initial pose. Avoid a
-- full-map global match when the environment contains repeated geometry.
POSE_GRAPH.global_sampling_ratio = 0.
POSE_GRAPH.global_constraint_search_after_n_seconds = 1e9

return options
