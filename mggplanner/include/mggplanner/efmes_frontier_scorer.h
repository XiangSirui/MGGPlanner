#ifndef MGGPLANNER_EFMES_FRONTIER_SCORER_H_
#define MGGPLANNER_EFMES_FRONTIER_SCORER_H_

#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace explorer {

/**
 * EFMES-inspired frontier potential (Zuo et al., Meas. Sci. Technol. 36 086316,
 * 2025): connectivity-based cluster size N(b) on a coarse 2D grid (Manhattan
 * link distance <= 2 between snapped cells), P(b) ~ N(b) / d_SPFA, and
 * multi-robot repulsive potential (paper Eq. (2)(3)) as an exponential discount.
 * Outputs per-frontier multipliers in (0,1] for blending into the planner score.
 */
class EfmesFrontierScorer {
 public:
  struct FrontierSample {
    int frontier_id = -1;
    double x = 0.0;
    double y = 0.0;
    /** Shortest-path distance from querying robot to this frontier (m). */
    double path_dist_m = 0.0;
  };

  /**
   * @param self_robot_id robots != self contribute repulsion at frontier XY
   * @param cluster_cell_m grid resolution for connectivity (paper: discrete map)
   * @param path_dist_eps denominator offset for P = N/d
   * @param d0_rep_m repulsion influence radius (paper d0)
   * @param k_rep repulsion gain K_rep (paper Eq. (3) style; may be precomputed)
   * @param repulsion_weight positive factor on sum V_rep inside exp(-.)
   * @param score_blend in [0,1] blend toward normalized potential (like DPFP)
   * @param min_multiplier floor on the blended multiplier
   */
  static std::unordered_map<int, double> computeFrontierMultipliers(
      const std::vector<FrontierSample>& frontiers,
      const std::unordered_map<int, Eigen::Vector3d>& team_positions,
      int self_robot_id, double cluster_cell_m, double path_dist_eps,
      double d0_rep_m, double k_rep, double repulsion_weight, double score_blend,
      double min_multiplier);

  static double repulsivePotentialXY(double px, double py, double rx, double ry,
                                     double d0_m, double k_rep);
};

}  // namespace explorer

#endif  // MGGPLANNER_EFMES_FRONTIER_SCORER_H_
