#ifndef MGGPLANNER_DPFP_FRONTIER_PRIORITIZER_H_
#define MGGPLANNER_DPFP_FRONTIER_PRIORITIZER_H_

#include <unordered_map>
#include <vector>

#include <eigen3/Eigen/Dense>

namespace explorer {

/**
 * Probabilistic frontier prioritization from arXiv:2604.03042 (Devassy et al.):
 * P(I|xi) from normalized information gain, soft clustering via isotropic GMM
 * (practical stand-in for full DP-GMM in the paper), mixture component k* for
 * the querying robot pose, then P(k*|xi) and joint P(k*,I|xi)=P(I|xi)P(k*|xi).
 *
 * When the merged viewpoint count is small, K is clamped; EM uses isotropic
 * covariances for numerical stability on typical frontier clouds.
 */
class DpfpFrontierPrioritizer {
 public:
  struct Viewpoint {
    int frontier_id = -1;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    /** Non-negative scalar gain (e.g. unknown voxel count). */
    double information_gain = 0.0;
  };

  /**
   * @param viewpoints merged candidate frontiers (same frame as robot_position)
   * @param robot_position_world pose used for k* (Sec II-B.3)
   * @param max_components upper bound on mixture size K (paper uses DP-GMM;
   *        we sweep K in [1,Kmax] and pick best BIC after a short EM fit)
   * @param em_iterations EM outer iterations per K
   * @param floor_var isotropic variance floor (m^2)
   * @param eps small constant for log/prob numerics
   * @return map frontier_id -> joint probability; empty if viewpoints empty
   */
  static std::unordered_map<int, double> computeJointPriorities(
      const std::vector<Viewpoint>& viewpoints,
      const Eigen::Vector3d& robot_position_world, int max_components,
      int em_iterations, double floor_var, double eps);
};

}  // namespace explorer

#endif  // MGGPLANNER_DPFP_FRONTIER_PRIORITIZER_H_
