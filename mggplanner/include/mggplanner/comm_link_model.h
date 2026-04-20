#ifndef COMM_LINK_MODEL_H_
#define COMM_LINK_MODEL_H_

#include <vector>

#include <eigen3/Eigen/Dense>

#include "planner_common/map_manager.h"
#include "planner_common/params.h"

namespace explorer {

/** One segment for RViz team-link visualization. */
struct CommTeamLinkViz {
  Eigen::Vector3d p;
  Eigen::Vector3d q;
  double snr_db = 0.0;
  bool edge_ok = false;
};

struct CommLinkRayFanMetrics {
  double distance_m = 0.0;
  double los_ratio = 0.0;
  double blocked_ratio = 0.0;
  double unknown_ratio = 0.0;
  double path_loss_db = 0.0;
  double snr_db = 0.0;
  /** Map not integrated or all rays inconclusive (e.g. all unknown). */
  bool inconclusive = false;
  bool map_ready = false;
};

/**
 * Build 3D ray directions on a spherical cap around LOS (pi->pj).
 * half_angle_rad is the cone half-angle; ray_count controls cap sampling density.
 */
std::vector<Eigen::Vector3d> buildCommLinkRayDirections(
    const Eigen::Vector3d& pi, const Eigen::Vector3d& pj, int ray_count,
    double half_angle_rad);

CommLinkRayFanMetrics evaluateCommLinkRayFan(
    const MapManager& map, const PlanningParams& pp,
    const Eigen::Vector3d& pi, const Eigen::Vector3d& pj);

/**
 * Communication edge: within max_range_m, then map-coupled ray fan + path loss /
 * SNR and blocked-ray ratio when PlanningParams::comm_link_model_enable (default
 * on). When disabled, distance-only within max_range_m. When map is not ready or
 * metrics are inconclusive, uses comm_link_fallback_distance_if_inconclusive.
 */
bool commLinkEdgeStrict(const MapManager& map, const PlanningParams& pp,
                        const Eigen::Vector3d& pi, const Eigen::Vector3d& pj,
                        double max_range_m, bool use_2d_distance,
                        CommLinkRayFanMetrics* metrics_out = nullptr);

}  // namespace explorer

#endif
