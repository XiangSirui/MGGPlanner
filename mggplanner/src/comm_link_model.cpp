#include "mggplanner/comm_link_model.h"

#include <algorithm>
#include <cmath>

namespace explorer {

namespace {

double pairwiseDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                        bool use_2d) {
  const double dx = a.x() - b.x();
  const double dy = a.y() - b.y();
  if (use_2d) {
    return std::sqrt(dx * dx + dy * dy);
  }
  const double dz = a.z() - b.z();
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

std::vector<Eigen::Vector3d> buildCommLinkRayDirections(
    const Eigen::Vector3d& pi, const Eigen::Vector3d& pj, int ray_count,
    double half_angle_rad) {
  std::vector<Eigen::Vector3d> dirs;
  if (ray_count < 1) {
    return dirs;
  }
  Eigen::Vector3d chord = pj - pi;
  const double dist = chord.norm();
  if (dist < 1e-9) {
    dirs.push_back(Eigen::Vector3d(0.0, 0.0, 1.0));
    return dirs;
  }
  Eigen::Vector3d u = chord / dist;
  Eigen::Vector3d ref(0.0, 0.0, 1.0);
  if (std::fabs(u.dot(ref)) > 0.95) {
    ref = Eigen::Vector3d(0.0, 1.0, 0.0);
  }
  Eigen::Vector3d v = (ref - u.dot(ref) * u).normalized();
  Eigen::Vector3d w = u.cross(v).normalized();
  dirs.reserve(static_cast<size_t>(ray_count));
  if (ray_count == 1) {
    dirs.push_back(u);
    return dirs;
  }
  // 3D spherical-cap sampling around LOS axis (u): unlike the old 2D fan,
  // this captures slope / overhang effects in caves.
  static const double kGoldenAngle = M_PI * (3.0 - std::sqrt(5.0));
  for (int k = 0; k < ray_count; ++k) {
    const double t = (static_cast<double>(k) + 0.5) /
                     static_cast<double>(ray_count);
    const double theta = half_angle_rad * std::sqrt(t);
    const double phi = kGoldenAngle * static_cast<double>(k);
    const double st = std::sin(theta);
    const double ct = std::cos(theta);
    const double cp = std::cos(phi);
    const double sp = std::sin(phi);
    Eigen::Vector3d tangent = cp * v + sp * w;
    Eigen::Vector3d ddir = ct * u + st * tangent;
    const double nn = ddir.norm();
    if (nn > 1e-12) {
      ddir /= nn;
    }
    dirs.push_back(ddir);
  }
  return dirs;
}

CommLinkRayFanMetrics evaluateCommLinkRayFan(
    const MapManager& map, const PlanningParams& pp,
    const Eigen::Vector3d& pi, const Eigen::Vector3d& pj) {
  CommLinkRayFanMetrics m;
  m.distance_m = (pj - pi).norm();
  if (!map.getStatus()) {
    m.inconclusive = true;
    m.map_ready = false;
    return m;
  }
  m.map_ready = true;
  const double d_geom = m.distance_m;
  if (d_geom < 1e-9) {
    m.los_ratio = 1.0;
    m.path_loss_db = pp.comm_link_pl0_db;
    m.snr_db = pp.comm_link_ptx_dbm - m.path_loss_db - pp.comm_link_noise_floor_dbm;
    return m;
  }

  const int K = std::max(3, pp.comm_link_ray_count);
  const std::vector<Eigen::Vector3d> dirs =
      buildCommLinkRayDirections(pi, pj, K, pp.comm_link_ray_half_angle_rad);

  int n_free = 0;
  int n_occ = 0;
  int n_unk = 0;
  for (const Eigen::Vector3d& dir : dirs) {
    const Eigen::Vector3d end = pi + dir * d_geom;
    const MapManager::VoxelStatus vs =
        map.getRayStatus(pi, end, pp.comm_link_ray_stop_at_unknown);
    if (vs == MapManager::kFree) {
      ++n_free;
    } else if (vs == MapManager::kOccupied) {
      ++n_occ;
    } else {
      ++n_unk;
    }
  }

  const double invK = 1.0 / static_cast<double>(dirs.size());
  m.los_ratio = static_cast<double>(n_free) * invK;
  m.blocked_ratio = static_cast<double>(n_occ) * invK;
  m.unknown_ratio = static_cast<double>(n_unk) * invK;

  if (n_free == 0 && n_occ == 0 && n_unk == static_cast<int>(dirs.size())) {
    m.inconclusive = true;
    return m;
  }

  const double n_los = pp.comm_link_path_loss_exp_los;
  const double n_nlos = pp.comm_link_path_loss_exp_nlos;
  const double blend = std::min(
      1.0, 2.0 * m.blocked_ratio + 0.4 * m.unknown_ratio);
  const double n_eff = n_los + (n_nlos - n_los) * blend;

  const double d0 = std::max(0.1, pp.comm_link_d0_m);
  const double d_ref = std::max(d_geom, d0);
  m.path_loss_db = pp.comm_link_pl0_db +
                   10.0 * n_eff * std::log10(d_ref / d0) +
                   pp.comm_link_unknown_penalty_db * m.unknown_ratio;
  m.snr_db = pp.comm_link_ptx_dbm - m.path_loss_db - pp.comm_link_noise_floor_dbm;
  return m;
}

bool commLinkEdgeStrict(const MapManager& map, const PlanningParams& pp,
                        const Eigen::Vector3d& pi, const Eigen::Vector3d& pj,
                        const double max_range_m, const bool use_2d_distance,
                        CommLinkRayFanMetrics* metrics_out) {
  const double dist = pairwiseDistance(pi, pj, use_2d_distance);
  if (dist > max_range_m) {
    if (metrics_out) {
      metrics_out->distance_m = dist;
      metrics_out->inconclusive = false;
      metrics_out->map_ready = map.getStatus();
    }
    return false;
  }
  if (dist < 1e-6) {
    if (metrics_out) {
      metrics_out->distance_m = dist;
      metrics_out->los_ratio = 1.0;
      metrics_out->snr_db =
          pp.comm_link_ptx_dbm - pp.comm_link_pl0_db - pp.comm_link_noise_floor_dbm;
      metrics_out->inconclusive = false;
      metrics_out->map_ready = map.getStatus();
    }
    return true;
  }

  if (!pp.comm_link_model_enable) {
    if (metrics_out) {
      metrics_out->distance_m = dist;
      metrics_out->inconclusive = false;
      metrics_out->map_ready = map.getStatus();
      metrics_out->snr_db = pp.comm_link_ptx_dbm - pp.comm_link_pl0_db -
                          pp.comm_link_noise_floor_dbm;
    }
    return true;
  }

  CommLinkRayFanMetrics m = evaluateCommLinkRayFan(map, pp, pi, pj);
  m.distance_m = dist;

  if (!m.map_ready || m.inconclusive) {
    const bool ok = pp.comm_link_fallback_distance_if_inconclusive;
    if (metrics_out) {
      *metrics_out = m;
      metrics_out->distance_m = dist;
    }
    return ok;
  }

  const bool blocked_ok =
      m.blocked_ratio <= pp.comm_link_max_blocked_ratio + 1e-9;
  const bool snr_ok = m.snr_db >= pp.comm_link_min_snr_db - 1e-9;
  const bool ok = blocked_ok && snr_ok;
  if (metrics_out) {
    *metrics_out = m;
    metrics_out->distance_m = dist;
  }
  return ok;
}

}  // namespace explorer
