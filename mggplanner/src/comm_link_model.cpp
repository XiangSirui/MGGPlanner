#include "mggplanner/comm_link_model.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

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

/** Thermal noise + noise figure when bandwidth is set; else YAML noise floor. */
double commLinkEffectiveNoiseDbm(const PlanningParams& pp) {
  if (pp.comm_link_bandwidth_hz > 0.0) {
    const double bw = std::max(1.0, pp.comm_link_bandwidth_hz);
    return -174.0 + 10.0 * std::log10(bw) + pp.comm_link_noise_figure_db;
  }
  return pp.comm_link_noise_floor_dbm;
}

/** PL0 at d0 from Friis if frequency is set; otherwise use YAML PL0. */
double commLinkEffectivePl0Db(const PlanningParams& pp) {
  if (pp.comm_link_carrier_freq_hz > 0.0) {
    static const double kSpeedOfLight = 299792458.0;
    const double d0 = std::max(0.1, pp.comm_link_d0_m);
    const double fspl_db = 20.0 * std::log10(4.0 * M_PI * d0 *
                                             pp.comm_link_carrier_freq_hz /
                                             kSpeedOfLight);
    return fspl_db - pp.comm_link_tx_antenna_gain_dbi -
           pp.comm_link_rx_antenna_gain_dbi + pp.comm_link_misc_system_loss_db;
  }
  return pp.comm_link_pl0_db;
}

uint64_t splitmix64(uint64_t z) {
  z += 0x9e3779b97f4a7c15ULL;
  z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
  z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
  return z ^ (z >> 31);
}

uint64_t hashLinkEndpoints(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  auto q = [](double v) {
    return static_cast<int64_t>(std::llround(v * 100.0));
  };
  uint64_t h = 1469598103934665603ULL;
  for (int64_t v : {q(a.x()), q(a.y()), q(a.z()), q(b.x()), q(b.y()), q(b.z())}) {
    h ^= static_cast<uint64_t>(v);
    h *= 1099511628211ULL;
  }
  return splitmix64(h);
}

/** Deterministic N(0,1) from 64-bit seed (Box–Muller). */
double deterministicGaussian01(uint64_t seed) {
  const uint64_t s1 = splitmix64(seed);
  const uint64_t s2 = splitmix64(seed ^ 0xCAFEBABECAFEBABEULL);
  const double scale = 1.0 / static_cast<double>(1ULL << 53);
  double u1 = static_cast<double>(s1 >> 11) * scale;
  double u2 = static_cast<double>(s2 >> 11) * scale;
  u1 = std::max(u1, std::numeric_limits<double>::min());
  u2 = std::max(u2, std::numeric_limits<double>::min());
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
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
  dirs.reserve(static_cast<size_t>(ray_count));
  dirs.push_back(u);
  if (ray_count == 1) {
    return dirs;
  }
  Eigen::Vector3d ref(0.0, 0.0, 1.0);
  if (std::fabs(u.dot(ref)) > 0.95) {
    ref = Eigen::Vector3d(0.0, 1.0, 0.0);
  }
  Eigen::Vector3d v = (ref - u.dot(ref) * u).normalized();
  Eigen::Vector3d w = u.cross(v).normalized();
  const int cap_rays = ray_count - 1;
  static const double kGoldenAngle = M_PI * (3.0 - std::sqrt(5.0));
  for (int k = 0; k < cap_rays; ++k) {
    const double t = (static_cast<double>(k) + 0.5) /
                     static_cast<double>(cap_rays);
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
  const double noise_dbm = commLinkEffectiveNoiseDbm(pp);
  const double pl0_db = commLinkEffectivePl0Db(pp);
  m.noise_floor_dbm = noise_dbm;
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
    m.path_loss_db = pl0_db;
    m.shadowing_db = 0.0;
    m.snr_db = pp.comm_link_ptx_dbm - m.path_loss_db - m.shadowing_db - noise_dbm;
    return m;
  }

  const MapManager::VoxelStatus geom_los =
      map.getRayStatus(pi, pj, pp.comm_link_ray_stop_at_unknown);
  double geom_loss_db = 0.0;
  if (geom_los == MapManager::kOccupied) {
    geom_loss_db = pp.comm_link_geom_los_obstructed_loss_db;
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
  const double pl_mean = pl0_db +
                         10.0 * n_eff * std::log10(d_ref / d0) +
                         pp.comm_link_unknown_penalty_db * m.unknown_ratio +
                         geom_loss_db;

  m.shadowing_db = 0.0;
  if (pp.comm_link_shadowing_std_db > 1e-9) {
    const uint64_t h = hashLinkEndpoints(pi, pj);
    m.shadowing_db = pp.comm_link_shadowing_std_db * deterministicGaussian01(h);
  }

  m.path_loss_db = pl_mean;
  m.snr_db = pp.comm_link_ptx_dbm - pl_mean - m.shadowing_db - noise_dbm;
  return m;
}

bool commLinkEdgeStrict(const MapManager& map, const PlanningParams& pp,
                        const Eigen::Vector3d& pi, const Eigen::Vector3d& pj,
                        const double max_range_m, const bool use_2d_distance,
                        CommLinkRayFanMetrics* metrics_out) {
  const double noise_dbm = commLinkEffectiveNoiseDbm(pp);
  const double pl0_db = commLinkEffectivePl0Db(pp);
  const double dist = pairwiseDistance(pi, pj, use_2d_distance);
  if (dist > max_range_m) {
    if (metrics_out) {
      metrics_out->distance_m = dist;
      metrics_out->inconclusive = false;
      metrics_out->map_ready = map.getStatus();
      metrics_out->noise_floor_dbm = noise_dbm;
    }
    return false;
  }
  if (dist < 1e-6) {
    if (metrics_out) {
      metrics_out->distance_m = dist;
      metrics_out->los_ratio = 1.0;
      metrics_out->shadowing_db = 0.0;
      metrics_out->noise_floor_dbm = noise_dbm;
      metrics_out->path_loss_db = pl0_db;
      metrics_out->snr_db = pp.comm_link_ptx_dbm - pl0_db - noise_dbm;
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
      metrics_out->noise_floor_dbm = noise_dbm;
      metrics_out->shadowing_db = 0.0;
      metrics_out->path_loss_db = pl0_db;
      metrics_out->snr_db = pp.comm_link_ptx_dbm - pl0_db - noise_dbm;
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
