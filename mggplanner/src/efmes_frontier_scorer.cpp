#include "mggplanner/efmes_frontier_scorer.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace explorer {

namespace {

struct UnionFind {
  explicit UnionFind(int n) : parent(static_cast<size_t>(n)) {
    std::iota(parent.begin(), parent.end(), 0);
  }
  int find(int x) {
    const size_t u = static_cast<size_t>(x);
    if (parent[u] == u) return x;
    parent[u] = static_cast<size_t>(find(static_cast<int>(parent[u])));
    return static_cast<int>(parent[u]);
  }
  void unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b) parent[static_cast<size_t>(a)] = static_cast<size_t>(b);
  }
  std::vector<size_t> parent;
};

}  // namespace

double EfmesFrontierScorer::repulsivePotentialXY(double px, double py,
                                                 double rx, double ry,
                                                 double d0_m, double k_rep) {
  if (d0_m <= 0.0 || k_rep <= 0.0) return 0.0;
  const double d = std::hypot(px - rx, py - ry);
  if (d > d0_m) return 0.0;
  const double t = (d - d0_m) / d0_m;
  return k_rep * t * t;
}

std::unordered_map<int, double> EfmesFrontierScorer::computeFrontierMultipliers(
    const std::vector<FrontierSample>& frontiers,
    const std::unordered_map<int, Eigen::Vector3d>& team_positions,
    int self_robot_id, double cluster_cell_m, double path_dist_eps,
    double d0_rep_m, double k_rep, double repulsion_weight, double score_blend,
    double min_multiplier) {
  std::unordered_map<int, double> out;
  const int n = static_cast<int>(frontiers.size());
  if (n <= 0) return out;

  cluster_cell_m = std::max(1e-3, cluster_cell_m);
  path_dist_eps = std::max(1e-6, path_dist_eps);
  score_blend = std::min(1.0, std::max(0.0, score_blend));
  min_multiplier = std::min(1.0, std::max(1e-6, min_multiplier));

  std::vector<int> gx(static_cast<size_t>(n)), gy(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    gx[static_cast<size_t>(i)] = static_cast<int>(
        std::llround(frontiers[static_cast<size_t>(i)].x / cluster_cell_m));
    gy[static_cast<size_t>(i)] = static_cast<int>(
        std::llround(frontiers[static_cast<size_t>(i)].y / cluster_cell_m));
  }

  UnionFind uf(n);
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const int manh = std::abs(gx[static_cast<size_t>(i)] -
                                  gx[static_cast<size_t>(j)]) +
                         std::abs(gy[static_cast<size_t>(i)] -
                                  gy[static_cast<size_t>(j)]);
      if (manh <= 2) uf.unite(i, j);
    }
  }

  std::unordered_map<int, int> comp_size;
  for (int i = 0; i < n; ++i) {
    const int r = uf.find(i);
    comp_size[r] += 1;
  }

  std::vector<double> raw(static_cast<size_t>(n), 0.0);
  double raw_max = 1e-12;
  for (int i = 0; i < n; ++i) {
    const int r = uf.find(i);
    const int Nb = std::max(1, comp_size[r]);
    const double d =
        std::max(path_dist_eps, frontiers[static_cast<size_t>(i)].path_dist_m);
    double rep = 0.0;
    const double px = frontiers[static_cast<size_t>(i)].x;
    const double py = frontiers[static_cast<size_t>(i)].y;
    for (const auto& kv : team_positions) {
      if (kv.first == self_robot_id) continue;
      rep += repulsivePotentialXY(px, py, kv.second.x(), kv.second.y(), d0_rep_m,
                                  k_rep);
    }
    const double P = static_cast<double>(Nb) / d;
    raw[static_cast<size_t>(i)] =
        P * std::exp(-repulsion_weight * std::max(0.0, rep));
    raw_max = std::max(raw_max, raw[static_cast<size_t>(i)]);
  }

  for (int i = 0; i < n; ++i) {
    const int fid = frontiers[static_cast<size_t>(i)].frontier_id;
    const double norm = std::min(1.0, raw[static_cast<size_t>(i)] / raw_max);
    const double mult =
        (1.0 - score_blend) + score_blend * (min_multiplier +
                                               (1.0 - min_multiplier) * norm);
    out[fid] = std::min(1.0, std::max(min_multiplier, mult));
  }
  return out;
}

}  // namespace explorer
