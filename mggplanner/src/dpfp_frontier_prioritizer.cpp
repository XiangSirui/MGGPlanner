#include "mggplanner/dpfp_frontier_prioritizer.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace explorer {

namespace {

constexpr double kPi = 3.14159265358979323846;

double logIsotropicGaussian(const Eigen::Vector3d& x, const Eigen::Vector3d& mu,
                            double var, double eps) {
  var = std::max(var, eps);
  const double d = 3.0;
  const Eigen::Vector3d r = x - mu;
  const double quad = r.squaredNorm();
  return -0.5 * d * std::log(2.0 * kPi * var) - quad / (2.0 * var);
}

double bicFromLogLikIsotropic(const std::vector<Eigen::Vector3d>& X,
                              const std::vector<double>& log_w,
                              const std::vector<Eigen::Vector3d>& mu,
                              const std::vector<double>& var, int K,
                              double eps) {
  const int N = static_cast<int>(X.size());
  if (N <= 0 || K <= 0) return std::numeric_limits<double>::infinity();
  double ll = 0.0;
  for (int n = 0; n < N; ++n) {
    double m = -std::numeric_limits<double>::infinity();
    for (int k = 0; k < K; ++k) {
      m = std::max(m, log_w[static_cast<size_t>(k)] +
                          logIsotropicGaussian(X[static_cast<size_t>(n)],
                                               mu[static_cast<size_t>(k)],
                                               var[static_cast<size_t>(k)], eps));
    }
    double s = 0.0;
    for (int k = 0; k < K; ++k) {
      s += std::exp(log_w[static_cast<size_t>(k)] +
                    logIsotropicGaussian(X[static_cast<size_t>(n)],
                                         mu[static_cast<size_t>(k)],
                                         var[static_cast<size_t>(k)], eps) -
                    m);
    }
    ll += m + std::log(std::max(s, eps));
  }
  // Free parameters: (K-1) weights + K means (3 each) + K variances
  const int num_params = (K - 1) + K * 3 + K;
  return -2.0 * ll + num_params * std::log(static_cast<double>(N));
}

bool runEmIsotropicGmm(const std::vector<Eigen::Vector3d>& X, int K,
                       int em_iterations, double floor_var, double eps,
                       std::vector<double>* log_pi_out,
                       std::vector<Eigen::Vector3d>* mu_out,
                       std::vector<double>* var_out) {
  const int N = static_cast<int>(X.size());
  if (N <= 0 || K <= 0) return false;
  K = std::min(K, N);

  std::vector<Eigen::Vector3d> mu(K);
  for (int k = 0; k < K; ++k) {
    const int idx = (k * N) / K;
    mu[k] = X[static_cast<size_t>(idx % N)];
  }

  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  for (const auto& x : X) mean += x;
  mean /= static_cast<double>(N);
  double global_var = floor_var;
  for (const auto& x : X) {
    global_var += (x - mean).squaredNorm();
  }
  global_var /= std::max(1, 3 * N);
  global_var = std::max(global_var, floor_var);

  std::vector<double> var(K, global_var);
  std::vector<double> log_pi(K, -std::log(static_cast<double>(K)));

  std::vector<std::vector<double>> gamma(
      static_cast<size_t>(N), std::vector<double>(static_cast<size_t>(K), 0.0));

  for (int it = 0; it < em_iterations; ++it) {
    // E-step
    for (int n = 0; n < N; ++n) {
      double max_log = -std::numeric_limits<double>::infinity();
      for (int k = 0; k < K; ++k) {
        const double v = log_pi[static_cast<size_t>(k)] +
                         logIsotropicGaussian(X[static_cast<size_t>(n)], mu[static_cast<size_t>(k)],
                                                var[static_cast<size_t>(k)], eps);
        max_log = std::max(max_log, v);
      }
      double sum = 0.0;
      for (int k = 0; k < K; ++k) {
        const double v = std::exp(
            log_pi[static_cast<size_t>(k)] +
            logIsotropicGaussian(X[static_cast<size_t>(n)], mu[static_cast<size_t>(k)],
                                   var[static_cast<size_t>(k)], eps) -
            max_log);
        gamma[static_cast<size_t>(n)][static_cast<size_t>(k)] = v;
        sum += v;
      }
      if (sum <= 0.0 || !std::isfinite(sum)) {
        const double u = 1.0 / static_cast<double>(K);
        for (int k = 0; k < K; ++k) {
          gamma[static_cast<size_t>(n)][static_cast<size_t>(k)] = u;
        }
        sum = 1.0;
      }
      for (int k = 0; k < K; ++k) {
        gamma[static_cast<size_t>(n)][static_cast<size_t>(k)] /= sum;
      }
    }

    // M-step
    std::vector<double> Nk(static_cast<size_t>(K), 0.0);
    for (int n = 0; n < N; ++n) {
      for (int k = 0; k < K; ++k) {
        Nk[static_cast<size_t>(k)] += gamma[static_cast<size_t>(n)][static_cast<size_t>(k)];
      }
    }
    for (int k = 0; k < K; ++k) {
      if (Nk[static_cast<size_t>(k)] < 1e-6) {
        mu[static_cast<size_t>(k)] = X[static_cast<size_t>(k % N)];
        var[static_cast<size_t>(k)] = global_var;
        log_pi[static_cast<size_t>(k)] = -std::log(static_cast<double>(K));
        continue;
      }
      Eigen::Vector3d mk = Eigen::Vector3d::Zero();
      for (int n = 0; n < N; ++n) {
        mk += gamma[static_cast<size_t>(n)][static_cast<size_t>(k)] * X[static_cast<size_t>(n)];
      }
      mk /= Nk[static_cast<size_t>(k)];
      double vk = 0.0;
      for (int n = 0; n < N; ++n) {
        const double g = gamma[static_cast<size_t>(n)][static_cast<size_t>(k)];
        vk += g * (X[static_cast<size_t>(n)] - mk).squaredNorm();
      }
      vk /= (3.0 * Nk[static_cast<size_t>(k)]);
      vk = std::max(vk, floor_var);
      mu[static_cast<size_t>(k)] = mk;
      var[static_cast<size_t>(k)] = vk;
    }

    double wsum = 0.0;
    for (int k = 0; k < K; ++k) {
      wsum += Nk[static_cast<size_t>(k)];
    }
    if (wsum <= 0.0) return false;
    for (int k = 0; k < K; ++k) {
      log_pi[static_cast<size_t>(k)] =
          std::log(std::max(Nk[static_cast<size_t>(k)] / wsum, eps));
    }
  }

  *log_pi_out = std::move(log_pi);
  *mu_out = std::move(mu);
  *var_out = std::move(var);
  return true;
}

}  // namespace

std::unordered_map<int, double> DpfpFrontierPrioritizer::computeJointPriorities(
    const std::vector<Viewpoint>& viewpoints,
    const Eigen::Vector3d& robot_position_world, int max_components,
    int em_iterations, double floor_var, double eps) {
  std::unordered_map<int, double> out;
  const int N = static_cast<int>(viewpoints.size());
  if (N <= 0) return out;

  std::vector<Eigen::Vector3d> X;
  X.reserve(static_cast<size_t>(N));
  std::vector<double> I;
  I.reserve(static_cast<size_t>(N));
  double sum_I = 0.0;
  for (const auto& v : viewpoints) {
    X.push_back(v.position);
    const double ig = std::max(v.information_gain, eps);
    I.push_back(ig);
    sum_I += ig;
  }

  std::vector<double> P_I(static_cast<size_t>(N), 1.0 / static_cast<double>(N));
  if (sum_I > 0.0) {
    for (int n = 0; n < N; ++n) {
      P_I[static_cast<size_t>(n)] = I[static_cast<size_t>(n)] / sum_I;
    }
  }

  if (N == 1) {
    out[viewpoints[0].frontier_id] = std::max(P_I[0], eps);
    return out;
  }

  const int Kmax = std::max(1, std::min(max_components, N));
  double best_bic = std::numeric_limits<double>::infinity();
  std::vector<double> best_log_pi;
  std::vector<Eigen::Vector3d> best_mu;
  std::vector<double> best_var;
  int best_K = 1;

  for (int K = 1; K <= Kmax; ++K) {
    std::vector<double> log_pi;
    std::vector<Eigen::Vector3d> mu;
    std::vector<double> var;
    if (K == 1) {
      Eigen::Vector3d m = Eigen::Vector3d::Zero();
      for (const auto& x : X) m += x;
      m /= static_cast<double>(N);
      double v = floor_var;
      for (const auto& x : X) v += (x - m).squaredNorm();
      v /= std::max(1, 3 * N);
      v = std::max(v, floor_var);
      log_pi.push_back(0.0);
      mu.push_back(m);
      var.push_back(v);
    } else {
      if (!runEmIsotropicGmm(X, K, em_iterations, floor_var, eps, &log_pi, &mu,
                             &var)) {
        continue;
      }
    }
    // Convert log_pi to weights for BIC helper
    std::vector<double> w(static_cast<size_t>(K));
    for (int k = 0; k < K; ++k) {
      w[static_cast<size_t>(k)] = std::exp(log_pi[static_cast<size_t>(k)]);
    }
    const double bic = bicFromLogLikIsotropic(X, log_pi, mu, var, K, eps);
    if (std::isfinite(bic) && bic < best_bic) {
      best_bic = bic;
      best_log_pi = std::move(log_pi);
      best_mu = std::move(mu);
      best_var = std::move(var);
      best_K = K;
    }
  }

  if (best_mu.empty()) {
    const double u = 1.0 / static_cast<double>(N);
    for (int n = 0; n < N; ++n) {
      out[viewpoints[static_cast<size_t>(n)].frontier_id] =
          std::max(P_I[static_cast<size_t>(n)] * u, eps);
    }
    return out;
  }

  // k* for robot pose (Sec II-B.3)
  int k_star = 0;
  double best_mix = -std::numeric_limits<double>::infinity();
  for (int k = 0; k < best_K; ++k) {
    const double logp =
        best_log_pi[static_cast<size_t>(k)] +
        logIsotropicGaussian(robot_position_world, best_mu[static_cast<size_t>(k)],
                             best_var[static_cast<size_t>(k)], eps);
    if (logp > best_mix + 1e-12 ||
        (std::abs(logp - best_mix) <= 1e-12 && k < k_star)) {
      best_mix = logp;
      k_star = k;
    }
  }

  double denom_pk = 0.0;
  for (int n = 0; n < N; ++n) {
    denom_pk += std::exp(
        logIsotropicGaussian(X[static_cast<size_t>(n)],
                             best_mu[static_cast<size_t>(k_star)],
                             best_var[static_cast<size_t>(k_star)], eps));
  }
  denom_pk = std::max(denom_pk, eps);

  for (int n = 0; n < N; ++n) {
    const double pk =
        std::exp(logIsotropicGaussian(X[static_cast<size_t>(n)],
                                       best_mu[static_cast<size_t>(k_star)],
                                       best_var[static_cast<size_t>(k_star)], eps)) /
        denom_pk;
    const double joint = std::max(P_I[static_cast<size_t>(n)] * pk, eps);
    out[viewpoints[static_cast<size_t>(n)].frontier_id] = joint;
  }
  return out;
}

}  // namespace explorer
