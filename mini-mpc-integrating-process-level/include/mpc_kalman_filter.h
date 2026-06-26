/**
 * mpc_kalman_filter.h — Kalman Filter for MPC State and Disturbance Estimation
 *
 * Domain: State Estimation for Offset-Free Model Predictive Control
 *
 * For offset-free MPC, the state must be augmented with an integrating
 * output disturbance model. This filter estimates both the process state
 * and the unmeasured disturbance from output measurements.
 *
 * The classical approach (Muske & Badgwell, 2002; Pannocchia & Rawlings, 2003):
 *   x(k+1) = A*x(k) + B*u(k) + B_d*d(k)
 *   d(k+1) = d(k) + w_d(k)           [random walk disturbance]
 *   y(k)   = C*x(k) + d(k) + v(k)
 *
 * The Kalman filter provides:
 *   1. Optimal state estimate x̂(k|k) given noisy measurements
 *   2. Disturbance estimate d̂(k|k) for offset removal
 *   3. One-step prediction for MPC free response initialization
 *
 * Knowledge Coverage:
 *   L5 - Algorithms: Kalman filter, disturbance estimation, Riccati DARE
 *   L3 - Eng. Structures: Augmented state-space, observer design
 *
 * Reference: Muske & Badgwell (2002) AIChE Journal 48(4)
 *            Pannocchia & Rawlings (2003) IEEE TAC 48(2)
 *            Simon (2006) "Optimal State Estimation"
 *
 * MIT 2.171 · CMU 24-677 · Georgia Tech ECE 6550 · Purdue ME 575
 */

#ifndef MPC_KALMAN_FILTER_H
#define MPC_KALMAN_FILTER_H

#include "mpc_level_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ─── Kalman Filter Lifecycle ─────────────────────────────────────────── */

/**
 * kalman_init — Initialize Kalman filter from state-space model
 *
 * Sets up filter matrices A, B, C from the process model.
 * Initializes:
 *   - P_post = Q_w (large initial uncertainty)
 *   - x_post = 0 (start from zero deviation)
 *   - K_gain = computed from initial P
 *
 * @param kf     Filter structure to initialize
 * @param A      State transition matrix (n×n col-major)
 * @param B      Input matrix (n×1)
 * @param C      Output matrix (1×n)
 * @param n      Number of states
 * @param q_scale Initial process noise scale
 * @param r_meas Measurement noise variance
 */
void kalman_init(kalman_config_t *kf, const double *A, const double *B,
                  const double *C, int n, double q_scale, double r_meas);

/**
 * kalman_predict — Time update (prediction step)
 *
 * Prior estimate before measurement:
 *   x̂(k|k-1) = A * x̂(k-1|k-1) + B * u(k-1)
 *   P(k|k-1) = A * P(k-1|k-1) * A^T + Q_w
 *
 * The prediction provides the "open-loop" estimate used by MPC to
 * initialize the free response.
 *
 * @param kf  Filter state (x_prior, P_prior updated)
 * @param u   Current input u(k-1)
 */
void kalman_predict(kalman_config_t *kf, double u);

/**
 * kalman_update — Measurement update (correction step)
 *
 * Posterior estimate after measurement:
 *   K = P(k|k-1) * C^T / (C * P(k|k-1) * C^T + R_v)   [Kalman gain]
 *   x̂(k|k) = x̂(k|k-1) + K * (y(k) - C * x̂(k|k-1))     [innovation]
 *   P(k|k) = (I - K*C) * P(k|k-1)                       [Joseph form]
 *
 * The Joseph form of the covariance update is used for guaranteed
 * symmetry and positive semi-definiteness even with numerical errors.
 *
 * Innovation (residual): e(k) = y(k) - C * x̂(k|k-1)
 *   The innovation sequence should be zero-mean white if the filter
 *   is correctly tuned (whiteness test for filter health).
 *
 * @param kf     Filter state (x_post, P_post, K_gain updated)
 * @param y_meas Current measurement y(k)
 */
void kalman_update(kalman_config_t *kf, double y_meas);

/**
 * kalman_step — Combined predict + update for one sampling interval
 *
 * Convenience wrapper: kalman_predict() then kalman_update().
 *
 * @param kf     Filter state
 * @param u      Input u(k-1)
 * @param y_meas Measurement y(k)
 */
void kalman_step(kalman_config_t *kf, double u, double y_meas);

/* ─── Disturbance Estimation for Offset-Free MPC ──────────────────────── */

/**
 * kalman_augment_disturbance — Augment state with integrating disturbance
 *
 * Transforms (A, B, C) to augmented form:
 *
 *   A_aug = [A    B_d]
 *           [0     1 ]
 *
 *   B_aug = [B]
 *           [0]
 *
 *   C_aug = [C    1]
 *
 * This guarantees offset-free tracking under the detectability condition:
 *   rank([I-A  -B_d;  C    1]) = n + n_dist  (Hautus condition)
 *
 * For integrating process level control: B_d = [K*Ts, 0]^T for the
 * single output disturbance case (step disturbance on outlet flow).
 *
 * @param kf           Filter (A, B, C overwritten with augmented)
 * @param B_d          Disturbance input matrix (n × 1)
 * @param n_states     Original state dimension
 * @param n_dist       Number of disturbances (typ. 1)
 * Returns new state dimension on success, -1 on error.
 */
int kalman_augment_disturbance(kalman_config_t *kf, const double *B_d,
                                int n_states, int n_dist);

/**
 * kalman_disturbance_estimate — Extract disturbance estimate from filter
 *
 * After state augmentation, the last state component is the disturbance.
 * If the disturbance is modeled as random walk, this provides:
 *   d̂(k|k) = x̂_n(k|k)  (last element of state vector)
 *
 * This disturbance is used for offset correction in MPC:
 *   y_corrected(k+i|k) = y_model(k+i|k) + d̂(k|k)
 *
 * @param kf      Filter with augmented state
 * @param d       Output: disturbance estimate
 * Returns 0 on success.
 */
int kalman_disturbance_estimate(const kalman_config_t *kf, double *d);

/* ─── Steady-State Kalman Gain ────────────────────────────────────────── */

/**
 * kalman_steady_state_gain — Solve DARE for steady-state Kalman gain
 *
 * Solves the Discrete Algebraic Riccati Equation (DARE):
 *   P = A*P*A^T - A*P*C^T*(C*P*C^T + R)^{-1}*C*P*A^T + Q
 *
 * Algorithm: Iterate Riccati difference equation to convergence.
 * For stable detection, the iteration converges geometrically.
 *
 * Steady-state gain K_ss = P_ss*C^T*(C*P_ss*C^T + R)^{-1}
 *
 * For industrial MPC, the steady-state gain is often used directly
 * (constant-gain observer) to avoid online Riccati updates.
 * This is the approach used in DMCplus (AspenTech) and RMPCT (Honeywell).
 *
 * Convergence criterion: ||ΔP||_F < tol (Frobenius norm change).
 *
 * @param kf    Filter (A, C, Q_w, R_v used as input; K_gain output)
 * @param tol   Convergence tolerance
 * @param max_iter Maximum iterations
 * Returns number of iterations on success, -1 if not converged.
 */
int kalman_steady_state_gain(kalman_config_t *kf, double tol, int max_iter);

/* ─── Innovation Monitoring ───────────────────────────────────────────── */

/**
 * kalman_innovation_stats — Compute innovation sequence statistics
 *
 * Innovation = y(k) - C*x̂(k|k-1).
 *
 * For a well-tuned filter:
 *   E[e(k)] ≈ 0
 *   Var[e(k)] ≈ C*P_prior*C^T + R_v
 *   Autocorrelation ≈ 0 (white)
 *
 * This function computes running mean and variance for filter health
 * monitoring. Large persistent innovations indicate model mismatch
 * or unmodeled disturbance changes.
 *
 * @param kf       Filter with current prior
 * @param y_meas   Current measurement
 * @param e_mean   Output: running mean of innovation
 * @param e_var    Output: running variance
 * @param count    Current sample count (updated on return)
 */
void kalman_innovation_stats(const kalman_config_t *kf, double y_meas,
                              double *e_mean, double *e_var, long *count);

/**
 * kalman_whiteness_test — Approximate Ljung-Box test on innovation
 *
 * Tests H0: innovation is white noise.
 * Uses first L lags of autocorrelation.
 *
 * Q = n*(n+2) * Σ_{k=1}^{L} r_k²/(n-k)
 * Under H0, Q ~ χ²(L) approximately.
 * Large Q → reject H0 → filter needs retuning.
 *
 * Reference: Ljung & Box (1978) Biometrika 65(2):297-303
 *
 * @param innovations  Array of recent innovations
 * @param n           Number of samples
 * @param L           Number of lags to test
 * Returns Q statistic.
 */
double kalman_whiteness_test(const double *innovations, int n, int L);

/* ─── Utility ─────────────────────────────────────────────────────────── */

/**
 * kalman_get_state — Copy estimated state from filter
 *
 * @param kf   Filter
 * @param x    Output array (pre-allocated, n_states)
 */
void kalman_get_state(const kalman_config_t *kf, double *x);

/**
 * kalman_get_covariance — Copy posterior covariance
 *
 * @param kf   Filter
 * @param P    Output matrix (n×n col-major, pre-allocated)
 */
void kalman_get_covariance(const kalman_config_t *kf, double *P);

#ifdef __cplusplus
}
#endif

#endif /* MPC_KALMAN_FILTER_H */
